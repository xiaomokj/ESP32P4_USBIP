/*
 * MKLink Remote - CDC-ACM loopback probe.
 *
 * Minimal, dependency free CDC test: configure the line coding through the
 * control endpoint, send a payload on the bulk OUT endpoint and compare the echo
 * on the bulk IN endpoint. It is a bring-up aid only - disable
 * CONFIG_MKLINK_CDC_TEST_ENABLE before the usbip server claims the interfaces.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_types_ch9.h"

#include "mklink_core.h"
#include "mklink_usb.h"
#include "mklink_usb_raw.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.cdc";

#define CDC_SET_LINE_CODING        0x20
#define CDC_SET_CONTROL_LINE_STATE 0x22

/* The USB stack times a transfer out after CDC_TRANSFER_TIMEOUT_MS and the
 * helper waits a little longer, so the completion callback almost always runs
 * before the waiter gives up. */
#define CDC_TRANSFER_TIMEOUT_MS    1000
#define CDC_SYNC_WAIT_MS           1600

static mklink_cdc_result_t s_result;
static SemaphoreHandle_t s_result_lock;
static volatile bool s_running;

static void result_lock(void)
{
    if (s_result_lock == NULL) {
        s_result_lock = xSemaphoreCreateMutex();
    }
    if (s_result_lock != NULL) {
        xSemaphoreTake(s_result_lock, portMAX_DELAY);
    }
}

static void result_unlock(void)
{
    if (s_result_lock != NULL) {
        xSemaphoreGive(s_result_lock);
    }
}

void mklink_cdc_store_result(const mklink_cdc_result_t *res)
{
    result_lock();
    if (res != NULL) {
        s_result = *res;
    } else {
        memset(&s_result, 0, sizeof(s_result));
    }
    result_unlock();
}

bool mklink_cdc_get_result(mklink_cdc_result_t *out)
{
    if (out == NULL) {
        return false;
    }
    result_lock();
    bool ran = s_result.ran;
    *out = s_result;
    result_unlock();
    return ran;
}

/*
 * Synchronous transfer helper.
 *
 * The completion callback can fire *after* the waiter timed out (or after the
 * device was unplugged), so the context must outlive the call and the semaphore
 * must never be deleted while a transfer is still pending - doing that makes the
 * callback give a deleted queue and panics the whole system. Ownership rules:
 *
 *   WAITING -> DONE       : callback stored the result, waiter cleans up
 *   WAITING -> ABANDONED  : waiter gave up, callback cleans up
 */
typedef enum {
    SYNC_WAITING,
    SYNC_DONE,
    SYNC_ABANDONED,
} sync_state_t;

typedef struct {
    SemaphoreHandle_t done;
    sync_state_t state;
    esp_err_t status;
    size_t actual;
    portMUX_TYPE mux;
} sync_ctx_t;

static void sync_ctx_free(sync_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->done != NULL) {
        vSemaphoreDelete(ctx->done);
    }
    free(ctx);
}

static void sync_cb(void *arg, esp_err_t status, size_t actual)
{
    sync_ctx_t *ctx = (sync_ctx_t *)arg;
    if (ctx == NULL) {
        return;
    }

    bool orphan = false;
    portENTER_CRITICAL(&ctx->mux);
    if (ctx->state == SYNC_WAITING) {
        ctx->status = status;
        ctx->actual = actual;
        ctx->state = SYNC_DONE;
    } else if (ctx->state == SYNC_ABANDONED) {
        orphan = true;
    }
    portEXIT_CRITICAL(&ctx->mux);

    if (orphan) {
        sync_ctx_free(ctx);
    } else {
        xSemaphoreGive(ctx->done);
    }
}

esp_err_t mklink_cdc_sync_transfer(uint8_t addr, uint8_t ep, uint8_t *buf, size_t len, bool control,
                                   const void *setup, size_t *actual)
{
    if (!mklink_usb_device_alive(addr)) {
        return ESP_ERR_INVALID_STATE;
    }

    sync_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ctx->done = xSemaphoreCreateBinary();
    ctx->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    ctx->state = SYNC_WAITING;
    ctx->status = ESP_FAIL;
    if (ctx->done == NULL) {
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = control
        ? mklink_usb_raw_control(addr, (const usb_setup_packet_t *)setup, buf, len,
                                 CDC_TRANSFER_TIMEOUT_MS, sync_cb, ctx)
        : mklink_usb_raw_submit(addr, ep, buf, len, CDC_TRANSFER_TIMEOUT_MS, sync_cb, ctx);
    if (err != ESP_OK) {
        sync_ctx_free(ctx);
        return err;
    }

    /* Wait longer than the stack-side timeout so the callback usually wins. */
    (void)xSemaphoreTake(ctx->done, pdMS_TO_TICKS(CDC_SYNC_WAIT_MS));

    sync_state_t state;
    esp_err_t result = ESP_FAIL;
    size_t got = 0;
    portENTER_CRITICAL(&ctx->mux);
    state = ctx->state;
    if (state == SYNC_DONE) {
        result = ctx->status;
        got = ctx->actual;
    } else if (state == SYNC_WAITING) {
        ctx->state = SYNC_ABANDONED;   /* the callback now owns ctx */
    }
    portEXIT_CRITICAL(&ctx->mux);

    if (state == SYNC_DONE) {
        sync_ctx_free(ctx);
        if (actual != NULL) {
            *actual = got;
        }
        return result;
    }

    if (state == SYNC_WAITING) {
        /* Nudge the pending transfer so the callback runs and frees ctx soon. */
        mklink_usb_raw_abort(addr, ep);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t cdc_set_line_coding(uint8_t addr, uint8_t iface)
{
    uint8_t line_coding[7] = {
        0x00, 0xC2, 0x01, 0x00, /* 115200 baud, little endian */
        0x00,                   /* 1 stop bit */
        0x00,                   /* no parity */
        0x08,                   /* 8 data bits */
    };
    usb_setup_packet_t setup = {
        .bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        .bRequest = CDC_SET_LINE_CODING,
        .wValue = 0,
        .wIndex = iface,
        .wLength = sizeof(line_coding),
    };
    return mklink_cdc_sync_transfer(addr, 0, line_coding, sizeof(line_coding), true, &setup, NULL);
}

static esp_err_t cdc_set_control_lines(uint8_t addr, uint8_t iface)
{
    usb_setup_packet_t setup = {
        .bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        .bRequest = CDC_SET_CONTROL_LINE_STATE,
        .wValue = 0x0003, /* DTR | RTS */
        .wIndex = iface,
        .wLength = 0,
    };
    return mklink_cdc_sync_transfer(addr, 0, NULL, 0, true, &setup, NULL);
}

static void cdc_test_task(void *arg)
{
    uint8_t addr = (uint8_t)(uintptr_t)arg;
    mklink_cdc_result_t res = { .ran = true, .pass = false };

    mklink_usb_snapshot_t snap;
    if (!mklink_usb_get_snapshot(&snap) || !snap.connected) {
        strlcpy(res.detail, "no USB device", sizeof(res.detail));
        goto publish;
    }

    /* Composite probes expose several vendor bulk pairs (CMSIS-DAP). Only the
     * CDC *data* interface may be exercised here. */
    uint8_t comm_iface = 0xFF;
    uint8_t data_iface = 0xFF;
    if (!mklink_usb_cdc_ifaces(&comm_iface, &data_iface)) {
        strlcpy(res.detail, "no CDC data interface", sizeof(res.detail));
        goto publish;
    }

    mklink_ep_info_t eps[16];
    int ep_count = mklink_usb_endpoint_list(eps, 16);

    uint8_t ep_out = 0;
    uint8_t ep_in = 0;
    uint16_t mps_in = 64;

    for (int i = 0; i < ep_count; i++) {
        if (eps[i].attributes != 0x02 || eps[i].iface_num != data_iface) {
            continue;
        }
        if (eps[i].ep_addr & 0x80) {
            if (ep_in == 0) {
                ep_in = eps[i].ep_addr;
                mps_in = eps[i].mps;
            }
        } else if (ep_out == 0) {
            ep_out = eps[i].ep_addr;
        }
    }

    if (ep_in == 0 || ep_out == 0) {
        snprintf(res.detail, sizeof(res.detail), "no bulk pair on data if%u", data_iface);
        goto publish;
    }

    if (!mklink_usb_device_alive(addr)) {
        strlcpy(res.detail, "device removed before claim", sizeof(res.detail));
        goto publish;
    }

    esp_err_t err = mklink_usb_claim_interface(data_iface);
    if (err != ESP_OK) {
        snprintf(res.detail, sizeof(res.detail), "claim data if%u: %s", data_iface, esp_err_to_name(err));
        goto publish;
    }
    bool comm_claimed = false;
    if (comm_iface != 0xFF && comm_iface != data_iface) {
        if (mklink_usb_claim_interface(comm_iface) == ESP_OK) {
            comm_claimed = true;
        }
    }

    if (comm_claimed) {
        cdc_set_line_coding(addr, comm_iface);
        cdc_set_control_lines(addr, comm_iface);
    }

    if (!mklink_usb_device_alive(addr)) {
        strlcpy(res.detail, "device removed before transfer", sizeof(res.detail));
        goto release;
    }

    const char *payload = CONFIG_MKLINK_CDC_TEST_PAYLOAD;
    size_t payload_len = strlen(payload);
    size_t tx_actual = 0;
    err = mklink_cdc_sync_transfer(addr, ep_out, (uint8_t *)payload, payload_len, false, NULL, &tx_actual);
    if (err != ESP_OK) {
        snprintf(res.detail, sizeof(res.detail), "tx failed: %s", esp_err_to_name(err));
        goto release;
    }
    res.tx_bytes = (uint32_t)tx_actual;

    size_t rx_len = mps_in > 0 ? mps_in : 64;
    uint8_t rx_buf[512];
    if (rx_len > sizeof(rx_buf)) {
        rx_len = sizeof(rx_buf);
    }
    size_t rx_actual = 0;
    err = mklink_cdc_sync_transfer(addr, ep_in, rx_buf, rx_len, false, NULL, &rx_actual);
    if (err == ESP_ERR_TIMEOUT) {
        /* A CMSIS-DAP probe exposes a plain VCP that does not echo anything, so a
         * timeout is the expected outcome - report it as "no echo", not a fault. */
        strlcpy(res.detail, "no echo (probe VCP does not loop back)", sizeof(res.detail));
        goto release;
    }
    if (err != ESP_OK) {
        snprintf(res.detail, sizeof(res.detail), "rx failed: %s", esp_err_to_name(err));
        goto release;
    }
    res.rx_bytes = (uint32_t)rx_actual;

    if (rx_actual == 0) {
        strlcpy(res.detail, "no loopback data (probe is not in echo mode)", sizeof(res.detail));
    } else if (rx_actual >= payload_len && memcmp(rx_buf, payload, payload_len) == 0) {
        res.pass = true;
        strlcpy(res.detail, "loopback ok", sizeof(res.detail));
    } else {
        snprintf(res.detail, sizeof(res.detail), "mismatch (%u bytes)", (unsigned)rx_actual);
    }

release:
    if (comm_claimed) {
        mklink_usb_release_interface(comm_iface);
    }
    mklink_usb_release_interface(data_iface);

publish:
    mklink_cdc_store_result(&res);
    mklink_log("CDC test: %s (%u/%u bytes) %s", res.pass ? "PASS" : "FAIL",
               (unsigned)res.tx_bytes, (unsigned)res.rx_bytes, res.detail);
    esp_event_post(MKLINK_EVENT, MKLINK_EVENT_CDC_RESULT, &res, sizeof(res), 0);
    s_running = false;
    vTaskDelete(NULL);
}

esp_err_t mklink_cdc_test_start(void)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t addr = mklink_usb_current_addr();
    if (addr == MKLINK_USB_ADDR_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    s_running = true;
    if (xTaskCreate(cdc_test_task, "mk_cdc_test", 4096, (void *)(uintptr_t)addr, 5, NULL) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t mklink_cdc_retest(void)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    mklink_usb_snapshot_t snap;
    if (!mklink_usb_get_snapshot(&snap) || !snap.connected) {
        mklink_log("CDC test skipped: no USB device attached");
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t comm_iface = 0xFF;
    uint8_t data_iface = 0xFF;
    if (!mklink_usb_cdc_ifaces(&comm_iface, &data_iface)) {
        mklink_log("CDC test skipped: device has no CDC data interface");
        return ESP_ERR_NOT_FOUND;
    }
    return mklink_cdc_test_start();
}
