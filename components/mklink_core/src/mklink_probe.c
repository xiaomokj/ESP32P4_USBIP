/*
 * MKLink Remote - probe command bridge (REPL + stream state machine).
 *
 * Protocol facts this file depends on (verified against the official
 * Mklink-AI-Probe sources):
 *   - the command channel is a CDC function, 115200 8N1, UTF-8, "\n"-terminated
 *   - a command finishes with the ">>>" prompt, which may be split across reads
 *   - a streaming command can return its prompt and the first stream bytes in the
 *     same read, so the transition is "armed" before the write and the split is
 *     done at the prompt
 *   - while streaming the prompt must not be interpreted (payload may contain it)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_types_ch9.h"

#include "mklink_core.h"
#include "mklink_probe.h"
#include "mklink_priv.h"
#include "mklink_rtt.h"
#include "mklink_watch.h"

static const char *TAG = "mklink.probe";

#define CDC_SET_LINE_CODING        0x20
#define CDC_SET_CONTROL_LINE_STATE 0x22

#define PROBE_PROMPT          ">>>"
#define PROBE_PROMPT_LEN      3
#define PROBE_IDENTITY_CMD    "print('__mklink_probe_7f3a__')"
#define PROBE_IDENTITY_TOKEN  "__mklink_probe_7f3a__"
#define PROBE_REPLY_MAX       768
#define PROBE_TX_TIMEOUT_MS   1000

static SemaphoreHandle_t s_cmd_lock;
static SemaphoreHandle_t s_prompt_sem;
static TaskHandle_t s_reader;

static volatile bool s_ready;
static volatile bool s_reader_run;
static volatile bool s_armed;
static volatile bool s_exiting;
static probe_stream_t s_stream = PROBE_STREAM_NONE;
static probe_stream_t s_armed_kind = PROBE_STREAM_NONE;
static mklink_probe_stream_cb_t s_stream_cb;

static uint8_t s_addr = MKLINK_USB_ADDR_NONE;
static uint8_t s_cmd_iface = 0xFF;
static uint8_t s_comm_iface = 0xFF;   /* interface claimed for the command CDC */
static uint8_t s_data_iface = 0xFF;   /* interface claimed for the data CDC */
static uint8_t s_ep_in;
static uint8_t s_ep_out;

/* command-mode accumulation (also carries the response handed to the caller) */
static char s_reply[PROBE_REPLY_MAX];
static size_t s_reply_len;

static void lock_init(void)
{
    if (s_cmd_lock == NULL) {
        s_cmd_lock = xSemaphoreCreateMutex();
    }
    if (s_prompt_sem == NULL) {
        s_prompt_sem = xSemaphoreCreateBinary();
    }
}

static void prompt_drain(void)
{
    if (s_prompt_sem != NULL) {
        while (xSemaphoreTake(s_prompt_sem, 0) == pdTRUE) {
            /* drop stale prompt */
        }
    }
}

static uint8_t cmd_iface_of(const mklink_cdc_group_t *g)
{
    return (g->comm_iface != 0xFF) ? g->comm_iface : g->data_iface;
}

/* ---------------------------------------------------------------- transport */

static esp_err_t cdc_control(uint8_t iface, uint8_t request, uint16_t value,
                             const void *data, size_t len)
{
    usb_setup_packet_t setup = {
        .bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS |
                         USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        .bRequest = request,
        .wValue = value,
        .wIndex = iface,
        .wLength = (uint16_t)len,
    };
    return mklink_cdc_sync_transfer(s_addr, 0, (uint8_t *)data, len, true, &setup, NULL);
}

static esp_err_t cdc_configure_interface(uint8_t iface)
{
    uint8_t line_coding[7] = {
        0x00, 0xC2, 0x01, 0x00, /* 115200 baud */
        0x00,                   /* 1 stop bit  */
        0x00,                   /* no parity   */
        0x08,                   /* 8 data bits */
    };
    esp_err_t err = cdc_control(iface, CDC_SET_LINE_CODING, 0, line_coding, sizeof(line_coding));
    if (err != ESP_OK) {
        return err;
    }
    return cdc_control(iface, CDC_SET_CONTROL_LINE_STATE, 0x0003 /* DTR|RTS */, NULL, 0);
}

esp_err_t mklink_probe_write_raw(const void *data, size_t len)
{
    if (!s_ready || data == NULL || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return mklink_cdc_sync_transfer(s_addr, s_ep_out, (uint8_t *)data, len, false, NULL, NULL);
}

static esp_err_t write_line(const char *text)
{
    char buf[512];
    size_t len = strlen(text);
    if (len + 1 >= sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(buf, text, len);
    buf[len++] = '\n';
    return mklink_probe_write_raw(buf, len);
}

/* ---------------------------------------------------------------- rx path */

/* One consumer fan-out: local pages and the PC stream hub both get the bytes. */
static void probe_stream_dispatch(probe_stream_t kind, const uint8_t *data, size_t len);

static void deliver_stream(const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }
    if (s_stream_cb != NULL) {
        s_stream_cb(s_stream, data, len);
    } else {
        probe_stream_dispatch(s_stream, data, len);
    }
}

static void probe_stream_dispatch(probe_stream_t kind, const uint8_t *data, size_t len)
{
    if (kind == PROBE_STREAM_RTT) {
        mklink_rtt_on_stream(data, len);
        mklink_stream_hub_publish(1, data, len);
    } else if (kind == PROBE_STREAM_DUMP) {
        mklink_watch_on_stream(data, len);
        mklink_stream_hub_publish(2, data, len);
    }
}

static void handle_stream_rx(const uint8_t *data, size_t len)
{
    if (s_exiting) {
        /* Only while leaving the stream: look for the terminal prompt and cut
         * there. Payload before it still belongs to the stream. */
        const uint8_t *hit = NULL;
        for (size_t i = 0; i + PROBE_PROMPT_LEN <= len; i++) {
            if (memcmp(data + i, PROBE_PROMPT, PROBE_PROMPT_LEN) == 0) {
                hit = data + i;
                break;
            }
        }
        if (hit != NULL) {
            deliver_stream(data, (size_t)(hit - data));
            s_stream = PROBE_STREAM_NONE;
            s_exiting = false;
            xSemaphoreGive(s_prompt_sem);
            return;
        }
    }
    deliver_stream(data, len);
}

static void handle_command_rx(const uint8_t *data, size_t len)
{
    size_t room = sizeof(s_reply) - 1 - s_reply_len;
    if (len > room) {
        /* keep the newest bytes: a huge reply means we lost the prompt point */
        size_t drop = len - room;
        if (drop >= s_reply_len) {
            s_reply_len = 0;
        } else {
            memmove(s_reply, s_reply + drop, s_reply_len - drop);
            s_reply_len -= drop;
        }
        room = sizeof(s_reply) - 1 - s_reply_len;
    }
    memcpy(s_reply + s_reply_len, data, len < room ? len : room);
    s_reply_len += (len < room) ? len : room;
    s_reply[s_reply_len] = '\0';

    char *prompt = strstr(s_reply, PROBE_PROMPT);
    if (prompt == NULL) {
        return;
    }

    size_t keep = (size_t)(prompt - s_reply);
    s_reply[keep] = '\0';
    s_reply_len = keep;

    /* Bytes after the prompt are already stream data when we were armed. */
    const char *after = prompt + PROBE_PROMPT_LEN;
    size_t after_len = strlen(after);

    if (s_armed) {
        s_armed = false;
        s_stream = s_armed_kind;
        if (after_len > 0) {
            deliver_stream((const uint8_t *)after, after_len);
        }
    }

    xSemaphoreGive(s_prompt_sem);
}

static void probe_rx_task(void *arg)
{
    uint8_t buf[512];

    while (s_reader_run) {
        if (!mklink_usb_device_alive(s_addr)) {
            break;
        }

        size_t got = 0;
        esp_err_t err = mklink_cdc_sync_transfer(s_addr, s_ep_in, buf, sizeof(buf), false, NULL, &got);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            if (!mklink_usb_device_alive(s_addr)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (got == 0) {
            continue;
        }

        if (s_stream != PROBE_STREAM_NONE) {
            handle_stream_rx(buf, got);
        } else {
            handle_command_rx(buf, got);
        }
    }

    s_ready = false;
    s_stream = PROBE_STREAM_NONE;
    s_armed = false;
    s_exiting = false;
    s_reader = NULL;
    mklink_log("PROBE: link closed");
    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------- commands */

bool mklink_probe_ready(void)
{
    return s_ready && s_reader != NULL;
}

uint8_t mklink_probe_cmd_iface(void)
{
    return s_cmd_iface;
}

static esp_err_t cmd_locked(const char *cmd, char *reply, size_t cap, uint32_t timeout_ms)
{
    s_reply_len = 0;
    s_reply[0] = '\0';
    prompt_drain();

    esp_err_t err = write_line(cmd);
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(s_prompt_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (reply != NULL && cap > 0) {
        strlcpy(reply, s_reply, cap);
    }
    return ESP_OK;
}

/** Consume whatever prompt the REPL already had pending before we started
 *  reading: otherwise the first command is "completed" by a stale prompt and its
 *  response is still in flight. */
static void probe_sync(void)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        s_reply_len = 0;
        s_reply[0] = '\0';
        prompt_drain();
        if (write_line("") != ESP_OK) {
            break;
        }
        if (xSemaphoreTake(s_prompt_sem, pdMS_TO_TICKS(400)) == pdTRUE) {
            break;
        }
    }
    s_reply_len = 0;
    s_reply[0] = '\0';
}

esp_err_t mklink_probe_cmd(const char *cmd, char *reply, size_t cap, uint32_t timeout_ms)
{
    if (!mklink_probe_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_stream != PROBE_STREAM_NONE) {
        return ESP_ERR_INVALID_STATE; /* a stream owns the channel */
    }

    xSemaphoreTake(s_cmd_lock, portMAX_DELAY);
    esp_err_t err = cmd_locked(cmd, reply, cap, timeout_ms);
    xSemaphoreGive(s_cmd_lock);
    return err;
}

esp_err_t mklink_probe_enter_stream(probe_stream_t kind, const char *cmd, uint32_t timeout_ms)
{
    if (!mklink_probe_ready() || kind == PROBE_STREAM_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_cmd_lock, portMAX_DELAY);
    if (s_stream != PROBE_STREAM_NONE) {
        xSemaphoreGive(s_cmd_lock);
        return ESP_ERR_INVALID_STATE;
    }

    s_reply_len = 0;
    s_reply[0] = '\0';
    prompt_drain();

    s_armed_kind = kind;
    s_armed = true;

    esp_err_t err = write_line(cmd);
    if (err == ESP_OK) {
        if (xSemaphoreTake(s_prompt_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            s_armed = false;
            err = ESP_ERR_TIMEOUT;
        } else if (s_stream != kind) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    } else {
        s_armed = false;
    }

    xSemaphoreGive(s_cmd_lock);
    return err;
}

esp_err_t mklink_probe_exit_stream(const char *stop_cmd)
{
    if (!mklink_probe_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_cmd_lock, portMAX_DELAY);

    if (s_stream == PROBE_STREAM_NONE) {
        /* nothing to leave; still resync so the next command starts clean */
        esp_err_t sync = cmd_locked(PROBE_IDENTITY_CMD, NULL, 0, 800);
        xSemaphoreGive(s_cmd_lock);
        return (sync == ESP_OK) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    prompt_drain();
    s_exiting = true;

    esp_err_t err = ESP_ERR_TIMEOUT;
    for (int attempt = 0; attempt < 2 && err != ESP_OK; attempt++) {
        if (stop_cmd != NULL) {
            (void)write_line(stop_cmd);
        }
        if (xSemaphoreTake(s_prompt_sem, pdMS_TO_TICKS(800)) == pdTRUE) {
            err = ESP_OK;
        }
    }

    if (err != ESP_OK) {
        /* Force the state back even if the probe stayed silent, then try to
         * resynchronise with the identity command (any stale output is drained
         * because serial commands are ordered). */
        s_exiting = false;
        s_stream = PROBE_STREAM_NONE;
        ESP_LOGW(TAG, "stream stop timed out, resyncing");
    }

    esp_err_t sync = cmd_locked(PROBE_IDENTITY_CMD, NULL, 0, 2000);
    if (sync != ESP_OK) {
        ESP_LOGW(TAG, "resync after stream failed: %s", esp_err_to_name(sync));
    }

    xSemaphoreGive(s_cmd_lock);
    return (err == ESP_OK && sync == ESP_OK) ? ESP_OK : ESP_FAIL;
}

void mklink_probe_set_stream_cb(mklink_probe_stream_cb_t cb)
{
    s_stream_cb = cb;
}

/* ---------------------------------------------------------------- lifecycle */

void mklink_probe_attach(void)
{
    if (s_reader != NULL) {
        return;
    }
    lock_init();
    if (s_cmd_lock == NULL || s_prompt_sem == NULL) {
        ESP_LOGE(TAG, "no memory for probe locks");
        return;
    }

    mklink_cdc_group_t groups[MKLINK_MAX_CDC_GROUPS];
    int count = mklink_usb_cdc_group_list(groups, MKLINK_MAX_CDC_GROUPS);
    if (count == 0) {
        mklink_log("PROBE: device has no CDC group");
        return;
    }

    int pick = 0;
    for (int i = 0; i < count; i++) {
        if (cmd_iface_of(&groups[i]) == CONFIG_MKLINK_PROBE_CMD_IFACE) {
            pick = i;
            break;
        }
    }

    uint8_t addr = mklink_usb_current_addr();
    if (addr == MKLINK_USB_ADDR_NONE) {
        return;
    }

    mklink_cdc_group_t *g = &groups[pick];
    if (g->comm_iface != 0xFF) {
        (void)mklink_usb_claim_interface(g->comm_iface);
    }
    if (mklink_usb_claim_interface(g->data_iface) != ESP_OK) {
        mklink_log("PROBE: claim if%u failed", g->data_iface);
        return;
    }

    s_addr = addr;
    s_ep_in = g->ep_in;
    s_ep_out = g->ep_out;
    s_cmd_iface = cmd_iface_of(g);
    s_comm_iface = g->comm_iface;
    s_data_iface = g->data_iface;

    if (cdc_configure_interface(s_cmd_iface) != ESP_OK) {
        ESP_LOGW(TAG, "line coding on if%u failed", s_cmd_iface);
    }

    s_reply_len = 0;
    s_ready = true;
    s_stream = PROBE_STREAM_NONE;
    s_reader_run = true;
    if (xTaskCreate(probe_rx_task, "mk_probe_rx", 4096, NULL, 6, &s_reader) != pdPASS) {
        s_ready = false;
        s_reader_run = false;
        mklink_log("PROBE: reader task creation failed");
        return;
    }

    /* Let the REPL settle, consume any prompt it printed before we started
     * reading, then verify we are really talking to the probe. */
    vTaskDelay(pdMS_TO_TICKS(50));
    probe_sync();
    char reply[96] = {0};
    esp_err_t err = mklink_probe_cmd(PROBE_IDENTITY_CMD, reply, sizeof(reply), 1500);
    if (err == ESP_OK && strstr(reply, PROBE_IDENTITY_TOKEN) != NULL) {
        mklink_log("PROBE: command channel if%u ready", s_cmd_iface);
    } else {
        mklink_log("PROBE: handshake on if%u failed (%s)", s_cmd_iface, esp_err_to_name(err));
    }
}

void mklink_probe_detach(void)
{
    uint8_t addr = s_addr;
    uint8_t ep_in = s_ep_in;
    uint8_t comm = s_comm_iface;
    uint8_t data = s_data_iface;

    s_reader_run = false;
    s_ready = false;
    s_armed = false;
    s_exiting = false;
    s_stream = PROBE_STREAM_NONE;

    /* Unblock the reader's pending IN transfer so it can exit promptly. */
    if (addr != MKLINK_USB_ADDR_NONE && ep_in != 0) {
        mklink_usb_raw_abort(addr, ep_in);
    }

    /* Wait for the reader task to finish (it sets s_reader = NULL on exit). */
    for (int i = 0; i < 250 && s_reader != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Give the interfaces back so another owner (usbip) can claim them. */
    if (comm != 0xFF) {
        mklink_usb_release_interface(comm);
    }
    if (data != 0xFF && data != comm) {
        mklink_usb_release_interface(data);
    }

    s_addr = MKLINK_USB_ADDR_NONE;
    s_cmd_iface = 0xFF;
    s_comm_iface = 0xFF;
    s_data_iface = 0xFF;
    s_ep_in = 0;
    s_ep_out = 0;
}
