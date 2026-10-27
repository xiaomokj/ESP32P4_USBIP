/*
 * MKLink Remote - USB host bridge.
 *
 * Design notes:
 *  - The USB host library is installed here instead of using the BSP helper,
 *    because the BSP's library task calls usb_host_device_free_all() whenever
 *    the "no clients" flag is raised, which would race with our persistent
 *    client during boot.
 *  - Interfaces are *not* claimed while enumerating. The CDC loopback test (and
 *    the future usbip server) claims only what it needs, so the raw transfer
 *    layer stays usable for a network passthrough.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
#include "usb/usb_types_ch9.h"

#include "mklink_core.h"
#include "mklink_usb.h"
#include "mklink_priv.h"
#include "mklink_probe.h"

static const char *TAG = "mklink.usb";

#define MK_MAX_ENDPOINTS  16
#define MK_DESC_DUMP_LEN  512

static usb_host_client_handle_t s_client;
static TaskHandle_t s_lib_task;
static TaskHandle_t s_client_task;

static usb_device_handle_t s_dev_hdl;
static uint8_t s_dev_addr = MKLINK_USB_ADDR_NONE;
static uint32_t s_claimed_mask;
static mklink_usb_snapshot_t s_snap;
static mklink_ep_info_t s_eps[MK_MAX_ENDPOINTS];
static int s_ep_count;
static uint8_t s_cdc_comm_iface = 0xFF;
static uint8_t s_cdc_data_iface = 0xFF;
static uint8_t s_iface_cls[32];
static mklink_cdc_group_t s_cdc_groups[MKLINK_MAX_CDC_GROUPS];
static uint8_t s_cdc_group_count;
static char s_desc_dump[MK_DESC_DUMP_LEN + 1];
static SemaphoreHandle_t s_lock;

/* Device lifetime: s_dev_alive is cleared the moment the stack reports DEV_GONE
 * so no new transfer can be submitted, and the actual release/close happens later
 * from a teardown task (never inside the client callback). */
static volatile bool s_dev_alive;
static volatile bool s_teardown_pending;
static uint32_t s_dev_gen;
static volatile int s_xfer_inflight;
static uint8_t s_deferred_addr = MKLINK_USB_ADDR_NONE;

static void lock_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
}

static void lock(void)
{
    lock_init();
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

int mklink_utf16_to_utf8(const uint16_t *src, int src_len, char *dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0) {
        return 0;
    }
    size_t out = 0;
    for (int i = 0; i < src_len && out + 4 < dst_len; i++) {
        uint32_t cp = src[i];
        if (cp == 0) {
            break;
        }
        if (cp >= 0xD800 && cp <= 0xDBFF && i + 1 < src_len) {
            uint32_t low = src[i + 1];
            if (low >= 0xDC00 && low <= 0xDFFF) {
                cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
                i++;
            }
        }
        if (cp < 0x80) {
            dst[out++] = (char)cp;
        } else if (cp < 0x800) {
            dst[out++] = (char)(0xC0 | (cp >> 6));
            dst[out++] = (char)(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            dst[out++] = (char)(0xE0 | (cp >> 12));
            dst[out++] = (char)(0x80 | ((cp >> 6) & 0x3F));
            dst[out++] = (char)(0x80 | (cp & 0x3F));
        } else {
            dst[out++] = (char)(0xF0 | (cp >> 18));
            dst[out++] = (char)(0x80 | ((cp >> 12) & 0x3F));
            dst[out++] = (char)(0x80 | ((cp >> 6) & 0x3F));
            dst[out++] = (char)(0x80 | (cp & 0x3F));
        }
    }
    dst[out] = '\0';
    return (int)out;
}

static void append(char *buf, size_t len, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void append(char *buf, size_t len, const char *fmt, ...)
{
    size_t used = strlen(buf);
    if (used + 1 >= len) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf + used, len - used, fmt, args);
    va_end(args);
}

/* Device table / descriptor formatting now lives next to mklink_usb_describe(). */

/* Reset the bookkeeping only - the caller owns handle/interface teardown. */
static void clear_state_locked(void)
{
    s_dev_hdl = NULL;
    s_dev_addr = MKLINK_USB_ADDR_NONE;
    s_claimed_mask = 0;
    s_ep_count = 0;
    s_cdc_comm_iface = 0xFF;
    s_cdc_data_iface = 0xFF;
    s_cdc_group_count = 0;
    memset(s_cdc_groups, 0, sizeof(s_cdc_groups));
    s_dev_alive = false;
    memset(s_iface_cls, 0xFF, sizeof(s_iface_cls));
    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.cdc_iface = 0xFF;
    s_desc_dump[0] = '\0';
    mklink_cdc_store_result(NULL);
}

bool mklink_usb_device_alive(uint8_t dev_addr)
{
    return s_dev_alive && (s_dev_addr == dev_addr);
}

void mklink_usb_xfer_begin(void)
{
    __atomic_add_fetch(&s_xfer_inflight, 1, __ATOMIC_SEQ_CST);
}

void mklink_usb_xfer_end(void)
{
    __atomic_sub_fetch(&s_xfer_inflight, 1, __ATOMIC_SEQ_CST);
}

bool mklink_usb_busy(void)
{
    lock();
    bool busy = (s_dev_hdl != NULL);
    unlock();
    return busy;
}

static void on_device_connected(uint8_t dev_addr)
{
    lock();
    bool busy = (s_dev_hdl != NULL);
    bool tearing_down = s_teardown_pending;
    if (tearing_down) {
        /* Remember it: the teardown task retries once the handle is closed. */
        s_deferred_addr = dev_addr;
    }
    unlock();
    if (busy || tearing_down) {
        mklink_log("USB: device addr %u %s", dev_addr,
                   busy ? "ignored (another device already attached)" : "deferred (teardown in progress)");
        return;
    }

    /* Probe auto-discovery decides whether this device is our command bridge;
     * it describes the device, probes the CDC groups and adopts it on success. */
    mklink_discovery_on_device(dev_addr);
}

esp_err_t mklink_usb_describe(uint8_t addr, mklink_usb_desc_t *out)
{
    if (out == NULL || s_client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->addr = addr;
    out->cdc_count = 0;

    usb_device_handle_t hdl = NULL;
    esp_err_t err = usb_host_device_open(s_client, addr, &hdl);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "open addr %u failed: %s", addr, esp_err_to_name(err));
        return err;
    }

    const usb_device_desc_t *dd = NULL;
    const usb_config_desc_t *cd = NULL;
    if (usb_host_get_device_descriptor(hdl, &dd) != ESP_OK ||
        usb_host_get_active_config_descriptor(hdl, &cd) != ESP_OK || dd == NULL || cd == NULL) {
        ESP_LOGW(TAG, "addr %u: descriptor fetch failed", addr);
        usb_host_device_close(s_client, hdl);
        return ESP_ERR_NOT_FOUND;
    }

    out->hdl = hdl;
    out->vid = dd->idVendor;
    out->pid = dd->idProduct;

    /* Walk the raw descriptor chain: simpler and more robust than type lookups. */
    uint8_t iface_order[32];
    int iface_order_n = 0;
    uint8_t iface_num = 0;
    memset(out->iface_cls, 0xFF, sizeof(out->iface_cls));

    const uint8_t *p = (const uint8_t *)cd;
    const uint8_t *end = p + cd->wTotalLength;
    while (p + 2 <= end) {
        uint8_t dlen = p[0];
        uint8_t dtype = p[1];
        if (dlen < 2) {
            break;
        }
        if (dtype == USB_B_DESCRIPTOR_TYPE_INTERFACE && dlen >= 9) {
            iface_num = p[2];
            out->iface_cnt++;
            if (iface_num < 32) {
                out->iface_cls[iface_num] = p[5];
                if (iface_order_n < (int)sizeof(iface_order)) {
                    iface_order[iface_order_n++] = iface_num;
                }
            }
        } else if (dtype == USB_B_DESCRIPTOR_TYPE_ENDPOINT && dlen >= 7 && out->ep_count < MK_MAX_ENDPOINTS) {
            mklink_ep_info_t *ep = &out->eps[out->ep_count];
            ep->ep_addr = p[2];
            ep->attributes = p[3] & 0x03;
            ep->mps = (uint16_t)(p[4] | (p[5] << 8));
            ep->interval = p[6];
            ep->iface_num = iface_num;
            out->ep_count++;
        }
        p += dlen;
    }

    /* CDC functions: a data interface (0x0A) follows its communication interface
     * (0x02). Composite probes expose several of them, and only one carries the
     * command REPL, so discovery needs every group. */
    for (int i = 0; i < iface_order_n && out->cdc_count < MKLINK_MAX_CDC_GROUPS; i++) {
        uint8_t ifn = iface_order[i];
        if (out->iface_cls[ifn] != 0x0A) {
            continue;
        }

        mklink_cdc_group_t *g = &out->cdc[out->cdc_count];
        memset(g, 0, sizeof(*g));
        g->comm_iface = 0xFF;
        g->data_iface = ifn;
        if (i > 0 && out->iface_cls[iface_order[i - 1]] == 0x02) {
            g->comm_iface = iface_order[i - 1];
        }
        for (int e = 0; e < out->ep_count; e++) {
            if (out->eps[e].iface_num != ifn || out->eps[e].attributes != 0x02) {
                continue;
            }
            if (out->eps[e].ep_addr & 0x80) {
                if (g->ep_in == 0) {
                    g->ep_in = out->eps[e].ep_addr;
                    g->mps_in = out->eps[e].mps;
                }
            } else if (g->ep_out == 0) {
                g->ep_out = out->eps[e].ep_addr;
                g->mps_out = out->eps[e].mps;
            }
        }
        if (g->ep_in != 0 && g->ep_out != 0) {
            out->cdc_count++;
        }
    }

    usb_device_info_t info;
    if (usb_host_device_info(out->hdl, &info) == ESP_OK) {
        if (info.str_desc_manufacturer != NULL) {
            mklink_utf16_to_utf8(info.str_desc_manufacturer->wData,
                                 (info.str_desc_manufacturer->bLength - 2) / 2,
                                 out->manufacturer, sizeof(out->manufacturer));
        }
        if (info.str_desc_product != NULL) {
            mklink_utf16_to_utf8(info.str_desc_product->wData,
                                 (info.str_desc_product->bLength - 2) / 2,
                                 out->product, sizeof(out->product));
        }
    }
    if (out->product[0] == '\0') {
        strlcpy(out->product, "USB device", sizeof(out->product));
    }

    /* Topology summary, one interface per line (the UI log is 96 chars wide). */
    char *dump = out->topology;
    size_t cap = sizeof(out->topology);
    dump[0] = '\0';
    append(dump, cap, "%04X:%04X class %02X/%02X/%02X | cfg: %u iface / %u ep / %u cdc",
           out->vid, out->pid, dd->bDeviceClass, dd->bDeviceSubClass, dd->bDeviceProtocol,
           out->iface_cnt, out->ep_count, out->cdc_count);
    for (int i = 0; i < iface_order_n; i++) {
        uint8_t ifn = iface_order[i];
        append(dump, cap, "\nif%u c%02X", ifn, out->iface_cls[ifn]);
        for (int e = 0; e < out->ep_count; e++) {
            if (out->eps[e].iface_num != ifn) {
                continue;
            }
            append(dump, cap, " ep%02X%s/%u", out->eps[e].ep_addr,
                   out->eps[e].attributes == 0x02 ? "b" : (out->eps[e].attributes == 0x03 ? "i" : "s"),
                   out->eps[e].mps);
        }
    }

    return ESP_OK;
}

void mklink_usb_desc_release(mklink_usb_desc_t *desc)
{
    if (desc == NULL || desc->hdl == NULL) {
        return;
    }
    usb_host_device_close(s_client, desc->hdl);
    desc->hdl = NULL;
}

static uint8_t cdc_group_cmd_iface(const mklink_cdc_group_t *g)
{
    return (g->comm_iface != 0xFF) ? g->comm_iface : g->data_iface;
}

esp_err_t mklink_usb_adopt(mklink_usb_desc_t *desc)
{
    if (desc == NULL || desc->hdl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    if (s_dev_hdl != NULL) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }

    s_dev_hdl = desc->hdl;
    desc->hdl = NULL;              /* ownership moved to the bridge */
    s_dev_addr = desc->addr;
    s_ep_count = desc->ep_count;
    memcpy(s_eps, desc->eps, sizeof(mklink_ep_info_t) * desc->ep_count);
    memcpy(s_iface_cls, desc->iface_cls, sizeof(s_iface_cls));
    memcpy(s_cdc_groups, desc->cdc, sizeof(mklink_cdc_group_t) * desc->cdc_count);
    s_cdc_group_count = desc->cdc_count;
    s_cdc_comm_iface = desc->cdc_count ? desc->cdc[0].comm_iface : 0xFF;
    s_cdc_data_iface = desc->cdc_count ? desc->cdc[0].data_iface : 0xFF;

    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.connected = true;
    s_snap.vid = desc->vid;
    s_snap.pid = desc->pid;
    s_snap.dev_addr = desc->addr;
    s_snap.iface_cnt = desc->iface_cnt;
    s_snap.ep_cnt = desc->ep_count;
    s_snap.cdc_iface = desc->cdc_count ? cdc_group_cmd_iface(&desc->cdc[0]) : 0xFF;
    strlcpy(s_snap.manufacturer, desc->manufacturer, sizeof(s_snap.manufacturer));
    strlcpy(s_snap.product, desc->product, sizeof(s_snap.product));

    strlcpy(s_desc_dump, desc->topology, sizeof(s_desc_dump));
    s_dev_alive = true;
    s_dev_gen++;
    unlock();

    return ESP_OK;
}

void mklink_usb_confirm_adopted(void)
{
    mklink_usb_snapshot_t snap;
    char dump_copy[sizeof(s_desc_dump)];

    lock();
    snap = s_snap;
    strlcpy(dump_copy, s_desc_dump, sizeof(dump_copy));
    unlock();

    mklink_log("USB: %04X:%04X \"%s\" addr=%u if=%u ep=%u",
               snap.vid, snap.pid, snap.product, snap.dev_addr, snap.iface_cnt, snap.ep_cnt);
    for (char *line = strtok(dump_copy, "\n"); line != NULL; line = strtok(NULL, "\n")) {
        mklink_log("USB| %s", line);
    }

    esp_event_post(MKLINK_EVENT, MKLINK_EVENT_USB_ADDED, &snap, sizeof(snap), 0);

    /* The probe bridge is the primary consumer of the command interface. When the
     * usbip server owns the device (CONFIG_MKLINK_PROBE_ENABLE=n) it must stay
     * untouched so the passthrough can claim every interface cleanly. */
#if CONFIG_MKLINK_PROBE_ENABLE
    mklink_probe_attach();
#endif

#if CONFIG_MKLINK_CDC_TEST_ENABLE
    if (snap.cdc_iface != 0xFF) {
        mklink_cdc_test_start();
    }
#endif
}

void mklink_usb_discard_adopted(void)
{
    usb_device_handle_t hdl;

    mklink_probe_detach();

    lock();
    hdl = s_dev_hdl;
    unlock();

    if (hdl == NULL) {
        return;
    }

    for (int i = 0; i < 32; i++) {
        usb_host_interface_release(s_client, hdl, i);
    }
    usb_host_device_close(s_client, hdl);

    lock();
    clear_state_locked();
    unlock();

    ESP_LOGD(TAG, "device discarded (failed validation)");
}

int mklink_usb_cdc_group_list(mklink_cdc_group_t *out, size_t max)
{
    lock();
    int n = 0;
    for (int i = 0; i < s_cdc_group_count && (size_t)i < max; i++) {
        out[n++] = s_cdc_groups[i];
    }
    unlock();
    return n;
}

int mklink_usb_addr_list(uint8_t *addrs, size_t max)
{
    if (s_client == NULL || addrs == NULL || max == 0) {
        return 0;
    }
    int count = 0;
    if (usb_host_device_addr_list_fill((int)max, addrs, &count) != ESP_OK) {
        return 0;
    }
    return count;
}

typedef struct {
    usb_device_handle_t hdl;
    uint32_t claimed_mask;
    uint8_t addr;
} teardown_job_t;

static void teardown_run(teardown_job_t *job)
{
    /* Give the cancelled transfers a chance to deliver their callbacks (they own
     * and free their own contexts) before the handle disappears. */
    for (int i = 0; i < 50 && s_xfer_inflight > 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_xfer_inflight > 0) {
        ESP_LOGW(TAG, "teardown: %d transfer(s) still in flight", s_xfer_inflight);
    }

    /* Release every interface: the CDC task may have claimed one between the
     * snapshot and the removal, and a stuck claim makes device_close fail. */
    for (int i = 0; i < 32; i++) {
        esp_err_t err = usb_host_interface_release(s_client, job->hdl, i);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "release if%u: %s", i, esp_err_to_name(err));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    esp_err_t err = usb_host_device_close(s_client, job->hdl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "device close addr=%u: %s", job->addr, esp_err_to_name(err));
    }

    uint8_t deferred = MKLINK_USB_ADDR_NONE;
    lock();
    s_teardown_pending = false;
    deferred = s_deferred_addr;
    s_deferred_addr = MKLINK_USB_ADDR_NONE;
    unlock();

    ESP_LOGD(TAG, "teardown done (heap %u)", (unsigned)esp_get_free_heap_size());
    free(job);

    /* A device that showed up while we were tearing down was skipped; retry now. */
    if (deferred != MKLINK_USB_ADDR_NONE) {
        on_device_connected(deferred);
    }
}

static void usb_teardown_task(void *arg)
{
    teardown_run((teardown_job_t *)arg);
    vTaskDelete(NULL);
}

static void on_device_gone(uint8_t dev_addr)
{
    teardown_job_t *job = calloc(1, sizeof(*job));
    if (job == NULL) {
        ESP_LOGE(TAG, "no memory for teardown");
        return;
    }

    lock();
    if (s_dev_hdl == NULL || s_dev_addr != dev_addr) {
        unlock();
        free(job);
        return;
    }

    /* Block every new submission first, then hand the handle to the job. */
    s_dev_alive = false;
    s_dev_gen++;
    s_teardown_pending = true;

    mklink_probe_detach();

    mklink_usb_snapshot_t snap = s_snap;
    job->hdl = s_dev_hdl;
    job->claimed_mask = s_claimed_mask;
    job->addr = dev_addr;

    s_dev_hdl = NULL;
    s_claimed_mask = 0;
    clear_state_locked();
    unlock();

    mklink_log("USB: device addr=%u removed", dev_addr);
    esp_event_post(MKLINK_EVENT, MKLINK_EVENT_USB_REMOVED, &snap, sizeof(snap), 0);

    /* NOTE: do NOT flush endpoints here. usb_host_endpoint_flush() is serviced
     * by the client task itself, and this callback runs inside it - calling it
     * deadlocks the client, after which no USB event (including a re-plug) is
     * ever delivered again. The stack already flushes and retires every URB of
     * a removed device by itself, invoking their callbacks. */

    if (xTaskCreate(usb_teardown_task, "mk_usb_teardown", 3072, job, 5, NULL) != pdPASS) {
        ESP_LOGW(TAG, "teardown task creation failed, closing inline");
        teardown_run(job);
    }
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        on_device_connected(event_msg->new_dev.address);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        if (event_msg->dev_gone.dev_hdl == s_dev_hdl) {
            on_device_gone(s_dev_addr);
        }
        break;
    default:
        break;
    }
}

static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGD(TAG, "usb lib: no clients");
        }
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGD(TAG, "usb lib: all devices freed");
        }
    }
}

static void usb_client_task(void *arg)
{
    while (true) {
        usb_host_client_handle_events(s_client, portMAX_DELAY);
    }
}

esp_err_t mklink_usb_start(void)
{
    if (s_client != NULL) {
        return ESP_OK;
    }

    lock_init();

    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        return err;
    }

    if (xTaskCreate(usb_lib_task, "mk_usb_lib", 4096, NULL, 10, &s_lib_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    const usb_host_client_config_t client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL,
        },
    };
    err = usb_host_client_register(&client_cfg, &s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_client_register: %s", esp_err_to_name(err));
        return err;
    }

    if (xTaskCreate(usb_client_task, "mk_usb_cli", 4096, NULL, 5, &s_client_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.cdc_iface = 0xFF;

    mklink_log("USB: host stack ready (filter %04X:%04X)", CONFIG_MKLINK_USB_VID, CONFIG_MKLINK_USB_PID);
    return mklink_usb_rescan();
}

esp_err_t mklink_usb_rescan(void)
{
    if (s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addrs[8] = {0};
    int count = 0;
    if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &count) != ESP_OK) {
        return ESP_FAIL;
    }

    for (int i = 0; i < count; i++) {
        if (addrs[i] == 0) {
            continue;
        }
        on_device_connected(addrs[i]);
    }

    esp_event_post(MKLINK_EVENT, MKLINK_EVENT_USB_SCAN_DONE, NULL, 0, 0);
    return ESP_OK;
}

bool mklink_usb_get_snapshot(mklink_usb_snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }
    lock();
    bool connected = s_snap.connected;
    *out = s_snap;
    unlock();
    return connected;
}

size_t mklink_usb_descriptor_dump(char *dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0) {
        return 0;
    }
    lock();
    strlcpy(dst, s_desc_dump, dst_len);
    size_t len = strlen(dst);
    unlock();
    return len;
}

usb_host_client_handle_t mklink_usb_client_handle(void)
{
    return s_client;
}

uint8_t mklink_usb_current_addr(void)
{
    return s_dev_addr;
}

bool mklink_usb_handle_for_addr(uint8_t dev_addr, usb_device_handle_t *out)
{
    lock();
    bool ok = (s_dev_hdl != NULL && s_dev_addr == dev_addr);
    if (ok && out != NULL) {
        *out = s_dev_hdl;
    }
    unlock();
    return ok;
}

esp_err_t mklink_usb_claim_interface(uint8_t iface_num)
{
    if (s_dev_hdl == NULL || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (iface_num >= 32) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    bool already = (s_claimed_mask & (1u << iface_num)) != 0;
    usb_device_handle_t hdl = s_dev_hdl;
    unlock();

    if (already) {
        return ESP_OK;
    }

    esp_err_t err = usb_host_interface_claim(s_client, hdl, iface_num, 0);
    if (err == ESP_OK) {
        lock();
        s_claimed_mask |= (1u << iface_num);
        unlock();
    }
    return err;
}

esp_err_t mklink_usb_release_interface(uint8_t iface_num)
{
    if (s_dev_hdl == NULL || s_client == NULL || iface_num >= 32) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    bool claimed = (s_claimed_mask & (1u << iface_num)) != 0;
    usb_device_handle_t hdl = s_dev_hdl;
    if (claimed) {
        s_claimed_mask &= ~(1u << iface_num);
    }
    unlock();

    return claimed ? usb_host_interface_release(s_client, hdl, iface_num) : ESP_OK;
}

bool mklink_usb_cdc_ifaces(uint8_t *comm, uint8_t *data)
{
    lock();
    if (comm != NULL) {
        *comm = s_cdc_comm_iface;
    }
    if (data != NULL) {
        *data = s_cdc_data_iface;
    }
    bool found = (s_cdc_data_iface != 0xFF);
    unlock();
    return found;
}

int mklink_usb_endpoint_list(mklink_ep_info_t *out, size_t max)
{
    lock();
    int count = s_ep_count < (int)max ? s_ep_count : (int)max;
    memcpy(out, s_eps, sizeof(mklink_ep_info_t) * count);
    unlock();
    return count;
}
