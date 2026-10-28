/*
 * MKLink Remote - USB host bridge (multi-device).
 *
 * Design notes:
 *  - The USB host library is installed here instead of using the BSP helper,
 *    because the BSP's library task calls usb_host_device_free_all() whenever
 *    the "no clients" flag is raised, which would race with our persistent
 *    client during boot.
 *  - Devices are tracked in a small table so several devices behind a hub can be
 *    exported at once by the usbip server. Each slot owns its handle, descriptor
 *    snapshot, endpoint table and claim mask; the raw transfer layer addresses a
 *    slot purely by its USB device address.
 *  - Interfaces are *not* claimed while enumerating: usbip claims what it needs
 *    per connection, so the raw layer stays usable for a network passthrough.
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

#define MK_MAX_ENDPOINTS      16
#define MK_DESC_DUMP_LEN      512
#define MKLINK_USB_MAX_DEVICES 8

static usb_host_client_handle_t s_client;
static TaskHandle_t s_lib_task;
static TaskHandle_t s_client_task;

typedef struct {
    bool     used;
    volatile bool alive;
    uint8_t  addr;
    usb_device_handle_t hdl;
    uint32_t claimed_mask;
    mklink_usb_snapshot_t snap;
    mklink_ep_info_t eps[MK_MAX_ENDPOINTS];
    int      ep_count;
    uint8_t  iface_cls[32];
    mklink_cdc_group_t cdc[MKLINK_MAX_CDC_GROUPS];
    uint8_t  cdc_count;
    char     desc_dump[MK_DESC_DUMP_LEN + 1];
} usb_slot_t;

static usb_slot_t s_slots[MKLINK_USB_MAX_DEVICES];
static int s_pending_slot = -1;
static SemaphoreHandle_t s_lock;

/* In-flight transfer accounting, used to delay device teardown until aborted
 * transfers delivered their callbacks. */
static volatile int s_xfer_inflight;

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

/* ------------------------------------------------------------- slot helpers */

static void slot_clear_locked(usb_slot_t *s)
{
    s->used = false;
    s->alive = false;
    s->addr = MKLINK_USB_ADDR_NONE;
    s->hdl = NULL;
    s->claimed_mask = 0;
    s->ep_count = 0;
    s->cdc_count = 0;
    memset(s->cdc, 0, sizeof(s->cdc));
    memset(s->eps, 0, sizeof(s->eps));
    memset(s->iface_cls, 0xFF, sizeof(s->iface_cls));
    memset(&s->snap, 0, sizeof(s->snap));
    s->snap.cdc_iface = 0xFF;
    s->desc_dump[0] = '\0';
}

static usb_slot_t *slot_by_addr_locked(uint8_t addr)
{
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (s_slots[i].used && s_slots[i].addr == addr) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static usb_slot_t *slot_free_locked(void)
{
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (!s_slots[i].used) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static usb_slot_t *slot_first_locked(void)
{
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (s_slots[i].used) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static usb_slot_t *slot_nth_locked(int idx)
{
    int n = 0;
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (!s_slots[i].used) {
            continue;
        }
        if (n++ == idx) {
            return &s_slots[i];
        }
    }
    return NULL;
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

bool mklink_usb_device_alive(uint8_t dev_addr)
{
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool alive = (s != NULL && s->alive);
    unlock();
    return alive;
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
    bool busy = (slot_free_locked() == NULL);
    unlock();
    return busy;
}

static void on_device_connected(uint8_t dev_addr)
{
    /* Probe auto-discovery decides what to do with this device: it describes it
     * and adopts it into a free slot (probe bridge or generic passthrough). */
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
    out->dev_class = dd->bDeviceClass;

    /* Walk the raw descriptor chain: simpler and more robust than type lookups. */
    uint8_t iface_order[32];
    bool    iface_seen[32] = {false};
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
            uint8_t n = p[2];
            iface_num = n;
            if (n < 32) {
                /* Alternate settings repeat bInterfaceNumber, so dedupe with an
                 * explicit bitmap. Using iface_cls[n]==0xFF as the sentinel broke
                 * vendor-specific devices (their interface class really is 0xFF):
                 * every alternate setting looked new and was claimed repeatedly. */
                if (!iface_seen[n]) {
                    iface_seen[n] = true;
                    out->iface_cls[n] = p[5];
                    if (iface_order_n < (int)sizeof(iface_order)) {
                        iface_order[iface_order_n++] = n;
                    }
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

    /* bNumInterfaces is authoritative (it excludes alternate settings). */
    out->iface_cnt = cd->bNumInterfaces;

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
    usb_slot_t *s = slot_free_locked();
    if (s == NULL) {
        unlock();
        return ESP_ERR_NO_MEM;
    }

    s->used = true;
    s->alive = false;              /* published by confirm_adopted() */
    s->addr = desc->addr;
    s->hdl = desc->hdl;
    desc->hdl = NULL;              /* ownership moved to the slot */
    s->ep_count = desc->ep_count;
    s->cdc_count = desc->cdc_count;
    memcpy(s->eps, desc->eps, sizeof(mklink_ep_info_t) * desc->ep_count);
    memcpy(s->iface_cls, desc->iface_cls, sizeof(s->iface_cls));
    memcpy(s->cdc, desc->cdc, sizeof(mklink_cdc_group_t) * desc->cdc_count);
    s->claimed_mask = 0;

    memset(&s->snap, 0, sizeof(s->snap));
    s->snap.connected = true;
    s->snap.vid = desc->vid;
    s->snap.pid = desc->pid;
    s->snap.dev_addr = desc->addr;
    s->snap.iface_cnt = desc->iface_cnt;
    s->snap.ep_cnt = desc->ep_count;
    s->snap.cdc_iface = desc->cdc_count ? cdc_group_cmd_iface(&desc->cdc[0]) : 0xFF;
    strlcpy(s->snap.manufacturer, desc->manufacturer, sizeof(s->snap.manufacturer));
    strlcpy(s->snap.product, desc->product, sizeof(s->snap.product));
    strlcpy(s->desc_dump, desc->topology, sizeof(s->desc_dump));

    s_pending_slot = (int)(s - s_slots);
    unlock();

    return ESP_OK;
}

void mklink_usb_confirm_adopted(void)
{
    mklink_usb_snapshot_t snap;
    char dump_copy[MK_DESC_DUMP_LEN + 1];

    lock();
    usb_slot_t *s = (s_pending_slot >= 0) ? &s_slots[s_pending_slot] : NULL;
    if (s == NULL || !s->used) {
        unlock();
        return;
    }
    s->alive = true;
    snap = s->snap;
    strlcpy(dump_copy, s->desc_dump, sizeof(dump_copy));
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

    lock();
    s_pending_slot = -1;
    unlock();
}

void mklink_usb_discard_adopted(void)
{
    usb_device_handle_t hdl = NULL;
    usb_slot_t *s = NULL;

    lock();
    if (s_pending_slot >= 0) {
        s = &s_slots[s_pending_slot];
        hdl = s->hdl;
        slot_clear_locked(s);
        s_pending_slot = -1;
    }
    unlock();

    if (hdl == NULL) {
        return;
    }

    for (int i = 0; i < 32; i++) {
        usb_host_interface_release(s_client, hdl, i);
    }
    usb_host_device_close(s_client, hdl);
    ESP_LOGD(TAG, "device discarded (failed validation)");
}

int mklink_usb_cdc_group_list(mklink_cdc_group_t *out, size_t max)
{
    lock();
    usb_slot_t *s = slot_first_locked();
    int n = 0;
    if (s != NULL) {
        for (int i = 0; i < s->cdc_count && (size_t)i < max; i++) {
            out[n++] = s->cdc[i];
        }
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

    /* Release every interface: a consumer may have claimed one between the
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

    ESP_LOGD(TAG, "teardown done (heap %u)", (unsigned)esp_get_free_heap_size());
    free(job);
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

    mklink_usb_snapshot_t snap;
    bool found = false;

    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    if (s != NULL) {
        s->alive = false;          /* block new submissions immediately */
        snap = s->snap;
        job->hdl = s->hdl;
        job->claimed_mask = s->claimed_mask;
        job->addr = dev_addr;
        slot_clear_locked(s);      /* free the slot right away */
        found = true;
    }
    unlock();

    if (!found || job->hdl == NULL) {
        free(job);
        return;
    }

    mklink_log("USB: device addr=%u removed", dev_addr);
    esp_event_post(MKLINK_EVENT, MKLINK_EVENT_USB_REMOVED, &snap, sizeof(snap), 0);

    /* NOTE: do NOT flush endpoints here. usb_host_endpoint_flush() is serviced
     * by the client task itself, and this callback runs inside it - calling it
     * deadlocks the client, after which no USB event is ever delivered again.
     * The stack already flushes and retires every URB of a removed device. */
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
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
        /* The event only carries the handle: map it back to an address. */
        uint8_t addr = MKLINK_USB_ADDR_NONE;
        lock();
        for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
            if (s_slots[i].used && s_slots[i].hdl == event_msg->dev_gone.dev_hdl) {
                addr = s_slots[i].addr;
                break;
            }
        }
        unlock();
        if (addr != MKLINK_USB_ADDR_NONE) {
            on_device_gone(addr);
        }
        break;
    }
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

    lock();
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        slot_clear_locked(&s_slots[i]);
    }
    unlock();

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

int mklink_usb_device_count(void)
{
    lock();
    int n = 0;
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (s_slots[i].used && s_slots[i].alive) {
            n++;
        }
    }
    unlock();
    return n;
}

uint8_t mklink_usb_addr_at(int idx)
{
    lock();
    usb_slot_t *s = slot_nth_locked(idx);
    uint8_t addr = (s != NULL) ? s->addr : MKLINK_USB_ADDR_NONE;
    unlock();
    return addr;
}

bool mklink_usb_get_snapshot_at(int idx, mklink_usb_snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }
    lock();
    usb_slot_t *s = slot_nth_locked(idx);
    bool ok = (s != NULL && s->alive);
    if (ok) {
        *out = s->snap;
    }
    unlock();
    if (!ok) {
        memset(out, 0, sizeof(*out));
        out->cdc_iface = 0xFF;
    }
    return ok;
}

bool mklink_usb_snapshot_for_addr(uint8_t dev_addr, mklink_usb_snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool ok = (s != NULL && s->alive);
    if (ok) {
        *out = s->snap;
    }
    unlock();
    if (!ok) {
        memset(out, 0, sizeof(*out));
        out->cdc_iface = 0xFF;
    }
    return ok;
}

bool mklink_usb_get_snapshot(mklink_usb_snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }
    lock();
    usb_slot_t *s = slot_first_locked();
    bool connected = (s != NULL && s->alive);
    if (s != NULL) {
        *out = s->snap;
    } else {
        memset(out, 0, sizeof(*out));
        out->cdc_iface = 0xFF;
    }
    unlock();
    return connected;
}

size_t mklink_usb_descriptor_dump(char *dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0) {
        return 0;
    }
    lock();
    usb_slot_t *s = slot_first_locked();
    if (s != NULL) {
        strlcpy(dst, s->desc_dump, dst_len);
    } else {
        dst[0] = '\0';
    }
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
    lock();
    usb_slot_t *s = slot_first_locked();
    uint8_t addr = (s != NULL) ? s->addr : MKLINK_USB_ADDR_NONE;
    unlock();
    return addr;
}

bool mklink_usb_handle_for_addr(uint8_t dev_addr, usb_device_handle_t *out)
{
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool ok = (s != NULL && s->hdl != NULL);
    if (ok && out != NULL) {
        *out = s->hdl;
    }
    unlock();
    return ok;
}

esp_err_t mklink_usb_claim_interface(uint8_t dev_addr, uint8_t iface_num)
{
    if (iface_num >= 32) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    if (s == NULL || s->hdl == NULL) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    bool already = (s->claimed_mask & (1u << iface_num)) != 0;
    usb_device_handle_t hdl = s->hdl;
    unlock();

    if (already) {
        return ESP_OK;
    }

    esp_err_t err = usb_host_interface_claim(s_client, hdl, iface_num, 0);
    if (err == ESP_OK) {
        lock();
        s = slot_by_addr_locked(dev_addr);
        if (s != NULL) {
            s->claimed_mask |= (1u << iface_num);
        }
        unlock();
    }
    return err;
}

esp_err_t mklink_usb_release_interface(uint8_t dev_addr, uint8_t iface_num)
{
    if (iface_num >= 32) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    if (s == NULL || s->hdl == NULL) {
        unlock();
        return ESP_ERR_INVALID_ARG;
    }
    bool claimed = (s->claimed_mask & (1u << iface_num)) != 0;
    usb_device_handle_t hdl = s->hdl;
    if (claimed) {
        s->claimed_mask &= ~(1u << iface_num);
    }
    unlock();

    return claimed ? usb_host_interface_release(s_client, hdl, iface_num) : ESP_OK;
}

bool mklink_usb_cdc_ifaces(uint8_t *comm, uint8_t *data)
{
    lock();
    usb_slot_t *s = slot_first_locked();
    uint8_t c = 0xFF;
    uint8_t d = 0xFF;
    if (s != NULL && s->cdc_count > 0) {
        c = s->cdc[0].comm_iface;
        d = s->cdc[0].data_iface;
    }
    unlock();
    if (comm != NULL) {
        *comm = c;
    }
    if (data != NULL) {
        *data = d;
    }
    return (d != 0xFF);
}

int mklink_usb_endpoint_list(uint8_t dev_addr, mklink_ep_info_t *out, size_t max)
{
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    int count = 0;
    if (s != NULL) {
        count = s->ep_count < (int)max ? s->ep_count : (int)max;
        memcpy(out, s->eps, sizeof(mklink_ep_info_t) * count);
    }
    unlock();
    return count;
}
