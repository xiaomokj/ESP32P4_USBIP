/*
 * MKLink Remote - USB device table on top of the CherryUSB host stack.
 *
 * Design notes:
 *  - CherryUSB enumerates the root port and every external hub itself (the hub
 *    class driver handles port reset, split transactions and multi-TT hubs), so
 *    FS/LS devices behind a high speed hub work. The application only observes
 *    the result through the two weak hooks in usbh_core.h.
 *  - No CherryUSB class driver is compiled in except the hub one: a device that
 *    has to be exported over usbip must stay unbound so the passthrough can send
 *    arbitrary URBs to it.
 *  - Devices are tracked in a small table so several devices behind a hub can be
 *    exported at once. Each slot owns its descriptor cache, endpoint table and
 *    claim mask; the raw transfer layer addresses a slot purely by USB address.
 *  - The hub port registry mirrors the stack's live ports: it is what "describe"
 *    and "rescan" iterate, and it lets the disconnect hook reject ports the
 *    application never registered.
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
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "usbh_core.h"
#include "usb/usb_types_ch9.h"

#include "mklink_core.h"
#include "mklink_usb.h"
#include "mklink_priv.h"
#include "mklink_probe.h"

static const char *TAG = "mklink.usb";

#define MK_MAX_ENDPOINTS      16
#define MK_DESC_DUMP_LEN      512
/* MKLINK_USB_MAX_DEVICES lives in mklink_usb.h (the UI sizes its rows by it). */
#define MK_CFG_DESC_MAX       1024
/* One root port plus one component per downstream hub level. */
#define MKLINK_BUSID_MAX_DEPTH (CONFIG_USBHOST_MAX_EXTHUBS + 2)
/* DWC2 + dcache requires every DMA buffer to be cache line aligned. */
#define MK_DMA_ALIGN          CONFIG_USB_ALIGN_SIZE

typedef struct {
    bool     used;
    volatile bool alive;
    uint8_t  addr;
    uint8_t  speed;                  /* usbip speed code: 1=LOW 2=FULL 3=HIGH */
    char     busid[20];              /* stable id from the physical port path */
    struct usbh_hubport *hport;
    uint32_t claimed_mask;
    mklink_usb_snapshot_t snap;
    mklink_ep_info_t eps[MK_MAX_ENDPOINTS];
    int      ep_count;
    uint8_t  iface_cls[32];
    mklink_cdc_group_t cdc[MKLINK_MAX_CDC_GROUPS];
    uint8_t  cdc_count;
    uint8_t  dev_desc[18];           /* raw device descriptor */
    uint8_t *cfg_desc;
    size_t   cfg_len;
    char     desc_dump[MK_DESC_DUMP_LEN + 1];
} usb_slot_t;

typedef struct {
    bool used;
    uint8_t addr;
    struct usbh_hubport *hport;
} usb_port_t;

static usb_slot_t s_slots[MKLINK_USB_MAX_DEVICES];
static usb_port_t s_ports[MKLINK_USB_MAX_DEVICES];
static int s_pending_slot = -1;
static bool s_started;
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
    if (s->cfg_desc != NULL) {
        free(s->cfg_desc);
        s->cfg_desc = NULL;
    }
    s->cfg_len = 0;
    s->used = false;
    s->alive = false;
    s->addr = MKLINK_USB_ADDR_NONE;
    s->speed = 0;
    s->busid[0] = '\0';
    s->hport = NULL;
    s->claimed_mask = 0;
    s->ep_count = 0;
    s->cdc_count = 0;
    memset(s->cdc, 0, sizeof(s->cdc));
    memset(s->eps, 0, sizeof(s->eps));
    memset(s->iface_cls, 0xFF, sizeof(s->iface_cls));
    memset(s->dev_desc, 0, sizeof(s->dev_desc));
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
        if (s_slots[i].used && s_slots[i].alive) {
            return &s_slots[i];
        }
    }
    return NULL;
}

/*
 * Only published devices are enumerated.
 *
 * A slot is "used" from adopt() until the device goes away, but "alive" only
 * after confirm_adopted(): the window in between is an unpublished device that
 * is still being validated. Skipping it here keeps every index based API
 * (device_count / addr_at / busid_at / get_snapshot_at / usbip devlist) on the
 * same set, so the UI can never show "3 devices" above a 2 line list.
 */
static usb_slot_t *slot_nth_locked(int idx)
{
    int n = 0;
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (!s_slots[i].used || !s_slots[i].alive) {
            continue;
        }
        if (n++ == idx) {
            return &s_slots[i];
        }
    }
    return NULL;
}

/** Slot bound to a CherryUSB hub port (prefer the published one). */
static usb_slot_t *slot_by_hport_locked(struct usbh_hubport *hport)
{
    usb_slot_t *fallback = NULL;

    if (hport == NULL) {
        return NULL;
    }
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        usb_slot_t *s = &s_slots[i];
        if (!s->used || s->hport != hport) {
            continue;
        }
        if (s->alive) {
            return s;
        }
        if (fallback == NULL) {
            fallback = s;
        }
    }
    return fallback;
}

/* ------------------------------------------------------------- usbip busid */

/*
 * The usbip busid must be stable: the CherryUSB device address is allocated
 * dynamically and changes on every re-enumeration, so it cannot be used.
 * Instead build the physical port path the way Linux does ("1-1", "1-1.2"):
 * bus 1, then the root port, then one component per downstream hub port.
 *
 *   hport->parent        = hub the device sits on
 *   hub->parent          = hub port the hub itself sits on (NULL for the root hub)
 */
static int busid_build(struct usbh_hubport *hport, char *buf, size_t len)
{
    uint8_t ports[MKLINK_BUSID_MAX_DEPTH];
    int n = 0;

    if (buf == NULL || len == 0) {
        return 0;
    }
    buf[0] = '\0';
    if (hport == NULL) {
        return 0;
    }

    for (struct usbh_hubport *p = hport; p != NULL; p = (p->parent != NULL) ? p->parent->parent : NULL) {
        if (n >= MKLINK_BUSID_MAX_DEPTH) {
            ESP_LOGW(TAG, "port path deeper than %d, truncating busid", MKLINK_BUSID_MAX_DEPTH);
            break;
        }
        ports[n++] = p->port;
    }
    if (n == 0) {
        return 0;
    }

    int off = snprintf(buf, len, "1-%u", (unsigned)ports[n - 1]);
    for (int i = n - 2; i >= 0 && off > 0 && (size_t)off + 1 < len; i--) {
        off += snprintf(buf + off, len - (size_t)off, ".%u", (unsigned)ports[i]);
    }
    if (off < 0 || (size_t)off >= len) {
        buf[0] = '\0';
        return 0;
    }
    return off;
}

/* ------------------------------------------------------------- port table */

static usb_port_t *port_by_addr_locked(uint8_t addr)
{
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (s_ports[i].used && s_ports[i].addr == addr) {
            return &s_ports[i];
        }
    }
    return NULL;
}

static usb_port_t *port_by_hport_locked(struct usbh_hubport *hport)
{
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        if (s_ports[i].used && s_ports[i].hport == hport) {
            return &s_ports[i];
        }
    }
    return NULL;
}

/*
 * A slot that was adopted but never confirmed (the device vanished while its
 * descriptors were being validated) is unpublished yet still occupies a table
 * entry and, before slot_nth_locked() filtered on alive, shifted every index.
 * Reclaim those whose hub port is gone; the pending one belongs to the
 * discovery pass running right now and is left alone.
 */
static void slot_reap_stale_locked(void)
{
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        usb_slot_t *s = &s_slots[i];
        if (!s->used || s->alive || i == s_pending_slot) {
            continue;
        }
        if (s->hport != NULL && port_by_hport_locked(s->hport) != NULL) {
            continue;              /* port still registered: validation in flight */
        }
        ESP_LOGW(TAG, "reclaiming stale slot %d (addr %u)", i, (unsigned)s->addr);
        slot_clear_locked(s);
    }
}

static usb_port_t *port_register_locked(uint8_t addr, struct usbh_hubport *hport)
{
    usb_port_t *p = port_by_addr_locked(addr);
    if (p == NULL) {
        for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
            if (!s_ports[i].used) {
                p = &s_ports[i];
                break;
            }
        }
    }
    if (p == NULL) {
        return NULL;
    }
    p->used = true;
    p->addr = addr;
    p->hport = hport;
    return p;
}

static struct usbh_hubport *port_hport_for_addr(uint8_t addr)
{
    lock();
    usb_port_t *p = port_by_addr_locked(addr);
    struct usbh_hubport *hport = (p != NULL) ? p->hport : NULL;
    unlock();
    return hport;
}

static uint8_t speed_to_usbip(uint8_t cherry_speed)
{
    switch (cherry_speed) {
    case USB_SPEED_LOW:  return 1;
    case USB_SPEED_FULL: return 2;
    case USB_SPEED_HIGH: return 3;
    default:             return 0;
    }
}

static const char *speed_name(uint8_t usbip_speed)
{
    switch (usbip_speed) {
    case 1:  return "LS";
    case 2:  return "FS";
    case 3:  return "HS";
    default: return "??";
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

int mklink_usb_xfer_pending(void)
{
    return __atomic_load_n(&s_xfer_inflight, __ATOMIC_SEQ_CST);
}

bool mklink_usb_busy(void)
{
    lock();
    bool busy = (slot_free_locked() == NULL);
    unlock();
    return busy;
}

/* --------------------------------------------------- CherryUSB port hooks */

void usbh_device_connected_cb(struct usbh_hubport *hport)
{
    if (hport == NULL) {
        return;
    }

    /* Hubs are infrastructure: the CherryUSB hub class owns them, and the
     * devices behind them arrive as separate port events. */
    if (hport->self != NULL || hport->device_desc.bDeviceClass == 0x09) {
        return;
    }

    uint8_t addr = hport->dev_addr;

    lock();
    usb_port_t *p = port_register_locked(addr, hport);
    unlock();

    if (p == NULL) {
        mklink_log("USB: port table full, addr %u ignored", addr);
        return;
    }

    ESP_LOGD(TAG, "port %u connected (%s)", addr, speed_name(speed_to_usbip(hport->speed)));
    /* Discovery runs in its own task (descriptor fetch + REPL validation can
     * take seconds and must not stall the hub thread). */
    mklink_discovery_on_device(addr);
}

void usbh_device_disconnected_cb(struct usbh_hubport *hport)
{
    if (hport == NULL) {
        return;
    }
    if (hport->self != NULL || hport->device_desc.bDeviceClass == 0x09) {
        return;
    }

    uint8_t addr = hport->dev_addr;
    bool port_known = false;
    bool slot_hit = false;
    bool was_alive = false;
    int slot_idx = -1;
    mklink_usb_snapshot_t snap;

    memset(&snap, 0, sizeof(snap));
    snap.cdc_iface = 0xFF;

    lock();
    usb_port_t *p = port_by_hport_locked(hport);
    if (p != NULL) {
        port_known = true;
        p->used = false;
        p->hport = NULL;
        p->addr = MKLINK_USB_ADDR_NONE;
    }

    /* Find the table entry by hub port: the USB address may already belong to
     * another device (or the port was re-registered for a new device), so an
     * address-only lookup can miss and leave a stale slot behind. */
    usb_slot_t *s = slot_by_hport_locked(hport);
    if (s == NULL) {
        s = slot_by_addr_locked(addr);
    }
    if (s != NULL) {
        slot_hit = true;
        was_alive = s->alive;
        s->alive = false;          /* block new submissions immediately */
        snap = s->snap;
        slot_idx = (int)(s - s_slots);
    }
    unlock();

    if (!port_known && !slot_hit) {
        /* Enumeration failed before the application ever saw this port. */
        return;
    }

    /* Fail every pending URB first: their completions release the usbip
     * contexts while the device address is still meaningful. */
    mklink_usb_raw_kill_all(addr);

    if (!slot_hit) {
        return;
    }

    lock();
    slot_clear_locked(&s_slots[slot_idx]);
    if (s_pending_slot == slot_idx) {
        s_pending_slot = -1;       /* the validation target just disappeared */
    }
    unlock();

    if (!was_alive) {
        ESP_LOGD(TAG, "unpublished slot for addr %u reclaimed", addr);
        return;
    }

    mklink_log("USB: device addr=%u removed", addr);
    esp_event_post(MKLINK_EVENT, MKLINK_EVENT_USB_REMOVED, &snap, sizeof(snap), 0);
}

/* -------------------------------------------------------- descriptor read */

/** Fetch the raw configuration descriptor (device is live, config is 1). */
static esp_err_t read_config_descriptor(struct usbh_hubport *hport, uint8_t **out_buf, size_t *out_len)
{
    struct usb_setup_packet setup __attribute__((aligned(MK_DMA_ALIGN)));
    uint8_t header[16] __attribute__((aligned(MK_DMA_ALIGN)));
    uint8_t *buf = NULL;
    size_t total;
    int ret;

    memset(&setup, 0, sizeof(setup));
    setup.bmRequestType = USB_REQUEST_DIR_IN | USB_REQUEST_STANDARD | USB_REQUEST_RECIPIENT_DEVICE;
    setup.bRequest = USB_REQUEST_GET_DESCRIPTOR;
    setup.wValue = (uint16_t)(USB_DESCRIPTOR_TYPE_CONFIGURATION << 8);
    setup.wIndex = 0;
    setup.wLength = sizeof(header);

    ret = usbh_control_transfer(hport, &setup, header);
    if (ret < (int)sizeof(header)) {
        return ESP_FAIL;
    }

    total = (size_t)(header[2] | (header[3] << 8));
    if (total < 9 || total > MK_CFG_DESC_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* Round up: the DWC2 driver cleans/invalidates whole cache lines, and the
     * buffer must own every line it touches. */
    size_t alloc_len = (total + MK_DMA_ALIGN - 1) & ~(size_t)(MK_DMA_ALIGN - 1);
    buf = heap_caps_aligned_alloc(MK_DMA_ALIGN, alloc_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        buf = heap_caps_aligned_alloc(MK_DMA_ALIGN, alloc_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    setup.wLength = (uint16_t)total;
    ret = usbh_control_transfer(hport, &setup, buf);
    if (ret < 9) {
        free(buf);
        return ESP_FAIL;
    }

    *out_buf = buf;
    *out_len = (size_t)ret;
    return ESP_OK;
}

/** Read a UTF-16LE string descriptor and convert it to UTF-8. */
static void read_string(struct usbh_hubport *hport, uint8_t index, char *out, size_t out_len)
{
    struct usb_setup_packet setup __attribute__((aligned(MK_DMA_ALIGN)));
    uint8_t buf[256] __attribute__((aligned(MK_DMA_ALIGN)));
    int ret;
    int len;

    out[0] = '\0';
    if (index == 0) {
        return;
    }

    memset(&setup, 0, sizeof(setup));
    setup.bmRequestType = USB_REQUEST_DIR_IN | USB_REQUEST_STANDARD | USB_REQUEST_RECIPIENT_DEVICE;
    setup.bRequest = USB_REQUEST_GET_DESCRIPTOR;
    setup.wValue = (uint16_t)((USB_DESCRIPTOR_TYPE_STRING << 8) | index);
    setup.wIndex = 0x0409;   /* en-US */
    setup.wLength = sizeof(buf);

    ret = usbh_control_transfer(hport, &setup, buf);
    if (ret < 2) {
        return;
    }

    len = buf[0];
    if (len > ret) {
        len = ret;
    }
    if (len < 2) {
        return;
    }
    mklink_utf16_to_utf8((const uint16_t *)(const void *)(buf + 2), (len - 2) / 2, out, out_len);
}

esp_err_t mklink_usb_describe(uint8_t addr, mklink_usb_desc_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->addr = addr;
    memset(out->iface_cls, 0xFF, sizeof(out->iface_cls));

    struct usbh_hubport *hport = port_hport_for_addr(addr);
    if (hport == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    out->hport = hport;
    memcpy(out->dev_desc, &hport->device_desc, sizeof(out->dev_desc));
    out->vid = hport->device_desc.idVendor;
    out->pid = hport->device_desc.idProduct;
    out->dev_class = hport->device_desc.bDeviceClass;
    out->speed = speed_to_usbip(hport->speed);
    (void)busid_build(hport, out->busid, sizeof(out->busid));

    const struct usb_configuration_descriptor *cd = &hport->config.config_desc;
    out->iface_cnt = cd->bNumInterfaces;

    /* Walk the parsed configuration: CherryUSB already split it into
     * interfaces / alt settings / endpoints, so no raw descriptor parsing. */
    uint8_t iface_order[32];
    int iface_order_n = 0;

    for (uint8_t i = 0; i < cd->bNumInterfaces && i < CONFIG_USBHOST_MAX_INTERFACES; i++) {
        struct usbh_interface *intf = &hport->config.intf[i];
        if (intf->altsetting_num == 0) {
            continue;
        }
        struct usb_interface_descriptor *id = &intf->altsetting[0].intf_desc;
        uint8_t ifn = id->bInterfaceNumber;
        if (ifn < 32) {
            out->iface_cls[ifn] = id->bInterfaceClass;
            if (iface_order_n < (int)sizeof(iface_order)) {
                iface_order[iface_order_n++] = ifn;
            }
        }

        for (uint8_t e = 0; e < id->bNumEndpoints && out->ep_count < MK_MAX_ENDPOINTS; e++) {
            struct usb_endpoint_descriptor *ed = &intf->altsetting[0].ep[e].ep_desc;
            mklink_ep_info_t *ep = &out->eps[out->ep_count];
            ep->ep_addr = ed->bEndpointAddress;
            ep->attributes = ed->bmAttributes & 0x03;
            ep->mps = (uint16_t)(ed->wMaxPacketSize & 0x7FF);
            ep->interval = ed->bInterval;
            ep->iface_num = ifn;
            out->ep_count++;
        }
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

    read_string(hport, hport->device_desc.iManufacturer, out->manufacturer, sizeof(out->manufacturer));
    read_string(hport, hport->device_desc.iProduct, out->product, sizeof(out->product));
    if (out->product[0] == '\0') {
        strlcpy(out->product, "USB device", sizeof(out->product));
    }

    /* The usbip server streams the configuration descriptor to the client, so
     * keep the raw bytes. */
    if (read_config_descriptor(hport, &out->cfg_desc, &out->cfg_len) != ESP_OK) {
        ESP_LOGW(TAG, "addr %u: config descriptor read failed", addr);
    }

    /* Topology summary, one interface per line (the UI log is 96 chars wide). */
    char *dump = out->topology;
    size_t cap = sizeof(out->topology);
    dump[0] = '\0';
    append(dump, cap, "%04X:%04X %s class %02X/%02X/%02X | cfg: %u iface / %u ep / %u cdc",
           out->vid, out->pid, speed_name(out->speed),
           hport->device_desc.bDeviceClass, hport->device_desc.bDeviceSubClass,
           hport->device_desc.bDeviceProtocol, out->iface_cnt, out->ep_count, out->cdc_count);
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
    if (desc == NULL) {
        return;
    }
    if (desc->cfg_desc != NULL) {
        free(desc->cfg_desc);
        desc->cfg_desc = NULL;
    }
    desc->cfg_len = 0;
    desc->hport = NULL;
}

static uint8_t cdc_group_cmd_iface(const mklink_cdc_group_t *g)
{
    return (g->comm_iface != 0xFF) ? g->comm_iface : g->data_iface;
}

esp_err_t mklink_usb_adopt(mklink_usb_desc_t *desc)
{
    if (desc == NULL || desc->hport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    usb_port_t *p = port_by_addr_locked(desc->addr);
    if (p == NULL || p->hport != desc->hport) {
        unlock();
        return ESP_ERR_NOT_FOUND;      /* unplugged while validating */
    }

    usb_slot_t *s = slot_free_locked();
    if (s == NULL) {
        slot_reap_stale_locked();
        s = slot_free_locked();
    }
    if (s == NULL) {
        unlock();
        return ESP_ERR_NO_MEM;
    }

    s->used = true;
    s->alive = false;              /* published by confirm_adopted() */
    s->addr = desc->addr;
    s->speed = desc->speed;
    strlcpy(s->busid, desc->busid, sizeof(s->busid));
    s->hport = desc->hport;
    s->ep_count = desc->ep_count;
    s->cdc_count = desc->cdc_count;
    memcpy(s->eps, desc->eps, sizeof(mklink_ep_info_t) * desc->ep_count);
    memcpy(s->iface_cls, desc->iface_cls, sizeof(s->iface_cls));
    memcpy(s->cdc, desc->cdc, sizeof(mklink_cdc_group_t) * desc->cdc_count);
    s->claimed_mask = 0;
    memcpy(s->dev_desc, desc->dev_desc, sizeof(s->dev_desc));
    s->cfg_desc = desc->cfg_desc;      /* ownership moved to the slot */
    s->cfg_len = desc->cfg_len;
    desc->cfg_desc = NULL;
    desc->cfg_len = 0;

    memset(&s->snap, 0, sizeof(s->snap));
    s->snap.connected = true;
    s->snap.vid = desc->vid;
    s->snap.pid = desc->pid;
    s->snap.dev_addr = desc->addr;
    s->snap.iface_cnt = desc->iface_cnt;
    s->snap.ep_cnt = desc->ep_count;
    s->snap.cdc_iface = desc->cdc_count ? cdc_group_cmd_iface(&desc->cdc[0]) : 0xFF;
    strlcpy(s->snap.busid, desc->busid, sizeof(s->snap.busid));
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
        s_pending_slot = -1;       /* drop the dangling index */
        unlock();
        return;
    }
    s->alive = true;
    snap = s->snap;
    strlcpy(dump_copy, s->desc_dump, sizeof(dump_copy));
    unlock();

    mklink_log("USB: %04X:%04X \"%s\" %s addr=%u if=%u ep=%u",
               snap.vid, snap.pid, snap.product, snap.busid, snap.dev_addr, snap.iface_cnt, snap.ep_cnt);
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
    lock();
    if (s_pending_slot >= 0) {
        slot_clear_locked(&s_slots[s_pending_slot]);
        s_pending_slot = -1;
    }
    unlock();
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
    if (addrs == NULL || max == 0) {
        return 0;
    }
    int count = 0;
    lock();
    for (int i = 0; i < MKLINK_USB_MAX_DEVICES && (size_t)count < max; i++) {
        if (s_ports[i].used && s_ports[i].addr != 0) {
            addrs[count++] = s_ports[i].addr;
        }
    }
    unlock();
    return count;
}

esp_err_t mklink_usb_start(void)
{
    lock_init();
    lock();
    if (s_started) {
        unlock();
        return ESP_OK;
    }
    s_started = true;
    unlock();

    for (int i = 0; i < MKLINK_USB_MAX_DEVICES; i++) {
        slot_clear_locked(&s_slots[i]);
        s_slots[i].addr = MKLINK_USB_ADDR_NONE;
        s_ports[i].used = false;
        s_ports[i].addr = MKLINK_USB_ADDR_NONE;
        s_ports[i].hport = NULL;
    }

    /* Completion/control workers must exist before the hub thread can report a
     * device (the hooks below call into discovery and the raw layer). */
    if (mklink_usb_raw_init() != ESP_OK) {
        mklink_log("USB: transfer worker init failed");
        return ESP_ERR_NO_MEM;
    }

    /* bus 0 = the high speed OTG instance (UTMI PHY), the same port the board
     * wires to the Type-A socket. */
    int err = usbh_initialize(0, ESP_USB_HS0_BASE);
    if (err != 0) {
        mklink_log("USB: usbh_initialize failed (%d)", err);
        return ESP_FAIL;
    }

    mklink_log("USB: CherryUSB host stack ready (filter %04X:%04X)",
               CONFIG_MKLINK_USB_VID, CONFIG_MKLINK_USB_PID);
    mklink_log("USB: internal heap %u B (largest %u B), bus table %u B",
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
               (unsigned)sizeof(struct usbh_bus));

    /* Devices that finished enumeration before the stack was published to the
     * UI would be missed: replay the live ports. */
    return mklink_usb_rescan();
}

esp_err_t mklink_usb_rescan(void)
{
    uint8_t addrs[MKLINK_USB_MAX_DEVICES] = {0};
    int count = mklink_usb_addr_list(addrs, MKLINK_USB_MAX_DEVICES);

    for (int i = 0; i < count; i++) {
        if (addrs[i] == 0 || addrs[i] == MKLINK_USB_ADDR_NONE) {
            continue;
        }
        mklink_discovery_on_device(addrs[i]);
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

int mklink_usb_busid_at(int idx, char *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return 0;
    }
    lock();
    usb_slot_t *s = slot_nth_locked(idx);
    bool ok = (s != NULL && s->busid[0] != '\0');
    if (ok) {
        strlcpy(buf, s->busid, len);
    }
    unlock();
    if (!ok) {
        buf[0] = '\0';
        return 0;
    }
    return (int)strlen(buf);
}

int mklink_usb_busid_for_addr(uint8_t dev_addr, char *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return 0;
    }
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool ok = (s != NULL && s->busid[0] != '\0');
    if (ok) {
        strlcpy(buf, s->busid, len);
    }
    unlock();
    if (!ok) {
        buf[0] = '\0';
        return 0;
    }
    return (int)strlen(buf);
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

uint8_t mklink_usb_current_addr(void)
{
    lock();
    usb_slot_t *s = slot_first_locked();
    uint8_t addr = (s != NULL) ? s->addr : MKLINK_USB_ADDR_NONE;
    unlock();
    return addr;
}

bool mklink_usb_handle_for_addr(uint8_t dev_addr, struct usbh_hubport **out)
{
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool ok = (s != NULL && s->hport != NULL);
    if (ok && out != NULL) {
        *out = s->hport;
    }
    unlock();
    return ok;
}

esp_err_t mklink_usb_claim_interface(uint8_t dev_addr, uint8_t iface_num)
{
    if (iface_num >= 32) {
        return ESP_ERR_INVALID_ARG;
    }

    /* CherryUSB has no interface claim concept for raw URB users: a device that
     * has no class driver bound is already exclusively ours. Keep the mask so
     * the probe bridge / usbip can still tell who owns which interface. */
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    if (s == NULL) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s->claimed_mask |= (1u << iface_num);
    unlock();
    return ESP_OK;
}

esp_err_t mklink_usb_release_interface(uint8_t dev_addr, uint8_t iface_num)
{
    if (iface_num >= 32) {
        return ESP_ERR_INVALID_ARG;
    }

    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    if (s == NULL) {
        unlock();
        return ESP_ERR_INVALID_ARG;
    }
    s->claimed_mask &= ~(1u << iface_num);
    unlock();
    return ESP_OK;
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

esp_err_t mklink_usb_device_desc(uint8_t dev_addr, usb_device_desc_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool ok = (s != NULL);
    if (ok) {
        memcpy(out, s->dev_desc, sizeof(s->dev_desc));
    }
    unlock();
    return ok ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t mklink_usb_config_desc(uint8_t dev_addr, uint8_t *buf, size_t cap, size_t *out_len)
{
    if (buf == NULL || cap == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    if (s == NULL || s->cfg_desc == NULL) {
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    size_t len = s->cfg_len < cap ? s->cfg_len : cap;
    memcpy(buf, s->cfg_desc, len);
    unlock();
    if (out_len != NULL) {
        *out_len = len;
    }
    return ESP_OK;
}

esp_err_t mklink_usb_device_speed(uint8_t dev_addr, uint8_t *speed)
{
    if (speed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    usb_slot_t *s = slot_by_addr_locked(dev_addr);
    bool ok = (s != NULL);
    if (ok) {
        *speed = s->speed;
    }
    unlock();
    return ok ? ESP_OK : ESP_ERR_NOT_FOUND;
}
