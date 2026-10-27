/*
 * MKLink Remote - usbip server (stage 2 data plane).
 *
 * Implements the server side of the standard Linux USB/IP protocol so a remote
 * computer can import the attached probe as a native USB device:
 *
 *   OP_REQ_DEVLIST -> OP_REP_DEVLIST
 *   OP_REQ_IMPORT  -> OP_REP_IMPORT (device + config descriptors)
 *   CMD_SUBMIT / CMD_UNLINK -> RET_SUBMIT / RET_UNLINK
 *
 * Wire format and byte order were taken from the Linux kernel reference
 * (drivers/usb/usbip/usbip_common.h + tools/usb/usbip/libsrc/usbip_network.c):
 *
 *   - every data-plane PDU is a 48-byte struct usbip_header (base + union),
 *     all numeric fields in network byte order, setup[8] raw;
 *   - control-plane messages are struct op_common {u16 version,u16 code,u32
 *     status} (network order) followed by the op-specific payload.
 *
 * The actual USB transfers go through mklink_usb_raw_*; a small per-connection
 * task multiplexes incoming CMD_SUBMIT/CMD_UNLINK and completed transfers so
 * that an UNLINK can be serviced while a bulk IN URB is still pending.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "sys/select.h"
#include "sys/time.h"

#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

#include "mklink_core.h"
#include "mklink_usb.h"
#include "mklink_usb_raw.h"
#include "mklink_usbip.h"
#include "mklink_probe.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.usbip";

/* ------------------------------------------------------------------ const -- */

#define USBIP_PORT_DEFAULT      3240
#define USBIP_VERSION           0x0111
#define USBIP_BUS_ID_SIZE       32
#define USBIP_DEV_PATH_SIZE     256

#define USBIP_CMD_SUBMIT        0x0001
#define USBIP_CMD_UNLINK        0x0002
#define USBIP_RET_SUBMIT        0x0003
#define USBIP_RET_UNLINK        0x0004

#define USBIP_DIR_OUT           0x00
#define USBIP_DIR_IN            0x01

#define OP_REQUEST              (0x80 << 8)
#define OP_REPLY                (0x00 << 8)
#define OP_DEVLIST              0x05
#define OP_REQ_DEVLIST          (OP_REQUEST | OP_DEVLIST)
#define OP_REP_DEVLIST          (OP_REPLY   | OP_DEVLIST)
#define OP_IMPORT               0x03
#define OP_REQ_IMPORT           (OP_REQUEST | OP_IMPORT)
#define OP_REP_IMPORT           (OP_REPLY   | OP_IMPORT)

#define ST_OK                   0x00
#define ST_NODEV                0x04
#define ST_ERROR                0x05

/* USB/IP speed codes (libsrc/usbip_common.c). */
#define USBIP_SPEED_UNKNOWN     0
#define USBIP_SPEED_LOW         1
#define USBIP_SPEED_FULL        2
#define USBIP_SPEED_HIGH        3

/* Timeouts for the underlying esp-idf transfers (ms). 0 would mean "never" on
 * some host stack versions, so we use a large finite value for IN endpoints:
 * a CMSIS-DAP bulk IN only completes when the target answers. */
#define CTRL_TIMEOUT_MS         3000
#define OUT_TIMEOUT_MS          5000
#define IN_TIMEOUT_MS           60000

#define MAX_PENDING             16

/* ------------------------------------------------------------------ wire --- */

typedef struct {
    uint16_t version;
    uint16_t code;
    uint32_t status;
} __attribute__((packed)) op_common_t;

typedef struct {
    char     path[USBIP_DEV_PATH_SIZE];
    char     busid[USBIP_BUS_ID_SIZE];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bConfigurationValue;
    uint8_t  bNumConfigurations;
    uint8_t  bNumInterfaces;
} __attribute__((packed)) usbip_usb_device_t;

typedef struct {
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t padding;
} __attribute__((packed)) usbip_usb_interface_t;

typedef struct {
    uint32_t command;
    uint32_t seqnum;
    uint32_t devid;
    uint32_t direction;
    uint32_t ep;
} __attribute__((packed)) usbip_header_basic_t;

typedef struct {
    uint32_t transfer_flags;
    int32_t  transfer_buffer_length;
    int32_t  start_frame;
    int32_t  number_of_packets;
    int32_t  interval;
    uint8_t  setup[8];
} __attribute__((packed)) usbip_header_cmd_submit_t;

typedef struct {
    int32_t status;
    int32_t actual_length;
    int32_t start_frame;
    int32_t number_of_packets;
    int32_t error_count;
} __attribute__((packed)) usbip_header_ret_submit_t;

typedef struct {
    uint32_t seqnum;
} __attribute__((packed)) usbip_header_cmd_unlink_t;

typedef struct {
    int32_t status;
} __attribute__((packed)) usbip_header_ret_unlink_t;

typedef struct {
    usbip_header_basic_t base;
    union {
        usbip_header_cmd_submit_t  cmd_submit;
        usbip_header_ret_submit_t  ret_submit;
        usbip_header_cmd_unlink_t  cmd_unlink;
        usbip_header_ret_unlink_t  ret_unlink;
    } u;
} __attribute__((packed)) usbip_header_t;

_Static_assert(sizeof(usbip_header_t) == 48, "usbip_header must be 48 bytes");
_Static_assert(sizeof(usbip_usb_device_t) == 312, "usbip_usb_device must be 312 bytes");
_Static_assert(sizeof(op_common_t) == 8, "op_common must be 8 bytes");

static inline uint32_t be32(uint32_t v) { return __builtin_bswap32(v); }
static inline uint16_t be16(uint16_t v) { return __builtin_bswap16(v); }

/* --------------------------------------------------------------- transfer -- */

typedef struct {
    bool     active;
    uint32_t seqnum;
    uint8_t  ep_addr;      /* actual endpoint address (bit7 = IN) */
    bool     in;           /* IN data direction */
    uint8_t *buf;          /* IN: holds received data until consumed */
    size_t   len;
    volatile bool     done;
    volatile esp_err_t status;
    volatile size_t   actual;
    portMUX_TYPE mux;
} pending_xfer_t;

static void xfer_cb(void *arg, esp_err_t status, size_t actual)
{
    pending_xfer_t *x = (pending_xfer_t *)arg;
    portENTER_CRITICAL(&x->mux);
    x->status = status;
    x->actual = actual;
    x->done = true;
    portEXIT_CRITICAL(&x->mux);
}

static int32_t esp_to_errno(esp_err_t e)
{
    switch (e) {
    case ESP_OK:              return 0;
    case ESP_ERR_TIMEOUT:     return -ETIMEDOUT;
    case ESP_ERR_INVALID_STATE: return -ECONNRESET;  /* unlink / device gone */
    case ESP_ERR_NOT_FOUND:   return -ENODEV;
    case ESP_ERR_NO_MEM:      return -ENOMEM;
    case ESP_ERR_INVALID_ARG: return -EINVAL;
    case ESP_ERR_INVALID_SIZE:return -EINVAL;
    default:                  return -EIO;
    }
}

/* ------------------------------------------------------------------ io ----- */

static int send_all(int s, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t sent = 0;
    while (sent < len) {
        int n = send(s, p + sent, len - sent, 0);
        if (n <= 0) {
            return -1;
        }
        sent += (size_t)n;
    }
    return 0;
}

static int recv_all(int s, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;
    size_t got = 0;
    while (got < len) {
        int n = recv(s, p + got, len - got, 0);
        if (n <= 0) {
            return -1;
        }
        got += (size_t)n;
    }
    return 0;
}

static int send_op_common(int s, uint16_t code, uint32_t status)
{
    op_common_t common;
    memset(&common, 0, sizeof(common));
    common.version = be16(USBIP_VERSION);
    common.code    = be16(code);
    common.status  = be32(status);
    return send_all(s, &common, sizeof(common));
}

/* ------------------------------------------------------- device snapshot -- */

static uint8_t esp_speed_to_usbip(usb_speed_t speed)
{
    switch (speed) {
    case USB_SPEED_LOW:  return USBIP_SPEED_LOW;
    case USB_SPEED_FULL: return USBIP_SPEED_FULL;
    case USB_SPEED_HIGH: return USBIP_SPEED_HIGH;
    default:             return USBIP_SPEED_UNKNOWN;
    }
}

/**
 * Fill @p udev and copy the raw device (18 bytes) and config (wTotalLength)
 * descriptors of the active device. Returns 0 on success.
 */
static int collect_device(usbip_usb_device_t *udev,
                          uint8_t *dev_desc, size_t dev_desc_cap,
                          uint8_t *conf_desc, size_t conf_desc_cap,
                          size_t *conf_len_out,
                          usbip_usb_interface_t *uinf, int uinf_max, int *uinf_count)
{
    uint8_t addr = mklink_usb_current_addr();
    if (addr == MKLINK_USB_ADDR_NONE) {
        return -1;
    }

    usb_device_handle_t hdl = NULL;
    if (!mklink_usb_handle_for_addr(addr, &hdl)) {
        return -1;
    }

    const usb_device_desc_t *dd = NULL;
    const usb_config_desc_t *cd = NULL;
    if (usb_host_get_device_descriptor(hdl, &dd) != ESP_OK || dd == NULL) {
        return -1;
    }
    if (usb_host_get_active_config_descriptor(hdl, &cd) != ESP_OK || cd == NULL) {
        return -1;
    }

    usb_device_info_t info;
    memset(&info, 0, sizeof(info));
    usb_host_device_info(hdl, &info);

    memset(udev, 0, sizeof(*udev));
    snprintf(udev->path, sizeof(udev->path),
             "/sys/devices/platform/mklink/1-1");
    strlcpy(udev->busid, "1-1", sizeof(udev->busid));
    udev->busnum = 1;
    udev->devnum = 1;
    udev->speed  = esp_speed_to_usbip(info.speed);
    udev->idVendor  = dd->idVendor;
    udev->idProduct = dd->idProduct;
    udev->bcdDevice = dd->bcdDevice;
    udev->bDeviceClass    = dd->bDeviceClass;
    udev->bDeviceSubClass = dd->bDeviceSubClass;
    udev->bDeviceProtocol = dd->bDeviceProtocol;
    udev->bConfigurationValue = cd->bConfigurationValue;
    udev->bNumConfigurations  = dd->bNumConfigurations;
    udev->bNumInterfaces      = cd->bNumInterfaces;

    if (dev_desc != NULL && dev_desc_cap >= 18) {
        memcpy(dev_desc, dd, 18);
    }

    /* Walk the config descriptor to collect interface class/subclass/protocol. */
    int uinf_n = 0;
    const uint8_t *p = (const uint8_t *)cd;
    const uint8_t *end = p + cd->wTotalLength;
    while (p + 2 <= end) {
        uint8_t dlen = p[0];
        uint8_t dtype = p[1];
        if (dlen < 2) {
            break;
        }
        if (dtype == USB_B_DESCRIPTOR_TYPE_INTERFACE && dlen >= 9 && uinf_n < uinf_max) {
            uinf[uinf_n].bInterfaceClass    = p[5];
            uinf[uinf_n].bInterfaceSubClass = p[6];
            uinf[uinf_n].bInterfaceProtocol = p[7];
            uinf[uinf_n].padding = 0;
            uinf_n++;
        }
        p += dlen;
    }
    if (uinf_count != NULL) {
        *uinf_count = uinf_n;
    }

    if (conf_desc != NULL && conf_desc_cap > 0) {
        size_t len = cd->wTotalLength;
        if (len > conf_desc_cap) {
            len = conf_desc_cap;
        }
        memcpy(conf_desc, cd, len);
        if (conf_len_out != NULL) {
            *conf_len_out = len;
        }
    }

    return 0;
}

/* ----------------------------------------------------------------- op ------ */

static void pack_usb_device(usbip_usb_device_t *udev)
{
    udev->busnum   = be32(udev->busnum);
    udev->devnum   = be32(udev->devnum);
    udev->speed    = be32(udev->speed);
    udev->idVendor = be16(udev->idVendor);
    udev->idProduct = be16(udev->idProduct);
    udev->bcdDevice = be16(udev->bcdDevice);
}

static int send_devlist_reply(int s)
{
    usbip_usb_device_t udev;
    usbip_usb_interface_t uinf[16];
    uint8_t dev_desc[18];
    int uinf_count = 0;

    int have = collect_device(&udev, dev_desc, sizeof(dev_desc),
                              NULL, 0, NULL, uinf, 16, &uinf_count);
    if (have != 0) {
        if (send_op_common(s, OP_REP_DEVLIST, ST_OK) != 0) {
            return -1;
        }
        uint32_t ndev = be32(0);
        return send_all(s, &ndev, sizeof(ndev));
    }

    if (send_op_common(s, OP_REP_DEVLIST, ST_OK) != 0) {
        return -1;
    }

    uint32_t ndev = be32(1);
    if (send_all(s, &ndev, sizeof(ndev)) != 0) {
        return -1;
    }

    pack_usb_device(&udev);
    if (send_all(s, &udev, sizeof(udev)) != 0) {
        return -1;
    }
    if (uinf_count > 0) {
        if (send_all(s, uinf, sizeof(usbip_usb_interface_t) * (size_t)uinf_count) != 0) {
            return -1;
        }
    }
    return 0;
}

static int send_import_reply(int s)
{
    usbip_usb_device_t udev;
    usbip_usb_interface_t uinf[16];
    int uinf_count = 0;

    if (collect_device(&udev, NULL, 0, NULL, 0, NULL, uinf, 16, &uinf_count) != 0) {
        return send_op_common(s, OP_REP_IMPORT, ST_NODEV);
    }

    /* Per usbipd's recv_request_import the import reply is ONLY op_common +
     * usbip_usb_device. The device/config descriptors are NOT sent here; the
     * client fetches them afterwards with GET_DESCRIPTOR URBs. Sending extra
     * bytes corrupts the following data plane. */
    if (send_op_common(s, OP_REP_IMPORT, ST_OK) != 0) {
        return -1;
    }
    pack_usb_device(&udev);
    if (send_all(s, &udev, sizeof(udev)) != 0) {
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------- data-plane pdu ---- */

static int send_ret_submit(int s, pending_xfer_t *x)
{
    usbip_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    int32_t status = esp_to_errno((esp_err_t)x->status);
    size_t actual = x->in ? x->actual : 0;

    hdr.base.command   = be32(USBIP_RET_SUBMIT);
    hdr.base.seqnum    = be32(x->seqnum);
    hdr.base.devid     = be32(0x00010001);
    hdr.base.direction = be32(x->in ? USBIP_DIR_IN : USBIP_DIR_OUT);
    hdr.base.ep        = be32(x->ep_addr & 0x0F);
    hdr.u.ret_submit.status        = be32((uint32_t)status);
    hdr.u.ret_submit.actual_length = be32((uint32_t)(int32_t)actual);
    hdr.u.ret_submit.start_frame   = 0;
    hdr.u.ret_submit.number_of_packets = 0;
    hdr.u.ret_submit.error_count   = 0;

    if (send_all(s, &hdr, sizeof(hdr)) != 0) {
        return -1;
    }
    if (x->in && actual > 0 && x->buf != NULL) {
        if (send_all(s, x->buf, (size_t)actual) != 0) {
            return -1;
        }
    }
    return 0;
}

static int send_ret_unlink(int s, uint32_t seqnum, int32_t status)
{
    usbip_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.base.command = be32(USBIP_RET_UNLINK);
    hdr.base.seqnum  = be32(seqnum);
    hdr.base.devid   = be32(0x00010001);
    hdr.base.direction = be32(USBIP_DIR_OUT);
    hdr.base.ep      = 0;
    hdr.u.ret_unlink.status = be32((uint32_t)status);
    return send_all(s, &hdr, sizeof(hdr));
}

/* ---------------------------------------------------------- client state --- */

typedef struct {
    pending_xfer_t xfer[MAX_PENDING];
    int count;
} client_state_t;

static pending_xfer_t *xfer_alloc(client_state_t *cs, uint32_t seqnum,
                                  uint8_t ep_addr, bool in, size_t len)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        if (!cs->xfer[i].active) {
            pending_xfer_t *x = &cs->xfer[i];
            memset(x, 0, sizeof(*x));
            x->active = true;
            x->seqnum = seqnum;
            x->ep_addr = ep_addr;
            x->in = in;
            x->len = len;
            x->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
            if (len > 0) {
                x->buf = malloc(len);
                if (x->buf == NULL) {
                    memset(x, 0, sizeof(*x));
                    return NULL;
                }
            }
            return x;
        }
    }
    return NULL;
}

static void xfer_release(pending_xfer_t *x)
{
    if (x->buf != NULL) {
        free(x->buf);
    }
    memset(x, 0, sizeof(*x));
}

static pending_xfer_t *xfer_find(client_state_t *cs, uint32_t seqnum)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        if (cs->xfer[i].active && cs->xfer[i].seqnum == seqnum && !cs->xfer[i].done) {
            return &cs->xfer[i];
        }
    }
    return NULL;
}

static bool xfer_done(pending_xfer_t *x, esp_err_t *status, size_t *actual)
{
    bool done;
    portENTER_CRITICAL(&x->mux);
    done = x->done;
    if (done) {
        *status = (esp_err_t)x->status;
        *actual = x->actual;
    }
    portEXIT_CRITICAL(&x->mux);
    return done;
}

/* Drain completed transfers: send RET_SUBMIT and release the slot. */
static int drain_completions(int s, client_state_t *cs)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        pending_xfer_t *x = &cs->xfer[i];
        if (!x->active) {
            continue;
        }
        esp_err_t status;
        size_t actual;
        if (!xfer_done(x, &status, &actual)) {
            continue;
        }
        if (send_ret_submit(s, x) != 0) {
            return -1;
        }
        xfer_release(x);
    }
    return 0;
}

static int handle_submit(int s, client_state_t *cs, usbip_header_t *hdr)
{
    usbip_header_basic_t *b = &hdr->base;
    usbip_header_cmd_submit_t *sub = &hdr->u.cmd_submit;

    sub->transfer_flags        = be32(sub->transfer_flags);
    sub->transfer_buffer_length = (int32_t)be32((uint32_t)sub->transfer_buffer_length);
    sub->start_frame           = (int32_t)be32((uint32_t)sub->start_frame);
    sub->number_of_packets     = (int32_t)be32((uint32_t)sub->number_of_packets);
    sub->interval              = (int32_t)be32((uint32_t)sub->interval);

    uint32_t seqnum  = b->seqnum;
    bool     dir_in  = (b->direction == USBIP_DIR_IN);
    uint8_t  ep_num  = (uint8_t)(b->ep & 0x0F);
    bool     control = (ep_num == 0);
    int32_t  len     = sub->transfer_buffer_length;

    uint8_t addr = mklink_usb_current_addr();
    if (addr == MKLINK_USB_ADDR_NONE) {
        /* No device: fail the URB immediately. */
        pending_xfer_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        tmp.seqnum = seqnum;
        tmp.ep_addr = 0;
        tmp.in = dir_in;
        tmp.status = ESP_ERR_NOT_FOUND;
        tmp.actual = 0;
        send_ret_submit(s, &tmp);
        return 0;
    }

    uint8_t ep_addr = control ? 0x00 : (uint8_t)(ep_num | (dir_in ? 0x80 : 0x00));

    if (len < 0) {
        len = 0;
    }

    if (control) {
        /* Control transfer: setup in sub->setup[8]. Direction from bmRequestType. */
        usb_setup_packet_t setup;
        memcpy(&setup, sub->setup, 8);
        bool in = (setup.bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0;

        pending_xfer_t *x = xfer_alloc(cs, seqnum, 0x00, in, (size_t)len);
        if (x == NULL) {
            pending_xfer_t tmp;
            memset(&tmp, 0, sizeof(tmp));
            tmp.seqnum = seqnum;
            tmp.in = in;
            tmp.status = ESP_ERR_NO_MEM;
            send_ret_submit(s, &tmp);
            return 0;
        }

        if (!in && len > 0) {
            /* Data stage OUT: payload follows the header. */
            if (recv_all(s, x->buf, (size_t)len) != 0) {
                xfer_release(x);
                return -1;
            }
        }

        esp_err_t err = mklink_usb_raw_control(addr, &setup, x->buf, (size_t)len,
                                               CTRL_TIMEOUT_MS, xfer_cb, x);
        if (err != ESP_OK) {
            xfer_release(x);
            pending_xfer_t tmp;
            memset(&tmp, 0, sizeof(tmp));
            tmp.seqnum = seqnum;
            tmp.in = in;
            tmp.status = err;
            send_ret_submit(s, &tmp);
            return 0;
        }
        return 0;
    }

    /* Bulk / interrupt. */
    pending_xfer_t *x = xfer_alloc(cs, seqnum, ep_addr, dir_in, (size_t)len);
    if (x == NULL) {
        pending_xfer_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        tmp.seqnum = seqnum;
        tmp.in = dir_in;
        tmp.ep_addr = ep_addr;
        tmp.status = ESP_ERR_NO_MEM;
        send_ret_submit(s, &tmp);
        return 0;
    }

    if (!dir_in && len > 0) {
        if (recv_all(s, x->buf, (size_t)len) != 0) {
            xfer_release(x);
            return -1;
        }
    }

    uint32_t timeout = dir_in ? IN_TIMEOUT_MS : OUT_TIMEOUT_MS;
    esp_err_t err = mklink_usb_raw_submit(addr, ep_addr, x->buf, (size_t)len,
                                          timeout, xfer_cb, x);
    if (err != ESP_OK) {
        xfer_release(x);
        pending_xfer_t tmp;
        memset(&tmp, 0, sizeof(tmp));
        tmp.seqnum = seqnum;
        tmp.in = dir_in;
        tmp.ep_addr = ep_addr;
        tmp.status = err;
        send_ret_submit(s, &tmp);
        return 0;
    }
    return 0;
}

static int handle_unlink(int s, client_state_t *cs, usbip_header_t *hdr)
{
    uint32_t seqnum = hdr->u.cmd_unlink.seqnum;
    pending_xfer_t *x = xfer_find(cs, seqnum);
    if (x != NULL) {
        uint8_t addr = mklink_usb_current_addr();
        if (addr != MKLINK_USB_ADDR_NONE) {
            mklink_usb_raw_abort(addr, x->ep_addr);
        }
    }
    return send_ret_unlink(s, seqnum, 0);
}

static int handle_one_pdu(int s, client_state_t *cs)
{
    usbip_header_t hdr;
    if (recv_all(s, &hdr, sizeof(hdr)) != 0) {
        return -1;
    }

    hdr.base.command   = be32(hdr.base.command);
    hdr.base.seqnum    = be32(hdr.base.seqnum);
    hdr.base.devid     = be32(hdr.base.devid);
    hdr.base.direction = be32(hdr.base.direction);
    hdr.base.ep        = be32(hdr.base.ep);

    switch (hdr.base.command) {
    case USBIP_CMD_SUBMIT:
        return handle_submit(s, cs, &hdr);
    case USBIP_CMD_UNLINK:
        hdr.u.cmd_unlink.seqnum = be32(hdr.u.cmd_unlink.seqnum);
        return handle_unlink(s, cs, &hdr);
    default:
        return -1;
    }
}

/* ----------------------------------------------------------------- client -- */

static void client_task(void *arg)
{
    int s = (int)(intptr_t)arg;
    client_state_t cs;
    memset(&cs, 0, sizeof(cs));
    bool imported = false;

    /* The kernel usbipd sets TCP_NODELAY: a 48-byte header + payload would
     * otherwise be delayed by Nagle until the previous segment is ACKed, and the
     * client times out waiting for the full reply. */
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    /* ---- control phase ---- */
    for (;;) {
        op_common_t common;
        if (recv_all(s, &common, sizeof(common)) != 0) {
            goto done;
        }
        common.version = be16(common.version);
        common.code    = be16(common.code);
        common.status  = be32(common.status);

        if (common.code == OP_REQ_DEVLIST) {
            if (send_devlist_reply(s) != 0) {
                goto done;
            }
        } else if (common.code == OP_REQ_IMPORT) {
            char busid[USBIP_BUS_ID_SIZE];
            if (recv_all(s, busid, sizeof(busid)) != 0) {
                goto done;
            }
            if (send_import_reply(s) != 0) {
                goto done;
            }
            break;  /* enter the data plane */
        } else {
            send_op_common(s, (uint16_t)(common.code & ~OP_REQUEST), ST_ERROR);
            goto done;
        }
    }

    /* Take ownership of the device: stop the local REPL reader (which holds the
     * command CDC interfaces) and claim only the interfaces that actually exist
     * so the raw bulk/interrupt transfers are accepted. */
    mklink_probe_detach();

    mklink_usb_snapshot_t snap;
    int iface_cnt = 0;
    if (mklink_usb_get_snapshot(&snap) && snap.iface_cnt > 0 && snap.iface_cnt <= 32) {
        iface_cnt = snap.iface_cnt;
    }
    for (int i = 0; i < iface_cnt; i++) {
        (void)mklink_usb_claim_interface((uint8_t)i);
    }
    imported = true;
    mklink_log("USBIP: client imported the device, forwarding URBs");

    /* ---- data plane ---- */
    for (;;) {
        if (drain_completions(s, &cs) != 0) {
            break;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s, &rfds);
        /* Short poll so completed URBs are reported back within a few ms; a
         * long select here stalls RET_SUBMIT and the client times out. */
        struct timeval tv = { .tv_sec = 0, .tv_usec = 5000 };
        int r = select(s + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            break;
        }
        if (r > 0 && FD_ISSET(s, &rfds)) {
            if (handle_one_pdu(s, &cs) != 0) {
                break;
            }
        }
    }

done:
    /* Only an import touches the USB device state. A plain devlist must leave
     * the probe bridge and its claimed interfaces alone, otherwise the next
     * attach runs into "endpoint already allocated" errors. */
    if (imported) {
        uint8_t addr = mklink_usb_current_addr();
        if (addr != MKLINK_USB_ADDR_NONE) {
            mklink_usb_raw_unlink_all(addr);
        }
        /* Release every interface we claimed (non-claimed ones are a no-op). */
        for (int i = 0; i < 32; i++) {
            (void)mklink_usb_release_interface((uint8_t)i);
        }
    }
    for (int i = 0; i < MAX_PENDING; i++) {
        xfer_release(&cs.xfer[i]);
    }
    shutdown(s, SHUT_RDWR);
    close(s);
    mklink_log("USBIP: client disconnected");
    vTaskDelete(NULL);
}

/* ----------------------------------------------------------------- server -- */

static volatile bool s_running;
static uint16_t s_port;
static TaskHandle_t s_server_task;
static int s_listen_sock = -1;

static void server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        mklink_log("USBIP: socket() failed: %d", errno);
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(s_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listen_sock, 2) != 0) {
        mklink_log("USBIP: bind/listen on %u failed: %d", (unsigned)s_port, errno);
        close(listen_sock);
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    s_listen_sock = listen_sock;
    mklink_log("USBIP: server listening on %u", (unsigned)s_port);

    while (s_running) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int sock = accept(listen_sock, (struct sockaddr *)&peer, &peer_len);
        if (sock < 0) {
            if (s_running) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        char ip[16];
        esp_ip4addr_ntoa((esp_ip4_addr_t *)&peer.sin_addr, ip, sizeof(ip));
        mklink_log("USBIP: client %s connected", ip);

        /* Serve each client in its own task: it needs a large stack (socket
         * select + per-connection transfer table) and it deletes itself on
         * exit, so it must never run in the accept loop's context. */
        if (xTaskCreate(client_task, "mk_usbip_cli", 8192, (void *)(intptr_t)sock,
                        5, NULL) != pdPASS) {
            mklink_log("USBIP: failed to spawn client task");
            close(sock);
        }
    }

    s_listen_sock = -1;
    close(listen_sock);
    mklink_log("USBIP: server stopped");
    vTaskDelete(NULL);
}

esp_err_t mklink_usbip_start(uint16_t port)
{
    if (s_running) {
        return ESP_OK;
    }
    s_port = (port != 0) ? port : USBIP_PORT_DEFAULT;
    s_running = true;
    if (xTaskCreate(server_task, "mk_usbip", 4096, NULL, 5, &s_server_task) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t mklink_usbip_stop(void)
{
    s_running = false;
    /* Closing the listen socket unblocks the accept() call in server_task. */
    if (s_listen_sock >= 0) {
        int s = s_listen_sock;
        s_listen_sock = -1;
        shutdown(s, SHUT_RDWR);
        close(s);
    }
    return ESP_OK;
}

bool mklink_usbip_running(void)
{
    return s_running;
}

uint16_t mklink_usbip_port(void)
{
    return s_port;
}
