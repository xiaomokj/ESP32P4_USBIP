/*
 * MKLink Remote - internal interfaces shared between mklink_core sources.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "usb/usb_host.h"
#include "mklink_core.h"
#include "mklink_usb_raw.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MKLINK_USB_ADDR_NONE 0xFF

/* ---- USB device description (used by probe auto-discovery) ---- */

/** One CDC function pair as exposed by a composite device. */
typedef struct {
    uint8_t comm_iface;   /* CDC communication interface (class 0x02), 0xFF if absent */
    uint8_t data_iface;   /* CDC data interface (class 0x0A) */
    uint8_t ep_out;       /* bulk OUT on the data interface */
    uint8_t ep_in;        /* bulk IN on the data interface */
    uint16_t mps_in;
    uint16_t mps_out;
} mklink_cdc_group_t;

#define MKLINK_MAX_CDC_GROUPS 4

/** Descriptor snapshot of an opened device, before it is "adopted". */
typedef struct {
    uint8_t addr;
    usb_device_handle_t hdl;         /* owned by this struct until adopted/released */
    uint16_t vid, pid;
    uint8_t dev_class;               /* bDeviceClass (0x09 = HUB, skipped) */
    uint8_t iface_cnt;
    uint8_t ep_count;
    uint8_t cdc_count;
    uint8_t iface_cls[32];
    mklink_ep_info_t eps[16];
    mklink_cdc_group_t cdc[MKLINK_MAX_CDC_GROUPS];
    char manufacturer[MKLINK_USB_STRING_LEN];
    char product[MKLINK_USB_PRODUCT_LEN];
    char topology[512];
} mklink_usb_desc_t;

/** Open @p addr and read device/config descriptors + CDC groups. */
esp_err_t mklink_usb_describe(uint8_t addr, mklink_usb_desc_t *out);

/** Close a described (not adopted) device. */
void mklink_usb_desc_release(mklink_usb_desc_t *desc);

/** Make @p desc the active device (takes ownership of its handle).
 *  Does not log or publish an event - call mklink_usb_confirm_adopted() once the
 *  device passed validation, or mklink_usb_discard_adopted() to roll back. */
esp_err_t mklink_usb_adopt(mklink_usb_desc_t *desc);
void mklink_usb_confirm_adopted(void);
void mklink_usb_discard_adopted(void);

/** @return true when no further device can be adopted (table full). */
bool mklink_usb_busy(void);

/** Number of enumerated/adopted devices. */
int mklink_usb_device_count(void);
/** Address of the @p idx-th adopted device (MKLINK_USB_ADDR_NONE if out of range). */
uint8_t mklink_usb_addr_at(int idx);
/** Snapshot of the @p idx-th adopted device. */
bool mklink_usb_get_snapshot_at(int idx, mklink_usb_snapshot_t *out);
/** Snapshot of the device with @p dev_addr. */
bool mklink_usb_snapshot_for_addr(uint8_t dev_addr, mklink_usb_snapshot_t *out);

/** CDC groups of the first adopted device (used by the probe bridge). */
int mklink_usb_cdc_group_list(mklink_cdc_group_t *out, size_t max);

/** Fill @p addrs with the currently attached device addresses. */
int mklink_usb_addr_list(uint8_t *addrs, size_t max);

/* ---- probe auto-discovery ---- */
esp_err_t mklink_discovery_start(void);
/** Evaluate a freshly attached device (called from the USB client event). */
void mklink_discovery_on_device(uint8_t dev_addr);
/** Full rescan (manual "rescan USB" / boot). */
esp_err_t mklink_discovery_rescan(void);

/* ---- stream hub (TCP fan-out for PC clients) ---- */
esp_err_t mklink_stream_hub_start(uint16_t port);
/** Push a frame to every subscribed client. type: 1=RTT bytes, 2=dump frame. */
void mklink_stream_hub_publish(uint8_t type, const uint8_t *data, size_t len);

/* ---- net_service ---- */
esp_err_t mklink_net_start(uint16_t port);

/* ---- wifi manager internals ---- */
/** Create the WiFi manager lock. Called from mklink_core_init(). */
esp_err_t mklink_wifi_early_init(void);

/* ---- usbip server (public API in mklink_usbip.h) ---- */
#include "mklink_usbip.h"

/* ---- usb_bridge internals (used by the raw transfer layer and CDC test) ---- */
usb_host_client_handle_t mklink_usb_client_handle(void);
bool mklink_usb_handle_for_addr(uint8_t dev_addr, usb_device_handle_t *out);
uint8_t mklink_usb_current_addr(void);
esp_err_t mklink_usb_claim_interface(uint8_t dev_addr, uint8_t iface_num);
esp_err_t mklink_usb_release_interface(uint8_t dev_addr, uint8_t iface_num);
int mklink_usb_endpoint_list(uint8_t dev_addr, mklink_ep_info_t *out, size_t max);

/** @return true when a CDC data interface was found; @p comm / @p data are 0xFF if absent. */
bool mklink_usb_cdc_ifaces(uint8_t *comm, uint8_t *data);

/** @return true while @p dev_addr is enumerated and no teardown is in progress. */
bool mklink_usb_device_alive(uint8_t dev_addr);

/** In-flight transfer accounting, used to delay device teardown until the
 *  aborted transfers have delivered their callbacks. */
void mklink_usb_xfer_begin(void);
void mklink_usb_xfer_end(void);

/** Abort any transfer pending on @p ep_addr (flush). Safe to call after removal. */
esp_err_t mklink_usb_raw_abort(uint8_t dev_addr, uint8_t ep_addr);

/* ---- cdc test entry, invoked by the usb bridge on enumeration ---- */
esp_err_t mklink_cdc_test_start(void);

/** Synchronous control/bulk transfer on the active device (shared helper).
 *  @p ep_addr is ignored for control transfers; @p setup != NULL selects control. */
esp_err_t mklink_cdc_sync_transfer(uint8_t addr, uint8_t ep_addr, uint8_t *buf, size_t len,
                                   bool control, const void *setup, size_t *actual);

/* ---- helpers ---- */
void mklink_cdc_store_result(const mklink_cdc_result_t *res);
int mklink_utf16_to_utf8(const uint16_t *src, int src_len, char *dst, size_t dst_len);

#ifdef __cplusplus
}
#endif
