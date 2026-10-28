/*
 * MKLink Remote - core services (event bus, log ring, service orchestration)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Event base shared by every MKLink subsystem. All payloads are fixed size. */
ESP_EVENT_DECLARE_BASE(MKLINK_EVENT);

typedef enum {
    MKLINK_EVENT_USB_ADDED = 0,   /* payload: mklink_usb_snapshot_t */
    MKLINK_EVENT_USB_REMOVED,     /* payload: mklink_usb_snapshot_t */
    MKLINK_EVENT_USB_SCAN_DONE,   /* payload: none */
    MKLINK_EVENT_CDC_RESULT,      /* payload: mklink_cdc_result_t */
    MKLINK_EVENT_WIFI_STATE,      /* payload: mklink_wifi_status_t */
    MKLINK_EVENT_LOG,             /* payload: mklink_log_line_t */
    MKLINK_EVENT_MAX,
} mklink_event_id_t;

#define MKLINK_USB_STRING_LEN   32
#define MKLINK_USB_PRODUCT_LEN  64
#define MKLINK_SSID_LEN         33
#define MKLINK_IP_LEN           16
#define MKLINK_LOG_LINE_LEN     96
#define MKLINK_LOG_MAX_LINES    200

typedef struct {
    bool     connected;
    uint16_t vid;
    uint16_t pid;
    uint8_t  dev_addr;   /* dynamic USB address, not stable across re-enumeration */
    uint8_t  iface_cnt;
    uint8_t  ep_cnt;
    uint8_t  cdc_iface;  /* 0xFF when the device has no CDC interface */
    char     busid[20];  /* stable usbip id built from the physical port path: "1-1.2" */
    char     manufacturer[MKLINK_USB_STRING_LEN];
    char     product[MKLINK_USB_PRODUCT_LEN];
} mklink_usb_snapshot_t;

typedef struct {
    bool    ap_up;
    bool    sta_up;
    int8_t  rssi;
    uint8_t rssi_level;  /* 0..3, maps to the status bar icon */
    uint8_t ap_clients;
    char    sta_ssid[MKLINK_SSID_LEN];
    char    sta_ip[MKLINK_IP_LEN];
    char    ap_ssid[MKLINK_SSID_LEN];
    char    ap_ip[MKLINK_IP_LEN];
    char    ap_pass[65];  /* shown on the dashboard so a phone can join quickly */
} mklink_wifi_status_t;

typedef struct {
    bool     ran;
    bool     pass;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    char     detail[64];
} mklink_cdc_result_t;

typedef struct {
    char line[MKLINK_LOG_LINE_LEN];
} mklink_log_line_t;

/**
 * @brief Create the default event loop and internal primitives.
 *
 * Safe to call once from app_main() before any other mklink_* API.
 */
esp_err_t mklink_core_init(void);

/**
 * @brief Start WiFi (AP+STA), USB host and the TCP status service.
 */
esp_err_t mklink_core_start_services(void);

/** Log helper: writes to the UI ring buffer *and* the console. */
void mklink_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Copy the newest @p max_lines log lines into @p dst (oldest first).
 * @return Number of bytes written into @p dst (always NUL terminated on success).
 */
size_t mklink_log_tail(char *dst, size_t dst_len, int max_lines);

/** Firmware version string shown in the UI. */
const char *mklink_core_version(void);

#ifdef __cplusplus
}
#endif
