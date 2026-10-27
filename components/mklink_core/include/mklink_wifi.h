/*
 * MKLink Remote - WiFi manager (ESP32-C6 co-processor, AP+STA)
 *
 * Single owner of esp_wifi: the settings app must go through these calls instead
 * of touching esp_wifi directly, otherwise two state machines fight over the
 * C6 hosted link.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "mklink_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Bring up APSTA (idempotent). Returns ESP_OK if already running. */
esp_err_t mklink_wifi_ensure_started(void);

bool mklink_wifi_is_started(void);

/**
 * @brief Connect to an AP as station, persist the credentials in NVS.
 */
esp_err_t mklink_wifi_connect_sta(const char *ssid, const char *password);

esp_err_t mklink_wifi_disconnect_sta(void);

/** Remove the stored STA credentials (keeps AP running). */
esp_err_t mklink_wifi_forget_sta(void);

/** Fill @p out with the current AP/STA state. */
esp_err_t mklink_wifi_status(mklink_wifi_status_t *out);

/** Signal strength bucket 0..3 used by the status bar icon. */
uint8_t mklink_wifi_rssi_level(int8_t rssi);

#ifdef __cplusplus
}
#endif
