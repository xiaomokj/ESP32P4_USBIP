/*
 * MKLink Remote - core: event bus, log ring buffer, service orchestration.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "mklink_core.h"
#include "mklink_wifi.h"
#include "mklink_usb.h"
#include "mklink_priv.h"

static const char *TAG = "mklink";

ESP_EVENT_DEFINE_BASE(MKLINK_EVENT);

static SemaphoreHandle_t s_log_lock;
static char s_log_lines[MKLINK_LOG_MAX_LINES][MKLINK_LOG_LINE_LEN];
static int s_log_head;   /* next slot to write */
static int s_log_count;  /* number of valid lines */
static bool s_core_ready;

static void ensure_log_lock(void)
{
    if (s_log_lock == NULL) {
        s_log_lock = xSemaphoreCreateMutex();
    }
}

void mklink_log(const char *fmt, ...)
{
    char line[MKLINK_LOG_LINE_LEN];

    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    /* Console first: never let a broken UI starve the serial log. */
    ESP_LOGI(TAG, "%s", line);

    ensure_log_lock();
    if (s_log_lock != NULL && xSemaphoreTake(s_log_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        strlcpy(s_log_lines[s_log_head], line, MKLINK_LOG_LINE_LEN);
        s_log_head = (s_log_head + 1) % MKLINK_LOG_MAX_LINES;
        if (s_log_count < MKLINK_LOG_MAX_LINES) {
            s_log_count++;
        }
        xSemaphoreGive(s_log_lock);
    }

    if (s_core_ready) {
        mklink_log_line_t payload;
        strlcpy(payload.line, line, sizeof(payload.line));
        /* Best effort: drop the event when the queue is full, the ring keeps the text. */
        esp_event_post(MKLINK_EVENT, MKLINK_EVENT_LOG, &payload, sizeof(payload), 0);
    }
}

size_t mklink_log_tail(char *dst, size_t dst_len, int max_lines)
{
    if (dst == NULL || dst_len == 0) {
        return 0;
    }
    dst[0] = '\0';

    if (max_lines <= 0) {
        return 0;
    }

    ensure_log_lock();
    if (s_log_lock == NULL || xSemaphoreTake(s_log_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return 0;
    }

    int lines = s_log_count < max_lines ? s_log_count : max_lines;
    int start = (s_log_head - lines + MKLINK_LOG_MAX_LINES) % MKLINK_LOG_MAX_LINES;
    size_t used = 0;

    for (int i = 0; i < lines; i++) {
        const char *src = s_log_lines[(start + i) % MKLINK_LOG_MAX_LINES];
        size_t src_len = strlen(src);
        if (used + src_len + 2 >= dst_len) {
            break;
        }
        memcpy(dst + used, src, src_len);
        used += src_len;
        dst[used++] = '\n';
        dst[used] = '\0';
    }

    xSemaphoreGive(s_log_lock);
    return used;
}

esp_err_t mklink_core_init(void)
{
    ensure_log_lock();

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(err));
        return err;
    }

    esp_err_t wifi_err = mklink_wifi_early_init();
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "wifi lock: %s", esp_err_to_name(wifi_err));
        return wifi_err;
    }

    s_core_ready = true;
    mklink_log("MKLink Remote %s starting", mklink_core_version());
    return ESP_OK;
}

esp_err_t mklink_core_start_services(void)
{
    esp_err_t wifi_err = mklink_wifi_ensure_started();
    if (wifi_err != ESP_OK) {
        mklink_log("WiFi start failed: %s", esp_err_to_name(wifi_err));
    }

    esp_err_t disc_err = mklink_discovery_start();
    if (disc_err != ESP_OK) {
        ESP_LOGW(TAG, "probe discovery: %s", esp_err_to_name(disc_err));
    }

    esp_err_t usb_err = mklink_usb_start();
    if (usb_err != ESP_OK) {
        mklink_log("USB host start failed: %s", esp_err_to_name(usb_err));
    }

    esp_err_t hub_err = mklink_stream_hub_start(CONFIG_MKLINK_STREAM_PORT);
    if (hub_err != ESP_OK) {
        ESP_LOGW(TAG, "stream hub: %s", esp_err_to_name(hub_err));
    }

    esp_err_t net_err = mklink_net_start(CONFIG_MKLINK_NET_PORT);
    if (net_err != ESP_OK) {
        mklink_log("TCP status service failed: %s", esp_err_to_name(net_err));
    }

#if CONFIG_MKLINK_USBIP_AUTOSTART
    esp_err_t usbip_err = mklink_usbip_start(CONFIG_MKLINK_USBIP_PORT);
    if (usbip_err != ESP_OK) {
        mklink_log("USBIP autostart failed: %s", esp_err_to_name(usbip_err));
    }
#endif

    mklink_wifi_status_t wifi;
    mklink_wifi_status(&wifi);
    mklink_log("Services up: AP \"%s\" %s, TCP %d",
               wifi.ap_ssid, wifi.ap_ip, CONFIG_MKLINK_NET_PORT);
    return ESP_OK;
}

const char *mklink_core_version(void)
{
    return "0.1.0";
}
