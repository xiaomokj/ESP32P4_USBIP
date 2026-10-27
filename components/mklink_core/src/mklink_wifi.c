/*
 * MKLink Remote - WiFi manager: ESP32-C6 (esp_hosted) APSTA owner.
 *
 * Both the SoftAP (direct link) and the STA (LAN link) run at the same time.
 * Credentials live in NVS under the "mklink_wifi" namespace and are restored on
 * boot; the station reconnect uses an exponential backoff timer.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "mklink_core.h"
#include "mklink_wifi.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.wifi";

#define NVS_NS_WIFI         "mklink_wifi"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASS        "pass"
#define STA_RETRY_MAX       8
#define STA_RETRY_MIN_MS    1000
#define STA_RETRY_MAX_MS    30000

static bool s_started;
static bool s_sta_connected;
static bool s_ap_started;
static uint8_t s_ap_clients;
static int s_retry;
static esp_timer_handle_t s_retry_timer;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static wifi_config_t s_sta_cfg;
static bool s_sta_cfg_valid;

static void post_state(void)
{
    mklink_wifi_status_t st;
    if (mklink_wifi_status(&st) == ESP_OK) {
        esp_event_post(MKLINK_EVENT, MKLINK_EVENT_WIFI_STATE, &st, sizeof(st), 0);
    }
}

uint8_t mklink_wifi_rssi_level(int8_t rssi)
{
    if (rssi >= -55) {
        return 3;
    }
    if (rssi >= -67) {
        return 2;
    }
    if (rssi >= -78) {
        return 1;
    }
    return 0;
}

static bool load_sta_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NS_WIFI, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t len = ssid_len;
    esp_err_t err = nvs_get_str(handle, NVS_KEY_SSID, ssid, &len);
    if (err == ESP_OK) {
        len = pass_len;
        err = nvs_get_str(handle, NVS_KEY_PASS, pass, &len);
    }
    nvs_close(handle);

    if (err != ESP_OK || ssid[0] == '\0') {
        return false;
    }
    return true;
}

static esp_err_t save_sta_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS_WIFI, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, NVS_KEY_PASS, pass != NULL ? pass : "");
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static void sta_retry_cb(void *arg)
{
    if (!s_started || s_sta_connected || !s_sta_cfg_valid) {
        return;
    }
    mklink_log("WiFi: reconnecting to \"%s\" (attempt %d)", (const char *)s_sta_cfg.sta.ssid, s_retry + 1);
    esp_wifi_connect();
}

static void schedule_reconnect(void)
{
    if (!s_sta_cfg_valid || s_retry >= STA_RETRY_MAX) {
        return;
    }

    uint32_t delay = STA_RETRY_MIN_MS << (s_retry > 4 ? 4 : s_retry);
    if (delay > STA_RETRY_MAX_MS) {
        delay = STA_RETRY_MAX_MS;
    }
    s_retry++;

    if (s_retry_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = sta_retry_cb,
            .name = "mklink_sta_retry",
        };
        if (esp_timer_create(&args, &s_retry_timer) != ESP_OK) {
            return;
        }
    }
    esp_timer_stop(s_retry_timer);
    esp_timer_start_once(s_retry_timer, (uint64_t)delay * 1000ULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            if (s_sta_cfg_valid && !s_sta_connected) {
                esp_wifi_connect();
            }
            break;
        case WIFI_EVENT_STA_CONNECTED:
            mklink_log("WiFi: associated with \"%s\"", (const char *)s_sta_cfg.sta.ssid);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            s_sta_connected = false;
            post_state();
            mklink_log("WiFi: station disconnected, scheduling retry");
            schedule_reconnect();
            break;
        case WIFI_EVENT_AP_START:
            s_ap_started = true;
            post_state();
            break;
        case WIFI_EVENT_AP_STACONNECTED: {
            s_ap_clients++;
            mklink_log("WiFi: hotspot client joined (%u total)", s_ap_clients);
            post_state();
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED:
            if (s_ap_clients > 0) {
                s_ap_clients--;
            }
            mklink_log("WiFi: hotspot client left (%u total)", s_ap_clients);
            post_state();
            break;
        default:
            break;
        }
        return;
    }

    if (base == IP_EVENT) {
        if (id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
            s_sta_connected = true;
            s_retry = 0;
            mklink_log("WiFi: station got IP " IPSTR, IP2STR(&evt->ip_info.ip));
            post_state();
        } else if (id == IP_EVENT_STA_LOST_IP) {
            s_sta_connected = false;
            post_state();
        }
    }
}

static SemaphoreHandle_t s_mutex;

esp_err_t mklink_wifi_early_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
    return (s_mutex != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t wifi_start_locked(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_sta_netif == NULL || s_ap_netif == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    /* Credentials are managed by this component, not by the WiFi driver. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t ap_cfg = {0};
    strlcpy((char *)ap_cfg.ap.ssid, CONFIG_MKLINK_WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(CONFIG_MKLINK_WIFI_AP_SSID);
    strlcpy((char *)ap_cfg.ap.password, CONFIG_MKLINK_WIFI_AP_PASSWORD, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.channel = CONFIG_MKLINK_WIFI_AP_CHANNEL;
    ap_cfg.ap.max_connection = CONFIG_MKLINK_WIFI_AP_MAX_CONN;
    ap_cfg.ap.authmode = strlen(CONFIG_MKLINK_WIFI_AP_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap_cfg.ap.pmf_cfg.required = false;

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        return err;
    }

    char ssid[MKLINK_SSID_LEN] = {0};
    char pass[65] = {0};
    if (load_sta_credentials(ssid, sizeof(ssid), pass, sizeof(pass))) {
        memset(&s_sta_cfg, 0, sizeof(s_sta_cfg));
        strlcpy((char *)s_sta_cfg.sta.ssid, ssid, sizeof(s_sta_cfg.sta.ssid));
        strlcpy((char *)s_sta_cfg.sta.password, pass, sizeof(s_sta_cfg.sta.password));
        /* Allow open networks as well; the AP decides the real auth mode. */
        s_sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        s_sta_cfg.sta.pmf_cfg.capable = true;
        s_sta_cfg_valid = true;
        esp_wifi_set_config(WIFI_IF_STA, &s_sta_cfg);
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    s_started = true;
    mklink_log("WiFi: AP+STA started, hotspot \"%s\"", CONFIG_MKLINK_WIFI_AP_SSID);

    if (s_sta_cfg_valid && CONFIG_MKLINK_WIFI_STA_AUTOCONNECT) {
        s_retry = 0;
        esp_wifi_connect();
    }

    return ESP_OK;
}

esp_err_t mklink_wifi_ensure_started(void)
{
    /* The settings app and the boot sequence both call this, so the start path
     * has to be serialized - otherwise two esp_netif instances are created. */
    if (s_mutex == NULL) {
        esp_err_t init_err = mklink_wifi_early_init();
        if (init_err != ESP_OK) {
            return init_err;
        }
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!s_started) {
        err = wifi_start_locked();
    }
    xSemaphoreGive(s_mutex);
    return err;
}

bool mklink_wifi_is_started(void)
{
    return s_started;
}

esp_err_t mklink_wifi_connect_sta(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = mklink_wifi_ensure_started();
    if (err != ESP_OK) {
        return err;
    }

    memset(&s_sta_cfg, 0, sizeof(s_sta_cfg));
    strlcpy((char *)s_sta_cfg.sta.ssid, ssid, sizeof(s_sta_cfg.sta.ssid));
    if (password != NULL) {
        strlcpy((char *)s_sta_cfg.sta.password, password, sizeof(s_sta_cfg.sta.password));
    }
    s_sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    s_sta_cfg.sta.pmf_cfg.capable = true;
    s_sta_cfg_valid = true;
    s_retry = 0;

    if (s_retry_timer != NULL) {
        esp_timer_stop(s_retry_timer);
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &s_sta_cfg);
    if (err != ESP_OK) {
        mklink_log("WiFi: set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    save_sta_credentials(ssid, password != NULL ? password : "");
    mklink_log("WiFi: connecting to \"%s\"", ssid);

    esp_wifi_disconnect();
    err = esp_wifi_connect();
    return err;
}

esp_err_t mklink_wifi_disconnect_sta(void)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_retry_timer != NULL) {
        esp_timer_stop(s_retry_timer);
    }
    s_retry = STA_RETRY_MAX; /* stop the backoff loop until the next explicit connect */
    s_sta_connected = false;
    s_sta_cfg_valid = false;
    esp_err_t err = esp_wifi_disconnect();
    post_state();
    return err;
}

esp_err_t mklink_wifi_forget_sta(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS_WIFI, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    nvs_erase_key(handle, NVS_KEY_SSID);
    nvs_erase_key(handle, NVS_KEY_PASS);
    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t mklink_wifi_status(mklink_wifi_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    strlcpy(out->ap_ssid, CONFIG_MKLINK_WIFI_AP_SSID, sizeof(out->ap_ssid));
    if (strlen(CONFIG_MKLINK_WIFI_AP_PASSWORD) >= 8) {
        strlcpy(out->ap_pass, CONFIG_MKLINK_WIFI_AP_PASSWORD, sizeof(out->ap_pass));
    }
    out->ap_up = s_ap_started;
    out->ap_clients = s_ap_clients;

    if (s_ap_netif != NULL) {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK) {
            esp_ip4addr_ntoa(&ip.ip, out->ap_ip, sizeof(out->ap_ip));
        }
    }

    if (s_sta_netif != NULL) {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(s_sta_netif, &ip) == ESP_OK && ip.ip.addr != 0) {
            esp_ip4addr_ntoa(&ip.ip, out->sta_ip, sizeof(out->sta_ip));
        }
        strlcpy(out->sta_ssid, (const char *)s_sta_cfg.sta.ssid, sizeof(out->sta_ssid));
    }

    out->sta_up = s_sta_connected;
    if (s_sta_connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            out->rssi = ap.rssi;
        }
    }
    out->rssi_level = mklink_wifi_rssi_level(out->rssi);
    return ESP_OK;
}
