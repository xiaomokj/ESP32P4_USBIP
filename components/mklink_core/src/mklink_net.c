/*
 * MKLink Remote - TCP status service.
 *
 * Answers a single-line JSON document on every connection, so "is the network
 * path up?" can be checked from a phone (AP) or a laptop (LAN) with nothing but
 * netcat. Also serves as the placeholder for the future usbip endpoint.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "mklink_core.h"
#include "mklink_rtt.h"
#include "mklink_watch.h"
#include "mklink_usb.h"
#include "mklink_wifi.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.net";

#define NET_MAX_CLIENTS 3

static void json_escape(char *dst, size_t dst_len, const char *src)
{
    size_t out = 0;
    for (size_t i = 0; src[i] != '\0' && out + 2 < dst_len; i++) {
        char c = src[i];
        if (c == '"' || c == '\\') {
            dst[out++] = '\\';
        }
        if ((unsigned char)c < 0x20) {
            c = ' ';
        }
        dst[out++] = c;
    }
    dst[out] = '\0';
}

static int build_status_json(char *buf, size_t len)
{
    mklink_usb_snapshot_t usb = {0};
    bool usb_ok = mklink_usb_get_snapshot(&usb);

    mklink_wifi_status_t wifi = {0};
    mklink_wifi_status(&wifi);

    mklink_cdc_result_t cdc = {0};
    bool cdc_ok = mklink_cdc_get_result(&cdc);

    char product[80];
    char ssid[48];
    char ap_ssid[48];
    char ap_pass[72];
    json_escape(product, sizeof(product), usb_ok ? usb.product : "");
    json_escape(ssid, sizeof(ssid), wifi.sta_ssid);
    json_escape(ap_ssid, sizeof(ap_ssid), wifi.ap_ssid);
    json_escape(ap_pass, sizeof(ap_pass), wifi.ap_pass);

    return snprintf(buf, len,
                    "{\"app\":\"mklink-remote\",\"fw\":\"%s\","
                    "\"usb\":{\"connected\":%s,\"vid\":\"%04X\",\"pid\":\"%04X\","
                    "\"product\":\"%s\",\"ifaces\":%u,\"eps\":%u,\"cdc_iface\":%d},"
                    "\"cdc\":{\"ran\":%s,\"pass\":%s,\"tx\":%u,\"rx\":%u},"
                    "\"wifi\":{\"sta_up\":%s,\"sta_ssid\":\"%s\",\"sta_ip\":\"%s\",\"rssi\":%d,"
                    "\"ap_up\":%s,\"ap_ssid\":\"%s\",\"ap_pass\":\"%s\",\"ap_ip\":\"%s\",\"ap_clients\":%u}}\n",
                    mklink_core_version(),
                    usb_ok ? "true" : "false", usb.vid, usb.pid, product, usb.iface_cnt, usb.ep_cnt,
                    usb.cdc_iface == 0xFF ? -1 : (int)usb.cdc_iface,
                    cdc_ok ? "true" : "false", cdc.pass ? "true" : "false", (unsigned)cdc.tx_bytes,
                    (unsigned)cdc.rx_bytes,
                    wifi.sta_up ? "true" : "false", ssid, wifi.sta_ip, wifi.rssi,
                    wifi.ap_up ? "true" : "false", ap_ssid, ap_pass, wifi.ap_ip, wifi.ap_clients);
}

/* --------------------------------------------------------------- HTTP ---- */

static void http_reply(int sock, int code, const char *status, const char *ctype,
                       const char *body, size_t body_len)
{
    char head[192];
    int n = snprintf(head, sizeof(head),
                     "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
                     "Connection: close\r\n\r\n",
                     code, status, ctype, (unsigned)body_len);
    if (n > 0) {
        (void)send(sock, head, (size_t)n, 0);
    }
    if (body != NULL && body_len > 0) {
        (void)send(sock, body, body_len, 0);
    }
}

static const char *ct_json = "application/json";
static const char *ct_text = "text/plain";

/**
 * Requests we serve:
 *   GET  /            -> status JSON (unchanged fields)
 *   POST /watch       -> body {"vars":[{"name","addr","type"}], "period_ms":1}
 *   GET  /watch       -> current variable table + values
 *   GET  /rtt         -> RTT counters (the raw stream lives on the hub port)
 */
static void handle_request(int sock, const char *request, size_t request_len,
                           const char *body, size_t body_len, bool body_complete)
{
    char method[8] = {0};
    char path[64] = {0};

    if (sscanf(request, "%7s %63s", method, path) != 2) {
        http_reply(sock, 400, "Bad Request", ct_json, "{\"error\":\"bad request\"}\n", 26);
        return;
    }

    const char *q = strchr(path, '?');
    if (q != NULL) {
        *(char *)q = '\0';
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/") == 0) {
        char json[512];
        int len = build_status_json(json, sizeof(json));
        http_reply(sock, 200, "OK", ct_json, json, len > 0 ? (size_t)len : 0);
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/watch") == 0) {
        char json[1024];
        int len = mklink_watch_to_json(json, sizeof(json));
        if (len <= 0) {
            const char *empty = "{\"vars\":[]}\n";
            http_reply(sock, 200, "OK", ct_json, empty, strlen(empty));
        } else {
            http_reply(sock, 200, "OK", ct_json, json, (size_t)len);
        }
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/rtt") == 0) {
        mklink_rtt_status_t st;
        mklink_rtt_status(&st);
        char json[192];
        int len = snprintf(json, sizeof(json),
                           "{\"running\":%s,\"bytes\":%u,\"dropped\":%u,\"cb\":\"%s\"}\n",
                           st.running ? "true" : "false", (unsigned)st.bytes_total,
                           (unsigned)st.dropped, st.cb_addr);
        http_reply(sock, 200, "OK", ct_json, json, len > 0 ? (size_t)len : 0);
        return;
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/watch") == 0) {
        if (!body_complete) {
            http_reply(sock, 413, "Payload Too Large", ct_json, "{\"error\":\"body incomplete\"}\n", 28);
            return;
        }
        esp_err_t err = mklink_watch_apply_json(body, body_len);
        if (err != ESP_OK) {
            const char *msg = "{\"ok\":false}\n";
            http_reply(sock, 400, "Bad Request", ct_json, msg, strlen(msg));
            return;
        }
        mklink_watch_status_t st;
        mklink_watch_snapshot(NULL, 0, &st);
        char json[128];
        int len = snprintf(json, sizeof(json),
                           "{\"ok\":true,\"vars\":%u,\"regions\":%u}\n",
                           (unsigned)st.var_count, (unsigned)st.region_count);
        http_reply(sock, 200, "OK", ct_json, json, len > 0 ? (size_t)len : 0);

        /* starting is explicit in the JSON ("start":true) or implied */
        if (strstr(body, "\"start\":false") == NULL) {
            mklink_watch_start();
        }
        return;
    }

    http_reply(sock, 404, "Not Found", ct_text, "not found\n", 10);
}

static void net_client_task(void *arg)
{
    int sock = (int)(intptr_t)arg;
    char req[2048];
    size_t used = 0;
    bool body_complete = true;

    struct timeval tv = {.tv_sec = 5, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    size_t want_body = 0;
    while (used + 1 < sizeof(req)) {
        int n = recv(sock, req + used, sizeof(req) - used - 1, 0);
        if (n <= 0) {
            break;
        }
        used += (size_t)n;
        req[used] = '\0';

        char *header_end = strstr(req, "\r\n\r\n");
        if (header_end == NULL) {
            continue;
        }

        char *cl = strstr(req, "Content-Length:");
        if (cl == NULL) {
            cl = strstr(req, "content-length:");
        }
        if (cl != NULL) {
            want_body = (size_t)strtoul(cl + 15, NULL, 10);
        }
        size_t header_len = (size_t)(header_end + 4 - req);
        if (used - header_len >= want_body) {
            break;
        }
    }

    char *header_end = strstr(req, "\r\n\r\n");
    const char *body = "";
    size_t body_len = 0;
    if (header_end != NULL) {
        body = header_end + 4;
        body_len = used - (size_t)(body - req);
        body_complete = (body_len >= want_body);
    }

    handle_request(sock, req, used, body, body_len, body_complete);

    shutdown(sock, SHUT_RDWR);
    close(sock);
    vTaskDelete(NULL);
}

static void net_server_task(void *arg)
{
    uint16_t port = (uint16_t)(uintptr_t)arg;

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        mklink_log("TCP: socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        mklink_log("TCP: bind(%u) failed: %d", port, errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, NET_MAX_CLIENTS) != 0) {
        mklink_log("TCP: listen failed: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    mklink_log("TCP: status service listening on %u", port);

    while (true) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int sock = accept(listen_sock, (struct sockaddr *)&peer, &peer_len);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char ip[16];
        esp_ip4addr_ntoa((esp_ip4_addr_t *)&peer.sin_addr, ip, sizeof(ip));
        mklink_log("TCP: client %s connected", ip);

        if (xTaskCreate(net_client_task, "mk_net_cli", 3072, (void *)(intptr_t)sock, 4, NULL) != pdPASS) {
            close(sock);
        }
    }
}

esp_err_t mklink_net_start(uint16_t port)
{
    if (xTaskCreate(net_server_task, "mk_net", 4096, (void *)(uintptr_t)port, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
