/*
 * MKLink Remote - stream hub.
 *
 * Lets a PC watch the same RTT / dump traffic that is drawn on the panel:
 *
 *   client -> one text line: {"cmd":"rtt"} | {"cmd":"watch"} | {"cmd":"status"}
 *   server -> frames:        [type u8][len u16 LE][payload]
 *                             1 = RTT bytes, 2 = dump frame (verbatim),
 *                             3 = status text
 *
 * Every client owns a 32 KB ring; when it cannot keep up the oldest bytes are
 * dropped, so a slow or stalled PC never blocks the USB reader.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "mklink_core.h"
#include "mklink_rtt.h"
#include "mklink_wifi.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.hub";

#define HUB_PORT_DEFAULT   8081
#define HUB_MAX_CLIENTS    2
#define HUB_RING_SIZE      32768
#define HUB_MAX_FRAME      1500

typedef struct {
    int sock;
    bool used;
    bool subscribed;
    uint8_t kind;              /* 1 = rtt, 2 = watch */
    SemaphoreHandle_t lock;
    uint8_t ring[HUB_RING_SIZE];
    size_t head;
    size_t tail;
    uint32_t dropped;
} hub_client_t;

static hub_client_t s_clients[HUB_MAX_CLIENTS];
static SemaphoreHandle_t s_clients_lock;
static uint16_t s_port = HUB_PORT_DEFAULT;

static void lock_clients(void)
{
    if (s_clients_lock != NULL) {
        xSemaphoreTake(s_clients_lock, portMAX_DELAY);
    }
}

static void unlock_clients(void)
{
    if (s_clients_lock != NULL) {
        xSemaphoreGive(s_clients_lock);
    }
}

static void client_reset(hub_client_t *c)
{
    c->used = false;
    c->subscribed = false;
    c->kind = 0;
    c->head = 0;
    c->tail = 0;
    c->dropped = 0;
    if (c->sock >= 0) {
        close(c->sock);
        c->sock = -1;
    }
}

void mklink_stream_hub_publish(uint8_t type, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || s_clients_lock == NULL) {
        return;
    }

    for (int i = 0; i < HUB_MAX_CLIENTS; i++) {
        hub_client_t *c = &s_clients[i];
        if (!c->used || !c->subscribed || c->kind != type) {
            continue;
        }

        xSemaphoreTake(c->lock, portMAX_DELAY);
        for (size_t n = 0; n < len; n++) {
            size_t next = (c->head + 1) % HUB_RING_SIZE;
            if (next == c->tail) {
                c->tail = (c->tail + 1) % HUB_RING_SIZE;
                c->dropped++;
            }
            c->ring[c->head] = data[n];
            c->head = next;
        }
        xSemaphoreGive(c->lock);
    }
}

static void hub_send_frame(hub_client_t *c, uint8_t type, const uint8_t *payload, size_t len)
{
    uint8_t header[3] = {type, (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF)};
    if (send(c->sock, header, sizeof(header), 0) != (int)sizeof(header)) {
        return;
    }
    if (len > 0) {
        (void)send(c->sock, payload, len, 0);
    }
}

static void hub_client_task(void *arg)
{
    hub_client_t *c = (hub_client_t *)arg;
    char line[96];
    size_t used = 0;
    uint8_t buf[HUB_MAX_FRAME];

    /* subscription line */
    while (used + 1 < sizeof(line)) {
        int n = recv(c->sock, line + used, sizeof(line) - used - 1, 0);
        if (n <= 0) {
            goto done;
        }
        used += (size_t)n;
        line[used] = '\0';
        if (strchr(line, '\n') != NULL) {
            break;
        }
    }

    if (strstr(line, "\"rtt\"") != NULL) {
        c->kind = 1;
    } else if (strstr(line, "\"watch\"") != NULL) {
        c->kind = 2;
    } else if (strstr(line, "\"status\"") != NULL) {
        c->kind = 3;
    } else {
        const char *err = "{\"ok\":false,\"error\":\"unknown cmd\"}\n";
        hub_send_frame(c, 3, (const uint8_t *)err, strlen(err));
        goto done;
    }

    c->subscribed = true;
    {
        char ack[96];
        snprintf(ack, sizeof(ack), "{\"ok\":true,\"stream\":%s}\n",
                 (c->kind == 1) ? "rtt" : (c->kind == 2) ? "watch" : "status");
        hub_send_frame(c, 3, (const uint8_t *)ack, strlen(ack));
    }

    while (c->used) {
        if (c->kind == 3) {
            char st[160];
            mklink_wifi_status_t wifi;
            mklink_wifi_status(&wifi);
            snprintf(st, sizeof(st),
                     "{\"rtt_running\":%s,\"sta\":\"%s\",\"ap\":\"%s\"}\n",
                     mklink_rtt_is_running() ? "true" : "false", wifi.sta_ip, wifi.ap_ip);
            hub_send_frame(c, 3, (const uint8_t *)st, strlen(st));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        size_t n = 0;
        xSemaphoreTake(c->lock, portMAX_DELAY);
        while (n < sizeof(buf) && c->tail != c->head) {
            buf[n++] = c->ring[c->tail];
            c->tail = (c->tail + 1) % HUB_RING_SIZE;
        }
        xSemaphoreGive(c->lock);

        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        hub_send_frame(c, c->kind, buf, n);
    }

done:
    lock_clients();
    client_reset(c);
    unlock_clients();
    vTaskDelete(NULL);
}

static void hub_accept_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket: %d", errno);
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
        ESP_LOGE(TAG, "bind/listen on %u failed", (unsigned)s_port);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    mklink_log("HUB: stream service listening on %u", (unsigned)s_port);

    while (true) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int sock = accept(listen_sock, (struct sockaddr *)&peer, &peer_len);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        hub_client_t *slot = NULL;
        lock_clients();
        for (int i = 0; i < HUB_MAX_CLIENTS; i++) {
            if (!s_clients[i].used) {
                slot = &s_clients[i];
                slot->used = true;
                slot->sock = sock;
                slot->subscribed = false;
                slot->kind = 0;
                slot->head = slot->tail = 0;
                slot->dropped = 0;
                if (slot->lock == NULL) {
                    slot->lock = xSemaphoreCreateMutex();
                }
                break;
            }
        }
        unlock_clients();

        if (slot == NULL) {
            close(sock);
            continue;
        }
        if (xTaskCreate(hub_client_task, "mk_hub_cli", 4096, slot, 4, NULL) != pdPASS) {
            lock_clients();
            client_reset(slot);
            unlock_clients();
        }
    }
}

esp_err_t mklink_stream_hub_start(uint16_t port)
{
    if (s_clients_lock == NULL) {
        s_clients_lock = xSemaphoreCreateMutex();
    }
    if (s_clients_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_port = (port != 0) ? port : HUB_PORT_DEFAULT;

    if (xTaskCreate(hub_accept_task, "mk_hub", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
