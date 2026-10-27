/*
 * MKLink Remote - RTT consumer.
 *
 * Bytes streamed by the probe are pushed into an 8 KB ring; the UI drains it at
 * its own pace, so a burst of target logging never blocks the USB reader.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "mklink_core.h"
#include "mklink_probe.h"
#include "mklink_rtt.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.rtt";

#define RTT_RING_SIZE 8192

static SemaphoreHandle_t s_lock;
static uint8_t *s_ring;
static size_t s_head;      /* write index */
static size_t s_tail;      /* read index */
static uint32_t s_dropped;
static uint32_t s_total;
static mklink_rtt_status_t s_status;

static void lock_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
    if (s_ring == NULL) {
        s_ring = calloc(1, RTT_RING_SIZE);
    }
}

static void lock(void)
{
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

void mklink_rtt_on_stream(const uint8_t *data, size_t len)
{
    lock_init();
    if (s_ring == NULL) {
        return;
    }

    lock();
    for (size_t i = 0; i < len; i++) {
        size_t next = (s_head + 1) % RTT_RING_SIZE;
        if (next == s_tail) {
            /* drop the oldest byte rather than stalling the USB reader */
            s_tail = (s_tail + 1) % RTT_RING_SIZE;
            s_dropped++;
        }
        s_ring[s_head] = data[i];
        s_head = next;
    }
    s_total += (uint32_t)len;
    s_status.bytes_total = s_total;
    s_status.dropped = s_dropped;
    unlock();
}

size_t mklink_rtt_read(uint8_t *out, size_t cap)
{
    if (s_ring == NULL || out == NULL) {
        return 0;
    }

    size_t n = 0;
    lock();
    while (n < cap && s_tail != s_head) {
        out[n++] = s_ring[s_tail];
        s_tail = (s_tail + 1) % RTT_RING_SIZE;
    }
    unlock();
    return n;
}

static void parse_start_reply(const char *reply)
{
    lock_init();
    lock();
    s_status.up_channels = 0;
    s_status.down_channels = 0;
    s_status.cb_addr[0] = '\0';

    const char *addr = strstr(reply, "SEGGER RTT addr");
    if (addr != NULL) {
        addr = strstr(addr, "0x");
        if (addr != NULL) {
            size_t i = 0;
            while (addr[i] != '\0' && i + 1 < sizeof(s_status.cb_addr) &&
                   ((addr[i] >= '0' && addr[i] <= '9') || (addr[i] >= 'a' && addr[i] <= 'f') ||
                    (addr[i] >= 'A' && addr[i] <= 'F') || addr[i] == 'x')) {
                s_status.cb_addr[i] = addr[i];
                i++;
            }
        }
    }

    for (const char *p = reply; (p = strstr(p, "UpBuffer")) != NULL; p += 8) {
        s_status.up_channels++;
    }
    for (const char *p = reply; (p = strstr(p, "DownBuffer")) != NULL; p += 10) {
        s_status.down_channels++;
    }
    unlock();
}

esp_err_t mklink_rtt_start(uint32_t addr, uint32_t search_size, uint8_t channel)
{
    if (!mklink_probe_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (mklink_rtt_is_running()) {
        return ESP_ERR_INVALID_STATE;
    }

    lock_init();
    lock();
    s_head = 0;
    s_tail = 0;
    s_dropped = 0;
    s_total = 0;
    unlock();

    char cmd[96];
    if (search_size == 0) {
        snprintf(cmd, sizeof(cmd), "RTTView.start(0x%08X, 0, %u)", (unsigned)addr, channel);
    } else {
        snprintf(cmd, sizeof(cmd), "RTTView.start(0x%08X, %u, %u)", (unsigned)addr,
                 (unsigned)search_size, channel);
    }

    /* The probe bridge dispatches stream bytes to mklink_rtt_on_stream(), so no
     * callback registration is needed here. */
    esp_err_t err = mklink_probe_enter_stream(PROBE_STREAM_RTT, cmd, 2500);
    if (err != ESP_OK) {
        mklink_log("RTT: start failed: %s", esp_err_to_name(err));
        return err;
    }

    lock();
    s_status.running = true;
    s_status.bytes_total = 0;
    s_status.dropped = 0;
    unlock();

    mklink_log("RTT: streaming (addr 0x%08X, search %u, ch %u)",
               (unsigned)addr, (unsigned)search_size, channel);
    return ESP_OK;
}

esp_err_t mklink_rtt_stop(void)
{
    if (!mklink_rtt_is_running()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = mklink_probe_exit_stream("RTTView.stop()");

    lock();
    s_status.running = false;
    unlock();

    mklink_log("RTT: stopped (%u bytes, %u dropped)",
               (unsigned)s_status.bytes_total, (unsigned)s_status.dropped);
    return err;
}

esp_err_t mklink_rtt_send(const void *data, size_t len)
{
    if (!mklink_rtt_is_running()) {
        return ESP_ERR_INVALID_STATE;
    }
    return mklink_probe_write_raw(data, len);
}

void mklink_rtt_status(mklink_rtt_status_t *out)
{
    if (out == NULL) {
        return;
    }
    lock_init();
    lock();
    *out = s_status;
    unlock();
}

bool mklink_rtt_is_running(void)
{
    lock_init();
    lock();
    bool running = s_status.running;
    unlock();
    return running;
}
