/*
 * MKLink Remote - SEGGER RTT viewer (host side).
 *
 * The probe firmware owns the RTT implementation: `RTTView.start(addr, size,
 * channel)` makes it scan the target RAM for the "SEGGER RTT" control block and
 * stream channel 0 upwards; the host only consumes bytes and can push bytes back
 * through the down buffer.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool running;
    char cb_addr[16];      /* control block address reported by the probe */
    uint8_t up_channels;
    uint8_t down_channels;
    uint32_t bytes_total;
    uint32_t dropped;      /* bytes thrown away because the ring was full */
} mklink_rtt_status_t;

/** Start streaming RTT. @p addr is the scan start (mode 0), @p search_size 0
 *  selects the fixed-address mode of the probe firmware. */
esp_err_t mklink_rtt_start(uint32_t addr, uint32_t search_size, uint8_t channel);

esp_err_t mklink_rtt_stop(void);

/** Push bytes into the RTT down buffer (no newline is appended). */
esp_err_t mklink_rtt_send(const void *data, size_t len);

/** Consume up to @p cap bytes from the stream; @return bytes written to @p out. */
size_t mklink_rtt_read(uint8_t *out, size_t cap);

void mklink_rtt_status(mklink_rtt_status_t *out);

bool mklink_rtt_is_running(void);

/** Stream callback entry, wired by the probe bridge. */
void mklink_rtt_on_stream(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
