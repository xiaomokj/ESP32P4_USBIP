/*
 * MKLink Remote - probe command bridge.
 *
 * The MKLink probe implements RTT and memory sampling in *its* firmware; the
 * host side just speaks a Python REPL over the probe's CDC command interface:
 *
 *   - 115200 8N1, UTF-8 text, one command per line ("\n")
 *   - a command is done when the ">>>" prompt arrives (it can be split across
 *     USB packets, so the tail is always kept)
 *   - a command that starts a stream (RTTView.start / cmd.dump_memory) may send
 *     its closing prompt and the first stream bytes in the *same* read, hence the
 *     "arm before write, split at the prompt, then hand over" dance below
 *   - while streaming the prompt must NOT be searched for (stream data can
 *     legitimately contain ">>>")
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

typedef enum {
    PROBE_STREAM_NONE = 0,
    PROBE_STREAM_RTT,
    PROBE_STREAM_DUMP,
} probe_stream_t;

typedef void (*mklink_probe_stream_cb_t)(probe_stream_t kind, const uint8_t *data, size_t len);

/** @return true when the command interface is claimed and the REPL answered. */
bool mklink_probe_ready(void);

/** Command interface number currently in use (0xFF when not attached). */
uint8_t mklink_probe_cmd_iface(void);

/** Run a REPL command and copy the response (prompt stripped) into @p reply. */
esp_err_t mklink_probe_cmd(const char *cmd, char *reply, size_t cap, uint32_t timeout_ms);

/** Arm the stream transition, then send @p cmd. Any bytes after the terminal
 *  prompt are delivered to the stream callback. */
esp_err_t mklink_probe_enter_stream(probe_stream_t kind, const char *cmd, uint32_t timeout_ms);

/** Leave the current stream: send @p stop_cmd (optional), then resynchronise the
 *  REPL with the identity command so the next command starts from a clean state. */
esp_err_t mklink_probe_exit_stream(const char *stop_cmd);

/** Write raw bytes into the stream (used for the RTT down buffer). */
esp_err_t mklink_probe_write_raw(const void *data, size_t len);

void mklink_probe_set_stream_cb(mklink_probe_stream_cb_t cb);

/** Called by the USB bridge when a device was adopted / removed. */
void mklink_probe_attach(void);
void mklink_probe_detach(void);

#ifdef __cplusplus
}
#endif
