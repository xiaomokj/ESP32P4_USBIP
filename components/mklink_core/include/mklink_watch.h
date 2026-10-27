/*
 * MKLink Remote - SuperWatch (real-time variable sampling).
 *
 * The probe samples memory for us: `cmd.dump_memory(addr, size, ..., period)`
 * makes it stream binary frames tagged with "MPMDMPMD" at the requested period
 * (default 1 ms). The probe knows nothing about symbols or types, so the PC
 * uploads a variable table (name / address / type) over HTTP and the decoding
 * happens here.
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

#define MKLINK_WATCH_MAX_VARS      24
#define MKLINK_WATCH_MAX_REGIONS   15
#define MKLINK_WATCH_HISTORY       600      /* samples kept per variable */
#define MKLINK_WATCH_CMD_MAX       511      /* Pika REPL line limit */

typedef enum {
    MKLINK_WATCH_U8 = 0,
    MKLINK_WATCH_U16,
    MKLINK_WATCH_U32,
    MKLINK_WATCH_U64,
    MKLINK_WATCH_I8,
    MKLINK_WATCH_I16,
    MKLINK_WATCH_I32,
    MKLINK_WATCH_I64,
    MKLINK_WATCH_F32,
    MKLINK_WATCH_F64,
} mklink_watch_type_t;

typedef struct {
    char name[24];
    uint32_t addr;
    uint8_t type;
    bool valid;
    float value;
    float min;
    float max;
} mklink_watch_var_t;

typedef struct {
    bool running;
    uint32_t frames;
    uint32_t bad_frames;
    uint32_t period_us;
    uint16_t flags;
    uint8_t var_count;
    uint8_t region_count;
} mklink_watch_status_t;

/** Apply a JSON table: {"period_ms":1,"vars":[{"name":"g_temp","addr":"0x20000054","type":"f32"}]} */
esp_err_t mklink_watch_apply_json(const char *json, size_t len);

esp_err_t mklink_watch_start(void);
esp_err_t mklink_watch_stop(void);

/** Copy the current values (and status). @return number of variables copied. */
int mklink_watch_snapshot(mklink_watch_var_t *out, size_t max, mklink_watch_status_t *status);

/** Stream entry called by the probe bridge for PROBE_STREAM_DUMP. */
void mklink_watch_on_stream(const uint8_t *data, size_t len);

/** JSON snapshot of the table (used by GET /watch). */
int mklink_watch_to_json(char *out, size_t cap);

#ifdef __cplusplus
}
#endif
