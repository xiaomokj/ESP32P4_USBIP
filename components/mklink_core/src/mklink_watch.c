/*
 * MKLink Remote - SuperWatch consumer.
 *
 * Frame format (verified against the official mklink/dump_memory.py):
 *
 *   magic  "MPMDMPMD"   8 B
 *   ts     microseconds 8 B (LE)
 *   flen   frame length 2 B
 *   rcnt   region count  1 B
 *   then either OLD (flen small):
 *      regions[ idx(1) + size(2 LE) + data ]
 *      flags 2 B, crc32 4 B
 *   or B1 (chunked, total > 2048):
 *      flags 2 B, total 4 B, block_size 2 B (=2048), block_index 2 B,
 *      block_count 2 B, block_crc 4 B, regions[...], crc32 4 B
 *
 * With <=24 scalar variables the payload stays tiny, so OLD frames are what we
 * normally see; B1 chunks are counted and dropped (documented in the README).
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_crc.h"

#include "mklink_core.h"
#include "mklink_probe.h"
#include "mklink_rtt.h"
#include "mklink_watch.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.watch";

#define DUMP_MAGIC          "MPMDMPMD"
#define DUMP_MAGIC_LEN      8
#define OLD_HEADER_LEN      19
#define OLD_TRAILER_LEN     6
#define B1_HEADER_LEN       35
#define B1_TRAILER_LEN      4
#define FLAG_REGION_ERROR   0x0004
#define RX_BUF_SIZE         4096

typedef struct {
    uint32_t addr;
    uint16_t size;
    uint8_t first_var;
    uint8_t var_count;
} region_t;

typedef struct {
    uint8_t var_index;
    uint8_t region_index;
    uint16_t offset;       /* inside the region */
    uint8_t size;
    uint8_t type;
} decode_t;

static SemaphoreHandle_t s_lock;
static mklink_watch_var_t s_vars[MKLINK_WATCH_MAX_VARS];
static region_t s_regions[MKLINK_WATCH_MAX_REGIONS];
static decode_t s_decode[MKLINK_WATCH_MAX_VARS];
static uint8_t s_var_count;
static uint8_t s_region_count;
static uint8_t s_decode_count;
static uint32_t s_period_us = 1000;
static mklink_watch_status_t s_status;

static uint8_t s_rx[RX_BUF_SIZE];
static size_t s_rx_len;
static bool s_b1_warned;

static void lock_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
}

static void lock(void)
{
    lock_init();
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

/* ------------------------------------------------------------------ helpers */

static uint8_t type_size(uint8_t type)
{
    switch (type) {
    case MKLINK_WATCH_U8:
    case MKLINK_WATCH_I8:
        return 1;
    case MKLINK_WATCH_U16:
    case MKLINK_WATCH_I16:
        return 2;
    case MKLINK_WATCH_U32:
    case MKLINK_WATCH_I32:
    case MKLINK_WATCH_F32:
        return 4;
    case MKLINK_WATCH_U64:
    case MKLINK_WATCH_I64:
    case MKLINK_WATCH_F64:
        return 8;
    default:
        return 4;
    }
}

static bool parse_type(const char *text, uint8_t *out)
{
    static const struct {
        const char *name;
        uint8_t type;
    } map[] = {
        {"u8", MKLINK_WATCH_U8},   {"uint8", MKLINK_WATCH_U8},
        {"u16", MKLINK_WATCH_U16}, {"uint16", MKLINK_WATCH_U16},
        {"u32", MKLINK_WATCH_U32}, {"uint32", MKLINK_WATCH_U32},
        {"u64", MKLINK_WATCH_U64}, {"uint64", MKLINK_WATCH_U64},
        {"i8", MKLINK_WATCH_I8},   {"int8", MKLINK_WATCH_I8},
        {"i16", MKLINK_WATCH_I16}, {"int16", MKLINK_WATCH_I16},
        {"i32", MKLINK_WATCH_I32}, {"int32", MKLINK_WATCH_I32},
        {"i64", MKLINK_WATCH_I64}, {"int64", MKLINK_WATCH_I64},
        {"f32", MKLINK_WATCH_F32}, {"float", MKLINK_WATCH_F32},
        {"f64", MKLINK_WATCH_F64}, {"double", MKLINK_WATCH_F64},
    };
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        if (strcasecmp(text, map[i].name) == 0) {
            *out = map[i].type;
            return true;
        }
    }
    return false;
}

static float decode_value(const uint8_t *p, uint8_t type)
{
    switch (type) {
    case MKLINK_WATCH_U8:
        return (float)p[0];
    case MKLINK_WATCH_I8:
        return (float)(int8_t)p[0];
    case MKLINK_WATCH_U16: {
        uint16_t v;
        memcpy(&v, p, 2);
        return (float)v;
    }
    case MKLINK_WATCH_I16: {
        int16_t v;
        memcpy(&v, p, 2);
        return (float)v;
    }
    case MKLINK_WATCH_U32: {
        uint32_t v;
        memcpy(&v, p, 4);
        return (float)v;
    }
    case MKLINK_WATCH_I32: {
        int32_t v;
        memcpy(&v, p, 4);
        return (float)v;
    }
    case MKLINK_WATCH_F32: {
        float v;
        memcpy(&v, p, 4);
        return v;
    }
    case MKLINK_WATCH_U64: {
        uint64_t v;
        memcpy(&v, p, 8);
        return (float)v;
    }
    case MKLINK_WATCH_I64: {
        int64_t v;
        memcpy(&v, p, 8);
        return (float)v;
    }
    case MKLINK_WATCH_F64: {
        double v;
        memcpy(&v, p, 8);
        return (float)v;
    }
    default:
        return 0.0f;
    }
}

/* ------------------------------------------------------------------ table */

static int cmp_var_addr(const void *a, const void *b)
{
    const mklink_watch_var_t *va = (const mklink_watch_var_t *)a;
    const mklink_watch_var_t *vb = (const mklink_watch_var_t *)b;
    return (va->addr > vb->addr) - (va->addr < vb->addr);
}

static esp_err_t build_regions(void)
{
    s_region_count = 0;
    s_decode_count = 0;

    for (uint8_t i = 0; i < s_var_count; i++) {
        const mklink_watch_var_t *var = &s_vars[i];
        uint16_t size = type_size(var->type);

        if (s_region_count > 0) {
            region_t *last = &s_regions[s_region_count - 1];
            /* only merge strictly contiguous addresses (a gap kills throughput) */
            if (last->addr + last->size == var->addr) {
                last->size = (uint16_t)(last->size + size);
                last->var_count++;
                s_decode[s_decode_count++] = (decode_t){
                    .var_index = i,
                    .region_index = (uint8_t)(s_region_count - 1),
                    .offset = (uint16_t)(last->size - size),
                    .size = (uint8_t)size,
                    .type = var->type,
                };
                continue;
            }
        }

        if (s_region_count >= MKLINK_WATCH_MAX_REGIONS) {
            mklink_log("WATCH: too many regions (max %d)", MKLINK_WATCH_MAX_REGIONS);
            return ESP_ERR_INVALID_SIZE;
        }

        s_regions[s_region_count] = (region_t){
            .addr = var->addr,
            .size = size,
            .first_var = i,
            .var_count = 1,
        };
        s_decode[s_decode_count++] = (decode_t){
            .var_index = i,
            .region_index = s_region_count,
            .offset = 0,
            .size = (uint8_t)size,
            .type = var->type,
        };
        s_region_count++;
    }
    return ESP_OK;
}

static esp_err_t build_command(char *out, size_t cap, float period_s)
{
    size_t used = 0;
    int n = snprintf(out, cap, "cmd.dump_memory(");
    if (n <= 0) {
        return ESP_FAIL;
    }
    used = (size_t)n;

    for (uint8_t i = 0; i < s_region_count; i++) {
        n = snprintf(out + used, cap - used, "%s0x%08X, %u",
                     (i == 0) ? "" : ", ", (unsigned)s_regions[i].addr,
                     (unsigned)s_regions[i].size);
        if (n <= 0 || used + (size_t)n >= cap) {
            return ESP_ERR_INVALID_SIZE;
        }
        used += (size_t)n;
    }

    n = snprintf(out + used, cap - used, ", %g)", (double)period_s);
    if (n <= 0 || used + (size_t)n >= cap) {
        return ESP_ERR_INVALID_SIZE;
    }
    used += (size_t)n;

    if (used > MKLINK_WATCH_CMD_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t mklink_watch_apply_json(const char *json, size_t len)
{
    if (json == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool was_running = false;
    lock_init();
    lock();
    was_running = s_status.running;
    memset(s_vars, 0, sizeof(s_vars));
    s_var_count = 0;
    unlock();

    if (was_running) {
        mklink_watch_stop();
    }

    cJSON *period = cJSON_GetObjectItem(root, "period_ms");
    uint32_t period_ms = cJSON_IsNumber(period) ? (uint32_t)period->valuedouble : 1;
    if (period_ms == 0) {
        period_ms = 1;
    }

    cJSON *vars = cJSON_GetObjectItem(root, "vars");
    esp_err_t result = ESP_OK;

    if (!cJSON_IsArray(vars)) {
        result = ESP_ERR_INVALID_ARG;
    } else {
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, vars) {
            if (s_var_count >= MKLINK_WATCH_MAX_VARS) {
                mklink_log("WATCH: table truncated at %d variables", MKLINK_WATCH_MAX_VARS);
                break;
            }

            cJSON *name = cJSON_GetObjectItem(item, "name");
            cJSON *addr = cJSON_GetObjectItem(item, "addr");
            cJSON *type = cJSON_GetObjectItem(item, "type");
            if (!cJSON_IsString(name) || addr == NULL || !cJSON_IsString(type)) {
                continue;
            }

            uint32_t address = 0;
            if (cJSON_IsString(addr)) {
                address = (uint32_t)strtoul(addr->valuestring, NULL, 0);
            } else if (cJSON_IsNumber(addr)) {
                address = (uint32_t)addr->valuedouble;
            } else {
                continue;
            }

            uint8_t type_id = 0;
            if (!parse_type(type->valuestring, &type_id)) {
                mklink_log("WATCH: unknown type '%s' for %s", type->valuestring, name->valuestring);
                continue;
            }

            mklink_watch_var_t *var = &s_vars[s_var_count];
            strlcpy(var->name, name->valuestring, sizeof(var->name));
            var->addr = address;
            var->type = type_id;
            var->valid = false;
            var->min = 0.0f;
            var->max = 0.0f;
            s_var_count++;
        }

        if (s_var_count == 0) {
            result = ESP_ERR_NOT_FOUND;
        } else {
            lock();
            qsort(s_vars, s_var_count, sizeof(s_vars[0]), cmp_var_addr);
            s_period_us = period_ms * 1000;
            result = build_regions();
            if (result == ESP_OK) {
                s_status.var_count = s_var_count;
                s_status.region_count = s_region_count;
                s_status.period_us = s_period_us;
                s_status.frames = 0;
                s_status.bad_frames = 0;
                s_status.flags = 0;
            }
            unlock();
        }
    }

    cJSON_Delete(root);

    if (result == ESP_OK) {
        mklink_log("WATCH: table %u vars / %u regions / %u ms",
                   (unsigned)s_var_count, (unsigned)s_region_count, (unsigned)period_ms);
    }
    return result;
}

int mklink_watch_to_json(char *out, size_t cap)
{
    if (out == NULL || cap == 0) {
        return 0;
    }

    lock();
    int n = snprintf(out, cap,
                     "{\"running\":%s,\"frames\":%u,\"bad\":%u,\"period_us\":%u,"
                     "\"flags\":%u,\"vars\":[",
                     s_status.running ? "true" : "false", (unsigned)s_status.frames,
                     (unsigned)s_status.bad_frames, (unsigned)s_status.period_us,
                     (unsigned)s_status.flags);
    for (uint8_t i = 0; i < s_var_count && n > 0 && (size_t)n < cap; i++) {
        n += snprintf(out + n, cap - (size_t)n,
                      "%s{\"name\":\"%s\",\"addr\":\"0x%08X\",\"value\":%.4f}",
                      (i == 0) ? "" : ",", s_vars[i].name, (unsigned)s_vars[i].addr,
                      (double)s_vars[i].value);
    }
    if (n > 0 && (size_t)n < cap) {
        n += snprintf(out + n, cap - (size_t)n, "]}");
    }
    unlock();
    return n;
}

/* ------------------------------------------------------------------ frames */

static void update_vars(const uint8_t *payload, uint16_t frame_len, uint8_t region_count)
{
    size_t offset = 0;
    for (uint8_t i = 0; i < region_count; i++) {
        if (offset + 3 > frame_len) {
            break;
        }
        uint8_t index = payload[offset];
        uint16_t size = (uint16_t)(payload[offset + 1] | (payload[offset + 2] << 8));
        offset += 3;
        if (offset + size > frame_len) {
            break;
        }

        for (uint8_t d = 0; d < s_decode_count; d++) {
            const decode_t *dec = &s_decode[d];
            if (dec->region_index != index) {
                continue;
            }
            if ((uint32_t)dec->offset + dec->size > size) {
                continue;
            }
            mklink_watch_var_t *var = &s_vars[dec->var_index];
            float value = decode_value(payload + offset + dec->offset, dec->type);
            if (!var->valid) {
                var->min = value;
                var->max = value;
                var->valid = true;
            } else {
                if (value < var->min) {
                    var->min = value;
                }
                if (value > var->max) {
                    var->max = value;
                }
            }
            var->value = value;
        }

        offset += size;
    }
}

static void parse_rx(void)
{
    size_t pos = 0;

    while (s_rx_len >= OLD_HEADER_LEN) {
        /* resync on the magic */
        if (memcmp(s_rx + pos, DUMP_MAGIC, DUMP_MAGIC_LEN) != 0) {
            pos++;
            continue;
        }
        if (s_rx_len - pos < OLD_HEADER_LEN) {
            break;
        }

        const uint8_t *frame = s_rx + pos;
        uint16_t frame_len = (uint16_t)(frame[16] | (frame[17] << 8));
        uint8_t region_count = frame[18];

        if (frame_len < OLD_HEADER_LEN + OLD_TRAILER_LEN || s_rx_len - pos < frame_len) {
            break;   /* wait for the rest */
        }

        /* B1 chunk heuristic: block_size fields only exist in chunked frames */
        bool b1 = false;
        if (frame_len >= B1_HEADER_LEN + B1_TRAILER_LEN) {
            uint16_t block_size = (uint16_t)(frame[23] | (frame[24] << 8));
            uint16_t block_index = (uint16_t)(frame[25] | (frame[26] << 8));
            uint16_t block_count = (uint16_t)(frame[27] | (frame[28] << 8));
            b1 = (block_size == 2048 && block_count > 0 && block_index < block_count);
        }

        lock();
        s_status.frames++;
        if (b1) {
            s_status.flags |= 0x0100;   /* our own "chunked frame seen" marker */
            if (!s_b1_warned) {
                s_b1_warned = true;
                mklink_log("WATCH: chunked (B1) frames are skipped, reduce the variable set");
            }
        } else {
            uint16_t flags = (uint16_t)(frame[frame_len - 6] | (frame[frame_len - 5] << 8));
            s_status.flags = flags;
            if (flags & FLAG_REGION_ERROR) {
                s_status.bad_frames++;
            } else {
                update_vars(frame + OLD_HEADER_LEN,
                            (uint16_t)(frame_len - OLD_HEADER_LEN - OLD_TRAILER_LEN),
                            region_count);
            }
        }
        unlock();

        pos += frame_len;
    }

    if (pos > 0) {
        memmove(s_rx, s_rx + pos, s_rx_len - pos);
        s_rx_len -= pos;
    }
}

void mklink_watch_on_stream(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    while (len > 0) {
        size_t room = RX_BUF_SIZE - s_rx_len;
        if (room == 0) {
            /* the parser is behind; drop the buffer rather than block USB */
            lock();
            s_status.bad_frames++;
            unlock();
            s_rx_len = 0;
            room = RX_BUF_SIZE;
        }
        size_t chunk = (len < room) ? len : room;
        memcpy(s_rx + s_rx_len, data, chunk);
        s_rx_len += chunk;
        data += chunk;
        len -= chunk;
        parse_rx();
    }
}

/* ------------------------------------------------------------------ control */

esp_err_t mklink_watch_start(void)
{
    if (!mklink_probe_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_var_count == 0 || s_region_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (mklink_rtt_is_running()) {
        mklink_rtt_stop();
    }

    char cmd[MKLINK_WATCH_CMD_MAX + 16];
    esp_err_t err = build_command(cmd, sizeof(cmd), (float)s_period_us / 1000000.0f);
    if (err != ESP_OK) {
        mklink_log("WATCH: command too long, reduce the variable set");
        return err;
    }

    s_rx_len = 0;
    s_b1_warned = false;

    err = mklink_probe_enter_stream(PROBE_STREAM_DUMP, cmd, 3000);
    if (err != ESP_OK) {
        mklink_log("WATCH: start failed: %s", esp_err_to_name(err));
        return err;
    }

    lock();
    s_status.running = true;
    s_status.frames = 0;
    s_status.bad_frames = 0;
    unlock();

    mklink_log("WATCH: sampling %u vars in %u regions", (unsigned)s_var_count, (unsigned)s_region_count);
    return ESP_OK;
}

esp_err_t mklink_watch_stop(void)
{
    if (!s_status.running) {
        return ESP_ERR_INVALID_STATE;
    }

    char cmd[MKLINK_WATCH_CMD_MAX + 16];
    esp_err_t err = build_command(cmd, sizeof(cmd), -1.0f);
    if (err != ESP_OK) {
        /* stop with a single region and period -1 still terminates the probe */
        snprintf(cmd, sizeof(cmd), "cmd.dump_memory(0x%08X, %u, -1)",
                 (unsigned)s_regions[0].addr, (unsigned)s_regions[0].size);
    }

    err = mklink_probe_exit_stream(cmd);

    lock();
    s_status.running = false;
    unlock();

    mklink_log("WATCH: stopped after %u frames (%u bad)",
               (unsigned)s_status.frames, (unsigned)s_status.bad_frames);
    return err;
}

int mklink_watch_snapshot(mklink_watch_var_t *out, size_t max, mklink_watch_status_t *status)
{
    lock_init();
    lock();
    int n = 0;
    for (uint8_t i = 0; i < s_var_count && (size_t)i < max; i++) {
        out[n++] = s_vars[i];
    }
    if (status != NULL) {
        *status = s_status;
    }
    unlock();
    return n;
}
