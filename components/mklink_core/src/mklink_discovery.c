/*
 * MKLink Remote - probe auto-discovery.
 *
 * Port of the official Mklink-AI-Probe discovery logic
 * (mklink/discovery.py + mklink/usb_interfaces.py) to the USB host side:
 *
 *   1. Fast path: VID/PID match + a CDC function whose command interface equals
 *      CONFIG_MKLINK_PROBE_CMD_IFACE (MI_04 on official firmware) -> adopt the
 *      device without sending a single byte.
 *   2. Otherwise adopt it temporarily and probe each CDC group like the official
 *      `_probe_port()` does:
 *          write "\n"          -> flush a half-typed command left in the REPL
 *          read  until ">>>"   -> "hello microkeen" means success
 *          write print('__mklink_probe_7f3a__')   -> a response line equal to the
 *                                                    token is required
 *      A bare ">>>" is not enough: a target UART could print the same text.
 *   3. Anything that fails validation is closed again, so a USB stick or a
 *      random CDC device is never mistaken for the probe.
 *
 * The validation runs on the *adopted* device because interface claiming and
 * transfers all operate on the active device handle; mklink_usb_adopt() installs
 * the device without publishing it, mklink_usb_confirm_adopted() publishes it
 * once validated, mklink_usb_discard_adopted() rolls back on failure.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_types_ch9.h"

#include "mklink_core.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.disc";

#define CDC_SET_LINE_CODING        0x20
#define CDC_SET_CONTROL_LINE_STATE 0x22

#define PROBE_IDENTITY_TOKEN "__mklink_probe_7f3a__"
#define PROBE_HELLO_TOKEN    "hello microkeen"
#define PROBE_READ_TIMEOUT   1000   /* usb transfer timeout, ms */
#define PROBE_MAX_ADDRS      8

static SemaphoreHandle_t s_lock;
static volatile bool s_running;
static volatile uint8_t s_pending_addr = MKLINK_USB_ADDR_NONE;

/* remember the last good identity to try it first next time (RAM only) */
static bool s_have_last;
static uint16_t s_last_vid;
static uint16_t s_last_pid;
static uint8_t s_last_cmd_iface;

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

static uint8_t cmd_iface_of(const mklink_cdc_group_t *g)
{
    return (g->comm_iface != 0xFF) ? g->comm_iface : g->data_iface;
}

/* ---------------------------------------------------------------- transport */

static esp_err_t cdc_control(uint8_t addr, uint8_t iface, uint8_t request,
                             uint16_t value, const void *data, size_t len)
{
    usb_setup_packet_t setup = {
        .bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS |
                         USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        .bRequest = request,
        .wValue = value,
        .wIndex = iface,
        .wLength = (uint16_t)len,
    };
    return mklink_cdc_sync_transfer(addr, 0, (uint8_t *)data, len, true, &setup, NULL);
}

static esp_err_t cdc_configure(uint8_t addr, uint8_t iface)
{
    uint8_t line_coding[7] = {
        0x00, 0xC2, 0x01, 0x00, /* 115200 baud (little endian) */
        0x00,                   /* 1 stop bit  */
        0x00,                   /* no parity   */
        0x08,                   /* 8 data bits */
    };
    esp_err_t err = cdc_control(addr, iface, CDC_SET_LINE_CODING, 0, line_coding, sizeof(line_coding));
    if (err != ESP_OK) {
        return err;
    }
    return cdc_control(addr, iface, CDC_SET_CONTROL_LINE_STATE, 0x0003 /* DTR|RTS */, NULL, 0);
}

static esp_err_t cdc_write(uint8_t addr, uint8_t ep, const char *text)
{
    return mklink_cdc_sync_transfer(addr, ep, (uint8_t *)text, strlen(text), false, NULL, NULL);
}

/** One bulk-IN read; returns the number of bytes appended to @p out. */
static size_t cdc_read(uint8_t addr, uint8_t ep, char *out, size_t cap)
{
    uint8_t buf[128];
    size_t got = 0;
    esp_err_t err = mklink_cdc_sync_transfer(addr, ep, buf, sizeof(buf), false, NULL, &got);
    if (err != ESP_OK || got == 0) {
        return 0;
    }

    size_t used = strlen(out);
    size_t room = (cap > used + 1) ? (cap - used - 1) : 0;
    if (got > room) {
        got = room;
    }
    memcpy(out + used, buf, got);
    out[used + got] = '\0';
    return got;
}

static bool contains_ci(const char *haystack, const char *needle)
{
    size_t n = strlen(needle);
    for (const char *p = haystack; *p != '\0'; p++) {
        size_t i = 0;
        while (i < n && p[i] != '\0' && tolower((unsigned char)p[i]) == tolower((unsigned char)needle[i])) {
            i++;
        }
        if (i == n) {
            return true;
        }
    }
    return false;
}

/** @return true when one response line (stripped) equals @p token. */
static bool has_exact_line(const char *response, const char *token)
{
    size_t token_len = strlen(token);
    const char *p = response;

    while (*p != '\0') {
        while (*p == '\r' || *p == '\n' || *p == ' ') {
            p++;
        }
        const char *start = p;
        while (*p != '\0' && *p != '\r' && *p != '\n') {
            p++;
        }
        size_t len = (size_t)(p - start);
        if (len == token_len && strncmp(start, token, token_len) == 0) {
            return true;
        }
    }
    return false;
}

/* ---------------------------------------------------------------- probing */

/** Official `_probe_port()`: newline sync -> hello -> identity token. */
static bool probe_cdc_group(const mklink_cdc_group_t *group, const char **how)
{
    uint8_t addr = mklink_usb_current_addr();
    if (addr == MKLINK_USB_ADDR_NONE) {
        return false;
    }

    bool comm_claimed = false;
    if (group->comm_iface != 0xFF && mklink_usb_claim_interface(group->comm_iface) == ESP_OK) {
        comm_claimed = true;
    }
    if (mklink_usb_claim_interface(group->data_iface) != ESP_OK) {
        if (comm_claimed) {
            mklink_usb_release_interface(group->comm_iface);
        }
        return false;
    }

    uint8_t cfg_iface = cmd_iface_of(group);
    cdc_configure(addr, cfg_iface);

    char rx[256] = {0};
    bool ok = false;

    /* 1) A bare newline terminates whatever half-typed command the REPL still
     *    holds; the official tool relies on this before the identity command. */
    cdc_write(addr, group->ep_out, "\n");
    (void)cdc_read(addr, group->ep_in, rx, sizeof(rx));

    if (contains_ci(rx, PROBE_HELLO_TOKEN)) {
        ok = true;
        *how = "hello";
    } else if (strstr(rx, ">>>") != NULL) {
        /* 2) Identity command: only an exact token line counts. */
        rx[0] = '\0';
        cdc_write(addr, group->ep_out, "print('" PROBE_IDENTITY_TOKEN "')\n");

        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            if (cdc_read(addr, group->ep_in, rx, sizeof(rx)) == 0 && strstr(rx, ">>>") == NULL) {
                continue;
            }
            if (has_exact_line(rx, PROBE_IDENTITY_TOKEN)) {
                ok = true;
                *how = "token";
            } else if (strstr(rx, ">>>") != NULL) {
                break; /* prompt reached without the token -> not a probe */
            }
        }
    } else {
        /* no prompt at all: this group is not a REPL (or target is not running) */
        ESP_LOGD(TAG, "if%u: no prompt (rx %u bytes)", cfg_iface, (unsigned)strlen(rx));
    }

    mklink_usb_release_interface(group->data_iface);
    if (comm_claimed) {
        mklink_usb_release_interface(group->comm_iface);
    }
    return ok;
}

static bool is_probe_id(uint16_t vid, uint16_t pid)
{
    return (CONFIG_MKLINK_PROBE_VID != 0 && vid == CONFIG_MKLINK_PROBE_VID &&
            pid == CONFIG_MKLINK_PROBE_PID);
}

static bool passes_usb_filter(uint16_t vid, uint16_t pid)
{
    if (CONFIG_MKLINK_USB_VID != 0 && vid != CONFIG_MKLINK_USB_VID) {
        return false;
    }
    if (CONFIG_MKLINK_USB_PID != 0 && pid != CONFIG_MKLINK_USB_PID) {
        return false;
    }
    return true;
}

static void remember_success(uint16_t vid, uint16_t pid, uint8_t cmd_iface)
{
    lock();
    s_have_last = true;
    s_last_vid = vid;
    s_last_pid = pid;
    s_last_cmd_iface = cmd_iface;
    unlock();
}

/** @return true when @p device was adopted as the command bridge. */
static bool discovery_pass(uint8_t addr)
{
    if (mklink_usb_busy()) {
        ESP_LOGD(TAG, "addr %u ignored: a device is already active", addr);
        return false;
    }

    mklink_usb_desc_t desc;
    esp_err_t err = mklink_usb_describe(addr, &desc);
    if (err != ESP_OK) {
        mklink_log("PROBE: addr %u describe failed: %s", addr, esp_err_to_name(err));
        return false;
    }

    if (!passes_usb_filter(desc.vid, desc.pid)) {
        mklink_log("PROBE: skip %04X:%04X (usb filter %04X:%04X)",
                   desc.vid, desc.pid, CONFIG_MKLINK_USB_VID, CONFIG_MKLINK_USB_PID);
        mklink_usb_desc_release(&desc);
        return false;
    }

    if (desc.cdc_count == 0) {
        mklink_log("PROBE: %04X:%04X has no CDC function, not a command bridge",
                   desc.vid, desc.pid);
        mklink_usb_desc_release(&desc);
        return false;
    }

    /* ---- fast path (official MI_04 rule) ---- */
    int fast_group = -1;
    for (int i = 0; i < desc.cdc_count; i++) {
        if (cmd_iface_of(&desc.cdc[i]) == CONFIG_MKLINK_PROBE_CMD_IFACE) {
            fast_group = i;
            break;
        }
    }

    if (is_probe_id(desc.vid, desc.pid) && fast_group >= 0) {
        if (mklink_usb_adopt(&desc) == ESP_OK) {
            mklink_usb_confirm_adopted();
            remember_success(desc.vid, desc.pid, CONFIG_MKLINK_PROBE_CMD_IFACE);
            mklink_log("PROBE: %04X:%04X if%d (fast) ready",
                       desc.vid, desc.pid, CONFIG_MKLINK_PROBE_CMD_IFACE);
            return true;
        }
        mklink_usb_desc_release(&desc);
        return false;
    }

    /* ---- identity probing, preferred group first ---- */
    if (mklink_usb_adopt(&desc) != ESP_OK) {
        mklink_usb_desc_release(&desc);
        return false;
    }

    static const char *how = NULL;
    bool ok = false;
    int probed = 0;

    for (int pass = 0; pass < 2 && !ok; pass++) {
        for (int i = 0; i < desc.cdc_count && !ok; i++) {
            bool preferred = (cmd_iface_of(&desc.cdc[i]) == CONFIG_MKLINK_PROBE_CMD_IFACE);
            if ((pass == 0) != preferred) {
                continue;
            }
            probed++;
            how = NULL;
            if (probe_cdc_group(&desc.cdc[i], &how)) {
                ok = true;
                mklink_usb_confirm_adopted();
                remember_success(desc.vid, desc.pid, cmd_iface_of(&desc.cdc[i]));
                mklink_log("PROBE: %04X:%04X if%d (%s) ready",
                           desc.vid, desc.pid, cmd_iface_of(&desc.cdc[i]), how);
            } else {
                mklink_log("PROBE: if%d no identity", cmd_iface_of(&desc.cdc[i]));
            }
        }
    }

    if (!ok) {
        mklink_usb_discard_adopted();
        mklink_log("PROBE: %04X:%04X rejected after %d probe(s)", desc.vid, desc.pid, probed);
    }
    return ok;
}

/* ---------------------------------------------------------------- task ---- */

static void discovery_task(void *arg)
{
    uint8_t addr = (uint8_t)(uintptr_t)arg;

    if (addr != MKLINK_USB_ADDR_NONE) {
        discovery_pass(addr);
    } else {
        uint8_t addrs[PROBE_MAX_ADDRS] = {0};
        int count = mklink_usb_addr_list(addrs, PROBE_MAX_ADDRS);
        if (count == 0) {
            mklink_log("PROBE: no USB device attached");
        }
        for (int i = 0; i < count; i++) {
            if (addrs[i] == 0) {
                continue;
            }
            if (discovery_pass(addrs[i])) {
                break; /* one command bridge is enough */
            }
        }
    }

    /* a device that arrived while we were busy gets its own pass */
    uint8_t pending;
    lock();
    pending = s_pending_addr;
    s_pending_addr = MKLINK_USB_ADDR_NONE;
    s_running = false;
    unlock();

    vTaskDelete(NULL);

    if (pending != MKLINK_USB_ADDR_NONE) {
        mklink_discovery_on_device(pending);
    }
}

static void start_task(uint8_t addr)
{
    lock();
    if (s_running) {
        s_pending_addr = addr;
        unlock();
        return;
    }
    s_running = true;
    unlock();

    if (xTaskCreate(discovery_task, "mk_disc", 4096, (void *)(uintptr_t)addr, 5, NULL) != pdPASS) {
        lock();
        s_running = false;
        unlock();
        ESP_LOGW(TAG, "discovery task creation failed");
    }
}

void mklink_discovery_on_device(uint8_t dev_addr)
{
    start_task(dev_addr);
}

esp_err_t mklink_discovery_rescan(void)
{
    mklink_log("PROBE: rescan requested");
    start_task(MKLINK_USB_ADDR_NONE);
    return ESP_OK;
}

esp_err_t mklink_discovery_start(void)
{
    lock_init();
    return ESP_OK;
}
