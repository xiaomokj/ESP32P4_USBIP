/*
 * MKLink Remote - USB host layer (probe enumeration + descriptor cache)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "mklink_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Install the USB host stack, register a persistent client and enumerate. */
esp_err_t mklink_usb_start(void);

/** Re-enumerate devices that were already attached before the client existed. */
esp_err_t mklink_usb_rescan(void);

/** Latest device snapshot (returns false when no probe is attached). */
bool mklink_usb_get_snapshot(mklink_usb_snapshot_t *out);

/** Number of enumerated devices (several can sit behind a USB hub). */
int mklink_usb_device_count(void);

/** USB address of the @p idx-th device; 0xFF when @p idx is out of range. */
uint8_t mklink_usb_addr_at(int idx);

/** Snapshot of the @p idx-th device (false when out of range / not alive). */
bool mklink_usb_get_snapshot_at(int idx, mklink_usb_snapshot_t *out);

/** Snapshot of the device with @p dev_addr (false when it is gone). */
bool mklink_usb_snapshot_for_addr(uint8_t dev_addr, mklink_usb_snapshot_t *out);

/** Latest CDC loopback result. */
bool mklink_cdc_get_result(mklink_cdc_result_t *out);

/** Run the CDC loopback test again (no-op when no CDC device is present). */
esp_err_t mklink_cdc_retest(void);

/**
 * @brief Copy the human readable descriptor dump into @p dst (NUL terminated).
 * @return Number of bytes written.
 */
size_t mklink_usb_descriptor_dump(char *dst, size_t dst_len);

#ifdef __cplusplus
}
#endif
