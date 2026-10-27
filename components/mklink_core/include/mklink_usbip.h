/*
 * MKLink Remote - usbip server public API.
 *
 * Exports the enumerated probe as a native USB device to a remote computer
 * over the standard Linux usbip protocol (TCP port 3240). On the PC this is a
 * plain `usbip attach -r <IP> -b <busid>`, after which the probe shows up as a
 * local USB device and Keil / pyOCD / CMSIS-DAP tools work unchanged.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start the usbip server. @p port == 0 selects the default (3240). */
esp_err_t mklink_usbip_start(uint16_t port);

/** Stop the server and drop any active client. */
esp_err_t mklink_usbip_stop(void);

/** @return true while the server is listening. */
bool mklink_usbip_running(void);

/** @return the TCP port in use (0 when not running). */
uint16_t mklink_usbip_port(void);

#ifdef __cplusplus
}
#endif
