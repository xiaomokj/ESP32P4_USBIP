/*
 * MKLink Remote - raw USB transfer API
 *
 * This is the single data plane the future usbip server will reuse: the device is
 * already enumerated and addressed by the local host stack, so the remote vhci
 * client only needs bulk/interrupt/control transfers on top of a live handle.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "usb/usb_types_ch9.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  ep_addr;    /* bEndpointAddress, bit7 = IN */
    uint8_t  attributes; /* bmAttributes & 0x03: 0=CTRL 1=ISO 2=BULK 3=INTR */
    uint16_t mps;        /* wMaxPacketSize */
    uint8_t  interval;
    uint8_t  iface_num;
} mklink_ep_info_t;

/** Completion callback. @p actual_bytes excludes the 8 byte setup packet. */
typedef void (*mklink_usb_xfer_cb_t)(void *ctx, esp_err_t status, size_t actual_bytes);

/**
 * @brief Submit a bulk/interrupt transfer on an already claimed endpoint.
 *
 * Buffers are copied into a PSRAM transfer object; @p buf can be reused by the
 * caller as soon as this function returns. For IN endpoints @p len should be a
 * multiple of the endpoint MPS.
 */
esp_err_t mklink_usb_raw_submit(uint8_t dev_addr, uint8_t ep_addr, uint8_t *buf, size_t len,
                                uint32_t timeout_ms, mklink_usb_xfer_cb_t cb, void *ctx);

/** Submit a control transfer (setup packet + optional data stage). */
esp_err_t mklink_usb_raw_control(uint8_t dev_addr, const usb_setup_packet_t *setup,
                                 uint8_t *buf, size_t len, uint32_t timeout_ms,
                                 mklink_usb_xfer_cb_t cb, void *ctx);

/** Cancel every outstanding transfer of the device (used by usbip UNLINK). */
esp_err_t mklink_usb_raw_unlink_all(uint8_t dev_addr);

#ifdef __cplusplus
}
#endif
