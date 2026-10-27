/*
 * MKLink Remote - raw USB transfer layer.
 *
 * Kept free of any protocol logic: it only maps (device address, endpoint,
 * buffer) onto ESP-IDF usb_host transfers. The usbip server will drive it.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

#include "mklink_usb_raw.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.usbraw";

typedef struct {
    usb_transfer_t *transfer;
    mklink_usb_xfer_cb_t cb;
    void *ctx;
    uint8_t *buf;
    size_t len;
    size_t data_offset;
    bool in;
} xfer_wrap_t;

static void transfer_done(usb_transfer_t *transfer)
{
    xfer_wrap_t *wrap = (xfer_wrap_t *)transfer->context;
    esp_err_t err;
    switch (transfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        err = ESP_OK;
        break;
    case USB_TRANSFER_STATUS_CANCELED:
        err = ESP_ERR_INVALID_STATE;
        break;
    case USB_TRANSFER_STATUS_TIMED_OUT:
        err = ESP_ERR_TIMEOUT;
        break;
    default:
        err = ESP_FAIL;
        break;
    }

    size_t actual = 0;
    if (transfer->actual_num_bytes > (int)wrap->data_offset) {
        actual = (size_t)transfer->actual_num_bytes - wrap->data_offset;
    }
    if (wrap->in && actual > 0 && actual <= wrap->len) {
        memcpy(wrap->buf, transfer->data_buffer + wrap->data_offset, actual);
    }

    if (wrap->cb != NULL) {
        wrap->cb(wrap->ctx, err, actual);
    }

    usb_host_transfer_free(transfer);
    free(wrap);
    mklink_usb_xfer_end();
}

static esp_err_t submit_common(uint8_t dev_addr, uint8_t ep_addr, const usb_setup_packet_t *setup,
                              uint8_t *buf, size_t len, uint32_t timeout_ms,
                              mklink_usb_xfer_cb_t cb, void *ctx)
{
    usb_device_handle_t dev_hdl = NULL;
    if (!mklink_usb_device_alive(dev_addr)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!mklink_usb_handle_for_addr(dev_addr, &dev_hdl)) {
        return ESP_ERR_NOT_FOUND;
    }

    xfer_wrap_t *wrap = calloc(1, sizeof(xfer_wrap_t));
    if (wrap == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wrap->cb = cb;
    wrap->ctx = ctx;
    wrap->buf = buf;
    wrap->len = len;
    wrap->data_offset = (setup != NULL) ? sizeof(usb_setup_packet_t) : 0;
    wrap->in = (setup != NULL) ? ((setup->bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0)
                               : ((ep_addr & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) != 0);

    size_t alloc_len = len + wrap->data_offset;
    if (alloc_len < 64) {
        alloc_len = 64; /* keep a sane minimum so short transfers still fit */
    }

    esp_err_t err = usb_host_transfer_alloc(alloc_len, 0, &wrap->transfer);
    if (err != ESP_OK) {
        free(wrap);
        return err;
    }

    usb_transfer_t *t = wrap->transfer;
    t->device_handle = dev_hdl;
    t->bEndpointAddress = ep_addr;
    t->num_bytes = (int)(len + wrap->data_offset);
    t->timeout_ms = timeout_ms;
    t->callback = transfer_done;
    t->context = wrap;
    t->flags = 0;

    if (setup != NULL) {
        memcpy(t->data_buffer, setup, sizeof(usb_setup_packet_t));
    }
    if (!wrap->in && len > 0 && buf != NULL) {
        memcpy(t->data_buffer + wrap->data_offset, buf, len);
    }

    if (setup != NULL) {
        err = usb_host_transfer_submit_control(mklink_usb_client_handle(), t);
    } else {
        err = usb_host_transfer_submit(t);
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "submit ep=0x%02X len=%u failed: %s", ep_addr, (unsigned)len, esp_err_to_name(err));
        usb_host_transfer_free(t);
        free(wrap);
        return err;
    }

    mklink_usb_xfer_begin();
    return ESP_OK;
}

esp_err_t mklink_usb_raw_submit(uint8_t dev_addr, uint8_t ep_addr, uint8_t *buf, size_t len,
                                uint32_t timeout_ms, mklink_usb_xfer_cb_t cb, void *ctx)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return submit_common(dev_addr, ep_addr, NULL, buf, len, timeout_ms, cb, ctx);
}

esp_err_t mklink_usb_raw_control(uint8_t dev_addr, const usb_setup_packet_t *setup,
                                 uint8_t *buf, size_t len, uint32_t timeout_ms,
                                 mklink_usb_xfer_cb_t cb, void *ctx)
{
    if (setup == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > 0 && buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return submit_common(dev_addr, 0x00, setup, buf, len, timeout_ms, cb, ctx);
}

esp_err_t mklink_usb_raw_abort(uint8_t dev_addr, uint8_t ep_addr)
{
    usb_device_handle_t dev_hdl = NULL;
    if (!mklink_usb_handle_for_addr(dev_addr, &dev_hdl)) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = usb_host_endpoint_flush(dev_hdl, ep_addr);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "flush ep=0x%02X: %s", ep_addr, esp_err_to_name(err));
    }
    return err;
}

esp_err_t mklink_usb_raw_unlink_all(uint8_t dev_addr)
{
    /* Flush every endpoint of the device: pending transfers complete as CANCELED
     * and their contexts are released by the completion callback. */
    mklink_ep_info_t eps[16];
    int count = mklink_usb_endpoint_list(eps, 16);
    esp_err_t result = ESP_ERR_NOT_FOUND;

    for (int i = 0; i < count; i++) {
        esp_err_t err = mklink_usb_raw_abort(dev_addr, eps[i].ep_addr);
        if (err == ESP_OK) {
            result = ESP_OK;
        }
    }
    if (count == 0) {
        /* control endpoint is not part of the endpoint table */
        result = mklink_usb_raw_abort(dev_addr, 0x00);
    }
    return result;
}
