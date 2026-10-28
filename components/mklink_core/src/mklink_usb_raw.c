/*
 * MKLink Remote - raw USB transfer layer on top of CherryUSB URBs.
 *
 * Kept free of any protocol logic: it only maps (device address, endpoint,
 * buffer) onto CherryUSB URBs and reports completion through the callback the
 * usbip server / probe bridge installed.
 *
 * Two things make this layer more than a thin wrapper:
 *
 *  - CherryUSB completes URBs from the USB interrupt, and its dwc2 port frees
 *    the host channel before invoking the completion callback. The caller's
 *    callback must therefore never run in that context: completions are pushed
 *    to a queue and dispatched by a worker task, which also owns the buffer
 *    copies and the memory release.
 *  - `usbh_submit_urb()` blocks when `urb->timeout != 0`, so every asynchronous
 *    transfer is submitted with timeout 0 and the usbip timeout is implemented
 *    with a one-shot esp_timer that kills the URB.
 *
 * Control transfers are serviced by a second worker: the device has a single
 * EP0 URB, so two concurrent control requests would corrupt its state machine.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "usbh_core.h"
#include "usb/usb_types_ch9.h"

#include "mklink_usb_raw.h"
#include "mklink_priv.h"

static const char *TAG = "mklink.usbraw";

#define MK_XFER_QUEUE_LEN   64
#define MK_CTRL_QUEUE_LEN   8
#define MK_ACTIVE_MAX       32

/* DWC2 + dcache: every DMA buffer (setup packet included) must be aligned to
 * the L1 cache line, otherwise usbh_submit_urb() trips a hard assert. */
#define MK_DMA_ALIGN        CONFIG_USB_ALIGN_SIZE

typedef struct {
    struct usbh_urb urb;
    struct usbh_hubport *hport;
    uint8_t *xfer_buf;                  /* DMA safe copy */
    uint8_t *user_buf;
    size_t   len;
    bool     in;
    bool     control;
    uint8_t  dev_addr;
    uint8_t  ep_addr;
    volatile int nbytes;                /* completion argument */
    mklink_usb_xfer_cb_t cb;
    void    *ctx;
    esp_timer_handle_t timer;
    bool     timer_armed;
} xfer_wrap_t;

typedef struct {
    /* Keep the setup packet first so a MK_DMA_ALIGN aligned allocation makes it
     * DMA safe as well. */
    struct usb_setup_packet setup __attribute__((aligned(MK_DMA_ALIGN)));
    struct usbh_hubport *hport;
    uint8_t *user_buf;
    size_t   len;
    bool     in;
    uint8_t  dev_addr;
    bool     canceled;
    mklink_usb_xfer_cb_t cb;
    void    *ctx;
} ctrl_job_t;

static QueueHandle_t s_done_q;
static QueueHandle_t s_ctrl_q;
static SemaphoreHandle_t s_lock;          /* guards s_active / s_ctrl_jobs */
static xfer_wrap_t *s_active[MK_ACTIVE_MAX];
static ctrl_job_t *s_ctrl_jobs[MK_CTRL_QUEUE_LEN];
static bool s_ready;

/* ------------------------------------------------------------------ helpers */

static esp_err_t map_status(int nbytes)
{
    if (nbytes >= 0) {
        return ESP_OK;
    }
    switch (-nbytes) {
    case USB_ERR_TIMEOUT:
        return ESP_ERR_TIMEOUT;
    case USB_ERR_SHUTDOWN:
        return ESP_ERR_INVALID_STATE;
    case USB_ERR_STALL:
        /* usbip maps this to -EPIPE, which is what a stalled pipe must report. */
        return ESP_ERR_NOT_SUPPORTED;
    case USB_ERR_NOMEM:
        return ESP_ERR_NO_MEM;
    case USB_ERR_INVAL:
        return ESP_ERR_INVALID_ARG;
    default:
        return ESP_FAIL;
    }
}

static uint8_t *xfer_alloc(size_t len)
{
    if (len < MK_DMA_ALIGN) {
        len = MK_DMA_ALIGN;   /* DWC2 needs a DMA pointer even for zero length stages */
    }
    /* The controller cleans/invalidates whole cache lines, so the allocation has
     * to cover every line it may touch. */
    len = (len + MK_DMA_ALIGN - 1) & ~(size_t)(MK_DMA_ALIGN - 1);
    uint8_t *p = heap_caps_aligned_alloc(MK_DMA_ALIGN, len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p == NULL) {
        p = heap_caps_aligned_alloc(MK_DMA_ALIGN, len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    return p;
}

static struct usb_endpoint_descriptor *find_ep(struct usbh_hubport *hport, uint8_t ep_addr, bool *is_intr)
{
    const struct usb_configuration_descriptor *cd = &hport->config.config_desc;

    for (uint8_t i = 0; i < cd->bNumInterfaces && i < CONFIG_USBHOST_MAX_INTERFACES; i++) {
        struct usbh_interface *intf = &hport->config.intf[i];
        for (uint8_t a = 0; a < intf->altsetting_num && a < CONFIG_USBHOST_MAX_INTF_ALTSETTINGS; a++) {
            struct usbh_interface_altsetting *as = &intf->altsetting[a];
            for (uint8_t e = 0; e < as->intf_desc.bNumEndpoints && e < CONFIG_USBHOST_MAX_ENDPOINTS; e++) {
                struct usb_endpoint_descriptor *ed = &as->ep[e].ep_desc;
                if (ed->bEndpointAddress == ep_addr) {
                    if (is_intr != NULL) {
                        *is_intr = ((ed->bmAttributes & 0x03) == USB_ENDPOINT_TYPE_INTERRUPT);
                    }
                    return ed;
                }
            }
        }
    }
    return NULL;
}

static void active_add(xfer_wrap_t *w)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MK_ACTIVE_MAX; i++) {
        if (s_active[i] == NULL) {
            s_active[i] = w;
            break;
        }
    }
    xSemaphoreGive(s_lock);
}

static void active_remove(xfer_wrap_t *w)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MK_ACTIVE_MAX; i++) {
        if (s_active[i] == w) {
            s_active[i] = NULL;
            break;
        }
    }
    xSemaphoreGive(s_lock);
}

/* --------------------------------------------------------------- completion */

static void xfer_finish(xfer_wrap_t *w)
{
    int nbytes = w->nbytes;

    if (w->timer != NULL) {
        if (w->timer_armed) {
            esp_timer_stop(w->timer);
        }
        esp_timer_delete(w->timer);
        w->timer = NULL;
    }

    active_remove(w);

    size_t actual = 0;
    if (nbytes > 0) {
        actual = (size_t)nbytes;
        if (w->in && w->user_buf != NULL && w->xfer_buf != NULL) {
            size_t copy = actual < w->len ? actual : w->len;
            memcpy(w->user_buf, w->xfer_buf, copy);
            actual = copy;
        }
    }

    esp_err_t status = map_status(nbytes);
    if (nbytes < 0) {
        ESP_LOGD(TAG, "dev %u ep 0x%02X failed: %d", w->dev_addr, w->ep_addr, nbytes);
    }

    mklink_usb_xfer_cb_t cb = w->cb;
    void *ctx = w->ctx;
    uint8_t dev_addr = w->dev_addr;
    uint8_t ep_addr = w->ep_addr;

    if (w->xfer_buf != NULL) {
        free(w->xfer_buf);
        w->xfer_buf = NULL;
    }
    free(w);

    if (cb != NULL) {
        cb(ctx, status, actual);
    } else {
        ESP_LOGD(TAG, "dev %u ep 0x%02X completed without callback", dev_addr, ep_addr);
    }

    /* Counted after the callback so mklink_usb_raw_unlink_all() can wait until
     * the owner has released (or accepted) its transfer context. */
    mklink_usb_xfer_end();
}

static void xfer_done_cb(void *arg, int nbytes)
{
    xfer_wrap_t *w = (xfer_wrap_t *)arg;
    w->nbytes = nbytes;

    if (xPortInIsrContext()) {
        BaseType_t higher = pdFALSE;
        if (xQueueSendFromISR(s_done_q, &w, &higher) != pdTRUE) {
            ESP_DRAM_LOGE(TAG, "completion queue full");
        }
        if (higher == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    } else {
        /* usbh_kill_urb() reports synchronously from the caller's task. The queue
         * is deeper than MK_ACTIVE_MAX, so this send never blocks (and therefore
         * never deadlocks against a caller holding s_lock). */
        if (xQueueSend(s_done_q, &w, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "completion queue full (task context)");
        }
    }
}

static void xfer_timeout_cb(void *arg)
{
    xfer_wrap_t *w = (xfer_wrap_t *)arg;

    ESP_LOGD(TAG, "dev %u ep 0x%02X timeout", w->dev_addr, w->ep_addr);
    /* Completes the URB through xfer_done_cb() (already-finished URBs report
     * -USB_ERR_INVAL and are ignored). */
    (void)usbh_kill_urb(&w->urb);
}

static void done_task(void *arg)
{
    xfer_wrap_t *w;
    while (xQueueReceive(s_done_q, &w, portMAX_DELAY) == pdTRUE) {
        xfer_finish(w);
    }
}

/* ---------------------------------------------------------- control worker */

static void ctrl_job_free(ctrl_job_t *job)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MK_CTRL_QUEUE_LEN; i++) {
        if (s_ctrl_jobs[i] == job) {
            s_ctrl_jobs[i] = NULL;
            break;
        }
    }
    xSemaphoreGive(s_lock);
    free(job);
}

static void ctrl_task(void *arg)
{
    ctrl_job_t *job;
    while (xQueueReceive(s_ctrl_q, &job, portMAX_DELAY) == pdTRUE) {
        esp_err_t status = ESP_OK;
        size_t actual = 0;
        uint8_t *buf = NULL;

        if (job->canceled || !mklink_usb_device_alive(job->dev_addr)) {
            status = ESP_ERR_INVALID_STATE;
            goto report;
        }

        if (job->len > 0) {
            buf = xfer_alloc(job->len);
            if (buf == NULL) {
                status = ESP_ERR_NO_MEM;
                goto report;
            }
            if (!job->in && job->user_buf != NULL) {
                memcpy(buf, job->user_buf, job->len);
            }
        }

        int ret = usbh_control_transfer(job->hport, &job->setup, buf);
        status = map_status(ret);
        if (ret > 0) {
            actual = (size_t)ret;
            if (job->in && buf != NULL && job->user_buf != NULL) {
                size_t copy = actual < job->len ? actual : job->len;
                memcpy(job->user_buf, buf, copy);
                actual = copy;
            }
        }

report:
        if (buf != NULL) {
            free(buf);
        }
        mklink_usb_xfer_cb_t cb = job->cb;
        void *ctx = job->ctx;
        ctrl_job_free(job);
        if (cb != NULL) {
            cb(ctx, status, actual);
        }
        mklink_usb_xfer_end();
    }
}

/* ------------------------------------------------------------------- public */

esp_err_t mklink_usb_raw_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    s_done_q = xQueueCreate(MK_XFER_QUEUE_LEN, sizeof(xfer_wrap_t *));
    s_ctrl_q = xQueueCreate(MK_CTRL_QUEUE_LEN, sizeof(ctrl_job_t *));
    if (s_lock == NULL || s_done_q == NULL || s_ctrl_q == NULL) {
        ESP_LOGE(TAG, "init failed");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(done_task, "mk_usb_done", 3072, NULL, 6, NULL) != pdPASS ||
        xTaskCreate(ctrl_task, "mk_usb_ctrl", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task creation failed");
        return ESP_ERR_NO_MEM;
    }

    s_ready = true;
    return ESP_OK;
}

esp_err_t mklink_usb_raw_submit(uint8_t dev_addr, uint8_t ep_addr, uint8_t *buf, size_t len,
                                uint32_t timeout_ms, mklink_usb_xfer_cb_t cb, void *ctx)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mklink_usb_device_alive(dev_addr)) {
        return ESP_ERR_INVALID_STATE;
    }

    struct usbh_hubport *hport = NULL;
    if (!mklink_usb_handle_for_addr(dev_addr, &hport) || hport == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    bool is_intr = false;
    struct usb_endpoint_descriptor *ep = find_ep(hport, ep_addr, &is_intr);
    if (ep == NULL) {
        ESP_LOGW(TAG, "dev %u: no endpoint 0x%02X", dev_addr, ep_addr);
        return ESP_ERR_NOT_FOUND;
    }

    xfer_wrap_t *w = calloc(1, sizeof(*w));
    if (w == NULL) {
        return ESP_ERR_NO_MEM;
    }

    w->xfer_buf = xfer_alloc(len);
    if (w->xfer_buf == NULL) {
        free(w);
        return ESP_ERR_NO_MEM;
    }

    w->hport = hport;
    w->user_buf = buf;
    w->len = len;
    w->in = (ep_addr & 0x80) != 0;
    w->control = false;
    w->dev_addr = dev_addr;
    w->ep_addr = ep_addr;
    w->cb = cb;
    w->ctx = ctx;
    w->nbytes = -USB_ERR_SHUTDOWN;

    if (!w->in) {
        memcpy(w->xfer_buf, buf, len);
    }

    if (is_intr) {
        usbh_int_urb_fill(&w->urb, hport, ep, w->xfer_buf, (uint32_t)len, 0, xfer_done_cb, w);
    } else {
        usbh_bulk_urb_fill(&w->urb, hport, ep, w->xfer_buf, (uint32_t)len, 0, xfer_done_cb, w);
    }

    if (timeout_ms > 0) {
        esp_timer_create_args_t args = {
            .callback = xfer_timeout_cb,
            .arg = w,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "mk_urb_tmo",
        };
        if (esp_timer_create(&args, &w->timer) == ESP_OK) {
            w->timer_armed = (esp_timer_start_once(w->timer, (uint64_t)timeout_ms * 1000) == ESP_OK);
        } else {
            w->timer = NULL;
        }
    }

    active_add(w);
    mklink_usb_xfer_begin();

    int ret = usbh_submit_urb(&w->urb);
    if (ret < 0) {
        ESP_LOGW(TAG, "dev %u ep 0x%02X submit failed: %d", dev_addr, ep_addr, ret);
        active_remove(w);
        if (w->timer != NULL) {
            if (w->timer_armed) {
                esp_timer_stop(w->timer);
            }
            esp_timer_delete(w->timer);
        }
        free(w->xfer_buf);
        free(w);
        mklink_usb_xfer_end();
        return map_status(ret);
    }

    return ESP_OK;
}

esp_err_t mklink_usb_raw_control(uint8_t dev_addr, const usb_setup_packet_t *setup,
                                 uint8_t *buf, size_t len, uint32_t timeout_ms,
                                 mklink_usb_xfer_cb_t cb, void *ctx)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (setup == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > 0 && buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mklink_usb_device_alive(dev_addr)) {
        return ESP_ERR_INVALID_STATE;
    }

    struct usbh_hubport *hport = NULL;
    if (!mklink_usb_handle_for_addr(dev_addr, &hport) || hport == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    ctrl_job_t *job = heap_caps_aligned_alloc(MK_DMA_ALIGN, sizeof(*job),
                                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (job == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(job, 0, sizeof(*job));

    /* Same 8-byte layout as struct usb_setup_packet, all fields little endian. */
    memcpy(&job->setup, setup, sizeof(job->setup));
    job->hport = hport;
    job->user_buf = buf;
    job->len = len;
    job->in = (setup->bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0;
    job->dev_addr = dev_addr;
    job->cb = cb;
    job->ctx = ctx;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MK_CTRL_QUEUE_LEN; i++) {
        if (s_ctrl_jobs[i] == NULL) {
            s_ctrl_jobs[i] = job;
            slot = i;
            break;
        }
    }
    xSemaphoreGive(s_lock);

    if (slot < 0) {
        free(job);
        return ESP_ERR_NO_MEM;
    }

    /* The worker serializes EP0, so the per-transfer timeout is applied inside
     * usbh_control_transfer() (CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT). */
    (void)timeout_ms;

    if (xQueueSend(s_ctrl_q, &job, pdMS_TO_TICKS(50)) != pdTRUE) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_ctrl_jobs[slot] = NULL;
        xSemaphoreGive(s_lock);
        free(job);
        return ESP_ERR_NO_MEM;
    }

    mklink_usb_xfer_begin();
    return ESP_OK;
}

esp_err_t mklink_usb_raw_abort(uint8_t dev_addr, uint8_t ep_addr)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = ESP_ERR_NOT_FOUND;

    if (ep_addr == 0x00) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (int i = 0; i < MK_CTRL_QUEUE_LEN; i++) {
            if (s_ctrl_jobs[i] != NULL && s_ctrl_jobs[i]->dev_addr == dev_addr) {
                s_ctrl_jobs[i]->canceled = true;
                result = ESP_OK;
            }
        }
        xSemaphoreGive(s_lock);
        return result;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MK_ACTIVE_MAX; i++) {
        xfer_wrap_t *w = s_active[i];
        if (w != NULL && w->dev_addr == dev_addr && w->ep_addr == ep_addr) {
            if (usbh_kill_urb(&w->urb) == 0) {
                result = ESP_OK;
            }
        }
    }
    xSemaphoreGive(s_lock);

    return result;
}

void mklink_usb_raw_kill_all(uint8_t dev_addr)
{
    if (!s_ready) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MK_CTRL_QUEUE_LEN; i++) {
        if (s_ctrl_jobs[i] != NULL && s_ctrl_jobs[i]->dev_addr == dev_addr) {
            s_ctrl_jobs[i]->canceled = true;
        }
    }
    for (int i = 0; i < MK_ACTIVE_MAX; i++) {
        xfer_wrap_t *w = s_active[i];
        if (w != NULL && w->dev_addr == dev_addr) {
            (void)usbh_kill_urb(&w->urb);
        }
    }
    xSemaphoreGive(s_lock);
}

esp_err_t mklink_usb_raw_unlink_all(uint8_t dev_addr)
{
    mklink_usb_raw_kill_all(dev_addr);

    /* Completions are dispatched by the worker tasks: the caller (the usbip
     * connection) must not free its transfer contexts before they ran. */
    for (int i = 0; i < 500 && mklink_usb_xfer_pending() > 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (mklink_usb_xfer_pending() > 0) {
        ESP_LOGW(TAG, "dev %u: %d transfer(s) still pending after unlink",
                 dev_addr, mklink_usb_xfer_pending());
    }
    return ESP_OK;
}
