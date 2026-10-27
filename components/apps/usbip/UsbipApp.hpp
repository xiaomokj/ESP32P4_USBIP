/*
 * USBIP - generic USB-over-IP gateway.
 *
 * Exports whatever USB device is plugged into the host port over the standard
 * usbip protocol, so any PC on the network can attach it as a local USB device
 * (`usbip attach -r <IP> -b 1-1`).
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class UsbipApp : public ESP_Brookesia_PhoneApp {
public:
    UsbipApp();
    ~UsbipApp();

    bool run(void) override;
    bool back(void) override;
    bool close(void) override;

    bool init(void) override;
    bool pause(void) override;
    bool resume(void) override;

private:
    static void refreshTimerCb(lv_timer_t *timer);
    static void serviceToggleCb(lv_event_t *e);
    static void rescanClickedCb(lv_event_t *e);

    void buildUi(void);
    void refresh(void);

    /* ---- USB device card ---- */
    lv_obj_t *_usb_dot = nullptr;
    lv_obj_t *_usb_name = nullptr;
    lv_obj_t *_usb_id = nullptr;
    lv_obj_t *_usb_meta = nullptr;

    /* ---- network card ---- */
    lv_obj_t *_net_dot = nullptr;
    lv_obj_t *_net_line = nullptr;
    lv_obj_t *_net_ip = nullptr;
    lv_obj_t *_net_hint = nullptr;

    /* ---- log ---- */
    lv_obj_t *_log_label = nullptr;

    /* ---- usbip service ---- */
    lv_obj_t *_svc_btn = nullptr;
    lv_obj_t *_svc_btn_label = nullptr;
    lv_obj_t *_svc_status = nullptr;
    lv_obj_t *_svc_hint = nullptr;

    lv_timer_t *_timer = nullptr;
    const ESP_Brookesia_StatusBar *_status_bar = nullptr;
    int _last_rssi_level = -1;
    uint32_t _tick = 0;
};
