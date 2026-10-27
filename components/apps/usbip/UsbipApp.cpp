/*
 * USBIP - generic USB-over-IP gateway.
 *
 * Layout: header / two status cards (USB device + network) / usbip service
 * panel / log panel / action row. Any USB device plugged into the host port is
 * exported; the PC runs `usbip attach -r <IP> -b 1-1` and sees it natively.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>

#include "UsbipApp.hpp"
#include "mklink_usbip/MkLinkUi.hpp"
#include "mklink_core.h"
#include "mklink_usb.h"
#include "mklink_wifi.h"
#include "mklink_usbip.h"

LV_IMG_DECLARE(img_app_usbip);
LV_FONT_DECLARE(mklink_font_cjk_16);

#define COL_BG        lv_color_hex(0x0B0F14)
#define COL_CARD      lv_color_hex(0x121821)
#define COL_BORDER    lv_color_hex(0x243040)
#define COL_TEXT      lv_color_hex(0xE5E7EB)
#define COL_MUTED     lv_color_hex(0x94A3B8)
#define COL_DIM       lv_color_hex(0x64748B)
#define COL_ACCENT    lv_color_hex(0x22D3EE)
#define COL_ACCENT_2  lv_color_hex(0x3B82F6)
#define COL_OK        lv_color_hex(0x34D399)
#define COL_WARN      lv_color_hex(0xFBBF24)

#define FONT_TITLE    &lv_font_montserrat_28
#define FONT_VALUE    &lv_font_montserrat_20
#define FONT_META     &lv_font_montserrat_16
#define FONT_SMALL    &lv_font_montserrat_12
#define FONT_CJK      &mklink_font_cjk_16

static lv_obj_t *make_dot(lv_obj_t *parent, lv_color_t color)
{
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_set_size(dot, 10, 10);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, color, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    return dot;
}

static lv_obj_t *make_card(lv_obj_t *parent, const char *title, lv_obj_t **dot_out)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_height(card, LV_PCT(100));
    lv_obj_set_flex_grow(card, 1);
    lv_obj_set_style_bg_color(card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, COL_BORDER, 0);
    lv_obj_set_style_pad_all(card, 12, 0);
    lv_obj_set_style_pad_row(card, 4, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *head = make_flat(card, LV_PCT(100), 18);
    lv_obj_set_flex_flow(head, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(head, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(head, 6, 0);

    make_label(head, title, FONT_CJK, COL_MUTED);

    if (dot_out != NULL) {
        lv_obj_t *spacer = make_flat(head, 0, 1);
        lv_obj_set_flex_grow(spacer, 1);
        *dot_out = make_dot(head, COL_DIM);
    }
    return card;
}

UsbipApp::UsbipApp() : ESP_Brookesia_PhoneApp("USBIP", &img_app_usbip, true)
{
}

UsbipApp::~UsbipApp()
{
}

bool UsbipApp::init(void)
{
    ESP_Brookesia_Phone *phone = getPhone();
    if (phone != NULL) {
        _status_bar = phone->getHome().getStatusBar();
    }
    return true;
}

void UsbipApp::buildUi(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);

    lv_obj_t *root = make_flat(scr, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root, 18, 0);
    lv_obj_set_style_pad_row(root, 10, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);

    /* ---------------- header ---------------- */
    lv_obj_t *header = make_flat(root, LV_PCT(100), 38);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(header, 8, 0);

    make_label(header, "USBIP", FONT_TITLE, COL_TEXT);
    lv_obj_t *tag = make_label(header, "WIRELESS", FONT_SMALL, COL_ACCENT);
    lv_obj_set_style_text_letter_space(tag, 3, 0);

    lv_obj_t *spacer = make_flat(header, 0, 1);
    lv_obj_set_flex_grow(spacer, 1);

    make_label(header, mklink_core_version(), FONT_SMALL, COL_DIM);

    /* ---------------- status cards ---------------- */
    lv_obj_t *cards = make_flat(root, LV_PCT(100), 190);
    lv_obj_set_flex_flow(cards, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(cards, 12, 0);

    lv_obj_t *usb_card = make_card(cards, "USB 设备", &_usb_dot);
    _usb_name = make_label(usb_card, "未连接", FONT_CJK, COL_TEXT);
    lv_label_set_long_mode(_usb_name, LV_LABEL_LONG_DOT);
    lv_obj_set_width(_usb_name, LV_PCT(100));
    _usb_id = make_label(usb_card, "-- : --", FONT_META, COL_ACCENT);
    _usb_meta = make_label(usb_card, "插入任意 USB 设备后自动识别", FONT_CJK, COL_DIM);
    lv_label_set_long_mode(_usb_meta, LV_LABEL_LONG_DOT);
    lv_obj_set_width(_usb_meta, LV_PCT(100));

    lv_obj_t *net_card = make_card(cards, "网络地址", &_net_dot);
    _net_line = make_label(net_card, "等待联网", FONT_CJK, COL_TEXT);
    lv_label_set_long_mode(_net_line, LV_LABEL_LONG_DOT);
    lv_obj_set_width(_net_line, LV_PCT(100));
    _net_ip = make_label(net_card, "IP --", FONT_META, COL_ACCENT);
    _net_hint = make_label(net_card, "", FONT_CJK, COL_DIM);
    lv_label_set_long_mode(_net_hint, LV_LABEL_LONG_DOT);
    lv_obj_set_width(_net_hint, LV_PCT(100));

    /* ---------------- usbip service panel ---------------- */
    lv_obj_t *svc = lv_obj_create(root);
    lv_obj_set_width(svc, LV_PCT(100));
    lv_obj_set_style_bg_color(svc, COL_CARD, 0);
    lv_obj_set_style_bg_opa(svc, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(svc, 18, 0);
    lv_obj_set_style_border_width(svc, 1, 0);
    lv_obj_set_style_border_color(svc, COL_BORDER, 0);
    lv_obj_set_style_pad_all(svc, 14, 0);
    lv_obj_set_style_pad_row(svc, 6, 0);
    lv_obj_set_flex_flow(svc, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(svc, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *svc_head = make_flat(svc, LV_PCT(100), 52);
    lv_obj_set_flex_flow(svc_head, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(svc_head, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    make_label(svc_head, "USB 透传服务", FONT_CJK, COL_MUTED);

    _svc_btn = lv_btn_create(svc_head);
    lv_obj_set_size(_svc_btn, LV_SIZE_CONTENT, 52);
    lv_obj_set_style_pad_hor(_svc_btn, 28, 0);
    lv_obj_set_style_pad_ver(_svc_btn, 0, 0);
    lv_obj_set_style_radius(_svc_btn, 26, 0);
    lv_obj_set_style_bg_color(_svc_btn, COL_ACCENT, 0);
    lv_obj_set_style_bg_grad_color(_svc_btn, COL_ACCENT_2, 0);
    lv_obj_set_style_bg_grad_dir(_svc_btn, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(_svc_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(_svc_btn, 0, 0);
    lv_obj_set_style_shadow_width(_svc_btn, 16, 0);
    lv_obj_set_style_shadow_color(_svc_btn, COL_ACCENT_2, 0);
    lv_obj_set_style_shadow_opa(_svc_btn, LV_OPA_30, 0);
    lv_obj_add_event_cb(_svc_btn, serviceToggleCb, LV_EVENT_CLICKED, this);
    _svc_btn_label = make_label(_svc_btn, "启动服务", FONT_CJK, lv_color_hex(0x06121A));
    lv_obj_set_style_text_letter_space(_svc_btn_label, 2, 0);
    lv_obj_center(_svc_btn_label);

    _svc_status = make_label(svc, "服务未启动", FONT_CJK, COL_MUTED);
    _svc_hint = make_label(svc, "启动后电脑执行: usbip attach -r <IP> -b 1-1",
                           FONT_CJK, COL_DIM);
    lv_label_set_long_mode(_svc_hint, LV_LABEL_LONG_DOT);
    lv_obj_set_width(_svc_hint, LV_PCT(100));

    /* ---------------- log panel ---------------- */
    lv_obj_t *log_panel = lv_obj_create(root);
    lv_obj_set_width(log_panel, LV_PCT(100));
    lv_obj_set_flex_grow(log_panel, 1);
    lv_obj_set_style_bg_color(log_panel, COL_CARD, 0);
    lv_obj_set_style_bg_opa(log_panel, LV_OPA_70, 0);
    lv_obj_set_style_radius(log_panel, 16, 0);
    lv_obj_set_style_border_width(log_panel, 1, 0);
    lv_obj_set_style_border_color(log_panel, COL_BORDER, 0);
    lv_obj_set_style_pad_all(log_panel, 12, 0);
    lv_obj_clear_flag(log_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(log_panel, LV_FLEX_FLOW_COLUMN);

    make_label(log_panel, "运行日志", FONT_CJK, COL_MUTED);
    _log_label = make_label(log_panel, "", FONT_CJK, COL_MUTED);
    lv_obj_set_width(_log_label, LV_PCT(100));
    lv_label_set_long_mode(_log_label, LV_LABEL_LONG_DOT);
    lv_obj_set_flex_grow(_log_label, 1);
    lv_obj_set_style_text_line_space(_log_label, 2, 0);

    /* ---------------- action row ---------------- */
    lv_obj_t *footer = make_flat(root, LV_PCT(100), 52);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(footer, 12, 0);

    lv_obj_t *rescan = lv_btn_create(footer);
    lv_obj_set_height(rescan, LV_PCT(100));
    lv_obj_set_flex_grow(rescan, 1);
    lv_obj_set_style_radius(rescan, 26, 0);
    lv_obj_set_style_bg_color(rescan, COL_ACCENT, 0);
    lv_obj_set_style_bg_grad_color(rescan, COL_ACCENT_2, 0);
    lv_obj_set_style_bg_grad_dir(rescan, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(rescan, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(rescan, 0, 0);
    lv_obj_set_style_shadow_width(rescan, 16, 0);
    lv_obj_set_style_shadow_color(rescan, COL_ACCENT_2, 0);
    lv_obj_set_style_shadow_opa(rescan, LV_OPA_30, 0);
    lv_obj_add_event_cb(rescan, rescanClickedCb, LV_EVENT_CLICKED, this);
    lv_obj_t *rescan_label = make_label(rescan, "重扫 USB", FONT_CJK, lv_color_hex(0x06121A));
    lv_obj_set_style_text_letter_space(rescan_label, 2, 0);
    lv_obj_center(rescan_label);
}

void UsbipApp::refresh(void)
{
    if (_usb_name == NULL) {
        return;
    }

    char buf[96];

    /* ---- USB device ---- */
    mklink_usb_snapshot_t usb;
    bool usb_ok = mklink_usb_get_snapshot(&usb);
    set_bg_color_if_changed(_usb_dot, usb_ok ? COL_OK : COL_DIM);
    if (usb_ok) {
        set_text_if_changed(_usb_name, usb.product[0] ? usb.product : "USB device");
        snprintf(buf, sizeof(buf), "%04X : %04X", usb.vid, usb.pid);
        set_text_if_changed(_usb_id, buf);
        if (usb.manufacturer[0]) {
            snprintf(buf, sizeof(buf), "%s · %u 接口 · %u 端点",
                     usb.manufacturer, usb.iface_cnt, usb.ep_cnt);
        } else {
            snprintf(buf, sizeof(buf), "%u 接口 · %u 端点", usb.iface_cnt, usb.ep_cnt);
        }
        set_text_if_changed(_usb_meta, buf);
        set_text_color_if_changed(_usb_id, COL_ACCENT);
    } else {
        set_text_if_changed(_usb_name, "未连接");
        set_text_if_changed(_usb_id, "-- : --");
        set_text_if_changed(_usb_meta, "插入任意 USB 设备后自动识别");
        set_text_color_if_changed(_usb_id, COL_DIM);
    }

    /* ---- network ---- */
    mklink_wifi_status_t wifi;
    mklink_wifi_status(&wifi);

    bool sta = wifi.sta_up;
    bool ap = wifi.ap_up;
    const char *ip = sta ? wifi.sta_ip : (ap ? wifi.ap_ip : NULL);

    set_bg_color_if_changed(_net_dot, ip != NULL ? COL_OK : COL_DIM);
    if (sta) {
        snprintf(buf, sizeof(buf), "局域网 · %s", wifi.sta_ssid);
        set_text_if_changed(_net_line, buf);
        set_text_color_if_changed(_net_line, COL_TEXT);
    } else if (ap) {
        set_text_if_changed(_net_line, "热点模式（未接路由器）");
        set_text_color_if_changed(_net_line, COL_MUTED);
    } else {
        set_text_if_changed(_net_line, "等待联网");
        set_text_color_if_changed(_net_line, COL_MUTED);
    }
    if (ip != NULL) {
        snprintf(buf, sizeof(buf), "IP %s", ip);
    } else {
        snprintf(buf, sizeof(buf), "IP --");
    }
    set_text_if_changed(_net_ip, buf);

    int rssi_level = sta ? wifi.rssi_level : 0;
    if (_status_bar != NULL && rssi_level != _last_rssi_level) {
        _status_bar->setWifiIconState(rssi_level);
        _last_rssi_level = rssi_level;
    }

    /* ---- usbip service ---- */
    bool svc_up = mklink_usbip_running();
    if (svc_up) {
        snprintf(buf, sizeof(buf), "运行中 · 端口 %u · 设备 %s",
                 (unsigned)mklink_usbip_port(), usb_ok ? "在线" : "未接入");
        set_text_if_changed(_svc_status, buf);
        if (ip != NULL) {
            snprintf(buf, sizeof(buf), "电脑: usbip attach -r %s -b 1-1", ip);
        } else {
            snprintf(buf, sizeof(buf), "电脑: usbip attach -r <IP> -b 1-1");
        }
        set_text_if_changed(_svc_hint, buf);
        set_text_if_changed(_net_hint, "电脑端可直接 attach 本机设备");
        set_text_color_if_changed(_svc_status, COL_OK);
        set_text_if_changed(_svc_btn_label, "停止服务");
        set_bg_color_if_changed(_svc_btn, COL_WARN);
    } else {
        set_text_if_changed(_svc_status, "服务未启动");
        set_text_if_changed(_svc_hint, "启动后电脑执行: usbip attach -r <IP> -b 1-1");
        set_text_if_changed(_net_hint, "启动服务后才能远程连接");
        set_text_color_if_changed(_svc_status, COL_MUTED);
        set_text_if_changed(_svc_btn_label, "启动服务");
        set_bg_color_if_changed(_svc_btn, COL_ACCENT);
    }

    /* ---- log ---- */
    char log_buf[640];
    if (mklink_log_tail(log_buf, sizeof(log_buf), 6) > 0) {
        set_text_if_changed(_log_label, log_buf);
    }
}

bool UsbipApp::run(void)
{
    buildUi();
    refresh();

    if (_timer == nullptr) {
        _timer = lv_timer_create(refreshTimerCb, 200, this);
    } else {
        lv_timer_resume(_timer);
    }
    return true;
}

bool UsbipApp::back(void)
{
    if (_timer != nullptr) {
        lv_timer_pause(_timer);
    }
    notifyCoreClosed();
    return true;
}

bool UsbipApp::close(void)
{
    _timer = nullptr;
    _usb_name = nullptr;
    _net_line = nullptr;
    _log_label = nullptr;
    _svc_btn = nullptr;
    _svc_status = nullptr;
    return true;
}

bool UsbipApp::pause(void)
{
    if (_timer != nullptr) {
        lv_timer_pause(_timer);
    }
    return true;
}

bool UsbipApp::resume(void)
{
    if (_timer != nullptr) {
        lv_timer_resume(_timer);
        refresh();
    }
    return true;
}

void UsbipApp::refreshTimerCb(lv_timer_t *timer)
{
    UsbipApp *app = (UsbipApp *)timer->user_data;
    if (app == nullptr) {
        return;
    }
    app->_tick++;
    if (app->_tick % 2 == 0) {
        app->refresh();
    }
}

void UsbipApp::serviceToggleCb(lv_event_t *e)
{
    UsbipApp *app = (UsbipApp *)lv_event_get_user_data(e);
    if (mklink_usbip_running()) {
        mklink_usbip_stop();
        mklink_log("USBIP: service stopped from panel");
    } else {
        if (mklink_usbip_start(CONFIG_MKLINK_USBIP_PORT) == ESP_OK) {
            mklink_log("USBIP: service started on %u", (unsigned)mklink_usbip_port());
        } else {
            mklink_log("USBIP: failed to start service");
        }
    }
    if (app != nullptr) {
        app->refresh();
    }
}

void UsbipApp::rescanClickedCb(lv_event_t *e)
{
    UsbipApp *app = (UsbipApp *)lv_event_get_user_data(e);
    mklink_log("UI: manual USB rescan requested");
    if (mklink_usb_rescan() != ESP_OK) {
        mklink_log("UI: USB host is not ready yet");
    }
    if (app != nullptr) {
        app->refresh();
    }
}
