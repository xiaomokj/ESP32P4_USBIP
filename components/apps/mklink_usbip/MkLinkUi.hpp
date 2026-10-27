/*
 * MKLink Remote - small LVGL helpers shared by the dashboard and the
 * RTT/Watch pages.
 *
 * Why the "if_changed" helpers matter: lv_label_set_text() calls
 * lv_obj_invalidate() unconditionally and re-allocates the string, and
 * lv_obj_set_style_*() invalidates the whole object even when the value is
 * identical. Calling them from a periodic timer therefore keeps the display
 * busy even when nothing changed - which is what made tab switching feel
 * laggy. Everything here compares first and only touches LVGL when needed.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <string.h>

#include "lvgl.h"

/** Transparent, borderless, padding-free container (a plain layout box). */
inline lv_obj_t *mk_flat(lv_obj_t *parent, int32_t w, int32_t h)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_radius(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    return obj;
}

inline lv_obj_t *mk_label(lv_obj_t *parent, const char *text, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text != nullptr ? text : "");
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    return label;
}

inline void set_text_if_changed(lv_obj_t *obj, const char *text)
{
    if (obj == nullptr || text == nullptr) {
        return;
    }
    const char *current = lv_label_get_text(obj);
    if (current != nullptr && strcmp(current, text) == 0) {
        return;
    }
    lv_label_set_text(obj, text);
}

inline void set_bg_color_if_changed(lv_obj_t *obj, lv_color_t color)
{
    if (obj == nullptr) {
        return;
    }
    lv_color_t current = lv_obj_get_style_bg_color(obj, LV_PART_MAIN);
    if (lv_color_to32(current) == lv_color_to32(color)) {
        return;
    }
    lv_obj_set_style_bg_color(obj, color, 0);
}

inline void set_text_color_if_changed(lv_obj_t *obj, lv_color_t color)
{
    if (obj == nullptr) {
        return;
    }
    lv_color_t current = lv_obj_get_style_text_color(obj, LV_PART_MAIN);
    if (lv_color_to32(current) == lv_color_to32(color)) {
        return;
    }
    lv_obj_set_style_text_color(obj, color, 0);
}

/* Aliases kept so existing call sites read naturally. */
inline lv_obj_t *make_flat(lv_obj_t *parent, int32_t w, int32_t h)
{
    return mk_flat(parent, w, h);
}

inline lv_obj_t *make_label(lv_obj_t *parent, const char *text, const lv_font_t *font, lv_color_t color)
{
    return mk_label(parent, text, font, color);
}
