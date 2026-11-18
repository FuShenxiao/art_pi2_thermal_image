/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THERMAL_PREVIEW_H
#define THERMAL_PREVIEW_H

#include <rtthread.h>
#include <lvgl.h>

#define PREVIEW_SPLIT_WIDTH (LV_HOR_RES_MAX / 2)

typedef void (*thermal_view_switch_cb)(rt_bool_t thermal_fullscreen);

rt_err_t thermal_preview_init(lv_obj_t *screen, thermal_view_switch_cb switch_cb);
rt_bool_t thermal_preview_is_fullscreen(void);
#ifdef BSP_USING_PHOTO_CAPTURE
lv_draw_buf_t *thermal_preview_snapshot(void);
#endif

#endif
