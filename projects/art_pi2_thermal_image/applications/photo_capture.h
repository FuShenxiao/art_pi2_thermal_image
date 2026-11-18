/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PHOTO_CAPTURE_H
#define PHOTO_CAPTURE_H

#include <rtthread.h>
#include <lvgl.h>

typedef lv_draw_buf_t *(*photo_snapshot_cb)(void);

rt_err_t photo_capture_init(lv_obj_t *screen, photo_snapshot_cb camera_cb,
                            photo_snapshot_cb thermal_cb);

#endif
