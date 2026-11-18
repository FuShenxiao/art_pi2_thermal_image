/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#ifdef BSP_USING_MLX90640

#include <math.h>
#include <limits.h>
#include <stdint.h>
#include <rtdevice.h>
#include <drv_gpio.h>
#include <lvgl.h>

#include "thermal_preview.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "mlx90640_port.h"

#define LOG_TAG "thermal"
#include <drv_log.h>

#if !defined(BSP_USING_I2C5) || !defined(BSP_USING_OV5640)
#error "MLX90640 preview requires software I2C5 and the camera preview"
#endif

#define THERMAL_ADDRESS        0x33U
#define THERMAL_WIDTH          32U
#define THERMAL_HEIGHT         24U
#define THERMAL_PIXELS         (THERMAL_WIDTH * THERMAL_HEIGHT)
#define THERMAL_MAX_SCALE      20U
#define THERMAL_DRAW_WIDTH     (THERMAL_WIDTH * THERMAL_MAX_SCALE)
#define THERMAL_DRAW_HEIGHT    (THERMAL_HEIGHT * THERMAL_MAX_SCALE)
#define THERMAL_DRAW_BYTES     (THERMAL_DRAW_WIDTH * THERMAL_DRAW_HEIGHT * 2U)
#define THERMAL_BUFFER_ALIGN   32U
#define THERMAL_REFRESH_2HZ    0x02U
#define THERMAL_EMISSIVITY     0.95f
#define THERMAL_REFLECTED_DIFF 8.0f
#define THERMAL_EVENT_START    0x01U
#define USER1_PIN              GET_PIN(C, 0)
#define USER1_DEBOUNCE_SAMPLES 3U
#define THERMAL_THREAD_PRIORITY \
    ((PKG_LVGL_THREAD_PRIO + 2 < RT_THREAD_PRIORITY_MAX) ? \
     PKG_LVGL_THREAD_PRIO + 2 : RT_THREAD_PRIORITY_MAX - 1)

enum thermal_status
{
    THERMAL_WAITING,
    THERMAL_READY,
    THERMAL_SENSOR_ERROR,
    THERMAL_FRAME_ERROR,
};

struct thermal_shared
{
    rt_bool_t active;
    rt_uint32_t generation;
    rt_uint32_t sequence;
    enum thermal_status status;
    rt_uint16_t pixels[THERMAL_PIXELS];
    rt_uint16_t hottest_pixel;
    rt_int32_t hottest_tenths;
};

extern struct rt_memheap system_heap;

static struct rt_mutex thermal_lock;
static struct rt_event thermal_event;
static struct thermal_shared shared;
static rt_bool_t worker_available;
static rt_bool_t thermal_fullscreen;
static rt_bool_t thermal_layout_dirty;
static rt_uint32_t thermal_scale;
static rt_uint32_t thermal_left;
static rt_uint32_t thermal_top;
static thermal_view_switch_cb camera_switch;

static lv_obj_t *thermal_root;
static lv_obj_t *thermal_image;
static lv_obj_t *thermal_status_label;
static lv_obj_t *hot_marker;
static lv_obj_t *hot_label;
static lv_timer_t *button_timer;
static rt_uint16_t display_cells[THERMAL_PIXELS];
static void *display_allocation;
static rt_uint16_t *display_pixels;
static lv_image_dsc_t thermal_image_dsc;
static rt_uint32_t displayed_sequence;
static rt_int32_t button_candidate = PIN_HIGH;
static rt_int32_t button_stable = PIN_HIGH;
static rt_uint8_t button_samples;
static rt_bool_t button_read_error;

static uint16_t sensor_frame[MLX90640_EEPROM_DUMP_NUM + 2U];
static paramsMLX90640 calibration;
static float temperatures[THERMAL_PIXELS];
static rt_uint16_t rendered_pixels[THERMAL_PIXELS];
static rt_bool_t sensor_initialized;

static rt_bool_t request_is_current(rt_uint32_t generation)
{
    rt_bool_t current;

    if (rt_mutex_take(&thermal_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot lock thermal state");
        return RT_FALSE;
    }
    current = shared.active && shared.generation == generation;
    rt_mutex_release(&thermal_lock);
    return current;
}

static void publish_error(rt_uint32_t generation, enum thermal_status status)
{
    if (rt_mutex_take(&thermal_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot publish thermal error");
        return;
    }
    if (shared.active && shared.generation == generation)
    {
        shared.status = status;
        shared.sequence++;
    }
    rt_mutex_release(&thermal_lock);
}

static void publish_frame(rt_uint32_t generation, rt_uint16_t hottest_pixel,
                          rt_int32_t hottest_tenths)
{
    if (rt_mutex_take(&thermal_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot publish thermal frame");
        return;
    }
    if (shared.active && shared.generation == generation)
    {
        rt_memcpy(shared.pixels, rendered_pixels, sizeof(shared.pixels));
        shared.hottest_pixel = hottest_pixel;
        shared.hottest_tenths = hottest_tenths;
        shared.status = THERMAL_READY;
        shared.sequence++;
    }
    rt_mutex_release(&thermal_lock);
}

static rt_uint16_t thermal_color(rt_uint32_t level)
{
    rt_uint32_t red, green, blue;

    if (level < 256U)
    {
        red = 0;
        green = level;
        blue = 255;
    }
    else if (level < 512U)
    {
        red = 0;
        green = 255;
        blue = 511U - level;
    }
    else if (level < 768U)
    {
        red = level - 512U;
        green = 255;
        blue = 0;
    }
    else
    {
        red = 255;
        green = 1020U - level;
        blue = 0;
    }

    return (rt_uint16_t)(((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3));
}

static rt_err_t render_temperatures(rt_uint16_t *hottest_pixel, rt_int32_t *hottest_tenths)
{
    float minimum = temperatures[0];
    float maximum = temperatures[0];
    float span;
    rt_uint16_t hottest = 0;
    rt_uint32_t i;

    for (i = 0; i < THERMAL_PIXELS; i++)
    {
        if (!isfinite(temperatures[i]) || temperatures[i] < -273.15f)
        {
            LOG_E("invalid temperature at pixel %lu", (unsigned long)i);
            return -RT_ERROR;
        }
        if (temperatures[i] < minimum)
        {
            minimum = temperatures[i];
        }
        if (temperatures[i] > maximum)
        {
            maximum = temperatures[i];
            hottest = (rt_uint16_t)i;
        }
    }

    span = maximum - minimum;
    if (!isfinite(span) || maximum > (float)(INT32_MAX / 20) ||
        maximum < (float)(INT32_MIN / 20))
    {
        LOG_E("thermal temperature range is invalid");
        return -RT_ERROR;
    }

    /* Flip display rows only; preserve left/right order and calibrated sensor data. */
    for (i = 0; i < THERMAL_PIXELS; i++)
    {
        rt_uint32_t level = 512U;
        if (span >= 0.01f)
        {
            float scaled = (temperatures[i] - minimum) * (1020.0f / span);
            if (scaled <= 0.0f)
            {
                level = 0;
            }
            else if (scaled >= 1020.0f)
            {
                level = 1020U;
            }
            else
            {
                level = (rt_uint32_t)(scaled + 0.5f);
            }
        }
        rendered_pixels[(THERMAL_HEIGHT - 1U - i / THERMAL_WIDTH) * THERMAL_WIDTH +
                        i % THERMAL_WIDTH] = thermal_color(level);
    }

    *hottest_pixel = (rt_uint16_t)((THERMAL_HEIGHT - 1U - hottest / THERMAL_WIDTH) * THERMAL_WIDTH +
                                  hottest % THERMAL_WIDTH);
    *hottest_tenths = (rt_int32_t)(maximum * 10.0f + (maximum >= 0.0f ? 0.5f : -0.5f));
    return RT_EOK;
}

static rt_err_t thermal_sensor_init(void)
{
    int result;

    if (mlx90640_port_init() != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = MLX90640_DumpEE(THERMAL_ADDRESS, sensor_frame);
    if (result != MLX90640_NO_ERROR)
    {
        LOG_E("EEPROM read failed: %d", result);
        return -RT_ERROR;
    }

    rt_memset(&calibration, 0, sizeof(calibration));
    result = MLX90640_ExtractParameters(sensor_frame, &calibration);
    if (result != MLX90640_NO_ERROR)
    {
        LOG_E("invalid MLX90640 calibration: %d", result);
        return -RT_ERROR;
    }
    if (calibration.kVdd == 0 || calibration.KtPTAT == 0.0f || calibration.gainEE == 0)
    {
        LOG_E("MLX90640 calibration contains zero gain or denominator");
        return -RT_ERROR;
    }
    result = MLX90640_SetChessMode(THERMAL_ADDRESS);
    if (result == MLX90640_NO_ERROR)
    {
        result = MLX90640_SetRefreshRate(THERMAL_ADDRESS, THERMAL_REFRESH_2HZ);
    }
    if (result != MLX90640_NO_ERROR)
    {
        LOG_E("MLX90640 configuration failed: %d", result);
        return -RT_ERROR;
    }

    sensor_initialized = RT_TRUE;
    LOG_I("MLX90640 calibrated, chess mode, 2 Hz");
    return RT_EOK;
}

static void thermal_worker(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        rt_uint32_t event;
        rt_uint32_t generation;
        rt_uint8_t subpages;
        rt_uint8_t invalid_frames;
        int result;

        if (rt_event_recv(&thermal_event, THERMAL_EVENT_START,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &event) != RT_EOK)
        {
            LOG_E("thermal worker event failed");
            rt_thread_mdelay(100);
            continue;
        }
        if (rt_mutex_take(&thermal_lock, RT_WAITING_FOREVER) != RT_EOK)
        {
            LOG_E("cannot read thermal request");
            continue;
        }
        generation = shared.generation;
        if (!shared.active)
        {
            rt_mutex_release(&thermal_lock);
            continue;
        }
        rt_mutex_release(&thermal_lock);

        if (!sensor_initialized && thermal_sensor_init() != RT_EOK)
        {
            publish_error(generation, THERMAL_SENSOR_ERROR);
            continue;
        }
        if (!request_is_current(generation))
        {
            continue;
        }
        result = MLX90640_I2CWrite(THERMAL_ADDRESS, MLX90640_STATUS_REG,
                                   MLX90640_INIT_STATUS_VALUE);
        if (result != MLX90640_NO_ERROR)
        {
            LOG_E("cannot start thermal capture: %d", result);
            sensor_initialized = RT_FALSE;
            publish_error(generation, THERMAL_SENSOR_ERROR);
            continue;
        }

        rt_memset(temperatures, 0, sizeof(temperatures));
        subpages = 0;
        invalid_frames = 0;
        while (request_is_current(generation))
        {
            float ambient;
            rt_uint16_t hottest_pixel;
            rt_int32_t hottest_tenths;

            result = MLX90640_GetFrameData(THERMAL_ADDRESS, sensor_frame);
            if (!request_is_current(generation))
            {
                break;
            }
            if (result < 0 || result > 1)
            {
                if (result == -MLX90640_FRAME_DATA_ERROR && ++invalid_frames < 3U)
                {
                    LOG_W("invalid or overwritten thermal subpage; retrying");
                    subpages = 0;
                    continue;
                }
                LOG_E("thermal frame read failed: %d", result);
                sensor_initialized = RT_FALSE;
                publish_error(generation, THERMAL_FRAME_ERROR);
                break;
            }
            invalid_frames = 0;
            ambient = MLX90640_GetTa(sensor_frame, &calibration);
            if (!isfinite(ambient))
            {
                LOG_E("invalid MLX90640 ambient temperature");
                sensor_initialized = RT_FALSE;
                publish_error(generation, THERMAL_FRAME_ERROR);
                break;
            }
            MLX90640_CalculateTo(sensor_frame, &calibration, THERMAL_EMISSIVITY,
                                 ambient - THERMAL_REFLECTED_DIFF, temperatures);
            subpages |= (rt_uint8_t)(1U << (unsigned int)result);
            if (subpages != 3U)
            {
                continue;
            }

            MLX90640_BadPixelsCorrection(calibration.brokenPixels, temperatures, 1, &calibration);
            MLX90640_BadPixelsCorrection(calibration.outlierPixels, temperatures, 1, &calibration);
            if (render_temperatures(&hottest_pixel, &hottest_tenths) != RT_EOK)
            {
                sensor_initialized = RT_FALSE;
                publish_error(generation, THERMAL_FRAME_ERROR);
                break;
            }
            publish_frame(generation, hottest_pixel, hottest_tenths);
        }
    }
}

static void show_thermal_status(const char *text)
{
    lv_obj_add_flag(thermal_image, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(hot_marker, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(hot_label, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(thermal_status_label, text);
    lv_obj_center(thermal_status_label);
    lv_obj_remove_flag(thermal_status_label, LV_OBJ_FLAG_HIDDEN);
}

static rt_err_t thermal_alloc_display(void)
{
    if (display_pixels != RT_NULL)
    {
        return RT_EOK;
    }

    display_allocation = rt_memheap_alloc(&system_heap, THERMAL_DRAW_BYTES + THERMAL_BUFFER_ALIGN - 1U);
    if (display_allocation == RT_NULL)
    {
        LOG_E("not enough PSRAM for thermal display");
        return -RT_ENOMEM;
    }
    display_pixels = (rt_uint16_t *)RT_ALIGN((rt_ubase_t)display_allocation,
                                            (rt_ubase_t)THERMAL_BUFFER_ALIGN);
    thermal_image_dsc.data = (const uint8_t *)display_pixels;
    return RT_EOK;
}

static void expand_thermal_pixels(void)
{
    rt_uint16_t line[THERMAL_DRAW_WIDTH];
    rt_uint32_t width = THERMAL_WIDTH * thermal_scale;
    rt_uint32_t row, column, x, y;

    for (row = 0; row < THERMAL_HEIGHT; row++)
    {
        for (column = 0; column < THERMAL_WIDTH; column++)
        {
            for (x = 0; x < thermal_scale; x++)
            {
                line[column * thermal_scale + x] = display_cells[row * THERMAL_WIDTH + column];
            }
        }
        for (y = 0; y < thermal_scale; y++)
        {
            rt_memcpy(&display_pixels[(row * thermal_scale + y) * width],
                      line, width * sizeof(line[0]));
        }
    }
}

static void show_thermal_frame(rt_uint16_t hottest_pixel, rt_int32_t hottest_tenths)
{
    int32_t view_width = thermal_fullscreen ? LV_HOR_RES_MAX : PREVIEW_SPLIT_WIDTH;
    int32_t x = (int32_t)thermal_left + (hottest_pixel % THERMAL_WIDTH) * thermal_scale +
                thermal_scale / 2;
    int32_t y = (int32_t)thermal_top + (hottest_pixel / THERMAL_WIDTH) * thermal_scale +
                thermal_scale / 2;
    int32_t label_x, label_y, label_w, label_h;
    rt_int32_t magnitude = hottest_tenths < 0 ? -hottest_tenths : hottest_tenths;

    lv_image_cache_drop(&thermal_image_dsc);
    lv_image_set_src(thermal_image, &thermal_image_dsc);
    lv_obj_remove_flag(thermal_image, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(thermal_status_label, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_pos(hot_marker, x - 6, y - 6);
    lv_obj_remove_flag(hot_marker, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text_fmt(hot_label, "MAX %s%ld.%ld \xc2\xb0" "C",
                          hottest_tenths < 0 ? "-" : "",
                          (long)(magnitude / 10), (long)(magnitude % 10));
    lv_obj_update_layout(hot_label);
    label_w = lv_obj_get_width(hot_label);
    label_h = lv_obj_get_height(hot_label);
    label_x = x + 12;
    label_y = y + 12;
    if (label_x + label_w > view_width)
    {
        label_x = x - label_w - 12;
    }
    if (label_y + label_h > LV_VER_RES_MAX)
    {
        label_y = y - label_h - 12;
    }
    if (label_x < 0) label_x = 0;
    if (label_y < 0) label_y = 0;
    lv_obj_set_pos(hot_label, label_x, label_y);
    lv_obj_remove_flag(hot_label, LV_OBJ_FLAG_HIDDEN);
}

static void thermal_set_layout(void)
{
    rt_uint32_t view_width = thermal_fullscreen ? LV_HOR_RES_MAX : PREVIEW_SPLIT_WIDTH;
    rt_uint32_t width, height;

    thermal_scale = LV_MIN(view_width / THERMAL_WIDTH, LV_VER_RES_MAX / THERMAL_HEIGHT);
    width = THERMAL_WIDTH * thermal_scale;
    height = THERMAL_HEIGHT * thermal_scale;
    thermal_left = (view_width - width) / 2U;
    thermal_top = (LV_VER_RES_MAX - height) / 2U;

    lv_image_cache_drop(&thermal_image_dsc);
    thermal_image_dsc.header.w = width;
    thermal_image_dsc.header.h = height;
    thermal_image_dsc.header.stride = width * 2U;
    thermal_image_dsc.data_size = width * height * 2U;
    lv_obj_set_size(thermal_root, view_width, LV_VER_RES_MAX);
    lv_obj_set_size(thermal_image, width, height);
    lv_obj_set_pos(thermal_image, thermal_left, thermal_top);
    lv_obj_set_width(thermal_status_label, view_width - 16U);
    lv_obj_center(thermal_status_label);
    lv_obj_add_flag(thermal_image, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(hot_marker, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(hot_label, LV_OBJ_FLAG_HIDDEN);
    thermal_layout_dirty = RT_TRUE;
}

static rt_err_t request_thermal_capture(void)
{
    rt_uint32_t generation;
    rt_err_t result;

    if (rt_mutex_take(&thermal_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot start thermal capture");
        return -RT_ERROR;
    }
    shared.active = RT_TRUE;
    generation = ++shared.generation;
    shared.status = THERMAL_WAITING;
    shared.sequence++;
    rt_mutex_release(&thermal_lock);

    result = rt_event_send(&thermal_event, THERMAL_EVENT_START);
    if (result != RT_EOK)
    {
        LOG_E("cannot wake thermal worker");
        publish_error(generation, THERMAL_SENSOR_ERROR);
    }
    return result;
}

static void set_thermal_fullscreen(rt_bool_t enabled)
{
    rt_bool_t retry;

    thermal_fullscreen = enabled;
    camera_switch(enabled);
    thermal_set_layout();
    if (!worker_available)
    {
        return;
    }

    show_thermal_status("MLX90640: waiting for frame...");
    if (rt_mutex_take(&thermal_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot read thermal status");
        show_thermal_status("MLX90640: synchronization error");
        return;
    }
    retry = shared.status == THERMAL_SENSOR_ERROR || shared.status == THERMAL_FRAME_ERROR;
    rt_mutex_release(&thermal_lock);
    if (retry && request_thermal_capture() != RT_EOK)
    {
        show_thermal_status("MLX90640: worker unavailable");
    }
}

static void thermal_ui_update(lv_timer_t *timer)
{
    rt_int32_t pin = rt_pin_read(USER1_PIN);
    rt_uint16_t hottest_pixel = 0;
    rt_int32_t hottest_tenths = 0;
    enum thermal_status status = THERMAL_WAITING;
    rt_bool_t changed = RT_FALSE;

    RT_UNUSED(timer);

    if (pin != PIN_LOW && pin != PIN_HIGH)
    {
        button_samples = 0;
        if (!button_read_error)
        {
            LOG_E("USER1 read failed: %d", pin);
            button_read_error = RT_TRUE;
        }
    }
    else
    {
        button_read_error = RT_FALSE;
        if (pin == button_candidate)
        {
            if (button_samples < USER1_DEBOUNCE_SAMPLES)
            {
                button_samples++;
            }
        }
        else
        {
            button_candidate = pin;
            button_samples = 1;
        }
        if (button_samples == USER1_DEBOUNCE_SAMPLES && button_stable != button_candidate)
        {
            button_stable = button_candidate;
            if (button_stable == PIN_LOW)
            {
                set_thermal_fullscreen(!thermal_fullscreen);
            }
        }
    }

    if (!worker_available ||
        rt_mutex_take(&thermal_lock, RT_WAITING_NO) != RT_EOK)
    {
        return;
    }
    if (shared.sequence != displayed_sequence || thermal_layout_dirty)
    {
        thermal_layout_dirty = RT_FALSE;
        displayed_sequence = shared.sequence;
        status = shared.status;
        if (status == THERMAL_READY)
        {
            rt_memcpy(display_cells, shared.pixels, sizeof(display_cells));
            hottest_pixel = shared.hottest_pixel;
            hottest_tenths = shared.hottest_tenths;
        }
        changed = RT_TRUE;
    }
    rt_mutex_release(&thermal_lock);

    if (changed)
    {
        if (status == THERMAL_READY)
        {
            expand_thermal_pixels();
            show_thermal_frame(hottest_pixel, hottest_tenths);
        }
        else if (status == THERMAL_SENSOR_ERROR)
        {
            show_thermal_status("MLX90640: check I2C and calibration");
        }
        else if (status == THERMAL_FRAME_ERROR)
        {
            show_thermal_status("MLX90640: frame error; see console");
        }
    }
}

rt_bool_t thermal_preview_is_fullscreen(void)
{
    return thermal_fullscreen;
}

#ifdef BSP_USING_PHOTO_CAPTURE
lv_draw_buf_t *thermal_preview_snapshot(void)
{
    lv_draw_buf_t *snapshot;

    if (!worker_available || thermal_root == RT_NULL ||
        lv_obj_has_flag(thermal_image, LV_OBJ_FLAG_HIDDEN))
    {
        LOG_E("thermal image is not ready for a snapshot");
        return RT_NULL;
    }
    snapshot = lv_snapshot_take(thermal_root, LV_COLOR_FORMAT_RGB565);
    if (snapshot == RT_NULL)
    {
        LOG_E("thermal snapshot allocation or rendering failed");
    }
    return snapshot;
}
#endif

rt_err_t thermal_preview_init(lv_obj_t *screen, thermal_view_switch_cb switch_cb)
{
    rt_thread_t thread;

    if (screen == RT_NULL || switch_cb == RT_NULL)
    {
        LOG_E("invalid thermal preview configuration");
        return -RT_EINVAL;
    }
    camera_switch = switch_cb;
    thermal_root = lv_obj_create(screen);
    if (thermal_root == RT_NULL)
    {
        LOG_E("cannot create thermal screen");
        return -RT_ENOMEM;
    }
    lv_obj_remove_style_all(thermal_root);
    lv_obj_set_size(thermal_root, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_pos(thermal_root, 0, 0);
    lv_obj_set_style_bg_color(thermal_root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(thermal_root, LV_OPA_COVER, 0);
    lv_obj_clear_flag(thermal_root, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    thermal_image = lv_image_create(thermal_root);
    hot_marker = lv_obj_create(thermal_root);
    hot_label = lv_label_create(thermal_root);
    thermal_status_label = lv_label_create(thermal_root);
    if (thermal_image == RT_NULL || hot_marker == RT_NULL || hot_label == RT_NULL ||
        thermal_status_label == RT_NULL)
    {
        LOG_E("cannot create thermal widgets");
        lv_obj_delete(thermal_root);
        return -RT_ENOMEM;
    }

    thermal_image_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    thermal_image_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    lv_obj_add_flag(thermal_image, LV_OBJ_FLAG_HIDDEN);

    lv_obj_remove_style_all(hot_marker);
    lv_obj_set_size(hot_marker, 12, 12);
    lv_obj_set_style_bg_color(hot_marker, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(hot_marker, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(hot_marker, lv_color_black(), 0);
    lv_obj_set_style_border_width(hot_marker, 2, 0);
    lv_obj_set_style_radius(hot_marker, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(hot_marker, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(hot_marker, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_text_font(hot_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(hot_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(hot_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(hot_label, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(hot_label, 3, 0);
    lv_obj_add_flag(hot_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_font(thermal_status_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(thermal_status_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(thermal_status_label, LV_TEXT_ALIGN_CENTER, 0);
    thermal_set_layout();
    show_thermal_status("MLX90640: starting...");

    rt_pin_mode(USER1_PIN, PIN_MODE_INPUT_PULLUP);
    button_timer = lv_timer_create(thermal_ui_update, 20, RT_NULL);
    if (button_timer == RT_NULL)
    {
        LOG_E("cannot create USER1 timer");
        lv_obj_delete(thermal_root);
        return -RT_ENOMEM;
    }

    if (rt_mutex_init(&thermal_lock, "mlxlock", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        LOG_E("cannot create thermal mutex");
        show_thermal_status("MLX90640: synchronization error");
        return -RT_ERROR;
    }
    if (rt_event_init(&thermal_event, "mlxevt", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        LOG_E("cannot create thermal event");
        show_thermal_status("MLX90640: synchronization error");
        rt_mutex_detach(&thermal_lock);
        return -RT_ERROR;
    }
    if (thermal_alloc_display() != RT_EOK)
    {
        show_thermal_status("MLX90640: not enough PSRAM");
        rt_event_detach(&thermal_event);
        rt_mutex_detach(&thermal_lock);
        return -RT_ENOMEM;
    }
    thread = rt_thread_create("mlx906", thermal_worker, RT_NULL, 8192,
                              THERMAL_THREAD_PRIORITY, 10);
    if (thread == RT_NULL)
    {
        LOG_E("cannot create thermal worker");
        show_thermal_status("MLX90640: worker unavailable");
        rt_event_detach(&thermal_event);
        rt_mutex_detach(&thermal_lock);
        return -RT_ENOMEM;
    }
    if (rt_thread_startup(thread) != RT_EOK)
    {
        LOG_E("cannot start thermal worker");
        show_thermal_status("MLX90640: worker unavailable");
        rt_thread_delete(thread);
        rt_event_detach(&thermal_event);
        rt_mutex_detach(&thermal_lock);
        return -RT_ERROR;
    }
    worker_available = RT_TRUE;
    if (request_thermal_capture() != RT_EOK)
    {
        show_thermal_status("MLX90640: worker unavailable");
        return -RT_ERROR;
    }
    return RT_EOK;
}

#endif /* BSP_USING_MLX90640 */
