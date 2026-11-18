/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#ifdef BSP_USING_OV5640

#include <rtdevice.h>
#include <board.h>
#include <drv_gpio.h>
#include <lvgl.h>

#include "ov5640.h"
#ifdef BSP_USING_MLX90640
#include "thermal_preview.h"
#endif
#ifdef BSP_USING_PHOTO_CAPTURE
#include "photo_capture.h"
#endif

#define LOG_TAG "camera"
#include <drv_log.h>

#if !defined(BSP_USING_I2C4) || !defined(BSP_USING_PSRAM) || !defined(BSP_USING_LCD_800_RGB)
#error "OV5640 preview requires I2C4, PSRAM and the 800x480 RGB LCD"
#endif

#if LV_COLOR_DEPTH != 16 || LV_HOR_RES_MAX != 800 || LV_VER_RES_MAX != 480
#error "OV5640 preview requires an 800x480 RGB565 LVGL display"
#endif

#define CAMERA_WIDTH          800U
#define CAMERA_HEIGHT         480U
#define CAMERA_FRAME_SIZE     (CAMERA_WIDTH * CAMERA_HEIGHT * 2U)
#define CAMERA_FRAME_ALIGN    32U
#define CAMERA_TIMEOUT_MS     3000U
#define CAMERA_I2C_ADDRESS    0x78U
#define CAMERA_PWDN_PIN       GET_PIN(D, 6)
#define CAMERA_RESET_PIN      GET_PIN(E, 2)
#define CAMERA_COUNTER_ERROR  0x80000000U

extern struct rt_memheap system_heap;

static struct rt_i2c_bus_device *camera_bus;
static OV5640_Object_t sensor;
static DCMIPP_HandleTypeDef camera_dcmi;
static void *frame_allocations[2];
static lv_image_dsc_t frames[2];
static lv_obj_t *camera_root;
static lv_obj_t *preview_image;
static lv_obj_t *status_label;
static lv_timer_t *preview_timer;
static rt_uint8_t capture_index;
static rt_tick_t capture_started_at;
static rt_bool_t dcmi_initialized;
static rt_bool_t capture_started;
static rt_bool_t image_displayed;
static volatile rt_uint32_t completed_size;
static volatile rt_uint32_t capture_error;
static volatile rt_bool_t frame_ready;

static int32_t camera_i2c_init(void)
{
    camera_bus = rt_i2c_bus_device_find("i2c4");
    if (camera_bus == RT_NULL)
    {
        LOG_E("software I2C bus i2c4 not found");
        return OV5640_ERROR;
    }

    return OV5640_OK;
}

static int32_t camera_i2c_deinit(void)
{
    return OV5640_OK;
}

static int32_t camera_i2c_read(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
    rt_uint8_t reg_addr[2] = {(rt_uint8_t)(reg >> 8), (rt_uint8_t)reg};
    struct rt_i2c_msg messages[2] =
    {
        {address >> 1, RT_I2C_WR, sizeof(reg_addr), reg_addr},
        {address >> 1, RT_I2C_RD, length, data},
    };

    if (camera_bus == RT_NULL || data == RT_NULL || length == 0)
    {
        LOG_E("invalid OV5640 I2C read");
        return OV5640_ERROR;
    }

    if (rt_i2c_transfer(camera_bus, messages, 2) != 2)
    {
        LOG_E("I2C read failed at register 0x%04x", reg);
        return OV5640_ERROR;
    }

    return OV5640_OK;
}

static int32_t camera_i2c_write(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
    rt_uint8_t reg_addr[2] = {(rt_uint8_t)(reg >> 8), (rt_uint8_t)reg};
    struct rt_i2c_msg messages[2] =
    {
        {address >> 1, RT_I2C_WR, sizeof(reg_addr), reg_addr},
        {address >> 1, RT_I2C_NO_START, length, data},
    };

    if (camera_bus == RT_NULL || data == RT_NULL || length == 0)
    {
        LOG_E("invalid OV5640 I2C write");
        return OV5640_ERROR;
    }

    if (rt_i2c_transfer(camera_bus, messages, 2) != 2)
    {
        LOG_E("I2C write failed at register 0x%04x", reg);
        return OV5640_ERROR;
    }

    if (reg == OV5640_SYSTEM_CTROL0 && length == 1 && (data[0] & 0x80U))
    {
        rt_thread_mdelay(5);
    }

    return OV5640_OK;
}

static int32_t camera_get_tick(void)
{
    return (int32_t)rt_tick_get_millisecond();
}

static rt_err_t camera_sensor_init(void)
{
    uint32_t id = 0;
    OV5640_IO_t io =
    {
        .Init = camera_i2c_init,
        .DeInit = camera_i2c_deinit,
        .Address = CAMERA_I2C_ADDRESS,
        .WriteReg = camera_i2c_write,
        .ReadReg = camera_i2c_read,
        .GetTick = camera_get_tick,
    };

    rt_pin_write(CAMERA_PWDN_PIN, PIN_HIGH);
    rt_pin_write(CAMERA_RESET_PIN, PIN_LOW);
    rt_pin_mode(CAMERA_PWDN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(CAMERA_RESET_PIN, PIN_MODE_OUTPUT);
    rt_thread_mdelay(10);
    rt_pin_write(CAMERA_PWDN_PIN, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(CAMERA_RESET_PIN, PIN_HIGH);
    rt_thread_mdelay(20);

    sensor.Mode = PARALLEL_MODE;
    if (OV5640_RegisterBusIO(&sensor, &io) != OV5640_OK ||
        OV5640_ReadID(&sensor, &id) != OV5640_OK)
    {
        LOG_E("OV5640 communication failed");
        return -RT_ERROR;
    }
    if (id != OV5640_ID)
    {
        LOG_E("unexpected camera ID 0x%04lx", (unsigned long)id);
        return -RT_ERROR;
    }
    if (OV5640_Init(&sensor, OV5640_R800x480, OV5640_RGB565) != OV5640_OK ||
        OV5640_MirrorFlipConfig(&sensor, OV5640_FLIP) != OV5640_OK)
    {
        LOG_E("OV5640 configuration failed");
        return -RT_ERROR;
    }

    LOG_I("OV5640 detected, 800x480 RGB565");
    return RT_EOK;
}

static void camera_free_buffers(void)
{
    unsigned int i;

    for (i = 0; i < 2; i++)
    {
        if (frame_allocations[i] != RT_NULL)
        {
            rt_memheap_free(frame_allocations[i]);
            frame_allocations[i] = RT_NULL;
        }
    }
}

static rt_err_t camera_alloc_buffers(void)
{
    unsigned int i;

    for (i = 0; i < 2; i++)
    {
        frame_allocations[i] = rt_memheap_alloc(&system_heap, CAMERA_FRAME_SIZE + CAMERA_FRAME_ALIGN - 1U);
        if (frame_allocations[i] == RT_NULL)
        {
            camera_free_buffers();
            LOG_E("not enough PSRAM for camera frame buffers");
            return -RT_ENOMEM;
        }

        frames[i].header.magic = LV_IMAGE_HEADER_MAGIC;
        frames[i].header.cf = LV_COLOR_FORMAT_RGB565;
        frames[i].header.w = CAMERA_WIDTH;
        frames[i].header.h = CAMERA_HEIGHT;
        frames[i].header.stride = CAMERA_WIDTH * 2U;
        frames[i].data_size = CAMERA_FRAME_SIZE;
        frames[i].data = (const uint8_t *)RT_ALIGN((rt_ubase_t)frame_allocations[i],
                                                   (rt_ubase_t)CAMERA_FRAME_ALIGN);
    }

    return RT_EOK;
}

static rt_err_t camera_dcmi_init(void)
{
    DCMIPP_ParallelConfTypeDef parallel = {0};
    DCMIPP_PipeConfTypeDef pipe = {0};

    camera_dcmi.Instance = DCMIPP;
    if (HAL_DCMIPP_Init(&camera_dcmi) != HAL_OK)
    {
        LOG_E("DCMIPP initialization failed");
        return -RT_ERROR;
    }
    dcmi_initialized = RT_TRUE;

    parallel.PCKPolarity = DCMIPP_PCKPOLARITY_RISING;
    parallel.HSPolarity = DCMIPP_HSPOLARITY_HIGH;
    parallel.VSPolarity = DCMIPP_VSPOLARITY_HIGH;
    parallel.ExtendedDataMode = DCMIPP_INTERFACE_8BITS;
    parallel.Format = DCMIPP_FORMAT_RGB565;
    parallel.SwapBits = DCMIPP_SWAPBITS_DISABLE;
    parallel.SwapCycles = DCMIPP_SWAPCYCLES_ENABLE;
    parallel.SynchroMode = DCMIPP_SYNCHRO_HARDWARE;
    pipe.FrameRate = DCMIPP_FRAME_RATE_ALL;

    if (HAL_DCMIPP_PARALLEL_SetConfig(&camera_dcmi, &parallel) != HAL_OK ||
        HAL_DCMIPP_PIPE_SetConfig(&camera_dcmi, DCMIPP_PIPE0, &pipe) != HAL_OK ||
        HAL_DCMIPP_PIPE_EnableLimitEvent(&camera_dcmi, DCMIPP_PIPE0, CAMERA_FRAME_SIZE / 4U) != HAL_OK)
    {
        LOG_E("DCMIPP configuration failed");
        return -RT_ERROR;
    }

    /* Keep the hardware write limit, but a full frame normally reaches it. */
    __HAL_DCMIPP_DISABLE_IT(&camera_dcmi, DCMIPP_IT_PIPE0_LIMIT);
    return RT_EOK;
}

static rt_err_t camera_start_capture(rt_uint8_t index)
{
    capture_index = index;
    frame_ready = RT_FALSE;
    completed_size = 0;
    capture_started_at = rt_tick_get();
    __DSB();

    if (HAL_DCMIPP_PIPE_Start(&camera_dcmi, DCMIPP_PIPE0,
                              (uint32_t)frames[index].data, DCMIPP_MODE_SNAPSHOT) != HAL_OK)
    {
        LOG_E("DCMIPP capture start failed");
        return -RT_ERROR;
    }

    capture_started = RT_TRUE;
    return RT_EOK;
}

static void camera_fail(const char *message)
{
    rt_uint32_t error = capture_error;

    if (preview_timer != RT_NULL)
    {
        lv_timer_pause(preview_timer);
    }

    if (dcmi_initialized)
    {
        HAL_NVIC_DisableIRQ(DCMIPP_IRQn);
        if (capture_started && HAL_DCMIPP_PIPE_Stop(&camera_dcmi, DCMIPP_PIPE0) != HAL_OK)
        {
            LOG_E("DCMIPP did not stop cleanly");
        }
        if (HAL_DCMIPP_DeInit(&camera_dcmi) != HAL_OK)
        {
            LOG_E("DCMIPP deinitialization failed");
        }
        dcmi_initialized = RT_FALSE;
    }

    rt_pin_write(CAMERA_PWDN_PIN, PIN_HIGH);
    if (!capture_started && !image_displayed)
    {
        camera_free_buffers();
    }

    LOG_E("%s (DCMIPP error 0x%08lx, frame %lu/%lu bytes)", message,
          (unsigned long)error, (unsigned long)completed_size, (unsigned long)CAMERA_FRAME_SIZE);
    if (status_label != RT_NULL)
    {
        lv_label_set_text(status_label, message);
#ifdef BSP_USING_MLX90640
        if (!thermal_preview_is_fullscreen())
#endif
        {
            lv_obj_remove_flag(status_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_center(status_label);
        }
    }
}

#ifdef BSP_USING_MLX90640
static void camera_view_switched(rt_bool_t thermal_fullscreen)
{
    if (thermal_fullscreen)
    {
        lv_obj_add_flag(camera_root, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_remove_flag(camera_root, LV_OBJ_FLAG_HIDDEN);
    if (dcmi_initialized && image_displayed)
    {
        lv_obj_remove_flag(preview_image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(status_label, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(preview_image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(status_label, LV_OBJ_FLAG_HIDDEN);
    }

    if (dcmi_initialized && !capture_started &&
        camera_start_capture((rt_uint8_t)(capture_index ^ 1U)) != RT_EOK)
    {
        camera_fail("OV5640: capture restart failed");
    }
}
#endif

static void camera_preview_update(lv_timer_t *timer)
{
    RT_UNUSED(timer);

    if (capture_error != 0)
    {
        camera_fail("OV5640: DCMIPP capture error");
        return;
    }

    if (frame_ready)
    {
        rt_uint8_t finished = capture_index;

        if (completed_size != CAMERA_FRAME_SIZE)
        {
            camera_fail("OV5640: incomplete frame; check wiring");
            return;
        }

        /* PSRAM is non-cacheable; the old image is no longer being rendered here. */
        __DMB();
        lv_image_cache_drop(&frames[finished]);
        lv_image_set_src(preview_image, &frames[finished]);
        image_displayed = RT_TRUE;
#ifdef BSP_USING_MLX90640
        if (thermal_preview_is_fullscreen())
        {
#ifndef BSP_USING_PHOTO_CAPTURE
            frame_ready = RT_FALSE;
            capture_started = RT_FALSE;
            return;
#endif
        }
#endif
        lv_obj_remove_flag(preview_image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(status_label, LV_OBJ_FLAG_HIDDEN);

        if (camera_start_capture((rt_uint8_t)(finished ^ 1U)) != RT_EOK)
        {
            camera_fail("OV5640: capture restart failed");
        }
    }
    else if (capture_started &&
             (rt_tick_get() - capture_started_at) >= rt_tick_from_millisecond(CAMERA_TIMEOUT_MS))
    {
        camera_fail("OV5640: no frame; check clock and wiring");
    }
}

void DCMIPP_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DCMIPP_IRQHandler(&camera_dcmi);
    rt_interrupt_leave();
}

void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t pipe)
{
    uint32_t size;

    if (hdcmipp == &camera_dcmi && pipe == DCMIPP_PIPE0)
    {
        __HAL_DCMIPP_CLEAR_FLAG(hdcmipp, DCMIPP_FLAG_PIPE0_LIMIT);
        if (HAL_DCMIPP_PIPE_GetDataCounter(hdcmipp, pipe, &size) != HAL_OK)
        {
            capture_error |= CAMERA_COUNTER_ERROR;
        }
        else
        {
            completed_size = size;
            __DMB();
            frame_ready = RT_TRUE;
        }
    }
}

void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t pipe)
{
    if (hdcmipp == &camera_dcmi && pipe == DCMIPP_PIPE0)
    {
        capture_error |= hdcmipp->ErrorCode;
    }
}

void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp)
{
    if (hdcmipp == &camera_dcmi)
    {
        capture_error |= hdcmipp->ErrorCode;
    }
}

#ifdef BSP_USING_PHOTO_CAPTURE
static lv_draw_buf_t *camera_preview_snapshot(void)
{
    lv_draw_buf_t *snapshot;

    if (!dcmi_initialized || !image_displayed || capture_error != 0 ||
        lv_obj_has_flag(preview_image, LV_OBJ_FLAG_HIDDEN))
    {
        LOG_E("camera image is not ready for a snapshot");
        return RT_NULL;
    }
    snapshot = lv_snapshot_take(camera_root, LV_COLOR_FORMAT_RGB565);
    if (snapshot == RT_NULL)
    {
        LOG_E("camera snapshot allocation or rendering failed");
    }
    return snapshot;
}
#endif

void lv_user_gui_init(void)
{
    lv_obj_t *screen;

    if (lv_display_get_default() == RT_NULL)
    {
        LOG_E("LVGL display is not available");
        return;
    }

    screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    camera_root = lv_obj_create(screen);
    if (camera_root == RT_NULL)
    {
        LOG_E("could not create camera panel");
        return;
    }
    lv_obj_remove_style_all(camera_root);
#ifdef BSP_USING_MLX90640
    lv_obj_set_size(camera_root, PREVIEW_SPLIT_WIDTH, LV_VER_RES_MAX);
    lv_obj_set_pos(camera_root, PREVIEW_SPLIT_WIDTH, 0);
#else
    lv_obj_set_size(camera_root, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_pos(camera_root, 0, 0);
#endif
    lv_obj_set_style_bg_color(camera_root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(camera_root, LV_OPA_COVER, 0);
    lv_obj_clear_flag(camera_root, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    preview_image = lv_image_create(camera_root);
    status_label = lv_label_create(camera_root);
    if (preview_image == RT_NULL || status_label == RT_NULL)
    {
        LOG_E("could not create camera preview widgets");
        return;
    }

    lv_obj_set_size(preview_image, CAMERA_WIDTH, CAMERA_HEIGHT);
    lv_obj_center(preview_image);
    lv_image_set_pivot(preview_image, CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2);
    lv_image_set_rotation(preview_image, 2700); /* LVGL uses clockwise tenths of a degree. */
    lv_image_set_scale(preview_image, LV_VER_RES_MAX * LV_SCALE_NONE / CAMERA_WIDTH);
    lv_obj_add_flag(preview_image, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(status_label, lv_pct(96));
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(status_label, "OV5640: initializing...");
    lv_obj_center(status_label);

#ifdef BSP_USING_MLX90640
    if (thermal_preview_init(screen, camera_view_switched) != RT_EOK)
    {
        LOG_E("thermal preview initialization failed");
    }
#endif

#ifdef BSP_USING_PHOTO_CAPTURE
    if (photo_capture_init(screen, camera_preview_snapshot, thermal_preview_snapshot) != RT_EOK)
    {
        LOG_E("USER2 photo capture initialization failed");
    }
#endif

    if (camera_sensor_init() != RT_EOK)
    {
        camera_fail("OV5640: initialization failed; check I2C");
        return;
    }
    if (camera_alloc_buffers() != RT_EOK || camera_dcmi_init() != RT_EOK)
    {
        camera_fail("OV5640: capture initialization failed");
        return;
    }

    preview_timer = lv_timer_create(camera_preview_update, 20, RT_NULL);
    if (preview_timer == RT_NULL)
    {
        camera_fail("OV5640: could not create preview timer");
        return;
    }
    if (camera_start_capture(0) != RT_EOK)
    {
        camera_fail("OV5640: capture start failed");
    }
}

#endif /* BSP_USING_OV5640 */
