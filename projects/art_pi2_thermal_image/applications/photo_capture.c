/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#ifdef BSP_USING_PHOTO_CAPTURE

#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
#include <strings.h>
#endif
#include <rtdevice.h>
#include <drv_gpio.h>

#include "photo_capture.h"

#define LOG_TAG "photo"
#include <drv_log.h>

#if !defined(BSP_USING_SDCARD_FS) || !defined(RT_USING_DFS_ELMFAT) || RT_DFS_ELM_USE_LFN == 0
#error "Photo capture requires TF-card FAT storage with long filenames"
#endif

#define PHOTO_DIRECTORY       "/sdcard/photos"
#define PHOTO_USER2_PIN       GET_PIN(E, 14)
#define PHOTO_EVENT_SAVE      0x01U
#define PHOTO_DEBOUNCE_COUNT  3U
#define PHOTO_THREAD_PRIORITY \
    ((PKG_LVGL_THREAD_PRIO + 4 < RT_THREAD_PRIORITY_MAX) ? \
     PKG_LVGL_THREAD_PRIO + 4 : RT_THREAD_PRIORITY_MAX - 1)

enum photo_state
{
    PHOTO_IDLE,
    PHOTO_SAVING,
    PHOTO_FINISHED,
};

struct photo_job
{
    enum photo_state state;
    lv_draw_buf_t *camera;
    lv_draw_buf_t *thermal;
    rt_err_t result;
    rt_uint32_t number;
};

static struct rt_mutex photo_lock;
static struct rt_event photo_event;
static struct photo_job photo_job;
static photo_snapshot_cb camera_snapshot;
static photo_snapshot_cb thermal_snapshot;
static lv_obj_t *photo_message;
static rt_tick_t photo_message_started;
static rt_int32_t photo_button_candidate = PIN_HIGH;
static rt_int32_t photo_button_stable = PIN_HIGH;
static rt_uint8_t photo_button_samples;
static rt_bool_t photo_button_error;
static rt_uint8_t bmp_row[(LV_HOR_RES_MAX * 3U + 3U) & ~3U];

static void put_le16(rt_uint8_t *p, rt_uint16_t value)
{
    p[0] = (rt_uint8_t)value;
    p[1] = (rt_uint8_t)(value >> 8);
}

static void put_le32(rt_uint8_t *p, rt_uint32_t value)
{
    p[0] = (rt_uint8_t)value;
    p[1] = (rt_uint8_t)(value >> 8);
    p[2] = (rt_uint8_t)(value >> 16);
    p[3] = (rt_uint8_t)(value >> 24);
}

static rt_err_t write_all(int fd, const rt_uint8_t *data, rt_size_t size)
{
    while (size != 0)
    {
        ssize_t written = write(fd, data, size);
        if (written <= 0)
        {
            LOG_E("file write failed: fd=%d, result=%ld, errno=%d",
                  fd, (long)written, rt_get_errno());
            return -RT_EIO;
        }
        data += written;
        size -= (rt_size_t)written;
    }
    return RT_EOK;
}

static rt_err_t write_bmp(int fd, const lv_draw_buf_t *image)
{
    rt_uint8_t header[54] = {0};
    rt_uint32_t width = image->header.w;
    rt_uint32_t height = image->header.h;
    rt_uint32_t row_size = (width * 3U + 3U) & ~3U;
    rt_uint32_t x, y;
    uint64_t image_size = (uint64_t)row_size * height;

    if (image->header.cf != LV_COLOR_FORMAT_RGB565 || image->data == RT_NULL ||
        width == 0 || height == 0 || width > LV_HOR_RES_MAX ||
        image->header.stride < width * 2U ||
        (uint64_t)image->header.stride * height > image->data_size ||
        image_size > UINT32_MAX - sizeof(header))
    {
        LOG_E("invalid RGB565 snapshot for BMP");
        return -RT_EINVAL;
    }

    header[0] = 'B';
    header[1] = 'M';
    put_le32(header + 2, (rt_uint32_t)image_size + sizeof(header));
    put_le32(header + 10, sizeof(header));
    put_le32(header + 14, 40);
    put_le32(header + 18, width);
    put_le32(header + 22, height);
    put_le16(header + 26, 1);
    put_le16(header + 28, 24);
    put_le32(header + 34, (rt_uint32_t)image_size);
    if (write_all(fd, header, sizeof(header)) != RT_EOK)
    {
        return -RT_EIO;
    }

    for (y = height; y != 0; y--)
    {
        const rt_uint8_t *source = image->data + (y - 1U) * image->header.stride;
        rt_memset(bmp_row, 0, row_size);
        for (x = 0; x < width; x++)
        {
            rt_uint16_t pixel = (rt_uint16_t)source[2U * x] |
                                ((rt_uint16_t)source[2U * x + 1U] << 8);
            rt_uint8_t red = (pixel >> 11) & 31U;
            rt_uint8_t green = (pixel >> 5) & 63U;
            rt_uint8_t blue = pixel & 31U;
            bmp_row[3U * x] = (rt_uint8_t)((blue << 3) | (blue >> 2));
            bmp_row[3U * x + 1U] = (rt_uint8_t)((green << 2) | (green >> 4));
            bmp_row[3U * x + 2U] = (rt_uint8_t)((red << 3) | (red >> 2));
        }
        if (write_all(fd, bmp_row, row_size) != RT_EOK)
        {
            return -RT_EIO;
        }
    }
    return RT_EOK;
}

static rt_err_t next_photo_number(rt_uint32_t *next)
{
    struct stat info;
    DIR *directory;
    struct dirent *entry;
    rt_uint32_t highest = 0;
    rt_err_t result = RT_EOK;

    if (stat(PHOTO_DIRECTORY, &info) != 0)
    {
        if (rt_get_errno() != -ENOENT || mkdir(PHOTO_DIRECTORY, 0777) != 0 ||
            stat(PHOTO_DIRECTORY, &info) != 0)
        {
            LOG_E("cannot access %s: errno=%d", PHOTO_DIRECTORY, rt_get_errno());
            return -RT_EIO;
        }
    }
    if (!S_ISDIR(info.st_mode))
    {
        LOG_E("%s is not a directory", PHOTO_DIRECTORY);
        return -RT_EINVAL;
    }
    directory = opendir(PHOTO_DIRECTORY);
    if (directory == RT_NULL)
    {
        LOG_E("cannot scan photo directory: errno=%d", rt_get_errno());
        return -RT_EIO;
    }

    rt_set_errno(0);
    while ((entry = readdir(directory)) != RT_NULL)
    {
        const char *p = entry->d_name;
        rt_uint32_t value = 0;
        rt_bool_t overflow = RT_FALSE;

        if (*p < '0' || *p > '9')
        {
            continue;
        }
        while (*p >= '0' && *p <= '9')
        {
            rt_uint32_t digit = (rt_uint32_t)(*p++ - '0');
            if (value > (UINT32_MAX - digit) / 10U)
            {
                overflow = RT_TRUE;
            }
            else if (!overflow)
            {
                value = value * 10U + digit;
            }
        }
        if (rt_strcasecmp(p, "_thermal.bmp") != 0 && rt_strcasecmp(p, "_camera.bmp") != 0)
        {
            continue;
        }
        if (overflow || value == UINT32_MAX)
        {
            LOG_E("photo numbering is exhausted");
            result = -RT_EFULL;
            break;
        }
        if (value > highest)
        {
            highest = value;
        }
    }
    if (result == RT_EOK && rt_get_errno() != 0)
    {
        LOG_E("photo directory read failed: errno=%d", rt_get_errno());
        result = -RT_EIO;
    }
    if (closedir(directory) != 0)
    {
        LOG_E("photo directory close failed: errno=%d", rt_get_errno());
        result = -RT_EIO;
    }
    if (result == RT_EOK)
    {
        *next = highest + 1U;
    }
    return result;
}

static void remove_failed_photo(const char *path)
{
    if (unlink(path) != 0)
    {
        LOG_E("cannot remove incomplete photo %s: errno=%d", path, rt_get_errno());
    }
}

static rt_err_t save_photo_pair(const struct photo_job *job, rt_uint32_t *number)
{
    char thermal_path[64], camera_path[64];
    int thermal_fd, camera_fd;
    rt_err_t result = next_photo_number(number);

    if (result != RT_EOK)
    {
        return result;
    }
    rt_snprintf(thermal_path, sizeof(thermal_path), PHOTO_DIRECTORY "/%lu_thermal.bmp",
                (unsigned long)*number);
    rt_snprintf(camera_path, sizeof(camera_path), PHOTO_DIRECTORY "/%lu_camera.bmp",
                (unsigned long)*number);
    thermal_fd = open(thermal_path, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (thermal_fd < 0)
    {
        LOG_E("cannot create %s: errno=%d", thermal_path, rt_get_errno());
        return -RT_EIO;
    }
    camera_fd = open(camera_path, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (camera_fd < 0)
    {
        LOG_E("cannot create %s: errno=%d", camera_path, rt_get_errno());
        if (close(thermal_fd) != 0)
        {
            LOG_E("cannot close incomplete thermal photo: errno=%d", rt_get_errno());
        }
        remove_failed_photo(thermal_path);
        return -RT_EIO;
    }

    result = write_bmp(thermal_fd, job->thermal);
    if (result == RT_EOK)
    {
        result = write_bmp(camera_fd, job->camera);
    }
    if (result == RT_EOK && (fsync(thermal_fd) != 0 || fsync(camera_fd) != 0))
    {
        LOG_E("photo sync failed: errno=%d", rt_get_errno());
        result = -RT_EIO;
    }
    if (close(thermal_fd) != 0)
    {
        LOG_E("thermal photo close failed: errno=%d", rt_get_errno());
        result = -RT_EIO;
    }
    if (close(camera_fd) != 0)
    {
        LOG_E("camera photo close failed: errno=%d", rt_get_errno());
        result = -RT_EIO;
    }
    if (result != RT_EOK)
    {
        remove_failed_photo(thermal_path);
        remove_failed_photo(camera_path);
        return result;
    }

    LOG_I("saved %s and %s", thermal_path, camera_path);
    return RT_EOK;
}

static void photo_worker(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        struct photo_job job;
        rt_uint32_t event, number = 0;
        rt_err_t result;

        if (rt_event_recv(&photo_event, PHOTO_EVENT_SAVE,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &event) != RT_EOK)
        {
            LOG_E("photo worker event failed");
            rt_thread_mdelay(100);
            continue;
        }
        if (rt_mutex_take(&photo_lock, RT_WAITING_FOREVER) != RT_EOK)
        {
            LOG_E("cannot read photo job");
            continue;
        }
        job = photo_job;
        rt_mutex_release(&photo_lock);
        if (job.state != PHOTO_SAVING)
        {
            LOG_E("photo worker received an invalid job");
            continue;
        }

        result = save_photo_pair(&job, &number);
        if (rt_mutex_take(&photo_lock, RT_WAITING_FOREVER) != RT_EOK)
        {
            LOG_E("cannot complete photo job");
            continue;
        }
        photo_job.result = result;
        photo_job.number = number;
        photo_job.state = PHOTO_FINISHED;
        rt_mutex_release(&photo_lock);
    }
}

static void photo_show_message(const char *text)
{
    lv_label_set_text(photo_message, text);
    lv_obj_align(photo_message, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_remove_flag(photo_message, LV_OBJ_FLAG_HIDDEN);
    photo_message_started = rt_tick_get();
}

static void request_photo(void)
{
    lv_draw_buf_t *thermal, *camera;

    if (rt_mutex_take(&photo_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot request a photo");
        photo_show_message("Snapshot synchronization error");
        return;
    }
    if (photo_job.state != PHOTO_IDLE)
    {
        rt_mutex_release(&photo_lock);
        photo_show_message("Saving in progress; please wait");
        return;
    }
    rt_mutex_release(&photo_lock);

    thermal = thermal_snapshot();
    camera = thermal != RT_NULL ? camera_snapshot() : RT_NULL;
    if (thermal == RT_NULL || camera == RT_NULL)
    {
        if (thermal != RT_NULL) lv_draw_buf_destroy(thermal);
        if (camera != RT_NULL) lv_draw_buf_destroy(camera);
        photo_show_message("Snapshot unavailable; wait for both valid images");
        return;
    }
    if (rt_mutex_take(&photo_lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("cannot submit a photo");
        lv_draw_buf_destroy(thermal);
        lv_draw_buf_destroy(camera);
        photo_show_message("Snapshot synchronization error");
        return;
    }
    photo_job.camera = camera;
    photo_job.thermal = thermal;
    photo_job.state = PHOTO_SAVING;
    rt_mutex_release(&photo_lock);
    if (rt_event_send(&photo_event, PHOTO_EVENT_SAVE) != RT_EOK)
    {
        LOG_E("cannot wake photo worker");
        if (rt_mutex_take(&photo_lock, RT_WAITING_FOREVER) == RT_EOK)
        {
            photo_job.result = -RT_ERROR;
            photo_job.state = PHOTO_FINISHED;
            rt_mutex_release(&photo_lock);
        }
        else
        {
            LOG_E("cannot publish photo submission failure");
        }
    }
    photo_show_message("Saving camera and thermal images...");
}

static void photo_ui_update(lv_timer_t *timer)
{
    rt_int32_t pin = rt_pin_read(PHOTO_USER2_PIN);
    struct photo_job completed = {0};
    rt_bool_t finished = RT_FALSE;
    rt_bool_t idle = RT_FALSE;

    RT_UNUSED(timer);

    if (rt_mutex_take(&photo_lock, RT_WAITING_NO) == RT_EOK)
    {
        if (photo_job.state == PHOTO_FINISHED)
        {
            completed = photo_job;
            rt_memset(&photo_job, 0, sizeof(photo_job));
            finished = RT_TRUE;
        }
        idle = photo_job.state == PHOTO_IDLE;
        rt_mutex_release(&photo_lock);
    }
    if (finished)
    {
        char message[64];
        lv_draw_buf_destroy(completed.camera);
        lv_draw_buf_destroy(completed.thermal);
        if (completed.result == RT_EOK)
        {
            rt_snprintf(message, sizeof(message), "Saved pair %lu to TF card",
                        (unsigned long)completed.number);
            photo_show_message(message);
        }
        else
        {
            photo_show_message("Save failed; check TF card, free space and console");
        }
    }
    else if (idle && (rt_tick_get() - photo_message_started) >= rt_tick_from_millisecond(3000))
    {
        lv_obj_add_flag(photo_message, LV_OBJ_FLAG_HIDDEN);
    }

    if (pin != PIN_LOW && pin != PIN_HIGH)
    {
        photo_button_samples = 0;
        if (!photo_button_error)
        {
            LOG_E("USER2 read failed: %d", pin);
            photo_button_error = RT_TRUE;
        }
        return;
    }
    photo_button_error = RT_FALSE;
    if (pin == photo_button_candidate)
    {
        if (photo_button_samples < PHOTO_DEBOUNCE_COUNT) photo_button_samples++;
    }
    else
    {
        photo_button_candidate = pin;
        photo_button_samples = 1;
    }
    if (photo_button_samples == PHOTO_DEBOUNCE_COUNT && photo_button_stable != photo_button_candidate)
    {
        photo_button_stable = photo_button_candidate;
        if (photo_button_stable == PIN_LOW) request_photo();
    }
}

rt_err_t photo_capture_init(lv_obj_t *screen, photo_snapshot_cb camera_cb,
                            photo_snapshot_cb thermal_cb)
{
    rt_thread_t thread;
    lv_timer_t *timer;

    if (screen == RT_NULL || camera_cb == RT_NULL || thermal_cb == RT_NULL)
    {
        LOG_E("invalid photo capture configuration");
        return -RT_EINVAL;
    }
    if (rt_mutex_init(&photo_lock, "photolk", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        LOG_E("cannot create photo mutex");
        return -RT_ERROR;
    }
    if (rt_event_init(&photo_event, "photoev", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        LOG_E("cannot create photo event");
        rt_mutex_detach(&photo_lock);
        return -RT_ERROR;
    }
    thread = rt_thread_create("photo", photo_worker, RT_NULL, 8192, PHOTO_THREAD_PRIORITY, 10);
    if (thread == RT_NULL)
    {
        LOG_E("cannot create photo writer");
        rt_event_detach(&photo_event);
        rt_mutex_detach(&photo_lock);
        return -RT_ENOMEM;
    }
    photo_message = lv_label_create(screen);
    if (photo_message == RT_NULL)
    {
        LOG_E("cannot create photo status label");
        rt_thread_delete(thread);
        rt_event_detach(&photo_event);
        rt_mutex_detach(&photo_lock);
        return -RT_ENOMEM;
    }
    lv_obj_set_width(photo_message, lv_pct(96));
    lv_obj_set_style_text_align(photo_message, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(photo_message, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(photo_message, lv_color_white(), 0);
    lv_obj_set_style_bg_color(photo_message, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(photo_message, LV_OPA_80, 0);
    lv_obj_add_flag(photo_message, LV_OBJ_FLAG_HIDDEN);
    timer = lv_timer_create(photo_ui_update, 20, RT_NULL);
    if (timer == RT_NULL || rt_thread_startup(thread) != RT_EOK)
    {
        LOG_E("cannot start photo capture service");
        if (timer != RT_NULL) lv_timer_delete(timer);
        lv_obj_delete(photo_message);
        rt_thread_delete(thread);
        rt_event_detach(&photo_event);
        rt_mutex_detach(&photo_lock);
        return -RT_ERROR;
    }
    camera_snapshot = camera_cb;
    thermal_snapshot = thermal_cb;
    rt_pin_mode(PHOTO_USER2_PIN, PIN_MODE_INPUT_PULLUP);
    LOG_I("USER2 saves paired BMP snapshots in " PHOTO_DIRECTORY);
    return RT_EOK;
}

#endif /* BSP_USING_PHOTO_CAPTURE */
