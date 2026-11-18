/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "MLX90640_I2C_Driver.h"
#include "mlx90640_port.h"

#define LOG_TAG "mlx.i2c"
#include <drv_log.h>

static struct rt_i2c_bus_device *mlx_bus;
static rt_uint8_t read_bytes[MLX90640_EEPROM_DUMP_NUM * 2U];

rt_err_t mlx90640_port_init(void)
{
    mlx_bus = rt_i2c_bus_device_find("i2c5");
    if (mlx_bus == RT_NULL)
    {
        LOG_E("software I2C bus i2c5 not found");
        return -RT_ERROR;
    }

    return RT_EOK;
}

void MLX90640_I2CInit(void)
{
    (void)mlx90640_port_init();
}

void MLX90640_I2CFreqSet(int freq)
{
    LOG_W("software I2C rate is fixed; cannot set %d Hz", freq);
}

uint32_t MLX90640_GetTimeMs(void)
{
    return rt_tick_get_millisecond();
}

void MLX90640_DelayMs(uint32_t delayMs)
{
    rt_thread_mdelay((rt_int32_t)delayMs);
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data)
{
    rt_uint8_t address[2] = {(rt_uint8_t)(startAddress >> 8), (rt_uint8_t)startAddress};
    struct rt_i2c_msg messages[2] =
    {
        {slaveAddr, RT_I2C_WR, sizeof(address), address},
        {slaveAddr, RT_I2C_RD, (rt_uint16_t)(nMemAddressRead * 2U), read_bytes},
    };
    rt_uint16_t i;

    if (mlx_bus == RT_NULL || data == RT_NULL || nMemAddressRead == 0 ||
        nMemAddressRead > MLX90640_EEPROM_DUMP_NUM)
    {
        LOG_E("invalid MLX90640 I2C read");
        return -MLX90640_I2C_NACK_ERROR;
    }
    if (rt_i2c_transfer(mlx_bus, messages, 2) != 2)
    {
        LOG_E("I2C read failed at 0x%04x", startAddress);
        return -MLX90640_I2C_NACK_ERROR;
    }

    for (i = 0; i < nMemAddressRead; i++)
    {
        data[i] = ((uint16_t)read_bytes[2U * i] << 8) | read_bytes[2U * i + 1U];
    }
    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    rt_uint8_t bytes[4] =
    {
        (rt_uint8_t)(writeAddress >> 8), (rt_uint8_t)writeAddress,
        (rt_uint8_t)(data >> 8), (rt_uint8_t)data,
    };
    struct rt_i2c_msg message = {slaveAddr, RT_I2C_WR, sizeof(bytes), bytes};
    uint16_t readback;
    int result;

    if (mlx_bus == RT_NULL)
    {
        LOG_E("MLX90640 I2C bus is not initialized");
        return -MLX90640_I2C_NACK_ERROR;
    }
    if (rt_i2c_transfer(mlx_bus, &message, 1) != 1)
    {
        LOG_E("I2C write failed at 0x%04x", writeAddress);
        return -MLX90640_I2C_NACK_ERROR;
    }

    /* Status contains live flags and does not have a stable readback value. */
    if (writeAddress != MLX90640_STATUS_REG)
    {
        result = MLX90640_I2CRead(slaveAddr, writeAddress, 1, &readback);
        if (result != MLX90640_NO_ERROR)
        {
            return result;
        }
        if (readback != data)
        {
            LOG_E("I2C readback mismatch at 0x%04x", writeAddress);
            return -MLX90640_I2C_WRITE_ERROR;
        }
    }
    return MLX90640_NO_ERROR;
}

int MLX90640_I2CGeneralReset(void)
{
    rt_uint8_t command = 0x06;
    struct rt_i2c_msg message = {0, RT_I2C_WR, 1, &command};

    if (mlx_bus == RT_NULL || rt_i2c_transfer(mlx_bus, &message, 1) != 1)
    {
        LOG_E("MLX90640 I2C reset failed");
        return -MLX90640_I2C_NACK_ERROR;
    }
    return MLX90640_NO_ERROR;
}
