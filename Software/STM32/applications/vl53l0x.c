/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-17     Aaron       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "SENSOR"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "board.h"
#include "sensor.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "vl53l0x_sensor_v1.h"

struct rt_i2c_bus_device *i2c_bus1 = RT_NULL;
#define VL530LX_DEFAULT_ADDR 0x29

#ifdef WHEELTEC
#define XSHUT1 GET_PIN(C, 2) // LEFT INNER
#define XSHUT2 GET_PIN(C, 2) // RIGHT INNER
#define XSHUT3 GET_PIN(C, 3) // LEFT OUTER
#define XSHUT4 GET_PIN(C, 2) // RIGHT OUTER
#else
#define XSHUT1 GET_PIN(B, 1) // LEFT INNER
#define XSHUT2 GET_PIN(E, 0) // RIGHT INNER
#define XSHUT3 GET_PIN(E, 8) // LEFT OUTER
#define XSHUT4 GET_PIN(D, 0) // RIGHT OUTER
#endif

#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

typedef struct robot_tof_t {
    rt_int8_t enabled;
    rt_uint8_t addr;
    rt_int8_t xshut;
    VL53L0X_Dev_t vl53l0x_dev;
} robot_tof_t;

static robot_tof_t robot_tofs[4] = {
    { .enabled = 0, .addr = 0x59, .xshut = XSHUT4 },
    { .enabled = 0, .addr = 0x49, .xshut = XSHUT3 },
    { .enabled = 0, .addr = 0x39, .xshut = XSHUT2 },
    { .enabled = 0, .addr = 0x29, .xshut = XSHUT1 },
};

int32_t vl53l0x_reg_write(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint16_t data_size)
{
    struct rt_i2c_msg msg[2]={0};

    msg[0].addr     = slave_addr;
    msg[0].flags    = RT_I2C_WR;
    msg[0].len      = 1;
    msg[0].buf      = &reg;
    msg[1].addr     = slave_addr;
    msg[1].flags    = RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len      = data_size;
    msg[1].buf      = data;
    if(rt_i2c_transfer(i2c_bus1, msg, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        LOG_E("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

int32_t vl53l0x_reg_read(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint16_t data_size)
{
    struct rt_i2c_msg msg[2]={0};

    msg[0].addr  = slave_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = &reg;
    msg[1].addr  = slave_addr;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = data_size;
    msg[1].buf   = data;

    if(rt_i2c_transfer(i2c_bus1, msg, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        LOG_E("i2c bus read failed!\r\n");
        return -RT_ERROR;
    }
}

extern VL53L0X_Error vl53l0x_single_ranging_mode(VL53L0X_Dev_t *pdev);

int vl53l0x_get_value(VL53L0X_Dev_t *pdev)
{
    static VL53L0X_RangingMeasurementData_t vl53l0x_data;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    status = VL53L0X_PerformSingleRangingMeasurement(pdev, &vl53l0x_data);
    if (VL53L0X_ERROR_NONE != status)
    {
        return -1;
    }

    return vl53l0x_data.RangeMilliMeter;
}

void vl53l0x_poweroff(robot_tof_t *robot_tof)
{
    rt_pin_mode(robot_tof->xshut, PIN_MODE_OUTPUT);
    rt_pin_write(robot_tof->xshut, PIN_LOW);
}

void vl53l0x_poweron(robot_tof_t *robot_tof)
{
    rt_pin_mode(robot_tof->xshut, PIN_MODE_OUTPUT);
    rt_pin_write(robot_tof->xshut, PIN_HIGH);
}

void vl53l0x_modify_addr(robot_tof_t *robot_tof, uint8_t addr)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;
    buf[0] = 0x8A;
    buf[1] = addr & 0x7f;
    msgs.addr = VL530LX_DEFAULT_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 2;

    if (i2c_bus1 == RT_NULL)
    {
        LOG_E("can't find %s device!\n", "i2c1");
    }
    else
    {
        rt_i2c_transfer(i2c_bus1, &msgs, 1);
    }
}

void vl53l0x_init_all()
{
    VL53L0X_DeviceInfo_t vl53l0x_info;
    i2c_bus1 = (struct rt_i2c_bus_device *)rt_device_find("i2c1");

    // power off all
    for (int i = 0; i < 2; i++)
    {
        rt_pin_mode(robot_tofs[i].xshut, PIN_MODE_OUTPUT);
        vl53l0x_poweroff(&robot_tofs[i]);
    }
    rt_thread_mdelay(100);

    for (int i = 0; i < 2; i++)
    {
        // modify one by one
        vl53l0x_poweron(&robot_tofs[i]);
        vl53l0x_modify_addr(&robot_tofs[i], robot_tofs[i].addr);
        rt_thread_mdelay(100);
    }

    for (int i = 0; i < 2; i++)
    {
        robot_tofs[i].vl53l0x_dev.comms_type = 1,
        robot_tofs[i].vl53l0x_dev.comms_speed_khz = 400,
        robot_tofs[i].vl53l0x_dev.I2cDevAddr = (rt_uint32_t)(robot_tofs[i].addr) & 0xff;
        robot_tofs[i].vl53l0x_dev.RegRead = vl53l0x_reg_read;
        robot_tofs[i].vl53l0x_dev.RegWrite = vl53l0x_reg_write;

        /* vl53l0x init */
        if (VL53L0X_ERROR_NONE != VL53L0X_DataInit(&robot_tofs[i].vl53l0x_dev))
        {
            LOG_E("vl53l0x data init failed\r\n");
            continue;
        }

        /* vl53l0x read version */
        if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&robot_tofs[i].vl53l0x_dev, &vl53l0x_info))
        {
        }

        /* set single ranging mode */
        if (VL53L0X_ERROR_NONE != vl53l0x_single_ranging_mode(&robot_tofs[i].vl53l0x_dev))
        {
            LOG_E("vl53l0x single ranging init failed\r\n");
            continue;
        }

        robot_tofs[i].enabled = 1;
        rt_thread_mdelay(100);
    }
}

void sensor_thread_entry(void *parameter)
{
    vl53l0x_init_all();

    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            if (robot_tofs[i].enabled)
            {
                LOG_D("TOF[%d]: %d", i, vl53l0x_get_value(&robot_tofs[i].vl53l0x_dev));
            }
        }
        rt_thread_mdelay(1000);
    }
}
