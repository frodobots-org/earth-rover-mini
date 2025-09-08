

/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-21     yuxing       the first version
 */
#define DBG_TAG "INA226"
#define DBG_LVL DBG_LOG
#include "INA226.h"
#include <rtdbg.h>

#define INA226_I2CBUS_NAME  "i2c2"
struct rt_i2c_bus_device *INA226_i2c_bus;

void INA226_init(void)
{
    /********************** Software I²C Initialization ***************************/
    rt_err_t ret = 0;
    rt_uint8_t res;
    rt_device_t dev;
    dev = rt_device_find(INA226_I2CBUS_NAME);

    if (dev == RT_NULL)
    {
        LOG_E("can't find INA226 %s device\r\n", INA226_I2CBUS_NAME);
        return -RT_ERROR;
    }

    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        LOG_E("can't open INA226 %s device\r\n", INA226_I2CBUS_NAME);
        return -RT_ERROR;
    }
    // Get I2C device handle
    INA226_i2c_bus = (struct rt_i2c_bus_device *)dev;
    /********************** Software I²C Initialization ***************************/

    /********************** INA226 Initialization ***************************/
    /**
    * Configure conversion time = 8.244ms, averaging = 16 samples,
    * mode = shunt + bus continuous mode
    * → Total data conversion time = 8.244 * 16 = 131.9 ms
    */
    INA226_SetConfig(0x45FF);

    /** Max current of this device ≈ 3.6A
    * Shunt voltage max = 32768 * 0.0000025V = 0.08192V
    * Set shunt voltage-to-current conversion:
    *   - Shunt resistor = 0.01Ω
    *   - Resolution = 0.2mA
    *
    * Formula 1:
    *   Current_LSB = Expected Max Current / 2^15
    *   Current_LSB = 5 / 32768 = 0.000152A (~0.15mA) → choose 0.2mA
    *
    * Formula 2:
    *   CAL = 0.00512 / (Current_LSB * R)
    *   CAL = 0.00512 / (0.0002 * 0.01) = 2560 = 0x0a00
    */
    INA226_SetCalibrationReg(0x0a00);
    /********************** INA226 Initialization ***************************/
}

/**
**************************************************
* Description: Read BUS voltage and convert to float
**************************************************
*/
float INA226_GetBusV(void)
{
    uint16_t regData;
    float fVoltage;
    regData = INA226_GetBusVReg();
    fVoltage = regData * 0.00125f; /* LSB = 1.25mV */
    return fVoltage;
}

/**
**************************************************
* Description: Read current and convert to float
**************************************************
*/
float INA226_GetCurrent(void)
{
    uint16_t regData;
    float fCurrent;
    regData = INA226_GetCurrentReg();
    if(regData >= 0x8000)   regData = 0;  // Handle negative/overflow
    fCurrent = regData * 0.0002f; /* LSB = 0.2mA (user configured) */
    return fCurrent;
}

/**
**************************************************
* Description: Read power and convert to float
**************************************************
*/
float INA226_GetPower(void)
{
    uint16_t regData;
    float fPower;
    regData = INA226_GetPowerReg();
    fPower = regData * 0.005f; /* LSB = Current LSB * 25 = 0.005W */
    return fPower;
}

rt_err_t INA226_SetConfig(uint16_t ConfigWord)
{
    uint8_t SentTable[2];
    SentTable[0] = (ConfigWord & 0xFF00) >> 8;
    SentTable[1] = (ConfigWord & 0x00FF);
    return INA226_soft_write_Len(INA226_ADDRESS, INA226_CONFIG, 2, SentTable);
}

uint16_t INA226_GetBusVReg(void)
{
    uint8_t ReceivedTable[2];
    if (INA226_soft_read_Len(INA226_ADDRESS, INA226_BUSV, 2, ReceivedTable) != RT_EOK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint8_t INA226_SetCalibrationReg(uint16_t ConfigWord)
{
    uint8_t SentTable[2];
    SentTable[0] = (ConfigWord & 0xFF00) >> 8;
    SentTable[1] = (ConfigWord & 0x00FF);
    return INA226_soft_write_Len(INA226_ADDRESS, INA226_CALIB, 2, SentTable);
}

uint16_t INA226_GetPowerReg(void)
{
    uint8_t ReceivedTable[2];
    if (INA226_soft_read_Len(INA226_ADDRESS, INA226_POWER, 2, ReceivedTable) != RT_EOK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

uint16_t INA226_GetCurrentReg(void)
{
    uint8_t ReceivedTable[2];
    if (INA226_soft_read_Len(INA226_ADDRESS, INA226_CURRENT, 2, ReceivedTable) != RT_EOK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0] << 8 | ReceivedTable[1]);
}

/**
 * Software I²C – INA226 multi-byte write
 */
rt_err_t INA226_soft_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date)
{
    rt_uint8_t buf[len + 1];
    rt_uint8_t i;
    buf[0] = reg;
    for (i = 0; i < len; i++)
    {
        buf[i + 1] = date[i];
    }
    if (rt_i2c_master_send(INA226_i2c_bus, addr, 0, buf, len + 1) == len + 1)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * Software I²C – INA226 multi-byte read
 */
rt_err_t INA226_soft_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = buf;
    msgs[1].len   = len;

    if (rt_i2c_transfer(INA226_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * Software I²C – INA226 single-byte write
 */
rt_err_t INA226_soft_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];

    buf[0] = reg;
    buf[1] = data;

    if (rt_i2c_master_send(INA226_i2c_bus, INA226_ADDRESS, 0, buf, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * Software I²C – INA226 single-byte read
 */
rt_err_t INA226_soft_read_reg(rt_uint8_t reg, rt_uint8_t *data)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = INA226_ADDRESS;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = INA226_ADDRESS;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(INA226_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}
