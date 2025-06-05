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
    /**********************软件IIC初始化***************************/
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
        LOG_E("can't INA226 mpu6050 %s device\r\n", INA226_I2CBUS_NAME);
        return -RT_ERROR;
    }
    //获取i2c设备句柄
    INA226_i2c_bus = (struct rt_i2c_bus_device *)dev;
    /**********************软件IIC初始化***************************/

    /**********************INA226初始化***************************/
    /**
    * 设置转换时间8.244ms,求平均值次数16，设置模式为分流和总线连续模式
    * 总数据转换时间 = 8.244*16 = 131.9ms
    */
    INA226_SetConfig(0x45FF);

    /**本机最大电流约为3.6A
    * 分流电阻最大电压 = 32768 * 0.0000025V = 0.08192V
    * 设置分流电压转电流转换参数:电阻0.01R，分辨率0.2mA
    * 公式1
    * Current_LSB = 预期最大电流 / 2^15
    * Current_LSB = 5 / 32768 = 0.000152A ,选0.2ma
    * 公式2
    * CAL = 0.00512/(Current_LSB*R)
    * CAL = 0.00512/(0.0002*0.01)=2560 = 0x0a00
    */
    INA226_SetCalibrationReg(0x0a00);
    /**********************INA226初始化***************************/

}
/**
**************************************************
* 说明：读取BUS电压，并转换为浮点数据
**************************************************
*/
float INA226_GetBusV(void)
{
    uint16_t regData;
    float fVoltage;
    regData = INA226_GetBusVReg();
    fVoltage = regData * 0.00125f;/*电压的LSB = 1.25mV*/
    return fVoltage;
}
/**
**************************************************
* 说明：读取电流，并转换为浮点数据
**************************************************
*/
float INA226_GetCurrent(void)
{
    uint16_t regData;
    float fCurrent;
    regData = INA226_GetCurrentReg();
    if(regData >= 0x8000)   regData = 0;
    fCurrent = regData * 0.0002f;/*电流的LSB = 0.2mA，由用户配置*/
    return fCurrent;
}
/**
**************************************************
* 说明：读取功率，并转换为浮点数据
**************************************************
*/
float INA226_GetPower(void)
{
    uint16_t regData;
    float fPower;
    regData = INA226_GetPowerReg();
    fPower = regData * 0.005f;/*功率的LSB = 电流的LSB*25*/
    return fPower;
}

rt_err_t INA226_SetConfig(uint16_t ConfigWord)
{
    uint8_t SentTable[2];
    SentTable[0] = (ConfigWord & 0xFF00) >> 8;
    SentTable[1] = (ConfigWord & 0x00FF);
    return INA226_soft_write_Len(INA226_ADDRESS,INA226_CONFIG,2,SentTable);
}

uint16_t INA226_GetBusVReg(void)
{
    uint8_t ReceivedTable[2];
    if (INA226_soft_read_Len(INA226_ADDRESS,INA226_BUSV,2,ReceivedTable) != RT_EOK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_SetCalibrationReg(uint16_t ConfigWord)
{
    uint8_t SentTable[2];
    SentTable[0] = (ConfigWord & 0xFF00) >> 8;
    SentTable[1] = (ConfigWord & 0x00FF);
    return INA226_soft_write_Len(INA226_ADDRESS,INA226_CALIB,2,SentTable);
}

uint16_t INA226_GetPowerReg(void)
{
    uint8_t ReceivedTable[2];
    if (INA226_soft_read_Len(INA226_ADDRESS,INA226_POWER,2,ReceivedTable) != RT_EOK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetCurrentReg(void)
{
    uint8_t ReceivedTable[2];
    if (INA226_soft_read_Len(INA226_ADDRESS,INA226_CURRENT,2,ReceivedTable) != RT_EOK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

/**
 * 软件模拟IIC-INA226写函数(多字节)
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
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
 * 软件模拟IIC-INA226读函数(多字节)
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
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
 * 软件模拟IIC-INA226-单个字节写
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
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
 * 软件模拟IIC-INA226-单个字节读
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
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
