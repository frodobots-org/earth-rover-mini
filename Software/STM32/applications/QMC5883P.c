/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-12     yuxing       the first version
 */
#define DBG_TAG "qmc5883p"
#define DBG_LVL DBG_LOG
#include "qmc5883p.h"

rt_err_t qmc5883p_init( void )
{
    uint8_t set = 0;
    uint8_t Config=0;
    uint8_t ChipID = 0;
    /**ID查询**/
    if(qmc5883p_read_reg(QMC5883P_CHIP_ID_REG,&ChipID))
    {
        LOG_E("qmc5883p device no find\r\n");
        return RT_ERROR;
    }
    if(ChipID == 0x80){
        LOG_I("qmc5883p device is find\r\n");

    }
    else {
        LOG_E("qmc5883p device find error!\r\n");
        return RT_ERROR;
    }

    set = 0x40;
    if(qmc5883p_write_reg(0x0D,set))
    {
        LOG_E("set qmc5883p 0x0D error\r\n");
        return RT_ERROR;
    }

    set = 0x06;
    if(qmc5883p_write_reg(0x29,set))
    {
        LOG_E("set qmc5883p 0x29 error\r\n");
        return RT_ERROR;
    }

    Config = 0xCB;//ODR=100
    set = Config;
    if(qmc5883p_write_reg(QMC5883P_CTL_REG_ONE,set))
    {
        LOG_E("set qmc5883p_CTL_REG_ONE error\r\n");
        return RT_ERROR;
    }

    Config = 0x08;//+-8G
    set = Config;
    if(qmc5883p_write_reg(QMC5883P_CTL_REG_TWO,set))
    {
        LOG_E("set qmc5883p_CTL_REG_ONE error\r\n");
        return RT_ERROR;
    }
    return RT_EOK;
}

/**
 * 软件模拟IIC-QMC5883P-写函数
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t qmc5883p_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date)
{
    rt_uint8_t buf[len + 1];
    rt_uint8_t i;
    buf[0] = reg;
    for (i = 0; i < len; i++)
    {
        buf[i + 1] = date[i];
    }
    if (rt_i2c_master_send(mpu6050_i2c_bus, addr, 0, buf, len + 1) == len + 1)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-QMC5883P-读函数
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t qmc5883p_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
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

    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}
/**
 * 软件模拟IIC-QMC5883P-单个字节写
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t qmc5883p_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];

    buf[0] = reg;
    buf[1] = data;

    if (rt_i2c_master_send(mpu6050_i2c_bus, QMC5883P_IIC_ADDR, 0, buf, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-QMC5883P-单个字节读
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t qmc5883p_read_reg(rt_uint8_t reg, rt_uint8_t *data)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = QMC5883P_IIC_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = QMC5883P_IIC_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}
rt_err_t qmc5883p_read_mag_xyz( int32_t* data )
{
    uint8_t val[6] = {0};
    if(qmc5883p_read_Len( QMC5883P_IIC_ADDR , QMC5883P_DATA_OUT_X_LSB_REG , 6 , val))
    {
        LOG_E("get QMC5883P data error!\r\n");
        return -RT_ERROR;
    }
    int32_t data_t[3] = {0};
    data_t[0] = (int16_t)((val[1] << 8) | val[0]);
    data_t[1] = (int16_t)((val[3] << 8) | val[2]);
    data_t[2] = (int16_t)((val[5] << 8) | val[4]);
    for(int i = 0;i < 3;i++)
        *(data + i ) = data_t[i];
    return RT_EOK;
}
