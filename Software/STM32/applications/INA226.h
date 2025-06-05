/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-21     yuxing       the first version
 */
#ifndef APPLICATIONS_INA226_H_
#define APPLICATIONS_INA226_H_
#include "drv_soft_i2c.h"
#include "main.h"

#define INA226_COM_PORT hi2c1       /*通讯使用的IIC接口*/

#define INA226_ADDRESS      0x40    /*INA226的IIC 7位地址*/
#define INA226_I2C_TIMEOUT  10      /*IIC通讯超时*/

#define INA226_CALIB_VAL 1024
#define INA226_CURRENTLSB 0.5F // mA/bit
#define INA226_CURRENTLSB_INV 1/INA226_CURRENTLSB // bit/mA
#define INA226_POWERLSB_INV 1/(INA226_CURRENTLSB*25) // bit/mW

#define INA226_CONFIG   0x00 // Configuration Register (R/W)初始值4127
#define INA226_SHUNTV   0x01 // Shunt Voltage (R)初始值0，分流电压测量值
#define INA226_BUSV     0x02 // Bus Voltage (R)初始值0，总线电压测量值
#define INA226_POWER    0x03 // Power (R)初始值0，输出功率测量值
#define INA226_CURRENT  0x04 // Current (R)初始值0，分流电阻电流计算值
#define INA226_CALIB    0x05 // Calibration (R/W)，设置全量程和电流LSB
#define INA226_MASK     0x06 // Mask/Enable (R/W)，报警设置和转换准备标志
#define INA226_ALERTL   0x07 // Alert Limit (R/W)，报警阈值
#define INA226_MANUF_ID 0xFE // Manufacturer ID (R)，0x5449
#define INA226_DIE_ID   0xFF // Die ID (R),0x2260

#define INA226_MODE_POWER_DOWN          (0<<0) // Power-Down
#define INA226_MODE_TRIG_SHUNT_VOLTAGE  (1<<0) // Shunt Voltage, Triggered
#define INA226_MODE_TRIG_BUS_VOLTAGE    (2<<0) // Bus Voltage, Triggered
#define INA226_MODE_TRIG_SHUNT_AND_BUS  (3<<0) // Shunt and Bus, Triggered
#define INA226_MODE_POWER_DOWN2         (4<<0) // Power-Down
#define INA226_MODE_CONT_SHUNT_VOLTAGE  (5<<0) // Shunt Voltage, Continuous
#define INA226_MODE_CONT_BUS_VOLTAGE    (6<<0) // Bus Voltage, Continuous
#define INA226_MODE_CONT_SHUNT_AND_BUS  (7<<0) // Shunt and Bus, Continuous

// Shunt Voltage Conversion Time
#define INA226_VSH_140uS    (0<<3)
#define INA226_VSH_204uS    (1<<3)
#define INA226_VSH_332uS    (2<<3)
#define INA226_VSH_588uS    (3<<3)
#define INA226_VSH_1100uS   (4<<3)
#define INA226_VSH_2116uS   (5<<3)
#define INA226_VSH_4156uS   (6<<3)
#define INA226_VSH_8244uS   (7<<3)

// Bus Voltage Conversion Time (VBUS CT Bit Settings[6-8])
#define INA226_VBUS_140uS   (0<<6)
#define INA226_VBUS_204uS   (1<<6)
#define INA226_VBUS_332uS   (2<<6)
#define INA226_VBUS_588uS   (3<<6)
#define INA226_VBUS_1100uS  (4<<6)
#define INA226_VBUS_2116uS  (5<<6)
#define INA226_VBUS_4156uS  (6<<6)
#define INA226_VBUS_8244uS  (7<<6)

// Averaging Mode (AVG Bit Settings[9-11])
#define INA226_AVG_1    (0<<9)
#define INA226_AVG_4    (1<<9)
#define INA226_AVG_16   (2<<9)
#define INA226_AVG_64   (3<<9)
#define INA226_AVG_128  (4<<9)
#define INA226_AVG_256  (5<<9)
#define INA226_AVG_512  (6<<9)
#define INA226_AVG_1024 (7<<9)

// Reset Bit (RST bit [15])
#define INA226_RESET_ACTIVE     (1<<15)
#define INA226_RESET_INACTIVE   (0<<15)

// Mask/Enable Register
#define INA226_MER_SOL  (1<<15) // Shunt Voltage Over-Voltage
#define INA226_MER_SUL  (1<<14) // Shunt Voltage Under-Voltage
#define INA226_MER_BOL  (1<<13) // Bus Voltagee Over-Voltage
#define INA226_MER_BUL  (1<<12) // Bus Voltage Under-Voltage
#define INA226_MER_POL  (1<<11) // Power Over-Limit
#define INA226_MER_CNVR (1<<10) // Conversion Ready
#define INA226_MER_AFF  (1<<4) // Alert Function Flag
#define INA226_MER_CVRF (1<<3) // Conversion Ready Flag
#define INA226_MER_OVF  (1<<2) // Math Overflow Flag
#define INA226_MER_APOL (1<<1) // Alert Polarity Bit
#define INA226_MER_LEN  (1<<0) // Alert Latch Enable

#define INA226_MANUF_ID_DEFAULT     0x5449
#define INA226_DIE_ID_DEFAULT       0x2260

/**函数声明**/
void INA226_init(void);
float INA226_GetBusV(void);
float INA226_GetCurrent(void);
float INA226_GetPower(void);
rt_err_t INA226_SetConfig(uint16_t ConfigWord);
uint16_t INA226_GetBusVReg(void);
uint8_t INA226_SetCalibrationReg(uint16_t ConfigWord);
uint16_t INA226_GetPowerReg(void);
uint16_t INA226_GetCurrentReg(void);

rt_err_t INA226_soft_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date);
rt_err_t INA226_soft_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf);
rt_err_t INA226_soft_write_reg(rt_uint8_t reg, rt_uint8_t data);
rt_err_t INA226_soft_read_reg(rt_uint8_t reg, rt_uint8_t *data);

#endif /* APPLICATIONS_INA226_H_ */
