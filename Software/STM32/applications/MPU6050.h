/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-10-24     yuxing       the first version
 */
#ifndef APPLICATIONS_MPU6050_H_
#define APPLICATIONS_MPU6050_H_

#include "Hardware_i2c.h"
#include "drv_soft_i2c.h"
#include "./MPU6050_DMP/inv_mpu.h"
#include "./MPU6050_DMP/inv_mpu_dmp_motion_driver.h"

#define IIC_SOFT                        //切换软件模拟IIC(IIC1总线所有设备均可切换) - Switch to software-simulated I2C (all devices on IIC1 bus can switch)

extern struct rt_i2c_bus_device *mpu6050_i2c_bus;
//MPU6050 AD0控制脚 - MPU6050 AD0 control pin
#define MPU_AD0_CTRL            PAout(15)   //控制AD0电平,从而控制MPU地址 - Control AD0 pin level to set MPU6050 I2C address


//#define MPU_ACCEL_OFFS_REG        0X06    //accel_offs寄存器,可读取版本号,寄存器手册未提到 - accel_offs register, can read version ID (not in datasheet)
//#define MPU_PROD_ID_REG           0X0C    //prod id寄存器,在寄存器手册未提到 - product ID register (not in datasheet)

#define MPU_SELF_TESTX_REG      0X0D    //自检寄存器X - Self-test register X
#define MPU_SELF_TESTY_REG      0X0E    //自检寄存器Y - Self-test register Y
#define MPU_SELF_TESTZ_REG      0X0F    //自检寄存器Z - Self-test register Z
#define MPU_SELF_TESTA_REG      0X10    //自检寄存器A - Self-test register A
#define MPU_SAMPLE_RATE_REG     0X19    //采样频率分频器 - Sample rate divider
#define MPU_CFG_REG             0X1A    //配置寄存器 - Configuration register
#define MPU_GYRO_CFG_REG        0X1B    //陀螺仪配置寄存器 - Gyroscope config register
#define MPU_ACCEL_CFG_REG       0X1C    //加速度计配置寄存器 - Accelerometer config register
#define MPU_MOTION_DET_REG      0X1F    //运动检测阀值设置寄存器 - Motion detection threshold register
#define MPU_FIFO_EN_REG         0X23    //FIFO使能寄存器 - FIFO enable register
#define MPU_I2CMST_CTRL_REG     0X24    //IIC主机控制寄存器 - I2C Master control register
#define MPU_I2CSLV0_ADDR_REG    0X25    //IIC从机0器件地址寄存器 - I2C Slave 0 device address
#define MPU_I2CSLV0_REG         0X26    //IIC从机0数据地址寄存器 - I2C Slave 0 register address
#define MPU_I2CSLV0_CTRL_REG    0X27    //IIC从机0控制寄存器 - I2C Slave 0 control register
#define MPU_I2CSLV1_ADDR_REG    0X28    //IIC从机1器件地址寄存器 - I2C Slave 1 device address
#define MPU_I2CSLV1_REG         0X29    //IIC从机1数据地址寄存器 - I2C Slave 1 register address
#define MPU_I2CSLV1_CTRL_REG    0X2A    //IIC从机1控制寄存器 - I2C Slave 1 control register
#define MPU_I2CSLV2_ADDR_REG    0X2B    //IIC从机2器件地址寄存器 - I2C Slave 2 device address
#define MPU_I2CSLV2_REG         0X2C    //IIC从机2数据地址寄存器 - I2C Slave 2 register address
#define MPU_I2CSLV2_CTRL_REG    0X2D    //IIC从机2控制寄存器 - I2C Slave 2 control register
#define MPU_I2CSLV3_ADDR_REG    0X2E    //IIC从机3器件地址寄存器 - I2C Slave 3 device address
#define MPU_I2CSLV3_REG         0X2F    //IIC从机3数据地址寄存器 - I2C Slave 3 register address
#define MPU_I2CSLV3_CTRL_REG    0X30    //IIC从机3控制寄存器 - I2C Slave 3 control register
#define MPU_I2CSLV4_ADDR_REG    0X31    //IIC从机4器件地址寄存器 - I2C Slave 4 device address
#define MPU_I2CSLV4_REG         0X32    //IIC从机4数据地址寄存器 - I2C Slave 4 register address
#define MPU_I2CSLV4_DO_REG      0X33    //IIC从机4写数据寄存器 - I2C Slave 4 data out register
#define MPU_I2CSLV4_CTRL_REG    0X34    //IIC从机4控制寄存器 - I2C Slave 4 control register
#define MPU_I2CSLV4_DI_REG      0X35    //IIC从机4读数据寄存器 - I2C Slave 4 data in register

#define MPU_I2CMST_STA_REG      0X36    //IIC主机状态寄存器 - I2C Master status register
#define MPU_INTBP_CFG_REG       0X37    //中断/旁路设置寄存器 - Interrupt/Bypass config register
#define MPU_INT_EN_REG          0X38    //中断使能寄存器 - Interrupt enable register
#define MPU_INT_STA_REG         0X3A    //中断状态寄存器 - Interrupt status register

#define MPU_ACCEL_XOUTH_REG     0X3B    //加速度值,X轴高8位寄存器 - Acceleration X high byte
#define MPU_ACCEL_XOUTL_REG     0X3C    //加速度值,X轴低8位寄存器 - Acceleration X low byte
#define MPU_ACCEL_YOUTH_REG     0X3D    //加速度值,Y轴高8位寄存器 - Acceleration Y high byte
#define MPU_ACCEL_YOUTL_REG     0X3E    //加速度值,Y轴低8位寄存器 - Acceleration Y low byte
#define MPU_ACCEL_ZOUTH_REG     0X3F    //加速度值,Z轴高8位寄存器 - Acceleration Z high byte
#define MPU_ACCEL_ZOUTL_REG     0X40    //加速度值,Z轴低8位寄存器 - Acceleration Z low byte

#define MPU_TEMP_OUTH_REG       0X41    //温度值高八位寄存器 - Temperature high byte
#define MPU_TEMP_OUTL_REG       0X42    //温度值低8位寄存器 - Temperature low byte

#define MPU_GYRO_XOUTH_REG      0X43    //陀螺仪值,X轴高8位寄存器 - Gyro X high byte
#define MPU_GYRO_XOUTL_REG      0X44    //陀螺仪值,X轴低8位寄存器 - Gyro X low byte
#define MPU_GYRO_YOUTH_REG      0X45    //陀螺仪值,Y轴高8位寄存器 - Gyro Y high byte
#define MPU_GYRO_YOUTL_REG      0X46    //陀螺仪值,Y轴低8位寄存器 - Gyro Y low byte
#define MPU_GYRO_ZOUTH_REG      0X47    //陀螺仪值,Z轴高8位寄存器 - Gyro Z high byte
#define MPU_GYRO_ZOUTL_REG      0X48    //陀螺仪值,Z轴低8位寄存器 - Gyro Z low byte

#define MPU_I2CSLV0_DO_REG      0X63    //IIC从机0数据寄存器 - I2C Slave 0 data register
#define MPU_I2CSLV1_DO_REG      0X64    //IIC从机1数据寄存器 - I2C Slave 1 data register
#define MPU_I2CSLV2_DO_REG      0X65    //IIC从机2数据寄存器 - I2C Slave 2 data register
#define MPU_I2CSLV3_DO_REG      0X66    //IIC从机3数据寄存器 - I2C Slave 3 data register

#define MPU_I2CMST_DELAY_REG    0X67    //IIC主机延时管理寄存器 - I2C Master delay management
#define MPU_SIGPATH_RST_REG     0X68    //信号通道复位寄存器 - Signal path reset
#define MPU_MDETECT_CTRL_REG    0X69    //运动检测控制寄存器 - Motion detection control
#define MPU_USER_CTRL_REG       0X6A    //用户控制寄存器 - User control register
#define MPU_PWR_MGMT1_REG       0X6B    //电源管理寄存器1 - Power management register 1
#define MPU_PWR_MGMT2_REG       0X6C    //电源管理寄存器2 - Power management register 2
#define MPU_FIFO_CNTH_REG       0X72    //FIFO计数寄存器高八位 - FIFO count high byte
#define MPU_FIFO_CNTL_REG       0X73    //FIFO计数寄存器低八位 - FIFO count low byte
#define MPU_FIFO_RW_REG         0X74    //FIFO读写寄存器 - FIFO read/write
#define MPU_DEVICE_ID_REG       0X75    //器件ID寄存器 - Device ID register

//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位). - If AD0 pin is tied to GND, I2C addr = 0x68 (excluding LSB)
//如果接V3.3,则IIC地址为0X69(不包含最低位). - If tied to 3.3V, I2C addr = 0x69 (excluding LSB)
#define MPU_ADDR                0X68 << 1   //硬件IIC地址 - Hardware I2C address
#define MPU6050_ADDR_soft       0X68        //软件IIC地址 - Software I2C address

//因为模块AD0默认接GND,所以转为读写地址后,为0XD1和0XD0(如果接VCC,则为0XD3和0XD2)  
// - Since AD0 defaults to GND: R/W addresses are 0xD1/0xD0 (if tied to VCC → 0xD3/0xD2)
//#define MPU_READ                  0XD1
//#define MPU_WRITE                 0XD0

/**函数声明**/  // Helper Function declarations
uint8_t MPU6050_Init(void);
int8_t MPU6050_iic_write(unsigned char addr , unsigned char reg , uint16_t Size , uint8_t *tmp);
int8_t MPU6050_iic_read(unsigned char addr , unsigned char reg , uint16_t Size , uint8_t *data);
rt_err_t mpu_dmp_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date);
rt_err_t mpu_dmp_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf);
rt_err_t mpu6050_write_reg(rt_uint8_t reg, rt_uint8_t data);
rt_err_t mpu6050_read_reg(rt_uint8_t reg, rt_uint8_t *data);
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);

uint8_t MPU6050_acc_read(int32_t *x_data,int32_t *y_data,int32_t *z_data);
uint8_t MPU6050_gyro_read(int32_t *x_data,int32_t *y_data,int32_t *z_data);

#endif /* APPLICATIONS_MPU6050_H_ */
