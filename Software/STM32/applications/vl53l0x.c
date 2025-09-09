/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-17     Aaron        the first version
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
 
 /* I2C bus used for VL53L0X sensors */
 struct rt_i2c_bus_device *i2c_bus1 = RT_NULL;
 
 /* Default I2C address for VL53L0X before reassignment */
 #define VL530LX_DEFAULT_ADDR 0x29
 
 /* Define XSHUT (shutdown) control pins for each sensor 
  * Depending on platform (WHEELTEC or not), the pins are mapped differently.
  * Each pin allows powering on/off a specific VL53L0X sensor.
  */
 #ifdef WHEELTEC
 #define XSHUT1 GET_PIN(C, 2) // LEFT INNER sensor
 #define XSHUT2 GET_PIN(C, 2) // RIGHT INNER sensor
 #define XSHUT3 GET_PIN(C, 3) // LEFT OUTER sensor
 #define XSHUT4 GET_PIN(C, 2) // RIGHT OUTER sensor
 #else
 #define XSHUT1 GET_PIN(B, 1) // LEFT INNER sensor
 #define XSHUT2 GET_PIN(E, 0) // RIGHT INNER sensor
 #define XSHUT3 GET_PIN(E, 8) // LEFT OUTER sensor
 #define XSHUT4 GET_PIN(D, 0) // RIGHT OUTER sensor
 #endif
 
 #include "vl53l0x_platform.h"
 #include "vl53l0x_api.h"
 
 /* Robot TOF (Time-of-Flight) sensor structure
  * - enabled: flag to indicate if the sensor is initialized and active
  * - addr: assigned I2C address
  * - xshut: GPIO pin used to enable/disable sensor
  * - vl53l0x_dev: internal device struct required by ST’s VL53L0X API
  */
 typedef struct robot_tof_t {
     rt_int8_t enabled;
     rt_uint8_t addr;
     rt_int8_t xshut;
     VL53L0X_Dev_t vl53l0x_dev;
 } robot_tof_t;
 
 /* Array of 4 TOF sensors with predefined I2C addresses and XSHUT pins */
 static robot_tof_t robot_tofs[4] = {
     { .enabled = 0, .addr = 0x59, .xshut = XSHUT4 },
     { .enabled = 0, .addr = 0x49, .xshut = XSHUT3 },
     { .enabled = 0, .addr = 0x39, .xshut = XSHUT2 },
     { .enabled = 0, .addr = 0x29, .xshut = XSHUT1 },
 };
 
 /* Low-level register write function using RT-Thread I2C API */
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
         return RT_EOK; // Success
     }
     else
     {
         LOG_E("i2c bus write failed!\r\n");
         return -RT_ERROR; // Failure
     }
 }
 
 /* Low-level register read function using RT-Thread I2C API */
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
         return RT_EOK; // Success
     }
     else
     {
         LOG_E("i2c bus read failed!\r\n");
         return -RT_ERROR; // Failure
     }
 }
 
 /* External function declaration for single-ranging mode configuration */
 extern VL53L0X_Error vl53l0x_single_ranging_mode(VL53L0X_Dev_t *pdev);
 
 /* Perform a single measurement from the sensor and return distance in mm */
 int vl53l0x_get_value(VL53L0X_Dev_t *pdev)
 {
     static VL53L0X_RangingMeasurementData_t vl53l0x_data;
     VL53L0X_Error status = VL53L0X_ERROR_NONE;
 
     status = VL53L0X_PerformSingleRangingMeasurement(pdev, &vl53l0x_data);
     if (VL53L0X_ERROR_NONE != status)
     {
         return -1; // Measurement failed
     }
 
     return vl53l0x_data.RangeMilliMeter; // Return distance
 }
 
 /* Power off a VL53L0X sensor by pulling its XSHUT pin LOW */
 void vl53l0x_poweroff(robot_tof_t *robot_tof)
 {
     rt_pin_mode(robot_tof->xshut, PIN_MODE_OUTPUT);
     rt_pin_write(robot_tof->xshut, PIN_LOW);
 }
 
 /* Power on a VL53L0X sensor by pulling its XSHUT pin HIGH */
 void vl53l0x_poweron(robot_tof_t *robot_tof)
 {
     rt_pin_mode(robot_tof->xshut, PIN_MODE_OUTPUT);
     rt_pin_write(robot_tof->xshut, PIN_HIGH);
 }
 
 /* Modify the I2C address of a sensor (since all share the same default address).
  * This is required when multiple sensors are connected to the same I2C bus.
  */
 void vl53l0x_modify_addr(robot_tof_t *robot_tof, uint8_t addr)
 {
     rt_uint8_t buf[3];
     struct rt_i2c_msg msgs;
     buf[0] = 0x8A;                 // I2C address register
     buf[1] = addr & 0x7f;          // Mask to ensure 7-bit address
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
 
 /* Initialize all VL53L0X sensors on the robot
  * - Power cycle each sensor
  * - Assign unique I2C addresses
  * - Initialize device
  * - Configure to single-ranging mode
  */
 void vl53l0x_init_all()
 {
     VL53L0X_DeviceInfo_t vl53l0x_info;
     i2c_bus1 = (struct rt_i2c_bus_device *)rt_device_find("i2c1");
 
     // Step 1: power off all sensors
     for (int i = 0; i < 2; i++)
     {
         rt_pin_mode(robot_tofs[i].xshut, PIN_MODE_OUTPUT);
         vl53l0x_poweroff(&robot_tofs[i]);
     }
     rt_thread_mdelay(100);
 
     // Step 2: power on each sensor one at a time and assign new address
     for (int i = 0; i < 2; i++)
     {
         vl53l0x_poweron(&robot_tofs[i]);
         vl53l0x_modify_addr(&robot_tofs[i], robot_tofs[i].addr);
         rt_thread_mdelay(100);
     }
 
     // Step 3: initialize each sensor and configure measurement mode
     for (int i = 0; i < 2; i++)
     {
         robot_tofs[i].vl53l0x_dev.comms_type = 1;
         robot_tofs[i].vl53l0x_dev.comms_speed_khz = 400;
         robot_tofs[i].vl53l0x_dev.I2cDevAddr = (rt_uint32_t)(robot_tofs[i].addr) & 0xff;
         robot_tofs[i].vl53l0x_dev.RegRead = vl53l0x_reg_read;
         robot_tofs[i].vl53l0x_dev.RegWrite = vl53l0x_reg_write;
 
         /* Initialize sensor data */
         if (VL53L0X_ERROR_NONE != VL53L0X_DataInit(&robot_tofs[i].vl53l0x_dev))
         {
             LOG_E("vl53l0x data init failed\r\n");
             continue;
         }
 
         /* Optionally get device info (version, etc.) */
         if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&robot_tofs[i].vl53l0x_dev, &vl53l0x_info))
         {
             // Info can be logged if needed
         }
 
         /* Set single-ranging mode */
         if (VL53L0X_ERROR_NONE != vl53l0x_single_ranging_mode(&robot_tofs[i].vl53l0x_dev))
         {
             LOG_E("vl53l0x single ranging init failed\r\n");
             continue;
         }
 
         robot_tofs[i].enabled = 1;
         rt_thread_mdelay(100);
     }
 }
 
 /* Sensor thread that continuously polls TOF sensors and logs their readings */
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
         rt_thread_mdelay(1000); // Delay 1s between readings
     }
 }
 