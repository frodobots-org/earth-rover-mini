/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-30     yuxing       the first version
 */
#ifndef APPLICATIONS_QMC5883L_H_
#define APPLICATIONS_QMC5883L_H_

#include "Hardware_i2c.h"
#include "MPU6050.h"

/***************************以下是QMC5883L地磁传感器**************************/
/***************************Below are QMC5883L geomagnetic sensor definitions**************************/

// Device I2C addresses (depending on mode)
#define QMC5883L_ADDRESS        0x1A //0x1A   // I2C write address
#define QMC5883L_ADDRESS_SOFT   0x0D          // Software I2C address

// Register map for data output
#define QMC5883L_DATA_READ_X_LSB    0x00   // X-axis low byte
#define QMC5883L_DATA_READ_X_MSB    0x01   // X-axis high byte
#define QMC5883L_DATA_READ_Y_LSB    0x02   // Y-axis low byte
#define QMC5883L_DATA_READ_Y_MSB    0x03   // Y-axis high byte
#define QMC5883L_DATA_READ_Z_LSB    0x04   // Z-axis low byte
#define QMC5883L_DATA_READ_Z_MSB    0x05   // Z-axis high byte

#define QMC5883L_STATUS             0x06   // Status register: DOR | OVL | DRDY
                                           // DOR: Data Overrun
                                           // OVL: Overflow
                                           // DRDY: Data Ready

// Temperature sensor registers
#define QMC5883L_TEMP_READ_LSB      0x07
#define QMC5883L_TEMP_READ_MSB      0x08

// Configuration registers
#define QMC5883L_CONFIG_1           0x09   // Control: OSR | RNG | ODR | MODE
#define QMC5883L_CONFIG_2           0x0A   // Control: SOFT_RST | ROL_PNT | INT_ENB
#define QMC5883L_CONFIG_3           0x0B   // SET/RESET period (Fine Bias Register)
#define QMC5883L_ID                 0x0D   // Chip ID register

// If math constant M_PI is not already defined, define it here
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288f
#endif

// Scale factors and conversion constants
#define QMC5883L_SCALE_FACTOR       0.732421875f     // Raw to scaled unit conversion
#define QMC5883L_CONVERT_GAUSS_2G   12000.0f         // Conversion factor for ±2 Gauss full scale
#define QMC5883L_CONVERT_GAUSS_8G   3000.0f          // Conversion factor for ±8 Gauss full scale
#define QMC5883L_CONVERT_MICROTESLA 100              // Conversion to microtesla
#define QMC5883L_DECLINATION_ANGLE  93.67/1000       // Magnetic declination (in radians) for Tekirdag, Turkey

// ==========================================================================
// ENUMERATIONS for QMC5883L configuration and status
// ==========================================================================

// Sensor status indicators
typedef enum STATUS_VARIABLES
{
    NORMAL,                   // Normal operation
    NO_NEW_DATA,              // No new data available
    NEW_DATA_IS_READY,        // New data ready
    DATA_OVERFLOW,            // Measurement overflow occurred
    DATA_SKIPPED_FOR_READING  // Data missed/skipped due to read delay
}_qmc5883l_status;

// Operating mode settings
typedef enum MODE_VARIABLES
{
    MODE_CONTROL_STANDBY=0x00,       // Standby mode (low power)
    MODE_CONTROL_CONTINUOUS=0x01     // Continuous measurement mode
}_qmc5883l_MODE;

// Output Data Rate settings
typedef enum ODR_VARIABLES
{
    OUTPUT_DATA_RATE_10HZ=0x00,      // 10 Hz
    OUTPUT_DATA_RATE_50HZ=0x04,      // 50 Hz
    OUTPUT_DATA_RATE_100HZ=0x08,     // 100 Hz
    OUTPUT_DATA_RATE_200HZ=0x0C      // 200 Hz
}_qmc5883l_ODR;

// Full-scale range settings
typedef enum RNG_VARIABLES
{
    FULL_SCALE_2G=0x00,              // ±2 Gauss
    FULL_SCALE_8G=0x10               // ±8 Gauss
}_qmc5883l_RNG;

// Oversampling ratio settings
typedef enum OSR_VARIABLES
{
    OVER_SAMPLE_RATIO_512=0x00,      // High oversampling, lower noise, slower response
    OVER_SAMPLE_RATIO_256=0x40,      // Medium-high oversampling
    OVER_SAMPLE_RATIO_128=0x80,      // Medium oversampling
    OVER_SAMPLE_RATIO_64=0xC0        // Low oversampling, faster response
}_qmc5883l_OSR;

// Interrupt enable/disable
typedef enum INTTERRUPT_VARIABLES
{
    INTERRUPT_DISABLE,               // Disable interrupt
    INTERRUPT_ENABLE                 // Enable interrupt
}_qmc5883l_INT;

/***************************以上是QMC5883L地磁传感器**************************/
/***************************End of QMC5883L geomagnetic sensor definitions**************************/

// ==========================================================================
// FUNCTION PROTOTYPES
// ==========================================================================

// Legacy initialization for HMC5883L (similar device, included for compatibility)
uint8_t HMC5883L_Init(void);

// Initialize the QMC5883L sensor
rt_err_t QMC5883L_Init(void);

// Get calculated magnetic heading angle from sensor
void QMC5883L_GetAngle(int32_t* data);

// Write multiple bytes to QMC5883L register
rt_err_t QMC5883L_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date);

// Read multiple bytes from QMC5883L register
rt_err_t QMC5883L_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf);

// Write single byte to QMC5883L register
rt_err_t QMC5883L_write_reg(rt_uint8_t reg, rt_uint8_t data);

// Read single byte from QMC5883L register
rt_err_t QMC5883L_read_reg(rt_uint8_t reg, rt_uint8_t *data);

// Calculate heading angle (in radians) using X and Y axis magnetic data
float calculate_heading(float mag_x, float mag_y);

#endif /* APPLICATIONS_QMC5883L_H_ */
