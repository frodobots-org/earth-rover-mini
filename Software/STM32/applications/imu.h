/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-19     yuxing       the first version
 */

 #ifndef APPLICATIONS_IMU_H_
 #define APPLICATIONS_IMU_H_
 
 #include "main.h"
 #include "arm_math.h"
 
 // ==========================================================================
 // Event Flags (used for calibration control and status signaling)
 // ==========================================================================
 // Accelerometer + Gyroscope calibration events
 #define IMU_ACC_GYRO_EVENT_START    (1 << 0)  // Gyroscope calibration start event
 #define IMU_ACC_GYRO_EVENT_STOP     (1 << 1)  // Gyroscope calibration stop event
 #define IMU_ACC_GYRO_EVENT_DONE     (1 << 2)  // Gyroscope calibration finished event
 
 // Magnetometer calibration events
 #define IMU_MAG_EVENT_START         (1 << 3)  // Magnetometer calibration start event
 #define IMU_MAG_EVENT_STOP          (1 << 4)  // Magnetometer calibration stop event
 #define IMU_MAG_EVENT_DONE          (1 << 5)  // Magnetometer calibration finished event
 
 // LED feedback events during calibration
 #define IMU_CALIB_LED_START         (1 << 6)  // Magnetometer calibration LED on event
 #define IMU_CALIB_LED_DONE          (1 << 7)  // Magnetometer calibration LED off event
 
 // Motor control feedback events during calibration
 #define IMU_CALIB_MOTOR_CONTROL_START (1 << 8) // Motor control engaged for calibration
 #define IMU_CALIB_MOTOR_CONTROL_DONE  (1 << 9) // Motor control disengaged after calibration
 
 // ==========================================================================
 // RT-Thread synchronization primitives
 // ==========================================================================
 extern rt_event_t imu_event;       // IMU calibration event object
 extern rt_mutex_t imu_data_mutex;  // Mutex to protect shared IMU data access
 
 // ==========================================================================
 // Magnetometer type identifiers
 // ==========================================================================
 typedef enum
 {
     QMC5883L,  // QMC5883L model
     QMC5883P,  // QMC5883P model (variant of QMC5883L)
     DONE       // End marker / no more magnetometers
 } mag_type_t;
 
 // ==========================================================================
 // IMU Calibration State Machine
 // ==========================================================================
 typedef enum
 {
     CALIB_IDLE,          // Idle state, waiting for calibration command
     CALIB_START,         // Calibration started, memory/resources allocated
     CALIB_DATA_COLLECT,  // Collecting raw calibration data
     CALIB_HARD_START,    // Hard-iron calibration (removing constant magnetic offsets)
     CALIB_SOFT_START,    // Soft-iron calibration (correcting scaling and distortion)
     CALIB_DONE           // Calibration complete, return to idle
 } CalibrationState_t;
 
 // ==========================================================================
 // Sensor Data Structures
 // ==========================================================================
 
 // Gyroscope-based vibration/motion detection (angular rates in degrees/sec)
 typedef struct {
     double roll_gyro;    // Roll angular velocity
     double pitch_gyro;   // Pitch angular velocity
     double yaw_gyro;     // Yaw angular velocity
 } SensorData;
 
 // --------------------------------------------------------------------------
 // Raw IMU Data Types
 // --------------------------------------------------------------------------
 
 // Accelerometer raw data
 typedef struct {
     int32_t acc_x;
     int32_t acc_y;
     int32_t acc_z;
 } acc_data_t;
 
 // Accelerometer calibration offsets (bias)
 typedef struct {
     int32_t bias_x;
     int32_t bias_y;
     int32_t bias_z;
 } acc_calib_data_t;
 
 // Gyroscope raw data
 typedef struct {
     int32_t gyro_x;
     int32_t gyro_y;
     int32_t gyro_z;
 } gyro_data_t;
 
 // Gyroscope calibration offsets (bias)
 typedef struct {
     int32_t bias_x;
     int32_t bias_y;
     int32_t bias_z;
 } gyro_calib_data_t;
 
 // Magnetometer raw data
 typedef struct {
     int32_t mag_x;
     int32_t mag_y;
     int32_t mag_z;
 } mag_data_t;
 
 // Magnetometer calibration data (hard-iron offsets + soft-iron gains)
 typedef struct {
     int32_t offset_x;   // Hard-iron offset correction (X-axis)
     int32_t offset_y;   // Hard-iron offset correction (Y-axis)
     int32_t offset_z;   // Hard-iron offset correction (Z-axis)
     int32_t gain_x;     // Soft-iron gain correction (X-axis)
     int32_t gain_y;     // Soft-iron gain correction (Y-axis)
     int32_t gain_z;     // Soft-iron gain correction (Z-axis)
 } mag_calib_data_t;
 
 // --------------------------------------------------------------------------
 // Unified IMU Data Structure
 // --------------------------------------------------------------------------
 typedef struct {
     acc_data_t        acc_data;         // Raw accelerometer readings
     gyro_data_t       gyro_data;        // Raw gyroscope readings
     mag_data_t        mag_data;         // Raw magnetometer readings
 
     acc_calib_data_t  acc_calib_data;   // Accelerometer calibration data
     gyro_calib_data_t gyro_calib_data;  // Gyroscope calibration data
     mag_calib_data_t  mag_calib_data;   // Magnetometer calibration data
 
     int16_t           heading;          // Computed heading (yaw/compass direction)
 } thread_imu_data_t;
 
 // Shared IMU data instance (protected by imu_data_mutex)
 extern thread_imu_data_t thread_imu_data;
 
 #endif /* APPLICATIONS_IMU_H_ */
 