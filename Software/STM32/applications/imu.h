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

// 事件标志位（指令）
#define IMU_ACC_GYRO_EVENT_START    (1 << 0)  // 陀螺仪开始校准事件
#define IMU_ACC_GYRO_EVENT_STOP     (1 << 1)  // 陀螺仪停止校准事件
#define IMU_ACC_GYRO_EVENT_DONE     (1 << 2)  // 陀螺仪校准完成事件
#define IMU_MAG_EVENT_START    (1 << 3)  // 磁力计开始校准事件
#define IMU_MAG_EVENT_STOP     (1 << 4)  // 磁力计停止校准事件
#define IMU_MAG_EVENT_DONE     (1 << 5)  // 磁力计校准完成事件
#define IMU_CALIB_LED_START     (1 << 6)  // 磁力计校准LED事件
#define IMU_CALIB_LED_DONE     (1 << 7)  // 磁力计校准LED事件
#define IMU_CALIB_MOTOR_CONTROL_START (1 << 8)
#define IMU_CALIB_MOTOR_CONTROL_DONE  (1 << 9)

extern rt_event_t imu_event;//imu校准事件
extern rt_mutex_t imu_data_mutex;

typedef enum
{
    QMC5883L,
    QMC5883P,
    DONE
}mag_type_t;
//IMU校准状态机
typedef enum
{
    CALIB_IDLE,         // 空闲状态，等待校准命令
    CALIB_START,   // 校准开始、分配内存（初始化）
    CALIB_DATA_COLLECT, // 数据采集
    CALIB_HARD_START,    // 硬磁校准开始
    CALIB_SOFT_START,   // 软磁校准开始
    CALIB_DONE     // 校准完成，回到空闲状态
} CalibrationState_t;

//震动检测
typedef struct {
    double roll_gyro;   // 横滚加速度
    double pitch_gyro;  // 俯仰加速度
    double yaw_gyro;     // 航向角速度
} SensorData;


//imu_data_management
typedef struct{
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
}acc_data_t;
typedef struct{
    int32_t bias_x;
    int32_t bias_y;
    int32_t bias_z;
}acc_calib_data_t;
typedef struct{
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
}gyro_data_t;
typedef struct{
    int32_t bias_x;
    int32_t bias_y;
    int32_t bias_z;
}gyro_calib_data_t;
typedef struct{
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
}mag_data_t;
typedef struct{
    int32_t offset_x;
    int32_t offset_y;
    int32_t offset_z;
    int32_t gain_x;
    int32_t gain_y;
    int32_t gain_z;
}mag_calib_data_t;

typedef struct{
    acc_data_t acc_data;
    gyro_data_t gyro_data;
    mag_data_t mag_data;
    acc_calib_data_t acc_calib_data;
    gyro_calib_data_t gyro_calib_data;
    mag_calib_data_t mag_calib_data;
    int16_t heading;
}thread_imu_data_t;
extern thread_imu_data_t thread_imu_data;


#endif /* APPLICATIONS_IMU_H_ */
