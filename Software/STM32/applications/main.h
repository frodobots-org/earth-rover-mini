/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-14     luozs       the first version
 */
#ifndef APPLICATIONS_MAIN_H_
#define APPLICATIONS_MAIN_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <drv_common.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "state.h"
#include "./WS2812/ws2812b.h"
#include "./WS2812/ws2812binterface.h"
#include "ucp.h"

#define RPM_THRESHOLD_MAX   200//合理的RPM最大值 = Maximum reasonable RPM
#define RPM_THRESHOLD_MIN   -200//合理的RPM最小值 = Minimum reasonable RPM
#define KEY_LONG_PRESS_TIME 5000//开关机时间，单位ms = Power on/off press time, in ms
#define SYS_START_UP_TIME 1500//开机后开始识别按键的时间，单位ms = Time after startup before recognizing key presses, in ms
#define SECOND 1000
#define PER 10//定时周期100ms = Timer period 100 ms
#define RPP 11//每圈的脉冲数 = Pulses per revolution
#define REDUCTION_RATIO 90//减速比 = Gear reduction ratio
#define LOCKED_ROTOR_TIME 8000/PER	//单位ms = Locked rotor time, in ms
#define OVER_LOADER_TIME 16000/PER	//单位ms = Overload time, in ms
#define NOR_SPEED_OVERLOAD_RPM 50	//正常速度RPM = Normal speed RPM
#define FAST_SPEED_OVERLOAD_RPM 100	//最快速度RPM = Fastest speed RPM

//变加减速S型曲线参数 = Variable acceleration/deceleration S-curve parameters
#define S_START 0
#define S_END 50000
#define S_STEP 100

#define FILTER_SIZE 50
#define V_FILTER_SIZE 20 // 滑动滤波器的窗口大小 = Moving average filter window size
#define C_FILTER_SIZE 10 // 滑动滤波器的窗口大小 = Moving average filter window size
#define P_FILTER_SIZE 10 // 滑动滤波器的窗口大小 = Moving average filter window size
#define T_FILTER_SIZE 50 // 滑动滤波器的窗口大小(温度) = Moving average filter window size (temperature)
#define FULLPWR  12.00   //  满电电压，放大100倍 = Full battery voltage, scaled by 100
#define CUTPWR  800    //  截止电压，放大100倍 = Cutoff voltage, scaled by 100
#define LOWPWR  9.60    //  最低电压，放大100倍 = Minimum voltage, scaled by 100
#define PERIOD 50000//周期，单位ns，频率20KHz。很关键的参数，调试好后尽量不动50000 = Period, in ns, frequency 20 kHz. Very critical parameter — once tuned, try not to change 50000

typedef struct robot_state_t
{
			int16_t rpm [ 4 ];
			uint16_t battery;   //建议有无效值
			float voltage;  //电压V，放大100倍
			float current;  //电流A，放大100倍
			float power;    //功率W
			uint32_t over_loader_num;  //过载次数
			uint32_t locked_rotor_num;  //堵转次数
			uint32_t rs485_crcerr_num;  //RS485丢包次数
			uint8_t lamp;
			uint8_t tof;
			int16_t speed;
			int16_t steer;
			uint8_t key;
			uint8_t pwr;    //外设电路供电开关
			uint8_t break_mode;  //刹车模式
			uint8_t break_status;
			system_state_t status;  //MCU系统状态 0：充电中  1：充电完毕  2：系统初始状态 3：系统运行状态 4：系统关机状态
			ucp_state_e net_led_status; //头部联网状态示灯

} robot_state_t;
typedef struct
{
      float buffer [ FILTER_SIZE ];  // 存储样本的缓冲区
      rt_int32_t index;                  // 当前样本在缓冲区中的索引
      rt_int32_t count;                  // 缓冲区中当前存储的样本数量
      float sum;                // 缓冲区中样本的总和，用于快速计算平均值
} SlidingFilter;
//宏定义系统RGB状态灯的引脚
#define LED_SYS_R_PIN GET_PIN(C, 4)
#define LED_SYS_G_PIN GET_PIN(A, 7)
#define LED_SYS_B_PIN GET_PIN(C, 5)

//电机PID驱动(打开宏PID_OFF关闭PID)
//#define PID_OFF
//定义4路电机引脚
#define PWM_DEV_NAME1 "pwm1"
#define PWM_DEV_NAME9 "pwm9"
#define PWM_DEV_NAME10 "pwm10"
#define PWM_DEV_NAME11 "pwm11"
#define MOTOR_LF_IN 4	//左后电机的PWM引脚位TIM1的CH4
#define MOTOR_LB_IN 2	//左后电机的PWM引脚位TIM1的CH2
#define MOTOR_RF_IN 1	//左后电机的PWM引脚位TIM9的CH1
#define MOTOR_RB_IN 1	//左后电机的PWM引脚位TIM11的CH1

#define MOTOR_L_IN 4    //左电机的PWM引脚位TIM1的CH4
#define MOTOR_R_IN 2    //右电机的PWM引脚位TIM1的CH2

#define MOTOR_LF_BREAK 3//左前刹车TIM1_CH3
#define MOTOR_LB_BREAK 1//左前刹车TIM1_CH1
#define MOTOR_RF_BREAK 2//左前刹车TIM9_CH2
#define MOTOR_RB_BREAK 1//左前刹车TIM10_CH1

#define MOTOR_DIR_LF GET_PIN(A, 0)	//左前D
#define MOTOR_DIR_LB GET_PIN(D, 12)	//左后C
#define MOTOR_DIR_RF GET_PIN(B, 4)	//右前B
#define MOTOR_DIR_RB GET_PIN(A, 15)	//右后A

#define MOTOR_DIR_L GET_PIN(A, 0)  //左轮方向引脚
#define MOTOR_DIR_R GET_PIN(D, 12) //右轮方向引脚

#define MOTOR_PWM_R GET_PIN(E, 11)
#define MOTOR_PWM_L GET_PIN(E, 14)

//#define MOTOR_BREAK_LF GET_PIN(E, 13)
//#define MOTOR_BREAK_LB GET_PIN(E, 9)
//#define MOTOR_BREAK_RF GET_PIN(E, 6)
//#define MOTOR_BREAK_RB GET_PIN(B, 8)

#define MOTOR_FG_LF GET_PIN(A, 1)
#define MOTOR_FG_LB GET_PIN(D, 13)
#define MOTOR_FG_RF GET_PIN(B, 5)
#define MOTOR_FG_RB GET_PIN(B, 3)

#define MOTOR_VIN_ENABLE_PIN GET_PIN(E, 4) //FIXME

//宏定义4路车灯的引脚
#define LED_CAR1 GET_PIN(A, 8)
#define LED_CAR2 GET_PIN(B, 12)
#define LED_CAR3 GET_PIN(A, 6)
#define LED_CAR4 GET_PIN(E, 3)

#define PWR_ON       GET_PIN(C, 8)
#define PWR_CLK       GET_PIN(D, 7)
#define PWR_OUT_ON       GET_PIN(C, 0)
#define PWR_DEC      GET_PIN(C, 9)
#define CHARGE_DET  GET_PIN(C, 7)
#define CHG_INT GET_PIN(E, 1)
//#define BEEP      GET_PIN(A, 5)

#define LED_SYS_R      GET_PIN(C, 4)
#define LED_SYS_G      GET_PIN(A, 7)
#define LED_SYS_B      GET_PIN(C, 5)

#define LED_PWR_R      GET_PIN(C, 3)
#define LED_PWR_Y      GET_PIN(C, 2)
#define LED_PWR_G      GET_PIN(C, 1)

//宏定义RS485收发控制引脚
#define RS485_TR_EN_PIN      GET_PIN(D, 7)

//方向系数
int8_t coeff_fl;
int8_t coeff_fr;
int8_t coeff_bl;
int8_t coeff_br;

typedef struct robot_path_t{
    int32_t path_fl;
    int32_t path_fr;
    int32_t path_bl;
    int32_t path_br;
    int32_t path_differ_f;
    int32_t path_differ_b;
}robot_path;
robot_path path;

typedef struct speed_pid_t{
    volatile rt_int32_t pid_rpm_br;
    volatile rt_int32_t pid_rpm_fr;
    volatile rt_int32_t pid_rpm_bl;
    volatile rt_int32_t pid_rpm_fl;

    volatile rt_int32_t pid_rpm_br_smoothing;
    volatile rt_int32_t pid_rpm_fr_smoothing;
    volatile rt_int32_t pid_rpm_bl_smoothing;
    volatile rt_int32_t pid_rpm_fl_smoothing;

}speed_pid;
extern speed_pid speed_pid_s;

typedef struct pid_motor_control_t{
    volatile int16_t PID_target_br;
    volatile int16_t PID_target_fr;
    volatile int16_t PID_target_bl;
    volatile int16_t PID_target_fl;

    volatile int16_t PID_last_target_br;
    volatile int16_t PID_last_target_fr;
    volatile int16_t PID_last_target_bl;
    volatile int16_t PID_last_target_fl;

    volatile int16_t PID_target_br_smooth;
    volatile int16_t PID_target_fr_smooth;
    volatile int16_t PID_target_bl_smooth;
    volatile int16_t PID_target_fl_smooth;

    volatile int16_t PID_target_sign_br;
    volatile int16_t PID_target_sign_fr;
    volatile int16_t PID_target_sign_bl;
    volatile int16_t PID_target_sign_fl;

    volatile int8_t PID_sign_br;
    volatile int8_t PID_sign_fr;
    volatile int8_t PID_sign_bl;
    volatile int8_t PID_sign_fl;

    volatile int8_t PID_last_sign_br;
    volatile int8_t PID_last_sign_fr;
    volatile int8_t PID_last_sign_bl;
    volatile int8_t PID_last_sign_fl;

    volatile int8_t PID_time_sign_br;
    volatile int8_t PID_time_sign_fr;
    volatile int8_t PID_time_sign_bl;
    volatile int8_t PID_time_sign_fl;


    volatile float PID_control_yaw;

}pid_motor_control;
extern pid_motor_control pid_motor;

rt_int32_t pid_fr,pid_br,pid_fl,pid_bl;

typedef enum {
    NO_OSCILLATION = 0,
    LIGHT_OSCILLATION,
    MODERATE_OSCILLATION,
    HEAVY_OSCILLATION,
    EXTREME_OSCILLATION
} OscillationLevel;

typedef struct IMU_data_t{
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t pre_acc_x;
    int32_t pre_acc_y;
    int32_t pre_acc_z;

    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t pre_gyro_x;
    int32_t pre_gyro_y;
    int32_t pre_gyro_z;

    int32_t Mag_data[3];
    int32_t pre_Mag_data[3];

    int32_t acc_bias[3];
    int32_t gyro_bias[3];
    int32_t mag_bias[3];

    float imu_data[3];
    int16_t heading;
    float yaw_filter;
    uint8_t IMU_calib_mode;
    uint8_t MAG_calib_mode;
    uint8_t MAG_calib_flag;
    OscillationLevel shock_flag;
}IMU_data;
extern IMU_data IMU_updata;

enum color
{
	  BLACK, RED, YELLOW, GREEN, BLUE, CYAN, PURPLE, WHITE
};


typedef struct ctl_cmd {
    uint16_t     header;
    uint8_t      id;
    uint16_t     len;
    uint8_t      index;
    int16_t      speed;
    int16_t      angular;
    int16_t      front_led;
    int16_t      back_led;
    uint16_t     version;
    uint16_t     reserve1;
    uint32_t     reserve2;
    uint16_t     checksum;
}__attribute__((packed)) ctl_cmd_t;

typedef struct ctl {
    uint16_t     header;
    uint8_t      id;
    uint16_t     len;
    uint8_t      index;
    int16_t      speed;
    int16_t      angular;
    int16_t      front_led;
    int16_t      back_led;
    uint16_t     version;
    uint16_t     reserve1;
    uint32_t     reserve2;
    uint16_t     checksum;
}ctl_t;

/**
 * 全局变量声明
 */
extern volatile robot_state_t robot_state;
extern int16_t PID_target;
extern int16_t PID_target_sign;
extern RTC_HandleTypeDef hrtc;
#endif /* APPLICATIONS_MAIN_H_ */
