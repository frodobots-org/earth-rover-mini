/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-25     Administrator       the first version
 */
#ifndef APPLICATIONS_PID_H_
#define APPLICATIONS_PID_H_
#include <main.h>
#include <rtdef.h>

/**贝塞尔曲线时间(ms)**/
#define ACC_TIME 750

typedef struct
{
        double kp;             //比例系数
        double ki;             //积分系数
        double kd;             //微分系数
        double limit_max;      //最大值限幅
        double limit_min;      //最小值限幅
        double integral_error; //积分误差
        double error_k0;       //当前误差
        double error_k1;       //上次误差
        double error_k2;       //上上次误差
        double PID_target;
        rt_int32_t pid_result; //pid输出结果

} pid_struct;

typedef struct dir_pid_t{
    double pid_target_delta_right;
    double pid_target_delta_left;
    u_int8_t dir_pid_switch;
    u_int8_t dir_pid_change_switch_fr;
    u_int8_t dir_pid_change_switch_fl;
    u_int8_t dir_time;
}dir_pid;
extern dir_pid pid_heading_control;

extern pid_struct input_pid_fr;
extern pid_struct input_pid_br;
extern pid_struct input_pid_fl;
extern pid_struct input_pid_bl;

extern pid_struct dir_pid_control;

/**串口调试用**/
extern int16_t PID_uart_P;
extern int16_t PID_uart_I;
extern int16_t PID_uart_D;

extern int16_t PID_dir_P;
extern int16_t PID_dir_I;
extern int16_t PID_dir_D;

extern int16_t PID_target;
extern int16_t PID_target_sign;
/**串口调试用**/
extern int16_t MIX_TARGET;

//函数声明
void PID_param_init();
void PID_param_dir_init(pid_struct *input_pid);
double get_Inc_pid_result(double input_curvalue, double input_target, pid_struct *input_pid ,double sum_time,double base_time);
double get_heading_pid_result(double input_curvalue, double input_target, pid_struct *input_pid, OscillationLevel shock_flag);
float linear_acceleration(float start_velocity, float target_velocity, float duration, float time);
float exponential_smoothing(float current_velocity, float target_velocity, float alpha);
double s_curve_velocity(double v_current, double v_target, double t, double T);

#endif /* APPLICATIONS_PID_H_ */
