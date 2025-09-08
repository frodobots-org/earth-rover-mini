/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-25     Administrator       the first version
 */

 #include "pid.h"
 #include <ulog.h>
 
 /** ----------------- Debug / UART adjustable PID parameters ----------------- **/
 int16_t PID_uart_P = 6000; // Proportional gain for UART tuning
 int16_t PID_uart_I = 20;   // Integral gain for UART tuning
 int16_t PID_uart_D = 0;    // Derivative gain for UART tuning
 
 int16_t PID_dir_P = 35;    // Direction loop P
 int16_t PID_dir_I = 38;    // Direction loop I
 int16_t PID_dir_D = 0;     // Direction loop D
 
 int16_t PID_target = 1330 * 0.3 * 1; // Max PID target RPM
 int16_t PID_target_sign = 0;
 
 /** ----------------- PID structures ----------------- **/
 pid_struct input_pid_fr;  // Front-right motor speed PID
 pid_struct input_pid_br;  // Back-right motor speed PID
 pid_struct input_pid_fl;  // Front-left motor speed PID
 pid_struct input_pid_bl;  // Back-left motor speed PID
 
 pid_struct dir_pid_control; // Position/heading PID control
 
 dir_pid pid_heading_control; // Heading control struct
 
 /** Maximum speed limit for speed PID loops **/
 int16_t MIX_TARGET = 991; // roughly 4 km/h
 
 /************************************************************
  * @brief Initialize a speed-loop PID struct with default gains
  * @param input_pid: pointer to PID structure to initialize
  * @note Sets PID to default values for incremental PID calculation
  ************************************************************/
 void PID_param_init(pid_struct *input_pid)
 {
     input_pid->error_k0 = 0;
     input_pid->error_k1 = 0;
     input_pid->error_k2 = 0;
     input_pid->integral_error = 0;
     input_pid->limit_max = PERIOD;
     input_pid->limit_min = -PERIOD;
     input_pid->pid_result = 0;
     input_pid->kd = 0;
     input_pid->ki = 20;
     input_pid->kp = 6000;
 }
 
 /************************************************************
  * @brief Initialize a position/heading-loop PID struct
  * @param input_pid: pointer to PID structure to initialize
  * @note Sets gains for wheel speed differential / heading control
  ************************************************************/
 void PID_param_dir_init(pid_struct *input_pid)
 {
     input_pid->error_k0 = 0;
     input_pid->error_k1 = 0;
     input_pid->error_k2 = 0;
     input_pid->integral_error = 0;
     input_pid->limit_max = MIX_TARGET * 0.20; // Max wheel speed adjustment
     input_pid->limit_min = -MIX_TARGET * 0.20;
     input_pid->pid_result = 0;
     input_pid->kd = 0;
     input_pid->ki = 38;
     input_pid->kp = 35;
 }
 
 /************************************************************
  * @brief Incremental PID computation for speed-loop
  * @param input_curvalue: current speed/RPM
  * @param input_target: target speed/RPM
  * @param input_pid: PID struct
  * @param sum_time: accumulated control time
  * @param base_time: base sample period
  * @return computed incremental PID output
  ************************************************************/
 double get_Inc_pid_result(double input_curvalue, double input_target, pid_struct *input_pid, double sum_time, double base_time)
 {
     // Adjust PID gains for initial period for smoother startup
     if ((sum_time / base_time) <= 0.3)
     {
         input_pid->kd = 15;
         input_pid->ki = 25;
         input_pid->kp = 2000;
     }
     else
     {
         input_pid->kd = 0;
         input_pid->ki = 20;
         input_pid->kp = 6000;
     }
 
     // Compute incremental PID
     input_pid->error_k0 = (input_target - input_curvalue); // Current error
 
     input_pid->pid_result += (input_pid->kp / 100) * (input_pid->error_k0 - input_pid->error_k1)  // Proportional
                           + (input_pid->ki / 10) * input_pid->error_k0                             // Integral
                           + (input_pid->kd / 10) * (input_pid->error_k0 - 2*input_pid->error_k1 + input_pid->error_k2); // Derivative
 
     // Limit PID output
     if (input_pid->pid_result >= input_pid->limit_max)
         input_pid->pid_result = input_pid->limit_max;
     else if (input_pid->pid_result <= -input_pid->limit_max)
         input_pid->pid_result = -input_pid->limit_max;
 
     // Shift error history
     input_pid->error_k2 = input_pid->error_k1;
     input_pid->error_k1 = input_pid->error_k0;
 
     return input_pid->pid_result;
 }
 
 /************************************************************
  * @brief Incremental PID computation for heading control (wheel differential)
  * @param input_curvalue: current heading angle
  * @param input_target: target heading angle
  * @param input_pid: PID struct
  * @param shock_flag: oscillation level
  * @return PID output for wheel differential
  ************************************************************/
 double get_heading_pid_result(double input_curvalue, double input_target, pid_struct *input_pid, OscillationLevel shock_flag)
 {
     // Adjust PID limits based on oscillation severity
     switch (shock_flag)
     {
         case NO_OSCILLATION:
         case LIGHT_OSCILLATION:
         case MODERATE_OSCILLATION:
         case HEAVY_OSCILLATION:
             input_pid->limit_max = MIX_TARGET * 0.20;
             input_pid->limit_min = -MIX_TARGET * 0.20;
             break;
         case EXTREME_OSCILLATION:
             input_pid->limit_max = 0;
             input_pid->limit_min = 0;
             break;
     }
 
     // Compute PID error
     input_pid->error_k0 = input_target - input_curvalue;
 
     // Wrap heading error into [-180, 180]
     if (input_pid->error_k0 > 180)
         input_pid->error_k0 -= 360;
     else if (input_pid->error_k0 < -180)
         input_pid->error_k0 += 360;
 
     // Incremental PID computation
     input_pid->pid_result += (input_pid->kp) * (input_pid->error_k0 - input_pid->error_k1)
                            + (input_pid->ki / 1000) * input_pid->error_k0
                            + (input_pid->kd / 1000) * (input_pid->error_k0 - 2*input_pid->error_k1 + input_pid->error_k2);
 
     // Limit output
     if (input_pid->pid_result >= input_pid->limit_max)
         input_pid->pid_result = input_pid->limit_max;
     else if (input_pid->pid_result <= -input_pid->limit_max)
         input_pid->pid_result = -input_pid->limit_max;
 
     // Shift error history
     input_pid->error_k2 = input_pid->error_k1;
     input_pid->error_k1 = input_pid->error_k0;
 
     return input_pid->pid_result;
 }
 
 /************************************************************
  * @brief Compute linear acceleration from start to target velocity
  * @param start_velocity: initial velocity
  * @param target_velocity: desired velocity
  * @param duration: total acceleration duration
  * @param time: elapsed time
  * @return linearly interpolated velocity
  ************************************************************/
 float linear_acceleration(float start_velocity, float target_velocity, float duration, float time)
 {
     if (time > duration) return target_velocity;
     return start_velocity + (target_velocity - start_velocity) * (time / duration);
 }
 
 /************************************************************
  * @brief Exponential smoothing for velocity control
  * @param current_velocity: current velocity
  * @param target_velocity: desired velocity
  * @param alpha: smoothing factor (0..1)
  * @return smoothed velocity
  ************************************************************/
 float exponential_smoothing(float current_velocity, float target_velocity, float alpha)
 {
     return (1 - alpha) * current_velocity + alpha * target_velocity;
 }
 
 /************************************************************
  * @brief S-curve (Bezier) velocity smoothing
  * @param v_current: current velocity
  * @param v_target: target velocity
  * @param t: elapsed time
  * @param T: total duration
  * @return smoothed velocity along S-curve
  ************************************************************/
 double s_curve_velocity(double v_current, double v_target, double t, double T)
 {
     double x = t / T;   // normalize time
     if (x < 0) x = 0;
     if (x > 1) x = 1;
 
     // cubic Bezier curve smoothing: 3x^2 - 2x^3
     return v_current + (v_target - v_current) * (3 * x * x - 2 * x * x * x);
 }
 