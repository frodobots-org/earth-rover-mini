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
 
 /** ----------------- Bezier curve acceleration time (ms) ----------------- **/
 #define ACC_TIME 750
 
 /** ----------------- PID structure for speed or position loops ----------------- **/
 typedef struct
 {
     double kp;             // Proportional gain
     double ki;             // Integral gain
     double kd;             // Derivative gain
     double limit_max;      // Maximum output limit
     double limit_min;      // Minimum output limit
     double integral_error; // Cumulative integral error
     double error_k0;       // Current error
     double error_k1;       // Previous error
     double error_k2;       // Error two steps ago
     double PID_target;     // Target value for PID
     rt_int32_t pid_result; // Output of PID
 } pid_struct;
 
 /** ----------------- Structure for heading/direction PID control ----------------- **/
 typedef struct dir_pid_t {
     double pid_target_delta_right;  // Speed delta for right wheels
     double pid_target_delta_left;   // Speed delta for left wheels
     uint8_t dir_pid_switch;         // Enable/disable heading PID
     uint8_t dir_pid_change_switch_fr; // Front-right wheel switch for PID change
     uint8_t dir_pid_change_switch_fl; // Front-left wheel switch for PID change
     uint8_t dir_time;               // Time counter for direction control
 } dir_pid;
 
 /** ----------------- External PID instances ----------------- **/
 extern dir_pid pid_heading_control;
 
 extern pid_struct input_pid_fr; // Front-right speed PID
 extern pid_struct input_pid_br; // Back-right speed PID
 extern pid_struct input_pid_fl; // Front-left speed PID
 extern pid_struct input_pid_bl; // Back-left speed PID
 
 extern pid_struct dir_pid_control; // Heading/position PID
 
 /** ----------------- UART adjustable debug PID parameters ----------------- **/
 extern int16_t PID_uart_P;
 extern int16_t PID_uart_I;
 extern int16_t PID_uart_D;
 
 extern int16_t PID_dir_P;
 extern int16_t PID_dir_I;
 extern int16_t PID_dir_D;
 
 extern int16_t PID_target;      // PID target RPM
 extern int16_t PID_target_sign; // Direction sign for PID
 
 /** Maximum allowed speed for speed PID loops **/
 extern int16_t MIX_TARGET;
 
 /** ----------------- Function declarations ----------------- **/
 
 /**
  * @brief Initialize speed-loop PID parameters
  * @param input_pid: pointer to PID structure
  */
 void PID_param_init(pid_struct *input_pid);
 
 /**
  * @brief Initialize heading/position PID parameters
  * @param input_pid: pointer to PID structure
  */
 void PID_param_dir_init(pid_struct *input_pid);
 
 /**
  * @brief Compute incremental PID output for speed loop
  */
 double get_Inc_pid_result(double input_curvalue, double input_target, pid_struct *input_pid,
                           double sum_time, double base_time);
 
 /**
  * @brief Compute incremental PID output for heading control (wheel differential)
  */
 double get_heading_pid_result(double input_curvalue, double input_target, pid_struct *input_pid,
                               OscillationLevel shock_flag);
 
 /**
  * @brief Linear acceleration interpolation
  */
 float linear_acceleration(float start_velocity, float target_velocity, float duration, float time);
 
 /**
  * @brief Exponential smoothing for velocity transitions
  */
 float exponential_smoothing(float current_velocity, float target_velocity, float alpha);
 
 /**
  * @brief S-curve (Bezier) velocity smoothing
  */
 double s_curve_velocity(double v_current, double v_target, double t, double T);
 
 #endif /* APPLICATIONS_PID_H_ */
 