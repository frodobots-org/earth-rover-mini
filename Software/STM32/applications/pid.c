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

/**串口调试用**/
int16_t PID_uart_P = 6000;//1500
int16_t PID_uart_I = 20;//20
int16_t PID_uart_D = 0;

int16_t PID_dir_P = 35;
int16_t PID_dir_I = 38;
int16_t PID_dir_D = 0;

int16_t PID_target = 1330 * 0.3 * 1;//限制最大rpm(pid目标值)
int16_t PID_target_sign = 0;
/**串口调试用**/

/**速度环PID参数初始化结构体**/
pid_struct input_pid_fr;
pid_struct input_pid_br;
pid_struct input_pid_fl;
pid_struct input_pid_bl;

/**位置环PID参数初始化结构体**/
pid_struct dir_pid_control;

dir_pid pid_heading_control;

/**速度环最大速度限制**/
int16_t MIX_TARGET =991; //最大限速4km/h
/**
 * @brief  PID速度环参数初始化
 *   @note   无
 * @retval 无
 */
void PID_param_init(pid_struct *input_pid)
{
    /* 初始化参数 */
//    LOG_I("PID_init begin \n");
    input_pid->error_k0 = 0;
    input_pid->error_k1 = 0;
    input_pid->integral_error = 0;
    input_pid->limit_max = PERIOD;
    input_pid->limit_min = -PERIOD;
    input_pid->pid_result = 0;
    input_pid->kd = 0;
    input_pid->ki = 20;
    input_pid->kp = 6000;
//    LOG_I("PID_init end \n");

}

/**
 * @brief  PID位置环参数初始化(FIXME)
 *   @note   无
 * @retval 无
 */
void PID_param_dir_init(pid_struct *input_pid)
{
    /* 初始化参数 */
//    LOG_I("PID_init begin \n");
    input_pid->error_k0 = 0;
    input_pid->error_k1 = 0;
    input_pid->integral_error = 0;
    input_pid->limit_max = MIX_TARGET * 0.20;//速差最大调整幅度
    input_pid->limit_min = -MIX_TARGET * 0.20;
    input_pid->pid_result = 0;
    input_pid->kd = 0;
    input_pid->ki = 38;
    input_pid->kp = 35;
//    LOG_I("PID_init end \n");

}

/************************************************************
 *@func:获取增量式PID输出的结果(速度闭环)
 *@param:输入当前值
 *@param:输入目标值
 *@param:输入使用的PID参数
 *@return:计算结果
 *@noteֵֵ
 ************************************************************/
double get_Inc_pid_result(double input_curvalue, double input_target, pid_struct *input_pid ,double sum_time,double base_time)
{
    if((sum_time / base_time) <= 0.3)
    {
        input_pid->kd = 15;
        input_pid->ki = 25;
        input_pid->kp = 2000;
    }
    else {
        input_pid->kd = 0;
        input_pid->ki = 20;
        input_pid->kp = 6000;
    }
    /*-----------------------------PID计算-----------------------------*/
    input_pid->error_k0 = ( input_target - input_curvalue ); //Error[K]
//    input_pid->integral_error+=input_pid->error_k0;

    input_pid->pid_result += (input_pid->kp / 100) * (input_pid->error_k0 - input_pid->error_k1)  //比例计算
                                 + (input_pid->ki / 10) * input_pid->error_k0                //积分计算
                                 + (input_pid->kd / 10) * (input_pid->error_k0 - 2*input_pid->error_k1 + input_pid->error_k2); //微分计算
    /*---------------------------PID输出限幅---------------------------*/
    if (input_pid->pid_result >= input_pid->limit_max)
        input_pid->pid_result = input_pid->limit_max; //PID输出正向最大值限幅
    else if (input_pid->pid_result <= -input_pid->limit_max)
        input_pid->pid_result = -input_pid->limit_max; //PID输出负向最大值限幅
    /*-----------------------------------------------------------------*/
    input_pid->error_k2 = input_pid->error_k1; //Error[K-2]
    input_pid->error_k1 = input_pid->error_k0; //Error[K-1]

    return input_pid->pid_result;

}
/************************************************************
 *@func:获取增量式PID输出的结果(航向角闭环)
 *@param:输入当前值
 *@param:输入目标值
 *@param:输入使用的PID参数
 *@return:该函数旨在根据航向角偏差计算出左右轮速差值
 *@noteֵֵ
 ************************************************************/
double get_heading_pid_result(double input_curvalue, double input_target, pid_struct *input_pid, OscillationLevel shock_flag)
{
    //判断震荡等级改变PID调整幅值
    switch(shock_flag)
    {
    case NO_OSCILLATION:
    {
        input_pid->limit_max = MIX_TARGET * 0.20 * 1;
        input_pid->limit_min = -MIX_TARGET * 0.20 * -1;
    }
        break;
    case LIGHT_OSCILLATION:
    {
        input_pid->limit_max = MIX_TARGET * 0.20 * 1;
        input_pid->limit_min = -MIX_TARGET * 0.20 * -1;
    }
        break;
    case MODERATE_OSCILLATION:
    {
        input_pid->limit_max = MIX_TARGET * 0.20 * 1;
        input_pid->limit_min = -MIX_TARGET * 0.20 * -1;
    }
        break;
    case HEAVY_OSCILLATION:
    {
        input_pid->limit_max = MIX_TARGET * 0.20 * 1;//0.5
        input_pid->limit_min = -MIX_TARGET * 0.20 * -1;//-0.5
    }
        break;
    case EXTREME_OSCILLATION:
    {
        input_pid->limit_max = MIX_TARGET * 0.20 * 0;
        input_pid->limit_min = -MIX_TARGET * 0.20 * 0;
    }
        break;
    }

    /*-----------------------------PID计算-----------------------------*/
    input_pid->error_k0 = ( input_target - input_curvalue ); //Error[K]

    /**调整角度差值到正常范围**/
    if (input_pid->error_k0 > 180)
        input_pid->error_k0 -= 360;
    else if (input_pid->error_k0 < -180)
        input_pid->error_k0 += 360;

    input_pid->pid_result += (input_pid->kp ) * (input_pid->error_k0 - input_pid->error_k1)  //比例计算
                                 + (input_pid->ki / 1000) * input_pid->error_k0                //积分计算
                                 + (input_pid->kd / 1000) * (input_pid->error_k0 - 2*input_pid->error_k1 + input_pid->error_k2); //微分计算
    /*---------------------------PID输出限幅---------------------------*/
    if (input_pid->pid_result >= input_pid->limit_max)
        input_pid->pid_result = input_pid->limit_max; //PID输出正向最大值限幅
    else if (input_pid->pid_result <= -input_pid->limit_max)
        input_pid->pid_result = -input_pid->limit_max; //PID输出负向最大值限幅
    /*-----------------------------------------------------------------*/
    input_pid->error_k2 = input_pid->error_k1; //Error[K-2]
    input_pid->error_k1 = input_pid->error_k0; //Error[K-1]

    return input_pid->pid_result;

}


float linear_acceleration(float start_velocity, float target_velocity, float duration, float time) {
    if (time > duration) return target_velocity;
    return start_velocity + (target_velocity - start_velocity) * (time / duration);
}
/**指数平滑过渡**/
float exponential_smoothing(float current_velocity, float target_velocity, float alpha) {
    return (1 - alpha) * current_velocity + alpha * target_velocity;
}

/**贝塞尔曲线平滑**/
double s_curve_velocity(double v_current, double v_target, double t, double T) {
    // 归一化时间
    double x = t / T;

    // 确保 x 在 [0, 1] 之间
    if (x < 0) x = 0;
    if (x > 1) x = 1;

    // 计算S型速度
    return v_current + (v_target - v_current) * (3 * x * x - 2 * x * x * x);
}
