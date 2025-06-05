/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-14     luozs       the first version
 */

/*
 * 程序清单：这是一个 hwtimer 设备使用例程
 * 例程导出了 hwtimer_sample 命令到控制终端
 * 命令调用格式：hwtimer_sample
 * 程序功能：硬件定时器超时回调函数周期性的打印当前tick值，2次tick值之差换算为时间等同于定时时间值。
 */

#include "main.h"
#include "pid.h"
#include <rtdef.h>
#define DBG_TAG "HWTIMER"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern volatile uint32_t PWM_FallingCount_2;
extern volatile uint32_t PWM_FallingCount_3;
extern volatile uint32_t PWM_FallingCount_4;
extern volatile uint32_t PWM_FallingCount_5;
rt_int32_t pid_fr_output,pid_fl_output;

#define HWTIMER_DEV_NAME   "timer13"     /* 定时器名称 */

rt_int32_t rpm [ 4 ] = { 0 , 0 , 0 , 0 };
extern volatile robot_state_t robot_state;
extern void motor_encoder_read ( rt_int32_t *rpm );
extern void motor_encoder_reset ( void );

extern struct rt_device_pwm *pwm_dev1;
extern struct rt_device_pwm *pwm_dev9;
extern struct rt_device_pwm *pwm_dev10;
extern struct rt_device_pwm *pwm_dev11;

int16_t safe_convert_int32_to_int16 ( int32_t value )
{
	  if ( value > INT16_MAX )
	  {
			LOG_W( "INT16_MAX" );
			return INT16_MAX;  // 如果超出最大值，返回最大值
	  }
	  else if ( value < INT16_MIN )
	  {
			LOG_W( "INT16_MIN" );
			return INT16_MIN;  // 如果超出最小值，返回最小值
	  }
	  else
	  {

			return (int16_t) value;  // 在范围内，直接转换
	  }
}

rt_int16_t Weight1 = 60 * SECOND / PER , Weight2 = 4 * REDUCTION_RATIO * RPP;
double Weight3 = 0;

rt_int32_t rpm_fl = 0 , rpm_fr = 0 , rpm_bl = 0 , rpm_br = 0;

extern void motor_control ( rt_uint8_t mode );
extern void robot_power_off ( void );
extern void robot_power_on ( void );
extern void robot_power_out_on ( void );
//#define PERIOD 50000//周期，单位ns，频率20KHz。很关键的参数，调试好后尽量不动

rt_int32_t average[4][10] = {0};
rt_int32_t sum_1 = 0;sum_2 = 0;sum_3= 0;sum_4 = 0;
uint8_t average_count = 0;
double sum_t_fl = 0;
double sum_t_fr = 0;

SlidingFilter rpm_filter;
/* 定时器超时回调函数 */
extern rt_sem_t imu_sem;
uint8_t delay_num = 0;

static rt_err_t timeout_cb ( rt_device_t dev , rt_size_t size )
{
    /**释放信号量，通知IMU线程进行数据读取**/
    if (imu_sem != RT_NULL)//防止信号量未创建完毕提前进入定时器进行释放
    {
        rt_sem_release(imu_sem);
    }

    if(PWM_FallingCount_4 >= 33920)
        PWM_FallingCount_4 = 2000000;
    if(PWM_FallingCount_5 >= 33920)
            PWM_FallingCount_5 = 2000000;

    rpm_fr=6*(10000/((PWM_FallingCount_4*9*28)/1000));
    rpm_fl=6*(10000/((PWM_FallingCount_5*9*28)/1000));

    speed_pid_s.pid_rpm_fr = 1000000/PWM_FallingCount_4;
    speed_pid_s.pid_rpm_fl = 1000000/PWM_FallingCount_5;

//    if(speed_pid_s.pid_rpm_br <= 100 && speed_pid_s.pid_rpm_fr <= 100 && speed_pid_s.pid_rpm_bl <= 100 && speed_pid_s.pid_rpm_fl <= 100 )
//    {
//        switch_fliter = 0;
//        bzero(average,sizeof(average));
//    }

#ifndef PID_OFF
    /********************PID控制************************/

    /********数组平均**********/
    average[1][average_count%10] = speed_pid_s.pid_rpm_fr;
    average[3][average_count%10] = speed_pid_s.pid_rpm_fl;

    /********累加求和**********/
    for(int i=0;i<=9;i++)
    {
        sum_2 += average[1][i];
        sum_4 += average[3][i];
    }
    speed_pid_s.pid_rpm_fr = sum_2/10;
    speed_pid_s.pid_rpm_fl = sum_4/10;

    sum_2 = 0;
    sum_4 = 0;

    average_count++;
    if(average_count == 249)
        average_count = 0;
    /********数组平均**********/

    /********************PID参数调整测试*********************/
//      input_pid_fl.kp = PID_uart_P;
//      input_pid_fl.ki = PID_uart_I;
//      input_pid_fl.kd = PID_uart_D;
//
//      input_pid_fr.kp = PID_uart_P;
//      input_pid_fr.ki = PID_uart_I;
//      input_pid_fr.kd = PID_uart_D;
//
//      dir_pid_control.kp = PID_dir_P;
//      dir_pid_control.ki = PID_dir_I;
//      dir_pid_control.kd = PID_dir_D;

//      pid_motor.PID_target_fl = PID_target;
//      pid_motor.PID_target_fr = PID_target;

//      pid_motor.PID_target_sign_fl = PID_target_sign;
//      pid_motor.PID_target_sign_fr = 1;

      /********************PID参数调整测试*********************/
    /*************目标值正反转改变****************/
    // param1:PID_target_sign_br 获取头部正反转信号
    // param2:PID_sign_br 改变get_Inc_pid_result()函数中当前值与目标值的正负号

    if(pid_motor.PID_target_sign_fr == 0)
       {
           pid_motor.PID_sign_fr = 1;
       }
       else {
           pid_motor.PID_sign_fr = -1;
       }

    if(pid_motor.PID_target_sign_fl == 0)
       {
           pid_motor.PID_sign_fl = 1;
       }
       else {
           pid_motor.PID_sign_fl = -1;
       }
    /*************目标值正反转改变****************/
//    pid_motor.PID_target_fr = 1000;
//    pid_motor.PID_target_fl = 1000;
    /*************航向角目标值判断****************/
//#define HEADING_OFF
#ifndef HEADING_OFF
    if( ((pid_motor.PID_sign_fr * pid_motor.PID_target_fr) == 0 && (pid_motor.PID_sign_fl * pid_motor.PID_target_fl) == 0) || (IMU_updata.shock_flag ==EXTREME_OSCILLATION))//不管控则一直刷新PID航向角目标值
    {
        pid_heading_control.dir_pid_switch = 0;//闭环导航关闭
        pid_heading_control.dir_time = 0;
        pid_motor.PID_control_yaw = IMU_updata.imu_data[2];
        dir_pid_control.error_k0 = 0;
        dir_pid_control.error_k1 = 0;
        dir_pid_control.error_k2 = 0;
        dir_pid_control.pid_result = 0;

    }
    else if( (pid_motor.PID_sign_fr * pid_motor.PID_target_fr) != (pid_motor.PID_sign_fl * pid_motor.PID_target_fl)) //左右轮目标值不相等(转弯或原地转向)则刷新PID航向角目标值
    {
        pid_heading_control.dir_pid_switch = 0;//闭环导航关闭
        pid_heading_control.dir_time = 0;
        pid_motor.PID_control_yaw = IMU_updata.imu_data[2];
        dir_pid_control.error_k0 = 0;
        dir_pid_control.error_k1 = 0;
        dir_pid_control.error_k2 = 0;
        dir_pid_control.pid_result = 0;

    }
    else if( (pid_motor.PID_sign_fr * pid_motor.PID_target_fr) == (pid_motor.PID_sign_fl * pid_motor.PID_target_fl)) //左右轮目标值相等且不为0(走直线)则不刷新PID航向角目标值，启动航向闭环
    {
        if(pid_heading_control.dir_pid_switch == 0)
        {
            pid_motor.PID_control_yaw = IMU_updata.imu_data[2];//航向延时(1S)没到则一直刷新目标值
            pid_heading_control.dir_time ++;
        }

        if(pid_heading_control.dir_time >= 100)
        {
            pid_heading_control.dir_pid_switch = 1;//闭环导航开启
        }
    }

    //中途变速过程关闭航向闭环
    if((pid_motor.PID_last_target_fr != pid_motor.PID_target_fr) || (pid_motor.PID_last_target_fl != pid_motor.PID_target_fl))
    {
        pid_heading_control.dir_pid_switch = 0;//闭环导航关闭
        pid_heading_control.dir_time = 0;
        pid_motor.PID_control_yaw = IMU_updata.imu_data[2];
        dir_pid_control.error_k0 = 0;
        dir_pid_control.error_k1 = 0;
        dir_pid_control.error_k2 = 0;
        dir_pid_control.pid_result = 0;
    }
    /*************航向角目标值判断****************/
    /*************航向角差值解算(差值在速度闭环的目标值基础上进行偏移)****************/

    pid_heading_control.pid_target_delta_right = get_heading_pid_result(IMU_updata.imu_data[2],pid_motor.PID_control_yaw,&dir_pid_control,IMU_updata.shock_flag);
    pid_heading_control.pid_target_delta_left = -pid_heading_control.pid_target_delta_right;

    /*************航向角差值解算(差值在速度闭环的目标值基础上进行偏移)****************/
#else
    pid_heading_control.dir_pid_switch = 0; //关闭航向闭环
#endif

    /**正反转、原地旋转信号优化**/
    if((pid_motor.PID_last_sign_fr != pid_motor.PID_sign_fr) || (pid_motor.PID_last_sign_fl != pid_motor.PID_sign_fl))
    {
        pid_motor.PID_time_sign_fr = 1;
        pid_motor.PID_time_sign_fl = 1;
    }
    /**原地旋转信号优化**/

    //目标值未发生改变，不更新当前速度smoothing进行贝塞尔曲线平滑，否则更新贝塞尔初始速度以及目标值smooth
    if((pid_motor.PID_last_sign_fr == pid_motor.PID_sign_fr) && (pid_motor.PID_last_target_fr == pid_motor.PID_target_fr) && pid_motor.PID_time_sign_fr == 0)
    {
        if(sum_t_fr >= ACC_TIME)
        {
            sum_t_fr = ACC_TIME;
        }
        /**特殊判断是否是原地旋转操作，如果是则不采用贝塞尔曲线闭环**/
        if(pid_motor.PID_sign_fr != pid_motor.PID_sign_fl)
            pid_motor.PID_target_fr_smooth = pid_motor.PID_target_fr;
        else{
            pid_motor.PID_target_fr_smooth = s_curve_velocity(speed_pid_s.pid_rpm_fr_smoothing, pid_motor.PID_target_fr,  sum_t_fr,  ACC_TIME);
            sum_t_fr +=20;
        }
    }
    else {
        if(pid_motor.PID_last_sign_fr != pid_motor.PID_sign_fr)//如果出现瞬间反转信号，将初始目标值设为0(先自由减速至0)
            pid_motor.PID_target_fr_smooth = 0;

        speed_pid_s.pid_rpm_fr_smoothing = pid_motor.PID_target_fr_smooth;//更新当前速度为上一次目标值speed_pid_s.pid_rpm_fr

        sum_t_fr = ACC_TIME * 0; //清0计数值
    }

    if(pid_motor.PID_time_sign_fr == 0)
    {
        if(speed_pid_s.pid_rpm_fr >= 300 && pid_heading_control.dir_pid_switch == 1)//航向闭环控制开启
        {
            pid_motor.PID_target_fr_smooth += pid_motor.PID_sign_fr * pid_heading_control.pid_target_delta_right;
        }
        pid_fr = get_Inc_pid_result(pid_motor.PID_sign_fr * speed_pid_s.pid_rpm_fr,pid_motor.PID_sign_fr * pid_motor.PID_target_fr_smooth,&input_pid_fr,sum_t_fr,ACC_TIME);
    }

    //目标值未发生改变，不更新当前速度进行贝塞尔曲线平滑，否则更新当前速度以及目标值
    if((pid_motor.PID_last_sign_fl == pid_motor.PID_sign_fl) && (pid_motor.PID_last_target_fl == pid_motor.PID_target_fl) && pid_motor.PID_time_sign_fl == 0)
    {
        if(sum_t_fl >= ACC_TIME)
        {
            sum_t_fl = ACC_TIME;
        }
        /**特殊判断是否是原地旋转操作，如果是则不采用贝塞尔曲线闭环**/
        if(pid_motor.PID_sign_fr != pid_motor.PID_sign_fl)
            pid_motor.PID_target_fl_smooth = pid_motor.PID_target_fl;
        else {
            pid_motor.PID_target_fl_smooth = s_curve_velocity(speed_pid_s.pid_rpm_fl_smoothing, pid_motor.PID_target_fl, sum_t_fl,  ACC_TIME);
            sum_t_fl +=20;
        }
    }
    else {
        if(pid_motor.PID_last_sign_fl != pid_motor.PID_sign_fl)//如果出现瞬间反转信号，将初始目标值设为0(先自由减速至0)
            pid_motor.PID_target_fl_smooth = 0;

        speed_pid_s.pid_rpm_fl_smoothing = pid_motor.PID_target_fl_smooth;//更新当前速度为上一次目标值speed_pid_s.pid_rpm_fl

        sum_t_fl = ACC_TIME * 0; //清0计数值
    }

    if(pid_motor.PID_time_sign_fl == 0)
    {
        if(speed_pid_s.pid_rpm_fl >= 300 && pid_heading_control.dir_pid_switch == 1 )//航向闭环控制开启
        {
            pid_motor.PID_target_fl_smooth += pid_motor.PID_sign_fl * pid_heading_control.pid_target_delta_left;
        }
        pid_fl = get_Inc_pid_result(pid_motor.PID_sign_fl * speed_pid_s.pid_rpm_fl,pid_motor.PID_sign_fl * pid_motor.PID_target_fl_smooth,&input_pid_fl,sum_t_fl,ACC_TIME);
    }

    /**记录上一次的状态**/
    pid_motor.PID_last_sign_fr = pid_motor.PID_sign_fr;
    pid_motor.PID_last_target_fr = pid_motor.PID_target_fr;
    pid_motor.PID_last_sign_fl = pid_motor.PID_sign_fl;
    pid_motor.PID_last_target_fl = pid_motor.PID_target_fl;//记录上一次的目标值

    /**检测到正反转、原地旋转先让机器达到完全静止状态**/
    if(pid_motor.PID_time_sign_fl == 1 && pid_motor.PID_time_sign_fr == 1)
    {
        if(speed_pid_s.pid_rpm_fl == 0 && speed_pid_s.pid_rpm_fr == 0)//直线行走状态待两轮速度降低至0后，再由新目标值管控
        {
            //等待一段时间后执行
            delay_num ++;
            if(delay_num == 15)
            {
                speed_pid_s.pid_rpm_fl_smoothing = 0;
                pid_motor.PID_target_fl_smooth = 0;
                sum_t_fl = ACC_TIME * 0; //清0计数值
                pid_motor.PID_time_sign_fl = 0;//解锁新目标值管控
                input_pid_fl.pid_result = 0;
                input_pid_fl.error_k0 = 0;
                input_pid_fl.error_k1 = 0;
                bzero(average,sizeof(average));

                speed_pid_s.pid_rpm_fr_smoothing = 0;
                pid_motor.PID_target_fr_smooth = 0;
                sum_t_fr = ACC_TIME * 0; //清0计数值
                pid_motor.PID_time_sign_fr = 0;
                input_pid_fr.pid_result = 0;
                input_pid_fr.error_k0 = 0;
                input_pid_fr.error_k1 = 0;
                bzero(average,sizeof(average));
                delay_num = 0;
            }

            pid_fl = 0;
            pid_fr = 0;
        }
        else {
            pid_fl = 0;
            pid_fr = 0;
        }
    }

    //检测到横滚角和俯仰角角度>=18°，开环
    if(abs(IMU_updata.imu_data[0]) >= 18.0f || abs(IMU_updata.imu_data[1]) >= 18.0f)
    {
        pid_fl = ((pid_motor.PID_sign_fl * (pid_motor.PID_target_fl * 100) / MIX_TARGET) * PERIOD)/100;
        pid_fr = ((pid_motor.PID_sign_fr * (pid_motor.PID_target_fr * 100) / MIX_TARGET) * PERIOD)/100;

        speed_pid_s.pid_rpm_fl_smoothing = speed_pid_s.pid_rpm_fl;
        sum_t_fl = ACC_TIME * 0; //清0计数值
        input_pid_fl.pid_result = pid_fl;
        input_pid_fl.error_k0 = 0;
        input_pid_fl.error_k1 = 0;

        speed_pid_s.pid_rpm_fr_smoothing = speed_pid_s.pid_rpm_fr;
        sum_t_fr = ACC_TIME * 0; //清0计数值
        input_pid_fr.pid_result = pid_fr;
        input_pid_fr.error_k0 = 0;
        input_pid_fr.error_k1 = 0;

        pid_heading_control.dir_pid_switch = 0;//闭环导航关闭
        pid_heading_control.dir_time = 0;
        pid_motor.PID_control_yaw = IMU_updata.imu_data[2];
        dir_pid_control.error_k0 = 0;
        dir_pid_control.error_k1 = 0;
        dir_pid_control.error_k2 = 0;
        dir_pid_control.pid_result = 0;
    }

//当目标值为0时，将输出占空比直接清零减速(四个电机同步)，将PID累积量一并清零防止下次指令受到前一次积累量的干扰
    if(pid_motor.PID_target_fl == 0 || pid_motor.PID_target_fr == 0 )
    {
        pid_br = 0;
        pid_fr = 0;
        pid_bl = 0;
        pid_fl = 0;
        bzero(average,sizeof(average));
        input_pid_br.pid_result = 0;
        input_pid_br.error_k0 = 0;
        input_pid_br.error_k1 = 0;

        input_pid_fr.pid_result = 0;
        input_pid_fr.error_k0 = 0;
        input_pid_fr.error_k1 = 0;

        input_pid_bl.pid_result = 0;
        input_pid_bl.error_k0 = 0;
        input_pid_bl.error_k1 = 0;

        input_pid_fl.pid_result = 0;
        input_pid_fl.error_k0 = 0;
        input_pid_fl.error_k1 = 0;

    }


    /**************************电机运动方向控制****************************/
    // RIGHT
    if(pid_fr >= 0)
    {
        rt_pin_write ( MOTOR_DIR_R , PIN_HIGH );  //右前
    }
    else {
        rt_pin_write ( MOTOR_DIR_R , PIN_LOW );  //右后
    }
    pid_fr_output = abs(pid_fr);
    rt_pwm_set ( pwm_dev1 , MOTOR_R_IN , PERIOD , pid_fr_output );
    /*********************/

    /*********************/
    // LEFT
    if(pid_fl >= 0)
    {
        rt_pin_write ( MOTOR_DIR_L , PIN_LOW );  //左前
    }
    else {
        rt_pin_write ( MOTOR_DIR_L , PIN_HIGH );  //左后
    }
    pid_fl_output = abs(pid_fl);
    rt_pwm_set ( pwm_dev1 , MOTOR_L_IN , PERIOD , pid_fl_output );
    /*********************/

    /********************PID控制************************/
#endif
    //里程累加
//        path.path_fr+= speed_pid_s.pid_rpm_fr;
//        path.path_br+= speed_pid_s.pid_rpm_br;
//        path.path_bl+= speed_pid_s.pid_rpm_bl;
//        path.path_fl+= speed_pid_s.pid_rpm_fl;

//        LOG_I("%d,%d,%d,%d\n",rpm_br, rpm_fr, rpm_bl,rpm_fl);
    //脉冲捕获数据手动清零
    PWM_FallingCount_2 = 2000000;
    PWM_FallingCount_3 = 2000000;
    PWM_FallingCount_4 = 2000000;
    PWM_FallingCount_5 = 2000000;

	  return 0;
}

extern void motor_init_encoder ( void );

int hwtimer_init ( void )
{
	  rt_err_t ret = RT_EOK;
	  rt_hwtimerval_t timeout_s; /* 定时器超时值 */
	  rt_device_t hw_dev = RT_NULL; /* 定时器设备句柄 */
	  rt_hwtimer_mode_t mode; /* 定时器模式 */
	  rt_uint32_t freq = 10000; /* 计数频率 */

	  Weight3 = (double) Weight1 / (double) Weight2;
	  motor_init_encoder ( );
	  PID_param_init(&input_pid_fr);
	  PID_param_init(&input_pid_fl);
	  PID_param_dir_init(&dir_pid_control);

	  /* 查找定时器设备 */
	  hw_dev = rt_device_find ( HWTIMER_DEV_NAME );
	  if ( hw_dev == RT_NULL )
	  {
			LOG_D( "hwtimer sample run failed! can't find %s device!\n" , HWTIMER_DEV_NAME );
			return RT_ERROR;
	  }

	  /* 以读写方式打开设备 */
	  ret = rt_device_open ( hw_dev , RT_DEVICE_OFLAG_RDWR );
	  if ( ret != RT_EOK )
	  {
			LOG_D( "open %s device failed!\n" , HWTIMER_DEV_NAME );
			return ret;
	  }

	  /* 设置超时回调函数 */
	  rt_device_set_rx_indicate ( hw_dev , timeout_cb );

	  /* 设置计数频率(若未设置该项，默认为1Mhz 或 支持的最小计数频率) */
	  rt_device_control ( hw_dev , HWTIMER_CTRL_FREQ_SET , &freq );
	  /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
	  mode = HWTIMER_MODE_PERIOD;
	  ret = rt_device_control ( hw_dev , HWTIMER_CTRL_MODE_SET , &mode );
	  if ( ret != RT_EOK )
	  {
			LOG_D( "set mode failed! ret is :%d\n" , ret );
			return ret;
	  }

	  /* 设置定时器超时值为5s并启动定时器 */
	  timeout_s.sec = 0; /* 秒 */
	  timeout_s.usec = PER * 1000; /* 微秒 */
	  if ( rt_device_write ( hw_dev , 0 , &timeout_s , sizeof(timeout_s) ) != sizeof(timeout_s) )
	  {
			LOG_D( "set timeout value failed\n" );
			return RT_ERROR;
	  }

	  return ret;
}

