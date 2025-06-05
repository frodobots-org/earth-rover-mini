/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-16     luozs       the first version
 */
#include "main.h"
#include <math.h>

#define DBG_TAG "strategy"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

rt_timer_t overload_timer;  //过载或堵转保护判断时间
rt_timer_t cool_timer;  //冷却禁用时间

rt_bool_t volatile protect_flag = RT_FALSE;  //被底部监管保护的值
rt_bool_t volatile nedd_control_flag = RT_FALSE;
rt_uint32_t continue_time = 0;  //连续监控时间


void protect_overload_thread_entry ( void *parameter )
{
	  while ( 1 )
	  {
			//如果电压偏低则认为是还没开机，则不进行保护判断
			if ( robot_state.voltage <= 81 )
			{
				  robot_state.speed = 0;
				  robot_state.steer = 0;
				  LOG_W( "控制系统上电状态：%d ，执行系统上电状态：%d，疑似系统未上总电，如有需要请检查电源是否开启！" , robot_state.pwr , 0 );
			}
			else
			{
				  //只要值不为0，则进行监控
				  if ( (robot_state.speed != 0 || robot_state.steer != 0) && robot_state.pwr == 1 )
				  {
						//判断堵转的条件
						if ( (robot_state.rpm [ 0 ] == 0) || (robot_state.rpm [ 1 ] == 0)
						            || (robot_state.rpm [ 2 ] == 0) || (robot_state.rpm [ 3 ] == 0) )
						{
							  continue_time++;
							  if ( continue_time >= LOCKED_ROTOR_TIME )  //连续达到8s速度还上不来
							  {
									robot_state.speed = 0;
									robot_state.steer = 0;
									protect_flag = RT_TRUE;
									if ( nedd_control_flag == RT_FALSE )
									{
										  robot_state.locked_rotor_num++;
										  nedd_control_flag = RT_TRUE;
										  rt_timer_start ( cool_timer );
										  LOG_W( "发生连续堵转，控制停转，启动自然冷却！已发生堵转次数：%d" , robot_state.locked_rotor_num );
									}
							  }
						}
						//判断过载的条件-常规速度（该条件包含堵转）
						if ( ((abs ( robot_state.speed ) != 0) && (abs ( robot_state.speed ) <= 75))
						            || ((abs ( robot_state.steer ) != 0) && (abs ( robot_state.steer ) <= 75)) )
						{
							  if ( abs ( robot_state.rpm [ 0 ] ) <= NOR_SPEED_OVERLOAD_RPM
								          || abs ( robot_state.rpm [ 1 ] ) <= NOR_SPEED_OVERLOAD_RPM
								          || abs ( robot_state.rpm [ 2 ] ) <= NOR_SPEED_OVERLOAD_RPM
								          || abs ( robot_state.rpm [ 3 ] ) <= NOR_SPEED_OVERLOAD_RPM )
							  {
									continue_time++;
									if ( continue_time >= OVER_LOADER_TIME )  //连续达到16s速度还上不来
									{
										  robot_state.speed = 0;
										  robot_state.steer = 0;
										  protect_flag = RT_TRUE;
										  if ( nedd_control_flag == RT_FALSE )
										  {
												robot_state.over_loader_num++;
												nedd_control_flag = RT_TRUE;
												rt_timer_start ( cool_timer );
												LOG_W( "常规速度过载，控制停转，启动自然冷却！已发生过载次数：%d " , robot_state.over_loader_num );
										  }
									}
							  }
						}
						//判断过载的条件-最快速度（该条件包含堵转）
						if ( ((abs ( robot_state.speed ) != 0) && (abs ( robot_state.speed ) > 75))
						            || ((abs ( robot_state.steer ) != 0) && (abs ( robot_state.steer ) > 75)) )
						{
							  if ( abs ( robot_state.rpm [ 0 ] ) <= FAST_SPEED_OVERLOAD_RPM
								          || abs ( robot_state.rpm [ 1 ] ) <= FAST_SPEED_OVERLOAD_RPM
								          || abs ( robot_state.rpm [ 2 ] ) <= FAST_SPEED_OVERLOAD_RPM
								          || abs ( robot_state.rpm [ 3 ] ) <= FAST_SPEED_OVERLOAD_RPM )
							  {
									continue_time++;
									if ( continue_time >= OVER_LOADER_TIME )  //连续达到16s速度还上不来
									{
										  robot_state.speed = 0;
										  robot_state.steer = 0;
										  protect_flag = RT_TRUE;
										  if ( nedd_control_flag == RT_FALSE )
										  {
												robot_state.over_loader_num++;
												nedd_control_flag = RT_TRUE;
												rt_timer_start ( cool_timer );
												LOG_W( "加速下过载，控制停转，启动自然冷却！已发生过载次数：%d" , robot_state.over_loader_num );
										  }
									}
							  }
						}
				  }
				  else  //后续可优化该条件，意思是只要以上条件的持续不够则不累计，后续可根据实际测试情况调整
				  {
						rt_thread_delay ( 100 );
						if ( (robot_state.rpm [ 0 ] != 0) && (robot_state.rpm [ 1 ] != 0)
						            && (robot_state.rpm [ 2 ] != 0) && (robot_state.rpm [ 3 ] != 0) )
						{
							  continue_time = 0;
							  protect_flag = RT_FALSE;
						}
				  }
			}
			rt_thread_delay ( 100 );
	  }
}

static void cool_timer_timeout ( void *parameter )
{
	  continue_time = 0;
	  protect_flag = RT_FALSE;
	  nedd_control_flag = RT_FALSE;
	  LOG_I( "冷却时间达到，允许再次启动！" );
}

void strategy_init ( void )
{
	  cool_timer = rt_timer_create ( "cool_timer" , cool_timer_timeout , RT_NULL , 3000 , RT_TIMER_FLAG_ONE_SHOT );
}
