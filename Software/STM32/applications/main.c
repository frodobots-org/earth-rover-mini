/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-29     RT-Thread    first version
 */
#include "main.h"
#include "fal.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "pwm_ic.h"
#include "QMC5883L.h"
#include "./WS2812/ws2812b.h"

static rt_thread_t uart_tid = RT_NULL;
static rt_thread_t uart_tim_tid = RT_NULL;
static rt_thread_t motor_tid = RT_NULL;
static rt_thread_t state_tid = RT_NULL;
//static rt_thread_t sensor_tid = RT_NULL;
static rt_thread_t protect_tid = RT_NULL;
static rt_thread_t imu_tid = RT_NULL;
static rt_thread_t pid_uart_tid = RT_NULL;
static rt_thread_t motor_test_tid = RT_NULL;

// Externally defined functions for thread startup
extern void uart_thread_entry ( void *parameter );
extern void uart_send_thread_entry ( void *parameter );
extern void serial_thread_entry(void *parameter);
extern void motor_thread_entry ( void *parameter );
extern void state_thread_entry ( void *parameter );
extern void sensor_thread_entry ( void *parameter );
extern void protect_overload_thread_entry ( void *parameter );
extern void imu_thread_entry ( void *parameter );
extern void pid_uart_thread_entry ( void *parameter );
extern void motor_test_thread_entry ( void *parameter );
extern void set_pwr_rgbled_color ( rt_bool_t red , rt_bool_t green , rt_bool_t blue );

extern void set_sys_rgb_led_color_flash (rt_uint16_t on_time, system_state_t sys_status);
extern void LED_system_init(rt_uint16_t on_time,float black_coeff);
extern void robot_lamp_on ( void );
extern void robot_lamp_off ( void );

extern int wdt_init ( void );
extern int hwtimer_init ( void );
extern void motor_init_pwm ( void );
extern void strategy_init ( void );
extern void led_pwr_status_init ( void );

extern void robot_power_on ( void );
extern void robot_power_off ( void );
extern void robot_power_out_on ( void );
extern void robot_power_out_off ( void );
RTC_HandleTypeDef hrtc; //local instnace of RTC

extern void rt_memory_info(rt_uint32_t *total, rt_uint32_t *used, rt_uint32_t *max_used);

void check_heap_size(void)
{
    rt_uint32_t total, used, max_used;
    rt_memory_info(&total, &used, &max_used);

    rt_kprintf("Heap Total: %d bytes\n", total);
    rt_kprintf("Heap Used: %d bytes\n", used);
    rt_kprintf("Heap Max Used: %d bytes\n", max_used);
}
MSH_CMD_EXPORT(check_heap_size, check RT-Thread heap size);

int main ( void )
{
	  fal_init ( ); // initialized all flash devices and partitions
//	  strategy_init ( );

	// defines thread for Universal Asynchronous Receive-Transmission protocol
	// Function: rt_thread_create
	// Input: 
	// 		- Thread name
	// 		- Thread Entry Function
	//		- Extra Parameters (unsure what these parameters are for)
	//		- Stack size needed
	// 		- Priority Level
	// 		- Tick (unsure what tick is)
	// Output: rt_thread struct. Struct has name, type, flags, and MANY more parameters.
	  uart_tid = rt_thread_create ( "uart" , uart_thread_entry , RT_NULL , 24 * 1024 , 20 , 15 ); // stack size = 24*1024, priority = 20, tick = 15

	  if ( uart_tid != RT_NULL )
	  {
	      rt_thread_startup ( uart_tid ); // put thread obj to general thread startup function
	  }

	  uart_tim_tid = rt_thread_create ( "uart_tim_send" , uart_send_thread_entry , RT_NULL , 12 * 1024 , 20 , 15 );

	  if ( uart_tim_tid != RT_NULL )
	  {
	      rt_thread_startup ( uart_tim_tid );
	  }

	  motor_tid = rt_thread_create ( "motor" , motor_thread_entry , RT_NULL , 4 * 1024 , 15 , 25 );
	  if ( motor_tid != RT_NULL )
	  {
			rt_thread_startup ( motor_tid );
	  }

	  state_tid = rt_thread_create ( "state" , state_thread_entry , RT_NULL , 12 * 1024 , 25 , 11 );
	  if ( state_tid != RT_NULL )
	  {
			rt_thread_startup ( state_tid );
	  }

	  imu_tid = rt_thread_create ( "imu" , imu_thread_entry , RT_NULL , 12 * 1024 , 24 , 25 );
	  if ( imu_tid != RT_NULL )
	  {
	      rt_thread_startup ( imu_tid );
	  }

//	  motor_test_tid = rt_thread_create ( "motor_test" , motor_test_thread_entry , RT_NULL , 4 * 1024 , 24 , 26 );
//	  if ( motor_test_tid != RT_NULL )
//	  {
//	      rt_thread_startup ( motor_test_tid );
//	  }

//	  pid_uart_tid = rt_thread_create ( "pid_uart" , pid_uart_thread_entry , RT_NULL , 16 * 1024 , 30 , 10 );
//	  if ( pid_uart_tid != RT_NULL )
//	  {
//	      rt_thread_startup ( pid_uart_tid );
//	  }

//	  sensor_tid = rt_thread_create ( "state" , sensor_thread_entry , RT_NULL , 4096 , 25 , 10 );
//	  if ( sensor_tid != RT_NULL )
//	  {
//			rt_thread_startup ( sensor_tid );
//	  }

//	  protect_tid = rt_thread_create ( "protect" , protect_overload_thread_entry , RT_NULL , 4096 , 25 , 10 );
//	  if ( protect_tid != RT_NULL )
//	  {
//			rt_thread_startup ( protect_tid );
//	  }
	  wdt_init ( );
	  hwtimer_init ( ); // assuming this inits the hwtimer.c functionality

	  /**WS2812 Init**/
	  ws2812_init(); // Initializes WS2812 driver
	  ws2812_clearn_all(2);
	  
	  robot_power_init ();//开关机初始化 = Power on/off initialization
	  robot_charge_init();//充电检测初始化 = Charging detection initialization
	  
	  // 检测 OTA 状态 = Check OTA status
	  hrtc.Instance = RTC;
	  HAL_RTC_Init(&hrtc);
	  uint32_t ota_reboot=HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
//	  rt_kprintf("ota_flag = 0x%X\n",ota_flag);
	  if (ota_reboot == 0xA5A5) {
	      // OTA 后首次重启逻辑 = First reboot after OTA logic
	      robot_state.pwr = 1;
	      robot_state.status = SYSTEM_STATE_RUNNING;
	      robot_state.net_led_status = UCP_NETWORK_CONNECTED;
	      robot_power_on ( );
	      robot_power_out_on ( );

	      // 清除标志，防止下次复位依然触发 = Clear the flag to prevent triggering again on next reset
	      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x0000);
	  } else {
	      // 普通启动逻辑 = Normal startup logic
	      robot_state.status = SYSTEM_STATE_INITIAL;
	  }

	  while ( 1 )
	  {
		  // 充电状态检测任务(优先判定) = Charging status detection task (priority determination)
	      robot_charge_task();

	      switch (robot_state.status) {
              case SYSTEM_STATE_CHARGING:
			  		// Function: set_sys_rgb_led_color_flash
					// Input: 
					// 		- rt_uint16_t (int) on_time
					// 		- system_state_t (int) sys_status = code associated with robot state.
					// Output: NULL

					// Function: LED_system_init
					// Input: 
					// 		- 
					// Output: NULL

					// Function: rt_thread_mdelay
					// Input: 
					// 		- 
					// Output: NULL
					
				  set_sys_rgb_led_color_flash(1000,0);// 充电中，更新电源指示灯为慢闪烁状态 = Charging, update power indicator to slow flashing
				  break;
              case SYSTEM_STATE_CHARGED:
				  set_sys_rgb_led_color_flash(1000,1);// 充电完毕，更新电源指示灯为常亮状态 = Charging complete, update power indicator to steady on
				  break;
	          case SYSTEM_STATE_INITIAL:
				  LED_system_init(100,0.5);// 初始状态(开机提示) = Initial state (power-on prompt)
	              break;
	          case SYSTEM_STATE_RUNNING:
				  set_sys_rgb_led_color_flash(1000,3);// 系统开机运行中，执行正常任务 = System running, performing normal tasks
	              break;
	          case SYSTEM_STATE_WARNING:
				  set_sys_rgb_led_color_flash(300,0);// 系统电量不足10%，红灯快闪烁警告 = System power less than 10%, red light fast flashing warning
	              break;
	          case SYSTEM_STATE_SHUTDOWN:
				  rt_thread_mdelay ( 800 );// 系统关机或停止运行，进入低功耗状态 = System shutdown or stopped, enter low power state
	              break;
	          default:
	              break;
	      }
		  // main function - boot up main threads for taking care of individual parts of robot, once done, and robot_state is setup: only look at power state and take care of LED signals for power.

//			clock_information();
//			rt_kprintf("System Clock information\n");
//			rt_kprintf("SYSCLK_Frequency = %d\n", HAL_RCC_GetSysClockFreq());
//			rt_kprintf("HCLK_Frequency   = %d\n", HAL_RCC_GetHCLKFreq());
//			rt_kprintf("PCLK1_Frequency  = %d\n", HAL_RCC_GetPCLK1Freq());
//			rt_kprintf("PCLK2_Frequency  = %d\n", HAL_RCC_GetPCLK2Freq());
//			LOG_I( "LED_PWR_G = %d" ,rt_pin_read(LED_PWR_G));
//			rt_thread_mdelay ( 100 );
//			rt_pin_write ( LED_PWR_G , 1 );
//			LOG_I( "LED_PWR_G = %d" ,rt_pin_read(LED_PWR_G));
//			LOG_I( "OTA successful! \n");

	  }
	  return RT_EOK;
}

#if 1
static int ota_app_vtor_reconfig ( void )
{
#define NVIC_VTOR_MASK 0x3FFFFF80
#define RT_APP_PART_ADDR 0x08020000
	  SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

	  return 0;
}
INIT_BOARD_EXPORT( ota_app_vtor_reconfig );
#endif

