/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-15     Aaron       the first version
 */
#include "main.h"

#define DBG_TAG "STATE"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "state.h"
#include <rtthread.h>

#include <stdlib.h>
#include "QMC5883L.h"
#include "pid.h"
#include "INA226.h"
#include "./WS2812/ws2812b.h"
#include "imu.h"

#define LOW_VOLTAGE_PROTECT   // 低压保护 = Low-voltage protection
extern void vl53l0x_init_all ( );

rt_adc_device_t adc_dev;
#define ADC_DEV_NAME "adc1" /* ADC 设备名称 = ADC device name */
#define ADC_DEV_MOTOR_RIGHT_CURRENT 3 /* ADC 通道 = ADC channel */
#define ADC_DEV_MOTOR_LEFT_CURRENT 4 /* ADC 通道 = ADC channel */
#define REFER_VOLTAGE 330 /* 参考电压 3.3V, 数据精度乘以100保留2位小数 
                           * Reference voltage = 3.3V, multiply by 100 to keep 2 decimal places */
#define CONVERT_BITS (1 << 12) /* 转换位数为12位 = Conversion resolution is 12 bits */
rt_mutex_t state_data_mutex;

rt_timer_t key_timer;
rt_timer_t pwr_timer;
volatile robot_state_t robot_state;

/* LED color constants (in RGB hex) */
uint32_t red      = 0xFF0000U;
uint32_t green    = 0x00FF00U;
uint32_t blue     = 0x0000FFU;
uint32_t yellow   = 0xFF7F00U;
uint32_t purple   = 0x8F00FFU;
uint32_t cyan     = 0x00FFFFU;
uint32_t sunlight = 0xFFFF00U;
uint32_t unknow   = 0xFFD700U;
uint32_t black    = 0x000000U;

/* 
 * Robot Power Control Functions
 * These toggle GPIO pins that control robot's main power circuits.
 */
void robot_power_on ( void )
{
    rt_pin_write ( PWR_ON , PIN_HIGH );
    rt_thread_delay ( 10 );
    rt_pin_write ( PWR_CLK , PIN_LOW );
    rt_thread_delay ( 20 );
    rt_pin_write ( PWR_CLK , PIN_HIGH );
    rt_thread_delay ( 20 );
    rt_pin_write ( PWR_CLK , PIN_LOW );
}

void robot_power_off ( void )
{
    rt_pin_write ( PWR_ON , PIN_LOW );
    rt_thread_delay ( 10 );
    rt_pin_write ( PWR_CLK , PIN_LOW );
    rt_thread_delay ( 20 );
    rt_pin_write ( PWR_CLK , PIN_HIGH );
    rt_thread_delay ( 20 );
    rt_pin_write ( PWR_CLK , PIN_LOW );
}

void robot_power_out_on ( void )
{
    rt_pin_write ( PWR_OUT_ON , PIN_LOW );
}

void robot_power_out_off ( void )
{
    rt_pin_write ( PWR_OUT_ON , PIN_HIGH );
}

/** ====================== WS2812B LED Status Handling ====================== **/
typedef enum {
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_RED
} Color;

/**
 * 系统运行状态灯显示 = System running status indicator light display 
 * 
 * Controls WS2812B RGB LEDs to indicate:
 *   - Battery level (with hysteresis to prevent flicker)
 *   - Charging state (slow blink)
 *   - IMU calibration progress (purple blink on secondary LED)
 *   - Network connection state (various colors/blink rates)
 *   - OTA update (blue fast blink)
 */
void set_sys_rgb_led_color_flash (rt_uint16_t on_time, system_state_t sys_status)
{
    rt_uint32_t received_flags = 0;
    uint32_t color_buf[2] = {0};
    static Color current_color = COLOR_GREEN;
    static uint8_t charge_flag = 1;
    static uint8_t imu_calib = 0;

    /** 电量指示灯状态 = Battery indicator light status **/
    switch (current_color) {
    case COLOR_GREEN:
    {
        color_buf[0] = green;
        if (robot_state.battery < 67.0) { 
            // 60% - 3% 滞回 = 60% - 3% hysteresis
            // Switch from green → yellow when battery < 67%
            current_color = COLOR_YELLOW;
            color_buf[0] = yellow;
        }
    }
    break;
    case COLOR_YELLOW:
    {
        color_buf[0] = yellow;
        if (robot_state.battery > 73.0) { 
            // 60% + 3% 滞回 = Switch yellow → green when battery > 73%
            current_color = COLOR_GREEN;
            color_buf[0] = green;
        } else if (robot_state.battery < 27.0) { 
            // 20% - 3% 滞回 = Switch yellow → red when battery < 27%
            current_color = COLOR_RED;
            color_buf[0] = red;
        }
    }
    break;
    case COLOR_RED:
    {
        color_buf[0] = red;
        if (robot_state.battery > 33.0) { 
            // 20% + 3% 滞回 = Switch red → yellow when battery > 33%
            current_color = COLOR_YELLOW;
            color_buf[0] = yellow;
        }
    }
    break;
    }

    /* Charging state handling: slow blink */
    if(sys_status == SYSTEM_STATE_CHARGING) // 判断到充电状态电源指示灯进行慢闪 
    {
        if(charge_flag)
        {
            // LED ON
        }
        else {
            color_buf[0] = black; // LED OFF → blink effect
        }
        charge_flag = !charge_flag;
    }

    /** IMU校准事件 = IMU calibration event **/
    if(imu_event != RT_NULL)
    {
        if (rt_event_recv(imu_event, IMU_CALIB_LED_START | IMU_CALIB_LED_DONE,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_NO, &received_flags) == RT_EOK)
        {
            if(received_flags & IMU_CALIB_LED_START)
            {
                imu_calib = 1;  // Start calibration
            }
            if(received_flags & IMU_CALIB_LED_DONE)
            {
                imu_calib = 0;  // End calibration
            }
        }
    }

    /* If IMU calibration is ongoing → secondary LED blinks purple */
    if(imu_calib) 
    {
        color_buf[1] = purple;
        ws2812b_write_test(2,color_buf); 
        rt_thread_delay( on_time * 0.3);
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf); 
        rt_thread_delay( on_time * 0.3);
        return;
    }

    /** 联网状态判定 = Network connection status determination **/
    color_buf[1] = black;
    if(robot_state.pwr == 1) // if robot is powered on 
    {
        switch ( robot_state.net_led_status )
        {
        case UCP_STATE_UNKNOWN :  // 默认状态 = Default state
        {
            color_buf[1] = red; // solid red
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time );
        }
        break;
        case UCP_STATE_SIMABSENT :  // 没有SIM卡(慢闪) = No SIM card (slow blink)
        {
            color_buf[1] = red;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time);
            color_buf[1] = black;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time );
        }
        break;
        case UCP_NETWORK_DISCONNECTED :  // 联网中(快闪) = Connecting to network (fast blink)
        {
            color_buf[1] = green;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time * 0.3  );
            color_buf[1] = black;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time * 0.3  );
        }
        break;
        case UCP_NETWORK_CONNECTED : // 联网结束(绿色常亮) = Connected (green solid)
        {
            color_buf[1] = green;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time );
        }
        break;
        case UCP_OTA_ING :   // OTA升级(蓝色快闪) = OTA upgrade (blue fast blink)
        {
            color_buf[1] = blue;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time * 0.3  );
            color_buf[1] = black;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time * 0.3  );
        }
        break;
        }
    }
    else {
        /* If robot is powered off → just show battery color */
        ws2812b_write_test(2,color_buf);
        rt_thread_delay( on_time );
    }
}


void LED_system_init(rt_uint16_t on_time,float black_coeff)
/**
 * @brief Initialize the system LED sequence at startup.
 *
 * This function performs a startup LED animation by cycling through
 * different colors (red, yellow, green, blue, cyan, purple). 
 * Each color is displayed briefly, followed by a black (off) period 
 * scaled by `black_coeff`. The animation only runs if the robot is 
 * powered OFF (`robot_state.pwr == 0`). 
 *
 * @param on_time     Duration (ms) each color stays ON
 * @param black_coeff Multiplier for the OFF delay (longer blackout between colors)
 */
{
    LOG_E("LED_SYSTEM_INIT!!!");
    uint32_t color_buf[2] = {0};   // buffer for storing LED colors (two channels)

    // If robot is OFF, blink RED first
    if(robot_state.pwr == 0)
    {
        color_buf[0] = red;
        color_buf[1] = red;
        ws2812b_write_test(2,color_buf);     // send red to LEDs
        rt_thread_mdelay(on_time);           // keep ON
        color_buf[0] = black;
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf);     // turn LEDs OFF
        rt_thread_mdelay(on_time * black_coeff);
    }
    else {
        return; // if robot already ON, skip startup sequence
    }

    // YELLOW stage
    if(robot_state.pwr == 0)
    {
        color_buf[0] = yellow;
        color_buf[1] = yellow;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time);
        color_buf[0] = black;
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time * black_coeff);
    }
    else {
        return;
    }

    // GREEN stage
    if(robot_state.pwr == 0)
    {
        color_buf[0] = green;
        color_buf[1] = green;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time);
        color_buf[0] = black;
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time * black_coeff);
    }
    else {
        return;
    }

    // BLUE stage
    if(robot_state.pwr == 0)
    {
        color_buf[0] = blue;
        color_buf[1] = blue;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time);
        color_buf[0] = black;
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time * black_coeff);
    }
    else {
        return;
    }

    // CYAN stage
    if(robot_state.pwr == 0)
    {
        color_buf[0] = cyan;
        color_buf[1] = cyan;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time);
        color_buf[0] = black;
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time * black_coeff);
    }
    else {
        return;
    }

    // PURPLE stage
    if(robot_state.pwr == 0)
    {
        color_buf[0] = purple;
        color_buf[1] = purple;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time);
        color_buf[0] = black;
        color_buf[1] = black;
        ws2812b_write_test(2,color_buf);
        rt_thread_mdelay(on_time * black_coeff);
    }
    else {
        return;
    }
}

/* --------------------------------------------------------------------
 * Key button handling
 * --------------------------------------------------------------------
 */

rt_timer_t key_timer;                  // timer used for detecting press duration
volatile rt_bool_t key_pressed = RT_FALSE;   // whether key is currently pressed
volatile rt_tick_t key_press_start_time = 0; // timestamp when press started
volatile rt_tick_t key_press_stop_time = 0;  // timestamp when press stopped

//#define SW_MODE   // Software mode switch (use debounce + direct power toggling)
#ifndef SW_MODE
/**
 * @brief Key/button interrupt handler (ISR).
 *
 * Note: At startup, button presses cannot be detected immediately 
 * because the system is still initializing. 
 *
 * This version uses a timer to measure press duration and handles
 * press/release detection via interrupts.
 */
void key_isr ( void *args )  
// 要注意开机的时候是没办法检测到按键的变化的，因为那会按键才刚按下，系统才刚开始初始化 
// = Note: At startup, key/button changes cannot be detected because the 
//   system has just begun initializing.
{
    int value = rt_pin_read ( PWR_DEC ); // read button pin

    if ( value == 0 ) /* 按键按下（低电平） = Button pressed (active low) */
    {
        if ( !key_pressed )
        {
            key_pressed = RT_TRUE;
            key_press_start_time = rt_tick_get_millisecond(); // log press start
            rt_timer_start ( key_timer ); // start timer for measuring duration
//            rt_kprintf("0\n");
        }
    }
    else /* 按键松开（高电平） = Button released (high level) */
    {
        if ( key_pressed )
        {
            key_pressed = RT_FALSE;
            key_press_stop_time = rt_tick_get_millisecond(); // log release
            rt_timer_stop ( key_timer );  // 预防按键达不到要求的时间，需要关掉定时器 
                                          // = To prevent false detections when the 
                                          // button press time is too short
//            rt_kprintf("1\n");
        }
    }
}
#else
/**
 * @brief Alternate key ISR (software mode with debounce).
 *
 * In this mode, a simple debounce delay is used. Press toggles 
 * the robot ON, release toggles it OFF.
 */
void key_isr ( void *args )  
// 要注意开机的时候是没办法检测到按键的变化的，因为那会按键才刚按下，系统才刚开始初始化
// = Note: At startup, button changes cannot be detected because 
//   the system has just started initializing.
{
    rt_thread_mdelay ( 20 ); // 延时消抖 = Delay for debounce
    int value = rt_pin_read ( PWR_DEC );

    if ( value == 0 ) /* 按键按下（低电平） = Button pressed (low level)*/
    {
        if(robot_state.pwr == 0)
        {
            robot_state.pwr = 1;   // set power state
            robot_power_on();      // enable power rails
            robot_power_out_on();  // enable external power output
        }
    }
    else /* 按键松开（高电平）= Button released (high level) */
    {
        if(robot_state.pwr == 1)
        {
            robot_state.pwr = 0;   // clear power state
            robot_power_off();     // shut down power rails
        }
    }

}
#endif
/* 按键检测任务 = Button detection task */
#ifndef SW_MODE
/**
 * @brief Button detection task (timer + press duration method).
 *
 * This function monitors button press/release events and determines
 * whether to power the robot ON or OFF based on press duration.
 *
 * - Uses long press detection (different thresholds for ON vs OFF).
 * - Includes low-voltage protection before allowing power-on.
 * - Handles special case where edges cannot be detected at startup.
 */
void key_detect_task ( void )
{
    rt_uint32_t timer_time = KEY_LONG_PRESS_TIME - SYS_START_UP_TIME;

    if ( key_pressed == RT_TRUE )
    {
        rt_tick_t duration = key_press_stop_time - key_press_start_time;

        if ( key_press_stop_time < key_press_start_time )  
            // 按键还未松开 = Button is still pressed
        {
            return;
        }

        // 达到开关机时间 = Reached power on/off time
        if ( duration > 0 )
        {
            if ( robot_state.pwr == 0 && duration >= timer_time )  
                // 开机时因无法识别边沿，故只能查询，需减去系统启动到查询的时间
                // = At startup, edges cannot be detected, so polling is used, 
                //   and SYS_START_UP_TIME is subtracted.
            {
#ifdef LOW_VOLTAGE_PROTECT
                // 低压保护 = Low-voltage protection
                if(robot_state.voltage <= 9.30f)
                {
                    duration = 0;
                    key_press_stop_time = 0;
                    robot_state.pwr = 0;
                    robot_state.status = SYSTEM_STATE_SHUTDOWN;
                    ws2812_clearn_all(2); // 关机前关闭系统状态灯 = Turn off system LED before shutdown
                    robot_power_off();
                    robot_power_out_off();
                    return;
                }
#endif
                duration = 0;
                key_press_stop_time = 0;
                robot_state.pwr = 1;
                robot_state.status = SYSTEM_STATE_RUNNING;
                robot_power_on();
                robot_power_out_on();
                LOG_I("PWR ON");
            }
            else if ( robot_state.pwr == 1 && duration >= KEY_LONG_PRESS_TIME )
            {
                duration = 0;
                key_press_stop_time = 0;
                robot_state.pwr = 0;
                robot_state.status = SYSTEM_STATE_SHUTDOWN;
                ws2812_clearn_all(2); // 关机前关闭系统状态灯 = Turn off system LED before shutdown
                robot_power_off();
                robot_power_out_off();
                LOG_I("PWR OFF");
            }
        }
    }
    else  
        // 主要是考虑开机时没有办法识别按键的边沿,以及开机后插入
        // = Accounts for startup edge detection failure and late button presses
    {
        if ( key_pressed != RT_TRUE )  
            // 开机时一瞬间无法进入按键中断，从这里进
            // = At power-on, interrupts may miss button edges, so handle here
        {
            if ( rt_pin_read(PWR_DEC) == PIN_LOW )
            {
                if ( robot_state.pwr == 0 )  
                    // 如果是开机，则修改定时器时间
                    // = If powering on, adjust timer time
                {
                    key_pressed = RT_TRUE;
                    key_press_start_time = rt_tick_get_millisecond();
                    rt_timer_control(key_timer, RT_TIMER_CTRL_SET_TIME, &timer_time);
                    rt_timer_start(key_timer);
                }
            }
        }
    }
}
#else
/**
 * @brief Simple button detection task (software mode).
 *
 * In this simplified mode, pressing the button directly powers ON the robot.  
 * No long-press detection or timer logic is used.
 */
void key_detect_task ( void )
{
    if ( rt_pin_read(PWR_DEC) == PIN_LOW )
    {
        if(robot_state.pwr == 0)
        {
            robot_state.pwr = 1;
            robot_power_on();
            robot_power_out_on();
            LOG_I("PWR ON");
        }
    }
}
#endif

/**
 * @brief Key press timer timeout callback.
 *
 * Called when the button press timer expires.  
 * Used to determine whether the press qualifies as a "long press."
 */
static void key_timer_timeout ( void *parameter )
{
    rt_uint32_t timer_time = KEY_LONG_PRESS_TIME;
    int val = rt_pin_read(PWR_DEC);
    if ( val == PIN_LOW )
    {
        // LOG_I("Is long press!");
        key_pressed = RT_TRUE;
        key_press_stop_time = rt_tick_get_millisecond();
    }
    // 因开机按键识别改动，故每次都恢复定时时间 
    // = Due to startup button recognition changes, reset timer each time
    rt_timer_control(key_timer, RT_TIMER_CTRL_SET_TIME, &timer_time);
}

/* --------------------------------------------------------------------
 * Charging port detection
 * --------------------------------------------------------------------
 */

/**
 * @brief Initialize charging port detection pins.
 *
 * - CHARGE_DET: detects whether robot is connected to charger.
 * - CHG_INT: detects if charging has been interrupted.
 */
void robot_charge_init( void )
{
    rt_pin_mode(CHARGE_DET, PIN_MODE_INPUT_PULLDOWN); // detect Type-C/charging port plugged in
    rt_pin_mode(CHG_INT, PIN_MODE_INPUT_PULLDOWN);    // detect charge interrupt
}

/**
 * @brief Charging task: checks charging state and updates robot status.
 *
 * Behavior:
 * - If charging detected (CHARGE_DET or CHG_INT high):
 *   - If voltage >= 12.15V → considered fully charged.
 *   - Else → still charging.
 *   - If robot is OFF, automatically powers ON but disables external head supply.
 *
 * - If not charging:
 *   - Enforce low-voltage protection (shutdown at <= 9.30V).
 *   - Update status to WARNING or RUNNING depending on battery percentage.
 *   - If powered OFF without charging, disable MCU output.
 */
void robot_charge_task( void )
{
    if ( (rt_pin_read(CHARGE_DET) == PIN_HIGH) || (rt_pin_read(CHG_INT) == PIN_HIGH) )
    { 
        // Charging detected
        if(robot_state.voltage >= 12.15f) 
        {
            robot_state.status = SYSTEM_STATE_CHARGED;   // battery full
        }
        else { 
            robot_state.status = SYSTEM_STATE_CHARGING; // still charging
        }

        if(robot_state.pwr == 0)
        { 
            // charging but robot is OFF
            LOG_I("PWR OFF -1");
            robot_power_on();
            robot_power_out_off(); 
            // 未开机状态下充电关闭头部供电 
            // = When charging while OFF, disable head power supply
        }
    }
    else {
#ifdef LOW_VOLTAGE_PROTECT
        // 低压保护 = Low-voltage protection
        if(robot_state.voltage <= 9.30f)
        {
            LOG_E("voltage : %.2f",robot_state.voltage);
            LOG_E("OFF-2XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX%x",robot_state.voltage);
            LOG_E("PWR OFF-2");
            robot_state.status = SYSTEM_STATE_SHUTDOWN; // SYSTEM_STATE_INITIAL
            ws2812_clearn_all(2); // 关机前关闭系统状态灯 = Turn off LED before shutdown
            robot_power_off();
        }
#endif
        if(robot_state.pwr == 1)
        {
            // Manage warning/running status based on battery level
            if(robot_state.status == SYSTEM_STATE_WARNING)
            {
                if(robot_state.battery <= 13) // %3 hysteresis
                {
                    robot_state.status = SYSTEM_STATE_WARNING;
                }
                else
                    robot_state.status = SYSTEM_STATE_RUNNING;
            }
            else {
                if(robot_state.battery <= 10) // %3 hysteresis
                {
                    robot_state.status = SYSTEM_STATE_WARNING;
                }
                else
                    robot_state.status = SYSTEM_STATE_RUNNING;
            }
        }
        else {
            // robot is OFF and no charging
//            robot_state.status = SYSTEM_STATE_INITIAL;
            LOG_I("PWR OFF-3 pwr=%d", robot_state.pwr);
            ws2812_clearn_all(2); // 关机前关闭系统状态灯 = Turn off LED before shutdown
            robot_power_off();    // 若未检测到type-c插入并且未启动按键开机，则关闭MCU电源输出
                                  // = If no Type-C plug detected and no button press, cut MCU power

            if(robot_state.pwr == 1)
            {
                LOG_I("PWR ON-1");
                robot_power_on();
                robot_power_out_on();
            }
        }
    }
}void robot_power_init ( void )
{
    rt_pin_mode ( PWR_ON , PIN_MODE_OUTPUT );
    rt_pin_mode ( PWR_CLK , PIN_MODE_OUTPUT );
    rt_pin_mode ( PWR_OUT_ON , PIN_MODE_OUTPUT );
    rt_pin_mode ( PWR_DEC , PIN_MODE_INPUT_PULLUP );

    // Attach an interrupt service routine (ISR) for the power button pin
    rt_pin_attach_irq ( PWR_DEC , PIN_IRQ_MODE_RISING_FALLING , key_isr , RT_NULL );
    rt_pin_irq_enable ( PWR_DEC , PIN_IRQ_ENABLE );

    // 给一个默认电平 = Give a default logic level
    rt_pin_write ( PWR_ON , PIN_HIGH );   // 默认开机 = Default to powered ON state
    rt_pin_write ( PWR_CLK , PIN_LOW );   // Default clock pin set to LOW
    rt_pin_write ( PWR_OUT_ON , PIN_LOW );// 默认低（打开）= Default low (interpreted as ON/open for output power)
}

void robot_lamp_init ( void )
{
    // Initialize all "car" LEDs as output pins
    rt_pin_mode ( LED_CAR1 , PIN_MODE_OUTPUT );
    rt_pin_mode ( LED_CAR2 , PIN_MODE_OUTPUT );
    rt_pin_mode ( LED_CAR3 , PIN_MODE_OUTPUT );
    rt_pin_mode ( LED_CAR4 , PIN_MODE_OUTPUT );
}


void robot_lamp_on ( void )
{
    // Turn ON all indicator LEDs
    rt_pin_write ( LED_CAR1 , PIN_HIGH );
    rt_pin_write ( LED_CAR2 , PIN_HIGH );
    rt_pin_write ( LED_CAR3 , PIN_HIGH );
    rt_pin_write ( LED_CAR4 , PIN_HIGH );
}

void robot_lamp_off ( void )
{
    // Turn OFF all indicator LEDs
    rt_pin_write ( LED_CAR1 , PIN_LOW );
    rt_pin_write ( LED_CAR2 , PIN_LOW );
    rt_pin_write ( LED_CAR3 , PIN_LOW );
    rt_pin_write ( LED_CAR4 , PIN_LOW );
}

// Example of a possible power-on flag mechanism, but commented out
// rt_uint8_t pwr_on_flag = 0;
// static void pwr_timer_timeout ( void *parameter )
// {
// 	  pwr_on_flag = 1;
// }

#define ADC_SAMPLES 100             // ADC采样数量 = Number of samples to collect for ADC (Analog-to-Digital Converter)
#define ADC_CHANNEL ADC_CHANNEL_0   // 假设使用ADC1的通道0 = Assume we are using ADC1 channel 0

ADC_HandleTypeDef hadc1;
rt_uint32_t adc_values [ ADC_SAMPLES ];   // Array to store raw ADC sample values
rt_uint32_t adc_filtered_value;           // Stores the processed/filtered ADC result

// 中值滤波器 (Median filter): Sorts the sample set and returns the median value.
// This helps reduce noise spikes in analog readings.
rt_uint32_t MedianFilter ( rt_uint32_t *values , rt_uint32_t size )
{
    rt_uint32_t temp;
    for ( rt_uint32_t i = 0 ; i < size / 2 ; i++ )
    {
        for ( rt_uint32_t j = i + 1 ; j < size ; j++ )
        {
            if ( values [ j ] < values [ i ] )
            {
                // Swap values to sort
                temp = values [ i ];
                values [ i ] = values [ j ];
                values [ j ] = temp;
            }
        }
    }
    // Return the middle element (median value)
    return values [ size / 2 ];
}


// 初始化滑动滤波器 = Initialize sliding filter (moving average filter)
// Sets buffer to zero, resets counters and sum
void sliding_filter_init ( SlidingFilter *filter , rt_uint16_t size )
{
    for ( int i = 0 ; i < size ; i++ )
    {
        filter->buffer [ i ] = 0.0;
    }
    filter->index = 0;  // Start index at 0
    filter->count = 0;  // No samples collected yet
    filter->sum = 0.0;  // Reset running sum
}

// 添加新样本到滑动滤波器并返回滤波后的值
// = Add a new sample to the sliding (moving average) filter and return the smoothed value.
float sliding_filter_add_sample ( SlidingFilter *filter , rt_int32_t size , float new_sample )
{
    // 从总和中减去即将被替换的旧样本的值
    // = Subtract the old sample (about to be overwritten) from the sum
    if ( filter->count == size )
    {
        filter->sum -= filter->buffer [ filter->index ];
    }
    else
    {
        // Increase sample count until buffer is full
        filter->count++;
    }

    // 添加新样本到缓冲区并更新其总和
    // = Store the new sample in the buffer and update the running sum
    filter->buffer [ filter->index ] = new_sample;
    filter->sum += new_sample;

    // 更新索引，如果需要则回绕到开始
    // = Advance index and wrap around if necessary (circular buffer behavior)
    filter->index = (filter->index + 1) % size;

    // 计算并返回平均值 = Calculate and return the average (smoothed value)
    return filter->sum / filter->count;
}

/** 限幅滤波接口 = Limit Filter Interface 
 *  Purpose: Rejects sudden spikes by comparing new value with previous one.
 *  If the change exceeds a threshold, the previous value is kept instead.
 */
float limit_filter(float new_value, float previous_value, float threshold) {
    if (fabs(new_value - previous_value) > threshold) {
        return previous_value;  // Sudden spike detected → discard new value
    }
    return new_value;  // Accept new value if within threshold
}

#define GAIN 50 // TP181模块放大器增益 = Amplifier gain of the TP181 module

/**
 * TP181电机电流转化 = TP181 motor current conversion
 * Converts raw ADC output from the TP181 sensor into real motor current.
 * Formula accounts for ADC resolution, reference voltage, amplifier gain, 
 * and the shunt resistor value.
 */
static float motor_current_output(rt_uint32_t voltage_out)
{
    float Rshunt = 0.015f; // 分流电阻阻值15mΩ = Shunt resistor = 15 mΩ
    float current = 0.0f;

    // Convert ADC reading → voltage → amplified signal → actual current
    current = (((((float)voltage_out / (4096 - 1)) * 3.30f) - 3.30f/2.0f)/(GAIN * Rshunt));

    return current;
}


#define THREAD_PRIORITY      9
#define THREAD_TIMESLICE     5

#define EVENT_FLAG3 (1 << 3)
#define EVENT_FLAG5 (1 << 5)

// External variables representing RPM readings from four wheels
extern rt_int32_t rpm_fl;  // Front-left wheel RPM
extern rt_int32_t rpm_fr;  // Front-right wheel RPM
extern rt_int32_t rpm_bl;  // Back-left wheel RPM
extern rt_int32_t rpm_br;  // Back-right wheel RPM

// External accumulators for wheel encoder ticks
extern rt_int32_t sum_t_fl;
extern rt_int32_t sum_t_fr;

static uint16_t calculate_battery_percentage(float voltage);

/**
 * state_thread_entry
 * ------------------
 * Main system state management thread.
 * - Reads sensors (motor currents, INA226 power data, wheel RPM)
 * - Applies filters (sliding + limit filters)
 * - Updates global robot_state (current, voltage, power, RPM, battery %)
 * - Handles lamp control based on system state
 */
void state_thread_entry ( void *parameter )
{
    rt_uint32_t motor_r_adc_value = 0;
    rt_uint32_t motor_l_adc_value = 0;
    float motor_r_current = 0.0f;
    float motor_l_current = 0.0f;
    rt_int32_t value_br=0,value_fr=0,value_bl=0,value_fl=0;
    float voltage = 0;
    float true_voltage = 0;
    float current = 0;
    float power = 0;

    // Initialize peripherals
    robot_lamp_init ( );   // Setup lamp pins
    INA226_init();         // INA226 = power monitoring sensor (voltage, current, power)

    // Initialize sliding filters for smoothing sensor data
    SlidingFilter v_filter , c_filter , p_filter , motor_l_filter,motor_r_filter,rpm_l_filter,rpm_r_filter;
    sliding_filter_init ( &v_filter , V_FILTER_SIZE );        
    sliding_filter_init ( &c_filter , C_FILTER_SIZE );        
    sliding_filter_init ( &p_filter , P_FILTER_SIZE );        
    sliding_filter_init ( &rpm_l_filter , C_FILTER_SIZE );    
    sliding_filter_init ( &rpm_r_filter , C_FILTER_SIZE );    
    sliding_filter_init ( &motor_l_filter , C_FILTER_SIZE );  
    sliding_filter_init ( &motor_r_filter , C_FILTER_SIZE );  

    // Create timer for detecting long key presses
    key_timer = rt_timer_create ( "key_timer" , key_timer_timeout , RT_NULL , KEY_LONG_PRESS_TIME ,
            RT_TIMER_FLAG_ONE_SHOT );

    // Find ADC device
    adc_dev = (rt_adc_device_t) rt_device_find ( ADC_DEV_NAME );
    if ( adc_dev == RT_NULL )
    {
        LOG_E( "adc sample run failed! can't find %s device!\n" , ADC_DEV_NAME );
        return RT_ERROR;
    }

    // Enable ADC channels for motor current measurement
    rt_adc_enable ( adc_dev , ADC_DEV_MOTOR_RIGHT_CURRENT );
    rt_adc_enable ( adc_dev , ADC_DEV_MOTOR_LEFT_CURRENT );

    bzero(&path,sizeof(&path));
    uint8_t filter_time =0;

    // 创建state_data互斥锁 = Create mutex for protecting shared state data
    state_data_mutex = rt_mutex_create("state_data_mutex", RT_IPC_FLAG_PRIO);
    if (state_data_mutex == RT_NULL)
    {
        LOG_E("state_data_mutex creation failed!\n");
        return -RT_ERROR;
    }

    // ==============================
    // Main loop
    // ==============================
    while ( 1 )
    {
        if(filter_time < 50) filter_time ++;

        // Lamp control: Only ON when powered
        if ( robot_state.lamp && robot_state.lamp != 0xff && robot_state.pwr == 1 )
        {
            robot_lamp_on ( );
        }
        else
        {
            robot_lamp_off ( );
        }

        // Check key button state
        key_detect_task ( );

        /** TP181 motor current acquisition **/
        motor_r_adc_value = rt_adc_read ( adc_dev , ADC_DEV_MOTOR_RIGHT_CURRENT );
        motor_l_adc_value = rt_adc_read ( adc_dev , ADC_DEV_MOTOR_LEFT_CURRENT );

        // Apply sliding filter to smooth ADC values
        motor_r_adc_value = sliding_filter_add_sample( &motor_r_filter , 20 ,motor_r_adc_value);
        motor_l_adc_value = sliding_filter_add_sample( &motor_l_filter , 20 ,motor_l_adc_value);

        // Convert ADC → motor current (with calibration offsets)
        motor_r_current = motor_current_output(motor_r_adc_value-119);
        motor_l_current = motor_current_output(motor_l_adc_value-114);

        /** INA226 power data acquisition **/
        current = INA226_GetCurrent();  // Current
        voltage = INA226_GetBusV();     // Voltage
        power = INA226_GetPower();      // Power

        /** Apply limit + sliding filter depending on system state **/
        if(filter_time == 50)  // After ~5s runtime
        {
            current = sliding_filter_add_sample ( &c_filter , C_FILTER_SIZE , current );
            if(voltage >= 4.0f && voltage <= 13.0f)
                true_voltage = sliding_filter_add_sample ( &v_filter , V_FILTER_SIZE , 
                                    limit_filter(voltage,robot_state.voltage,0.6f) );
            power = sliding_filter_add_sample ( &p_filter , P_FILTER_SIZE , power );
        }
        else // During startup → only sliding filter
        {
            current = sliding_filter_add_sample ( &c_filter , C_FILTER_SIZE , current );
            if(voltage >= 4.0f && voltage <= 13.0f)
                true_voltage = sliding_filter_add_sample ( &v_filter , V_FILTER_SIZE , voltage );
            power = sliding_filter_add_sample ( &p_filter , P_FILTER_SIZE , power );
        }

        /** Wheel RPM acquisition **/
        value_fr = sliding_filter_add_sample ( &rpm_r_filter , 10 , rpm_fr );
        value_fl = sliding_filter_add_sample ( &rpm_l_filter , 10 , rpm_fl );

        // Protect shared robot_state with mutex
        rt_mutex_take(state_data_mutex, RT_WAITING_FOREVER);

        robot_state.current = current;
        robot_state.voltage = true_voltage;
        robot_state.power = power;

        // Apply coefficients to RPM values for calibration
        robot_state.rpm [ 3 ] = value_fr*coeff_br;
        robot_state.rpm [ 1 ] = value_fr*coeff_fr;
        robot_state.rpm [ 2 ] = value_fl*coeff_bl;
        robot_state.rpm [ 0 ] = value_fl*coeff_fl;

        // Calculate battery percentage
        robot_state.battery = calculate_battery_percentage(robot_state.voltage);

        rt_mutex_release(state_data_mutex);

        // Loop runs every 100 ms
        rt_thread_mdelay ( 100 );
    }
}

/**
 * Calculate battery percentage based on voltage.
 * Mapping follows approximate discharge curve of Li-ion battery pack:
 * - >12.15V → 100%
 * - 11.55–12.15V → 70–100% (green)
 * - 10.65–11.55V → 30–70% (yellow)
 * - 9.6–10.65V → 0–30% (red)
 * - <9.6V → 0% (battery depleted)
 */
static uint16_t calculate_battery_percentage(float voltage) {
    if(voltage >= 12.15f)
        return 100;
    else if (voltage >= 11.55f && voltage <= 12.15f) {
        return 70 + (int)((voltage - 11.55f) / (12.15f - 11.55f) * 30);
    } else if (voltage >= 10.65f && voltage < 11.55f) {
        return 30 + (int)((voltage - 10.65f) / (11.55f - 10.65f) * 40);
    } else if (voltage >= 9.6f && voltage < 10.65f) {
        return (int)((voltage - 9.6f) / (10.65f - 9.6f) * 30);
    } else {
        return 0;
    }
}
