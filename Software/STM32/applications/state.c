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

#define LOW_VOLTAGE_PROTECT //低压保护 = Low-voltage protection
extern void vl53l0x_init_all ( );

rt_adc_device_t adc_dev;
#define ADC_DEV_NAME "adc1" /* ADC 设 备 名 称 */
#define ADC_DEV_MOTOR_RIGHT_CURRENT 3 /* ADC 通 道 */
#define ADC_DEV_MOTOR_LEFT_CURRENT 4 /* ADC 通 道 */
#define REFER_VOLTAGE 330 /* 参 考 电 压 3.3V,数 据 精 度 乘 以100保 留2位 小 数 */
#define CONVERT_BITS (1 << 12) /* 转 换 位 数 为12位 */
rt_mutex_t state_data_mutex;

rt_timer_t key_timer;
rt_timer_t pwr_timer;
volatile robot_state_t robot_state;

uint32_t red = 0xFF0000U;
uint32_t green = 0x00FF00U;
uint32_t blue = 0x0000FFU;

uint32_t yellow = 0xFF7F00U;
uint32_t purple = 0x8F00FFU;
uint32_t cyan = 0x00FFFFU;
uint32_t sunlight = 0xFFFF00U;
uint32_t unknow = 0xFFD700U;
uint32_t black =  0x000000U;

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

/**WS2812B**/
typedef enum {
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_RED
} Color;

//系统运行状态灯显示
void set_sys_rgb_led_color_flash (rt_uint16_t on_time, system_state_t sys_status)
{
    rt_uint32_t received_flags = 0;
    uint32_t color_buf[2] = {0};
    static Color current_color = COLOR_GREEN;
    static uint8_t charge_flag = 1;
    static uint8_t imu_calib = 0;
    /**电量指示灯状态**/
    switch (current_color) {
    case COLOR_GREEN:
    {
        color_buf[0] = green;
        if (robot_state.battery < 67.0) { // 60% - 3% 滞回
            current_color = COLOR_YELLOW;
            color_buf[0] = yellow;
        }
    }
    break;
    case COLOR_YELLOW:
    {
        color_buf[0] = yellow;
        if (robot_state.battery > 73.0) { // 60% + 3% 滞回
            current_color = COLOR_GREEN;
            color_buf[0] = green;
        } else if (robot_state.battery < 27.0) { // 20% - 3% 滞回
            current_color = COLOR_RED;
            color_buf[0] = red;
        }
    }
    break;
    case COLOR_RED:
    {
        color_buf[0] = red;
        if (robot_state.battery > 33.0) { // 20% + 3% 滞回
            current_color = COLOR_YELLOW;
            color_buf[0] = yellow;
        }
    }
    break;
    }

    if(sys_status == SYSTEM_STATE_CHARGING)//判断到充电状态电源指示灯进行慢闪
    {
        if(charge_flag)
        {

        }
        else {
            color_buf[0] = black;
        }
        charge_flag = !charge_flag;
    }
    //IMU校准事件
    if(imu_event != RT_NULL)
    {
        if (rt_event_recv(imu_event, IMU_CALIB_LED_START | IMU_CALIB_LED_DONE,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_NO, &received_flags) == RT_EOK)
        {
            if(received_flags & IMU_CALIB_LED_START)
            {
                imu_calib = 1;
            }
            if(received_flags & IMU_CALIB_LED_DONE)
            {
                imu_calib = 0;
            }

        }
    }
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
    /**联网状态判定**/
    color_buf[1] = black;
    if(robot_state.pwr == 1)
    {
        switch ( robot_state.net_led_status )
        {
        case UCP_STATE_UNKNOWN :  //默认状态
        {
            color_buf[1] = red;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time );
        }
        break;
        case UCP_STATE_SIMABSENT :  //没有SIM卡(慢闪)
        {
            color_buf[1] = red;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time);
            color_buf[1] = black;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time );
        }
        break;
        case UCP_NETWORK_DISCONNECTED :  //联网中(快闪)
        {
            color_buf[1] = green;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time * 0.3  );
            color_buf[1] = black;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time * 0.3  );
        }
        break;
        case UCP_NETWORK_CONNECTED : //联网结束(绿色常亮)
        {
            color_buf[1] = green;
            ws2812b_write_test(2,color_buf);
            rt_thread_delay( on_time );
        }
        break;
        case UCP_OTA_ING :   //OTA升级(蓝色快闪)
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
        ws2812b_write_test(2,color_buf);
        rt_thread_delay( on_time );
    }

}
void LED_system_init(rt_uint16_t on_time,float black_coeff)
{
    LOG_E("LED_SYSTEM_INIT!!!");
    uint32_t color_buf[2] = {0};
    if(robot_state.pwr == 0)
    {
    color_buf[0] = red;
    color_buf[1] = red;
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

rt_timer_t key_timer;
volatile rt_bool_t key_pressed = RT_FALSE;
volatile rt_tick_t key_press_start_time = 0;
volatile rt_tick_t key_press_stop_time = 0;

//#define SW_MODE
#ifndef SW_MODE
void key_isr ( void *args )  //要注意开机的时候是没办法检测到按键的变化的，因为那会按键才刚按下，系统才刚开始初始化
{
    int value = rt_pin_read ( PWR_DEC );

    if ( value == 0 ) /* 按键按下（低电平） */
    {
        if ( !key_pressed )
        {
            key_pressed = RT_TRUE;
            key_press_start_time = rt_tick_get_millisecond ( );
            rt_timer_start ( key_timer );
//            rt_kprintf("0\n");
        }
    }
    else /* 按键松开（高电平） */
    {
        if ( key_pressed )
        {
            key_pressed = RT_FALSE;
            key_press_stop_time = rt_tick_get_millisecond ( );
            rt_timer_stop ( key_timer );  //预防按键达不到要求的时间，需要关掉定时器
//            rt_kprintf("1\n");
        }
    }
}
#else
void key_isr ( void *args )  //要注意开机的时候是没办法检测到按键的变化的，因为那会按键才刚按下，系统才刚开始初始化
{
    rt_thread_mdelay ( 20 );//延时消抖
    int value = rt_pin_read ( PWR_DEC );

    if ( value == 0 ) /* 按键按下（低电平） */
    {
        if(robot_state.pwr == 0)
        {
            robot_state.pwr = 1;
            robot_power_on ( );
            robot_power_out_on ( );
        }
    }
    else /* 按键松开（高电平） */
    {
        if(robot_state.pwr == 1)
        {
            robot_state.pwr = 0;
            robot_power_off ( );
        }
    }

}
#endif

/* 按键检测任务 */
#ifndef SW_MODE
void key_detect_task ( void )
{
    rt_uint32_t timer_time = KEY_LONG_PRESS_TIME - SYS_START_UP_TIME;

    if ( key_pressed == RT_TRUE )
    {
        rt_tick_t duration = key_press_stop_time - key_press_start_time;

        if ( key_press_stop_time < key_press_start_time )  //按键还未松开
        {
            return;
        }
        //达到开关机时间
        if ( duration > 0 )
        {
            if ( robot_state.pwr == 0 && duration >= timer_time )  //开机时因无法识别边沿，故只能查询，需减去系统启动到查询的时间
            {
#ifdef LOW_VOLTAGE_PROTECT
                //低压保护
                if(robot_state.voltage <= 9.30f)
                {
                    duration = 0;
                    key_press_stop_time = 0;
                    robot_state.pwr = 0;
                    robot_state.status = SYSTEM_STATE_SHUTDOWN;
                    ws2812_clearn_all(2);//关机前关闭系统状态灯
                    robot_power_off ( );
                    robot_power_out_off ( );
                    return;
                }
#endif
                duration = 0;
                key_press_stop_time = 0;
                robot_state.pwr = 1;
                robot_state.status = SYSTEM_STATE_RUNNING;
                robot_power_on ( );
                robot_power_out_on ( );
                LOG_I( "PWR ON" );
            }
            else if ( robot_state.pwr == 1 && duration >= KEY_LONG_PRESS_TIME )
            {
                duration = 0;
                key_press_stop_time = 0;
                robot_state.pwr = 0;
                robot_state.status = SYSTEM_STATE_SHUTDOWN;
                ws2812_clearn_all(2);//关机前关闭系统状态灯
                robot_power_off ( );
                robot_power_out_off ( );
                LOG_I( "PWR OFF" );
            }
        }
    }
    else  //主要是考虑开机时没有办法识别按键的边沿,以及开机后插入
    {
        if ( key_pressed != RT_TRUE ) //开机时一瞬间无法进入按键中断，从这里进
        {
            if ( rt_pin_read ( PWR_DEC ) == PIN_LOW )
            {
                if ( robot_state.pwr == 0 )  //如果是开机，则修改定时器时间
                {
                    key_pressed = RT_TRUE;
                    key_press_start_time = rt_tick_get_millisecond ( );
                    rt_timer_control ( key_timer , RT_TIMER_CTRL_SET_TIME , &timer_time );
                    rt_timer_start ( key_timer );
                }
            }
        }
    }
}
#else
void key_detect_task ( void )
{
    if ( rt_pin_read ( PWR_DEC ) == PIN_LOW )
    {
        if(robot_state.pwr == 0)
        {
            robot_state.pwr = 1;
            robot_power_on ( );
            robot_power_out_on ( );
            LOG_I( "PWR ON" );
        }
    }
}
#endif

static void key_timer_timeout ( void *parameter )
{
    rt_uint32_t timer_time = KEY_LONG_PRESS_TIME;
    int val = rt_pin_read ( PWR_DEC );
    if ( val == PIN_LOW )
    {
        //			LOG_I( "Is long press !" );
        key_pressed = RT_TRUE;
        key_press_stop_time = rt_tick_get_millisecond ( );
    }
    rt_timer_control ( key_timer , RT_TIMER_CTRL_SET_TIME , &timer_time );  //因开机按键识别改动，故每次都恢复定时时间
}

//充电口检测任务初始化
void robot_charge_init( void )
{
    rt_pin_mode ( CHARGE_DET , PIN_MODE_INPUT_PULLDOWN );
    rt_pin_mode ( CHG_INT , PIN_MODE_INPUT_PULLDOWN );
}

void robot_charge_task( void )
{
    if ( (rt_pin_read ( CHARGE_DET ) == PIN_HIGH) ||  (rt_pin_read ( CHG_INT ) == PIN_HIGH) )
    { // checks if robot is plugged in from the CHARGE_DET pin, if true:
        if(robot_state.voltage >= 12.15f) // if we reach more that 121.5 Volts(?) consider fully charged
        {
            robot_state.status = SYSTEM_STATE_CHARGED;
        }
        else { // else, we're still charging
            robot_state.status = SYSTEM_STATE_CHARGING;
        }
        if(robot_state.pwr == 0)
        { // charging but not powered on:
            LOG_I( "PWR OFF -1" );
            robot_power_on ( );
            robot_power_out_off ( );//未开机状态下充电关闭头部供电 = When not powered on, charging disables head power supply.
        }

    }
    else {
#ifdef LOW_VOLTAGE_PROTECT
        //低压保护
        if(robot_state.voltage <= 9.30f)
        {
            LOG_E("voltage : %.2f",robot_state.voltage);
            LOG_E("OFF-2XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX%x",robot_state.voltage);
            LOG_E( "PWR OFF-2" );
            robot_state.status = SYSTEM_STATE_SHUTDOWN;//SYSTEM_STATE_INITIAL
            ws2812_clearn_all(2);//关机前关闭系统状态灯
            robot_power_off ( );
        }
#endif
        if(robot_state.pwr == 1)
        {
            if(robot_state.status == SYSTEM_STATE_WARNING)
            {
                if(robot_state.battery <= 13)//%3滞回
                {
                    robot_state.status = SYSTEM_STATE_WARNING;
                }
                else
                robot_state.status = SYSTEM_STATE_RUNNING;
            }
            else {
                if(robot_state.battery <= 10)//%3滞回
                {
                    robot_state.status = SYSTEM_STATE_WARNING;
                }
                else
                robot_state.status = SYSTEM_STATE_RUNNING;
            }

        }
        else {
//            robot_state.status = SYSTEM_STATE_INITIAL;
            LOG_I( "PWR OFF-3 pwr=%d", robot_state.pwr);
            ws2812_clearn_all(2);//关机前关闭系统状态灯
            robot_power_off ( );//若未检测到type-c插入并且未启动按键开机，则关闭MCU电源输出

            if(robot_state.pwr == 1)
            {
                LOG_I( "PWR ON-1" );
                robot_power_on ( );
                robot_power_out_on ( );

            }
        }

    }
}

void robot_power_init ( void )
{
    rt_pin_mode ( PWR_ON , PIN_MODE_OUTPUT );
    rt_pin_mode ( PWR_CLK , PIN_MODE_OUTPUT );
    rt_pin_mode ( PWR_OUT_ON , PIN_MODE_OUTPUT );
    rt_pin_mode ( PWR_DEC , PIN_MODE_INPUT_PULLUP );

    rt_pin_attach_irq ( PWR_DEC , PIN_IRQ_MODE_RISING_FALLING , key_isr , RT_NULL );
    rt_pin_irq_enable ( PWR_DEC , PIN_IRQ_ENABLE );

    //给一个默认电平
    rt_pin_write ( PWR_ON , PIN_HIGH );//默认开机
    rt_pin_write ( PWR_CLK , PIN_LOW );
    rt_pin_write ( PWR_OUT_ON , PIN_LOW );//默认低（打开）
}

void robot_lamp_init ( void )
{
    rt_pin_mode ( LED_CAR1 , PIN_MODE_OUTPUT );
    rt_pin_mode ( LED_CAR2 , PIN_MODE_OUTPUT );
    rt_pin_mode ( LED_CAR3 , PIN_MODE_OUTPUT );
    rt_pin_mode ( LED_CAR4 , PIN_MODE_OUTPUT );
}


void robot_lamp_on ( void )
{
    rt_pin_write ( LED_CAR1 , PIN_HIGH );
    rt_pin_write ( LED_CAR2 , PIN_HIGH );
    rt_pin_write ( LED_CAR3 , PIN_HIGH );
    rt_pin_write ( LED_CAR4 , PIN_HIGH );
}

void robot_lamp_off ( void )
{
    rt_pin_write ( LED_CAR1 , PIN_LOW );
    rt_pin_write ( LED_CAR2 , PIN_LOW );
    rt_pin_write ( LED_CAR3 , PIN_LOW );
    rt_pin_write ( LED_CAR4 , PIN_LOW );
}

//rt_uint8_t pwr_on_flag = 0;
//static void pwr_timer_timeout ( void *parameter )
//{
//	  pwr_on_flag = 1;
//}

#define ADC_SAMPLES 100  // ADC采样数量
#define ADC_CHANNEL ADC_CHANNEL_0  // 假设使用ADC1的通道0

ADC_HandleTypeDef hadc1;
rt_uint32_t adc_values [ ADC_SAMPLES ];
rt_uint32_t adc_filtered_value;

rt_uint32_t MedianFilter ( rt_uint32_t *values , rt_uint32_t size )
{
    rt_uint32_t temp;
    for ( rt_uint32_t i = 0 ; i < size / 2 ; i++ )
    {
        for ( rt_uint32_t j = i + 1 ; j < size ; j++ )
        {
            if ( values [ j ] < values [ i ] )
            {
                temp = values [ i ];
                values [ i ] = values [ j ];
                values [ j ] = temp;
            }
        }
    }
    return values [ size / 2 ];
}


// 初始化滑动滤波器
void sliding_filter_init ( SlidingFilter *filter , rt_uint16_t size )
{
    for ( int i = 0 ; i < size ; i++ )
    {
        filter->buffer [ i ] = 0.0;
    }
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0;
}

// 添加新样本到滑动滤波器并返回滤波后的值
float sliding_filter_add_sample ( SlidingFilter *filter , rt_int32_t size , float new_sample )
{
    // 从总和中减去即将被替换的旧样本的值
    if ( filter->count == size )
    {
        filter->sum -= filter->buffer [ filter->index ];
    }
    else
    {
        filter->count++;  // 增加缓冲区中的样本数量
    }

    // 添加新样本到缓冲区并更新其总和
    filter->buffer [ filter->index ] = new_sample;
    filter->sum += new_sample;

    // 更新索引，如果需要则回绕到开始
    filter->index = (filter->index + 1) % size;

    // 计算并返回平均值
    return filter->sum / filter->count;
}

/**限幅滤波接口**/
float limit_filter(float new_value, float previous_value, float threshold) {
    if (fabs(new_value - previous_value) > threshold) {
        return previous_value;
    }
    return new_value;
}

#define GAIN 50 //TP181模块放大器增益
/**
 * TP181电机电流转化
 */
static float motor_current_output(rt_uint32_t voltage_out)
{
    float Rshunt = 0.015f;//分流电阻阻值15mΩ
    float current = 0.0f;
    current = (((((float)voltage_out / (4096 - 1)) * 3.30f) - 3.30f/2.0f)/(GAIN * Rshunt));

    return current;
}


#define THREAD_PRIORITY      9
#define THREAD_TIMESLICE     5

#define EVENT_FLAG3 (1 << 3)
#define EVENT_FLAG5 (1 << 5)

extern rt_int32_t rpm_fl;
extern rt_int32_t rpm_fr;
extern rt_int32_t rpm_bl;
extern rt_int32_t rpm_br;

extern rt_int32_t sum_t_fl;
extern rt_int32_t sum_t_fr;

static uint16_t calculate_battery_percentage(float voltage);

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

    robot_lamp_init ( );
    INA226_init();//电源数据采集外设初始化

    SlidingFilter v_filter , c_filter , p_filter , motor_l_filter,motor_r_filter,rpm_l_filter,rpm_r_filter;
    sliding_filter_init ( &v_filter , V_FILTER_SIZE );  // 初始化滑动滤波器
    sliding_filter_init ( &c_filter , C_FILTER_SIZE );  // 初始化滑动滤波器
    sliding_filter_init ( &p_filter , P_FILTER_SIZE );  // 初始化滑动滤波器
    sliding_filter_init ( &rpm_l_filter , C_FILTER_SIZE );  // 初始化滑动滤波器
    sliding_filter_init ( &rpm_r_filter , C_FILTER_SIZE );  // 初始化滑动滤波器
    sliding_filter_init ( &motor_l_filter , C_FILTER_SIZE );  // 初始化滑动滤波器
    sliding_filter_init ( &motor_r_filter , C_FILTER_SIZE );  // 初始化滑动滤波器

    key_timer = rt_timer_create ( "key_timer" , key_timer_timeout , RT_NULL , KEY_LONG_PRESS_TIME ,
            RT_TIMER_FLAG_ONE_SHOT );

    adc_dev = (rt_adc_device_t) rt_device_find ( ADC_DEV_NAME );
    if ( adc_dev == RT_NULL )
    {
        LOG_E( "adc sample run failed! can't find %s device!\n" , ADC_DEV_NAME );
        return RT_ERROR;
    }

    rt_adc_enable ( adc_dev , ADC_DEV_MOTOR_RIGHT_CURRENT );
    rt_adc_enable ( adc_dev , ADC_DEV_MOTOR_LEFT_CURRENT );

    bzero(&path,sizeof(&path));
    uint8_t filter_time =0;
//    robot_state.lamp =100;//FIXME
    // 创建state_data互斥锁
    state_data_mutex = rt_mutex_create("state_data_mutex", RT_IPC_FLAG_PRIO);
    if (state_data_mutex == RT_NULL)
    {
        LOG_E("state_data_mutex creation failed!\n");
        return -RT_ERROR;
    }
    while ( 1 )
    {
        if(filter_time < 50)
        filter_time ++;

        if ( robot_state.lamp && robot_state.lamp != 0xff && robot_state.pwr == 1 )  //在开机上总电的时候才给开灯
        {
            robot_lamp_on ( );
        }
        else
        {
            robot_lamp_off ( );
        }

        key_detect_task ( );
//        robot_lamp_on ( );

        /**TP181电机电流采集**/
        motor_r_adc_value = rt_adc_read ( adc_dev , ADC_DEV_MOTOR_RIGHT_CURRENT );
        motor_l_adc_value = rt_adc_read ( adc_dev , ADC_DEV_MOTOR_LEFT_CURRENT );
        motor_r_adc_value = sliding_filter_add_sample( &motor_r_filter , 20 ,motor_r_adc_value);
        motor_l_adc_value = sliding_filter_add_sample( &motor_l_filter , 20 ,motor_l_adc_value);
        motor_r_current = motor_current_output(motor_r_adc_value-119);
        motor_l_current = motor_current_output(motor_l_adc_value-114);
//        rt_kprintf("motor_r_adc_value = %d , motor_l_adc_value = %d\n",motor_r_adc_value-119,motor_l_adc_value-114);
//        rt_kprintf(" %f , %f\n",motor_r_current,motor_l_current);
//        LOG_I("motor_r_current = %d , motor_l_current = %d\n",motor_r_adc_value,motor_l_adc_value);
        /**TP181电机电流采集**/

        /**INA电源数据采集**/
        //电流采集
        current = INA226_GetCurrent();
        //电压采集
        voltage = INA226_GetBusV();
        //功率采集
        power = INA226_GetPower();

        /**限幅滤波**/
        if(filter_time == 50)//系统开机运行状态下开启滑动+限幅滤波 robot_state.status == SYSTEM_STATE_RUNNING
        {
            current = sliding_filter_add_sample ( &c_filter , C_FILTER_SIZE , current );
            if(voltage >= 4.0f && voltage <= 13.0f)
            true_voltage = sliding_filter_add_sample ( &v_filter , V_FILTER_SIZE , limit_filter(voltage,robot_state.voltage,0.6f) );  //获取实时电压值limit_filter(voltage,robot_state.voltage,0.6f);
            power = sliding_filter_add_sample ( &p_filter , P_FILTER_SIZE , power );
        }
        else {//其他状态下只采用滑动滤波
            /**开机的时间(5S)里进行滑动滤波实时更新电源数据**/
            current = sliding_filter_add_sample ( &c_filter , C_FILTER_SIZE , current );  // 应用滑动滤波
            if(voltage >= 4.0f && voltage <= 13.0f)
            true_voltage = sliding_filter_add_sample ( &v_filter , V_FILTER_SIZE , voltage );  // 应用滑动滤波
            power = sliding_filter_add_sample ( &p_filter , P_FILTER_SIZE , power );  // 应用滑动滤波
        }
//        rt_kprintf("current = %f , voltage = %f , robot_state.voltage = %f\n",current,voltage,robot_state.voltage);
        /**INA电源数据采集**/
//        rt_kprintf("%f , %f\n",voltage,robot_state.voltage);

        /**rpm数据上传**/
        value_fr = sliding_filter_add_sample ( &rpm_r_filter , 10 , rpm_fr );  // 应用滑动滤波
        value_fl = sliding_filter_add_sample ( &rpm_l_filter , 10 , rpm_fl );  // 应用滑动滤波

        // 申请互斥锁(关联数据需保证数据同步性)
        rt_mutex_take(state_data_mutex, RT_WAITING_FOREVER);

        robot_state.current = current;
        robot_state.voltage = true_voltage;
        robot_state.power = power;
        robot_state.rpm [ 3 ] = value_fr*coeff_br;
        robot_state.rpm [ 1 ] = value_fr*coeff_fr;
        robot_state.rpm [ 2 ] = value_fl*coeff_bl;
        robot_state.rpm [ 0 ] = value_fl*coeff_fl;
        /**系统电量计算**/
        robot_state.battery = calculate_battery_percentage(robot_state.voltage);

        // 释放互斥锁
        rt_mutex_release(state_data_mutex);

        rt_thread_mdelay ( 100 );
    }

}

// 根据电压计算电量百分比(根据电池电压-容量放电曲线修改)
static uint16_t calculate_battery_percentage(float voltage) {
    if(voltage >= 12.15f)
        return 100;
    else if (voltage >= 11.55f && voltage <= 12.15f) { // 绿灯 (70% ~ 100%)
        return 70 + (int)((voltage - 11.55f) / (12.15f - 11.55f) * 30);
    } else if (voltage >= 10.65f && voltage < 11.55f) { // 黄灯 (30% ~ 70%)
        return 30 + (int)((voltage - 10.65f) / (11.55f - 10.65f) * 40);
    } else if (voltage >= 9.6f && voltage < 10.65f) { // 红灯 (0% ~ 30%)
        return (int)((voltage - 9.6f) / (10.65f - 9.6f) * 30);
    } else { // 低于9.6V，认为电量耗尽
        return 0;
    }
}
