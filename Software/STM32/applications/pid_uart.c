/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-07     yuxing       the first version
 */
#include "main.h"
#include "pid.h"
#define DBG_TAG "UART_PID"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "state.h"

#define DATA_SIZE 100
#define PID_UART_NAME "uart2"

static struct rt_semaphore rx_sem;
static rt_device_t serial;

static rt_err_t pid_uart_input ( rt_device_t dev , rt_size_t size )
{
    rt_sem_release ( &rx_sem );
    return RT_EOK;
}

void pid_uart_thread_entry ( void *parameter )
{
    char buffer [ DATA_SIZE ] = {0};
        char ch;
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
        rt_off_t pos = 0;

        /* step1：查找串口设备 */
        serial = rt_device_find ( "uart2" );
        if ( !serial )
        {
            LOG_D( "find %s failed!\n" , PID_UART_NAME );
            return;
        }

        /* step2：修改串口配置参数 */
        config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
        config.data_bits = DATA_BITS_8;           //数据位 8
        config.stop_bits = STOP_BITS_1;           //停止位 1
        config.bufsz = 2048;                   //修改缓冲区 buff size 为 128
        config.parity = PARITY_NONE;           //无奇偶校验位

        /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
        rt_device_control ( serial , RT_DEVICE_CTRL_CONFIG , &config );

        rt_sem_init ( &rx_sem , "rx_sem" , 0 , RT_IPC_FLAG_FIFO );
        /* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备 */
        rt_device_open ( serial , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );
        rt_device_set_rx_indicate ( serial , pid_uart_input );

        rt_err_t ret;
        while ( 1 )
        {
            while ( rt_device_read ( serial , -1 , &ch , 1 ) != 1 )
            {
                if ( (ret = rt_sem_take ( &rx_sem , rt_tick_from_millisecond ( 500 ) )) == -RT_ETIMEOUT )
                {
                    //超时没有接收到消息阻塞该线程
                }
            }
            /***********PID数据接收************/
            if (ch == 0xFD )
            {
                pos=0;

            }
            else {
                buffer [ pos++ ] = ch;
            }

            if(pos == 12)
            {
                PID_uart_P = (buffer[0]<<8)+buffer[1];
                PID_uart_I = (buffer[2]<<8)+buffer[3];
                PID_uart_D = (buffer[4]<<8)+buffer[5];
                PID_dir_P = (buffer[6]<<8)+buffer[7];
                PID_dir_I = (buffer[8]<<8)+buffer[9];
                PID_dir_D = (buffer[10]<<8)+buffer[11];
                rt_thread_mdelay ( 5 );
                rt_device_write ( serial , 0 , buffer , 6 );

            }

            /***********PID数据接收************/
        }

}
