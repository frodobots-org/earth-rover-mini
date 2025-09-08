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

static struct rt_semaphore rx_sem;   /* Semaphore used for UART RX synchronization */
static rt_device_t serial;           /* UART device handle */

/**
 * UART RX interrupt callback.
 * Called automatically by RT-Thread when data is received.
 */
static rt_err_t pid_uart_input ( rt_device_t dev , rt_size_t size )
{
    rt_sem_release ( &rx_sem );  /* Release semaphore to unblock waiting thread */
    return RT_EOK;
}

/**
 * Thread entry function for PID UART communication.
 * This thread:
 *   - Initializes UART2
 *   - Configures UART parameters (baudrate, data bits, stop bits, buffer size, parity)
 *   - Receives data frames from UART
 *   - Parses PID parameters (P, I, D values for two controllers)
 *   - Sends back acknowledgment
 */
void pid_uart_thread_entry ( void *parameter )
{
    char buffer [ DATA_SIZE ] = {0};   /* Buffer to store incoming PID data */
    char ch;                           /* Temporary char storage for UART RX */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; 
    rt_off_t pos = 0;                  /* Position index in buffer */

    /* step1：查找串口设备 */
    /* step1: Find the UART device by name */
    serial = rt_device_find ( "uart2" );
    if ( !serial )
    {
        LOG_D( "find %s failed!\n" , PID_UART_NAME );
        return;
    }

    /* step2：修改串口配置参数 */
    /* step2: Modify UART configuration parameters */
    config.baud_rate = BAUD_RATE_115200;    // 修改波特率为 115200
                                           // Set baud rate to 115200
    config.data_bits = DATA_BITS_8;         // 数据位 8
                                           // Use 8 data bits
    config.stop_bits = STOP_BITS_1;         // 停止位 1
                                           // Use 1 stop bit
    config.bufsz = 2048;                    // 修改缓冲区 buff size 为 128
                                           // Set UART RX buffer size to 2048 bytes
    config.parity = PARITY_NONE;            // 无奇偶校验位
                                           // No parity bit

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    /* step3: Apply configuration to UART device using control command */
    rt_device_control ( serial , RT_DEVICE_CTRL_CONFIG , &config );

    /* Initialize RX semaphore in FIFO scheduling mode */
    rt_sem_init ( &rx_sem , "rx_sem" , 0 , RT_IPC_FLAG_FIFO );

    /* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备 */
    /* step4: Open UART device in read/write mode with interrupt-driven RX */
    rt_device_open ( serial , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );

    /* Set RX indication callback function */
    rt_device_set_rx_indicate ( serial , pid_uart_input );

    rt_err_t ret;
    while ( 1 )
    {
        /* Try to read one byte from UART */
        while ( rt_device_read ( serial , -1 , &ch , 1 ) != 1 )
        {
            /* If no data available, wait on RX semaphore with 500ms timeout */
            if ( (ret = rt_sem_take ( &rx_sem , rt_tick_from_millisecond ( 500 ) )) == -RT_ETIMEOUT )
            {
                // 超时没有接收到消息阻塞该线程
                // Timeout: no message received, thread remains blocked
            }
        }

        /***********PID数据接收************/
        /* PID data reception and parsing */

        if (ch == 0xFD )    /* Frame start marker detected */
        {
            pos = 0;        /* Reset buffer index */
        }
        else 
        {
            buffer [ pos++ ] = ch;   /* Store received byte in buffer */
        }

        /* When 12 bytes are collected, interpret them as PID parameters */
        if(pos == 12)
        {
            PID_uart_P = (buffer[0]<<8) + buffer[1];   /* Main PID P gain */
            PID_uart_I = (buffer[2]<<8) + buffer[3];   /* Main PID I gain */
            PID_uart_D = (buffer[4]<<8) + buffer[5];   /* Main PID D gain */

            PID_dir_P  = (buffer[6]<<8) + buffer[7];   /* Direction PID P gain */
            PID_dir_I  = (buffer[8]<<8) + buffer[9];   /* Direction PID I gain */
            PID_dir_D  = (buffer[10]<<8)+ buffer[11];  /* Direction PID D gain */

            /* Small delay to allow system to process */
            rt_thread_mdelay ( 5 );

            /* Send acknowledgment back (first 6 bytes echoed) */
            rt_device_write ( serial , 0 , buffer , 6 );
        }

        /***********PID数据接收************/
    }
}
