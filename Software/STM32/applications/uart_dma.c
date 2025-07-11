///*
// * Copyright (c) 2006-2021, RT-Thread Development Team
// *
// * SPDX-License-Identifier: Apache-2.0
// *
// * Change Logs:
// * Date           Author       Notes
// * 2024-11-13     yuxing       the first version
// */
//#include <main.h>
//
//#define DBG_TAG "uart_dma"
//#define DBG_LVL DBG_LOG
//#include <rtdbg.h>
//#include "drivers/serial.h"
//#include "ucp.h"
//#include "main.h"
//
//#define SAMPLE_UART_NAME       "uart3"      /* 串口设备名称 */
//struct serial_configure config_dma = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
//struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
//char rx_buffer[RT_SERIAL_RB_BUFSZ + 1];
//char msg_pool[1024*8]; /* 消息缓冲区 */
//
///* 环形缓冲区 */
//#define RING_BUFFER_LEN        1024
//
//static struct rt_ringbuffer *rb;
//static struct rt_semaphore rx_sem;
//rt_timer_t ucp_timer;  //ACK超时定时器
//
///* 串口接收消息结构*/
//struct rx_msg
//{
//    rt_device_t dev;
//    rt_size_t size;
//};
///* 串口设备句柄 */
//static rt_device_t serial;
///* 消息队列控制块 */
//static struct rt_messagequeue rx_mq;
//
//typedef struct uart_message
//{
//    u_int8_t get_data_flag;
//    u_int8_t imu_set_flag;
//    u_int8_t mag_set_flag;
//}uart_flag;
//static uart_flag ucp_flag;
///* 接收数据回调函数DMA */
//static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
//{
//    struct rx_msg msg;
//    rt_err_t result;
//    msg.dev = dev;
//    msg.size = size;
//
//    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
//    if ( result == -RT_EFULL)
//    {
//        /* 消息队列满 */
//        rt_kprintf("message queue full！\n");
//    }
//    return result;
//}
///* 接收数据回调函数INT */
//static rt_err_t uart_int_input ( rt_device_t dev , rt_size_t size )
//{
//    rt_sem_release ( &rx_sem );
//    return RT_EOK;
//}
//
////CRC16校验
//static uint8_t crc_hi_table [ ] = { 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 ,
//        0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 ,
//        0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 ,
//        0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 ,
//        0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 ,
//        0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 ,
//        0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 ,
//        0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 ,
//        0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 ,
//        0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 ,
//        0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 ,
//        0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 ,
//        0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 ,
//        0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 ,
//        0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 ,
//        0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 ,
//        0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 ,
//        0x00 , 0xC1 , 0x81 , 0x40 };
//
//static uint8_t crc_lo_table [ ] = { 0x00 , 0xC0 , 0xC1 , 0x01 , 0xC3 , 0x03 , 0x02 , 0xC2 , 0xC6 , 0x06 , 0x07 , 0xC7 ,
//        0x05 , 0xC5 , 0xC4 , 0x04 , 0xCC , 0x0C , 0x0D , 0xCD , 0x0F , 0xCF , 0xCE , 0x0E , 0x0A , 0xCA , 0xCB ,
//        0x0B , 0xC9 , 0x09 , 0x08 , 0xC8 , 0xD8 , 0x18 , 0x19 , 0xD9 , 0x1B , 0xDB , 0xDA , 0x1A , 0x1E , 0xDE ,
//        0xDF , 0x1F , 0xDD , 0x1D , 0x1C , 0xDC , 0x14 , 0xD4 , 0xD5 , 0x15 , 0xD7 , 0x17 , 0x16 , 0xD6 , 0xD2 ,
//        0x12 , 0x13 , 0xD3 , 0x11 , 0xD1 , 0xD0 , 0x10 , 0xF0 , 0x30 , 0x31 , 0xF1 , 0x33 , 0xF3 , 0xF2 , 0x32 ,
//        0x36 , 0xF6 , 0xF7 , 0x37 , 0xF5 , 0x35 , 0x34 , 0xF4 , 0x3C , 0xFC , 0xFD , 0x3D , 0xFF , 0x3F , 0x3E ,
//        0xFE , 0xFA , 0x3A , 0x3B , 0xFB , 0x39 , 0xF9 , 0xF8 , 0x38 , 0x28 , 0xE8 , 0xE9 , 0x29 , 0xEB , 0x2B ,
//        0x2A , 0xEA , 0xEE , 0x2E , 0x2F , 0xEF , 0x2D , 0xED , 0xEC , 0x2C , 0xE4 , 0x24 , 0x25 , 0xE5 , 0x27 ,
//        0xE7 , 0xE6 , 0x26 , 0x22 , 0xE2 , 0xE3 , 0x23 , 0xE1 , 0x21 , 0x20 , 0xE0 , 0xA0 , 0x60 , 0x61 , 0xA1 ,
//        0x63 , 0xA3 , 0xA2 , 0x62 , 0x66 , 0xA6 , 0xA7 , 0x67 , 0xA5 , 0x65 , 0x64 , 0xA4 , 0x6C , 0xAC , 0xAD ,
//        0x6D , 0xAF , 0x6F , 0x6E , 0xAE , 0xAA , 0x6A , 0x6B , 0xAB , 0x69 , 0xA9 , 0xA8 , 0x68 , 0x78 , 0xB8 ,
//        0xB9 , 0x79 , 0xBB , 0x7B , 0x7A , 0xBA , 0xBE , 0x7E , 0x7F , 0xBF , 0x7D , 0xBD , 0xBC , 0x7C , 0xB4 ,
//        0x74 , 0x75 , 0xB5 , 0x77 , 0xB7 , 0xB6 , 0x76 , 0x72 , 0xB2 , 0xB3 , 0x73 , 0xB1 , 0x71 , 0x70 , 0xB0 ,
//        0x50 , 0x90 , 0x91 , 0x51 , 0x93 , 0x53 , 0x52 , 0x92 , 0x96 , 0x56 , 0x57 , 0x97 , 0x55 , 0x95 , 0x94 ,
//        0x54 , 0x9C , 0x5C , 0x5D , 0x9D , 0x5F , 0x9F , 0x9E , 0x5E , 0x5A , 0x9A , 0x9B , 0x5B , 0x99 , 0x59 ,
//        0x58 , 0x98 , 0x88 , 0x48 , 0x49 , 0x89 , 0x4B , 0x8B , 0x8A , 0x4A , 0x4E , 0x8E , 0x8F , 0x4F , 0x8D ,
//        0x4D , 0x4C , 0x8C , 0x44 , 0x84 , 0x85 , 0x45 , 0x87 , 0x47 , 0x46 , 0x86 , 0x82 , 0x42 , 0x43 , 0x83 ,
//        0x41 , 0x81 , 0x80 , 0x40 };
//
//static uint16_t crc16 ( char* msg , size_t len )
//{
//    uint8_t crc_hi = 0xFF;
//    uint8_t crc_lo = 0xFF;
//    uint8_t index;
//
//    while ( len-- )
//    {
//        index = crc_lo ^ *msg++;
//        crc_lo = crc_hi ^ crc_hi_table [ index ];
//        crc_hi = crc_lo_table [ index ];
//    }
//    return (crc_hi << 8 | crc_lo);
//}
//
////information report(数据包ID:0x05)
//static void uart_report_state (void)
//{
//    uint16_t version = APP_VERSION;
//    ucp_hd_t hd;
//    hd.len = 0x28;
//    hd.id = 0x05;
//    static uint8_t index = 0;
//    uint16_t crc = 0;
//    uint8_t data [ 44 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = index++;
//    data [ 6 ] = robot_state.battery & 0xff;
//    data [ 7 ] = robot_state.battery >> 8;
//    data [ 8 ] = robot_state.rpm [ 0 ] & 0xff;
//    data [ 9 ] = robot_state.rpm [ 0 ] >> 8;
//    data [ 10 ] = robot_state.rpm [ 1 ] & 0xff;
//    data [ 11 ] = robot_state.rpm [ 1 ] >> 8;
//    data [ 12 ] = robot_state.rpm [ 2 ] & 0xff;
//    data [ 13 ] = robot_state.rpm [ 2 ] >> 8;
//    data [ 14 ] = robot_state.rpm [ 3 ] & 0xff;
//    data [ 15 ] = robot_state.rpm [ 3 ] >> 8;
//    data [ 16 ] = IMU_updata.acc_x & 0xff;
//    data [ 17 ] = IMU_updata.acc_x >> 8;
//    data [ 18 ] = IMU_updata.acc_y & 0xff;
//    data [ 19 ] = IMU_updata.acc_y >> 8;
//    data [ 20 ] = IMU_updata.acc_z & 0xff;
//    data [ 21 ] = IMU_updata.acc_z >> 8;
//    data [ 22 ] = IMU_updata.gyro_x & 0xff;
//    data [ 23 ] = IMU_updata.gyro_x >> 8;
//    data [ 24 ] = IMU_updata.gyro_y & 0xff;
//    data [ 25 ] = IMU_updata.gyro_y >> 8;
//    data [ 26 ] = IMU_updata.gyro_z & 0xff;
//    data [ 27 ] = IMU_updata.gyro_z >> 8;
//    data [ 28 ] = IMU_updata.Mag_data[0] & 0xff;
//    data [ 29 ] = IMU_updata.Mag_data[0] >> 8;
//    data [ 30 ] = IMU_updata.Mag_data[1] & 0xff;
//    data [ 31 ] = IMU_updata.Mag_data[1] >> 8;
//    data [ 32 ] = IMU_updata.Mag_data[2] & 0xff;
//    data [ 33 ] = IMU_updata.Mag_data[2] >> 8;
//    data [ 34 ] = IMU_updata.heading & 0xff;
//    data [ 35 ] = IMU_updata.heading >> 8;
//    data [ 36 ] = 0;
//    data [ 37 ] = 0;
//    data [ 38 ] = 0;
//    data [ 39 ] = 0;
//    data [ 40 ] = version & 0xff;
//    data [ 41 ] = version >> 8;
//    crc = crc16 ( data , 42 );
//    data [ 42 ] = crc & 0xff;
//    data [ 43 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
////陀螺仪参数校准写入(数据包ID:0x06)
//static void IMU_PERS_SET(void)
//{
//    ucp_hd_t hd;
//    hd.len = 0x10;
//    hd.id = 0x06;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 20 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    data [ 6 ] = IMU_updata.acc_bias[0] & 0xff;
//    data [ 7 ] = IMU_updata.acc_bias[0] >> 8;
//    data [ 8 ] = IMU_updata.acc_bias[1] & 0xff;
//    data [ 9 ] = IMU_updata.acc_bias[1] >> 8;
//    data [ 10 ] = IMU_updata.acc_bias[2] & 0xff;
//    data [ 11 ] = IMU_updata.acc_bias[2] >> 8;
//    data [ 12 ] = IMU_updata.gyro_bias[0] & 0xff;
//    data [ 13 ] = IMU_updata.gyro_bias[0] >> 8;
//    data [ 14 ] = IMU_updata.gyro_bias[1] & 0xff;
//    data [ 15 ] = IMU_updata.gyro_bias[1] >> 8;
//    data [ 16 ] = IMU_updata.gyro_bias[2] & 0xff;
//    data [ 17 ] = IMU_updata.gyro_bias[2] >> 8;
//    crc = crc16 ( data , 18 );
//    data [ 18 ] = crc & 0xff;
//    data [ 19 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
////磁力计参数校准写入(数据包ID:0x07)
//static void MAG_PERS_SET(void)
//{
//    ucp_hd_t hd;
//    hd.len = 0x0A;
//    hd.id = 0x07;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 14 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    data [ 6 ] = IMU_updata.mag_bias[0] & 0xff;
//    data [ 7 ] = IMU_updata.mag_bias[0] >> 8;
//    data [ 8 ] = IMU_updata.mag_bias[1] & 0xff;
//    data [ 9 ] = IMU_updata.mag_bias[1] >> 8;
//    data [ 10 ] = IMU_updata.mag_bias[2] & 0xff;
//    data [ 11 ] = IMU_updata.mag_bias[2] >> 8;
//    crc = crc16 ( data , 12 );
//    data [ 12 ] = crc & 0xff;
//    data [ 13 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//
//}
//
////(陀螺仪+磁力计)参数校准获取(数据包ID:0x08)
//static void IMU_PERS_GET(void)
//{
//    ucp_hd_t hd;
//    hd.len = 0x04;
//    hd.id = 0x08;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 8 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    crc = crc16 ( data , 6 );
//    data [ 6 ] = crc & 0xff;
//    data [ 7 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
////回应的系统状态(数据包ID:0x01)
//static void Keep_alive_ACK(uint8_t err)
//{
//    ucp_hd_t hd;
//    hd.len = 0x05;
//    hd.id = 0x01;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 9 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    data [ 6 ] = err;
//    crc = crc16 ( data , 7 );
//    data [ 7 ] = crc & 0xff;
//    data [ 8 ] = crc >> 8;
//
////    rt_kprintf("data[7] = %x , data[8] = %x\n",data[7],data[8]);
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
////回应获取(陀螺仪/磁力计)校准开始通知的状态(数据包ID:0x03)
//static void IMU_Correct_Start_ACK(uint8_t type,uint8_t err)
//{
//    ucp_hd_t hd;
//    hd.len = 0x06;
//    hd.id = 0x03;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 10 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    data [ 6 ] = type;
//    data [ 7 ] = err;
//    crc = crc16 ( data , 8 );
//    data [ 8 ] = crc & 0xff;
//    data [ 9 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
////回应获取(陀螺仪/磁力计)校准结束通知的状态(数据包ID:0x04)
//static void IMU_Correct_End_ACK(uint8_t type,uint8_t err)
//{
//    ucp_hd_t hd;
//    hd.len = 0x06;
//    hd.id = 0x04;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 10 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff ;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    data [ 6 ] = type;
//    data [ 7 ] = err;
//    crc = crc16 ( data , 8 );
//    data [ 8 ] = crc & 0xff;
//    data [ 9 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
////回应OTA升级的状态(数据包ID:0x09)
//static void OTA_Start_ACK(uint8_t err)
//{
//    ucp_hd_t hd;
//    hd.len = 0x05;
//    hd.id = 0x09;
//    hd.index = 0x00;
//    uint16_t crc = 0;
//    uint8_t data [ 10 ] = {0};
//    data [ 0 ] = 0xfd;
//    data [ 1 ] = 0xff ;
//    data [ 2 ] = hd.len & 0xff;
//    data [ 3 ] = hd.len >> 8;
//    data [ 4 ] = hd.id;
//    data [ 5 ] = hd.index;
//    data [ 6 ] = err;
//    crc = crc16 ( data , 7 );
//    data [ 7 ] = crc & 0xff;
//    data [ 8 ] = crc >> 8;
//
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_HIGH );
////    rt_thread_mdelay ( 5 );
//    rt_device_write ( serial , 0 , data , sizeof(data) );
////    rt_pin_write ( RS485_TR_EN_PIN , PIN_LOW );
//}
//
//
//void serial_thread_entry(void *parameter)
//{
//    struct rx_msg msg;
//    rt_err_t result;
//    rt_uint32_t rx_length;
//    int16_t ota_version = 0;
//    uint16_t crc;
//    uint8_t ring_buffer[64] = {0};
//    rt_int32_t ring_length = 0;
//    uint8_t handle_id = 0;
//    int8_t handle_len = 0;
//    uint8_t ring_buffer_p = 0;//初始化数组指针
//    uint8_t calib_mode = 0;
//    int ota = 0;
//    int get_init = 0;
//    uart_dma_sample();//配置初始化
//    uart_timout_init();//定时器初始化
//
//    while (1)
//    {
//        if(!get_init)//刚开机，首次向头部拿取数据
//        {
//            IMU_PERS_GET();
//            get_init = 1;//取消开机状态
//            ucp_flag.get_data_flag = 1;
//            rt_timer_start ( ucp_timer );//开启ACK超时判定
//        }
//        if ( ota )
//        {
//            rt_timer_stop(ucp_timer);
//            //                LOG_D( "Start OTA" );
//            // Hand over resource for OTA
//            rt_device_close ( serial );
//            uart_int_sample();
//            rt_kprintf("start OTA dma!\n");
//            update_driver();
//            // Recover serial for control data
//            rt_device_open ( serial , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );
//            rt_device_set_rx_indicate ( serial , uart_input );
//            rt_kprintf("end OTA dma!\n");
////            uart_dma_sample();//配置初始化
//            //                LOG_D( "End OTA" );
//            ota = 0;
//        }
//        rt_thread_mdelay ( 500 );
//
//        rt_memset(&msg, 0, sizeof(msg));
//        /* 从消息队列中读取消息*/
//        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
//        if (result == 0)
//        {
//            /* 从串口读取数据*/
//            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
//            rx_buffer[rx_length] = '\0';
////            for(int i=0;i < rx_length; i++)
////            {
////                rt_kprintf("%x ",rx_buffer[i]);
////            }
////            return ;
////            LOG_I("rx_length = %d \n",rx_length);
//            //如果读到数据就将数据添加到环形缓存区
//            if ( rx_length != 0 )
//            {
//                rt_ringbuffer_put ( rb , rx_buffer , rx_length );//将读到的这批数据放到环形缓冲区中
//            }
//            ring_length = rt_ringbuffer_data_len ( rb );
////            LOG_I("ring_length = %d \n",ring_length);
//            /******************************/
//            while(ring_length >= 7 && (ring_length >= (handle_len - 1)))//判断环形缓冲区剩余字节数
//            {
//                if(handle_id == 0)//判断上一个数据包是否处理完毕
//                {
//                    for(int i = 0; i < ring_length ;i++)
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p , 1 );//读取1个字节数据
//                        ring_buffer_p ++;
//                        if(ring_buffer[0] == 0xfd)
//                        {
//                            rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p , 1 );//读取1个字节数据
//                            ring_buffer_p ++;
//                            ring_length = ring_length - 1;
//                            if(ring_buffer[1] == 0xff) //解析包头2
//                            {
//                                //已读到包头前2个字节，开始处理后两个字节的len、一个字节的ID
//                                rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p , 3 );//读取3个字节数据
//                                ring_buffer_p = ring_buffer_p + 3;
//                                ring_length = ring_length - 3;
//                                if(ring_buffer[4] >= 0x01 && ring_buffer[4] <= 0x0A)//判断ID号是否正确
//                                {
//                                    //5字节包头完整无误
//                                    handle_len = (ring_buffer[3] << 8) + ring_buffer[2];
//                                    handle_id = ring_buffer[4];
//                                    ring_length = ring_length - ( i + 1 );//处理数据包前更新缓冲区剩余数据长度
//                                    break;
//                                }
//                                else {
//                                    ring_buffer_p = 0;
//                                }
//
//                            }
//                            else {
//                                ring_buffer_p = 0;
//                            }
//                        }
//                        else {
//                            ring_buffer_p = 0;
//                        }
//
//                        if(i == (ring_length - 1)) //环形缓冲区被清空(极端情况)
//                        {
//                            ring_length = 0;
//                            break;
//                        }
//                    }
//                }
//                /******************************/
//                switch(handle_id)
//                {
//                case 0:
//                    ;
//                    break;
//                case 0x01:  //keep-alive(200ms)
//                {
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);//更新缓存区剩余数据长度
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，回复系统状态(FIXME)
//                            Keep_alive_ACK(RT_EOK);
//                            LOG_I("keep-alive!");
//
//                        }
//                        else {
//                            //校验失败，回复错误码
//                            Keep_alive_ACK(RT_ERROR);
//                            LOG_E("keep-alive error!");
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//                }
//                break;
//                case 0x02:  //motor control(100ms)
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，更新控制量
//                            robot_state.speed = (ring_buffer [ 7 ] << 8) + ring_buffer [ 6 ];
//                            robot_state.steer = (ring_buffer [ 9 ] << 8) + ring_buffer [ 8 ];
//                            robot_state.lamp = (ring_buffer [ 11 ] << 8) + ring_buffer [ 10 ];
//                            ota_version = (ring_buffer [ 15 ] << 8) + ring_buffer [ 14 ];
//                            if ( ota_version > APP_VERSION )
//                            {
//                                LOG_D( "New firmware version" );
//                                ota = 1;
//                            }
//                            //回复系统信息(RPM、IMU、voltage)
//                            uart_report_state();
//                            LOG_I("motor control!");
////                            for(int i=0;i<handle_len+4;i++)
////                            {
////                                rt_kprintf("%x ",ring_buffer[i]);
////                            }
////                            rt_kprintf("\n");
//                        }
//                        else {
//                            LOG_I("motor control error!");
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//                    break;
//                case 0x03: //IMU校准启动
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1 );
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，启动IMU校准模式(1：磁力计 ， 2：陀螺仪)
//                            calib_mode = ring_buffer [ 6 ];
//                            if(calib_mode == 1)
//                            {
//                                IMU_updata.MAG_calib_mode = 1;
//                            }
//                            else if(calib_mode == 2){
//                                IMU_updata.IMU_calib_mode = 1;
//                            }
//                            //回复ACK
//                            IMU_Correct_Start_ACK(calib_mode,0);
//                            LOG_I("0x03校验成功，IMU校准启动!");
//                        }
//                        else {
//                            //回复ACK
//                            IMU_Correct_Start_ACK(calib_mode,1);
//                            LOG_E("0x03校验失败，IMU校准启动 error!");
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//
//                    break;
//                case 0x04: //IMU校准结束
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1 );
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，关闭IMU校准模式(1：磁力计 ， 2：陀螺仪)
//                            calib_mode = ring_buffer [ 6 ];
//                            if(calib_mode == 1)
//                            {
//                                IMU_updata.MAG_calib_mode = 0;
//                            }
//                            else if(calib_mode == 2){
//                                IMU_updata.IMU_calib_mode = 0;
//                            }
//                            //回复ACK
//                            IMU_Correct_End_ACK(calib_mode,0);
//                            //向头部写入IMU校准数据
//                            if(calib_mode == 1)
//                            {
//                                //写入磁力计数据
//                                MAG_PERS_SET();
//                                //应答超时处理 FIXME(没有回复定期发送)
//                                ucp_flag.mag_set_flag = 1;
//                                rt_timer_start ( ucp_timer );//开启ACK超时判定
//                            }
//                            else if(calib_mode == 2){
//                                //写入陀螺仪数据
//                                IMU_PERS_SET();
//                                //应答超时处理 FIXME(没有回复定期发送)
//                                ucp_flag.imu_set_flag = 1;
//                                rt_timer_start ( ucp_timer );//开启ACK超时判定
//                            }
//                            LOG_I("0x04校验成功，IMU校准结束!");
//                        }
//                        else {
//                            //回复ACK
//                            IMU_Correct_End_ACK(calib_mode,1);
//                            LOG_E("0x04校验失败，IMU校准结束 error!");
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//
//                    break;
//                case 0x05:
//                    break;
//
//                case 0x06:
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，取消(IMU数据写入)重新发送
//                            ucp_flag.imu_set_flag = 0;
//                            LOG_I("0x06校验成功，取消(IMU数据写入)重新发送!");
//                        }
//                        else {
//                            ucp_flag.imu_set_flag = 1;
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//
//                    break;
//
//                case 0x07:
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，取消(磁力计数据写入)重新发送
//                            ucp_flag.mag_set_flag = 0;
//                            LOG_I("0x07校验成功，取消(磁力计数据写入)重新发送!");
//                        }
//                        else {
//                            ucp_flag.mag_set_flag = 1;
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//
//                    break;
//                case 0x08:
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功，接收IMU&磁力计初始数据
//                            IMU_updata.acc_bias[0] = (ring_buffer [ 8 ] << 8) + ring_buffer [ 7 ];
//                            IMU_updata.acc_bias[1] = (ring_buffer [ 10 ] << 8) + ring_buffer [ 9 ];
//                            IMU_updata.acc_bias[2] = (ring_buffer [ 12 ] << 8) + ring_buffer [ 11 ];
//                            IMU_updata.gyro_bias[0] = (ring_buffer [ 14 ] << 8) + ring_buffer [ 13 ];
//                            IMU_updata.gyro_bias[1] = (ring_buffer [ 16 ] << 8) + ring_buffer [ 15 ];
//                            IMU_updata.gyro_bias[2] = (ring_buffer [ 18 ] << 8) + ring_buffer [ 17 ];
//                            IMU_updata.mag_bias[0] = (ring_buffer [ 20 ] << 8) + ring_buffer [ 19 ];
//                            IMU_updata.mag_bias[1] = (ring_buffer [ 22 ] << 8) + ring_buffer [ 21 ];
//                            IMU_updata.mag_bias[2] = (ring_buffer [ 24 ] << 8) + ring_buffer [ 23 ];
//                            LOG_I("0x08校验成功，已接收IMU&磁力计初始数据!");
//                            //取消(IMU+磁力计数据获取请求)重新发送
//                            ucp_flag.get_data_flag = 0;
//                        }
//                        else {
//                            ucp_flag.get_data_flag = 1;
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//
//                    break;
//                case 0x09:
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功,回应ACK,准备OTA
//                            ota_version = (ring_buffer [ 7 ] << 8) + ring_buffer [ 6 ];
//                            LOG_I("0x09校验成功，开始校验版本号!");
//                            if ( ota_version > APP_VERSION )
//                            {
//                                LOG_D( "New firmware version,start OTA" );
////                                OTA_Start_ACK(0);
//                                {
//                                    rt_timer_stop(ucp_timer);
//                                    //                LOG_D( "Start OTA" );
//                                    // Hand over resource for OTA
//                                    rt_device_close ( serial );
//                                    rt_kprintf("start OTA dma!\n");
//                                    update_driver();
//                                    // Recover serial for control data
//                                    rt_device_open ( serial , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );
//                                    rt_device_set_rx_indicate ( serial , uart_input );
//                                    rt_kprintf("end OTA dma!\n");
//                                    uart_dma_sample();//配置初始化
//                                    //                LOG_D( "End OTA" );
//                                }
//                            }
//                            else {
////                                OTA_Start_ACK(1);
//                            }
//
//
//                        }
//                        else {
//                            OTA_Start_ACK(1);
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//                    break;
//
//                case 0x0A:
//                    if(ring_length >= (handle_len - 1))
//                    {
//                        rt_ringbuffer_get ( rb , ring_buffer + ring_buffer_p, handle_len - 1);
//                        ring_length = ring_length - (handle_len - 1);
//                        /**数据校验**/
//                        crc = crc16 ( ring_buffer , handle_len + 2 );
//                        if ( (crc & 0xff) == ring_buffer [ handle_len + 2 ] && (crc >> 8) == ring_buffer [ handle_len + 3 ] )
//                        {
//                            //校验成功,更新底部联网指示灯状态
//                            robot_state.net_led_status = ring_buffer [ 6 ];
//                        }
//                        else {
//
//                        }
//                        handle_id = 0;
//                        ring_buffer_p = 0;
//                    }
//                    break;
//                }
//            }
//
//        }
//    }
//}
//
//int uart_dma_sample(void)
//{
//    rt_err_t ret = RT_EOK;
//
//    /* 查找串口设备 */
//    serial = rt_device_find(SAMPLE_UART_NAME);
//    if (!serial)
//    {
//        rt_kprintf("find %s failed!\n", SAMPLE_UART_NAME);
//        return RT_ERROR;
//    }
//
//    /* step2：修改串口配置参数 */
//    config_dma.baud_rate = 115200;        //修改波特率为 115200
//    config_dma.data_bits = DATA_BITS_8;           //数据位 8
//    config_dma.stop_bits = STOP_BITS_1;           //停止位 1
//    config_dma.bufsz = 2048;                   //修改缓冲区 buff size 为 128
//    config_dma.parity = PARITY_NONE;           //无奇偶校验位
//
//    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
//    rt_device_control ( serial , RT_DEVICE_CTRL_CONFIG , &config_dma );
//    /* 初始化消息队列 */
//    rt_mq_init(&rx_mq, "rx_mq",
//               msg_pool,                 /* 存放消息的缓冲区 */
//               sizeof(struct rx_msg),    /* 一条消息的最大长度 */
//               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
//               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
//
//    /* 以 DMA 接收及轮询发送方式打开串口设备 */
//    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
//    /* 设置接收回调函数 */
//    rt_device_set_rx_indicate(serial, uart_input);
//
//    /* 创建环形缓存区 */
//    rb = rt_ringbuffer_create ( RING_BUFFER_LEN );
//    if ( rb == RT_NULL )
//    {
//        rt_kprintf ( "Can't create ringbffer\r\n" );
//        return RT_ERROR;
//    }
//
//    return ret;
//}
//
//int uart_int_sample(void)
//{
//    rt_err_t ret = RT_EOK;
//    /* 查找串口设备 */
//    serial = rt_device_find(SAMPLE_UART_NAME);
//    if (!serial)
//    {
//        rt_kprintf("find %s failed!\n", SAMPLE_UART_NAME);
//        return RT_ERROR;
//    }
//    int config_d = RT_DEVICE_FLAG_DMA_RX; // 假设你要清除 DMA 接收中断
//
//    rt_err_t result = rt_device_control(serial, RT_DEVICE_CTRL_CLR_INT, &config_d);
//    if (result == RT_EOK)
//    {
//        // 成功清除中断
//        LOG_I("成功清除DMA接收中断！");
//    }
//    /* step2：修改串口配置参数 */
//        config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
//        config.data_bits = DATA_BITS_8;           //数据位 8
//        config.stop_bits = STOP_BITS_1;           //停止位 1
//        config.bufsz = 2048;                   //修改缓冲区 buff size 为 128
//        config.parity = PARITY_NONE;           //无奇偶校验位
//
//        /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
//        rt_device_control ( serial , RT_DEVICE_CTRL_CLR_INT , &config );
//
//        rt_sem_init ( &rx_sem , "rx_sem" , 0 , RT_IPC_FLAG_FIFO );
//        /* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备 */
//        rt_device_open ( serial , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX );
//        rt_device_set_rx_indicate ( serial , uart_int_input );
//
//}
//
//static void uart_timer_timeout ( void *parameter )
//{
//    if(ucp_flag.get_data_flag == 1)
//    {
//        IMU_PERS_GET();//再次发送
//        LOG_I( "冷却时间达到，再次请求数据获取！" );
//        rt_timer_start ( ucp_timer );//开启ACK超时判定
//    }
//    if(ucp_flag.imu_set_flag == 1)
//    {
//        IMU_PERS_SET();
////        LOG_I( "冷却时间达到，再次请求IMU写入！" );
//        rt_timer_start ( ucp_timer );//开启ACK超时判定
//    }
//    if(ucp_flag.mag_set_flag == 1)
//    {
//        MAG_PERS_SET();
////        LOG_I( "冷却时间达到，再次请求MAG写入！" );
//        rt_timer_start ( ucp_timer );//开启ACK超时判定
//    }
////    LOG_I( "冷却时间达到！" );
//}
//void uart_timout_init ( void )
//{
//      ucp_timer = rt_timer_create ( "ucp_timer" , uart_timer_timeout , RT_NULL , 1000 , RT_TIMER_FLAG_ONE_SHOT );
//}
///* 导出到 msh 命令列表中 */
//MSH_CMD_EXPORT(uart_dma_sample, uart device dma sample);
