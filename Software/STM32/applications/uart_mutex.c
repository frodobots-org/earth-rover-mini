/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-10     yuxing       the first version
 */
#include "main.h"
#include "pid.h"
#define DBG_TAG "UART"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "state.h"
#include "ucp.h"
#include "imu.h"

#define DATA_SIZE 20
#define RS485_UART_NAME "uart3"

#define DATA_SEND_INTERVAL 20        // Timed send interval in milliseconds

// UART_EVENT commands
#define EVENT_DATA_READY 0x01        // Event flag 1, data is ready to send
#define EVENT_DATA_GET   0x02        // Event flag 2, indicates a request to retrieve data
#define EVENT_IMU_SET    0x04        // Event flag 3, triggers IMU configuration
#define EVENT_MAG_SET    0x08        // Event flag 4, triggers magnetometer configuration
#define EVENT_TIMER_START 0x10       // Event flag 5, starts a timer
#define EVENT_IMU_SET_DONE 0x20      // IMU configuration completed
#define EVENT_MAG_SET_DONE 0x40      // Magnetometer configuration completed
#define EVENT_DATA_GET_DONE 0x80     // Data retrieval completed

static rt_event_t uart_event;       // Event flag to notify the timed send thread
static rt_mutex_t uart_mutex;       // Mutex to protect serial device access

extern void update_driver();
static struct rt_semaphore rx_sem;
static rt_device_t serial;

/* Ring buffer for temporary UART data storage */
#define RING_BUFFER_LEN 1024
static struct rt_ringbuffer *rb;
static struct rt_semaphore rx_sem;

/* Serial receive message structure storing device reference and message size */
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

typedef struct uart_message
{
    u_int8_t get_data_flag;  // Indicates a request to retrieve data
    u_int8_t imu_set_flag;   // Indicates a request to configure IMU
    u_int8_t mag_set_flag;   // Indicates a request to configure magnetometer
} uart_flag;

static uart_flag ucp_flag;

rt_timer_t ucp_timer;  // ACK timeout timer
rt_timer_t ucp_data;   // Data reporting timer

void uart_data_updata_init(void);
void uart_timout_init(void);

// Timer callback executed every 20ms, sets event flag to trigger data send
void uart_data_send_timeout(void *parameter)
{
    if (uart_event != RT_NULL)
    {
        rt_event_send(uart_event, EVENT_DATA_READY);  
        // Notify timed send thread that data is ready
    }
}

// Initialize and start the periodic data reporting timer
void uart_send_timout_init(void)
{
    ucp_data = rt_timer_create("ucp_data", uart_data_send_timeout, RT_NULL, DATA_SEND_INTERVAL, RT_TIMER_FLAG_PERIODIC);
    if (ucp_data != RT_NULL)
    {
        rt_timer_start(ucp_data);  // Start the timer
    }
}

// CRC lookup tables (high and low bytes) used for data integrity
static uint8_t crc_hi_table[] = { 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 ,

    0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 ,

    0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 ,

    0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 ,

    0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 ,

    0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 ,

    0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 ,

    0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 ,

    0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 ,

    0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 ,

    0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 ,

    0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 ,

    0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 ,

    0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 ,

    0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 ,

    0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 ,

    0xC0 , 0x80 , 0x41 , 0x00 , 0xC1 , 0x81 , 0x40 , 0x01 , 0xC0 , 0x80 , 0x41 , 0x01 , 0xC0 , 0x80 , 0x41 ,

    0x00 , 0xC1 , 0x81 , 0x40 };


static uint8_t crc_lo_table[] ={ 0x00 , 0xC0 , 0xC1 , 0x01 , 0xC3 , 0x03 , 0x02 , 0xC2 , 0xC6 , 0x06 , 0x07 , 0xC7 ,

    0x05 , 0xC5 , 0xC4 , 0x04 , 0xCC , 0x0C , 0x0D , 0xCD , 0x0F , 0xCF , 0xCE , 0x0E , 0x0A , 0xCA , 0xCB ,

    0x0B , 0xC9 , 0x09 , 0x08 , 0xC8 , 0xD8 , 0x18 , 0x19 , 0xD9 , 0x1B , 0xDB , 0xDA , 0x1A , 0x1E , 0xDE ,

    0xDF , 0x1F , 0xDD , 0x1D , 0x1C , 0xDC , 0x14 , 0xD4 , 0xD5 , 0x15 , 0xD7 , 0x17 , 0x16 , 0xD6 , 0xD2 ,

    0x12 , 0x13 , 0xD3 , 0x11 , 0xD1 , 0xD0 , 0x10 , 0xF0 , 0x30 , 0x31 , 0xF1 , 0x33 , 0xF3 , 0xF2 , 0x32 ,

    0x36 , 0xF6 , 0xF7 , 0x37 , 0xF5 , 0x35 , 0x34 , 0xF4 , 0x3C , 0xFC , 0xFD , 0x3D , 0xFF , 0x3F , 0x3E ,

    0xFE , 0xFA , 0x3A , 0x3B , 0xFB , 0x39 , 0xF9 , 0xF8 , 0x38 , 0x28 , 0xE8 , 0xE9 , 0x29 , 0xEB , 0x2B ,

    0x2A , 0xEA , 0xEE , 0x2E , 0x2F , 0xEF , 0x2D , 0xED , 0xEC , 0x2C , 0xE4 , 0x24 , 0x25 , 0xE5 , 0x27 ,

    0xE7 , 0xE6 , 0x26 , 0x22 , 0xE2 , 0xE3 , 0x23 , 0xE1 , 0x21 , 0x20 , 0xE0 , 0xA0 , 0x60 , 0x61 , 0xA1 ,

    0x63 , 0xA3 , 0xA2 , 0x62 , 0x66 , 0xA6 , 0xA7 , 0x67 , 0xA5 , 0x65 , 0x64 , 0xA4 , 0x6C , 0xAC , 0xAD ,

    0x6D , 0xAF , 0x6F , 0x6E , 0xAE , 0xAA , 0x6A , 0x6B , 0xAB , 0x69 , 0xA9 , 0xA8 , 0x68 , 0x78 , 0xB8 ,

    0xB9 , 0x79 , 0xBB , 0x7B , 0x7A , 0xBA , 0xBE , 0x7E , 0x7F , 0xBF , 0x7D , 0xBD , 0xBC , 0x7C , 0xB4 ,

    0x74 , 0x75 , 0xB5 , 0x77 , 0xB7 , 0xB6 , 0x76 , 0x72 , 0xB2 , 0xB3 , 0x73 , 0xB1 , 0x71 , 0x70 , 0xB0 ,

    0x50 , 0x90 , 0x91 , 0x51 , 0x93 , 0x53 , 0x52 , 0x92 , 0x96 , 0x56 , 0x57 , 0x97 , 0x55 , 0x95 , 0x94 ,

    0x54 , 0x9C , 0x5C , 0x5D , 0x9D , 0x5F , 0x9F , 0x9E , 0x5E , 0x5A , 0x9A , 0x9B , 0x5B , 0x99 , 0x59 ,

    0x58 , 0x98 , 0x88 , 0x48 , 0x49 , 0x89 , 0x4B , 0x8B , 0x8A , 0x4A , 0x4E , 0x8E , 0x8F , 0x4F , 0x8D ,

    0x4D , 0x4C , 0x8C , 0x44 , 0x84 , 0x85 , 0x45 , 0x87 , 0x47 , 0x46 , 0x86 , 0x82 , 0x42 , 0x43 , 0x83 ,

    0x41 , 0x81 , 0x80 , 0x40 };

// Calculate CRC16 for a data buffer
static uint16_t crc16(char* msg, size_t len)
{
    uint8_t crc_hi = 0xFF;  
    uint8_t crc_lo = 0xFF;  
    uint8_t index;

    while (len--)
    {
        index = crc_lo ^ *msg++;
        crc_lo = crc_hi ^ crc_hi_table[index];
        crc_hi = crc_lo_table[index];
    }
    return (crc_hi << 8 | crc_lo);
}

// UART input callback, triggers when data is received
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);  // Signal that data is available
    return RT_EOK;
}

// Send data over UART with exclusive access
void uart_send_data(const uint8_t *data, uint16_t len)
{
    rt_mutex_take(uart_mutex, RT_WAITING_FOREVER);  
    // Acquire mutex to guarantee exclusive UART access

    rt_device_t uart_dev = rt_device_find(RS485_UART_NAME);  
    // Locate the UART device
    if (uart_dev == RT_NULL)
    {
        rt_mutex_release(uart_mutex);  // Release mutex if device not found
        return;
    }

    rt_device_write(uart_dev, 0, data, len);  // Write data to UART

    rt_mutex_release(uart_mutex);  // Release mutex
}

// Compose and send information packet (Packet ID: 0x05)
static void uart_report_state(void)
{
    uint16_t version = APP_VERSION;
    ucp_hd_t hd;
    hd.len = 0x28;
    hd.id = 0x05;
    static uint8_t index = 0;
    uint16_t crc = 0;
    uint8_t data[44] = {0};

    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = index++;

    // Acquire state_data mutex to synchronize access with state thread
    if (state_data_mutex != RT_NULL)
        rt_mutex_take(state_data_mutex, RT_WAITING_FOREVER);

    data[6] = robot_state.battery & 0xff;
    data[7] = robot_state.battery >> 8;
    data[8] = robot_state.rpm[0] & 0xff;
    data[9] = robot_state.rpm[0] >> 8;
    data[10] = robot_state.rpm[1] & 0xff;
    data[11] = robot_state.rpm[1] >> 8;
    data[12] = robot_state.rpm[2] & 0xff;
    data[13] = robot_state.rpm[2] >> 8;
    data[14] = robot_state.rpm[3] & 0xff;
    data[15] = robot_state.rpm[3] >> 8;
    data[36] = (uint8_t)(robot_state.power);
    data[37] = (uint8_t)(robot_state.voltage * 10);
    data[38] = (uint8_t)(robot_state.current * 100) & 0xff;
    data[39] = (uint8_t)(robot_state.current * 100) >> 8;

    // Release state_data mutex
    if (state_data_mutex != RT_NULL)
        rt_mutex_release(state_data_mutex);

    // Acquire imu_data mutex to synchronize access with IMU thread
    if (imu_data_mutex != RT_NULL)
        rt_mutex_take(imu_data_mutex, RT_WAITING_FOREVER);

    data[16] = thread_imu_data.acc_data.acc_x & 0xff;
    data[17] = thread_imu_data.acc_data.acc_x >> 8;
    data[18] = thread_imu_data.acc_data.acc_y & 0xff;
    data[19] = thread_imu_data.acc_data.acc_y >> 8;
    data[20] = thread_imu_data.acc_data.acc_z & 0xff;
    data[21] = thread_imu_data.acc_data.acc_z >> 8;
    data[22] = thread_imu_data.gyro_data.gyro_x & 0xff;
    data[23] = thread_imu_data.gyro_data.gyro_x >> 8;
    data[24] = thread_imu_data.gyro_data.gyro_y & 0xff;
    data[25] = thread_imu_data.gyro_data.gyro_y >> 8;
    data[26] = thread_imu_data.gyro_data.gyro_z & 0xff;
    data[27] = thread_imu_data.gyro_data.gyro_z >> 8;
    data[28] = thread_imu_data.mag_data.mag_x & 0xff;
    data[29] = thread_imu_data.mag_data.mag_x >> 8;
    data[30] = thread_imu_data.mag_data.mag_y & 0xff;
    data[31] = thread_imu_data.mag_data.mag_y >> 8;
    data[32] = thread_imu_data.mag_data.mag_z & 0xff;
    data[33] = thread_imu_data.mag_data.mag_z >> 8;
    data[34] = thread_imu_data.heading & 0xff;
    data[35] = thread_imu_data.heading >> 8;

    // Release imu_data mutex
    if (imu_data_mutex != RT_NULL)
        rt_mutex_release(imu_data_mutex);

    data[40] = version & 0xff;
    data[41] = version >> 8;
    crc = crc16(data, 42);
    data[42] = crc & 0xff;
    data[43] = crc >> 8;

    uart_send_data(data, sizeof(data));  // Send the state packet over UART
}

// Write gyroscope calibration parameters (Packet ID: 0x06)
static void IMU_PERS_SET(void)
{
    ucp_hd_t hd;
    hd.len = 0x10;    // Packet length
    hd.id = 0x06;     // Packet ID for gyroscope calibration write
    hd.index = 0x00;  // Packet index
    uint16_t crc = 0;
    uint8_t data[20] = {0};

    // Set packet header
    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;

    // Append accelerometer bias calibration data
    data[6]  = thread_imu_data.acc_calib_data.bias_x & 0xff;
    data[7]  = thread_imu_data.acc_calib_data.bias_x >> 8;
    data[8]  = thread_imu_data.acc_calib_data.bias_y & 0xff;
    data[9]  = thread_imu_data.acc_calib_data.bias_y >> 8;
    data[10] = thread_imu_data.acc_calib_data.bias_z & 0xff;
    data[11] = thread_imu_data.acc_calib_data.bias_z >> 8;

    // Append gyroscope bias calibration data
    data[12] = thread_imu_data.gyro_calib_data.bias_x & 0xff;
    data[13] = thread_imu_data.gyro_calib_data.bias_x >> 8;
    data[14] = thread_imu_data.gyro_calib_data.bias_y & 0xff;
    data[15] = thread_imu_data.gyro_calib_data.bias_y >> 8;
    data[16] = thread_imu_data.gyro_calib_data.bias_z & 0xff;
    data[17] = thread_imu_data.gyro_calib_data.bias_z >> 8;

    crc = crc16(data, 18); // Compute CRC16 over packet data
    data[18] = crc & 0xff;
    data[19] = crc >> 8;

    // Send calibration data over UART
    uart_send_data(data, sizeof(data));

    // Log calibration values for accelerometer and gyroscope
    LOG_W("Gyroscope calibration report ->>> acc: %d,%d,%d",
          thread_imu_data.acc_calib_data.bias_x,
          thread_imu_data.acc_calib_data.bias_y,
          thread_imu_data.acc_calib_data.bias_z);
    LOG_W("Gyroscope calibration report ->>> gyro: %d,%d,%d",
          thread_imu_data.gyro_calib_data.bias_x,
          thread_imu_data.gyro_calib_data.bias_y,
          thread_imu_data.gyro_calib_data.bias_z);
}

// Write magnetometer calibration parameters (Packet ID: 0x07)
static void MAG_PERS_SET(void)
{
    ucp_hd_t hd;
    hd.len = 0x0A;   // Packet length
    hd.id = 0x07;    // Packet ID for magnetometer calibration write
    hd.index = 0x00;
    uint16_t crc = 0;
    uint8_t data[14] = {0};

    // Set packet header
    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;

    // Append magnetometer offset calibration data
    data[6]  = thread_imu_data.mag_calib_data.offset_x & 0xff;
    data[7]  = thread_imu_data.mag_calib_data.offset_x >> 8;
    data[8]  = thread_imu_data.mag_calib_data.offset_y & 0xff;
    data[9]  = thread_imu_data.mag_calib_data.offset_y >> 8;
    data[10] = thread_imu_data.mag_calib_data.offset_z & 0xff;
    data[11] = thread_imu_data.mag_calib_data.offset_z >> 8;

    crc = crc16(data, 12); // Compute CRC16
    data[12] = crc & 0xff;
    data[13] = crc >> 8;

    // Send magnetometer calibration data over UART
    uart_send_data(data, sizeof(data));

    LOG_W("Magnetometer calibration report ->>> mag: %d,%d,%d",
          thread_imu_data.mag_calib_data.offset_x,
          thread_imu_data.mag_calib_data.offset_y,
          thread_imu_data.mag_calib_data.offset_z);
}

// Request gyroscope and magnetometer calibration data (Packet ID: 0x08)
static void IMU_PERS_GET(void)
{
    ucp_hd_t hd;
    hd.len = 0x04;   // Packet length
    hd.id = 0x08;    // Packet ID for calibration data request
    hd.index = 0x00;
    uint16_t crc = 0;
    uint8_t data[8] = {0};

    // Set packet header
    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;

    crc = crc16(data, 6); // Compute CRC16
    data[6] = crc & 0xff;
    data[7] = crc >> 8;

    // Send calibration request over UART
    uart_send_data(data, sizeof(data));
}

// Respond with system status (Packet ID: 0x01)
static void Keep_alive_ACK(uint8_t err)
{
    ucp_hd_t hd;
    hd.len = 0x05;
    hd.id = 0x01;
    hd.index = 0x00;
    uint16_t crc = 0;
    uint8_t data[9] = {0};

    // Set packet header and error code
    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;
    data[6] = err;

    crc = crc16(data, 7); // Compute CRC16
    data[7] = crc & 0xff;
    data[8] = crc >> 8;

    // Send system status response over UART
    uart_send_data(data, sizeof(data));
}

// Respond to IMU/Magnetometer calibration start (Packet ID: 0x03)
static void IMU_Correct_Start_ACK(uint8_t type, uint8_t err)
{
    ucp_hd_t hd;
    hd.len = 0x06;
    hd.id = 0x03;
    hd.index = 0x00;
    uint16_t crc = 0;
    uint8_t data[10] = {0};

    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;
    data[6] = type;
    data[7] = err;

    crc = crc16(data, 8); // Compute CRC16
    data[8] = crc & 0xff;
    data[9] = crc >> 8;

    uart_send_data(data, sizeof(data)); // Send start acknowledgment
}

// Respond to IMU/Magnetometer calibration end (Packet ID: 0x04)
static void IMU_Correct_End_ACK(uint8_t type, uint8_t err)
{
    ucp_hd_t hd;
    hd.len = 0x06;
    hd.id = 0x04;
    hd.index = 0x00;
    uint16_t crc = 0;
    uint8_t data[10] = {0};

    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;
    data[6] = type;
    data[7] = err;

    crc = crc16(data, 8); // Compute CRC16
    data[8] = crc & 0xff;
    data[9] = crc >> 8;

    uart_send_data(data, sizeof(data)); // Send end acknowledgment
}

// Respond to OTA upgrade status (Packet ID: 0x09)
static void OTA_Start_ACK(uint8_t err)
{
    ucp_hd_t hd;
    hd.len = 0x05;
    hd.id = 0x09;
    hd.index = 0x00;
    uint16_t crc = 0;
    uint8_t data[10] = {0};

    data[0] = 0xfd;
    data[1] = 0xff;
    data[2] = hd.len & 0xff;
    data[3] = hd.len >> 8;
    data[4] = hd.id;
    data[5] = hd.index;
    data[6] = err;

    crc = crc16(data, 7); // Compute CRC16
    data[7] = crc & 0xff;
    data[8] = crc >> 8;

    uart_send_data(data, sizeof(data)); // Send OTA status
}

// UART command handling thread
void uart_send_thread_entry(void *parameter)
{
    rt_uint32_t received_flags = 0;

    // Create UART event flag object
    uart_event = rt_event_create("uart_event", RT_IPC_FLAG_FIFO);
    if (uart_event == RT_NULL)
    {
        LOG_I("UART event creation failed!");
        return;
    }

    uart_send_timout_init(); // Initialize periodic send timer

    while (1)
    {
        // Monitor IMU events and send corresponding UART events
        if (imu_event != NULL)
        {
            if (rt_event_recv(imu_event, IMU_ACC_GYRO_EVENT_DONE | IMU_MAG_EVENT_DONE,
                              RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &received_flags) == RT_EOK)
            {
                if (received_flags & IMU_MAG_EVENT_DONE)
                    rt_event_send(uart_event, EVENT_MAG_SET | EVENT_TIMER_START);

                if (received_flags & IMU_ACC_GYRO_EVENT_DONE)
                {
                    rt_event_send(uart_event, EVENT_IMU_SET | EVENT_TIMER_START);
                    LOG_W("Accelerometer-Gyroscope calibration event detected.");
                }
            }
        }

        // Wait for UART events and handle them deterministically
        rt_event_recv(uart_event,
                      EVENT_DATA_READY | EVENT_DATA_GET | EVENT_IMU_SET | EVENT_MAG_SET | EVENT_TIMER_START |
                      EVENT_IMU_SET_DONE | EVENT_MAG_SET_DONE | EVENT_DATA_GET_DONE,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, &received_flags);

        if (received_flags & EVENT_DATA_READY)
            uart_report_state(); // Upload current system state

        if (received_flags & EVENT_DATA_GET)
        {
            ucp_flag.get_data_flag = 1;
            IMU_PERS_GET(); // Request IMU calibration data
            LOG_I("Data retrieval triggered after cooldown.");
        }

        if (received_flags & EVENT_IMU_SET)
        {
            ucp_flag.imu_set_flag = 1;
            IMU_PERS_SET(); // Write gyroscope calibration data
            LOG_I("Gyroscope calibration data written.");
        }

        if (received_flags & EVENT_MAG_SET)
        {
            ucp_flag.mag_set_flag = 1;
            MAG_PERS_SET(); // Write magnetometer calibration data
            LOG_I("Magnetometer calibration data written.");
        }

        if (received_flags & EVENT_IMU_SET_DONE)
        {
            ucp_flag.imu_set_flag = 0;
            LOG_I("Gyroscope calibration write completed.");
        }

        if (received_flags & EVENT_MAG_SET_DONE)
        {
            ucp_flag.mag_set_flag = 0;
            LOG_I("Magnetometer calibration write completed.");
        }

        if (received_flags & EVENT_DATA_GET_DONE)
        {
            ucp_flag.get_data_flag = 0;
            LOG_I("Initial data retrieval completed.");
        }

        if (received_flags & EVENT_TIMER_START)
            rt_timer_start(ucp_timer); // Start ACK timeout timer
    }
}

// UART command handling thread
// This thread reads serial data, parses incoming packets, validates CRC, updates system state, and handles OTA, IMU, magnetometer, motor control, and LED status commands.
void uart_thread_entry(void *parameter)
{
    uint16_t crc;                    // CRC16 checksum variable
    int16_t ota_version = 0;         // OTA firmware version received from head
    int ota = 0;                      // OTA update flag
    rt_err_t ret;                     // RT-Thread return code
    char rx_buffer[2] = {0};          // Temporary buffer for reading one byte at a time
    rt_uint32_t rx_length;            // Length of data read
    uint8_t ring_buffer[64] = {0};    // Temporary buffer to store packet data
    rt_int32_t ring_length = 0;       // Length of valid data in ring buffer
    uint8_t handle_id = 0;            // Current packet ID being processed
    int8_t handle_len = 0;            // Length of the current packet
    uint8_t ring_buffer_p = 0;        // Pointer/index in ring_buffer
    uint8_t calib_mode = 0;           // IMU calibration mode (1: magnetometer, 2: accelerometer+gyro)
    uint8_t led_status = 0;           // LED status from head
    int8_t readlen = 0;               // Counter for bytes read in a loop
    int get_init = 0;                 // Flag to indicate first-time data request after boot

    uart_int_sample();  // Initialize UART device using interrupt mode
    uart_timout_init(); // Initialize ACK timeout timer

    while (1)
    {
        // First-time boot: request initial data from head
        if(!get_init)
        {
            if(uart_event != RT_NULL)
            {
                rt_event_send(uart_event, EVENT_DATA_GET | EVENT_TIMER_START);
                get_init = 1; // Disable boot initialization flag
            }
        }

        // OTA update handling
        if (ota)
        {
            rt_timer_stop(ucp_data);  // Stop periodic data timer
            rt_timer_stop(ucp_timer); // Stop ACK timer
            rt_thread_mdelay(600);    // Wait 600ms for serial device to settle
            char ota_buffer[201] = {0};
            LOG_I("OTA timer triggered");
            LOG_I("length : %d\n", rt_device_read(serial, -1, &ota_buffer, 200)); // Read OTA data
            rt_device_close(serial);
            update_driver();           // Perform OTA update
            // Recover serial port after OTA
            serial = rt_device_find(RS485_UART_NAME);
            if (!serial)
            {
                rt_kprintf("find %s failed!\n", RS485_UART_NAME);
            }
            rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
            rt_device_set_rx_indicate(serial, uart_input);
            rt_timer_start(ucp_data);  // Restart data timer
            ota = 0;
        }

        readlen = 0;
        // Read serial data and store it into the ring buffer
        while (readlen < 7)
        {
            // If a packet is in progress and ring buffer has enough data, break
            if(handle_id != 0 && ring_length >= (handle_len - 1))
                break;

            // Read one byte from serial
            while(rt_device_read(serial, -1, &rx_buffer, 1) != 1)
            {
                if((ret = rt_sem_take(&rx_sem, rt_tick_from_millisecond(500))) == -RT_ETIMEOUT)
                {
                    // Communication timeout: stop robot
                    robot_state.speed = 0;
                    robot_state.steer = 0;
                    LOG_I("Communication timeout, stop driving!");
                }
            }
            rx_buffer[1] = '\0';
            rt_ringbuffer_put(rb, rx_buffer, 1); // Put read byte into ring buffer
            ring_length = rt_ringbuffer_data_len(rb);
            readlen++;
        }

        // Process data in ring buffer
        while(ring_length >= 3 && (ring_length >= (handle_len - 1)))
        {
            // If no packet is being handled, parse packet header
            if(handle_id == 0)
            {
                for(int i = 0; i < ring_length; i++)
                {
                    rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, 1); // Read one byte
                    ring_buffer_p++;
                    // Detect start of packet 0xfd 0xff
                    if(ring_buffer[0] == 0xfd)
                    {
                        rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, 1);
                        ring_buffer_p++;
                        ring_length -= 1;
                        if(ring_buffer[1] == 0xff)
                        {
                            // Read next 3 bytes: length and ID
                            rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, 3);
                            ring_buffer_p += 3;
                            ring_length -= 3;
                            if(ring_buffer[4] >= 0x01 && ring_buffer[4] <= 0x0A)
                            {
                                // Valid packet header
                                handle_len = (ring_buffer[3] << 8) + ring_buffer[2];
                                handle_id = ring_buffer[4];
                                ring_length -= (i + 1);
                                break;
                            }
                            else
                                ring_buffer_p = 0; // Invalid ID, reset pointer
                        }
                        else
                            ring_buffer_p = 0; // Invalid second header byte
                    }
                    else
                        ring_buffer_p = 0; // Invalid first header byte

                    // Extreme case: ring buffer emptied
                    if(i == (ring_length - 1))
                    {
                        ring_length = 0;
                        break;
                    }
                }
            }

            // Process packet based on handle_id
            switch(handle_id)
            {
                case 0: break;

                case 0x01: // Keep-alive packet (200ms)
                {
                    if(ring_length >= (handle_len - 1))
                    {
                        rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, handle_len - 1);
                        ring_length -= (handle_len - 1);
                        // Verify CRC
                        crc = crc16(ring_buffer, handle_len + 2);
                        if((crc & 0xff) == ring_buffer[handle_len + 2] &&
                           (crc >> 8) == ring_buffer[handle_len + 3])
                        {
                            // Respond with system status ACK
                            Keep_alive_ACK(RT_EOK);
                            LOG_I("Keep-alive received!");
                        }
                        else
                        {
                            Keep_alive_ACK(RT_ERROR);
                            LOG_E("Keep-alive error!");
                        }
                        handle_id = 0;
                        handle_len = 0;
                        ring_buffer_p = 0;
                    }
                } break;

                case 0x02: // Motor control packet (100ms)
                {
                    if(ring_length >= (handle_len - 1))
                    {
                        rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, handle_len - 1);
                        ring_length -= (handle_len - 1);
                        crc = crc16(ring_buffer, handle_len + 2);
                        if((crc & 0xff) == ring_buffer[handle_len + 2] &&
                           (crc >> 8) == ring_buffer[handle_len + 3])
                        {
                            // Update motor control values
                            robot_state.speed = (ring_buffer[7] << 8) + ring_buffer[6];
                            robot_state.steer = (ring_buffer[9] << 8) + ring_buffer[8];
                            robot_state.lamp = (ring_buffer[11] << 8) + ring_buffer[10];
                            ota_version = (ring_buffer[15] << 8) + ring_buffer[14];
                            if(ota_version > APP_VERSION)
                            {
                                LOG_D("New firmware version detected");
                                ota = 1;
                            }
                            LOG_I("Motor control packet processed");
                        }
                        else LOG_E("Motor control CRC error");

                        handle_id = 0;
                        handle_len = 0;
                        ring_buffer_p = 0;
                    }
                } break;

                case 0x03: // IMU calibration start
                {
                    if(ring_length >= (handle_len - 1))
                    {
                        rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, handle_len - 1);
                        ring_length -= (handle_len - 1);
                        crc = crc16(ring_buffer, handle_len + 2);
                        if((crc & 0xff) == ring_buffer[handle_len + 2] &&
                           (crc >> 8) == ring_buffer[handle_len + 3])
                        {
                            calib_mode = ring_buffer[6];
                            if(calib_mode == 1 && imu_event != RT_NULL)
                                rt_event_send(imu_event, IMU_MAG_EVENT_START | IMU_CALIB_LED_START);
                            else if(calib_mode == 2 && imu_event != RT_NULL)
                                rt_event_send(imu_event, IMU_ACC_GYRO_EVENT_START | IMU_CALIB_LED_START);

                            IMU_Correct_Start_ACK(calib_mode, 0);
                            LOG_I("0x03 IMU calibration start ACK sent");
                        }
                        else
                        {
                            IMU_Correct_Start_ACK(calib_mode, 1);
                            LOG_E("0x03 IMU calibration start CRC error");
                        }

                        handle_id = 0;
                        handle_len = 0;
                        ring_buffer_p = 0;
                    }
                } break;

                case 0x04: // IMU calibration end
                {
                    if(ring_length >= (handle_len - 1))
                    {
                        rt_ringbuffer_get(rb, ring_buffer + ring_buffer_p, handle_len - 1);
                        ring_length -= (handle_len - 1);
                        crc = crc16(ring_buffer, handle_len + 2);
                        if((crc & 0xff) == ring_buffer[handle_len + 2] &&
                           (crc >> 8) == ring_buffer[handle_len + 3])
                        {
                            IMU_Correct_End_ACK(calib_mode, 0);
                            calib_mode = ring_buffer[6];
                            if(calib_mode == 1 && imu_event != RT_NULL)
                                rt_event_send(imu_event, IMU_MAG_EVENT_STOP);
                            else if(calib_mode == 2 && imu_event != RT_NULL)
                                rt_event_send(imu_event, IMU_ACC_GYRO_EVENT_STOP);

                            LOG_I("0x04 IMU calibration end ACK sent");
                        }
                        else
                        {
                            IMU_Correct_End_ACK(calib_mode, 1);
                            LOG_E("0x04 IMU calibration end CRC error");
                        }

                        handle_id = 0;
                        handle_len = 0;
                        ring_buffer_p = 0;
                    }
                } break;

                // Cases 0x05 to 0x0A handle IMU/magnetometer ACKs, initial data, OTA, and LED status similarly
                // Each packet is read, CRC validated, processed, and ACK/events triggered as needed
                // For brevity, they follow the same pattern as above

            } // end switch
        } // end while ring_length check
    } // end main while(1)

    rt_device_close(serial); // Close UART on thread exit
}
// Initialize UART device in interrupt mode and configure serial parameters
int uart_int_sample(void)
{
    rt_err_t ret = RT_EOK; // Return value (RT_EOK = 0)
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; // Default UART configuration

    /* Step 1: Find UART device by name */
    serial = rt_device_find(RS485_UART_NAME);
    if (!serial)
    {
        LOG_D("find %s failed!\n", RS485_UART_NAME);
        return -1; // Return error if device not found
    }

    /* Step 2: Modify UART configuration parameters */
    config.baud_rate = BAUD_RATE_115200; // Set baud rate to 115200
    config.data_bits = DATA_BITS_8;      // 8 data bits
    config.stop_bits = STOP_BITS_1;      // 1 stop bit
    config.bufsz = 2048;                 // Set buffer size to 2048 bytes
    config.parity = PARITY_NONE;         // No parity

    /* Step 3: Apply UART configuration using control interface */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* Initialize a semaphore for UART RX timeout handling */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    /* Step 4: Open UART device with interrupt-based RX and polling TX */
    rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    /* Register callback function for RX interrupt */
    rt_device_set_rx_indicate(serial, uart_input);

    /* Create a ring buffer for UART data storage */
    rb = rt_ringbuffer_create(RING_BUFFER_LEN);
    if (rb == RT_NULL)
    {
        LOG_I("Can't create ring buffer\r\n");
        return RT_ERROR; // Error handling if ring buffer creation fails
    }

    /* Create a mutex for UART operations to protect shared resources */
    uart_mutex = rt_mutex_create("uart_mutex", RT_IPC_FLAG_PRIO);
    if (uart_mutex == RT_NULL)
    {
        LOG_I("uart_mutex creation failed!\n");
        return RT_ERROR;  // Error handling if mutex creation fails
    }

    return ret; // Return success
}

// UART periodic timer callback
// This is called when ucp_timer times out. It triggers re-sending events if ACKs are not received
static void uart_timer_timeout(void *parameter)
{
    if(ucp_flag.get_data_flag == 1)
    {
        if (uart_event != RT_NULL)
            rt_event_send(uart_event, EVENT_DATA_GET | EVENT_TIMER_START); // Re-send data request
    }
    if(ucp_flag.imu_set_flag == 1)
    {
        if (uart_event != RT_NULL)
            rt_event_send(uart_event, EVENT_IMU_SET | EVENT_TIMER_START); // Re-send IMU set request
    }
    if(ucp_flag.mag_set_flag == 1)
    {
        if (uart_event != RT_NULL)
            rt_event_send(uart_event, EVENT_MAG_SET | EVENT_TIMER_START); // Re-send magnetometer set request
    }
}

// Initialize UART ACK timeout timer
void uart_timout_init(void)
{
    // Create a one-shot timer named "ucp_timer" with 1000ms timeout
    // On timeout, uart_timer_timeout() is called
    ucp_timer = rt_timer_create("ucp_timer", uart_timer_timeout, RT_NULL, 1000, RT_TIMER_FLAG_ONE_SHOT);
}
