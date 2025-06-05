/*
 * Copyright 2024 Frodobots.ai. All rights reserved.
 */

//UART CONTROL PROCOTOL

#ifndef __UART_CP_H__
#define __UART_CP_H__

#include <stdint.h>

#define UCP_KEEP_ALIVE          (0X1)
#define UCP_MOTOR_CTL           (0X2)
#define UCP_IMU_CORRECTION_START    (0X3)
#define UCP_IMU_CORRECTION_END      (0X4)
#define UCP_RPM_REPORT          (0X5)
#define UCP_IMU_WRITE           (0X6)
#define UCP_MAG_WRITE           (0X7)
#define UCP_IMUMAG_READ         (0X8)
#define UCP_OTA             (0X9)
#define UCP_STATE           (0XA)

#pragma pack(push, 1)  // 1字节对齐
typedef enum ucp_err {
    UCP_ERR_OK = 0,
    UCP_ERR_UNKNOWN = 1,
    UCP_ERR_OTA_FORBIDDEN,
    UCP_ERR_MAX
} ucp_err_e;

typedef enum ucp_imu_correction_type {
    UICT_MAG = 1,
    UICT_IMU = 2
} uict_e;

typedef struct ucp_hd {
    uint16_t    len; // the length of the body.
    uint8_t     id;
    uint8_t     index;
} ucp_hd_t __attribute__((packed));

typedef struct ucp_alive_ping {
    ucp_hd_t    hd;
} ucp_alive_ping_t __attribute__((packed));

typedef struct ucp_alive_pong {
    ucp_hd_t    hd;
    uint8_t     err;
} ucp_alive_pong_t __attribute__((packed));

typedef struct ucp_ctl_cmd {
    ucp_hd_t    hd;
    int16_t     speed;
    int16_t     angular;
    int16_t     front_led;
    int16_t     back_led;
    uint16_t    version;
    uint16_t    reserve1;
    uint32_t    reserve2;
} ucp_ctl_cmd_t __attribute__((packed));

typedef struct ucp_imu_correct {
    ucp_hd_t    hd;
    uint8_t     type;
} ucp_imu_correct_t __attribute__((packed));

typedef struct ucp_imu_correct_ack {
    ucp_hd_t    hd;
    uint8_t     type;
    uint8_t     err;
} ucp_imu_correct_ack_t __attribute__((packed));

typedef struct ucp_rep { //report
    ucp_hd_t    hd;
    uint16_t    voltage;
    int16_t     rpm[4];
    int16_t     acc[3];
    int16_t     gyros[3];
    int16_t     mag[3];
    int16_t     heading;
    uint8_t     stop_switch;
    uint8_t     error_code;
    uint16_t    reserve;
    uint16_t    version;
} ucp_rep_t __attribute__((packed));

typedef struct ucp_mag_w {
    ucp_hd_t    hd;
    uint16_t    mag_bias_x;
    uint16_t    mag_bias_y;
    uint16_t    mag_bias_z;
} ucp_mag_w_t __attribute__((packed));

typedef struct ucp_mag_w_ack {
    ucp_hd_t    hd;
    uint8_t     err;
} ucp_mag_w_ack_t __attribute__((packed));

typedef struct ucp_imu_w {
    ucp_hd_t    hd;
    uint16_t    acc_bias_x;
    uint16_t    acc_bias_y;
    uint16_t    acc_bias_z;
    uint16_t    gyro_bias_x;
    uint16_t    gyro_bias_y;
    uint16_t    gyro_bias_z;
} ucp_imu_w_t __attribute__((packed));

typedef struct ucp_imu_w_ack {
    ucp_hd_t    hd;
    uint8_t     err;
} ucp_imu_w_ack_t __attribute__((packed));

typedef struct ucp_imu_r {
    ucp_hd_t    hd;
} ucp_imu_r_t __attribute__((packed));

typedef struct ucp_imu_r_ack {
    ucp_hd_t    hd;
    uint8_t     err;
    uint16_t    acc_bias_x;
    uint16_t    acc_bias_y;
    uint16_t    acc_bias_z;
    uint16_t    gyro_bias_x;
    uint16_t    gyro_bias_y;
    uint16_t    gyro_bias_z;
    uint16_t    mag_bias_x;
    uint16_t    mag_bias_y;
    uint16_t    mag_bias_z;
} ucp_imu_r_ack_t __attribute__((packed));

typedef struct ucp_ota {
    ucp_hd_t    hd;
    int16_t     version;
} ucp_ota_t;

typedef struct ucp_ota_ack {
    ucp_hd_t    hd;
    uint8_t err;
} ucp_ota_ack_t;

typedef enum ucp_state_e {
        UCP_STATE_UNKNOWN = 0,
        UCP_STATE_SIMABSENT = 1,
        UCP_NETWORK_DISCONNECTED,
        UCP_NETWORK_CONNECTED,
        UCP_OTA_ING
} ucp_state_e;

#pragma pack(pop)
#endif /*__UART_CP_H__*/
