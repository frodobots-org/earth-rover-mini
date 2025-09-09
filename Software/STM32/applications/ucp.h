/*
 * Copyright 2024 Frodobots.ai. All rights reserved.
 */

// UART CONTROL PROTOCOL HEADER FILE
// Defines command IDs, error codes, message structures, and enums
// for communication over UART between devices (e.g., robot controller and host).

#ifndef __UART_CP_H__
#define __UART_CP_H__

#include <stdint.h>

/* =========================================================================
 * UART Command Protocol (UCP) Command Identifiers
 * Each command ID corresponds to a specific type of UART message.
 * ========================================================================= */
#define UCP_KEEP_ALIVE              (0X1)   // Heartbeat: keep-alive message
#define UCP_MOTOR_CTL               (0X2)   // Motor control command
#define UCP_IMU_CORRECTION_START    (0X3)   // Start IMU calibration/correction
#define UCP_IMU_CORRECTION_END      (0X4)   // End IMU calibration/correction
#define UCP_RPM_REPORT              (0X5)   // RPM and sensor data report
#define UCP_IMU_WRITE               (0X6)   // Write IMU calibration parameters
#define UCP_MAG_WRITE               (0X7)   // Write magnetometer calibration parameters
#define UCP_IMUMAG_READ             (0X8)   // Read IMU and magnetometer calibration values
#define UCP_OTA                     (0X9)   // Over-the-Air update request
#define UCP_STATE                   (0XA)   // Device state report

#pragma pack(push, 1)  // 1-byte alignment for all structures (no padding)

/* =========================================================================
 * Error Codes
 * ========================================================================= */
typedef enum ucp_err {
    UCP_ERR_OK = 0,         // No error
    UCP_ERR_UNKNOWN = 1,    // Unknown error
    UCP_ERR_OTA_FORBIDDEN,  // OTA request rejected / forbidden
    UCP_ERR_MAX             // Maximum enum value (marker)
} ucp_err_e;

/* =========================================================================
 * IMU Correction Types
 * ========================================================================= */
typedef enum ucp_imu_correction_type {
    UICT_MAG = 1,   // Magnetometer calibration
    UICT_IMU = 2    // IMU (accelerometer/gyroscope) calibration
} uict_e;

/* =========================================================================
 * UART Protocol Message Structures
 * ========================================================================= */

/* Message header (present in every packet) */
typedef struct ucp_hd {
    uint16_t    len;    // Length of the message body (not including header)
    uint8_t     id;     // Message ID (one of the UCP_* defines)
    uint8_t     index;  // Sequence index (helps with tracking/retransmission)
} ucp_hd_t __attribute__((packed));

/* Keep-alive ping message (empty body) */
typedef struct ucp_alive_ping {
    ucp_hd_t    hd;
} ucp_alive_ping_t __attribute__((packed));

/* Keep-alive pong (reply) message */
typedef struct ucp_alive_pong {
    ucp_hd_t    hd;
    uint8_t     err;    // Error code if any
} ucp_alive_pong_t __attribute__((packed));

/* Motor control command */
typedef struct ucp_ctl_cmd {
    ucp_hd_t    hd;
    int16_t     speed;      // Linear speed
    int16_t     angular;    // Angular velocity
    int16_t     front_led;  // Front LED control
    int16_t     back_led;   // Back LED control
    uint16_t    version;    // Protocol or firmware version
    uint16_t    reserve1;   // Reserved for future use
    uint32_t    reserve2;   // Reserved for future use
} ucp_ctl_cmd_t __attribute__((packed));

/* IMU correction start/stop request */
typedef struct ucp_imu_correct {
    ucp_hd_t    hd;
    uint8_t     type;   // Type of correction (UICT_MAG or UICT_IMU)
} ucp_imu_correct_t __attribute__((packed));

/* IMU correction acknowledgment */
typedef struct ucp_imu_correct_ack {
    ucp_hd_t    hd;
    uint8_t     type;   // Correction type
    uint8_t     err;    // Error code
} ucp_imu_correct_ack_t __attribute__((packed));

/* Report message containing telemetry (status and sensor data) */
typedef struct ucp_rep { // report
    ucp_hd_t    hd;
    uint16_t    voltage;        // Battery/system voltage
    int16_t     rpm[4];         // Motor RPM for 4 wheels
    int16_t     acc[3];         // Accelerometer (x,y,z)
    int16_t     gyros[3];       // Gyroscope (x,y,z)
    int16_t     mag[3];         // Magnetometer (x,y,z)
    int16_t     heading;        // Heading angle (from IMU/mag)
    uint8_t     stop_switch;    // Emergency stop switch status
    uint8_t     error_code;     // Error code for system state
    uint16_t    reserve;        // Reserved
    uint16_t    version;        // Firmware/protocol version
} ucp_rep_t __attribute__((packed));

/* Magnetometer write request */
typedef struct ucp_mag_w {
    ucp_hd_t    hd;
    uint16_t    mag_bias_x;     // Magnetometer bias (x)
    uint16_t    mag_bias_y;     // Magnetometer bias (y)
    uint16_t    mag_bias_z;     // Magnetometer bias (z)
} ucp_mag_w_t __attribute__((packed));

/* Magnetometer write acknowledgment */
typedef struct ucp_mag_w_ack {
    ucp_hd_t    hd;
    uint8_t     err;    // Error code
} ucp_mag_w_ack_t __attribute__((packed));

/* IMU write request */
typedef struct ucp_imu_w {
    ucp_hd_t    hd;
    uint16_t    acc_bias_x;     // Accelerometer bias (x)
    uint16_t    acc_bias_y;     // Accelerometer bias (y)
    uint16_t    acc_bias_z;     // Accelerometer bias (z)
    uint16_t    gyro_bias_x;    // Gyroscope bias (x)
    uint16_t    gyro_bias_y;    // Gyroscope bias (y)
    uint16_t    gyro_bias_z;    // Gyroscope bias (z)
} ucp_imu_w_t __attribute__((packed));

/* IMU write acknowledgment */
typedef struct ucp_imu_w_ack {
    ucp_hd_t    hd;
    uint8_t     err;    // Error code
} ucp_imu_w_ack_t __attribute__((packed));

/* IMU read request (no body needed) */
typedef struct ucp_imu_r {
    ucp_hd_t    hd;
} ucp_imu_r_t __attribute__((packed));

/* IMU read acknowledgment (returns calibration parameters) */
typedef struct ucp_imu_r_ack {
    ucp_hd_t    hd;
    uint8_t     err;            // Error code
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

/* OTA (Over-the-Air update) request */
typedef struct ucp_ota {
    ucp_hd_t    hd;
    int16_t     version;    // Requested OTA version
} ucp_ota_t;

/* OTA acknowledgment */
typedef struct ucp_ota_ack {
    ucp_hd_t    hd;
    uint8_t     err;        // Error code
} ucp_ota_ack_t;

/* Device state enumeration */
typedef enum ucp_state_e {
    UCP_STATE_UNKNOWN = 0,      // Unknown state
    UCP_STATE_SIMABSENT = 1,    // SIM card absent
    UCP_NETWORK_DISCONNECTED,   // No network connection
    UCP_NETWORK_CONNECTED,      // Network connected
    UCP_OTA_ING                 // OTA update in progress
} ucp_state_e;

#pragma pack(pop)  // Restore default struct alignment
#endif /*__UART_CP_H__*/
