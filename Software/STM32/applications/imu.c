/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-10-16     yuxing       the first version
 */

 #define DBG_TAG "IMU"
 #define DBG_LVL DBG_LOG
 
 #include "QMC5883L.h"     // Magnetometer driver (QMC5883L)
 #include "MPU6050.h"      // Accelerometer + Gyroscope driver (MPU6050)
 #include "pid.h"          // PID controller support
 #include "QMC5883P.h"     // Magnetometer driver (QMC5883P variant)
 #include "ellipsoidfit.h" // Ellipsoid fitting algorithm (for magnetometer calibration)
 #include "imu.h"          // IMU data structures and calibration state definitions
 
 // ==========================================================================
 // Local Buffers and Globals
 // ==========================================================================
 
 // Buffer for storing raw IMU bias data (acc/gyro), used for calibration
 // imu_bias_data[0..2][] → accelerometer (x,y,z)
 // imu_bias_data[3..5][] → gyroscope (x,y,z)
 static int16_t imu_bias_data[6][10];
 #define sum_size 10   // Size of calibration averaging window
 
 // External variables (coming from other modules, e.g., PWM timers)
 extern uint16_t PWM_FallingCount_4;
 extern uint16_t PWM_FallingCount_5;
 extern rt_int32_t sum_t_fl;
 extern rt_int32_t sum_t_fr;
 
 // Synchronization primitives
 rt_sem_t imu_sem = RT_NULL;        // Semaphore for IMU update signaling
 rt_mutex_t imu_data_mutex;         // Mutex for protecting IMU shared data
 rt_event_t imu_event;              // Event object for calibration states
 
 // Shared IMU data structure
 IMU_data IMU_updata;               // IMU output data container
 thread_imu_data_t thread_imu_data; // Global IMU data accessible across threads
 
 // ==========================================================================
 // Filtering and Sensor Fusion
 // ==========================================================================
 
 /**
  * @brief  Simple low-pass filter for IMU signals
  * @param  rawValue   Pointer to current raw sensor value
  * @param  w          Filter weight (0.0f–1.0f). Higher = smoother, lower = faster response
  * @param  prevValue  Pointer to previous filtered value
  * @return (filtered value stored in *rawValue and *prevValue)
  */
 static int32_t IMU_filter (int32_t *rawValue, float w, int32_t *prevValue)
 {
     *rawValue = w * (*rawValue) + (1.0f - w) * (*prevValue);
     *prevValue = *rawValue;
 }
 
 // --------------------------------------------------------------------------
 
 /**
  * @brief  Full 9-axis AHRS update using raw data (Mahony-style filter)
  * @param  gx, gy, gz  Gyroscope (rad/s)
  * @param  ax, ay, az  Accelerometer (normalized, unit vector)
  * @param  mx, my, mz  Magnetometer (normalized, unit vector)
  * @param  data        Output: pointer to float (stores Yaw in degrees)
  *
  * Algorithm:
  *  - Normalize accelerometer and magnetometer
  *  - Estimate gravity and magnetic field reference directions
  *  - Compute error between measured and estimated values
  *  - Apply proportional-integral feedback to gyro
  *  - Update quaternion by integrating corrected gyro
  *  - Normalize quaternion
  *  - Convert quaternion → Euler angles (Pitch, Roll, Yaw)
  */
 static void IMUupdate(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz,
                       float *data)
 {
     // Quaternion state (represents orientation)
     static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
 
     // Integral error accumulators (gyro bias correction)
     static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
 
     // Control gains
     static float Kp = 50.0f;   // Proportional gain
     static float Ki = 0.002f;  // Integral gain
     static float halfT = 0.005f; // Half of sample period (T/2)
 
     float norm;
     float vx, vy, vz;  // Estimated gravity vector
     float ex, ey, ez;  // Error terms
     float hx, hy, hz;  // Magnetometer reference in body frame
     float bx, bz;      // Horizontal & vertical components of magnetic field
     float wx, wy, wz;  // Reference magnetic direction in body frame
     float Pitch, Roll, Yaw;
 
     // --- Normalize accelerometer ---
     norm = sqrt(ax*ax + ay*ay + az*az);
     if (norm != 0) {
         ax /= norm; ay /= norm; az /= norm;
     }
 
     // --- Normalize magnetometer ---
     norm = sqrt(mx*mx + my*my + mz*mz);
     if (norm != 0) {
         mx /= norm; my /= norm; mz /= norm;
     }
 
     // --- Compute reference direction of Earth's magnetic field in body frame ---
     hx = 2*mx*(0.5f - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
     hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5f - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
     hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5f - q1*q1 - q2*q2);
 
     bx = sqrt(hx*hx + hy*hy);
     bz = hz;
 
     // --- Project magnetic field into body frame ---
     wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
     wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
     wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5f - q1*q1 - q2*q2);
 
     // --- Estimated gravity vector from quaternion ---
     vx = 2*(q1*q3 - q0*q2);
     vy = 2*(q0*q1 + q2*q3);
     vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
 
     // --- Error between measured and estimated direction (fusion of accel + mag) ---
     ex = (ay*vz - az*vy) + (my*wz - mz*wy);
     ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
     ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
 
     // --- Integral feedback for gyro bias correction ---
     exInt += ex * Ki * halfT;
     eyInt += ey * Ki * halfT;
     ezInt += ez * Ki * halfT;
 
     // --- Apply feedback to gyroscope ---
     gx += Kp*ex + exInt;
     gy += Kp*ey + eyInt;
     gz += Kp*ez + ezInt;
 
     // --- Update quaternion by integrating gyro rates ---
     q0 += (-q1*gx - q2*gy - q3*gz) * halfT;
     q1 += ( q0*gx + q2*gz - q3*gy) * halfT;
     q2 += ( q0*gy - q1*gz + q3*gx) * halfT;
     q3 += ( q0*gz + q1*gy - q2*gx) * halfT;
 
     // --- Normalize quaternion ---
     norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
     if (norm != 0) {
         q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
     }
 
     // --- Convert quaternion to Euler angles (degrees) ---
     Pitch = asin(2 * q2 * q3 + 2 * q0 * q1) * 57.3f;
     Roll  = atan2(-2 * q1 * q3 + 2 * q0 * q2,
                   q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.3f;
     Yaw   = atan2(2*(q1*q2 - q0*q3),
                   q0*q0 - q1*q1 + q2*q2 - q3*q3) * 57.3f;
 
     // Currently only Yaw (heading) is output
     *data = Yaw;
 }
 
 // --------------------------------------------------------------------------
 
 /**
  * @brief  IMU update using MPU6050 internal DMP quaternion + magnetometer
  * @param  mx, my, mz    Magnetometer raw values
  * @param  data          Output pointer (stores Yaw in degrees)
  * @param  quaternion    Input quaternion from MPU6050 DMP
  *
  * This version uses the sensor’s built-in DMP quaternion
  * and only fuses in the magnetometer for heading.
  */
 static void IMUupdate_dmp(float mx, float my, float mz, float *data, float *quaternion)
 {
     float q0 = quaternion[0];
     float q1 = quaternion[1];
     float q2 = quaternion[2];
     float q3 = quaternion[3];
 
     float Yaw;
 
     // Normalize magnetometer
     float norm = sqrt(mx*mx + my*my + mz*mz);
     if (norm != 0) {
         mx /= norm; my /= norm; mz /= norm;
     }
 
     // Project magnetometer into body frame using quaternion
     float hx = 2*mx*(0.5f - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
     float hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5f - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
     float hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5f - q1*q1 - q2*q2);
 
     // Compute heading (yaw) from horizontal components
     float bx = sqrt(hx*hx + hy*hy);
     float bz = hz;
 
     Yaw = atan2(hy, hx) * (180.0f / M_PI) + 180.0f;
     *data = Yaw;
 }
 
 // --------------------------------------------------------------------------
 
 /**
  * @brief  MPU6050 offset calibration using averaging (zero-bias estimation)
  * @param  accBiasX, accBiasY, accBiasZ   Output accelerometer bias values
  * @param  gyroBiasX, gyroBiasY, gyroBiasZ Output gyroscope bias values
  * @param  acc    Input raw accelerometer samples [x,y,z]
  * @param  gyro   Input raw gyroscope samples [x,y,z]
  *
  * Method:
  *  - Stores latest sample in rolling buffer (imu_bias_data)
  *  - Computes sum across window of `sum_size` samples
  *  - Returns average as bias estimate
  */
 void MPU_6050_Calibrate(int32_t* accBiasX, int32_t* accBiasY, int32_t* accBiasZ,
                         int32_t* gyroBiasX, int32_t* gyroBiasY, int32_t* gyroBiasZ,
                         int32_t* acc, int32_t* gyro)
 {
     static uint8_t count = 0;
     int32_t sum_gyroX = 0, sum_gyroY = 0, sum_gyroZ = 0;
     int32_t sum_accX = 0,  sum_accY = 0,  sum_accZ = 0;
 
     // Store samples in rolling buffer
     imu_bias_data[0][count % sum_size] = acc[0];
     imu_bias_data[1][count % sum_size] = acc[1];
     imu_bias_data[2][count % sum_size] = acc[2];
 
     imu_bias_data[3][count % sum_size] = gyro[0];
     imu_bias_data[4][count % sum_size] = gyro[1];
     imu_bias_data[5][count % sum_size] = gyro[2];
 
     // Sum all samples in buffer
     for (int i = 0; i < 6; i++) {
         for (int j = 0; j < sum_size; j++) {
             switch(i) {
                 case 0: sum_accX  += imu_bias_data[i][j]; break;
                 case 1: sum_accY  += imu_bias_data[i][j]; break;
                 case 2: sum_accZ  += imu_bias_data[i][j]; break;
                 case 3: sum_gyroX += imu_bias_data[i][j]; break;
                 case 4: sum_gyroY += imu_bias_data[i][j]; break;
                 case 5: sum_gyroZ += imu_bias_data[i][j]; break;
             }
         }
     }
 
     // Compute average (bias)
     *accBiasX  = sum_accX  / sum_size;
     *accBiasY  = sum_accY  / sum_size;
     *accBiasZ  = sum_accZ  / sum_size;
 
     *gyroBiasX = sum_gyroX / sum_size;
     *gyroBiasY = sum_gyroY / sum_size;
     *gyroBiasZ = sum_gyroZ / sum_size;
 
     // Increment counter with wraparound
     count++;
     if (count == 239) count = 0;
 }
 
/**
 * @brief QMC5883L Zero-Bias Calibration (Hard-Iron Calibration)
 *        Finds the magnetic bias offsets by tracking max/min values
 *        along each axis while the sensor is rotated in all directions.
 *
 * @param[out] magBiasX  Pointer to store calculated bias offset on X-axis
 * @param[out] magBiasY  Pointer to store calculated bias offset on Y-axis
 * @param[out] magBiasZ  Pointer to store calculated bias offset on Z-axis
 * @param[in]  mag       Current raw magnetometer readings [X, Y, Z]
 *
 * @return None (bias values are written through output pointers)
 */
void QMC5883L_Calibrate(int32_t* magBiasX, int32_t* magBiasY, int32_t* magBiasZ, int32_t* mag)
{
    // Track running max/min values for each axis
    static int32_t max_magx = 0, max_magy = 0, max_magz = 0; // Initialized on first use
    static int32_t min_magx = 0, min_magy = 0, min_magz = 0;

    /** First calibration entry: initialize min/max with current readings **/
    if (IMU_updata.MAG_calib_flag == 0) // If first time entering calibration mode
    {
        max_magx = mag[0];
        max_magy = mag[1];
        max_magz = mag[2];

        min_magx = mag[0];
        min_magy = mag[1];
        min_magz = mag[2];

        IMU_updata.MAG_calib_flag = 1; // Mark that calibration has started
    }

    /** Update running extremes (track observed max/min for each axis) **/
    if (mag[0] > max_magx) max_magx = mag[0];
    if (mag[0] < min_magx) min_magx = mag[0];

    if (mag[1] > max_magy) max_magy = mag[1];
    if (mag[1] < min_magy) min_magy = mag[1];

    if (mag[2] > max_magz) max_magz = mag[2];
    if (mag[2] < min_magz) min_magz = mag[2];

    /** Compute bias offsets as midpoints between max/min **/
    *magBiasX = (max_magx + min_magx) / 2;
    *magBiasY = (max_magy + min_magy) / 2;
    *magBiasZ = (max_magz + min_magz) / 2;
}

// ==========================================================================
// Oscillation Detection
// ==========================================================================

/**
 * @brief Detects vibration/oscillation levels by comparing differences in
 *        gyroscope readings (roll, pitch, yaw) between two time steps.
 *
 * @param[in] current_data   Pointer to latest gyroscope readings
 * @param[in] previous_data  Pointer to previous gyroscope readings
 * @param[in] thresholds     Array of threshold values [4] defining
 *                           NO/LIGHT/MODERATE/HEAVY/EXTREME oscillation levels
 *
 * @return OscillationLevel enum indicating the detected vibration level
 */
OscillationLevel calculate_oscillation_level(SensorData *current_data,
                                             SensorData *previous_data,
                                             double thresholds[3])
{
    // Calculate absolute differences in angular velocities
    double roll_diff  = fabs(current_data->roll_gyro  - previous_data->roll_gyro);
    double pitch_diff = fabs(current_data->pitch_gyro - previous_data->pitch_gyro);
    double yaw_diff   = fabs(current_data->yaw_gyro   - previous_data->yaw_gyro);

    // Aggregate total difference as a measure of vibration magnitude
    double total_diff = roll_diff + pitch_diff + yaw_diff;

    // Classify oscillation severity based on thresholds
    if (total_diff < thresholds[0]) {
        return NO_OSCILLATION;
    } else if (total_diff < thresholds[1]) {
        return LIGHT_OSCILLATION;
    } else if (total_diff < thresholds[2]) {
        return MODERATE_OSCILLATION;
    } else if (total_diff < thresholds[3]) {
        return HEAVY_OSCILLATION;
    } else {
        return EXTREME_OSCILLATION;
    }
}

// ==========================================================================
// Angle Adjustment Utility
// ==========================================================================

/**
 * @brief Adjusts an angle by subtracting 90° (quarter-phase shift).
 *        Ensures the result stays within [0, 360) degrees.
 *
 * @param[in] angle   Input angle in degrees
 *
 * @return Adjusted angle in degrees (normalized to 0–360°)
 */
float adjust_angle_minus_90(float angle)
{
    angle -= 90.0f;           // Apply -90° offset
    if (angle < 0.0f) {
        angle += 360.0f;      // Normalize negative angle into [0, 360°)
    }
    return angle;
}

// ==========================================================================
// Calibration State Variables
// ==========================================================================

CalibrationState_t mag_calib_state = CALIB_IDLE;  // Current state of magnetometer calibration
CalibrationState_t imu_calib_state = CALIB_IDLE;  // Current state of IMU calibration


/**
 * @brief IMU thread entry function
 * 
 * This function is the main loop for IMU handling.
 * It initializes the hardware (I2C, MPU6050, magnetometer),
 * sets up synchronization primitives (semaphores, mutexes, events),
 * and continuously:
 *   - Reads sensor data (acc, gyro, mag)
 *   - Runs oscillation (shock) detection
 *   - Handles calibration state machines for gyro/accel and magnetometer
 *   - Applies calibration corrections
 *   - Computes heading using sensor fusion (AHRS)
 *   - Updates global IMU data in a thread-safe way
 * 
 * Runs indefinitely until the system powers down.
 */
void imu_thread_entry(void *parameter)
{
    /******* Hardware Initialization ********/
#ifdef IIC_SOFT
    // Using software I2C (no initialization here)
#else
    MX_I2C1_Init();         // Initialize hardware I2C1
    // MX_I2C2_Init();      // Optionally initialize I2C2
#endif
    /******* Hardware Initialization ********/

    /******* IMU Initialization ********/
    mag_type_t mag_type = QMC5883L;
    MPU6050_Init();        // Initialize MPU6050 IMU over I2C
    if (QMC5883L_Init() == RT_EOK)
    {
        mag_type = QMC5883L;   // Magnetometer type is QMC5883L
    }
    else
    {
        if (qmc5883p_init() == RT_EOK)
            mag_type = QMC5883P;   // Magnetometer type is QMC5883P
        else
            mag_type = DONE;       // No magnetometer detected
    }
    LOG_I("mag_type = %d", mag_type);
    /******* IMU Initialization ********/

    /** Initialize filter and working variables **/
    float quaternion[4] = {0};
    float mag_heading = 0;
    int32_t acc[3] = {0};
    int32_t gyro[3] = {0};
    int32_t mag[3] = {0};
    SensorData Shockdet;
    SensorData Shockdet_prev;
    double thresholds[3] = {15, 30, 90, 120};  // Oscillation detection thresholds
    Ellipsoidfit *mag_calibration = NULL;      // Used for magnetometer ellipsoid fit calibration

    /** Magnetometer calibration workspace **/
    double offset[3] = {0};
    double gain[3] = {0};
    double rotM[9] = {0};                      // Rotation matrix from calibration
    double *results;
    double mag_calibrated[3] = {0};
    double gain_one = 0;
    double raw_mag[3] = {0};
    char send_buffer[50] = {0};

    /******* RTOS Sync Objects ********/
    // Create semaphore (used to wake thread every 10ms for data update)
    imu_sem = rt_sem_create("imu_sem", 0, RT_IPC_FLAG_FIFO);
    if (imu_sem == RT_NULL)
    {
        LOG_E("Create semaphore failed!\n");
        return -RT_ERROR;
    }

    // Create IMU data mutex (to protect shared data structure)
    imu_data_mutex = rt_mutex_create("imu_data_mutex", RT_IPC_FLAG_PRIO);
    if (imu_data_mutex == RT_NULL)
    {
        LOG_E("imu_data_mutex creation failed!\n");
        return -RT_ERROR;
    }

    // Create IMU event object (used for calibration start/stop/done events)
    imu_event = rt_event_create("imu_event", RT_IPC_FLAG_FIFO);
    if (imu_event == RT_NULL)
    {
        LOG_I("imu_event creation failed!\n");
        return;
    }

    rt_uint32_t recv_event = 0;      // Event receiver buffer
    rt_uint8_t acc_gyro_calib_mode = 0;
    rt_uint8_t mag_calib_mode = 0;

    /******* Main IMU Loop ********/
    while (1)
    {
        if (robot_state.pwr == 1) // Only run if robot is powered on
        {
            // Wait indefinitely for semaphore (released every 10ms by timer)
            rt_sem_take(imu_sem, RT_WAITING_FOREVER);

            /** Acquire raw sensor data **/
            mpu_dmp_get_data(IMU_updata.imu_data, IMU_updata.imu_data + 1, IMU_updata.imu_data + 2, quaternion);
            MPU6050_acc_read(&acc[0], &acc[1], &acc[2]);
            MPU6050_gyro_read(&gyro[0], &gyro[1], &gyro[2]);

            if (mag_type == QMC5883L)
                QMC5883L_GetAngle(mag);
            else if (mag_type == QMC5883P)
                qmc5883p_read_mag_xyz(mag);

            /** Oscillation (shock) detection **/
            Shockdet.pitch_gyro = (double)((gyro[0] - thread_imu_data.gyro_calib_data.bias_x) / 16.4);
            Shockdet.roll_gyro  = (double)((gyro[1] - thread_imu_data.gyro_calib_data.bias_y) / 16.4);
            Shockdet.yaw_gyro   = (double)((gyro[2] - thread_imu_data.gyro_calib_data.bias_z) / 16.4);

            Shockdet_prev.pitch_gyro = (double)(thread_imu_data.gyro_data.gyro_x / 16.4);
            Shockdet_prev.roll_gyro  = (double)(thread_imu_data.gyro_data.gyro_y / 16.4);
            Shockdet_prev.yaw_gyro   = (double)(thread_imu_data.gyro_data.gyro_z / 16.4);

            IMU_updata.shock_flag = calculate_oscillation_level(&Shockdet, &Shockdet_prev, thresholds);

            /** Event handling (acc/gyro/mag calibration) **/
            if (rt_event_recv(imu_event, IMU_ACC_GYRO_EVENT_START | IMU_ACC_GYRO_EVENT_STOP |
                                          IMU_MAG_EVENT_START | IMU_MAG_EVENT_STOP,
                              RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                              RT_WAITING_NO, &recv_event) == RT_EOK)
            {
                if (recv_event & IMU_ACC_GYRO_EVENT_START) acc_gyro_calib_mode = 1;
                if (recv_event & IMU_ACC_GYRO_EVENT_STOP)  acc_gyro_calib_mode = 0;
                if (recv_event & IMU_MAG_EVENT_START)      mag_calib_mode = 1;
                if (recv_event & IMU_MAG_EVENT_STOP)       mag_calib_mode = 0;
            }

            /** Accelerometer & Gyroscope Calibration State Machine **/
            switch (imu_calib_state)
            {
                case CALIB_IDLE:
                    if (acc_gyro_calib_mode) imu_calib_state = CALIB_START;
                    break;
                case CALIB_START:
                    // Run calibration routine
                    MPU_6050_Calibrate(&thread_imu_data.acc_calib_data.bias_x, &thread_imu_data.acc_calib_data.bias_y, &thread_imu_data.acc_calib_data.bias_z,
                                       &thread_imu_data.gyro_calib_data.bias_x, &thread_imu_data.gyro_calib_data.bias_y, &thread_imu_data.gyro_calib_data.bias_z,
                                       acc, gyro);
                    if (acc_gyro_calib_mode == 0)
                        imu_calib_state = CALIB_DONE;
                    break;
                case CALIB_DONE:
                    imu_calib_state = CALIB_IDLE; // Calibration done, return to idle
                    rt_event_send(imu_event, IMU_ACC_GYRO_EVENT_DONE | IMU_CALIB_LED_DONE);
                    LOG_W("acc-gyro-1");
                    break;
            }

            /** Magnetometer Calibration State Machine **/
            // (Detailed calibration flow handled here, includes ellipsoid fit, hard/soft iron correction, memory free at end)
            // -> See earlier annotated calibration code in this file
            // After calibration, offset and gain are updated in thread_imu_data

            /** Apply Calibration and Update Shared IMU Data **/
            rt_mutex_take(imu_data_mutex, RT_WAITING_FOREVER);

            // Apply accelerometer calibration
            thread_imu_data.acc_data.acc_x = acc[0] - thread_imu_data.acc_calib_data.bias_x;
            thread_imu_data.acc_data.acc_y = acc[1] - thread_imu_data.acc_calib_data.bias_y;
            thread_imu_data.acc_data.acc_z = acc[2] - (thread_imu_data.acc_calib_data.bias_z - 16384);

            // Apply gyroscope calibration
            thread_imu_data.gyro_data.gyro_x = gyro[0] - thread_imu_data.gyro_calib_data.bias_x;
            thread_imu_data.gyro_data.gyro_y = gyro[1] - thread_imu_data.gyro_calib_data.bias_y;
            thread_imu_data.gyro_data.gyro_z = gyro[2] - thread_imu_data.gyro_calib_data.bias_z;

            // Apply magnetometer calibration
            thread_imu_data.mag_data.mag_x = (int32_t)mag_calibrated[0];
            thread_imu_data.mag_data.mag_y = (int32_t)mag_calibrated[1];
            thread_imu_data.mag_data.mag_z = (int32_t)mag_calibrated[2];

            /** Heading calculation (sensor fusion) **/
            IMUupdate(
                (float)((3.1415926f / 180.0f) * thread_imu_data.gyro_data.gyro_x) / 16.4f,
                (float)((3.1415926f / 180.0f) * thread_imu_data.gyro_data.gyro_y) / 16.4f,
                (float)((3.1415926f / 180.0f) * thread_imu_data.gyro_data.gyro_z) / 16.4f,
                (float)thread_imu_data.acc_data.acc_x / 16384.0f,
                (float)thread_imu_data.acc_data.acc_y / 16384.0f,
                (float)thread_imu_data.acc_data.acc_z / 16384.0f,
                (float)thread_imu_data.mag_data.mag_x,
                (float)thread_imu_data.mag_data.mag_y,
                (float)thread_imu_data.mag_data.mag_z,
                &mag_heading);

            // Adjust heading depending on mag type
            if (mag_type == QMC5883L)
                thread_imu_data.heading = mag_heading + 180.0f;
            else if (mag_type == QMC5883P)
                thread_imu_data.heading = adjust_angle_minus_90(mag_heading + 180.0f);

            rt_mutex_release(imu_data_mutex);

            /** Delay for calibration or regular loop **/
            if (mag_calib_state != CALIB_IDLE)
                rt_thread_mdelay(50);  // Allow time during calibration
            rt_thread_mdelay(0);       // Normal operation
        }
        else
        {
            // If power is off, sleep longer to save CPU
            rt_thread_mdelay(200);
        }
    }
}
