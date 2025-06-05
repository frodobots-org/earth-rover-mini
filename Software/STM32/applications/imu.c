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
#include "QMC5883L.h"
#include "MPU6050.h"
#include "pid.h"
#include "QMC5883P.h"
#include "ellipsoidfit.h"
#include "imu.h"


static int16_t imu_bias_data[6][10];
#define sum_size 10

extern uint16_t PWM_FallingCount_4;
extern uint16_t PWM_FallingCount_5;

rt_sem_t imu_sem = RT_NULL;
IMU_data IMU_updata;
extern rt_int32_t sum_t_fl;
extern rt_int32_t sum_t_fr;

rt_mutex_t imu_data_mutex;
thread_imu_data_t thread_imu_data;
rt_event_t imu_event;

//#define Kp 100.0f                        // 比例增益支配率收敛到加速度计/磁强计
//#define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
//#define halfT 0.005f                // 采样周期的一半
/**
 * brief:IMU低通滤波
 * parameter:3
 * return:data
 */
static int32_t IMU_filter ( int32_t *rawValue , float w , int32_t *prevValue )
{
      * rawValue = w * (* rawValue) + (1.0 - w) * (* prevValue);
      * prevValue = * rawValue;
}
/**
 * brief:IMU九轴姿态解算(原始数据)
 * parameter:陀螺仪数据(弧度每秒);加速度数据(归一化);磁力计数据(归一化)
 * return:data
 */
static void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float mx, float my, float mz, float *data)
{
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
    float Pitch, Roll, Yaw;
    static float Kp = 50.0f, Ki = 0.002f;
    static float halfT = 0.005f;

    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float hx, hy, hz, bx, bz;
    float wx, wy, wz;
    // 测量正常化, 把加速度计的三维向量转成单位向量
    norm = sqrt(ax*ax + ay*ay + az*az);  // 计算向量的模
    if(norm != 0)//nan(0x400000)
    {
        ax = ax / norm;  // 单位化
        ay = ay / norm;
        az = az / norm;
    }

    norm = sqrt(mx*mx + my*my + mz*mz);
    if(norm != 0)
    {
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;
    }

    // 这里计算得到的是地磁计在理论地磁坐标系下的机体上三个轴的分量
    hx = 2*mx*(0.5 - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
    hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5 - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
    hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5 - q1*q1 - q2*q2);

    //bx计算的是当前航向角和磁北的夹角，也就是北天东坐标下的航向角
    //当罗盘水平旋转的时候，航向角在0-360之间变化
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    //地磁计在n系(地球参考系)下磁向量转换到b系(机体坐标系)下，反向使用DCM得到
    wx = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2);

    // 估计方向的重力向量
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 计算加速度计测量值与估计重力方向之间的误差向量
//    ex = (ay*vz - az*vy);
//    ey = (az*vx - ax*vz);
//    ez = (ax*vy - ay*vx);

    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    // 积分误差比例积分增益
    exInt = exInt + ex*Ki*halfT;
    eyInt = eyInt + ey*Ki*halfT;
    ezInt = ezInt + ez*Ki*halfT;

    // 调整后的陀螺仪测量值
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    // 整合四元数率和正常化
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // 正常化四元数
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(norm != 0)
    {
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
    }

    // 计算姿态角
    Pitch = asin(2 * q2 * q3 + 2 * q0 * q1) * 57.3;  // pitch ,转换为度数
    Roll = atan2(-2 * q1 * q3 + 2 * q0 * q2, q0*q0-q1*q1-q2*q2+q3*q3) * 57.3;  // roll
    Yaw = atan2(2*(q1*q2 - q0*q3), q0*q0-q1*q1+q2*q2-q3*q3) * 57.3;  // yaw

//    data[0] = Pitch;
//    data[1] = Roll;
    *data = Yaw;

}
/**
 * brief:IMU九轴姿态解算(mpu6050四元数+磁力计原始数据)
 * parameter:无
 * return:data
 */
static void IMUupdate_dmp(float mx, float my, float mz, float *data, float *quaternion)
{
    // 从传入的四元数数据
    float q0 = quaternion[0];
    float q1 = quaternion[1];
    float q2 = quaternion[2];
    float q3 = quaternion[3];

    float Yaw;

    // 磁力计数据归一化
    float norm = sqrt(mx * mx + my * my + mz * mz);
    if(norm != 0)
    {
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;
    }

    // 地磁计在机体坐标系下的分量
    float hx = 2 * mx * (0.5f - q2 * q2 - q3 * q3) + 2 * my * (q1 * q2 - q0 * q3) + 2 * mz * (q1 * q3 + q0 * q2);
    float hy = 2 * mx * (q1 * q2 + q0 * q3) + 2 * my * (0.5f - q1 * q1 - q3 * q3) + 2 * mz * (q2 * q3 - q0 * q1);
    float hz = 2 * mx * (q1 * q3 - q0 * q2) + 2 * my * (q2 * q3 + q0 * q1) + 2 * mz * (0.5f - q1 * q1 - q2 * q2);

    // 计算航向角
    float bx = sqrt(hx * hx + hy * hy);
    float bz = hz;

    Yaw = atan2(hy, hx) * (180.0f / M_PI)+180.0f;  // 将结果转换为度
    *data = Yaw;
//    // 返回姿态角
//    data[0] = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (180.0 / M_PI);  // Pitch
//    data[1] = asin(-2.0f * (q2 * q0 - q1 * q3)) * (180.0 / M_PI);  // Roll
//    data[2] = (int16_t)Yaw;  // Yaw

}
/**
 * brief:MPU6050零偏校准(利用保存的数据窗口求平均)
 * parameter:无
 * return:data
 */
void MPU_6050_Calibrate(int32_t* accBiasX, int32_t* accBiasY, int32_t* accBiasZ,int32_t* gyroBiasX, int32_t* gyroBiasY, int32_t* gyroBiasZ, int32_t* acc, int32_t* gyro)
{
    static uint8_t count = 0;
    int32_t sum_gyroX = 0, sum_gyroY = 0, sum_gyroZ = 0, sum_accX = 0, sum_accY = 0, sum_accZ = 0;

    imu_bias_data[0][count % sum_size] = acc[0];
    imu_bias_data[1][count % sum_size] = acc[1];
    imu_bias_data[2][count % sum_size] = acc[2];

    imu_bias_data[3][count % sum_size] = gyro[0];
    imu_bias_data[4][count % sum_size] = gyro[1];
    imu_bias_data[5][count % sum_size] = gyro[2];

    for(int i = 0;i < 6;i++)
    {
        for(int j = 0;j < sum_size ;j++)
        {
            switch(i)
            {
            case 0: sum_accX += imu_bias_data[i][j];break;
            case 1: sum_accY += imu_bias_data[i][j];break;
            case 2: sum_accZ += imu_bias_data[i][j];break;
            case 3: sum_gyroX += imu_bias_data[i][j];break;
            case 4: sum_gyroY += imu_bias_data[i][j];break;
            case 5: sum_gyroZ += imu_bias_data[i][j];break;

            }
        }
    }

    *accBiasX = sum_accX / sum_size;
    *accBiasY = sum_accY / sum_size;
    *accBiasZ = sum_accZ / sum_size;

    *gyroBiasX = sum_gyroX / sum_size;
    *gyroBiasY = sum_gyroY / sum_size;
    *gyroBiasZ = sum_gyroZ / sum_size;

    count ++;
    if(count == 239)
        count = 0;

}
/**
 * brief:QMC5883L零偏校准(硬磁校准)
 * parameter:无
 * return:data
 */
void QMC5883L_Calibrate(int32_t* magBiasX, int32_t* magBiasY, int32_t* magBiasZ, int32_t* mag)
{
    static int32_t max_magx = 0,max_magy = 0,max_magz = 0;//初始化
    static int32_t min_magx = 0,min_magy = 0,min_magz = 0;

    /**更新第一次校准得到的数据**/
    if(IMU_updata.MAG_calib_flag == 0)//第一次进入校准模式先获取一次数据
    {
        max_magx = mag[0];
        max_magy = mag[1];
        max_magz = mag[2];

        min_magx = mag[0];
        min_magy = mag[1];
        min_magz = mag[2];

        IMU_updata.MAG_calib_flag = 1;
    }
    /**更新磁力计三轴数据的极值**/
    if(mag[0] > max_magx)
    {
        max_magx = mag[0];
    }
    if(mag[0] < min_magx)
    {
        min_magx = mag[0];
    }

    if(mag[1] > max_magy)
    {
        max_magy = mag[1];
    }
    if(mag[1] < min_magy)
    {
        min_magy = mag[1];
    }

    if(mag[2] > max_magz)
    {
        max_magz = mag[2];
    }
    if(mag[2] < min_magz)
    {
        min_magz = mag[2];
    }

    *magBiasX = (max_magx + min_magx)/2;
    *magBiasY = (max_magy + min_magy)/2;
    *magBiasZ = (max_magz + min_magz)/2;

}

// 震荡检测函数
OscillationLevel calculate_oscillation_level(SensorData *current_data, SensorData *previous_data, double thresholds[3]) {
    // 计算加速度差值和角速度差值
    double roll_diff = fabs(current_data->roll_gyro - previous_data->roll_gyro);
    double pitch_diff = fabs(current_data->pitch_gyro - previous_data->pitch_gyro);
    double yaw_diff = fabs(current_data->yaw_gyro - previous_data->yaw_gyro);

    // 计算震荡总幅度
    double total_diff = roll_diff + pitch_diff + yaw_diff;

    // 根据差值判断震荡等级
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

// 对角度进行90°相位偏移 (减90°)
float adjust_angle_minus_90(float angle) {
    angle -= 90.0f;  // 减去90度相位偏移
    if (angle < 0.0f) {
        angle += 360.0f;  // 负角度归一化到0~360°
    }
    return angle;
}
CalibrationState_t mag_calib_state = CALIB_IDLE;
CalibrationState_t imu_calib_state = CALIB_IDLE;
void imu_thread_entry ( void *parameter )
{
    /*******硬件初始化********/
#ifdef IIC_SOFT

#else
    MX_I2C1_Init();         //硬件IIC初始化
//    MX_I2C2_Init();
#endif
    /*******硬件初始化********/

    /*******IMU初始化********/
    mag_type_t mag_type = QMC5883L;
    MPU6050_Init();        //软件IIC初始化
    if(QMC5883L_Init() == RT_EOK)
    {
        mag_type = QMC5883L;
    }
    else
    {
        if(qmc5883p_init() == RT_EOK)
        mag_type = QMC5883P;
        else {
            mag_type = DONE;
        }
    }
    LOG_I("mag_type = %d",mag_type);
    /*******IMU初始化********/

    /**初始化滑动滤波**/
    float quaternion[4] = {0};
    float mag_heading = 0;
    int32_t acc[3] = {0};
    int32_t gyro[3] = {0};
    int32_t mag[3] = {0};
    SensorData Shockdet;
    SensorData Shockdet_prev;
    double thresholds[3] = {15,30,90,120};
//    CalibrationData *mag_calibration = NULL;
    Ellipsoidfit *mag_calibration = NULL;

    /**mag calibration**/
    double offset[3]={0};
    double gain[3] = {0};
    double rotM[9] = {0};
    double *results;
    double mag_calibrated[3] = {0};
    double gain_one = 0;
    double raw_mag[3] = {0};
    char send_buffer[50] = {0};
    /*******IMU初始化********/
    // 创建信号量
    imu_sem = rt_sem_create("imu_sem", 0, RT_IPC_FLAG_FIFO);
    if (imu_sem == RT_NULL)
    {
        LOG_E("Create semaphore failed!\n");
        return -RT_ERROR;
    }

    // 创建imu_data互斥锁
    imu_data_mutex = rt_mutex_create("imu_data_mutex", RT_IPC_FLAG_PRIO);
    if (imu_data_mutex == RT_NULL)
    {
        LOG_E("imu_data_mutex creation failed!\n");
        return -RT_ERROR;
    }
    //创建IMU事件
    imu_event = rt_event_create("imu_event", RT_IPC_FLAG_FIFO);
    if (imu_event == RT_NULL)
    {
        LOG_I("imu_event creation failed!\n");
        return;
    }
    rt_uint32_t recv_event = 0;
    rt_uint8_t acc_gyro_calib_mode = 0;
    rt_uint8_t mag_calib_mode = 0;
    while(1)
    {
        if( robot_state.pwr == 1 )
        {
        // 等待信号量，阻塞直到定时器释放信号量(10ms更新一次数据)
        rt_sem_take(imu_sem, RT_WAITING_FOREVER);

        /**数据获取**/
        mpu_dmp_get_data(IMU_updata.imu_data,IMU_updata.imu_data + 1,IMU_updata.imu_data + 2,quaternion);
        MPU6050_acc_read(&acc[0] , &acc[1] , &acc[2]);
        MPU6050_gyro_read(&gyro[0] , &gyro[1] , &gyro[2]);

        if(mag_type == QMC5883L)
        QMC5883L_GetAngle(mag);
        else if(mag_type == QMC5883P)
        qmc5883p_read_mag_xyz(mag);

        /**震荡检测**/
        Shockdet.pitch_gyro = (double)((gyro[0] - thread_imu_data.gyro_calib_data.bias_x) / 16.4);
        Shockdet.roll_gyro = (double)((gyro[1] - thread_imu_data.gyro_calib_data.bias_y) / 16.4);
        Shockdet.yaw_gyro = (double)((gyro[2] - thread_imu_data.gyro_calib_data.bias_z) / 16.4);
        Shockdet_prev.pitch_gyro = (double)(thread_imu_data.gyro_data.gyro_x / 16.4);
        Shockdet_prev.roll_gyro = (double)(thread_imu_data.gyro_data.gyro_y / 16.4);
        Shockdet_prev.yaw_gyro = (double)(thread_imu_data.gyro_data.gyro_z / 16.4);
        IMU_updata.shock_flag = calculate_oscillation_level( &Shockdet, &Shockdet_prev, thresholds );
        /**震荡检测**/

        //imu事件检测
        if (rt_event_recv(imu_event, IMU_ACC_GYRO_EVENT_START | IMU_ACC_GYRO_EVENT_STOP
                            | IMU_MAG_EVENT_START | IMU_MAG_EVENT_STOP,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            RT_WAITING_NO, &recv_event) == RT_EOK)
        {
            if(recv_event & IMU_ACC_GYRO_EVENT_START)
            {
                acc_gyro_calib_mode = 1;
            }
            if(recv_event & IMU_ACC_GYRO_EVENT_STOP)
            {
                acc_gyro_calib_mode = 0;

            }
            if(recv_event & IMU_MAG_EVENT_START)
            {
                mag_calib_mode = 1;
            }
            if(recv_event & IMU_MAG_EVENT_STOP)
            {
                mag_calib_mode = 0;
            }
        }
        /**加速度&陀螺仪校准**/
        switch (imu_calib_state)
        {
            case CALIB_IDLE:
                if(acc_gyro_calib_mode)
                {
                    imu_calib_state = CALIB_START;
                }
                break;
            case CALIB_START:
                MPU_6050_Calibrate(&thread_imu_data.acc_calib_data.bias_x,&thread_imu_data.acc_calib_data.bias_y,&thread_imu_data.acc_calib_data.bias_z,
                        &thread_imu_data.gyro_calib_data.bias_x,&thread_imu_data.gyro_calib_data.bias_y,&thread_imu_data.gyro_calib_data.bias_z,acc,gyro);
                if(acc_gyro_calib_mode == 0)
                {
                    imu_calib_state = CALIB_DONE;
                }
                break;
            case CALIB_DONE:
                imu_calib_state = CALIB_IDLE;//校准完成，回到空闲状态
                rt_event_send(imu_event, IMU_ACC_GYRO_EVENT_DONE | IMU_CALIB_LED_DONE);
                LOG_W("acc-gyro-1");
                break;
        }
        /**加速度&陀螺仪校准**/

        /**磁力计校准**/
#if 0
        if(IMU_updata.MAG_calib_mode)//
        {
            QMC5883L_Calibrate(&IMU_updata.mag_bias[0],&IMU_updata.mag_bias[1],&IMU_updata.mag_bias[2],mag);
        }
        else {
            IMU_updata.MAG_calib_flag = 0;//校准结束
        }
#else
        raw_mag[0] = (double)mag[0];
        raw_mag[1] = (double)mag[1];
        raw_mag[2] = (double)mag[2];
        offset[0] = (double)thread_imu_data.mag_calib_data.offset_x;
        offset[1] = (double)thread_imu_data.mag_calib_data.offset_y;
        offset[2] = (double)thread_imu_data.mag_calib_data.offset_z;
        gain[0] = (double)thread_imu_data.mag_calib_data.gain_x/10000;
        gain[1] = (double)thread_imu_data.mag_calib_data.gain_y/10000;
        gain[2] = (double)thread_imu_data.mag_calib_data.gain_z/10000;
        handle_calibrate_magnetometer(raw_mag,mag_calibrated,offset,gain,rotM);
        switch (mag_calib_state)
        {
        case CALIB_IDLE:
            // 等待校准信号
            if (mag_calib_mode)//IMU_updata.MAG_calib_mode
            {
                mag_calib_state = CALIB_START;
                LOG_W("1.磁力计校准开始！\n");
            }
            else {
//                    calibrate_sensor(raw_mag,mag_calibrated,offset,gain,rotM);//不采用旋转对齐，禁止使用
                //LOG
//                    snprintf(send_buffer,sizeof(send_buffer) - 1,"%d,%d,%d\r\n", (int32_t)mag_calibrated[0], (int32_t)mag_calibrated[1], (int32_t)mag_calibrated[2]);
//                    rt_kprintf("%s",send_buffer);
            }
            break;

        case CALIB_START:
            mag_calibration = create_ellipsoidfit_data(200);
            if(mag_calibration != NULL)
            {
                mag_calib_state = CALIB_DATA_COLLECT;
                LOG_W("2.内存分配成功！\n");
            }
            else {
                mag_calib_state = CALIB_IDLE; // 申请内存失败，回到空闲状态
                LOG_W("2.内存分配失败，回到空闲状态！\n");
            }

            break;

        case CALIB_DATA_COLLECT:
            mag_calibration->rawdata[mag_calibration->num * 3 + 0] = (float)mag[0];
            mag_calibration->rawdata[mag_calibration->num * 3 + 1] = (float)mag[1];
            mag_calibration->rawdata[mag_calibration->num * 3 + 2] = (float)mag[2];
            LOG_W("[%d]: %f %f %f\n", mag_calibration->num, (float)mag_calibration->rawdata[mag_calibration->num * 3 + 0], (float)mag_calibration->rawdata[mag_calibration->num * 3 + 1],
                    (float)mag_calibration->rawdata[mag_calibration->num * 3 + 2]);
            mag_calibration->num ++;
            if(mag_calibration->num >= mag_calibration->nsamples)
            {
                mag_calibration->num = 0;
                mag_calibration->full_flag = 1;
            }

            if(mag_calib_mode == 0)
            {
                mag_calib_state = CALIB_HARD_START;
                LOG_W("3.数据采集完成，磁力计校准结束！\n");
                LOG_W("4.开始解算硬磁数据！\n");
            }
            break;

        case CALIB_HARD_START:
            if(0)//判断极值校准还是椭球拟合校准 FIXME
            {
                //极值校准
                ellipsoidfit_hard_iron_offset(mag_calibration,mag);
                LOG_W("5.硬磁校准解算完成！\n");
                thread_imu_data.mag_calib_data.offset_x = (int32_t)mag[0];
                thread_imu_data.mag_calib_data.offset_y = (int32_t)mag[1];
                thread_imu_data.mag_calib_data.offset_z = (int32_t)mag[2];
                LOG_W("6.硬磁校准解算数据(极值算法)：%d , %d , %d\n",mag[0],mag[1],mag[2]);
                mag_calib_state = CALIB_DONE;
            }
            else
            {
                //椭球拟合校准
                mag_calib_state = CALIB_SOFT_START;
                LOG_W("7.开始解算软磁数据！\n");
            }
            break;

        case CALIB_SOFT_START:
            results = ellipsoid_fit(mag_calibration);
            for(int i = 0 ; i < 3; i++)
            {
                offset[i] = *(results + i);
                LOG_W("->offset[%d] = %f\n",i,(float)offset[i]);
            }

            for(int i = 3; i <6; i++)
            {
                gain[i-3] =*(results + i);
                LOG_W("->gain[%d] = %f\n",i-3,(float)gain[i-3]);
            }

            for(int i = 6; i <15; i++)
            {
                rotM[i-6] =*(results + i);
                LOG_W("->rotM[%d] = %f\n",i-6,(float)rotM[i-6]);
            }

//            refine_3D_fit(&gain,&rotM);//不采用旋转对齐，禁止使用

            for(int i = 3; i <6; i++)
            {
                LOG_W("->->gain[%d] = %f\n",i-3,(float)gain[i-3]);
            }

            for(int i = 6; i <15; i++)
            {
                LOG_W("->->rotM[%d] = %f\n",i-6,(float)rotM[i-6]);
            }

//            gain_one = sqrtf(gain[0]*gain[0] + gain[1]*gain[1] +gain[2]*gain[2]);//缩放因子归一化
            for(int i = 0; i <3; i++)
            {
                if(gain[i] > gain_one)
                    gain_one = gain[i];
            }
            for(int i = 0; i <3; i++)
            {
                gain[i] = gain[i] /gain_one;//缩放因子归一化
                LOG_W("->->->gain[%d] = %f\n",i,(float)gain[i]);
            }
            LOG_W("7.软磁数据解算完成！\n");
            LOG_W("8.软磁数据解算矩阵W：\n");

            thread_imu_data.mag_calib_data.offset_x = (int32_t)offset[0];
            thread_imu_data.mag_calib_data.offset_y = (int32_t)offset[1];
            thread_imu_data.mag_calib_data.offset_z = (int32_t)offset[2];
            thread_imu_data.mag_calib_data.gain_x = (int32_t)(gain[0]*10000);
            thread_imu_data.mag_calib_data.gain_y = (int32_t)(gain[1]*10000);
            thread_imu_data.mag_calib_data.gain_z = (int32_t)(gain[2]*10000);

            mag_calib_state = CALIB_DONE;

            break;
        case CALIB_DONE:
            if (mag_calibration != NULL) {
                LOG_W("9.释放内存！\n");
                ellipsoidfit_free_calibration_data(mag_calibration);
                mag_calibration = NULL;  // 将指针置为 NULL，避免悬空指针
                mag_calib_state = CALIB_IDLE;//校准完成，回到空闲状态
                rt_event_send(imu_event, IMU_MAG_EVENT_DONE | IMU_CALIB_LED_DONE | IMU_CALIB_MOTOR_CONTROL_DONE);
                LOG_W("10.磁力计校准完成！\n");
            }
            break;
        default:
            mag_calib_state = CALIB_IDLE;
            break;
        }
#endif
        /**磁力计校准**/

        // 申请互斥锁(关联数据需保证数据同步性)
        rt_mutex_take(imu_data_mutex, RT_WAITING_FOREVER);
        /**状态量偏移校准**/
        thread_imu_data.acc_data.acc_x = acc[0] - thread_imu_data.acc_calib_data.bias_x;
        thread_imu_data.acc_data.acc_y = acc[1] - thread_imu_data.acc_calib_data.bias_y;
        thread_imu_data.acc_data.acc_z = acc[2] - (thread_imu_data.acc_calib_data.bias_z - 16384);
        thread_imu_data.gyro_data.gyro_x = gyro[0] - thread_imu_data.gyro_calib_data.bias_x;
        thread_imu_data.gyro_data.gyro_y = gyro[1] - thread_imu_data.gyro_calib_data.bias_y;
        thread_imu_data.gyro_data.gyro_z = gyro[2] - thread_imu_data.gyro_calib_data.bias_z;
        thread_imu_data.mag_data.mag_x = (int32_t)mag_calibrated[0];
        thread_imu_data.mag_data.mag_y = (int32_t)mag_calibrated[1];
        thread_imu_data.mag_data.mag_z = (int32_t)mag_calibrated[2];

        /**yaw解算**/
        //磁力计解算航向角
//        IMU_updata.heading = atan2((float)(thread_imu_data.mag_data.mag_y),(float)(thread_imu_data.mag_data.mag_x))*57.3f+180.0f;

        //九轴融合解算航向角
        IMUupdate((float)((3.1415926f / 180.0f)*thread_imu_data.gyro_data.gyro_x) / 16.4f,(float)((3.1415926f / 180.0f)*thread_imu_data.gyro_data.gyro_y) / 16.4f,
                (float)((3.1415926f / 180.0f)*thread_imu_data.gyro_data.gyro_z) / 16.4f,(float)thread_imu_data.acc_data.acc_x/16384.0f,(float)thread_imu_data.acc_data.acc_y/16384.0f,
                (float)thread_imu_data.acc_data.acc_z/16384.0f,(float)thread_imu_data.mag_data.mag_x,(float)thread_imu_data.mag_data.mag_y,(float)thread_imu_data.mag_data.mag_z,&mag_heading);
        if(mag_type == QMC5883L)
        thread_imu_data.heading = mag_heading + 180.0f;
        else if(mag_type == QMC5883P)
        thread_imu_data.heading = adjust_angle_minus_90(mag_heading + 180.0f);
        // 释放互斥锁
        rt_mutex_release(imu_data_mutex);

//        snprintf(send_buffer,sizeof(send_buffer) - 1,"%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",(int32_t)IMU_updata.imu_data[0],(int32_t)IMU_updata.imu_data[1],
//                (int32_t)IMU_updata.imu_data[2],thread_imu_data.acc_data.acc_x,thread_imu_data.acc_data.acc_y,thread_imu_data.acc_data.acc_z,
//                thread_imu_data.gyro_data.gyro_x,thread_imu_data.gyro_data.gyro_y,thread_imu_data.gyro_data.gyro_z);
//        rt_kprintf("%s",send_buffer);

        //九轴融合解算航向角(dmp)暂不可用
//        IMUupdate_dmp(thread_imu_data.mag_data.mag_x,thread_imu_data.mag_data.mag_y,
//                thread_imu_data.mag_data.mag_z,&mag_heading,quaternion);
//        IMU_updata.heading = mag_heading;

          /**IMU-PID-system远程调试**/

//        rt_kprintf("direction : %f\n",atan2((float)(thread_imu_data.mag_data.mag_x-1500.0),(float)(thread_imu_data.mag_data.mag_y-2000.0))*57.3f+180.0f);

//        rt_kprintf("%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%f,%f,%d,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%d,%f,%f,%f,%d\r\n",IMU_updata.imu_data[2],pid_motor.PID_control_yaw,
//                pid_heading_control.pid_target_delta_left,pid_heading_control.pid_target_delta_right,
//                pid_motor.PID_sign_fl * speed_pid_s.pid_rpm_fl,pid_motor.PID_sign_fr * speed_pid_s.pid_rpm_fr,
//                pid_motor.PID_sign_fl * pid_motor.PID_target_fl_smooth,pid_motor.PID_sign_fr * pid_motor.PID_target_fr_smooth,
//                pid_fl,pid_fr,input_pid_fl.pid_result,input_pid_fl.error_k0,input_pid_fl.error_k1,input_pid_fr.pid_result,input_pid_fr.error_k0,input_pid_fr.error_k1,
//                sum_t_fl,sum_t_fr,speed_pid_s.pid_rpm_fr_smoothing, pid_motor.PID_target_fr,pid_motor.PID_time_sign_fr,
//                pid_motor.PID_time_sign_fl,pid_motor.PID_sign_fr,pid_motor.PID_last_target_fr,PWM_FallingCount_4,PWM_FallingCount_5
//                ,robot_state.battery,robot_state.voltage,IMU_updata.mag_bias[0],IMU_updata.mag_bias[1],IMU_updata.mag_bias[2],robot_state.speed
//                ,robot_state.steer,(float)IMU_updata.imu_data[0],(float)IMU_updata.imu_data[1],(float)IMU_updata.imu_data[2],IMU_updata.heading);

//        rt_kprintf("mag_bias_x %d ,mag_bias_y %d ,mag_bias_z %d ,acc_bias_x %d ,acc_bias_y %d ,acc_bias_z %d,gyro_bias_x %d ,gyro_bias_y %d ,gyro_bias_z %d \n",
//                IMU_updata.mag_bias[0],IMU_updata.mag_bias[1],IMU_updata.mag_bias[2],thread_imu_data.acc_calib_data.bias_x,thread_imu_data.acc_calib_data.bias_y,thread_imu_data.acc_calib_data.bias_z,
//                thread_imu_data.gyro_calib_data.bias_x,thread_imu_data.gyro_calib_data.bias_y,thread_imu_data.gyro_calib_data.bias_z);

        //IMU数据LOG
//        rt_kprintf("st_acc_x   %f , st_acc_y   %f , st_acc_z   %f \n",(float)thread_imu_data.acc_data.acc_x/16384.0f,(float)thread_imu_data.acc_data.acc_y/16384.0f,(float)thread_imu_data.acc_data.acc_z/16384.0f);
//        rt_kprintf("st_gyro_x   %f , st_gyro_y   %f , st_gyro_z   %f \n",(float)((3.1415926f / 180.0f) * thread_imu_data.gyro_data.gyro_x) / 16.4f,(float)((3.1415926f / 180.0f) * thread_imu_data.gyro_data.gyro_y)/ 16.4f,(float)((3.1415926f / 180.0f) * thread_imu_data.gyro_data.gyro_z)/ 16.4f);
//        LOG_I("dmp：%f , %f , %f\n",IMU_updata.imu_data[0],IMU_updata.imu_data[1],IMU_updata.imu_data[2]);
//        LOG_I("mag：%d , %d , %d , %d \n",thread_imu_data.mag_data.mag_x,thread_imu_data.mag_data.mag_y,thread_imu_data.mag_data.mag_z,IMU_updata.heading);
        //        GetMagTask(IMU_updata.Mag_data);
        // 线程会自动回到阻塞状态，等待下次信号量释放
        if(mag_calib_state != CALIB_IDLE)
        {
            rt_thread_mdelay ( 50 ); //FIXME
        }
        rt_thread_mdelay ( 0 ); //FIXME
        }
        else {
            rt_thread_mdelay ( 200 ); //FIXME
        }
    }

}
