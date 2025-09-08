/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-10-24     yuxing       the first version
 */
#define DBG_TAG "MPU6050"
#define DBG_LVL DBG_LOG
#include "MPU6050.h"

#define MPU6050_I2CBUS_NAME  "i2c1"
struct rt_i2c_bus_device *mpu6050_i2c_bus;


uint8_t MPU6050_Init(void)
{
#ifdef IIC_SOFT
    /**********************软件IIC初始化***************************/
    // - Software IIC initialization
    rt_uint8_t res;
    rt_device_t dev;
    dev = rt_device_find(MPU6050_I2CBUS_NAME);

    if (dev == RT_NULL)
    {
        LOG_E("can't find mpu6050 %s device\r\n", MPU6050_I2CBUS_NAME);
        return -RT_ERROR;
    }

    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        LOG_E("can't opend mpu6050 %s device\r\n", MPU6050_I2CBUS_NAME);
        return -RT_ERROR;
    }
    //获取i2c设备句柄
    // - Get i2c device handle
    mpu6050_i2c_bus = (struct rt_i2c_bus_device *)dev;
    /**********************软件IIC初始化***************************/
    // - Software IIC initialization

    res = mpu6050_write_reg(MPU_PWR_MGMT1_REG,0X80);//复位MPU6050
    // - Reset MPU6050
    if(res == RT_EOK)
        LOG_I("mpu6050 reset successful!\n");
    else {
        LOG_E("mpu6050 reset error!\n");
    }
    rt_thread_delay(100);  //延时100ms
    // - Delay 100ms
    mpu6050_write_reg(MPU_PWR_MGMT1_REG,0X00);//唤醒MPU6050
    // - Wake up MPU6050
    MPU_Set_Gyro_Fsr(3);                                //陀螺仪传感器,±2000dps
    // - Gyroscope sensor, ±2000dps
    MPU_Set_Accel_Fsr(0);                               //加速度传感器,±2g
    // - Accelerometer sensor, ±2g
    MPU_Set_Rate(50);                        //设置采样率50Hz
    // - Set sampling rate 50Hz
    mpu6050_write_reg(MPU_INT_EN_REG,0X00);   //关闭所有中断
    // - Disable all interrupts
    res = mpu6050_write_reg(MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
    // - Disable I2C master mode
    if(res == RT_EOK)
        LOG_I("mpu6050 master mode close successful!\n");
    else {
        LOG_E("mpu6050 master mode close error!\n");
    }
    mpu6050_write_reg(MPU_FIFO_EN_REG,0X00);  //关闭FIFO
    // - Disable FIFO
    res = mpu6050_write_reg(MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    // - INT pin active low, enable bypass mode, can directly read magnetometer
    if(res == RT_EOK)
        LOG_I("mpu6050 bypass set successful!\n");
    else {
        LOG_E("mpu6050 bypass set error!\n");
    }
    mpu6050_read_reg(MPU_DEVICE_ID_REG,&res);  //读取MPU6500的ID
    // - Read MPU6500 ID
    if(res==MPU6050_ADDR_soft)              //器件ID(0x68)正确res==MPU6050_ADDR_soft
    // - Device ID (0x68) correct, res == MPU6050_ADDR_soft
    {
        mpu6050_write_reg(MPU_PWR_MGMT1_REG,0X01);    //设置CLKSEL,PLL X轴为参考
        // - Set CLKSEL, PLL X-axis as reference
        mpu6050_write_reg(MPU_PWR_MGMT2_REG,0X00);    //加速度与陀螺仪都工作
        // - Accelerometer and gyroscope both working
        MPU_Set_Rate(50);                                        //设置采样率为50Hz
        // - Set sampling rate to 50Hz
    }else return -RT_ERROR;
    mpu_dmp_init();                                             //初始化DMP
    // - Initialize DMP


#else
    HAL_StatusTypeDef hi2c1_status = 0x80;
    uint8_t set = 0;
    uint8_t addr_val[3] = {MPU_PWR_MGMT1_REG,MPU_INTBP_CFG_REG,0};
    set = 0x80;
    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,MPU_ADDR,addr_val[0],1,&set,1,1000);//唤醒传感器
    // - Wake up sensor
    if(HAL_OK!=hi2c1_status){
        rt_kprintf("MPU6050 PWR1 reset error\r\n");
        return 1;
    }
    rt_thread_mdelay ( 100 );
    set = 0x00;
    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,MPU_ADDR,addr_val[0],1,&set,1,1000);//唤醒传感器
    // - Wake up sensor
    if(HAL_OK!=hi2c1_status){
        rt_kprintf("MPU6050 PWR1 error\r\n");
        return 1;
    }
    rt_thread_mdelay ( 100 );
    set = 0x02;
    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,MPU_ADDR,addr_val[1],1,&set,1,1000);//开启旁路模式(临时模块使用)
    // - Enable bypass mode (temporary module use)
    if(HAL_OK!=hi2c1_status){
        rt_kprintf("MPU6050_INTBP_CFG_REG open error\r\n");
        return 1;
    }
    rt_thread_mdelay ( 100 );
    MPU_Set_Gyro_Fsr(3);                                        //陀螺仪传感器,±2000dps
    // - Gyroscope sensor, ±2000dps
    MPU_Set_Accel_Fsr(0);                                       //加速度传感器,±2g
    // - Accelerometer sensor, ±2g
    MPU_Set_Rate(50);                                           //设置采样率50Hz
    // - Set sampling rate 50Hz

    set = 0x00;
    if(MPU6050_iic_write( MPU_ADDR , MPU_INT_EN_REG , 1 , &set))//关闭所有中断
    // - Disable all interrupts
    set = 0x00;
    MPU6050_iic_write( MPU_ADDR , MPU_USER_CTRL_REG , 1 , &set);//I2C主模式关闭
    // - Disable I2C master mode
    set = 0x00;
    MPU6050_iic_write( MPU_ADDR , MPU_FIFO_EN_REG , 1 , &set);//FIFO关闭
    // - Disable FIFO
    set = 0x80;
    MPU6050_iic_write( MPU_ADDR , MPU_INTBP_CFG_REG , 1 , &set);//INT引脚低电平有效
    // - INT pin active low
    set = 0x01;
    MPU6050_iic_write( MPU_ADDR , MPU_PWR_MGMT1_REG , 1 , &set);//设置CLKSEL,PLL X轴为参考
    // - Set CLKSEL, PLL X-axis as reference
    set = 0x00;
    MPU6050_iic_write( MPU_ADDR , MPU_PWR_MGMT2_REG , 1 , &set);//设置CLKSEL,PLL X轴为参考
    // - Set CLKSEL, PLL X-axis as reference
    MPU_Set_Rate(50);                                           //设置采样率50Hz
    // - Set sampling rate 50Hz

    mpu_dmp_init();                                             //初始化DMP
    // - Initialize DMP
#endif

}
/**
 * 硬件IIC-mpu6050-dmp写函数
 * - Hardware IIC - mpu6050 - dmp write function
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
int8_t MPU6050_iic_write(unsigned char addr , unsigned char reg , uint16_t Size , uint8_t *tmp)
{
    HAL_StatusTypeDef hi2c1_status = 0x00;

    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,tmp, Size,1000);
    if(HAL_OK !=hi2c1_status){
        return 1;
    }

    return 0;
}

/**
 * 硬件IIC-mpu6050-dmp读函数
 * - Hardware IIC - mpu6050 - dmp read function
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
int8_t MPU6050_iic_read(unsigned char addr , unsigned char reg , uint16_t Size , uint8_t *data)
{
    HAL_StatusTypeDef hi2c1_status = 0x00;

    hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,data, Size,1000);
    if(HAL_OK !=hi2c1_status){
        return 1;
    }

    return 0;
}

/**
 * 软件模拟IIC-mpu6050-dmp写函数
 * - Software simulated IIC - mpu6050 - dmp write function
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t mpu_dmp_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date) //mpu_dmp 移植兼容使用
// - mpu_dmp porting compatibility use
{
    rt_uint8_t buf[len + 1];
    rt_uint8_t i;
    buf[0] = reg;
    for (i = 0; i < len; i++)
    {
        buf[i + 1] = date[i];
    }
    if (rt_i2c_master_send(mpu6050_i2c_bus, addr, 0, buf, len + 1) == len + 1)
        return RT_EOK;
    else
        return -RT_ERROR;
}
/**
 * 软件模拟IIC-mpu6050-dmp读函数
 * - Software simulated IIC - mpu6050 - dmp read function
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t mpu_dmp_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf) //mpu_dmp 移植兼容使用
// - mpu_dmp porting compatibility use
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = buf;
    msgs[1].len   = len;

    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-mpu6050-单个字节写
 * - Software simulated IIC - mpu6050 - single byte write
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t mpu6050_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];

    buf[0] = reg;
    buf[1] = data;

    if (rt_i2c_master_send(mpu6050_i2c_bus, MPU6050_ADDR_soft, 0, buf, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-mpu6050-单个字节读
 * - Software simulated IIC - mpu6050 - single byte read
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t mpu6050_read_reg(rt_uint8_t reg, rt_uint8_t *data)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = MPU6050_ADDR_soft;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = MPU6050_ADDR_soft;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**********************************************
函数名称：MPU_Set_Gyro_Fsr
- Function name: MPU_Set_Gyro_Fsr
函数功能：设置MPU6050陀螺仪传感器满量程范围
- Function: Set full-scale range of MPU6050 gyroscope sensor
函数参数：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
- Parameters: fsr: 0, ±250dps; 1, ±500dps; 2, ±1000dps; 3, ±2000dps
函数返回值：0,设置成功  其他,设置失败
- Return value: 0, success; others, failure
**********************************************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    int8_t ret = 0;
    fsr = fsr<<3;
#ifdef IIC_SOFT
    ret = mpu_dmp_write_Len(MPU6050_ADDR_soft , MPU_GYRO_CFG_REG , 1 , &fsr);//设置陀螺仪满量程范围
    // - Set full-scale range of gyroscope
#else
    ret = MPU6050_iic_write(MPU_ADDR , MPU_GYRO_CFG_REG , 1 , &fsr);//设置陀螺仪满量程范围
    // - Set full-scale range of gyroscope
#endif
    if(ret)
    {
        LOG_E("MPU_GYRO_CFG_REG set error!");
    }
    else {
        LOG_I("MPU_GYRO_CFG_REG set successful!");
    }
    return ret;
}

/**********************************************
函数名称：MPU_Set_Accel_Fsr
- Function name: MPU_Set_Accel_Fsr
函数功能：设置MPU6050加速度传感器满量程范围
- Function: Set full-scale range of MPU6050 accelerometer
函数参数：fsr:0,±2g;1,±4g;2,±8g;3,±16g
- Parameters: fsr: 0, ±2g; 1, ±4g; 2, ±8g; 3, ±16g
函数返回值：0,设置成功  其他,设置失败
- Return value: 0, success; others, failure
**********************************************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    int8_t ret = 0;
    fsr = fsr<<3;
#ifdef IIC_SOFT
    ret = mpu_dmp_write_Len(MPU6050_ADDR_soft , MPU_ACCEL_CFG_REG , 1 , &fsr);//设置加速度计满量程范围
    // - Set full-scale range of accelerometer
#else
    ret = MPU6050_iic_write(MPU_ADDR , MPU_ACCEL_CFG_REG , 1 , &fsr);//设置加速度计满量程范围
    // - Set full-scale range of accelerometer
#endif
    if(ret)
    {
        LOG_E("MPU_ACCEL_CFG_REG set error!");
    }
    else {
        LOG_I("MPU_ACCEL_CFG_REG set successful!");
    }
    return ret;
}

/**********************************************
函数名称：MPU_Set_LPF
- Function name: MPU_Set_LPF
函数功能：设置MPU6050的数字低通滤波器
- Function: Set digital low-pass filter (LPF) of MPU6050
函数参数：lpf:数字低通滤波频率(Hz)
- Parameters: lpf: digital low-pass filter frequency (Hz)
函数返回值：0,设置成功  其他,设置失败
- Return value: 0, success; others, failure
**********************************************/
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;

    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;

    int8_t ret = 0;
#ifdef IIC_SOFT
    ret = mpu_dmp_write_Len(MPU6050_ADDR_soft , MPU_CFG_REG , 1 , &data);//设置加速度计满量程范围
    // - Set digital low-pass filter
#else
    ret = MPU6050_iic_write(MPU_ADDR , MPU_CFG_REG , 1 , &data);//设置加速度计满量程范围
    // - Set digital low-pass filter
#endif
    if(ret)
    {
        LOG_E("MPU_CFG_REG set error!");
    }
    else {
        LOG_I("MPU_CFG_REG set successful!");
    }
    return ret;
}

/**********************************************
函数名称：MPU_Set_Rate
- Function name: MPU_Set_Rate
函数功能：设置MPU6050的采样率(假定Fs=1KHz)
- Function: Set MPU6050 sample rate (assuming Fs=1KHz)
函数参数：rate:4~1000(Hz)  初始化中rate取50
- Parameters: rate: 4~1000(Hz), in initialization rate=50
函数返回值：0,设置成功  其他,设置失败
- Return value: 0, success; others, failure
**********************************************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;

    int8_t ret = 0;
#ifdef IIC_SOFT
    ret = mpu_dmp_write_Len(MPU6050_ADDR_soft , MPU_SAMPLE_RATE_REG , 1 , &data);//设置加速度计满量程范围
    // - Set sample rate
#else
    ret = MPU6050_iic_write(MPU_ADDR , MPU_SAMPLE_RATE_REG , 1 , &data);//设置加速度计满量程范围
    // - Set sample rate
#endif
    if(ret)
    {
        LOG_E("MPU_SAMPLE_RATE_REG set error!");
    }
    else {
        LOG_I("MPU_SAMPLE_RATE_REG set successful!");
    }
    return MPU_Set_LPF(rate/2);                                         //自动设置LPF为采样率的一半
    // - Automatically set LPF to half of sample rate
}
uint8_t MPU6050_acc_read(int32_t *x_data,int32_t *y_data,int32_t *z_data)
{
    HAL_StatusTypeDef hi2c1_status = 0x00;

    uint8_t addr[6] = {MPU_ACCEL_XOUTL_REG,MPU_ACCEL_XOUTH_REG,
            MPU_ACCEL_YOUTL_REG,MPU_ACCEL_YOUTH_REG,
            MPU_ACCEL_ZOUTL_REG,MPU_ACCEL_ZOUTH_REG};
    uint8_t val[6] = {0};
#ifdef IIC_SOFT
//    for(uint8_t i=0; i<6; i++){
//        if(mpu_dmp_read_Len( MPU6050_ADDR_soft , addr[i] , 1 , &val[i]))
//        {
//            LOG_E("MPU_ACCEL data[%d] read error!",i);
//        }
//    }
    if(mpu_dmp_read_Len( MPU6050_ADDR_soft , addr[1] , 6 , val))
    {
        LOG_E("MPU_ACCEL data read error!");
        // - MPU6050 accelerometer data read error
    }

#else
    for(uint8_t i=0; i<6; i++){
        hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,MPU_ADDR,addr[i],1,&val[i],1,1000);
        if(HAL_OK!=hi2c1_status){
            printf("get MPU6050 acc_read[0X%02X] error\r\n",addr[i]);
            // - Error reading MPU6050 accelerometer register
            return 1;
        }
    }
#endif
//    printf("read acc reg_data 1:%02X, 2:%02X, 3:%02X, 4:%02X, 5:%02X ,6:%02X\r\n"
//                    ,val[0],val[1],val[2],val[3],val[4],val[5]);
    int32_t data[3] = {0};
    data[0] = (int16_t)((((int16_t)((int8_t)val[0])) << 8) | (val[1]));
    data[1] = (int16_t)((((int16_t)((int8_t)val[2])) << 8) | (val[3]));
    data[2] = (int16_t)((((int16_t)((int8_t)val[4])) << 8) | (val[5]));
    *x_data = data[0];
    *y_data = data[1];
    *z_data = data[2];
    return 0;
}

/*角速度寄存器读取*/
// - Angular velocity register read
uint8_t MPU6050_gyro_read(int32_t *x_data,int32_t *y_data,int32_t *z_data)
{
    HAL_StatusTypeDef hi2c1_status = 0x00;
    uint8_t addr[6] = {MPU_GYRO_XOUTL_REG,MPU_GYRO_XOUTH_REG,
            MPU_GYRO_YOUTL_REG,MPU_GYRO_YOUTH_REG,
            MPU_GYRO_ZOUTL_REG,MPU_GYRO_ZOUTH_REG};
    uint8_t val[6] = {0};

#ifdef IIC_SOFT
//    for(uint8_t i=0; i<6; i++){
//        if(mpu_dmp_read_Len( MPU6050_ADDR_soft , addr[i] , 1 , &val[i]))
//        {
//            LOG_E("MPU_GYRO data[%d] read error!",i);
//        }
//    }
    if(mpu_dmp_read_Len( MPU6050_ADDR_soft , addr[1] , 6 , val))
    {
        LOG_E("MPU_GYRO data read error!");
        // - MPU6050 gyroscope data read error
    }
#else
    for(uint8_t i=0; i<6; i++){
        hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,MPU_ADDR,addr[i],1,&val[i],1,1000);
        if(HAL_OK!=hi2c1_status){
            printf("get MPU6050 gyro_read[0X%02X] error\r\n",addr[i]);
            // - Error reading MPU6050 gyroscope register
            return 1;
        }
    }
#endif
//    printf("read gyro reg_data 1:%02X, 2:%02X, 3:%02X, 4:%02X, 5:%02X ,6:%02X\r\n"
//                ,val[0],val[1],val[2],val[3],val[4],val[5]);
    int32_t data[3] = {0};
    data[0] = (int16_t)((((int16_t)((int8_t)val[0])) << 8) | (val[1]));
    data[1] = (int16_t)((((int16_t)((int8_t)val[2])) << 8) | (val[3]));
    data[2] = (int16_t)((((int16_t)((int8_t)val[4])) << 8) | (val[5]));

    *x_data = data[0];
    *y_data = data[1];
    *z_data = data[2];
    return 0;
}
