//*
* Copyright (c) 2006-2021, RT-Thread Development Team
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date           Author       Notes
* 2024-07-30     yuxing       the first version
*/
#define DBG_TAG "QMC5883L"
#define DBG_LVL DBG_LOG
#include "QMC5883L.h"

// ==========================================================================
// Legacy initialization function for HMC5883L sensor (similar to QMC5883L)
// Used for backward compatibility or testing
// ==========================================================================
uint8_t HMC5883L_Init(void)
{
   uint8_t set = 0;
   uint8_t Config=0;
   uint8_t ChipID = 0;
   // Register addresses for X, Y, Z axes (HMC5883L specific)
   uint8_t Mag[6] = {0x03 ,0x04,
                     0x05 ,0x06 ,
                     0x07 ,0x08 };
   uint8_t val[6] = {0};

   /** 模式配置 **/
   /** Mode configuration **/
   set = 0x80;
   QMC5883L_write_Len(0x1E,0x02,1,&set);

   /** ID查询 **/
   /** ID query (read identification registers) **/
   for(uint8_t i=0; i<6; i++){
       if(QMC5883L_read_Len(0x1E,Mag[i],1,&val[i])){
           LOG_E("get HMC5883L Angle[0X%02X] error\r\n",Mag[i]);
           return 1;
       }
   }

   // Print raw data values for debugging (X, Y, Z axes)
   rt_kprintf("hmc_x = %d , hmc_y = %d , hmc_z = %d \n",
              (int16_t)((val[0] << 8) | val[1]),
              (int16_t)((val[2] << 8) | val[3]),
              (int16_t)((val[4] << 8) | val[5]));
}

// ==========================================================================
// QMC5883L Initialization
// This function configures the sensor for continuous operation
// in either software I2C mode (IIC_SOFT) or hardware HAL I2C mode.
// ==========================================================================
rt_err_t QMC5883L_Init(void)
{
   uint8_t set = 0;
   uint8_t Config=0;
   uint8_t ChipID = 0;

#ifdef IIC_SOFT
   /** ID查询 **/
   /** Read device ID over software I2C **/
   if(QMC5883L_read_reg(QMC5883L_ID,&ChipID))
   {
       LOG_E("QMC5883L device no find\r\n");
       return RT_ERROR;
   }
   if(ChipID == 0xFF){
       LOG_I("QMC5883L device is find\r\n");
   }

   /** 软复位 **/
   /** Soft reset **/
   set = 0x80;
   if(QMC5883L_write_reg(QMC5883L_CONFIG_2,set))
   {
       LOG_E("set QMC5883L RESET-2 Period error\r\n");
       return RT_ERROR;
   }

   // Set/reset period (recommended value = 0x01)
   set = 0x01;
   if(QMC5883L_write_reg(QMC5883L_CONFIG_3,set))
   {
       LOG_E("set QMC5883L RESET-3 Period error\r\n");
       return RT_ERROR;
   }

   // Configure oversampling, range, data rate, and continuous mode
   Config =    OVER_SAMPLE_RATIO_512\
               |FULL_SCALE_2G\
               |OUTPUT_DATA_RATE_10HZ\
               |MODE_CONTROL_CONTINUOUS;

   set = Config;
   if(QMC5883L_write_reg(QMC5883L_CONFIG_1,set))
   {
       LOG_E("set QMC5883L RESET-1 Period error\r\n");
       return RT_ERROR;
   }

#else   // Hardware I2C (HAL driver based)
   HAL_StatusTypeDef hi2c1_status = 0x00;

   // Read Chip ID register using HAL I2C
   hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,QMC5883L_ID,1,&ChipID,1,1000);//QMC5883L_ID
   if(ChipID == 0xFF){
       LOG_I("QMC5883L device is find\r\n");
   }
   if(HAL_OK !=hi2c1_status){
       LOG_I("find QMC5883L error! \r\n");
       return 1;
   }

   // 软复位
   // Soft reset
   set = 0x80;
   hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,QMC5883L_CONFIG_2,1,&set,1,1000);
   if(HAL_OK!=hi2c1_status){
       LOG_I("set QMC5883L RESET Period error\r\n");
       return 1;
   }
   rt_thread_mdelay ( 100 ); // Allow time for reset to complete

   // Set/reset period (manual recommends value 0x01)
   set = 0x01;
   hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,QMC5883L_CONFIG_3,1,&set,1,1000);
   if(HAL_OK!=hi2c1_status){
       LOG_I("set QMC5883L RESET Period error\r\n");
       return 1;
   }

   // Optional extra register writes (commented out, may be used for tuning)
   // set = 0x40;
   // hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,0x20,1,&set,1,1000);
   // ...
   // set = 0x01;
   // hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,0x21,1,&set,1,1000);
   // ...

   // Configure oversampling, range, data rate, and continuous mode
   Config =    OVER_SAMPLE_RATIO_512\
               |FULL_SCALE_2G\
               |OUTPUT_DATA_RATE_10HZ\
               |MODE_CONTROL_CONTINUOUS;

   /**** OSR=512, RNG=±2G, ODR=200Hz, MODE= continuous ****/
   set = Config;
   hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,QMC5883L_CONFIG_1,1,&set,1,1000);
   if(HAL_OK!=hi2c1_status){
       LOG_I("set QMC5883L SET Mode error\r\n");
       return 1;
   }
#endif

   return RT_EOK;
}
/**
 * 软件模拟IIC-QMC5883L-写函数
 * Software-simulated I2C write function for QMC5883L
 *
 * @param addr  I2C device address
 * @param reg   Register to start writing from
 * @param len   Number of bytes to write
 * @param date  Pointer to data buffer containing bytes to write
 * @return      RT_EOK on success, -RT_ERROR on failure
 *
 * Note: Used for MPU DMP porting compatibility
 */
rt_err_t QMC5883L_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date)
{
    rt_uint8_t buf[len + 1];
    rt_uint8_t i;

    // First byte is the register address
    buf[0] = reg;

    // Copy the data payload into the buffer
    for (i = 0; i < len; i++)
    {
        buf[i + 1] = date[i];
    }

    // Perform I2C write: register + data
    if (rt_i2c_master_send(mpu6050_i2c_bus, addr, 0, buf, len + 1) == len + 1)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-QMC5883L-读函数
 * Software-simulated I2C read function for QMC5883L
 *
 * @param addr  I2C device address
 * @param reg   Register to read from
 * @param len   Number of bytes to read
 * @param buf   Buffer to store read data
 * @return      RT_EOK on success, -RT_ERROR on failure
 *
 * Note: Used for MPU DMP porting compatibility
 */
rt_err_t QMC5883L_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];

    // Step 1: Write register address to device
    msgs[0].addr  = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    // Step 2: Read data from the device starting at that register
    msgs[1].addr  = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = buf;
    msgs[1].len   = len;

    // Perform transfer: should succeed with 2 transactions
    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-QMC5883L-单个字节写
 * Software-simulated I2C single-byte write to QMC5883L
 *
 * @param reg   Register address
 * @param data  Single byte of data to write
 * @return      RT_EOK on success, -RT_ERROR on failure
 */
rt_err_t QMC5883L_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];

    buf[0] = reg;   // target register
    buf[1] = data;  // value to write

    // Write register and value to QMC5883L
    if (rt_i2c_master_send(mpu6050_i2c_bus, QMC5883L_ADDRESS_SOFT, 0, buf, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-QMC5883L-单个字节读
 * Software-simulated I2C single-byte read from QMC5883L
 *
 * @param reg   Register address
 * @param data  Pointer to buffer to store read byte
 * @return      RT_EOK on success, -RT_ERROR on failure
 */
rt_err_t QMC5883L_read_reg(rt_uint8_t reg, rt_uint8_t *data)
{
    struct rt_i2c_msg msgs[2];

    // Step 1: Write register address
    msgs[0].addr  = QMC5883L_ADDRESS_SOFT;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    // Step 2: Read a single byte from that register
    msgs[1].addr  = QMC5883L_ADDRESS_SOFT;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * QMC5883L_GetAngle
 * Reads raw magnetometer data (X, Y, Z axes) from the QMC5883L.
 *
 * Orientation note:
 *   - When chip pin 1 is at top-left, and the bottom side is the reference:
 *       0°   = South
 *       90°  = West
 *       180° = North
 *       270° = East
 *
 * @param data  Pointer to int32_t[3] array to store X, Y, Z axis values
 * @return      0 on success, 1 on failure
 */
void QMC5883L_GetAngle(int32_t* data)
{
    HAL_StatusTypeDef hi2c1_status = 0x00;
    uint8_t Mag[6] = {
        QMC5883L_DATA_READ_X_LSB, QMC5883L_DATA_READ_X_MSB,
        QMC5883L_DATA_READ_Y_LSB, QMC5883L_DATA_READ_Y_MSB,
        QMC5883L_DATA_READ_Z_LSB, QMC5883L_DATA_READ_Z_MSB
    };
    uint8_t val[6] = {0};
    uint8_t check = 0;

    /**
     * Sensor status register (0x06) description:
     * - DRDY (Data Ready): set to '1' when new data is available. Cleared to '0' after reading 0x06.
     * - OVL (Overflow): set to '1' if any axis measurement exceeds range. Cleared when next measurement is valid.
     * - DOR (Data Skip): set to '1' if data was skipped in continuous mode. Cleared after register read/write.
     */
    // Example status check code (commented out)
    // hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,0x06,1,&check,1,1000);
    // if(HAL_OK!=hi2c1_status) return 1;
    // LOG_I("0x06 = %d\r\n",check);

#ifdef IIC_SOFT
    // Read 6 bytes (X, Y, Z) over software I2C
    if(QMC5883L_read_Len(QMC5883L_ADDRESS_SOFT,QMC5883L_DATA_READ_X_LSB,6,val)){
        LOG_E("get QMC5883L data error\r\n");
        return 1;
    }
#else
    // Read 6 bytes one by one using HAL I2C
    for(uint8_t i=0; i<6; i++){
        hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,Mag[i],1,&val[i],1,1000);
        if(HAL_OK!=hi2c1_status){
            printf("get QMC5883L Angle[0X%02X] error\r\n",Mag[i]);
            return 1;
        }
    }
#endif

    // Convert raw register values into signed 16-bit integers
    int32_t data_t[3] = {0};
    data_t[0] = (int16_t)((val[1] << 8) | val[0]); // X-axis
    data_t[1] = (int16_t)((val[3] << 8) | val[2]); // Y-axis
    data_t[2] = (int16_t)((val[5] << 8) | val[4]); // Z-axis

    // Store results in output buffer
    for(int i = 0; i < 3; i++)
        *(data + i) = data_t[i];

    return 0;
}

/**
 * 磁力计原始数据计算航向角
 * Calculate compass heading angle from magnetometer data
 *
 * @param mag_x   X-axis magnetic field value
 * @param mag_y   Y-axis magnetic field value
 * @return        Heading angle in degrees [0, 360)
 *
 * Notes:
 *   - Uses atan2(mag_y, mag_x) to compute heading.
 *   - Converts from radians to degrees.
 *   - Ensures result is wrapped into [0, 360).
 */
float calculate_heading(float mag_x, float mag_y) {
    float heading = atan2(mag_y, mag_x) * (180.0 / M_PI);
    if (heading < 0) {
        heading += 360;
    }
    return heading;
}
