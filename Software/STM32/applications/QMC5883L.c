/*
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

uint8_t HMC5883L_Init(void)
{
    uint8_t set = 0;
    uint8_t Config=0;
    uint8_t ChipID = 0;
    uint8_t Mag[6] = {0x03 ,0x04,
            0x05 ,0x06 ,
            0x07 ,0x08 };
    uint8_t val[6] = {0};
    /**模式配置**/
    set = 0x80;
    QMC5883L_write_Len(0x1E,0x02,1,&set);
    /**ID查询**/
    for(uint8_t i=0; i<6; i++){
        if(QMC5883L_read_Len(0x1E,Mag[i],1,&val[i])){
            LOG_E("get HMC5883L Angle[0X%02X] error\r\n",Mag[i]);
            return 1;
        }
    }
        rt_kprintf("hmc_x = %d , hmc_y = %d , hmc_z = %d \n",(int16_t)((val[0] << 8) | val[1]),(int16_t)((val[2] << 8) | val[3]),
                (int16_t)((val[4] << 8) | val[5]));

}

rt_err_t QMC5883L_Init(void)
{
    uint8_t set = 0;
    uint8_t Config=0;
    uint8_t ChipID = 0;
#ifdef IIC_SOFT
    /**ID查询**/
    if(QMC5883L_read_reg(QMC5883L_ID,&ChipID))
    {
        LOG_E("QMC5883L device no find\r\n");
        return RT_ERROR;
    }
    if(ChipID == 0xFF){
        LOG_I("QMC5883L device is find\r\n");

    }
    /**软复位**/
    set = 0x80;
    if(QMC5883L_write_reg(QMC5883L_CONFIG_2,set))
    {
        LOG_E("set QMC5883L RESET-2 Period error\r\n");
        return RT_ERROR;
    }

    set = 0x01;
    if(QMC5883L_write_reg(QMC5883L_CONFIG_3,set))
    {
        LOG_E("set QMC5883L RESET-3 Period error\r\n");
        return RT_ERROR;
    }

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


#else
    HAL_StatusTypeDef hi2c1_status = 0x00;

    hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,QMC5883L_ID,1,&ChipID,1,1000);//QMC5883L_ID
    if(ChipID == 0xFF){
        LOG_I("QMC5883L device is find\r\n");

    }
    if(HAL_OK !=hi2c1_status){
                    LOG_I("find QMC5883L error! \r\n");
                    return 1;
           }
    //软复位
    set = 0x80;
       hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,QMC5883L_CONFIG_2,1,&set,1,1000);
       if(HAL_OK!=hi2c1_status){
                   LOG_I("set QMC5883L RESET Period error\r\n");
                   return 1;
          }
       rt_thread_mdelay ( 100 );

    set = 0x01;
    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,QMC5883L_CONFIG_3,1,&set,1,1000);//设置周期，手册建议写入值0x01
    if(HAL_OK!=hi2c1_status){
                LOG_I("set QMC5883L RESET Period error\r\n");
                return 1;
       }
//    set = 0x40;
//    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,0x20,1,&set,1,1000);
//    if(HAL_OK!=hi2c1_status){
//                LOG_I("set QMC5883L 0x20 error\r\n");
//                return 1;
//           }
//    set = 0x01;
//    hi2c1_status = HAL_I2C_Mem_Write(&hi2c1,QMC5883L_ADDRESS,0x21,1,&set,1,1000);
//    if(HAL_OK!=hi2c1_status){
//                    LOG_I("set QMC5883L 0x21 error\r\n");
//                    return 1;
//               }
    Config =    OVER_SAMPLE_RATIO_512\
            |FULL_SCALE_2G\
            |OUTPUT_DATA_RATE_10HZ\
            |MODE_CONTROL_CONTINUOUS;

    /****OSR=512,RNG=+/-2G,ODR=200Hz,MODE= continuous*******/
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
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t QMC5883L_write_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *date) //mpu_dmp 移植兼容使用
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
 * 软件模拟IIC-QMC5883L-读函数
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t QMC5883L_read_Len(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf) //mpu_dmp 移植兼容使用
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
 * 软件模拟IIC-QMC5883L-单个字节写
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t QMC5883L_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];

    buf[0] = reg;
    buf[1] = data;

    if (rt_i2c_master_send(mpu6050_i2c_bus, QMC5883L_ADDRESS_SOFT, 0, buf, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

/**
 * 软件模拟IIC-QMC5883L-单个字节读
 * @param addr
 * @param reg
 * @param Size
 * @param tmp
 * @return
 */
rt_err_t QMC5883L_read_reg(rt_uint8_t reg, rt_uint8_t *data)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = QMC5883L_ADDRESS_SOFT;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = QMC5883L_ADDRESS_SOFT;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(mpu6050_i2c_bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}


void QMC5883L_GetAngle(int32_t* data)//芯片第一脚在左上角，以下方做为指示，0为南，90为西，180为北，270为东
{
    HAL_StatusTypeDef hi2c1_status = 0x00;
    uint8_t Mag[6] = {QMC5883L_DATA_READ_X_LSB,QMC5883L_DATA_READ_X_MSB,
            QMC5883L_DATA_READ_Y_LSB,QMC5883L_DATA_READ_Y_MSB,
            QMC5883L_DATA_READ_Z_LSB,QMC5883L_DATA_READ_Z_MSB};
    uint8_t val[6] = {0};
    uint8_t check = 0;
/**
     * 当传感器数据已测量完毕并准备好DRDY位被置“1”，数据寄存器0x06一旦被读取，DRDY位将被置“0”。
    当有任意一个轴的测量值超过范围，OVL将被置“1”，当下一次测量不超测量范围时，OVL将会被置“0”。
    当处于连续模式下测量数据被跳过时DOR被置“1”，而当数据寄存器被读写后置“0”
   */
//    hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,0x06,1,&check,1,1000);
//            if(HAL_OK!=hi2c1_status){
////                printf("get 0x06 error\r\n");
//                return 1;
//            }
//            if(check << 7)
//            {
////                printf("data is read\r\n");
//            }
//            LOG_I("0x06 = %d\r\n",check); //传感器状态回读


#if 0
            check = 0;
            HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,0x0B,1,&check,1,1000);
            printf("0x0B = %02x\r\n",check);

            check = 0;
            HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,0x00,1,&check,1,1000);
            printf("0x00 = %d\r\n",check);

            check = 0;
            HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,0x01,1,&check,1,1000);
            printf("0x01 = %d\r\n",check);

#endif
#ifdef IIC_SOFT

            if(QMC5883L_read_Len(QMC5883L_ADDRESS_SOFT,QMC5883L_DATA_READ_X_LSB,6,val)){
                LOG_E("get QMC5883L data error\r\n");
                return 1;
            }

#else
                for(uint8_t i=0; i<6; i++){
                    hi2c1_status = HAL_I2C_Mem_Read(&hi2c1,QMC5883L_ADDRESS,Mag[i],1,&val[i],1,1000);
                    if(HAL_OK!=hi2c1_status){
                        printf("get QMC5883L Angle[0X%02X] error\r\n",Mag[i]);
                        return 1;
                    }
#endif
//        rt_thread_mdelay ( 20 );
//        printf("read imu reg_data 1:%02X, 2:%02X, 3:%02X, 4:%02X, 5:%02X ,6:%02X\r\n"
//                        ,val[0],val[1],val[2],val[3],val[4],val[5]);

    int32_t data_t[3] = {0};
            data_t[0] = (int16_t)((val[1] << 8) | val[0]);
            data_t[1] = (int16_t)((val[3] << 8) | val[2]);
            data_t[2] = (int16_t)((val[5] << 8) | val[4]);

            //数据处理
//            printf("DATA_READ_X = %d\t DATA_READ_Y = %d\t DATA_READ_Z = %d\n",data_t[0],data_t[1],data_t[2]);

            for(int i = 0;i < 3;i++)
            *(data + i ) = data_t[i];

        return 0;
}

/**磁力计原始数据计算航向角**/
float calculate_heading(float mag_x, float mag_y) {
    float heading = atan2(mag_y, mag_x) * (180.0 / M_PI);
    if (heading < 0) {
        heading += 360;
    }
    return heading;
}
