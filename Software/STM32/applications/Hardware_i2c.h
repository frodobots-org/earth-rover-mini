/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-23     yuxing       the first version
 */
#ifndef APPLICATIONS_HARDWARE_I2C_H_
#define APPLICATIONS_HARDWARE_I2C_H_

#include "main.h"
#include <rtdbg.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_GPIO_Init(void);


#endif /* APPLICATIONS_HARDWARE_I2C_H_ */
