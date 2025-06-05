/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-20     Aaron       the first version
 */
#ifndef APPLICATIONS_STATE_H_
#define APPLICATIONS_STATE_H_

typedef enum {
    SYSTEM_STATE_CHARGING,     // 系统充电中
    SYSTEM_STATE_CHARGED,      // 系统充电完毕
    SYSTEM_STATE_INITIAL,      // 系统初始状态
    SYSTEM_STATE_RUNNING,      // 系统运行中
    SYSTEM_STATE_WARNING,      // 系统电量警告
    SYSTEM_STATE_SHUTDOWN,     // 系统关机停止运行
} system_state_t;

void robot_power_init ( void );
void robot_charge_init( void );
void robot_charge_task( void );
extern rt_mutex_t state_data_mutex;

#endif /* APPLICATIONS_STATE_H_ */
