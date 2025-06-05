/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-06     yuxing       the first version
 */
#define DBG_TAG "MOTOR_TEST"
#define DBG_LVL DBG_LOG
#include "main.h"


void motor_test_thread_entry ( void *parameter )
{
    while(1)
    {
        robot_state.speed = 100;
        robot_state.steer = 0;
        rt_kprintf("speed = %d,steer = %d\n",robot_state.speed,robot_state.steer);
        rt_thread_mdelay ( 3000 );
        robot_state.speed = -100;
        robot_state.steer = 0;
        rt_kprintf("speed = %d,steer = %d\n",robot_state.speed,robot_state.steer);
        rt_thread_mdelay ( 3000 );
    }

}
