/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-16     luozs        the first version
 */
#include "main.h"
#include <math.h>

#define DBG_TAG "strategy"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

rt_timer_t overload_timer;    // Timer used for overload / locked-rotor protection checks
rt_timer_t cool_timer;        // Timer for cooling period before restart is allowed

rt_bool_t volatile protect_flag = RT_FALSE;        // Flag: system currently in protection state
rt_bool_t volatile nedd_control_flag = RT_FALSE;   // Flag: whether protection has already triggered control logic
rt_uint32_t continue_time = 0;                     // Counter for how long a fault condition has persisted


/**
 * @brief  Main protection thread.
 *         Monitors robot speed, steering, and motor RPMs.
 *         Detects conditions such as locked rotor or overload,
 *         and engages protective shutdown + cooldown timer if needed.
 */
void protect_overload_thread_entry(void *parameter)
{
    while (1)
    {
        // If voltage is too low, assume system hasn't powered up yet → no protection logic
        if (robot_state.voltage <= 81)
        {
            robot_state.speed = 0;
            robot_state.steer = 0;
            LOG_W("Control system power: %d, Execution system power: %d. "
                  "System may not be powered on, please check power source!", 
                  robot_state.pwr, 0);
        }
        else
        {
            // Only monitor if speed/steering commands are active AND power is enabled
            if ((robot_state.speed != 0 || robot_state.steer != 0) && robot_state.pwr == 1)
            {
                // ============================
                // Locked-rotor condition check
                // ============================
                if ((robot_state.rpm[0] == 0) || (robot_state.rpm[1] == 0) ||
                    (robot_state.rpm[2] == 0) || (robot_state.rpm[3] == 0))
                {
                    continue_time++;
                    if (continue_time >= LOCKED_ROTOR_TIME)  // RPM stuck at 0 for > LOCKED_ROTOR_TIME (≈ 8s)
                    {
                        robot_state.speed = 0;
                        robot_state.steer = 0;
                        protect_flag = RT_TRUE;

                        if (nedd_control_flag == RT_FALSE)
                        {
                            robot_state.locked_rotor_num++;
                            nedd_control_flag = RT_TRUE;
                            rt_timer_start(cool_timer);
                            LOG_W("Locked-rotor detected, stopping motors and starting cooldown. "
                                  "Locked-rotor count: %d", robot_state.locked_rotor_num);
                        }
                    }
                }

                // ============================
                // Overload check - Normal speed
                // ============================
                if (((abs(robot_state.speed) != 0) && (abs(robot_state.speed) <= 75)) ||
                    ((abs(robot_state.steer) != 0) && (abs(robot_state.steer) <= 75)))
                {
                    if (abs(robot_state.rpm[0]) <= NOR_SPEED_OVERLOAD_RPM ||
                        abs(robot_state.rpm[1]) <= NOR_SPEED_OVERLOAD_RPM ||
                        abs(robot_state.rpm[2]) <= NOR_SPEED_OVERLOAD_RPM ||
                        abs(robot_state.rpm[3]) <= NOR_SPEED_OVERLOAD_RPM)
                    {
                        continue_time++;
                        if (continue_time >= OVER_LOADER_TIME)  // Condition persists for > OVER_LOADER_TIME (≈ 16s)
                        {
                            robot_state.speed = 0;
                            robot_state.steer = 0;
                            protect_flag = RT_TRUE;

                            if (nedd_control_flag == RT_FALSE)
                            {
                                robot_state.over_loader_num++;
                                nedd_control_flag = RT_TRUE;
                                rt_timer_start(cool_timer);
                                LOG_W("Normal-speed overload detected, stopping motors and starting cooldown. "
                                      "Overload count: %d", robot_state.over_loader_num);
                            }
                        }
                    }
                }

                // ============================
                // Overload check - High speed
                // ============================
                if (((abs(robot_state.speed) != 0) && (abs(robot_state.speed) > 75)) ||
                    ((abs(robot_state.steer) != 0) && (abs(robot_state.steer) > 75)))
                {
                    if (abs(robot_state.rpm[0]) <= FAST_SPEED_OVERLOAD_RPM ||
                        abs(robot_state.rpm[1]) <= FAST_SPEED_OVERLOAD_RPM ||
                        abs(robot_state.rpm[2]) <= FAST_SPEED_OVERLOAD_RPM ||
                        abs(robot_state.rpm[3]) <= FAST_SPEED_OVERLOAD_RPM)
                    {
                        continue_time++;
                        if (continue_time >= OVER_LOADER_TIME)  // Condition persists for > OVER_LOADER_TIME (≈ 16s)
                        {
                            robot_state.speed = 0;
                            robot_state.steer = 0;
                            protect_flag = RT_TRUE;

                            if (nedd_control_flag == RT_FALSE)
                            {
                                robot_state.over_loader_num++;
                                nedd_control_flag = RT_TRUE;
                                rt_timer_start(cool_timer);
                                LOG_W("High-speed overload detected, stopping motors and starting cooldown. "
                                      "Overload count: %d", robot_state.over_loader_num);
                            }
                        }
                    }
                }
            }
            else
            {
                // If commands are not active or fault condition not sustained → reset monitoring
                rt_thread_delay(100);

                if ((robot_state.rpm[0] != 0) && (robot_state.rpm[1] != 0) &&
                    (robot_state.rpm[2] != 0) && (robot_state.rpm[3] != 0))
                {
                    continue_time = 0;
                    protect_flag = RT_FALSE;
                }
            }
        }
        rt_thread_delay(100);
    }
}

/**
 * @brief  Timer callback when cooling period expires.
 *         Resets flags so system can restart safely.
 */
static void cool_timer_timeout(void *parameter)
{
    continue_time = 0;
    protect_flag = RT_FALSE;
    nedd_control_flag = RT_FALSE;
    LOG_I("Cooldown period complete, restart allowed!");
}

/**
 * @brief  Initializes strategy module.
 *         Creates the cooldown timer (one-shot, 3 seconds).
 */
void strategy_init(void)
{
    cool_timer = rt_timer_create("cool_timer", 
                                 cool_timer_timeout, 
                                 RT_NULL, 
                                 3000, 
                                 RT_TIMER_FLAG_ONE_SHOT);
}
