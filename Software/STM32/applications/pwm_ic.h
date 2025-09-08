/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-25     Administrator       the first version
 */

 #ifndef APPLICATIONS_PWM_IC_H_
 #define APPLICATIONS_PWM_IC_H_
 
 #include <rtthread.h>
 #include <board.h>
 
 /* ----------------- Global Variables ----------------- */
 uint16_t PWM_RisingCount = 0;              // Counter for rising edges
 volatile uint32_t PWM_FallingCount_2 = 0;  // Captured falling edge count TIM2
 volatile uint32_t PWM_FallingCount_3 = 0;  // Captured falling edge count TIM3
 volatile uint32_t PWM_FallingCount_4 = 0;  // Captured falling edge count TIM4
 volatile uint32_t PWM_FallingCount_5 = 0;  // Captured falling edge count TIM5
 
 float duty = -1;        // Calculated duty cycle (percentage, -1 = invalid)
 uint32_t uiDutyCycle;   // Duty cycle (ticks)
 uint32_t uiCycle;       // PWM period (ticks)
 uint32_t uiFrequency;   // Frequency (Hz)
 
 /* Timer handle structures for each TIM peripheral */
 TIM_HandleTypeDef htim2;
 TIM_HandleTypeDef htim3;
 TIM_HandleTypeDef htim4;
 TIM_HandleTypeDef htim5;
 
 /* ----------------- TIM2 Initialization ----------------- */
 /**
  * @brief  Initialize TIM2 for PWM input capture
  *         Configured to measure duty cycle and frequency.
  */
 void MX_TIM2_Init(void)
 {
     TIM_ClockConfigTypeDef sClockSourceConfig = {0};
     TIM_SlaveConfigTypeDef sSlaveConfig = {0};
     TIM_IC_InitTypeDef sConfigIC = {0};
     TIM_MasterConfigTypeDef sMasterConfig = {0};
 
     /* Base timer configuration */
     htim2.Instance = TIM2;
     htim2.Init.Prescaler = 84 - 1;                      // Prescaler for 1 MHz (assuming 84 MHz APB1 timer clock)
     htim2.Init.CounterMode = TIM_COUNTERMODE_UP;        // Count up
     htim2.Init.Period = 65535;                          // Max auto-reload
     htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // No division
     htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 
     if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
         Error_Handler();
     }
 
     /* Internal clock source */
     sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
     if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
         Error_Handler();
     }
 
     /* Input capture mode */
     if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
         Error_Handler();
     }
 
     /* Configure TIM2 as slave: reset mode on rising edge */
     sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
     sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;            // Use TI1FP1 as trigger
     sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
     sSlaveConfig.TriggerFilter = 0x03;                    // Small digital filter
     if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
         Error_Handler();
     }
 
     /* Master mode disabled */
     sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
     sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
     if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
         Error_Handler();
     }
 
     /* Input capture channel config: Falling edge on CH2 */
     sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
     sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
     sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
     sConfigIC.ICFilter = 0x03;
     if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
         Error_Handler();
     }
 
     /* Start base timer and input capture interrupts */
     HAL_TIM_Base_Start_IT(&htim2);
     HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);  // Enable TIM2 CH2 input capture
 }
 
 /* --------------------------------------------------------------------
    NOTE: The following functions follow the SAME SCHEMA as MX_TIM2_Init:
          - MX_TIM3_Init(void)
          - MX_TIM4_Init(void)
          - MX_TIM5_Init(void)
 
    Each initializes its respective TIM peripheral for PWM input capture
    with nearly identical configuration (prescaler, counter, slave mode,
    and input capture settings). Only the timer instance (TIM3, TIM4, TIM5)
    and capture channel(s) differ.
    -------------------------------------------------------------------- */
 
 


void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
//  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
//  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
//  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
//  sSlaveConfig.TriggerFilter = 0;
//  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x03;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
//  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
//    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
//    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//    sConfigIC.ICFilter = 0;
//    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
//    {
//      Error_Handler();
//    }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  /* USER CODE BEGIN TIM2_Init 2 */
//  HAL_TIM_IC_Init(&htim2);
//  HAL_TIM_IC_ConfigChannel(&htim2, &TIM2_CH2Config, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);  /* 使能定时2通道2的PWM输入捕获 */

  /* USER CODE END TIM2_Init 2 */

}

void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  /**选择从模式: 复位模式* */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x00;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);  /* 使能定时2通道2的PWM输入捕获 */

  /* USER CODE END TIM2_Init 2 */

}

void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x00;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start_IT(&htim5);                /**启动TIM5定时器的中断模式**/

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  /**以中断方式打开定时5通道2的PWM输入捕获 **/

  /* USER CODE END TIM2_Init 2 */

}

/**
 * @brief  Timer Input Capture Callback
 *         Called automatically by HAL when a capture event occurs
 *         on any configured TIM input channel.
 *
 * @param  htim: Pointer to the active timer handle
 * @note   This function handles TIM2, TIM3, TIM4, and TIM5 input
 *         capture events. Each timer has two channels configured:
 *         - Channel 1: Rising edge (pulse width / duty measurement)
 *         - Channel 2: Falling edge (period measurement)
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* ----------------- TIM2 ----------------- */
    if (htim->Instance == htim2.Instance)
    {
        switch (htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                /* Capture rising edge count (used for duty cycle) */
                PWM_RisingCount = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//                duty = (float)PWM_FallingCount_2 / PWM_RisingCount * 100.0f;
                break;

            case HAL_TIM_ACTIVE_CHANNEL_2:
                /* Capture falling edge only if valid (basic noise filter) */
                if (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500)
                    PWM_FallingCount_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                /* Reset counter for next measurement */
                __HAL_TIM_SET_COUNTER(htim, 0);
                break;

            default:
                break;
        }
    }

    /* ----------------- TIM3 ----------------- */
    if (htim->Instance == htim3.Instance)
    {
        switch (htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                PWM_RisingCount = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//                duty = (float)PWM_FallingCount_3 / PWM_RisingCount * 100.0f;
                break;

            case HAL_TIM_ACTIVE_CHANNEL_2:
                if (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500)
                    PWM_FallingCount_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                __HAL_TIM_SET_COUNTER(htim, 0);
                break;

            default:
                break;
        }
    }

    /* ----------------- TIM4 ----------------- */
    if (htim->Instance == htim4.Instance)
    {
        switch (htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                PWM_RisingCount = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//                duty = (float)PWM_FallingCount_4 / PWM_RisingCount * 100.0f;
                break;

            case HAL_TIM_ACTIVE_CHANNEL_2:
                if (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500) // simple filter
                    PWM_FallingCount_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                __HAL_TIM_SET_COUNTER(htim, 0);
                break;

            default:
                break;
        }
    }

    /* ----------------- TIM5 ----------------- */
    if (htim->Instance == htim5.Instance)
    {
        switch (htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                PWM_RisingCount = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
//                duty = (float)PWM_FallingCount_5 / PWM_RisingCount * 100.0f;
                break;

            case HAL_TIM_ACTIVE_CHANNEL_2:
                if (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500) // simple filter
                    PWM_FallingCount_5 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);

                __HAL_TIM_SET_COUNTER(&htim5, 0);
                break;

            default:
                break;
        }
    }
}

/* --------------------------------------------------------------------
   NOTE:
   - Each TIMx handler follows the same schema:
     * Channel 1 → capture rising edge (pulse width for duty cycle)
     * Channel 2 → capture falling edge (period for frequency)
     * Apply a simple >500 tick threshold as noise filter
     * Reset timer counter after capture for fresh measurements
   - Duty cycle calculation (commented out) can be enabled once both
     rising and falling values are reliably captured.
   -------------------------------------------------------------------- */

//INIT_APP_EXPORT(MX_TIM2_Init);
//INIT_APP_EXPORT(MX_TIM5_Init);

#endif /* APPLICATIONS_PWM_IC_H_ */