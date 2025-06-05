/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-18     Administrator       the first version
 */
#ifndef APPLICATIONS_PWM_IC_H_
#define APPLICATIONS_PWM_IC_H_

#include <rtthread.h>
#include <board.h>

uint16_t PWM_RisingCount=0;
volatile uint32_t PWM_FallingCount_2=0,PWM_FallingCount_3=0,PWM_FallingCount_4=0,PWM_FallingCount_5=0;
float duty = -1;

uint32_t uiDutyCycle;
uint32_t uiCycle;
uint32_t uiFrequency;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;


/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0x03;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x03;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);  /* 使能定时2通道2的PWM输入捕获 */

  /* USER CODE END TIM2_Init 2 */

}
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim2.Instance)
    {
        switch(htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                PWM_RisingCount = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);      /* 占空比 */
//                duty = (float)PWM_FallingCount / PWM_RisingCount*100.00;

                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                if(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500)
                PWM_FallingCount_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);          /* 周期 */
                __HAL_TIM_SET_COUNTER(htim,0);
//                state_value.state_value_2 = 1;
//                HAL_NVIC_DisableIRQ(TIM2_IRQn);
//                TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);


                break;
            default:break;
        }
    }
    if(htim->Instance == htim3.Instance)
        {
            switch(htim->Channel)
            {
                case HAL_TIM_ACTIVE_CHANNEL_1:
                    PWM_RisingCount = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);      /* 占空比 */
//                    duty = (float)PWM_FallingCount / PWM_RisingCount*100.00;

                    break;
                case HAL_TIM_ACTIVE_CHANNEL_2:
                    if(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500)
                    PWM_FallingCount_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);          /* 周期 */
                    __HAL_TIM_SET_COUNTER(htim,0);
//                    state_value.state_value_3 = 1;
    //                TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);
//                    HAL_NVIC_DisableIRQ(TIM3_IRQn);

                    break;
                default:break;
            }
        }
    if(htim->Instance == htim4.Instance)
           {
               switch(htim->Channel)
               {
                   case HAL_TIM_ACTIVE_CHANNEL_1:
                       PWM_RisingCount = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);      /* 占空比 */
//                       duty = (float)PWM_FallingCount / PWM_RisingCount*100.00;

                       break;
                   case HAL_TIM_ACTIVE_CHANNEL_2:
                       if( HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500 )//滤波
                       PWM_FallingCount_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);          /* 周期 */
                       __HAL_TIM_SET_COUNTER(htim,0);
//                       state_value.state_value_4 = 1;
       //                TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);
//                       HAL_NVIC_DisableIRQ(TIM4_IRQn);

                       break;
                   default:break;
               }
           }
    if(htim->Instance == htim5.Instance)
               {
                   switch(htim->Channel)
                   {
                       case HAL_TIM_ACTIVE_CHANNEL_1:
                           PWM_RisingCount = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);      /* 占空比 */
//                           duty = (float)PWM_FallingCount / PWM_RisingCount*100.00;

                           break;
                       case HAL_TIM_ACTIVE_CHANNEL_2:
                           if( HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) > 500 )//滤波
                           PWM_FallingCount_5 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);          /* 周期 */
                           __HAL_TIM_SET_COUNTER(&htim5,0);
//                           state_value.state_value_5 = 1;
           //                TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);
//                           HAL_NVIC_DisableIRQ(TIM5_IRQn);

                           break;
                       default:break;
                   }
               }
}

//INIT_APP_EXPORT(MX_TIM2_Init);
//INIT_APP_EXPORT(MX_TIM5_Init);

#endif /* APPLICATIONS_PWM_IC_H_ */
