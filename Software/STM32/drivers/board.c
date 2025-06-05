/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-29     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include "stm32f4xx_hal.h"

RT_WEAK void rt_hw_board_init ( )
{
	  extern void hw_board_init ( char *clock_src , int32_t clock_src_freq , int32_t clock_target_freq );

	  /* Heap initialization */
#if defined(RT_USING_HEAP)
	  rt_system_heap_init ( (void *) HEAP_BEGIN , (void *) HEAP_END );
#endif

	  hw_board_init ( BSP_CLOCK_SOURCE , BSP_CLOCK_SOURCE_FREQ_MHZ , BSP_CLOCK_SYSTEM_FREQ_MHZ );

	  /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
	  rt_console_set_device ( RT_CONSOLE_DEVICE_NAME );
#endif

	  /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
	  rt_components_board_init ( );
#endif

}

void HAL_MspInit ( void )
{
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */
    // 1. 解锁备份域访问权限
    HAL_PWR_EnableBkUpAccess();

    // 2. 使能RTC时钟 (RTC外设在备份域，需要使能时钟才能访问)
    __HAL_RCC_RTC_ENABLE();

    // 可选：检查RTC是否已经启动，避免重复初始化
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
    {
        rt_kprintf("RTC clock not ready!\n");
    }
    __HAL_RCC_SYSCFG_CLK_ENABLE()
    ;
    __HAL_RCC_PWR_CLK_ENABLE()
    ;

    /* System interrupt init*/

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit ( ADC_HandleTypeDef* hadc )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hadc->Instance==ADC1)
    {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_ADC1_CLK_ENABLE();

      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**ADC1 GPIO Configuration
      PA3     ------> ADC1_IN3
      PA4     ------> ADC1_IN4
      */
      GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
    }

}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit ( ADC_HandleTypeDef* hadc )
{
    if(hadc->Instance==ADC1)
    {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_ADC1_CLK_DISABLE();

      /**ADC1 GPIO Configuration
      PA3     ------> ADC1_IN3
      PA4     ------> ADC1_IN4
      */
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3|GPIO_PIN_4);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
    }

}

/**
 * @brief TIM_OC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspInit ( TIM_HandleTypeDef* htim_oc )
{
	  if ( htim_oc->Instance == TIM1 )
	  {
			/* USER CODE BEGIN TIM1_MspInit 0 */

			/* USER CODE END TIM1_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM1_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM1_MspInit 1 */

			/* USER CODE END TIM1_MspInit 1 */
	  }
	  else if ( htim_oc->Instance == TIM9 )
	  {
			/* USER CODE BEGIN TIM9_MspInit 0 */

			/* USER CODE END TIM9_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM9_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM9_MspInit 1 */

			/* USER CODE END TIM9_MspInit 1 */
	  }
	  else if ( htim_oc->Instance == TIM10 )
	  {
			/* USER CODE BEGIN TIM10_MspInit 0 */

			/* USER CODE END TIM10_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM10_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM10_MspInit 1 */

			/* USER CODE END TIM10_MspInit 1 */
	  }
	  else if ( htim_oc->Instance == TIM11 )
	  {
			/* USER CODE BEGIN TIM11_MspInit 0 */

			/* USER CODE END TIM11_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM11_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM11_MspInit 1 */

			/* USER CODE END TIM11_MspInit 1 */
	  }

}

/**
 * @brief TIM_Encoder MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_encoder: TIM_Encoder handle pointer
 * @retval None
 */
//void HAL_TIM_Encoder_MspInit ( TIM_HandleTypeDef* htim_encoder )
//{
//	  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//	  if ( htim_encoder->Instance == TIM2 )
//	  {
//			/* USER CODE BEGIN TIM2_MspInit 0 */
//
//			/* USER CODE END TIM2_MspInit 0 */
//			/* Peripheral clock enable */
//			__HAL_RCC_TIM2_CLK_ENABLE()
//			;
//
//			__HAL_RCC_GPIOA_CLK_ENABLE()
//			;
//			__HAL_RCC_GPIOB_CLK_ENABLE()
//			;
//			/**TIM2 GPIO Configuration
//			 PA15     ------> TIM2_CH1
//			 PB3     ------> TIM2_CH2
//			 */
//			GPIO_InitStruct.Pin = GPIO_PIN_15;
//			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//			GPIO_InitStruct.Pull = GPIO_NOPULL;
//			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//			GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//			HAL_GPIO_Init ( GPIOA , &GPIO_InitStruct );
//
//			GPIO_InitStruct.Pin = GPIO_PIN_3;
//			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//			GPIO_InitStruct.Pull = GPIO_NOPULL;
//			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//			GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//			HAL_GPIO_Init ( GPIOB , &GPIO_InitStruct );
//
//			/* USER CODE BEGIN TIM2_MspInit 1 */
//
//			/* USER CODE END TIM2_MspInit 1 */
//	  }
//	  else if ( htim_encoder->Instance == TIM3 )
//	  {
//			/* USER CODE BEGIN TIM3_MspInit 0 */
//
//			/* USER CODE END TIM3_MspInit 0 */
//			/* Peripheral clock enable */
//			__HAL_RCC_TIM3_CLK_ENABLE()
//			;
//
//			__HAL_RCC_GPIOB_CLK_ENABLE()
//			;
//			/**TIM3 GPIO Configuration
//			 PB4     ------> TIM3_CH1
//			 PB5     ------> TIM3_CH2
//			 */
//			GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
//			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//			GPIO_InitStruct.Pull = GPIO_NOPULL;
//			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//			GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//			HAL_GPIO_Init ( GPIOB , &GPIO_InitStruct );
//
//			/* USER CODE BEGIN TIM3_MspInit 1 */
//
//			/* USER CODE END TIM3_MspInit 1 */
//	  }
//	  else if ( htim_encoder->Instance == TIM4 )
//	  {
//			/* USER CODE BEGIN TIM4_MspInit 0 */
//
//			/* USER CODE END TIM4_MspInit 0 */
//			/* Peripheral clock enable */
//			__HAL_RCC_TIM4_CLK_ENABLE()
//			;
//
//			__HAL_RCC_GPIOD_CLK_ENABLE()
//			;
//			/**TIM4 GPIO Configuration
//			 PD12     ------> TIM4_CH1
//			 PD13     ------> TIM4_CH2
//			 */
//			GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
//			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//			GPIO_InitStruct.Pull = GPIO_NOPULL;
//			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//			GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//			HAL_GPIO_Init ( GPIOD , &GPIO_InitStruct );
//
//			/* USER CODE BEGIN TIM4_MspInit 1 */
//
//			/* USER CODE END TIM4_MspInit 1 */
//	  }
//	  else if ( htim_encoder->Instance == TIM5 )
//	  {
//			/* USER CODE BEGIN TIM5_MspInit 0 */
//
//			/* USER CODE END TIM5_MspInit 0 */
//			/* Peripheral clock enable */
//			__HAL_RCC_TIM5_CLK_ENABLE()
//			;
//
//			__HAL_RCC_GPIOA_CLK_ENABLE()
//			;
//			/**TIM5 GPIO Configuration
//			 PA0-WKUP     ------> TIM5_CH1
//			 PA1     ------> TIM5_CH2
//			 */
//			GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//			GPIO_InitStruct.Pull = GPIO_NOPULL;
//			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//			GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
//			HAL_GPIO_Init ( GPIOA , &GPIO_InitStruct );
//
//			/* USER CODE BEGIN TIM5_MspInit 1 */
//
//			/* USER CODE END TIM5_MspInit 1 */
//	  }
//
//}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit ( TIM_HandleTypeDef* htim_base )
{
	  if ( htim_base->Instance == TIM1 )
	  {
			/* USER CODE BEGIN TIM1_MspInit 0 */

			/* USER CODE END TIM1_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM1_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM1_MspInit 1 */

			/* USER CODE END TIM1_MspInit 1 */
	  }
	  else if ( htim_base->Instance == TIM9 )
	  {
			/* USER CODE BEGIN TIM9_MspInit 0 */

			/* USER CODE END TIM9_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM9_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM9_MspInit 1 */

			/* USER CODE END TIM9_MspInit 1 */
	  }
	  else if ( htim_base->Instance == TIM10 )
	  {
			/* USER CODE BEGIN TIM10_MspInit 0 */

			/* USER CODE END TIM10_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM10_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM10_MspInit 1 */

			/* USER CODE END TIM10_MspInit 1 */
	  }
	  else if ( htim_base->Instance == TIM11 )
	  {
			/* USER CODE BEGIN TIM11_MspInit 0 */

			/* USER CODE END TIM11_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM11_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM11_MspInit 1 */

			/* USER CODE END TIM11_MspInit 1 */
	  }
	  else if ( htim_base->Instance == TIM13 )
	  {
			/* USER CODE BEGIN TIM13_MspInit 0 */

			/* USER CODE END TIM13_MspInit 0 */
			/* Peripheral clock enable */
			__HAL_RCC_TIM13_CLK_ENABLE()
			;
			/* USER CODE BEGIN TIM13_MspInit 1 */

			/* USER CODE END TIM13_MspInit 1 */
	  }
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
#if 1
      if (htim_base->Instance==TIM2)
      {
      /* USER CODE BEGIN TIM2_MspInit 0 */

      /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM2 GPIO Configuration
        PB3     ------> TIM2_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;//GPIO_AF1_TIM2
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 3);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
      /* USER CODE BEGIN TIM2_MspInit 1 */

      /* USER CODE END TIM2_MspInit 1 */
      }
      else if(htim_base->Instance==TIM3)
      {
      /* USER CODE BEGIN TIM3_MspInit 0 */

      /* USER CODE END TIM3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM3 GPIO Configuration
        PA7     ------> TIM3_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* TIM3 interrupt Init */

      /* USER CODE BEGIN TIM3_MspInit 1 */
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 3);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
      /* USER CODE END TIM3_MspInit 1 */
      }
      else if(htim_base->Instance==TIM4)
      {
      /* USER CODE BEGIN TIM4_MspInit 0 */

      /* USER CODE END TIM4_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PD13     ------> TIM4_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* TIM4 interrupt Init */
        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 3);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
      /* USER CODE BEGIN TIM4_MspInit 1 */

      /* USER CODE END TIM4_MspInit 1 */
      }
      else if(htim_base->Instance==TIM5)
      {
      /* USER CODE BEGIN TIM5_MspInit 0 */

      /* USER CODE END TIM5_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM5 GPIO Configuration
        PA1     ------> TIM5_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* TIM5 interrupt Init */
        HAL_NVIC_SetPriority(TIM5_IRQn, 0, 3);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
      /* USER CODE BEGIN TIM5_MspInit 1 */

      /* USER CODE END TIM5_MspInit 1 */
      }
#endif
}

void HAL_TIM_MspPostInit ( TIM_HandleTypeDef* htim )
{
	  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	  if ( htim->Instance == TIM1 )
	  {
			/* USER CODE BEGIN TIM1_MspPostInit 0 */

			/* USER CODE END TIM1_MspPostInit 0 */
			__HAL_RCC_GPIOE_CLK_ENABLE()
			;
			/**TIM1 GPIO Configuration
			 PE9     ------> TIM1_CH1
			 PE11     ------> TIM1_CH2
			 PE13     ------> TIM1_CH3
			 PE14     ------> TIM1_CH4
			 */
			GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;// FIXME
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
			HAL_GPIO_Init ( GPIOE , &GPIO_InitStruct );

			/* USER CODE BEGIN TIM1_MspPostInit 1 */

			/* USER CODE END TIM1_MspPostInit 1 */
	  }
	  else if ( htim->Instance == TIM9 )
	  {
			/* USER CODE BEGIN TIM9_MspPostInit 0 */

			/* USER CODE END TIM9_MspPostInit 0 */

			__HAL_RCC_GPIOE_CLK_ENABLE()
			;
			/**TIM9 GPIO Configuration
			 PE5     ------> TIM9_CH1
			 PE6     ------> TIM9_CH2
			 */
			GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
			HAL_GPIO_Init ( GPIOE , &GPIO_InitStruct );

			/* USER CODE BEGIN TIM9_MspPostInit 1 */

			/* USER CODE END TIM9_MspPostInit 1 */
	  }
	  else if ( htim->Instance == TIM10 )
	  {
			/* USER CODE BEGIN TIM10_MspPostInit 0 */

			/* USER CODE END TIM10_MspPostInit 0 */

			__HAL_RCC_GPIOE_CLK_ENABLE()
			;
			/**TIM10 GPIO Configuration
			 PB8     ------> TIM10_CH1
			 */
			GPIO_InitStruct.Pin = GPIO_PIN_8;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
			HAL_GPIO_Init ( GPIOB , &GPIO_InitStruct );

			/* USER CODE BEGIN TIM10_MspPostInit 1 */

			/* USER CODE END TIM10_MspPostInit 1 */
	  }
	  else if ( htim->Instance == TIM11 )
	  {
			/* USER CODE BEGIN TIM11_MspPostInit 0 */

			/* USER CODE END TIM11_MspPostInit 0 */

			__HAL_RCC_GPIOB_CLK_ENABLE()
			;
			/**TIM11 GPIO Configuration
			 PB9     ------> TIM11_CH1
			 */
			GPIO_InitStruct.Pin = GPIO_PIN_9;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
			HAL_GPIO_Init ( GPIOB , &GPIO_InitStruct );

			/* USER CODE BEGIN TIM11_MspPostInit 1 */

			/* USER CODE END TIM11_MspPostInit 1 */
	  }

//	  //后加
//	  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9| GPIO_PIN_13;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_PULLUP;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	  HAL_GPIO_Init ( GPIOE , &GPIO_InitStruct );

}
/**
 * @brief TIM_OC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspDeInit ( TIM_HandleTypeDef* htim_oc )
{
	  if ( htim_oc->Instance == TIM1 )
	  {
			/* USER CODE BEGIN TIM1_MspDeInit 0 */

			/* USER CODE END TIM1_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM1_CLK_DISABLE();
			/* USER CODE BEGIN TIM1_MspDeInit 1 */

			/* USER CODE END TIM1_MspDeInit 1 */
	  }
	  else if ( htim_oc->Instance == TIM9 )
	  {
			/* USER CODE BEGIN TIM9_MspDeInit 0 */

			/* USER CODE END TIM9_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM9_CLK_DISABLE();
			/* USER CODE BEGIN TIM9_MspDeInit 1 */

			/* USER CODE END TIM9_MspDeInit 1 */
	  }
	  else if ( htim_oc->Instance == TIM10 )
	  {
			/* USER CODE BEGIN TIM10_MspDeInit 0 */

			/* USER CODE END TIM10_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM10_CLK_DISABLE();
			/* USER CODE BEGIN TIM10_MspDeInit 1 */

			/* USER CODE END TIM10_MspDeInit 1 */
	  }

}

/**
 * @brief TIM_Encoder MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_encoder: TIM_Encoder handle pointer
 * @retval None
 */
//void HAL_TIM_Encoder_MspDeInit ( TIM_HandleTypeDef* htim_encoder )
//{
//	  if ( htim_encoder->Instance == TIM2 )
//	  {
//			/* USER CODE BEGIN TIM2_MspDeInit 0 */
//
//			/* USER CODE END TIM2_MspDeInit 0 */
//			/* Peripheral clock disable */
//			__HAL_RCC_TIM2_CLK_DISABLE();
//
//			/**TIM2 GPIO Configuration
//			 PA15     ------> TIM2_CH1
//			 PB3     ------> TIM2_CH2
//			 */
//			HAL_GPIO_DeInit ( GPIOA , GPIO_PIN_15 );
//
//			HAL_GPIO_DeInit ( GPIOB , GPIO_PIN_3 );
//
//			/* USER CODE BEGIN TIM2_MspDeInit 1 */
//
//			/* USER CODE END TIM2_MspDeInit 1 */
//	  }
//	  else if ( htim_encoder->Instance == TIM3 )
//	  {
//			/* USER CODE BEGIN TIM3_MspDeInit 0 */
//
//			/* USER CODE END TIM3_MspDeInit 0 */
//			/* Peripheral clock disable */
//			__HAL_RCC_TIM3_CLK_DISABLE();
//
//			/**TIM3 GPIO Configuration
//			 PB4     ------> TIM3_CH1
//			 PB5     ------> TIM3_CH2
//			 */
//			HAL_GPIO_DeInit ( GPIOB , GPIO_PIN_4 | GPIO_PIN_5 );
//
//			/* USER CODE BEGIN TIM3_MspDeInit 1 */
//
//			/* USER CODE END TIM3_MspDeInit 1 */
//	  }
//	  else if ( htim_encoder->Instance == TIM4 )
//	  {
//			/* USER CODE BEGIN TIM4_MspDeInit 0 */
//
//			/* USER CODE END TIM4_MspDeInit 0 */
//			/* Peripheral clock disable */
//			__HAL_RCC_TIM4_CLK_DISABLE();
//
//			/**TIM4 GPIO Configuration
//			 PD12     ------> TIM4_CH1
//			 PD13     ------> TIM4_CH2
//			 */
//			HAL_GPIO_DeInit ( GPIOD , GPIO_PIN_12 | GPIO_PIN_13 );
//
//			/* USER CODE BEGIN TIM4_MspDeInit 1 */
//
//			/* USER CODE END TIM4_MspDeInit 1 */
//	  }
//	  else if ( htim_encoder->Instance == TIM5 )
//	  {
//			/* USER CODE BEGIN TIM5_MspDeInit 0 */
//
//			/* USER CODE END TIM5_MspDeInit 0 */
//			/* Peripheral clock disable */
//			__HAL_RCC_TIM5_CLK_DISABLE();
//
//			/**TIM5 GPIO Configuration
//			 PA0-WKUP     ------> TIM5_CH1
//			 PA1     ------> TIM5_CH2
//			 */
//			HAL_GPIO_DeInit ( GPIOA , GPIO_PIN_0 | GPIO_PIN_1 );
//
//			/* USER CODE BEGIN TIM5_MspDeInit 1 */
//
//			/* USER CODE END TIM5_MspDeInit 1 */
//	  }
//
//}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit ( TIM_HandleTypeDef* htim_base )
{
	  if ( htim_base->Instance == TIM1 )
	  {
			/* USER CODE BEGIN TIM1_MspDeInit 0 */

			/* USER CODE END TIM1_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM1_CLK_DISABLE();
			/* USER CODE BEGIN TIM1_MspDeInit 1 */

			/* USER CODE END TIM1_MspDeInit 1 */
	  }
	  else if ( htim_base->Instance == TIM9 )
	  {
			/* USER CODE BEGIN TIM9_MspDeInit 0 */

			/* USER CODE END TIM9_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM9_CLK_DISABLE();
			/* USER CODE BEGIN TIM9_MspDeInit 1 */

			/* USER CODE END TIM9_MspDeInit 1 */
	  }
	  else if ( htim_base->Instance == TIM10 )
	  {
			/* USER CODE BEGIN TIM10_MspDeInit 0 */

			/* USER CODE END TIM10_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM10_CLK_DISABLE();
			/* USER CODE BEGIN TIM10_MspDeInit 1 */

			/* USER CODE END TIM10_MspDeInit 1 */
	  }
	  else if ( htim_base->Instance == TIM11 )
	  {
			/* USER CODE BEGIN TIM11_MspDeInit 0 */

			/* USER CODE END TIM11_MspDeInit 0 */
			/* Peripheral clock disable */
			__HAL_RCC_TIM11_CLK_DISABLE();
			/* USER CODE BEGIN TIM11_MspDeInit 1 */

			/* USER CODE END TIM11_MspDeInit 1 */
	  }
#if 0
      else if(htim_base->Instance==TIM2)
      {
      /* USER CODE BEGIN TIM2_MspDeInit 0 */

      /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /**TIM2 GPIO Configuration
        PB3     ------> TIM2_CH2
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        /* TIM2 interrupt DeInit */

      /* USER CODE BEGIN TIM2_MspDeInit 1 */

      /* USER CODE END TIM2_MspDeInit 1 */
      }
      else if(htim_base->Instance==TIM3)
      {
      /* USER CODE BEGIN TIM3_MspDeInit 0 */

      /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();

        /**TIM3 GPIO Configuration
        PA7     ------> TIM3_CH2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
        /* TIM3 interrupt DeInit */

      /* USER CODE BEGIN TIM3_MspDeInit 1 */

      /* USER CODE END TIM3_MspDeInit 1 */
      }
      else if(htim_base->Instance==TIM4)
      {
      /* USER CODE BEGIN TIM4_MspDeInit 0 */

      /* USER CODE END TIM4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /**TIM4 GPIO Configuration
        PD13     ------> TIM4_CH2
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_13);
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        /* TIM4 interrupt DeInit */

      /* USER CODE BEGIN TIM4_MspDeInit 1 */

      /* USER CODE END TIM4_MspDeInit 1 */
      }
      else if(htim_base->Instance==TIM5)
      {
      /* USER CODE BEGIN TIM5_MspDeInit 0 */

      /* USER CODE END TIM5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM5_CLK_DISABLE();

        /**TIM5 GPIO Configuration
        PA1     ------> TIM5_CH2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        /* TIM5 interrupt DeInit */

      /* USER CODE BEGIN TIM5_MspDeInit 1 */

      /* USER CODE END TIM5_MspDeInit 1 */
      }

#endif
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    HAL_TIM_Base_MspInit(tim_baseHandle);
}


void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11    ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }

}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC3     ------> SPI2_MOSI
    PB13     ------> SPI2_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC3     ------> SPI2_MOSI
    PB13     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }

}
