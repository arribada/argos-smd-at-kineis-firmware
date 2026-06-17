/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include "lpm.h"

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  switch (LPM_getMode()) {
  case LOW_POWER_MODE_SHUTDOWN:
  case LOW_POWER_MODE_STANDBY:
   /* Init from Shutdown/Standby mode wake-up. All registers (except the ones in the Backup
    * domain: RTC,the Backup registers, RCC Backup domain control register) are reset.
    * Global variable 'hrtc' was reset as part of RAM, need to init it again. An alternative
    * could be to store this global variable 'hrtc' in retention area (i.e. rtc backup registers)
    */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
    hrtc.Init.BinMode = RTC_BINARY_NONE;

    /* Restore RTC interrupt init. This part of HAL_RTC_MspInit (called in HAL_RTC_Init) usually */
    HAL_RTC_MspInit(&hrtc);

    /* RTC Alarm Interrupt Configuration: EXTI configuration
     * \todo This is quick fix, to be clean-up through issue
     * https://gitlab.com/kineis/segment-utilisateur/project_group/fw/kim2_fw/-/issues/236
     * An init function of MGR_RTC_ALARM shuold be called instead
     * RTC WKUP timer strategy is not that fixed yet. So far it is only used by the MGR_ARGOS.
     */
    __HAL_RTC_ALARM_EXTI_ENABLE_IT();
    __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();

    return;
  case LOW_POWER_MODE_SLEEP:
  case LOW_POWER_MODE_STOP:
    /* no impact on RTC peripheral nor hrtc global variable */
    return;
  case LOW_POWER_MODE_NONE:
  default:
    /* init from power off, do a full init of RTC peripheral and variable */
    break;
  }
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 5, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /** Disable wakeup (from auto-generated code above), it will be done later through MCU wrapper */
  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    /* LSI fallback when the LSE crystal failed to start (SystemClock_Config sets
     * g_rtc_use_lsi). Keeps the RTC — and thus every timed wake — alive on a
     * dead 32 kHz crystal instead of bricking a sealed unit. */
    extern volatile uint8_t g_rtc_use_lsi;
    PeriphClkInitStruct.RTCClockSelection =
        g_rtc_use_lsi ? RCC_RTCCLKSOURCE_LSI : RCC_RTCCLKSOURCE_LSE;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
