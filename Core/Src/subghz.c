/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz.c
  * @brief   This file provides code for the configuration
  *          of the SUBGHZ instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "subghz.h"

/* USER CODE BEGIN 0 */
#include "stm32wlxx_ll_pwr.h"  /* For LL_PWR_SelectSUBGHZSPI_NSS */
#include "mgr_log.h"

/* SX126x Radio Commands */
#define RADIO_GET_STATUS            0xC0
#define RADIO_SET_STANDBY           0x80
#define RADIO_SET_SLEEP             0x84
#define RADIO_CLR_IRQSTATUS         0x02
#define RADIO_CLR_ERRORS            0x07

#define STDBY_RC                    0x00
#define STDBY_XOSC                  0x01
/* USER CODE END 0 */

SUBGHZ_HandleTypeDef hsubghz;

/* SUBGHZ init function */
void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */
  /* Full SubGHz cleanup - required after bootloader jump.
   * The bootloader doesn't reset the radio, leaving it in an unknown state
   * which prevents interrupts from firing properly. */

  /* 1. Disable SubGHz interrupt at NVIC level first */
  HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);

  /* 2. Clear pending NVIC interrupt */
  NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);

  /* 3. Disable SubGHz SPI clock before reset */
  __HAL_RCC_SUBGHZSPI_CLK_DISABLE();

  /* 4. Force reset the SubGHz peripheral */
  __HAL_RCC_SUBGHZSPI_FORCE_RESET();
  for (volatile int i = 0; i < 10000; i++); /* Longer delay for complete reset */
  __HAL_RCC_SUBGHZSPI_RELEASE_RESET();

  /* 5. Clear the handle state so HAL_SUBGHZ_Init starts fresh */
  hsubghz.State = HAL_SUBGHZ_STATE_RESET;
  hsubghz.ErrorCode = HAL_SUBGHZ_ERROR_NONE;

  /* 6. Ensure radio is not in deep sleep by toggling NSS */
  __HAL_RCC_SUBGHZSPI_CLK_ENABLE();
  LL_PWR_UnselectSUBGHZSPI_NSS();
  for (volatile int i = 0; i < 1000; i++);
  LL_PWR_SelectSUBGHZSPI_NSS();
  for (volatile int i = 0; i < 1000; i++);
  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */
  {
    HAL_StatusTypeDef status;
    uint8_t radio_status = 0;

    /* Debug: Print NVIC state for SubGHz interrupt */
    uint32_t nvic_iser = NVIC->ISER[1];  /* IRQ 44 is bit 12 of ISER[1] */
    uint32_t nvic_ispr = NVIC->ISPR[1];  /* Pending register */
    MGR_LOG_DEBUG("SUBGHZ NVIC: ISER[1]=0x%08lX (bit12=%d), ISPR[1]=0x%08lX\r\n",
                  nvic_iser, (nvic_iser >> 12) & 1, nvic_ispr);

    /* Debug: Print PWR state */
    MGR_LOG_DEBUG("SUBGHZ PWR: CR3=0x%08lX, SR2=0x%08lX, SUBGHZSPICR=0x%08lX\r\n",
                  PWR->CR3, PWR->SR2, PWR->SUBGHZSPICR);

    /* Wake up radio from sleep by toggling NSS */
    LL_PWR_UnselectSUBGHZSPI_NSS();
    for (volatile int i = 0; i < 5000; i++);
    LL_PWR_SelectSUBGHZSPI_NSS();
    for (volatile int i = 0; i < 5000; i++);

    /* Get radio status to check if it's responding */
    status = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &radio_status, 1);
    MGR_LOG_DEBUG("SUBGHZ GetStatus: HAL=%d, status=0x%02X\r\n", status, radio_status);

    /* Clear any pending radio errors */
    uint8_t clr_errors[2] = {0x00, 0x00};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_ERRORS, clr_errors, 2);

    /* Clear any pending radio IRQs */
    uint8_t clr_irq[2] = {0xFF, 0xFF};  /* Clear all IRQ flags */
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_IRQSTATUS, clr_irq, 2);

    /* Put radio in STANDBY_RC mode - known good state */
    uint8_t standby_cfg = STDBY_RC;
    status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &standby_cfg, 1);
    MGR_LOG_DEBUG("SUBGHZ SetStandby: HAL=%d\r\n", status);

    /* Get status again to verify */
    status = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &radio_status, 1);
    MGR_LOG_DEBUG("SUBGHZ GetStatus after reset: HAL=%d, status=0x%02X\r\n", status, radio_status);

    /* FORCE enable SubGHz interrupt - HAL_SUBGHZ_MspInit may not be called */
    MGR_LOG_DEBUG("Trying HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn=%d)...\r\n", SUBGHZ_Radio_IRQn);
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

    nvic_iser = NVIC->ISER[1];
    MGR_LOG_DEBUG("After HAL: ISER[1]=0x%08lX (bit12=%d)\r\n", nvic_iser, (nvic_iser >> 12) & 1);

    /* Try direct NVIC register write */
    MGR_LOG_DEBUG("Trying direct NVIC->ISER[1] write...\r\n");
    NVIC->ISER[1] = (1UL << 12);  /* Direct write for IRQ 44 */
    __DSB();  /* Data Synchronization Barrier */
    __ISB();  /* Instruction Synchronization Barrier */

    nvic_iser = NVIC->ISER[1];
    MGR_LOG_DEBUG("After direct write: ISER[1]=0x%08lX (bit12=%d)\r\n", nvic_iser, (nvic_iser >> 12) & 1);

    /* Check if something is clearing it - read ICER */
    uint32_t nvic_icer = NVIC->ICER[1];
    MGR_LOG_DEBUG("NVIC ICER[1]=0x%08lX\r\n", nvic_icer);

    /* Check VTOR - is vector table correct? */
    MGR_LOG_DEBUG("SCB->VTOR=0x%08lX (expected 0x08008100)\r\n", SCB->VTOR);

    /* Final check */
    nvic_iser = NVIC->ISER[1];
    if ((nvic_iser >> 12) & 1) {
      MGR_LOG_DEBUG("SUBGHZ IRQ ENABLED OK!\r\n");
    } else {
      MGR_LOG_DEBUG("SUBGHZ IRQ STILL DISABLED - NVIC BLOCKED!\r\n");
      /* Try one more time with CMSIS function */
      NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
      nvic_iser = NVIC->ISER[1];
      MGR_LOG_DEBUG("After NVIC_EnableIRQ: ISER[1]=0x%08lX (bit12=%d)\r\n", nvic_iser, (nvic_iser >> 12) & 1);
    }
  }
  /* USER CODE END SUBGHZ_Init 2 */

}

void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef* subghzHandle)
{

  /* USER CODE BEGIN SUBGHZ_MspInit 0 */

  /* USER CODE END SUBGHZ_MspInit 0 */
    /* SUBGHZ clock enable */
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();

    /* SUBGHZ interrupt Init */
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
  /* USER CODE BEGIN SUBGHZ_MspInit 1 */

  /* USER CODE END SUBGHZ_MspInit 1 */
}

void HAL_SUBGHZ_MspDeInit(SUBGHZ_HandleTypeDef* subghzHandle)
{

  /* USER CODE BEGIN SUBGHZ_MspDeInit 0 */

  /* USER CODE END SUBGHZ_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SUBGHZSPI_CLK_DISABLE();

    /* SUBGHZ interrupt Deinit */
    HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
  /* USER CODE BEGIN SUBGHZ_MspDeInit 1 */

  /* USER CODE END SUBGHZ_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
