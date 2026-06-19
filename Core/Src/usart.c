/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#define LPUART1_EXTI_ENABLE_IT()   (EXTI->IMR1 |= EXTI_IMR1_IM28)

/* USER CODE END 0 */

UART_HandleTypeDef hlpuart1;

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
#if defined(USE_SPI_DRIVER) || defined(USE_UW_DOPPLER_APP)
  /* High-throughput console:
   *   - SPI mode      : SPI debug logs.
   *   - UW_DOPPLER    : heartbeat + AT command + crash replay traces are
   *                     verbose, and at 9600 baud a 120-byte heartbeat
   *                     takes 125 ms blocking → starves the main loop.
   * Both use HSI 16 MHz (see HAL_UART_MspInit below) which can sustain
   * 115200 cleanly. Cost: HSI must be on during transmission (already the
   * case in MONITORING since LPM=NONE). */
  hlpuart1.Init.BaudRate = 115200;
#else
  hlpuart1.Init.BaudRate = 9600;    /* UART mode (STDLN/GUI): low power */
#endif
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  hlpuart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  hlpuart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  LPUART1_EXTI_ENABLE_IT();
  /* USER CODE END LPUART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspInit 0 */

  /* USER CODE END LPUART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
#if defined(USE_SPI_DRIVER) || defined(USE_UW_DOPPLER_APP)
    /* HSI 16 MHz: 115200-capable (SPI debug logs + UW_DOPPLER heartbeat/AT) */
    PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
#else
    /* LSE 32.768 kHz: low-power 9600 baud (STDLN / GUI) */
    PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* LPUART1 clock enable */
    __HAL_RCC_LPUART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**LPUART1 GPIO Configuration
    PA2     ------> LPUART1_TX
    PA3     ------> LPUART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* LPUART1 interrupt Init */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspInit 1 */

  /* USER CODE END LPUART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspDeInit 0 */

  /* USER CODE END LPUART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPUART1_CLK_DISABLE();

    /**LPUART1 GPIO Configuration
    PA2     ------> LPUART1_TX
    PA3     ------> LPUART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* LPUART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspDeInit 1 */

  /* USER CODE END LPUART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#include <stdbool.h>
#include "mgr_at_cmd.h"
#include "stm32wlxx_hal.h"

static bool s_uart_enabled = false;

/* Last HAL tick at which a byte was received on LPUART1. Set by the RX
 * ISR (mcu_at_console_stm.c) via APP_UART_noteRxActivity(). Used by the
 * CONFIG-mode LED indicator to show "user is actively connected" vs
 * "session idle". Aligned uint32_t → single-word atomic on Cortex-M4. */
static volatile uint32_t s_last_rx_tick = 0;

void APP_UART_noteRxActivity(void)
{
  s_last_rx_tick = HAL_GetTick();
}

uint32_t APP_UART_lastRxTick(void)
{
  return s_last_rx_tick;
}

uint32_t APP_UART_msSinceRx(void)
{
  const uint32_t now = HAL_GetTick();
  const uint32_t last = s_last_rx_tick;
  if (last == 0u)
    return 0xFFFFFFFFu;  /* sentinel: no RX ever */
  return now - last;
}

void APP_UART_setEnabled(bool enabled)
{
  /* Use hlpuart1.gState as the source of truth so this call is safe to
   * make at any point, including before/after main.c's MX_LPUART1_UART_Init
   * direct call. Resyncs the cached flag transparently. */
  bool is_init = (hlpuart1.gState != HAL_UART_STATE_RESET);
  if (enabled && is_init) {
    s_uart_enabled = true;
    return;
  }
  if (!enabled && !is_init) {
    s_uart_enabled = false;
    return;
  }

  if (enabled) {
    MX_LPUART1_UART_Init();
    /* Re-register the AT command parser hook (MGR_AT_CMD_start records the
     * console handle the first time, so a second call is safe and binds the
     * stream callback after re-init). */
    (void)MGR_AT_CMD_start(&hlpuart1);
    s_uart_enabled = true;
  } else {
    HAL_UART_DeInit(&hlpuart1);
    /* HAL_UART_MspDeInit (called by HAL_UART_DeInit) puts PA2/PA3 to GPIO
     * mode (analog reset state). Disables LPUART1 IRQ as well. After this
     * call, direct-UART traces no-op because hlpuart1.gState == RESET. */
    s_uart_enabled = false;
  }
}

bool APP_UART_isEnabled(void)
{
  /* Trust the HAL state field as ground truth. The cached flag may diverge
   * if other code (e.g. main.c MX_LPUART1_UART_Init) bypasses the setter,
   * or after an error state. */
  return (hlpuart1.gState != HAL_UART_STATE_RESET);
}

/* USER CODE END 1 */
