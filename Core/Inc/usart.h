/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief Power-gate the LPUART1 peripheral for production OPERATIONAL mode.
 *
 * enabled=true  → MX_LPUART1_UART_Init() + restore the MGR_AT_CMD parser
 *                 hook. Use this when entering CONFIG mode so the AT command
 *                 surface becomes available.
 * enabled=false → HAL_UART_DeInit() + set PA2/PA3 to analog so the LPUART
 *                 + GPIO clock domains stop drawing current. Use this when
 *                 entering OPERATIONAL mode.
 *
 * Idempotent: re-calls with the same state are no-ops.
 *
 * @note Direct-UART traces (`HAL_UART_Transmit(&hlpuart1, ...)`) are gated on
 *       `hlpuart1.gState != HAL_UART_STATE_RESET` everywhere in the codebase,
 *       so calling this with `false` automatically silences them too.
 */
void APP_UART_setEnabled(bool enabled);

/** @brief Current UART gating state. */
bool APP_UART_isEnabled(void);

/** @brief Note that a byte has been received on LPUART1 (called from RX ISR). */
void APP_UART_noteRxActivity(void);

/** @brief HAL tick of the last RX byte (0 if none seen). */
uint32_t APP_UART_lastRxTick(void);

/** @brief Milliseconds since the last RX byte; UINT32_MAX if none seen. */
uint32_t APP_UART_msSinceRx(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

