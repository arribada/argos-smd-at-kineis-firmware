/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_RESET);
#if !defined(SMD_STDALONE)
  /* See mcu_misc.c: on STDALONE, PC1 = TPS63901 SEL — leave high-Z. */
  HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
#endif
  /*Configure GPIO pins : PA12 PA11 PA0 PA6
                           PA7 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_0
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_9;

  /* Configure GPIOA pins to analog except :
   * PA1 = SPI_SCK
   * PA2 = UART_TX
   * PA3 = UART_RX
   * PA13 = SWDIO
   * PA14 = SWCLK
   * PA15 = SPI NSS
   *
   * Options available with Argos SMD
   * PA9 / PA10  set to analog (unused, high impedance)
   * PA11 = Set to analog but can be used to DBG_RF-NRST
   * PA12 = Set to analog but can be used to DBG_RF-BUSY
   */
#if defined(USE_SPI_DRIVER)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
		  	  	  	  	  GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
#if !defined(USE_UW_DOPPLER_APP)
                        | GPIO_PIN_11 | GPIO_PIN_12
#endif
                        ;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

#elif defined(USE_UART_DRIVER)
  GPIO_InitStruct.Pin = GPIO_PIN_0
#if !defined(SMD_STDALONE)
                        | GPIO_PIN_1    /* PA1 = LED_RED on STDALONE */
#endif
                        | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
		  	  	  	  	  GPIO_PIN_7 | GPIO_PIN_8
#if !defined(SMD_STDALONE)
                        | GPIO_PIN_9 | GPIO_PIN_10   /* PA9/PA10 = I2C on STDALONE */
#endif
#if !defined(USE_UW_DOPPLER_APP)
                        | GPIO_PIN_11 | GPIO_PIN_12
#endif
                        | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
#endif

  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIOB pins to analog except :
   * PB4 = SPI MISO
   * PB5 = SPI MOSI
   * Options available with Argos SMD
   * PB3 = DBG_SWO
   * PB6 = UART
   * PB7 = UART
   * PB9 = GPIO
   * PB13 = GPIO
  */

#if defined(USE_SPI_DRIVER)
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | //GPIO_PIN_3 |
                        GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_15;
#elif defined(USE_UART_DRIVER)
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | //GPIO_PIN_3 |
#if !defined(SMD_STDALONE)
                        GPIO_PIN_4 | GPIO_PIN_5 |  /* PB4=LED_GREEN, PB5=LED_BLUE on STDALONE */
                        GPIO_PIN_6 | GPIO_PIN_7 |  /* PB6=REED, PB7=PWR_LATCH on STDALONE */
#endif
                        GPIO_PIN_8 |
#if !defined(SMD_STDALONE)
                        GPIO_PIN_9 |               /* PB9=VBAT_EN on STDALONE */
#endif
                        GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_15;
#endif
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIOC pins to analog except :
   * PC0 = PA_PSU_EN (output)
   * PC1 = PA_PSU_SEL / VSEL (TPS63901 voltage select): driven HIGH at boot
   *       on STDALONE to guarantee 3V3 mode for MCU+radio+TCXO operation.
   *       The external R11 (10M to VBAT) alone is high-impedance and could
   *       droop under leakage — drive actively. Use MCU_MISC_VSEL_*() to
   *       switch to 1.8V power-save (only when radio is idle).
   */
  GPIO_InitStruct.Pin =        GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |
                        GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                        GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// Configured inside mcu_misc.c file but added here also to have pull up set as startup
//  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PA_PSU_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA_PSU_EN_GPIO_Port, &GPIO_InitStruct);

  /* PA_PSU_SEL / VSEL: drive HIGH actively at boot (3V3 mode). */
  HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = PA_PSU_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA_PSU_SEL_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);


}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
