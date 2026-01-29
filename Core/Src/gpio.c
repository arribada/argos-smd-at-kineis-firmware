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
     PA10   ------> I2C1_SDA
     PA9   ------> I2C1_SCL
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
  HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
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
   * PA9 / PA10  set to analog but can be used with I2C
   * PA11 = Set to analog but can be used to DBG_RF-NRST
   * PA12 = Set to analog but can be used to DBG_RF-BUSY
   */
#if defined(USE_SPI_DRIVER)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
		  	  	  	  	  GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                        GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

#elif defined(USE_UART_DRIVER)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
		  	  	  	  	  GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
						  GPIO_PIN_12 | GPIO_PIN_15;
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
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_15;
#endif
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIOC pins to analog except :
   * PC0 = PA_PSU_EN (output)
   * PC1 = PA_PSU_SEL (output)
   */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
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
//  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PA_PSU_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA_PSU_SEL_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);

  /* Configure PA9 and PA10 as analog (Hi-Z when off) */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);


}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
