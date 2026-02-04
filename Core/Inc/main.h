/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);

/**
 * @brief Request DFU mode and reset to bootloader
 *
 * This function sets the DFU request flag in SRAM2 (backup RAM) and triggers
 * a system reset. After reset, the application will detect the flag and
 * jump to the bootloader for firmware update.
 *
 * @note Call this function from SPI command handler when DFU is requested.
 * @note SRAM2 is preserved across system reset.
 */
void request_dfu_mode(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JTMS_SWCLK_Pin GPIO_PIN_14
#define JTMS_SWCLK_GPIO_Port GPIOA
#define JTMS_SWDIO_Pin GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port GPIOA
#define EXT_WKUP_BUTTON_Pin GPIO_PIN_3
#define EXT_WKUP_BUTTON_GPIO_Port GPIOB
#define PA_PSU_EN_Pin GPIO_PIN_0
#define PA_PSU_EN_GPIO_Port GPIOC
#define PA_PSU_SEL_Pin GPIO_PIN_1
#define PA_PSU_SEL_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
