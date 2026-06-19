// SPDX-License-Identifier: no SPDX license
/**
 * @file    kns_assert.c
 * @brief   assert handling dependency regarding platform using the Kineis stack (handle reset)
 * @author  Kinéis
 *
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Kineis. All rights reserved.</center></h2>
 *
 * This software component is licensed by Kineis under Ultimate Liberty license, the "License";
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * * kineis.com
 *
 */

/**
 * @addtogroup KINEIS_SW_CONF
 * @brief Kineis SW configuration regarding asserts
 * @{
 */

/* Includes ------------------------------------------------------------------------------------ */

#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "mgr_log.h"
#include "usart.h"
#include "stm32wlxx_hal.h"

#pragma GCC visibility push(default)

/* Function ------------------------------------------------------------------------------------ */

/**
  * @brief  Handle assert reported by Kineis stack,
  *
  * So far, reports the name of the source file and the source line number where the kns_assert
  * error has occurred. Then call same Error handler as the whole FW.
  *
  * @attention consider reset the UC or stop/restart the stack
  *
  * @param[in] file: pointer to the source file name
  * @param[in] line: kns_assert error line source number
  * @retval None
  */
void kns_assert_failed(uint8_t *file, uint32_t line)
{
  /* Kineis-stack assert — ERROR-grade. Gated at LOGLVL=NONE only so the
   * GUI parser sees a clean stream; lower levels still surface the fault. */
  if (MGR_LOG_passes(MGR_LOG_LVL_ERROR)) {
    static char ka_buf[168];
    int ka_n = snprintf(ka_buf, sizeof(ka_buf),
      "\r\n%s!!! KNS_ASSERT %s:%lu !!!\r\n",
      MGR_LOG_levelTag(MGR_LOG_LVL_ERROR),
      (file ? (const char *)file : "(null)"), (unsigned long)line);
    if (ka_n > 0)
      HAL_UART_Transmit(&hlpuart1, (uint8_t *)ka_buf, (uint16_t)ka_n, 200);
  }
  MGR_LOG_ERR("ASSERT FAIL: %ld %s\r\n", line, file);
  Error_Handler();
}

#pragma GCC visibility pop

/**
 * @}
 */
