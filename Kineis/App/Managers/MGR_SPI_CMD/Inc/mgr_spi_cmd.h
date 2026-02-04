/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_spi_cmd.h
 * @brief SPI command manager - Pipelined Single-Transaction Protocol.
 *
 * This module implements a pipelined protocol where each transaction
 * contains the response to the previous command. No timing issues.
 *
 * @author Arribada
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_SPI_CMD_H
#define __MGR_SPI_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include "kns_types.h"
#include "kineis_sw_conf.h"
#include "mcu_spi_driver.h"
#include "mgr_spi_cmd_list.h"

/* Defines -------------------------------------------------------------------*/

/** @brief SPI critical path logging control
 *  Define SPI_VERBOSE_LOG to enable verbose logging in SPI critical path.
 *  DISABLED by default to reduce UART traffic.
 */
//#define SPI_VERBOSE_LOG 1

#ifdef SPI_VERBOSE_LOG
#define SPI_LOG_VERBOSE(fmt, ...) MGR_LOG_DEBUG(fmt, ##__VA_ARGS__)
#else
#define SPI_LOG_VERBOSE(fmt, ...) ((void)0)
#endif

/* Extern Variables ----------------------------------------------------------*/
extern CmdValue cmdInProgress;   /**< Current SPI command in progress */

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Start the SPI command manager.
 *
 * @param[in] context Handler or pointer to the required hardware settings.
 * @retval true if successfully started, false otherwise.
 */
bool MGR_SPI_CMD_start(void *context);

/**
 * @brief Process events from the Kineis stack as responses to SPI commands.
 *
 * @retval KNS_STATUS_OK if successful, error code otherwise.
 */
enum KNS_status_t MGR_SPI_CMD_macEvtProcess(void);

/**
 * @brief Handle state transitions for the SPI command manager.
 *
 * This function processes SPI transactions using the pipelined protocol.
 * Call this regularly from the main loop.
 */
void MGR_SPI_CMD_state_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MGR_SPI_CMD_H */

/**
 * @}
 */
