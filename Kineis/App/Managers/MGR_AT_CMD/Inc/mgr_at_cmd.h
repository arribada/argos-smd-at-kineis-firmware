/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_at_cmd.h
 * @author  Kineis
 * @brief AT commands manager main header file
 */

/**
 * @page mgr_at_cmd_page AT commands manager
 *
 * The AT cmd manager is in charge of parsing an incoming data stream and extract AT commands. Then
 * call the corresponding handler.
 * This version of code is support AT command set v0.6.
 *
 * * mgr_at_cmd_list.c   contains the **list of handlers for each AT cmd** (cf array
 *                       \ref cas_atcmd_list_array)
 * * mgr_at_cmd_common.h contains internal common utilities such as logging functions and the API
 *                       to send AT responses over the console.
 * * mgr_at_cmd.h        is the main header file for AT cmd manager core.
 * * mgr_at_cmd_list.h   is the main entry point to get the list of suported commands
 *                       (cf enum \ref atcmd_idx_t)
 *
 * @section mgr_at_cmd_subpages Sub-pages
 *
 * * @subpage user_data_page
 * * @subpage strutil_lib_page
 * * @subpage mcu_at_console_page
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_AT_CMD_H
#define __MGR_AT_CMD_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include "kns_types.h"

/*Functions ------------------------------------------------------------------*/

/**
 * @brief main API used to start AT command Manager
 *
 * This function will internally call MCU wrapper to start AT CMD console (such as UART RX isr
 * enabling)
 *
 * @param[in] context some handler on required HW setting.
 *            @note this handler will be directly sent as is, to MCU wrappers. Thus application can
 *            change its type as per needs.
 *
 * @retval true if AT command manager is correctly started, false otherwise
 */
bool MGR_AT_CMD_start(void *context);

/**
 * @brief API used to check there is some AT command in internal fifo
 *
 * @retval true if there is some AT command in fifo, false otherwise
 */
bool MGR_AT_CMD_isPendingAt(void);

/**
 * @brief API used to get next AT command stored in internal fifo
 *
 * @retval Pointer to the AT cmd to be decoded
 */
uint8_t *MGR_AT_CMD_popNextAt(void);

/**
 * @brief Decode and execute AT cmd if valid
 *
 * @param[in] pu8_atcmd pointer to AT command
 *
 * @retval true if command was decoded and correctly executed, false otherwise
 */
bool MGR_AT_CMD_decodeAt(uint8_t *pu8_atcmd);

/**
 * @brief Tick (HAL_GetTick) of the last decoded AT command, 0 if none yet.
 *
 * LPM schedulers use this as an "operator is on the console" signal to hold
 * off deep sleep while AT traffic is recent.
 */
uint32_t MGR_AT_CMD_getLastActivityTick(void);

/**
 * @brief Fct used to retrieve and process event coming from kineis stack as answers to AT commands
 *
 * Typically, this is about processing TX events such as TX-done, TX-timeout, RX-timeout in case of
 * TRX. In case of KIM2 HW, it is also about RX events such as RX-frame-received, DL-msg-received.
 *
 * @retval KNS_STATUS_OK if TX DONE or KNS_STATUS_TIMEOUT if timeout reached, else KNS_STATUS_ERROR
 */
enum KNS_status_t MGR_AT_CMD_macEvtProcess(void);

#endif /* __MGR_AT_CMD_H */

/**
 * @}
 */
