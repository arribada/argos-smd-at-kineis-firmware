/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_at_cmd_list_mac.h
 * @author Kinéis
 * @brief subset of AT commands concerning Kinéis Medium Acces Channel (MAC).
 *
 * @note This subset of commands are actually stubs, only present because GUI is sending such AT
 *       commands to KIM device.
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */


#ifndef __MGR_AT_CMD_MAC_H
#define __MGR_AT_CMD_MAC_H

/* Includes ------------------------------------------------------------------*/

#include "mgr_at_cmd_common.h"

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Process AT command "AT+KMAC" get/set the Kineis MAC profile
 *
 * 1) "AT+KMAC=<profile ID>,<profile context>"
 * Response format: "+OK" or "+ERROR=<error_code>" (See \ref ERROR_RETURN_T)
 *
 * Profile ID:
 * * 0 none
 * * 1 basic
 * * 2 blind
 *
 * 2) "AT+KMAC=?" returns the profile ID
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed
 *         false if error
 */
bool bMGR_AT_CMD_KMAC_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/**
 * @brief AT+KCFG — set/get the v11.1.0 MAC L1 timer configuration bitmap.
 *
 * "AT+KCFG=<bitmap>" → suspend/resume L1 timer (bit0=suspend, bit1=resume).
 * "AT+KCFG=?"        → return current bitmap as +KCFG=<raw>.
 */
bool bMGR_AT_CMD_KCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/**
 * @brief AT+KEVT — drain and dump pending MAC events from the UL queue.
 *
 * "AT+KEVT=?" → emit "+KEVT=<id>,<status>,<app_evt>" per event present in
 * KNS_Q_UL_MAC2APP, then "+OK". On builds where the MAC stack handles its
 * own events the queue is normally empty.
 */
bool bMGR_AT_CMD_KEVT_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

#endif /* __MGR_AT_CMD_MAC_H */
/**
 * @}
 */
