/**
 * @file    mgr_at_cmd_list_doppler.h
 * @brief   AT commands for DOPPLER application
 */

#ifndef __MGR_AT_CMD_LIST_DOPPLER_H
#define __MGR_AT_CMD_LIST_DOPPLER_H

#include "mgr_at_cmd_common.h"

/** @brief AT+DPLCFG: Get/Set DOPPLER config
 *
 * Status: +DPLCFG=<msg_count>,<msg_interval_s>,<sequence_interval_s>,<tpl_interval_s>
 * Action: AT+DPLCFG=<msg_count>,<msg_interval_s>,<sequence_interval_s>,<tpl_interval_s>
 */
bool bMGR_AT_CMD_DPLCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+DPLWKU: Read/Reset WKU boot counter (debug)
 *
 * Status: +DPLWKU=<boot_count>
 * Action: AT+DPLWKU=0 to reset counter
 */
bool bMGR_AT_CMD_DPLWKU_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+SAVE: Save current config to NVM flash */
bool bMGR_AT_CMD_DPLSAVE_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+LOG: Dump / Clear event log */
bool bMGR_AT_CMD_DPLLOG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+BATCFG: Get/Set battery protection config (STDALONE only) */
bool bMGR_AT_CMD_DPLBATCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

#endif /* __MGR_AT_CMD_LIST_DOPPLER_H */
