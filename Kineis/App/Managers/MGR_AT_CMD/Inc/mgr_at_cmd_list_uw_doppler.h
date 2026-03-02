/**
 * @file    mgr_at_cmd_list_uw_doppler.h
 * @brief   AT commands for UW_DOPPLER application (SWS, TX config, LED, deploy)
 */

#ifndef __MGR_AT_CMD_LIST_UW_DOPPLER_H
#define __MGR_AT_CMD_LIST_UW_DOPPLER_H

#include "mgr_at_cmd_common.h"

/** @brief AT+SWS: Get SWS status / Enable-disable SWS
 *
 * Status: +SWS=<state>,<adc>,<air_bl>,<water_bl>,<threshold>,<enabled>
 * Action: AT+SWS=<0|1> enable/disable
 */
bool bMGR_AT_CMD_SWS_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+SWSCFG: Get/Set SWS configuration
 *
 * Status: +SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,<interval_ms>,<max_dive_s>,<min_surf_s>
 * Action: AT+SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,<interval_ms>,<max_dive_s>,<min_surf_s>
 */
bool bMGR_AT_CMD_SWSCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+TXCFG: Get/Set TX scheduling configuration
 *
 * Status: +TXCFG=<interval_s>,<growth%>,<max_interval_s>,<max_count>
 * Action: AT+TXCFG=<interval_s>,<growth%>,<max_interval_s>,<max_count>
 */
bool bMGR_AT_CMD_TXCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+SWSFORCE: Force SWS measurement and return result
 *
 * Action only: triggers measurement, returns +SWS=<state>,<adc>
 */
bool bMGR_AT_CMD_SWSFORCE_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+LED: Get/Set LED mode
 *
 * Status: +LED=<mode> (0=off, 1=on, 2=24h)
 * Action: AT+LED=<0|1|2>
 */
bool bMGR_AT_CMD_LED_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+DEPLOY: Get/Set deploy mode
 *
 * Status: +DEPLOY=<mode> (0=not deployed, 1=deployed)
 * Action: AT+DEPLOY=<0|1>
 */
bool bMGR_AT_CMD_DEPLOY_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+LOG: Dump / Clear event log
 *
 * Status: Dump all events (oldest first), each as "#NNN t=TICK e=TYPE s=STATE d=DATA"
 * Action: Clear the event log
 */
bool bMGR_AT_CMD_LOG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+SAVE: Save current config to NVM flash
 *
 * Action: Saves all config (TX, SWS, deploy, LED) to flash with CRC32
 */
bool bMGR_AT_CMD_SAVE_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

#endif /* __MGR_AT_CMD_LIST_UW_DOPPLER_H */
