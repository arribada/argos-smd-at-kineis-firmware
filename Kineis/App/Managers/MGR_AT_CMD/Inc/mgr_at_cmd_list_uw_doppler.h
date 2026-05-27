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

/** @brief AT+SWSCFG: Get/Set SWS configuration (10 fields)
 *
 * Status / Action:
 *   +SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,
 *           <int_surface_ms>,<int_underwater_ms>,
 *           <max_dive_s>,<min_surface_s>,
 *           <delay_min_us>,<delay_max_us>
 *
 * Surface interval should be slow (e.g. 5000ms) to save power.
 * Underwater interval should be fast (e.g. 1000ms) for rapid surface detection.
 * Adaptive RC charge delay bounds (typical: 200-5000us).
 */
bool bMGR_AT_CMD_SWSCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+TXCFG: Get/Set TX scheduling configuration (5 fields)
 *
 * Status: +TXCFG=<interval_s>,<growth%>,<max_interval_s>,<max_count>,<jitter%>
 * Action: AT+TXCFG=<interval_s>,<growth%>,<max_interval_s>,<max_count>[,<jitter%>]
 *
 * jitter% applies +/-jitter% randomization on each TX interval (max 50%).
 * Use 0 to disable jitter. Recommended 5-15% for multi-tag deployments.
 * The 4-field legacy form is still accepted (sets jitter to 0).
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
 * Action: Saves all config (TX, SWS, deploy, LED, battery) to flash with CRC32
 */
bool bMGR_AT_CMD_SAVE_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+BATCFG: Get/Set battery protection config
 *
 * Status: +BATCFG=<min_tx_mV>,<current_mV>
 * Action: AT+BATCFG=<min_tx_mV>  (0 = disable threshold)
 */
bool bMGR_AT_CMD_BATCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+RATECFG: Get/Set persistent TX rate limiter (sliding window)
 *
 * Status: +RATECFG=<window_s>,<max_tx>
 * Action: AT+RATECFG=<window_s>,<max_tx>
 *   window_s : 60 .. 604800 (1 minute .. 7 days)
 *   max_tx   : 1  .. 256
 * Persisted to NVM via AT+SAVE.
 */
bool bMGR_AT_CMD_RATECFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+RATE: Query live rate-limiter state (read-only)
 *
 * Status: +RATE=<current_count>,<max_tx>,<window_s>,<blocked>,<retry_in_s>
 *   blocked    : 0 if a TX is allowed right now, 1 if rate-limited
 *   retry_in_s : 0 unless blocked, else seconds until the oldest entry expires
 */
bool bMGR_AT_CMD_RATE_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+RATECLEAR: Wipe the rate limiter ring buffer
 *
 * Action only. Doesn't reset configuration. Use after a deliberate event
 * (deployment, recovery, RTC resync) where past TX timestamps no longer
 * reflect real budget consumption.
 */
bool bMGR_AT_CMD_RATECLEAR_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+STATUS: One-shot snapshot of every interesting runtime value.
 *
 * Status only. Format (single line, comma-separated, machine-parseable):
 *  +STATUS=<state>,<uptime_s>,<reset_count>,<crash_count>,
 *          <last_err>,<sws_state>,<sws_adc>,<bat_mV>,<deploy>,
 *          <tx_session>,<rate_count>,<rate_max>,<rate_blocked>
 */
bool bMGR_AT_CMD_STATUS_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+DIAG: Quick self-test of subsystems.
 *
 * Action only. Triggers fresh measurements (SWS, BAT) and a short LED cycle
 * (R,G,B 200ms each). Returns:
 *  +DIAG=<sws_ok>,<sws_adc>,<bat_ok>,<bat_mV>,<reed_present>,<led_ok>
 */
bool bMGR_AT_CMD_DIAG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+RESET: Force a software reset.
 *
 * Action only. NVIC_SystemReset() with a tiny pre-delay so the +OK reply
 * physically leaves the UART. No confirmation required (per design choice).
 */
bool bMGR_AT_CMD_RESET_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+LBCFG: Low-battery mode configuration.
 *
 * Status: +LBCFG=<enter_mV>,<exit_mV>,<lb_interval_s>,<lb_max_s>,<lb_max_count>
 * Action: AT+LBCFG=<enter_mV>,<exit_mV>,<lb_interval_s>,<lb_max_s>,<lb_max_count>
 *   enter_mV=0  → disable LB mode entirely
 *   exit_mV    must be > enter_mV (hysteresis)
 * Persisted to NVM via AT+SAVE.
 */
bool bMGR_AT_CMD_LBCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+LB: Query live LB mode state (read-only).
 *
 * +LB=<active>,<current_mV>,<enter_mV>,<exit_mV>
 */
bool bMGR_AT_CMD_LB_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+TXSTATS: persistent TX counters (Sprint 4).
 *
 * Status: +TXSTATS=<attempts>,<done>,<timeouts>,<errors>,<consec_fail>,<worst_consec>
 * Action: AT+TXSTATS  → clear all counters
 */
bool bMGR_AT_CMD_TXSTATS_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+PMLOG: post-mortem flash log (Sprint 4).
 *
 * Status: dumps every valid entry, oldest first:
 *   +PMLOG=<count>
 *   #NNN s=SEV t=TYPE st=STATE d=DATA tk=TICK_S seq=SEQ
 * Action: erases the entire log page.
 */
bool bMGR_AT_CMD_PMLOG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief AT+TEST=<count>: trigger a forced TX burst for radio validation (Sprint 4).
 *
 * Action only. count clamped to [1..10].
 *   +TEST=<queued_count>
 * Bypasses rate limiter / cooldown / backoff / deploy mode / surface check.
 * Each TX still respects the 5 s MIN_INTER_TX_INTERVAL safety floor.
 */
bool bMGR_AT_CMD_TEST_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

#endif /* __MGR_AT_CMD_LIST_UW_DOPPLER_H */
