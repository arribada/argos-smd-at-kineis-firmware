// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list.c
 * @author  Kineis
 * @brief AT commands list main file
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "mgr_at_cmd_common.h"
#include "mgr_at_cmd_list.h"
#include "mgr_at_cmd_list_general.h"
#include "mgr_at_cmd_list_user_data.h"
#include "mgr_at_cmd_list_previpass.h"
#include "mgr_at_cmd_list_mac.h"
#include "mgr_at_cmd_list_certif.h"
#ifdef USE_UW_DOPPLER_APP
#include "mgr_at_cmd_list_uw_doppler.h"
#endif
#ifdef USE_DOPPLER_APP
#include "mgr_at_cmd_list_doppler.h"
#endif

const char *atcmd_version = "v0.8";

/** @attention update AT cmd version above if you add or remove commands in this list */
const struct atcmd_desc_t cas_atcmd_list_array[ATCMD_MAX_COUNT] = {
	/**< General commands */
	{ "AT+VERSION",      10, bMGR_AT_CMD_VERSION_cmd},
	{ "AT+PING",          7, bMGR_AT_CMD_PING_cmd},
	{ "AT+FW",            5, bMGR_AT_CMD_FW_cmd},
	{ "AT+ADDR",          7, bMGR_AT_CMD_ADDR_cmd},
	{ "AT+ID",            5, bMGR_AT_CMD_ID_cmd},
	{ "AT+SECKEY",        9, bMGR_AT_CMD_SECKEY_cmd},
	{ "AT+SN",            5, bMGR_AT_CMD_SN_cmd},
	{ "AT+RCONFRAW",     11, bMGR_AT_CMD_RCONFRAW_cmd},
	{ "AT+SAVE_RCONF",   13, bMGR_AT_CMD_SAVE_RCONF_cmd},
	{ "AT+RCONF",         8, bMGR_AT_CMD_RCONF_cmd},
	{ "AT+LPM",           6, bMGR_AT_CMD_LPM_cmd},
	{ "AT+MC",            5, bMGR_AT_CMD_MC_cmd},
	{ "AT+TCXO_WU",      10, bMGR_AT_CMD_TCXO_cmd},

	/**< User data commands */
	{ "AT+TX",            5, bMGR_AT_CMD_TX_cmd},
#ifdef USE_RX_STACK
	{ "AT+RX",            5, bMGR_AT_CMD_RX_cmd},
#endif

	/**< Certif commands */
	{ "AT+CW",            5, bMGR_AT_CMD_CW_cmd},

	/**< to set the date in modem */
	{ "AT+UDATE",         8, bMGR_AT_CMD_UDATE_cmd},

	/**< MAC commands (not functionnal, only to avoid GUI to crash) */
	{ "AT+KMAC",          7, bMGR_AT_CMD_KMAC_cmd},

	/**< Prepass command (stub for GUI compatibility) */
	{ "AT+PREPASS_EN",   13, bMGR_AT_CMD_PREPASS_EN_cmd},

	/**< DFU command - enter bootloader mode */
	{ "AT+BOOT",          7, bMGR_AT_CMD_BOOT_cmd},

#ifdef USE_UW_DOPPLER_APP
	/**< UW_DOPPLER commands (longest prefix first to avoid shadowing) */
	{ "AT+SWSFORCE",     11, bMGR_AT_CMD_SWSFORCE_cmd},
	{ "AT+SWSCFG",        9, bMGR_AT_CMD_SWSCFG_cmd},
	{ "AT+SWS",           6, bMGR_AT_CMD_SWS_cmd},
	{ "AT+TXCFG",         8, bMGR_AT_CMD_TXCFG_cmd},
	{ "AT+LED",           6, bMGR_AT_CMD_LED_cmd},
	{ "AT+DEPLOY",        9, bMGR_AT_CMD_DEPLOY_cmd},
	{ "AT+LOG",           6, bMGR_AT_CMD_LOG_cmd},
	{ "AT+SAVE",          7, bMGR_AT_CMD_SAVE_cmd},
	{ "AT+BATCFG",        9, bMGR_AT_CMD_BATCFG_cmd},
	/* Rate limiter — RATECLEAR / RATECFG before RATE to win prefix match */
	{ "AT+RATECLEAR",    12, bMGR_AT_CMD_RATECLEAR_cmd},
	{ "AT+RATECFG",      10, bMGR_AT_CMD_RATECFG_cmd},
	{ "AT+RATE",          7, bMGR_AT_CMD_RATE_cmd},
	/* Sprint 2 diagnostic / control */
	{ "AT+STATUS",        9, bMGR_AT_CMD_STATUS_cmd},
	{ "AT+DIAG",          7, bMGR_AT_CMD_DIAG_cmd},
	{ "AT+RESET",         8, bMGR_AT_CMD_RESET_cmd},
	{ "AT+SHUTDOWN",     11, bMGR_AT_CMD_SHUTDOWN_cmd},
	{ "AT+STANDBYTEST",  14, bMGR_AT_CMD_STANDBYTEST_cmd},
	{ "AT+STOPTEST",     11, bMGR_AT_CMD_STOPTEST_cmd},
	{ "AT+DUTYCFG",      10, bMGR_AT_CMD_DUTYCFG_cmd},
	{ "AT+UARTLOG",      10, bMGR_AT_CMD_UARTLOG_cmd},
	{ "AT+MODE",          7, bMGR_AT_CMD_MODE_cmd},
	{ "AT+LOGLVL",        9, bMGR_AT_CMD_LOGLVL_cmd},
	/* LB mode — LBCFG before LB to win prefix match */
	{ "AT+LBCFG",         8, bMGR_AT_CMD_LBCFG_cmd},
	{ "AT+LB",            5, bMGR_AT_CMD_LB_cmd},
	/* Sprint 4 telemetry / post-mortem / test burst */
	{ "AT+TXSTATS",      10, bMGR_AT_CMD_TXSTATS_cmd},
	{ "AT+PMLOG",         8, bMGR_AT_CMD_PMLOG_cmd},
	{ "AT+TEST",          7, bMGR_AT_CMD_TEST_cmd},
#endif

#ifdef USE_DOPPLER_APP
	/**< DOPPLER commands (DEPLOY before DPLCFG: longest prefix first) */
	{ "AT+DEPLOY",        9, bMGR_AT_CMD_DPLDEPLOY_cmd},
	{ "AT+DPLCFG",        9, bMGR_AT_CMD_DPLCFG_cmd},
	{ "AT+DPLWKU",        9, bMGR_AT_CMD_DPLWKU_cmd},
	{ "AT+LOG",           6, bMGR_AT_CMD_DPLLOG_cmd},
	{ "AT+SAVE",          7, bMGR_AT_CMD_DPLSAVE_cmd},
	{ "AT+BATCFG",        9, bMGR_AT_CMD_DPLBATCFG_cmd},
	{ "AT+LED",           6, bMGR_AT_CMD_DPLLED_cmd},
#endif
};

/**
 * @}
 */
