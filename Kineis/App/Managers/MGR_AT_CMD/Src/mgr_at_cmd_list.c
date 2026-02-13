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

const char *atcmd_version = "v0.7";

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
};

/**
 * @}
 */
