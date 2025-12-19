/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_at_cmd_list.h
 * @author  Kineis
 * @brief AT commands list header file
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_AT_CMD_CONF_H
#define __MGR_AT_CMD_CONF_H


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/

/** Indexes for AT commands present in system */
enum  atcmd_idx_t {
	// General commands
	AT_VERSION,      /**< Get AT commands version */
	AT_PING,         /**< Ping command */
	AT_FW,           /**< Get fw version command */
	AT_ADDR,         /**< Get/set device address command */
	AT_ID,           /**< Get/set device ID command */
	AT_SECKEY,       /**< Get/set device secret key */
	AT_SN,           /**< Get device serial number command */
	AT_RCONF,        /**< Get/Set radio configuration command */
	AT_SAVE_RCONF,   /**< Save radio configuration into Flash command */
	AT_LPM,          /**< Get/Set low power mode command */
	AT_MC,           /**< Get the message counter that will be used for next frame TX request */
	AT_TCXO_WU,      /**< Get/Set TCXO Warm up in ms */

	// User data commands
	AT_TX,           /**< Index for TX commands */
#ifdef USE_RX_STACK
	AT_RX,           /**< Index for TX commands */
#endif

	// Certif commands
	AT_CW,           /**< Index for CW/MW commands */

	// date commands
	AT_UDATE,        /**< Index for UTC date/time update */

	// MAC commands
	AT_KMAC,         /**< Index for change profile */

	// Prepass command
	AT_PREPASS_EN,        /**< Enable prepass, not implemented */

	ATCMD_MAX_COUNT,
	ATCMD_UNKNOWN_COMMAND = ATCMD_MAX_COUNT
};


/* Types ---------------------------------------------------------------------*/

/** Function entry point to process action/status commands. */
typedef bool (*pvATCMD_cmd_proc_fun_type)(uint8_t *pu8_cmdParamString,
		enum atcmd_type_t e_exec_mode);

/** Elementary structure for each AT command to hold necessary information
 * to handle action/status/information sub commands type
 */
struct atcmd_desc_t {
	/**<Name of the command */
	const char *pu8_cmdNameString;

	/**< Min input command length included AT name.
	 * For example AT basic command will have length=2,
	 * and AT+TRUN a length=7
	 */
	uint8_t u8_cmdNameLen;

	/**< Command function entry point to process action/status cmd type
	 * AT+XXXX=... or AT+XXXX=?
	 */
	pvATCMD_cmd_proc_fun_type f_ht_cmd_fun_proc;

};

/* Extern ---------------------------------------------------------------------------------------*/
extern const char *atcmd_version;
extern const struct atcmd_desc_t cas_atcmd_list_array[];



#endif /* __MGR_AT_CMD_CONF_H */
/**
 * @}
 */
