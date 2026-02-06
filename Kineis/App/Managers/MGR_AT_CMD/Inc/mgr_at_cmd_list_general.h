/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_at_cmd_list_general.h
 * @author  Kineis
 * @brief subset of AT commands concerning general purpose (get ID, FW version, ...)
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_AT_CMD_LIST_GENERAL_H
#define __MGR_AT_CMD_LIST_GENERAL_H

/* Includes ------------------------------------------------------------------*/
#include "mgr_at_cmd_common.h"

/* Functions -----------------------------------------------------------------*/

/** @brief Process AT command "AT+VERSION" to get the version of AT commands supported by the FW
 *
 * "AT+VERSION=?": will return the version of AT commands
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_VERSION_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+PING" to test the UART communication.
 *
 * "AT+PING=?": if the UART communication is ready the module returns "+OK".
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_PING_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+FW" to get the FW version
 *
 * Only status mode is authorized.
 *
 * "AT+FW=?" will log the KIM2 version followed by the ARTIC FW version.
 *
 * Response format: "+FW=<version>"
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_FW_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+ID" get/set the ID of module
 *
 * 1) "AT+ID=<ID>"
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * 2) "AT+ID=?" returns the ID of module
 * Response format: "+ID=<KIM_ID>" or "+ID=<error_code>". (See \ref ERROR_RETURN_T)
 * \<KIM_ID\> is an 6-digit string, representing the device identifier from the end
 * user point of view.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_ID_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+SECKEY" get/set the device secret key
 *
 * 1) "AT+SECKEY=<SECKEY>"
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * 2) "AT+SECKEY=?" returns the secret key of module
 * Response format: "+ID=<SECKEY>" or "+ID=<error_code>". (See \ref ERROR_RETURN_T)
 * \<SECKEY\> is an 16bytes secret key, representing the secret key used to encrypt/
 * decrypt message forwared to the satellite
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_SECKEY_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);
/** @brief Process AT command "AT+ADDR" get the address of module
 *
 * 1) "AT+ADDR=<address>"
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * 2) "AT+ADDR=?" returns the address of module
 * Response format: "+ADDR=<KIM_ADDR>" or "+ADDR=<error_code>". (See \ref ERROR_RETURN_T)
 * \<KIM_ADDR\> is an 8-digit hexadecimal string, corresponding to a 32-bit value. It
 * represents the device identifier from the network point of view.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_ADDR_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+SN" get the serial number of module
 *
 * 1) "AT+SN=<serial_number>"
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * 2) "AT+SN=?" returns the serial number of module
 * Response format: "+SN=<KIM_SN>" or "+SN=<error_code>". (See \ref ERROR_RETURN_T)
 * \<KIM_SN\> is an 14-digit string, representing the device serial number.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_SN_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+RCONF" get the radio configuration of module
 *
 * 1) "AT+RCONF=<radio configuration>"
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * Radio configuration format is a 32-ASCII-digit array corresponding to 16 bytes
 *
 * 2) "AT+RCONF=?" returns the radio configuration of module
 * Response format: "+RCONF=<fmin> <fmax> <rflevel> <modulation>" or "+RCONF=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_RCONF_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+SAVE_RCONF" save the radio configuration into Flash
 *
 * 1) "AT+SAVE_RCONF="
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_SAVE_RCONF_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Set/Get low power mode "AT+LPM"
 *
 * So far, on all kineis platforms and reference designs, LPm is described as \ref MgrLpm_LPM_t:
 * 	LOW_POWER_MODE_NONE     = 0x00,
 * 	LOW_POWER_MODE_SLEEP    = 0x01,
 * 	LOW_POWER_MODE_STOP     = 0x02,
 * 	LOW_POWER_MODE_STANDBY  = 0x04,
 * 	LOW_POWER_MODE_SHUTDOWN = 0x08
 *
 * 1) "AT+LPM=<LPM capabilities bitmap" will configure the expected LPM.
 * the FW will then allow to reach the deepest LPM configured inthis bitmap.
 *
 * @attention Ensure your configuration fits the LPM capabilities implemented in the FW
 *
 * 2) "AT+LPM=?" will reply te LPM configuration bitmap
 *
 * Response format: "+LPM=<bitmap>" depending on your implementation.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_LPM_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Process AT command "AT+MC" get the address of module
 *
 * 1) "AT+MC=<message counter>"
 * N/A Response format: "+ERROR=<error_code>". (See \ref ERROR_RETURN_T)
 *
 * 2) "AT+MC=?" returns the address of module
 * Response format: "+MC=<MC :Message counter used for next frame building>" or "+MC=<error_code>". (See \ref ERROR_RETURN_T)
 * \<MC\> is haxadecimal value on 3 digits corresponding to a 9-bit value. It
 * represents the device message counter used for antireplay and authentication of the radio frame.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_MC_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);


/** @brief Set/Get TCXO warmup time in ms
 *
 * 1) "AT+TCXO_WU=<Time in MS> will configure the expected Warmup time for the TCXO.
 *
 * @attention Warmup is not asynchronous, it will block the context so don't used it in ISR context
 *
 * 2) "AT+TCXO=?" will reply the TCXO time configured
 *
 * Response format: "+TCXO=<ms>" depending on your implementation.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_TCXO_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Get the raw radio configuration from flash (16 bytes in hex)
 *
 * Only status mode is authorized.
 *
 * "AT+RCONFRAW=?" returns the raw 16-byte radio configuration data from flash
 * in hexadecimal format (32 hex characters).
 *
 * Response format: "+RCONFRAW=<32 hex chars>"
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_RCONFRAW_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

/** @brief Enter bootloader/DFU mode
 *
 * Only action mode is authorized.
 *
 * "AT+BOOT" or "AT+BOOT=" triggers a jump to the bootloader for firmware update.
 * The device will reboot into DFU mode and accept AT+DFU=... commands.
 *
 * Response format: "+OK" followed by reboot (no further response expected)
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] e_exec_mode: type of the command (status command or action command)
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_AT_CMD_BOOT_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode);

#endif /* __MGR_AT_CMD_LIST_GENERAL_H */
/**
 * @}
 */
