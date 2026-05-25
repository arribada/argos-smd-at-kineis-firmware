// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_general.c
 * @author  Kineis
 * @brief subset of AT commands concerning general purpose (get ID, FW version, ...)
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "kns_types.h"
#include "mgr_at_cmd_common.h"
#include "mgr_at_cmd_list.h"
#include "mgr_at_cmd_list_general.h"
#include "mgr_at_cmd_list_user_data.h"
#include "mcu_at_console.h"
/* @todo PRODEV-68: remove specific flag when HW setting check is implemented on all platforms */
#if defined(SMD_PA) || defined(SMD_NOPA) || defined(SMD_STDALONE) || defined(SMD_OP)
#include "mcu_misc.h" // use to check RF HW setting vers radio confoguration
#endif
#include "kns_cfg.h"
#include "lpm.h" // used for AT+LPM command all is hardcoded so far
#include "build_info.h"
#include "mgr_log.h"
#include "mcu_nvm.h"
#include "mcu_aes.h"
#include "mcu_flash.h"
#include "main.h"  /* For request_dfu_mode() */

/* TCXO warmup limits - MEDIUM FIX: Replace magic number with constant */
#define TCXO_MAX_WARMUP_MS      30000UL

/* Functions -----------------------------------------------------------------*/

bool bMGR_AT_CMD_VERSION_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

	MCU_AT_CONSOLE_send("+VERSION=%s\r\n", atcmd_version);

	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_PING_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_FW_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

	MCU_AT_CONSOLE_send("+FW=%s\r\n", uc_fw_vers_commit_id);

	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_SECKEY_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	uint8_t sec_key[DSK_BYTE_LENGTH];
	uint16_t nbBits;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		if (MCU_AES_get_device_sec_key(sec_key) != KNS_STATUS_OK)
			/* TODO: add a new error code ? */
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		char sec_key_str[33];
		for (int i = 0; i < 16; i++) {
			sprintf(&sec_key_str[i * 2], "%02x", sec_key[i]);
		}
		sec_key_str[32] = '\0';
		MCU_AT_CONSOLE_send("+SECKEY=%s\r\n", sec_key_str);

		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		/* CRITICAL FIX: Prevent strlen underflow on empty string */
		size_t len = strlen((char*)pu8_cmdParamString);
		while (len > 0 && (pu8_cmdParamString[len - 1] == '\r' ||
		                   pu8_cmdParamString[len - 1] == '\n')) {
			pu8_cmdParamString[--len] = '\0';
		}
		if (len <= 10) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS);
		}
		nbBits = u16MGR_AT_CMD_convertAsciiBinary(pu8_cmdParamString + 10,
			(uint16_t)strlen((char*)(pu8_cmdParamString + 10)));
			
			if (nbBits != DSK_BYTE_LENGTH*8)
			{
				MGR_LOG_DEBUG("%s::nbBits error %u should be %u\r\n", __func__, nbBits, DSK_BYTE_LENGTH*8);
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			
			if (MCU_AES_set_device_sec_key(pu8_cmdParamString + 10) != KNS_STATUS_OK)
			{
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}

bool bMGR_AT_CMD_ADDR_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint8_t dev_addr[DEVICE_ADDR_LENGTH];
	uint16_t nbBits;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getAddr(dev_addr);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		MCU_AT_CONSOLE_send("+ADDR=%02x%02x%02x%02x\r\n", dev_addr[0], dev_addr[1],
								  dev_addr[2], dev_addr[3]);

		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		/* CRITICAL FIX: Prevent strlen underflow on empty string */
		size_t len = strlen((char*)pu8_cmdParamString);
		while (len > 0 && (pu8_cmdParamString[len - 1] == '\r' ||
		                   pu8_cmdParamString[len - 1] == '\n')) {
			pu8_cmdParamString[--len] = '\0';
		}
		if (len <= 8) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS);
		}
		nbBits = u16MGR_AT_CMD_convertAsciiBinary(pu8_cmdParamString + 8,
			(uint16_t)strlen((char*)(pu8_cmdParamString + 8)));
			
			if ((nbBits != 32) && (nbBits != 28))
			{
				MGR_LOG_DEBUG("%s::nbBits error %u\r\n", __func__, nbBits);
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			
			if (MCU_NVM_setAddr(pu8_cmdParamString + 8) != KNS_STATUS_OK)
			{
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}
bool bMGR_AT_CMD_ID_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint32_t dev_id;
	uint16_t nbBits;
	uint16_t nbChar;
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getId(&dev_id);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		/* ID is printed as a number, with decimal representation.
		 * ID is stored in memory in little endian format.
		 */
		MCU_AT_CONSOLE_send("+ID=%d\r\n", dev_id);

		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		/* CRITICAL FIX: Prevent strlen underflow on empty string */
		size_t len = strlen((char*)pu8_cmdParamString);
		while (len > 0 && (pu8_cmdParamString[len - 1] == '\r' ||
		                   pu8_cmdParamString[len - 1] == '\n')) {
			pu8_cmdParamString[--len] = '\0';
		}
		if (len <= 6) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS);
		}
		nbChar = (uint16_t)strlen((char*)pu8_cmdParamString+6);
			if (nbChar > 7)
			{
				MGR_LOG_DEBUG("[ERROR] Invalid ID length\r\n");	
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			
			nbBits = u16MGR_AT_CMD_convertAsciiToInt32(pu8_cmdParamString + 6, &dev_id);

			if ((nbBits > 32) || (nbBits < 1))
			{
				MGR_LOG_DEBUG("[ERROR] Invalid ID length conversion\r\n");
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}

			
			if (MCU_NVM_setID(&dev_id) != KNS_STATUS_OK)
			{
				MGR_LOG_DEBUG("[ERROR] failed to set ID\r\n");
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}

bool bMGR_AT_CMD_SN_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint8_t dev_sn[DEVICE_SN_LENGTH + 1];

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getSN(dev_sn);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		dev_sn[DEVICE_SN_LENGTH] = '\0';
		MCU_AT_CONSOLE_send("+SN=%s\r\n", dev_sn);

		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_RCONF_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	struct KNS_CFG_radio_t radio_cfg;
	/* CRITICAL FIX: Buffer increased to hold "UNKNOWN" + null (8 chars) */
	uint8_t modulation[16];
	uint16_t nbBits;
#if defined(SMD_PA) || defined(SMD_NOPA) || defined(SMD_STDALONE) || defined(SMD_OP)
	struct rfSettings_t hwSettings;
#endif

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getRadioInfo(&radio_cfg);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		if (radio_cfg.modulation == KNS_TX_MOD_LDA2)
			strcpy((char*)modulation, "LDA2");
		else if (radio_cfg.modulation == KNS_TX_MOD_LDA2L)
			strcpy((char*)modulation, "LDA2L");
		else if (radio_cfg.modulation == KNS_TX_MOD_VLDA4)
			strcpy((char*)modulation, "VLDA4");
		else if (radio_cfg.modulation == KNS_TX_MOD_HDA4)
			strcpy((char*)modulation, "HDA4");
		else if (radio_cfg.modulation == KNS_TX_MOD_LDK)
			strcpy((char*)modulation, "LDK");
		else
			strcpy((char*)modulation, "UNKNOWN");
		MCU_AT_CONSOLE_send("+RCONF=%u,%u,%d,%s\r\n", radio_cfg.min_frequency,
			radio_cfg.max_frequency, radio_cfg.rf_level, modulation);

		/** @todo PRODEV-68: check radio conf versus HW settings for other platforms */
#if defined(SMD_PA) || defined(SMD_NOPA) || defined(SMD_STDALONE) || defined(SMD_OP)
		if (MCU_MISC_getSettingsHwRf(radio_cfg.rf_level, &hwSettings) == KNS_STATUS_OK)
			return bMGR_AT_CMD_logSucceedMsg();
		return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
#else
		return bMGR_AT_CMD_logSucceedMsg();
#endif
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		/* CRITICAL FIX: Prevent strlen underflow on empty string */
		size_t len = strlen((char*)pu8_cmdParamString);
		while (len > 0 && (pu8_cmdParamString[len - 1] == '\r' ||
		                   pu8_cmdParamString[len - 1] == '\n')) {
			pu8_cmdParamString[--len] = '\0';
		}
		if (len <= 9) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS);
		}
		nbBits = u16MGR_AT_CMD_convertAsciiBinary(pu8_cmdParamString + 9,
			(uint16_t)strlen((char*)(pu8_cmdParamString + 9)));

		if (nbBits != 128)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) KNS_STATUS_RADIO_CONF_BAD);

		status = KNS_CFG_setRadioInfo(pu8_cmdParamString + 9);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_SAVE_RCONF_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		status = KNS_CFG_saveRadioInfo();
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LPM_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	int16_t scanParamRes;
	uint16_t lpm; /** @todo PRODEV-88:need to keep uint16_t remove compilation error.
	 	       * leads to hardfault error if using uint8_t
	 	       */
	uint16_t forced;
	enum MgrLpm_LPM_t forced_cur;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		forced_cur = LPM_getForcedMode();
		if (forced_cur != LOW_POWER_MODE_NONE)
			MCU_AT_CONSOLE_send("+LPM=0x%X,0x%X\r\n",
				lpm_config.allowedLPMbitmap, (unsigned int)forced_cur);
		else
			MCU_AT_CONSOLE_send("+LPM=0x%X\r\n", lpm_config.allowedLPMbitmap);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {

		/** Two accepted forms:
		 *   AT+LPM=0xXX          -> set allowed bitmap, clear forced mode
		 *   AT+LPM=0xXX,0xYY     -> set bitmap AND force mode (YY=0 clears)
		 */
		scanParamRes = (int16_t)sscanf((const char *)pu8_cmdParamString,
			(const char *)"AT+LPM=0x%hX,0x%hX", &lpm, &forced);
		if (scanParamRes == 2) {
			lpm &= LOW_POWER_MODE_NONE | LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP |
				LOW_POWER_MODE_STANDBY | LOW_POWER_MODE_SHUTDOWN;
			lpm_config.allowedLPMbitmap = (uint8_t)lpm;
			LPM_setForcedMode((enum MgrLpm_LPM_t)forced);
			return bMGR_AT_CMD_logSucceedMsg();
		}

		scanParamRes = (int16_t)sscanf((const char *)pu8_cmdParamString,
			(const char *)"AT+LPM=0x%hX", &lpm);
		if (scanParamRes == 1) {
			lpm &= LOW_POWER_MODE_NONE | LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP |
				LOW_POWER_MODE_STANDBY | LOW_POWER_MODE_SHUTDOWN;
			lpm_config.allowedLPMbitmap = (uint8_t)lpm;
			LPM_setForcedMode(LOW_POWER_MODE_NONE);
			return bMGR_AT_CMD_logSucceedMsg();
		}
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_MC_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint16_t mc;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getMC(&mc);
		if (status != KNS_STATUS_OK) {
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		}
		
		MCU_AT_CONSOLE_send("+MC=%d\r\n", mc);
		return bMGR_AT_CMD_logSucceedMsg();
	} else if(e_exec_mode == ATCMD_ACTION_MODE) {
		if (sscanf((char*)pu8_cmdParamString, "%*[^=]= %hu", &mc) != 1)
		{
			MGR_LOG_DEBUG("Missing parameter: AT+MC=VALUE\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		} else {
			if (KNS_CFG_setMC(mc) == KNS_STATUS_OK)
			{
				MGR_LOG_DEBUG("+MC=%u\r\n", mc);
				return bMGR_AT_CMD_logSucceedMsg();
			} else {
				MGR_LOG_DEBUG("Failed to update MC=%u\r\n", mc);
				return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) KNS_STATUS_FLASH_ERR);
			}
		}
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_TCXO_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	uint32_t tcxo_ms;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		/* MISRA C:2012 Rule 17.7 - Return value intentionally ignored */
		(void)MCU_MISC_TCXO_get_warmup(&tcxo_ms);
		MCU_AT_CONSOLE_send("+TCXO=%d\r\n", tcxo_ms);
		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		uint16_t nbChar;
		/* CRITICAL FIX: Prevent strlen underflow on empty string */
		size_t len = strlen((char*)pu8_cmdParamString);

		while ((len > 0U) && ((pu8_cmdParamString[len - 1U] == (uint8_t)'\r') ||
		                      (pu8_cmdParamString[len - 1U] == (uint8_t)'\n'))) {
			len--;
			pu8_cmdParamString[len] = (uint8_t)'\0';
		}

		if (len <= 11U) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS);
		}

		MGR_LOG_DEBUG("%s", __func__);
		nbChar = (uint16_t)strlen((char*)pu8_cmdParamString + 11);

		if (nbChar > 5U) {
			MGR_LOG_DEBUG("[ERROR] Invalid TCXO MS length, max value is %lu ms\r\n",
			              TCXO_MAX_WARMUP_MS);
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		(void)u16MGR_AT_CMD_convertAsciiToInt32(pu8_cmdParamString + 11, &tcxo_ms);

		if (tcxo_ms > TCXO_MAX_WARMUP_MS) {
			MGR_LOG_DEBUG("[ERROR] Invalid TCXO Warmup time, should be less than %lu ms\r\n",
			              TCXO_MAX_WARMUP_MS);
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		/* MISRA C:2012 Rule 17.7 - Return value intentionally ignored */
		(void)MCU_MISC_TCXO_set_warmup(tcxo_ms);
		return bMGR_AT_CMD_logSucceedMsg();
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}

bool bMGR_AT_CMD_RCONFRAW_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint8_t raw_rconf[FLASH_RADIOCONF_BYTE_SIZE];

		/* Read raw bytes directly from flash */
		enum KNS_status_t status = MCU_FLASH_read(
			FLASH_USER_START_ADDR + FLASH_RADIOCONF_OFFSET,
			raw_rconf,
			FLASH_RADIOCONF_BYTE_SIZE);

		if (status != KNS_STATUS_OK) {
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
		}

		/* Convert to hex string and send */
		char hex_str[33];  /* 16 bytes * 2 chars + null terminator */
		for (int i = 0; i < FLASH_RADIOCONF_BYTE_SIZE; i++) {
			sprintf(&hex_str[i * 2], "%02x", raw_rconf[i]);
		}
		hex_str[32] = '\0';

		/* Send response and +OK atomically to prevent debug logs interleaving */
		MCU_AT_CONSOLE_send("+RCONFRAW=%s\r\n+OK\r\n", hex_str);
		return true;
	}

	MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_BOOT_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	/* Accept both action mode (AT+BOOT=) and status mode (AT+BOOT=?) for flexibility */
	if (e_exec_mode == ATCMD_ACTION_MODE || e_exec_mode == ATCMD_STATUS_MODE) {
		/* Send OK response before jumping to bootloader */
		MCU_AT_CONSOLE_send("+BOOT=OK\r\n");

		/* Small delay to ensure response is transmitted */
		HAL_Delay(50);

		/* Jump to bootloader with UART protocol forced (no detection race) */
		request_dfu_mode(DFU_PROTO_UART);

		/* Should never reach here */
		return true;
	}

	MGR_LOG_VERBOSE("[ERROR] Invalid mode for AT+BOOT command\r\n");
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

/**
 * @}
 */
