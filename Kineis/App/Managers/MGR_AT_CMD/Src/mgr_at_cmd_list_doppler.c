/**
 * @file    mgr_at_cmd_list_doppler.c
 * @brief   AT command handlers for DOPPLER application
 *
 * Implements:
 *   - AT+DPLCFG  : DOPPLER config (msg_count, msg_interval, sequence_interval, tpl_interval)
 *   - AT+DPLWKU  : WKU boot counter read/reset
 *   - AT+SAVE    : Save config to NVM
 *   - AT+LOG     : Event log dump/clear
 *   - AT+BATCFG  : Battery config (STDALONE only)
 *   - AT+LED     : LED mode control (off / on / 24h auto-off)
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

#include <stdio.h>
#include <string.h>

#include "mgr_at_cmd_common.h"
#include "mgr_at_cmd_list.h"
#include "mgr_at_cmd_list_doppler.h"
#include "mcu_at_console.h"
#include "kns_app_doppler.h"
#include "mcu_flash.h"
#include "mgr_evtlog.h"
#include "mgr_wdg.h"
#include "mgr_log.h"
#include "main.h"

#if defined(BSP_HAS_VBAT_ADC)
#include "mgr_bat.h"
#endif

#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif

/* WDG refresh interval for long AT log dumps */
#define LOG_WDG_REFRESH_ENTRIES  32

bool bMGR_AT_CMD_DPLCFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		KNS_APP_DopplerCfg_t cfg = KNS_APP_doppler_getCfg();

		MCU_AT_CONSOLE_send("+DPLCFG=%u,%lu,%lu,%lu\r\n",
			(unsigned)cfg.msg_count,
			(unsigned long)cfg.msg_interval_s,
			(unsigned long)cfg.sequence_interval_s,
			(unsigned long)cfg.tpl_interval_s);

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int msg_count;
		unsigned long msg_interval, seq_interval, tpl_interval;

		if (sscanf((const char *)pu8_cmdParamString,
			"AT+DPLCFG=%u,%lu,%lu,%lu",
			&msg_count, &msg_interval, &seq_interval, &tpl_interval) != 4) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}

		/* Validate msg_count fits in uint8_t before casting */
		if (msg_count > 255) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		KNS_APP_DopplerCfg_t cfg;
		cfg.msg_count           = (uint8_t)msg_count;
		cfg.msg_interval_s      = (uint32_t)msg_interval;
		cfg.sequence_interval_s = (uint32_t)seq_interval;
		cfg.tpl_interval_s      = (uint32_t)tpl_interval;

		if (!KNS_APP_doppler_setCfg(&cfg)) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		MGR_LOG_DEBUG("[AT] DPLCFG set: %u,%lu,%lu,%lu\r\n",
			msg_count, msg_interval, seq_interval, tpl_interval);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_DPLWKU_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint64_t count = MCU_FLASH_read_wku_counter();
		uint32_t count_hi = (uint32_t)(count >> 32);
		uint32_t count_lo = (uint32_t)(count & 0xFFFFFFFFUL);
		if (count_hi > 0)
			MCU_AT_CONSOLE_send("+DPLWKU=%lu%08lu\r\n",
				(unsigned long)count_hi, (unsigned long)count_lo);
		else
			MCU_AT_CONSOLE_send("+DPLWKU=%lu\r\n", (unsigned long)count_lo);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int val;
		if (sscanf((const char *)pu8_cmdParamString, "AT+DPLWKU=%u", &val) != 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (val == 0) {
			MCU_FLASH_reset_wku_counter();
			MGR_LOG_DEBUG("[AT] WKU counter reset\r\n");
		} else {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_DPLSAVE_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE || e_exec_mode == ATCMD_STATUS_MODE) {
		if (!KNS_APP_doppler_nvmSave()) {
			MGR_LOG_DEBUG("[AT] DOPPLER NVM save failed\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
		}
		MGR_LOG_DEBUG("[AT] DOPPLER config saved to NVM\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_DPLLOG_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint16_t count = MGR_EVTLOG_count();
		MCU_AT_CONSOLE_send("+LOG=%u\r\n", (unsigned)count);

		for (uint16_t i = 0; i < count; i++) {
			if ((i % LOG_WDG_REFRESH_ENTRIES) == 0)
				MGR_WDG_refresh();
			const MGR_EVTLOG_Entry_t *e = MGR_EVTLOG_get(i);
			if (e) {
				MCU_AT_CONSOLE_send("#%03u t=%08lu e=%02u s=%02u d=%04u\r\n",
					(unsigned)i,
					(unsigned long)e->tick,
					(unsigned)e->type,
					(unsigned)e->state,
					(unsigned)e->data);
			}
		}

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_EVTLOG_clear();
		MGR_LOG_DEBUG("[AT] Event log cleared\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_DPLBATCFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
#if defined(BSP_HAS_VBAT_ADC)
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint16_t min_mV = MGR_BAT_getMinTxVoltage_mV();
		uint16_t cur_mV = MGR_BAT_readVoltage_mV();

		MCU_AT_CONSOLE_send("+BATCFG=%u,%u\r\n",
			(unsigned)min_mV, (unsigned)cur_mV);

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int min_mV;
		if (sscanf((const char *)pu8_cmdParamString, "AT+BATCFG=%u", &min_mV) != 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (min_mV > 4200) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}
		MGR_BAT_setMinTxVoltage_mV((uint16_t)min_mV);
		MGR_LOG_DEBUG("[AT] BATCFG min_tx=%umV\r\n", min_mV);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
#else
	(void)pu8_cmdParamString;
	(void)e_exec_mode;
	MCU_AT_CONSOLE_send("+BATCFG=N/A\r\n");
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
#endif
}

bool bMGR_AT_CMD_DPLLED_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
#if defined(BSP_HAS_LED_RGB)
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+LED=%u\r\n", (unsigned)MGR_LED_getMode());
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int mode;
		if (sscanf((const char *)pu8_cmdParamString, "AT+LED=%u", &mode) != 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (mode > 2) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}
		MGR_LED_setMode((MGR_LED_Mode_t)mode);
		/* Mark NVM dirty so AT+SAVE persists LED mode */
		KNS_APP_doppler_markDirty();
		MGR_LOG_DEBUG("[AT] LED mode=%u\r\n", mode);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
#else
	(void)pu8_cmdParamString;
	(void)e_exec_mode;
	MCU_AT_CONSOLE_send("+LED=N/A\r\n");
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
#endif
}

bool bMGR_AT_CMD_DPLDEPLOY_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+DEPLOY=%u\r\n",
			(unsigned)KNS_APP_doppler_getDeployMode());
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int mode;
		if (sscanf((const char *)pu8_cmdParamString, "AT+DEPLOY=%u", &mode) != 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (mode > 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}
		KNS_APP_doppler_setDeployMode((uint8_t)mode);
		MGR_LOG_DEBUG("[AT] Deploy mode=%u\r\n", mode);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

/**
 * @}
 */
