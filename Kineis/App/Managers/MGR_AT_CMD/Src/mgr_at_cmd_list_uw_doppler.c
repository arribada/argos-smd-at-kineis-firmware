/**
 * @file    mgr_at_cmd_list_uw_doppler.c
 * @brief   AT command handlers for UW_DOPPLER application
 *
 * Implements the following AT commands:
 *   - AT+SWS      : SWS status / enable-disable
 *   - AT+SWSCFG   : SWS configuration (thresholds, baselines, intervals)
 *   - AT+SWSFORCE : Force immediate SWS measurement
 *   - AT+TXCFG    : TX scheduling configuration (interval, growth, max)
 *   - AT+LED      : LED mode control (off / on / 24h)
 *   - AT+DEPLOY   : Deploy mode (enable/disable satellite TX)
 *   - AT+SAVE     : Save all config to NVM flash with CRC32
 *   - AT+LOG      : Event log dump (status) / clear (action)
 *
 * @attention The AT command table in mgr_at_cmd_list.c must list commands
 *            with longest prefix first (SWSFORCE before SWSCFG before SWS)
 *            to avoid prefix shadowing in the dispatcher.
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "mgr_at_cmd_common.h"
#include "mgr_at_cmd_list.h"
#include "mgr_at_cmd_list_uw_doppler.h"
#include "mcu_at_console.h"
#include "mgr_sws.h"
#include "mgr_nvm.h"
#include "mgr_evtlog.h"
#include "mgr_wdg.h"
#include "kns_app_uw_doppler.h"
#include "main.h"
#include "mgr_log.h"

#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif
#if defined(BSP_HAS_VBAT_ADC)
#include "mgr_bat.h"
#endif

/* Functions -----------------------------------------------------------------*/

bool bMGR_AT_CMD_SWS_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_SWS_Config_t cfg = MGR_SWS_getConfig();
		MGR_SWS_State_t state = MGR_SWS_getState();
		uint16_t adc = MGR_SWS_getLastADC();

		MCU_AT_CONSOLE_send("+SWS=%u,%u,%u,%u,%u,%u\r\n",
			(unsigned)state,
			(unsigned)adc,
			(unsigned)cfg.initial_air_baseline,
			(unsigned)cfg.initial_water_baseline,
			(unsigned)cfg.threshold_min,
			(unsigned)(cfg.enabled ? 1 : 0));

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int en;
		if (sscanf((const char *)pu8_cmdParamString, "AT+SWS=%u", &en) != 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		MGR_SWS_Config_t cfg = MGR_SWS_getConfig();
		cfg.enabled = (en != 0);
		MGR_SWS_setConfig(&cfg);
		MGR_LOG_DEBUG("[AT] SWS enabled=%u\r\n", cfg.enabled);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_SWSCFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_SWS_Config_t cfg = MGR_SWS_getConfig();

		/* Format: +SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,
		 *                <int_surf_ms>,<int_uw_ms>,<max_dive_s>,<min_surf_s>,
		 *                <delay_min_us>,<delay_max_us>
		 */
		MCU_AT_CONSOLE_send("+SWSCFG=%u,%u,%u,%u,%lu,%lu,%lu,%lu,%u,%u\r\n",
			(unsigned)cfg.threshold_min,
			(unsigned)cfg.threshold_max,
			(unsigned)cfg.initial_air_baseline,
			(unsigned)cfg.initial_water_baseline,
			(unsigned long)cfg.test_interval_surface_ms,
			(unsigned long)cfg.test_interval_underwater_ms,
			(unsigned long)cfg.max_dive_time_s,
			(unsigned long)cfg.min_surface_time_s,
			(unsigned)cfg.sample_delay_min_us,
			(unsigned)cfg.sample_delay_max_us);

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int thr_min, thr_max, air_init, water_init;
		unsigned int delay_min_us, delay_max_us;
		unsigned long int_surf_ms, int_uw_ms, max_dive_s, min_surf_s;

		if (sscanf((const char *)pu8_cmdParamString,
			"AT+SWSCFG=%u,%u,%u,%u,%lu,%lu,%lu,%lu,%u,%u",
			&thr_min, &thr_max, &air_init, &water_init,
			&int_surf_ms, &int_uw_ms, &max_dive_s, &min_surf_s,
			&delay_min_us, &delay_max_us) != 10) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (thr_min > 4095 || thr_max > 4095 || thr_min >= thr_max ||
		    water_init <= air_init ||
		    int_surf_ms == 0 || int_uw_ms == 0 ||
		    delay_min_us == 0 || delay_max_us < delay_min_us ||
		    delay_min_us > 65535 || delay_max_us > 65535) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		MGR_SWS_Config_t cfg = MGR_SWS_getConfig();
		cfg.threshold_min                = (uint16_t)thr_min;
		cfg.threshold_max                = (uint16_t)thr_max;
		cfg.initial_air_baseline         = (uint16_t)air_init;
		cfg.initial_water_baseline       = (uint16_t)water_init;
		cfg.test_interval_surface_ms     = (uint32_t)int_surf_ms;
		cfg.test_interval_underwater_ms  = (uint32_t)int_uw_ms;
		cfg.max_dive_time_s              = (uint32_t)max_dive_s;
		cfg.min_surface_time_s           = (uint32_t)min_surf_s;
		cfg.sample_delay_min_us          = (uint16_t)delay_min_us;
		cfg.sample_delay_max_us          = (uint16_t)delay_max_us;
		/* Keep current default within bounds */
		if (cfg.sample_delay_default_us < cfg.sample_delay_min_us)
			cfg.sample_delay_default_us = cfg.sample_delay_min_us;
		if (cfg.sample_delay_default_us > cfg.sample_delay_max_us)
			cfg.sample_delay_default_us = cfg.sample_delay_max_us;
		MGR_SWS_setConfig(&cfg);

		MGR_LOG_DEBUG("[AT] SWSCFG set: surf=%lums uw=%lums delay=%u-%uus\r\n",
			(unsigned long)int_surf_ms, (unsigned long)int_uw_ms,
			(unsigned)delay_min_us, (unsigned)delay_max_us);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_TXCFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		KNS_APP_UwDopplerTxCfg_t cfg = KNS_APP_uw_doppler_getTxCfg();

		/* Format: +TXCFG=<interval_s>,<growth%>,<max_interval_s>,<max_count>,<jitter%> */
		MCU_AT_CONSOLE_send("+TXCFG=%u,%u,%u,%u,%u\r\n",
			(unsigned)cfg.tx_initial_interval_s,
			(unsigned)cfg.tx_growth_percent,
			(unsigned)cfg.tx_max_interval_s,
			(unsigned)cfg.tx_max_count,
			(unsigned)cfg.tx_jitter_percent);

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int interval_s, growth, max_interval_s, max_count, jitter_pct;

		int n = sscanf((const char *)pu8_cmdParamString,
			"AT+TXCFG=%u,%u,%u,%u,%u",
			&interval_s, &growth, &max_interval_s, &max_count, &jitter_pct);
		if (n == 4) {
			jitter_pct = 0;  /* legacy 4-param form keeps jitter unchanged-default off */
		} else if (n != 5) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (interval_s == 0 || interval_s > 65535 || growth > 255 ||
		    max_interval_s == 0 || max_interval_s > 65535 || max_count > 255 ||
		    interval_s > max_interval_s || jitter_pct > 50) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		KNS_APP_UwDopplerTxCfg_t cfg;
		cfg.tx_initial_interval_s = (uint16_t)interval_s;
		cfg.tx_growth_percent = (uint8_t)growth;
		cfg.tx_max_interval_s = (uint16_t)max_interval_s;
		cfg.tx_max_count = (uint8_t)max_count;
		cfg.tx_jitter_percent = (uint8_t)jitter_pct;
		KNS_APP_uw_doppler_setTxCfg(&cfg);

		MGR_LOG_DEBUG("[AT] TXCFG set: T0=%us growth=%u%% max=%us count=%u jit=%u%%\r\n",
			interval_s, growth, max_interval_s, max_count, jitter_pct);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_SWSFORCE_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	/* Accept both modes for convenience */
	if (e_exec_mode == ATCMD_ACTION_MODE || e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_WDG_refresh();
		MGR_SWS_forceMeasurement();
		MGR_SWS_task();

		MGR_SWS_State_t state = MGR_SWS_getState();
		uint16_t adc = MGR_SWS_getLastADC();

		MCU_AT_CONSOLE_send("+SWS=%u,%u\r\n",
			(unsigned)state, (unsigned)adc);

		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LED_cmd(uint8_t *pu8_cmdParamString,
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

bool bMGR_AT_CMD_DEPLOY_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+DEPLOY=%u\r\n",
			(unsigned)KNS_APP_uw_doppler_getDeployMode());
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
		KNS_APP_uw_doppler_setDeployMode((uint8_t)mode);
		MGR_LOG_DEBUG("[AT] Deploy mode=%u\r\n", mode);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_SAVE_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE || e_exec_mode == ATCMD_STATUS_MODE) {
		if (!MGR_NVM_save()) {
			MGR_LOG_DEBUG("[AT] NVM save failed\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
		}
		MGR_LOG_DEBUG("[AT] Config saved to NVM\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LOG_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint16_t count = MGR_EVTLOG_count();
		MCU_AT_CONSOLE_send("+LOG=%u\r\n", (unsigned)count);

		/* Refresh watchdog every 32 entries (~1.5s at 9600 baud) */
		#define LOG_WDG_REFRESH_ENTRIES 32
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

bool bMGR_AT_CMD_BATCFG_cmd(uint8_t *pu8_cmdParamString,
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

/**
 * @}
 */
