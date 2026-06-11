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
#include "lpm.h"
#include "mgr_lpm_uw.h"
#if defined(BSP_HAS_REED_SWITCH)
#include "mgr_reed.h"
#endif
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
#if defined(BSP_HAS_REED_SWITCH)
#include "mgr_reed.h"
#endif
#include "mgr_rate.h"
#include "mgr_err.h"
#include "mgr_txstats.h"
#include "mgr_pmlog.h"
#include "mgr_gesture.h"
#include "stm32wlxx_hal.h"

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
		MGR_LOG_INFO("[AT] SWS enabled=%u\r\n", cfg.enabled);
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

		MGR_LOG_INFO("[AT] SWSCFG set: surf=%lums uw=%lums delay=%u-%uus\r\n",
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

		/* Format: +TXCFG=<interval_s>,<growth%>,<max_interval_s>,<max_count>,
		 *                <jitter%>,<cooldown_s>,<seq_restart_s> */
		MCU_AT_CONSOLE_send("+TXCFG=%u,%u,%u,%u,%u,%u,%u\r\n",
			(unsigned)cfg.tx_initial_interval_s,
			(unsigned)cfg.tx_growth_percent,
			(unsigned)cfg.tx_max_interval_s,
			(unsigned)cfg.tx_max_count,
			(unsigned)cfg.tx_jitter_percent,
			(unsigned)cfg.tx_cooldown_s,
			(unsigned)cfg.tx_seq_restart_s);

		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int interval_s, growth, max_interval_s, max_count, jitter_pct;
		unsigned int cooldown_s = 0;
		unsigned int seq_restart_s = 0;

		/* Accept legacy 4-/5-/6-arg forms and the new 7-arg form. Missing
		 * fields default to the current value (keep previous) so older GUI
		 * scripts keep working. */
		int n = sscanf((const char *)pu8_cmdParamString,
			"AT+TXCFG=%u,%u,%u,%u,%u,%u,%u",
			&interval_s, &growth, &max_interval_s, &max_count, &jitter_pct,
			&cooldown_s, &seq_restart_s);
		KNS_APP_UwDopplerTxCfg_t cur = KNS_APP_uw_doppler_getTxCfg();
		if (n == 4) {
			jitter_pct    = 0;
			cooldown_s    = cur.tx_cooldown_s;
			seq_restart_s = cur.tx_seq_restart_s;
		} else if (n == 5) {
			cooldown_s    = cur.tx_cooldown_s;
			seq_restart_s = cur.tx_seq_restart_s;
		} else if (n == 6) {
			seq_restart_s = cur.tx_seq_restart_s;
		} else if (n != 7) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (interval_s == 0 || interval_s > 65535 || growth > 255 ||
		    max_interval_s == 0 || max_interval_s > 65535 || max_count > 255 ||
		    interval_s > max_interval_s || jitter_pct > 50 ||
		    cooldown_s > 65535 || seq_restart_s > 65535) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		KNS_APP_UwDopplerTxCfg_t cfg;
		cfg.tx_initial_interval_s = (uint16_t)interval_s;
		cfg.tx_growth_percent = (uint8_t)growth;
		cfg.tx_max_interval_s = (uint16_t)max_interval_s;
		cfg.tx_max_count = (uint8_t)max_count;
		cfg.tx_jitter_percent = (uint8_t)jitter_pct;
		cfg.tx_cooldown_s = (uint16_t)cooldown_s;
		cfg.tx_seq_restart_s = (uint16_t)seq_restart_s;
		KNS_APP_uw_doppler_setTxCfg(&cfg);

		MGR_LOG_INFO("[AT] TXCFG set: T0=%us growth=%u%% max=%us count=%u "
			"jit=%u%% cool=%us seqrst=%us\r\n",
			interval_s, growth, max_interval_s, max_count, jitter_pct,
			cooldown_s, seq_restart_s);
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
		MGR_LOG_INFO("[AT] LED mode=%u\r\n", mode);
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
		MGR_LOG_INFO("[AT] Deploy mode=%u\r\n", mode);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_SAVE_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE || e_exec_mode == ATCMD_STATUS_MODE) {
		if (!MGR_NVM_save()) {
			MGR_LOG_WARN("[AT] NVM save failed\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
		}
		MGR_LOG_INFO("[AT] Config saved to NVM\r\n");
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
				/* Sprint 3: prepend severity character (T/I/W/E) so the
				 * host UI can filter or colour without re-mapping the
				 * full type table. */
				char sev = MGR_EVTLOG_severityChar(
					MGR_EVTLOG_getSeverity(
						(MGR_EVTLOG_Type_t)e->type));
				MCU_AT_CONSOLE_send(
					"#%03u %c t=%08lu e=%02u s=%02u d=%04u\r\n",
					(unsigned)i, sev,
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
		MGR_LOG_INFO("[AT] Event log cleared\r\n");
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
		MGR_LOG_INFO("[AT] BATCFG min_tx=%umV\r\n", min_mV);
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

bool bMGR_AT_CMD_RATECFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint32_t window_s = 0;
		uint16_t max_tx = 0;
		MGR_RATE_getConfig(&window_s, &max_tx, NULL);
		MCU_AT_CONSOLE_send("+RATECFG=%lu,%u\r\n",
			(unsigned long)window_s, (unsigned)max_tx);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned long window_s;
		unsigned int max_tx;
		if (sscanf((const char *)pu8_cmdParamString,
			"AT+RATECFG=%lu,%u", &window_s, &max_tx) != 2) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		/* setConfig clamps to MGR_RATE_MIN/MAX bounds — accept anything
		 * non-zero. Use AT+SAVE to persist. */
		MGR_RATE_setConfig((uint32_t)window_s, (uint16_t)max_tx);
		MGR_LOG_INFO("[AT] RATECFG window=%lus max_tx=%u\r\n",
			window_s, max_tx);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_RATE_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_STATUS_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	uint32_t window_s = 0;
	uint16_t max_tx = 0;
	uint16_t cur = 0;
	uint32_t retry = 0;
	MGR_RATE_getConfig(&window_s, &max_tx, &cur);
	bool blocked = MGR_RATE_isBlocked(&retry);

	MCU_AT_CONSOLE_send("+RATE=%u,%u,%lu,%u,%lu\r\n",
		(unsigned)cur, (unsigned)max_tx,
		(unsigned long)window_s,
		(unsigned)(blocked ? 1 : 0),
		(unsigned long)retry);
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_RATECLEAR_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	MGR_RATE_clear();
	MGR_LOG_INFO("[AT] RATECLEAR\r\n");
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_STATUS_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_STATUS_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	uint32_t uptime_s     = HAL_GetTick() / 1000u;
	uint32_t reset_count  = MGR_ERR_getResetCount();
	uint32_t crash_count  = MGR_ERR_getCrashCount();
	uint8_t  last_err     = (uint8_t)MGR_ERR_getLastError();
	uint8_t  state        = KNS_APP_uw_doppler_getStateRaw();
	uint8_t  sws_state    = (uint8_t)MGR_SWS_getState();
	uint16_t sws_adc      = MGR_SWS_getLastADC();
	uint8_t  deploy       = KNS_APP_uw_doppler_getDeployMode();
	uint32_t tx_session   = KNS_APP_uw_doppler_getTxCountSession();
#if defined(BSP_HAS_VBAT_ADC)
	uint16_t bat_mV       = MGR_BAT_readVoltage_mV();
#else
	uint16_t bat_mV       = 0;
#endif
	uint16_t rate_count = 0, rate_max = 0;
	uint32_t rate_window = 0;
	uint32_t rate_retry = 0;
	MGR_RATE_getConfig(&rate_window, &rate_max, &rate_count);
	uint8_t rate_blocked = MGR_RATE_isBlocked(&rate_retry) ? 1 : 0;
	uint8_t lb_act = KNS_APP_uw_doppler_isLbActive() ? 1 : 0;

	/* Sprint 3: append sws_fault bitfield (Sprint 3) — 0 = healthy. */
	uint8_t sws_fault = MGR_SWS_getFault();

	/* Single-line, comma-separated for easy GUI parsing. */
	MCU_AT_CONSOLE_send(
		"+STATUS=%u,%lu,%lu,%lu,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u\r\n",
		(unsigned)state,
		(unsigned long)uptime_s,
		(unsigned long)reset_count,
		(unsigned long)crash_count,
		(unsigned)last_err,
		(unsigned)sws_state,
		(unsigned)sws_adc,
		(unsigned)bat_mV,
		(unsigned)deploy,
		(unsigned long)tx_session,
		(unsigned)rate_count,
		(unsigned)rate_max,
		(unsigned)rate_blocked,
		(unsigned)lb_act,
		(unsigned)sws_fault);
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_DIAG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	/* SWS: trigger a fresh measurement, then read back the ADC. */
	MGR_SWS_forceMeasurement();
	uint16_t sws_adc = MGR_SWS_getLastADC();
	/* Health check combines the instant plausibility above with the rolling
	 * fault detector (stuck, OOR, no-variance). The fault bitfield is also
	 * reported as the 7th column so the GUI can break it down. */
	uint8_t sws_fault = MGR_SWS_getFault();
	uint8_t sws_ok = (sws_adc > 5 && sws_adc < 4090 &&
	                  sws_fault == MGR_SWS_FAULT_NONE) ? 1 : 0;

#if defined(BSP_HAS_VBAT_ADC)
	uint16_t bat_mV = MGR_BAT_readVoltage_mV();
	uint8_t  bat_ok = (bat_mV > 1500 && bat_mV < 5000) ? 1 : 0;
#else
	uint16_t bat_mV = 0;
	uint8_t  bat_ok = 0;
#endif

#if defined(BSP_HAS_REED_SWITCH)
	uint8_t reed_present = MGR_REED_isMagnetPresent() ? 1 : 0;
#else
	uint8_t reed_present = 0;
#endif

#if defined(BSP_HAS_LED_RGB)
	/* Visual cycle exercising every code path:
	 *   Solid:    R, G, B    (direct GPIO drive, 250 ms each)
	 *   Soft PWM: W, V, C, Y (SysTick time-mux, 700 ms each)
	 * Soft PWM colours get a longer slot so the user can confirm the
	 * persistence-of-vision blend looks continuous (no visible flicker).
	 * Total blocking time ~3.6 s — comfortably under IWDG (16 s). The
	 * WDG is refreshed at each step so even the longer composite holds
	 * leave headroom. */
	/* AT+DIAG is a commissioning command: the operator MUST see the
	 * color cycle to validate the LED chain regardless of AT+LED=0
	 * (which silences deployment status indicators, not test feedback). */
	MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_RED);    HAL_Delay(250); MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_GREEN);  HAL_Delay(250); MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_BLUE);   HAL_Delay(250); MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_WHITE);  HAL_Delay(700); MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_VIOLET); HAL_Delay(700); MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_CYAN);   HAL_Delay(700); MGR_WDG_refresh();
	MGR_LED_setForced(MGR_LED_YELLOW); HAL_Delay(700); MGR_WDG_refresh();
	MGR_LED_off();
	uint8_t led_ok = 1;
#else
	uint8_t led_ok = 0;
#endif

	MCU_AT_CONSOLE_send("+DIAG=%u,%u,%u,%u,%u,%u,%u\r\n",
		(unsigned)sws_ok, (unsigned)sws_adc,
		(unsigned)bat_ok, (unsigned)bat_mV,
		(unsigned)reed_present, (unsigned)led_ok,
		(unsigned)sws_fault);
	return bMGR_AT_CMD_logSucceedMsg();
}

extern void KNS_APP_uw_doppler_enterStandbyTest(uint32_t seconds);
extern void KNS_APP_uw_doppler_setDutyCfg(uint16_t uw_s, uint16_t surf_s, uint8_t enabled);
extern void KNS_APP_uw_doppler_getDutyCfg(uint16_t *uw_s, uint16_t *surf_s, uint8_t *enabled);

bool bMGR_AT_CMD_DUTYCFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint16_t uw_s = 0, surf_s = 0;
		uint8_t en = 0;
		KNS_APP_uw_doppler_getDutyCfg(&uw_s, &surf_s, &en);
		const uint16_t shutdown_thr = MGR_LPM_UW_getShutdownThreshold();
		const unsigned wake_sb = MGR_LPM_UW_isWakeFromStandby() ? 1u : 0u;
		const unsigned wake_tx = MGR_LPM_UW_isWakeShouldTx()    ? 1u : 0u;
		MCU_AT_CONSOLE_send("+DUTYCFG=%u,%u,%u,%u (sb_wake=%u,tx_due=%u)\r\n",
			(unsigned)uw_s, (unsigned)surf_s, (unsigned)en,
			(unsigned)shutdown_thr, wake_sb, wake_tx);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int uw_s = 0, surf_s = 0, en = 0, shutdown_thr = 0;
		/* 4-arg form sets duty + shutdown threshold; 3-arg keeps
		 * the previously-saved threshold untouched. */
		int parsed = sscanf((const char *)pu8_cmdParamString,
			"AT+DUTYCFG=%u,%u,%u,%u",
			&uw_s, &surf_s, &en, &shutdown_thr);
		if (parsed != 3 && parsed != 4)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		if (uw_s < 1u || uw_s > 65535u ||
		    surf_s < 1u || surf_s > 65535u || en > 1u)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		if (parsed == 4 && shutdown_thr > 65535u)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);

		KNS_APP_uw_doppler_setDutyCfg((uint16_t)uw_s,
			(uint16_t)surf_s, (uint8_t)en);
		if (parsed == 4)
			MGR_LPM_UW_setShutdownThreshold((uint16_t)shutdown_thr);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_STANDBYTEST_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	/* pu8_cmdParamString holds the FULL command incl. "AT+STANDBYTEST=N". */
	unsigned long seconds = 0u;
	if (sscanf((const char *)pu8_cmdParamString,
	           "AT+STANDBYTEST=%lu", &seconds) != 1)
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
	if (seconds < 1u || seconds > 60u)
		return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);

	MCU_AT_CONSOLE_send("+STANDBYTEST=%lu\r\n", seconds);
	(void)bMGR_AT_CMD_logSucceedMsg();
	/* Brief delay so +OK physically leaves the UART before entry. */
	HAL_Delay(100);

	KNS_APP_uw_doppler_enterStandbyTest((uint32_t)seconds);
	/* Never returns — chip enters STANDBY, wakes via cold boot. */
	return true;
}

bool bMGR_AT_CMD_STOPTEST_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	unsigned long seconds = 0u;
	if (sscanf((const char *)pu8_cmdParamString,
	           "AT+STOPTEST=%lu", &seconds) != 1)
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
	if (seconds < 1u || seconds > 60u)
		return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);

	MCU_AT_CONSOLE_send("+STOPTEST=%lu\r\n", seconds);
	(void)bMGR_AT_CMD_logSucceedMsg();
	HAL_Delay(100);

	/* Direct STOP2 entry — function RETURNS on wake (no cold-boot). */
	MGR_LPM_UW_enterStop2Timed((uint32_t)seconds);
	MCU_AT_CONSOLE_send("+STOPTEST=woke\r\n");
	return true;
}

bool bMGR_AT_CMD_UARTLOG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+UARTLOG=%u\r\n",
			(unsigned)(vMGR_LOG_isEnabled() ? 1u : 0u));
		return bMGR_AT_CMD_logSucceedMsg();
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int en = 0u;
		if (sscanf((const char *)pu8_cmdParamString,
		           "AT+UARTLOG=%u", &en) != 1)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		if (en > 1u)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		vMGR_LOG_setEnabled(en != 0u);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LPMTHR_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint16_t spin_ms = 0u, sleep_ms = 0u;
		uint8_t en = 0u;
		MGR_LPM_UW_getLpmThr(&spin_ms, &sleep_ms, &en);
		MCU_AT_CONSOLE_send("+LPMTHR=%u,%u,%u\r\n",
			(unsigned)spin_ms, (unsigned)sleep_ms, (unsigned)en);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int spin = 0u, sleep_x = 0u, en = 0u;
		if (sscanf((const char *)pu8_cmdParamString,
		           "AT+LPMTHR=%u,%u,%u", &spin, &sleep_x, &en) != 3)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		if (spin > 0xFFFFu || sleep_x > 0xFFFFu || en > 1u)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		if (sleep_x < spin)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		MGR_LPM_UW_setLpmThr((uint16_t)spin, (uint16_t)sleep_x,
		                     (uint8_t)en);
		MGR_LOG_INFO("[AT] LPMTHR spin=%ums sleep=%ums en=%u\r\n",
			spin, sleep_x, en);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LPMSTAT_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint32_t up = 0u, slept = 0u, cnt = 0u;
		MGR_LPM_UW_getLpmStats(&up, &slept, &cnt);
		/* duty_awake in 0.1 % units, computed on the host-friendly side:
		 * awake = up - slept. */
		uint32_t awake = up - slept;
		uint32_t duty_x1000 = (up > 0u)
			? (uint32_t)(((uint64_t)awake * 1000u) / up) : 0u;
		MCU_AT_CONSOLE_send("+LPMSTAT=%lu,%lu,%lu,%lu\r\n",
			(unsigned long)up, (unsigned long)slept,
			(unsigned long)cnt, (unsigned long)duty_x1000);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LOGLVL_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+LOGLVL=%u\r\n",
			(unsigned)MGR_LOG_getLevel());
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int lvl = 0u;
		if (sscanf((const char *)pu8_cmdParamString,
		           "AT+LOGLVL=%u", &lvl) != 1)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		if (lvl > (unsigned)MGR_LOG_LVL_NONE)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		MGR_LOG_setLevel((MGR_LOG_Level_t)lvl);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_MODE_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+MODE=%u\r\n",
			(unsigned)MGR_GESTURE_getMode());
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int mode = 0u;
		if (sscanf((const char *)pu8_cmdParamString,
		           "AT+MODE=%u", &mode) != 1)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		if (mode >= (unsigned)MGR_GESTURE_MODE_COUNT)
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);

		MGR_LOG_INFO("[AT] MODE=%u requested\r\n", mode);

		if (!MGR_GESTURE_requestMode((MGR_GESTURE_Mode_t)mode))
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);

		bMGR_AT_CMD_logSucceedMsg();

		/* For OPERATIONAL switches in production builds, the next main
		 * loop iter will tear down the UART. Give the +OK time to leave
		 * the FIFO before that happens. */
		HAL_Delay(50);
		return true;
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_RESET_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	MGR_LOG_INFO("[AT] RESET requested\r\n");
	bMGR_AT_CMD_logSucceedMsg();
	/* Give the +OK response 100 ms to physically leave the UART before the
	 * NVIC reset wipes the TX FIFO. The Cortex-M reset path is < 1 ms so
	 * without this the host would see truncated output. */
	HAL_Delay(100);
	NVIC_SystemReset();
	/* Never returns */
	return true;
}

bool bMGR_AT_CMD_SHUTDOWN_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	/* Optional parameter: AT+SHUTDOWN=<wake_seconds>
	 *   0   (or omitted)  → no RTC auto-wake; wake only on reed magnet.
	 *   N   (1..131072)   → RTC fires after N seconds, chip cold-boots.
	 * Useful to validate the wake path without needing 24 h of patience.
	 *
	 * A '=' followed by garbage must REJECT, not power the board off:
	 * fuzzing landed "AT+SHUTDOWN=zz" and the unchecked sscanf left
	 * wake_s at 0 → magnet-only power-off executed. Powering off a
	 * deployed tag is the most destructive command we have — parse
	 * strictly. */
	uint32_t wake_s = 0;
	if (pu8_cmdParamString != NULL &&
	    strchr((const char *)pu8_cmdParamString, '=') != NULL) {
		if (sscanf((const char *)pu8_cmdParamString,
			   "AT+SHUTDOWN=%lu", &wake_s) != 1) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		if (wake_s > 131072u) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}
	}

	MGR_LOG_INFO("[AT] SHUTDOWN requested (wake=%lus)\r\n",
		(unsigned long)wake_s);
	MGR_EVTLOG_log(EVT_SHUTDOWN, (uint16_t)(wake_s & 0xFFFFu));
	bMGR_AT_CMD_logSucceedMsg();
	/* Same 100 ms TX-flush delay as AT+RESET, then persist + release. */
	HAL_Delay(100);
	(void)MGR_NVM_save();
#if defined(BSP_HAS_REED_SWITCH)
	if (wake_s == 0u) {
		/* Magnet-only power-off: latch release + STOP2 fallback so the
		 * reed can wake even when VDD is bench-maintained (PB6 is not a
		 * WKUP pin — SHUTDOWN would be unwakeable by the reed). */
		MGR_LPM_UW_enterShutdownReed();
	}
	MGR_REED_releasePower();
#endif
	LPM_shutdownWithAutoWake(wake_s);
	/* Never returns — board loses power. */
	return true;
}

bool bMGR_AT_CMD_LBCFG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		KNS_APP_UwDopplerLbCfg_t cfg = KNS_APP_uw_doppler_getLbCfg();
		MCU_AT_CONSOLE_send("+LBCFG=%u,%u,%u,%u,%u\r\n",
			(unsigned)cfg.lb_enter_mV,
			(unsigned)cfg.lb_exit_mV,
			(unsigned)cfg.lb_tx_interval_s,
			(unsigned)cfg.lb_tx_max_s,
			(unsigned)cfg.lb_tx_max_count);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		unsigned int enter_mV, exit_mV, lb_int, lb_max, lb_count;
		if (sscanf((const char *)pu8_cmdParamString,
			"AT+LBCFG=%u,%u,%u,%u,%u",
			&enter_mV, &exit_mV, &lb_int, &lb_max, &lb_count) != 5) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		}
		/* enter_mV=0 disables — in that case skip the exit > enter rule. */
		if (enter_mV != 0) {
			if (exit_mV <= enter_mV)
				return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
			if (enter_mV < 1500 || exit_mV > 5000)
				return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}
		if (lb_int == 0 || lb_max == 0 || lb_int > lb_max ||
		    lb_int > 65535 || lb_max > 65535 || lb_count > 255) {
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		}

		KNS_APP_UwDopplerLbCfg_t cfg = {
			.lb_enter_mV       = (uint16_t)enter_mV,
			.lb_exit_mV        = (uint16_t)exit_mV,
			.lb_tx_interval_s  = (uint16_t)lb_int,
			.lb_tx_max_s       = (uint16_t)lb_max,
			.lb_tx_max_count   = (uint8_t)lb_count,
		};
		KNS_APP_uw_doppler_setLbCfg(&cfg);
		MGR_LOG_INFO("[AT] LBCFG enter=%umV exit=%umV intvl=%us max=%us cnt=%u\r\n",
			enter_mV, exit_mV, lb_int, lb_max, lb_count);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LB_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_STATUS_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	KNS_APP_UwDopplerLbCfg_t cfg = KNS_APP_uw_doppler_getLbCfg();
#if defined(BSP_HAS_VBAT_ADC)
	uint16_t cur_mV = MGR_BAT_readVoltage_mV();
#else
	uint16_t cur_mV = 0;
#endif
	uint8_t active = KNS_APP_uw_doppler_isLbActive() ? 1 : 0;

	MCU_AT_CONSOLE_send("+LB=%u,%u,%u,%u\r\n",
		(unsigned)active, (unsigned)cur_mV,
		(unsigned)cfg.lb_enter_mV, (unsigned)cfg.lb_exit_mV);
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_TXSTATS_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_TXSTATS_t s;
		MGR_TXSTATS_get(&s);
		MCU_AT_CONSOLE_send("+TXSTATS=%lu,%lu,%lu,%lu,%lu,%lu\r\n",
			(unsigned long)s.attempts,
			(unsigned long)s.done,
			(unsigned long)s.timeouts,
			(unsigned long)s.errors,
			(unsigned long)s.consecutive_fail,
			(unsigned long)s.worst_consecutive);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_TXSTATS_clear();
		MGR_LOG_INFO("[AT] TXSTATS cleared\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_PMLOG_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		uint16_t n = MGR_PMLOG_count();
		MCU_AT_CONSOLE_send("+PMLOG=%u\r\n", (unsigned)n);
		for (uint16_t i = 0; i < n; i++) {
			if ((i & 0x1Fu) == 0u) MGR_WDG_refresh();
			const MGR_PMLOG_Entry_t *e = MGR_PMLOG_get(i);
			if (!e) continue;
			MCU_AT_CONSOLE_send(
				"#%03u s=%u t=%02u st=%02u d=%04u tk=%lu seq=%u\r\n",
				(unsigned)i,
				(unsigned)e->severity,
				(unsigned)e->type,
				(unsigned)e->state,
				(unsigned)e->data,
				(unsigned long)e->tick_s,
				(unsigned)e->seq);
		}
		return bMGR_AT_CMD_logSucceedMsg();
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_PMLOG_clear();
		MGR_LOG_INFO("[AT] PMLOG cleared\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_TEST_cmd(uint8_t *pu8_cmdParamString,
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode != ATCMD_ACTION_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	unsigned int count = 0;
	if (sscanf((const char *)pu8_cmdParamString, "AT+TEST=%u", &count) != 1)
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
	if (count == 0 || count > 10)
		return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);

	uint8_t queued = KNS_APP_uw_doppler_startTestBurst((uint8_t)count);
	MCU_AT_CONSOLE_send("+TEST=%u\r\n", (unsigned)queued);
	return bMGR_AT_CMD_logSucceedMsg();
}

/**
 * @}
 */
