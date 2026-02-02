// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_certif.c
 * @author Kinéis
 * @brief subset of AT commands concerning specifics for certification
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Includes --------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kns_types.h"
#include "kns_rf.h"
#ifdef KNS_RF_IN_BLOCKING_MODE
#include "mgr_at_cmd.h" // needed for MGR_AT_CMD_isPendingAt
#endif
#include "mgr_at_cmd_common.h"
#include "mgr_log.h"
#include "kns_assert.h" // for kns_assert only

/* Defines -------------------------------------------------------------------------------------- */

#define SUPPORT_MW_WITH_DELAYED_RETX
#ifdef SUPPORT_MW_WITH_DELAYED_RETX
#include "kns_app_conf.h"
#include INCLUDE_NOP_H // for __NOP()

/** @note Define a delay macro which does not depend on systick interrupt in a way to be able to run
 * from any ISR (e.g. turn ON PA wrapper below will be called from SUBGHZ ISR during continuous
 * modulated wave)
 *
 * @attention at typical 32MHz clock and 4 instruction cycle per loop, max value is 536870 ms
 *
 * NOP             : 1 cycle
 * SUBS R3, #1     : 1 cycle
 * CMP  R3, #0     : 1 cycle
 * BNE  <ADDR>     : 1 cycle
 */
//#define DELAY_MS(time_ms) HAL_Delay(time_ms)
extern uint32_t SystemCoreClock;
#define FOR_LOOP_CYCLE_NB 4
/* MISRA C:2012 Rule 20.7 - Macro parameters shall be enclosed in parentheses */
#define DELAY_MS(time_ms) \
{ \
	uint32_t _nb_for_loop_per_ms = SystemCoreClock / 1000U / FOR_LOOP_CYCLE_NB; \
	for (uint32_t _count = (time_ms) * _nb_for_loop_per_ms; \
		_count > 0U ; \
		_count--) { __NOP(); } \
}
#endif

/* Private type ----------------------------------------------------------------------------------*/

/** @brief Structure used to store modulated wave configuration (status and RF params such as
 * frequence, modulation, power
 *
 * @note Power as no impact on KIMx modules as RF front end is fixed by hardware. so far, only the
 * reference design has a certain output power dynamic (\ref MCU_MISC_getSettingsHwRf in mcu_misc
 * wrapper).
 */
struct mwCfg_t {
	bool valid;
	bool isRfAlreadyOn;
	struct KNS_tx_rf_cfg_t rf_cfg;
};

/* Private variables -----------------------------------------------------------------------------*/

#ifdef USE_HDA4
static uint8_t bitstream[700] = {0};
#else
static uint8_t bitstream[40] = {0};
#endif
static uint16_t bitstream_bitlen = 304;
static bool isToBeTransmit = false; /** Used to catch if a MW burst is correctly transmit or not */

/** Modulated wave configuration */
struct mwCfg_t mw_cfg = {
	.valid = false,
	.isRfAlreadyOn = false,
	.rf_cfg = {
		.center_freq = 401630000,
		.modulation = KNS_TX_MOD_LDA2,
		.power = 22
	}
};

#ifdef SUPPORT_MW_WITH_DELAYED_RETX
/** Used for optional periodic transmission. This is the delay introduced between end of previous
 * transmission and beginning of the next one
 *
 * @attention If this delay is over 1s, the TXCOWU will be re-done at each transmit. It will also
 * reply +OK at each transmit
 */
uint16_t repPeriod_s = 0;
#endif

/* Private function prototypes (usefull for later internal functions) --------*/

/** @brief Send random TX data as per RF configuration
 *
 * This function is the core function used to send some modulated wave
 *
 * @param[in] rf_cfg: RF parameters for modulated wave
 *
 * @return true is OK, false otherwise
 */
static bool MGR_AT_CMD_sendRandomTxData(struct KNS_tx_rf_cfg_t *rf_cfg);

/* Private functions ---------------------------------------------------------*/

/** @brief  Log array of uint8_t
 * @param[in] data: pointer to table
 * @param[in] len: number of bytes to log from the table
 */
static void MGR_LOG_array(__attribute__((unused)) uint8_t *data, uint16_t len)
{
	uint16_t i;

	for (i = 0; i < len; i++) {
		MGR_LOG_DEBUG_RAW("%02X", data[i]);
	}
	MGR_LOG_DEBUG_RAW("\r\n");
}

#ifndef KNS_RF_IN_BLOCKING_MODE
/** @brief  Callback function notifying end of TX processing in case of modulated wave
 *
 * Its purpose is to re-transmit UL bitstream as soon as previous transmission is complete. It will
 * lead to a kind of continuous flow of TX bursts.
 *
 * @attention This callback fct is called from ISR context so far. Limit inner processing if
 * application is realtime critical. In case of realtime critical application, suggest to post/copy
 * the event to background task
 *
 * @param[in] evt: callback event context
 *
 * @retval KNS_STATUS_OK if TX DONE or KNS_STATUS_TIMEOUT if timeout reached, KNS_STATUS_ERROR
 *         otherwise
 */
static enum KNS_status_t eoAtMW_isr_cb(struct KNS_RF_evt_t *evt)
{
	/** Start new TX if continuous wave is still requested */
	if ((evt->id == TX_DONE) && (mw_cfg.valid == true)) {
		isToBeTransmit = false;
		KNS_RFTX_powerOff(NULL);
#ifdef SUPPORT_MW_WITH_DELAYED_RETX
		if (repPeriod_s > 1)
			mw_cfg.isRfAlreadyOn = false; // clear to force TCXOWU again
		if (repPeriod_s != 0)
			DELAY_MS(repPeriod_s * 1000);
#endif
		if (MGR_AT_CMD_sendRandomTxData(NULL))
			return KNS_STATUS_OK;
		else
			return KNS_STATUS_ERROR;
	}

	return KNS_STATUS_OK;
}
#endif

static bool MGR_AT_CMD_sendRandomTxData(struct KNS_tx_rf_cfg_t *rf_cfg)
{
	struct KNS_tx_rf_cfg_t rf_cfg_local;
	uint16_t idx;
	enum KNS_status_t status;
	enum KNS_status_t (*eop_isr_cb)(struct KNS_RF_evt_t *evt_ctxt);

	/** Keep stored RF config when input one is NULL */
	if (rf_cfg != NULL) {
		rf_cfg_local = *rf_cfg;
		mw_cfg.rf_cfg = *rf_cfg;
		mw_cfg.isRfAlreadyOn = false;
	} else
		rf_cfg_local = mw_cfg.rf_cfg;

	/** So far, in case a new command occurs while previous one is not completed, abort it.
	 * Send error response concerning previous frame.
	 */
	if (isToBeTransmit) {
		MGR_LOG_VERBOSE("[%s] Abort transmission of previous frame\r\n", __func__);
		isToBeTransmit = false;
		status = KNS_RFTX_abortRf(NULL);
		if (status != KNS_STATUS_OK) {
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
		}
		status = KNS_RFTX_powerOff(NULL);
		if (status != KNS_STATUS_OK) {
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
		}
	}

	/** Fill-up data bitstream with new random data */
	for (idx = 0U; idx < sizeof(bitstream); idx++) {
		bitstream[idx] = (uint8_t)random();
	}

	/** Fill-up bitlen to max size, depending on the modulation
	 * and set TX done callback if required
	 */
	eop_isr_cb = NULL;
	switch (rf_cfg_local.modulation) {
	case KNS_TX_MOD_LDA2:
	case KNS_TX_MOD_LDA2L:
		mw_cfg.valid = true;
		bitstream_bitlen = 304;
#ifndef KNS_RF_IN_BLOCKING_MODE
		eop_isr_cb = eoAtMW_isr_cb;
#endif
		break;
	case KNS_TX_MOD_VLDA4:
		mw_cfg.valid = true;
		bitstream_bitlen = 161;
#ifndef KNS_RF_IN_BLOCKING_MODE
		eop_isr_cb = eoAtMW_isr_cb;
#endif
		break;
	case KNS_TX_MOD_LDK:
		mw_cfg.valid = true;
		bitstream_bitlen = 270;
#ifndef KNS_RF_IN_BLOCKING_MODE
		eop_isr_cb = eoAtMW_isr_cb;
#endif
		break;
#ifdef USE_HDA4
	case KNS_TX_MOD_HDA4:
		mw_cfg.valid = true;
		bitstream_bitlen =  5551; // 5519-128-256=5135
#ifndef KNS_RF_IN_BLOCKING_MODE
		eop_isr_cb = eoAtMW_isr_cb;
#endif
		break;
#endif
	case KNS_TX_MOD_CW:
		mw_cfg.valid = false; // invalidate modulated wave config
		/** @attention For CW:
		 * * Ensure bit length is 0 as some driver may be based on this.
		 * * No need for callback as the RF driver only start CW for once
		 */
		bitstream_bitlen = 0;
		eop_isr_cb = NULL;
		break;
	case KNS_TX_MOD_NONE:
	default:
		return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		break;
	}

	isToBeTransmit = true;

	/** ---- Reply AT cmd response ----
	 * As ok to send frame, reply AT cmd response +OK directly before
	 * sending frames. Otherwise, TCXOWU may delay reply if done later
	 *
	 * Do this reply only the first burst
	 */
	if ((mw_cfg.valid == false) || (!mw_cfg.isRfAlreadyOn)) {
		(void)bMGR_AT_CMD_logSucceedMsg();
	}

	/** ---- Send frame ---- */
	status = KNS_RFTX_powerOn(NULL);
	if (status != KNS_STATUS_OK) {
		return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
	}

	status = KNS_RFTX_setCfg(&rf_cfg_local);
	if (status != KNS_STATUS_OK) {
		isToBeTransmit = false;
		(void)KNS_RFTX_abortRf(NULL);  /* MISRA Rule 17.7 - Error cleanup, return ignored */
		(void)KNS_RFTX_powerOff(NULL);
		return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
	}

	/** Keep pushing a bitstream, even with NULL length for CW */
	kns_assert(bitstream_bitlen <= (sizeof(bitstream) * 8U));
	status = KNS_RFTX_pushBitstream(bitstream, bitstream_bitlen);
	if (status != KNS_STATUS_OK) {
		isToBeTransmit = false;
		(void)KNS_RFTX_abortRf(NULL);
		(void)KNS_RFTX_powerOff(NULL);
		return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
	}

	/** Do the TCXO wramp for first transmission. No need to do it for next burst as
	 * power OFF/ON sequence between burst is saud to be very quick
	 */
	if ((mw_cfg.valid == true) && (!mw_cfg.isRfAlreadyOn)) {
		status = KNS_RFTX_tcxoWarmup(NULL);
	}

	if (status != KNS_STATUS_OK) {
		isToBeTransmit = false;
		(void)KNS_RFTX_abortRf(NULL);
		(void)KNS_RFTX_powerOff(NULL);
		return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
	}

	MGR_LOG_DEBUG("[%s] TX bitstream: 0x", __func__);
	MGR_LOG_array(bitstream, ((bitstream_bitlen + 7U) / 8U));

	status = KNS_RFTX_startImmediate(eop_isr_cb);
	if (status == KNS_STATUS_OK) {
		if (mw_cfg.valid == true) {
			mw_cfg.isRfAlreadyOn = true;
		}
		return true;
	}

	/** Stop RF and Send delayed error reply if failure during send process
	 */
	isToBeTransmit = false;
	(void)KNS_RFTX_abortRf(NULL);
	(void)KNS_RFTX_powerOff(NULL);
	return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) KNS_STATUS_RF_ERR);
}

/* Public functions ------------------------------------------------------------------------------*/

bool bMGR_AT_CMD_CW_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
#ifdef KNS_RF_IN_BLOCKING_MODE
	bool bool_status;
#endif
	int16_t scan_param_res;
	uint16_t cwMode = 0;
	uint16_t cwPwr = 0;
	int32_t cwFrq = 0;
	struct KNS_tx_rf_cfg_t rf_cfg;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Status mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

#ifdef SUPPORT_MW_WITH_DELAYED_RETX
	repPeriod_s = 0;
	scan_param_res = sscanf((const char *)pu8_cmdParamString,
			(const char *)"AT+CW=%hx,%ld,%hd,%hd",
			&cwMode, &cwFrq, &cwPwr, &repPeriod_s);
#else
	scan_param_res = sscanf((const char *)pu8_cmdParamString,
			(const char *)"AT+CW=%hx,%ld,%hd",
			&cwMode, &cwFrq, &cwPwr);
#endif

	switch (scan_param_res) {
	case (3):
		break;
#ifdef SUPPORT_MW_WITH_DELAYED_RETX
	case (4):
		MGR_LOG_VERBOSE("---- wait around %d sec between transmit ----\r\n", repPeriod_s);
		break;
#endif
	default:
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		break;

	}

	switch (cwMode) {
	case (1):
		rf_cfg.modulation = KNS_TX_MOD_CW;
		break;
	case (2):
		rf_cfg.modulation = KNS_TX_MOD_LDA2;
		break;
	case (3):
		rf_cfg.modulation = KNS_TX_MOD_LDA2L;
		break;
	case (4):
		rf_cfg.modulation = KNS_TX_MOD_VLDA4;
		break;
	case (5):
		rf_cfg.modulation = KNS_TX_MOD_LDK;
		break;
#ifdef USE_HDA4
	case (6):
		rf_cfg.modulation = KNS_TX_MOD_HDA4;
		break;
#endif
	case (0):
	default:
		rf_cfg.modulation = KNS_TX_MOD_NONE;
		break;
	}

	switch (cwMode) {
	case (1):
		rf_cfg.center_freq = cwFrq;
		rf_cfg.power = cwPwr;
		/** This function internaly returns AT response */
		return(MGR_AT_CMD_sendRandomTxData(&rf_cfg));
		break;
	case (2):
	case (3):
	case (4):
	case (5):
#ifdef USE_HDA4
	case (6):
#endif
	{
		rf_cfg.center_freq = cwFrq;
		rf_cfg.power = cwPwr;
#ifdef KNS_RF_IN_BLOCKING_MODE
		bool_status = MGR_AT_CMD_sendRandomTxData(&rf_cfg);
		// Transmit modulated burst contiunuously, but try to ccatch any new AT cmd received
		while (bool_status && !MGR_AT_CMD_isPendingAt()) {
			isToBeTransmit = false;
			KNS_RFTX_powerOff(NULL);
#ifdef SUPPORT_MW_WITH_DELAYED_RETX
			if (repPeriod_s > 1)
				mw_cfg.isRfAlreadyOn = false; // clear to force TCXOWU again
			if (repPeriod_s != 0)
				DELAY_MS(repPeriod_s * 1000);
#endif
			bool_status = MGR_AT_CMD_sendRandomTxData(NULL);
		};
		return false;
#else
		/** This function internaly returns AT response */
		return(MGR_AT_CMD_sendRandomTxData(&rf_cfg));
#endif
		break;
	}
	case (0):
	default:
		/** Stop carrier wave or modulated wave depending on mode */
		if (mw_cfg.valid == true) {
			/** Invalidate modulated wave configuration, */
			mw_cfg.valid = false;
			mw_cfg.isRfAlreadyOn = false;
		}
		switch (mw_cfg.rf_cfg.modulation) {
		case KNS_TX_MOD_LDA2:
		case KNS_TX_MOD_LDA2L:
		case KNS_TX_MOD_VLDA4:
		case KNS_TX_MOD_LDK:
		case KNS_TX_MOD_HDA4:
#ifndef KNS_RF_IN_BLOCKING_MODE
			/** Invalidate modulated wave configuration,
			 * Will stop infinite loop at next end-of-TX callback
			 */
			return bMGR_AT_CMD_logSucceedMsg(); // send immediate response
			break;
#endif
		case KNS_TX_MOD_CW:
		case KNS_TX_MOD_NONE:
		default:
			isToBeTransmit = false;
			status = KNS_RFTX_abortRf(NULL);
			if (status != KNS_STATUS_OK)
				return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
			status = KNS_RFTX_powerOff(NULL);
			if (status != KNS_STATUS_OK)
				return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
			else
				return bMGR_AT_CMD_logSucceedMsg();
			break;
		}
		break;
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
}

/**
 * @}
 */
