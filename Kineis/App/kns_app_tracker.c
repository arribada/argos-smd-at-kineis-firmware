/**
 * @file    kns_app_tracker.c
 * @brief   TRACKER application - SWS monitoring with BLIND-DOPPLER TX scheduling
 *
 * State machine:
 *   INIT_MAC → WAIT_MAC_READY → MONITORING → SURFACE_TX → WAIT_TX_DONE → MONITORING
 *
 * TX scheduling (BLIND-DOPPLER):
 *   On surface detection, first TX is immediate.
 *   Subsequent TXs use incremental intervals: T_n = T_initial * (1 + growth/100)^n
 *   Resets when going back underwater.
 */

#include <stdbool.h>
#include <stdlib.h>
#include "kns_app_tracker.h"
#include "mgr_sws.h"
#include "stm32wlxx_hal.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kns_cfg.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"

/* ---- State Machine ---- */

typedef enum {
	TRACKER_INIT_MAC,
	TRACKER_WAIT_MAC_READY,
	TRACKER_MONITORING,
	TRACKER_SURFACE_TX,
	TRACKER_WAIT_TX_DONE,
} TrackerState_t;

/* ---- Private variables ---- */

static __attribute__((__section__(".retentionRamData")))
TrackerState_t tracker_state;

static KNS_APP_TrackerTxCfg_t tx_cfg = {
	.tx_initial_interval_s = 10,
	.tx_growth_percent     = 10,
	.tx_max_interval_s     = 600,
	.tx_max_count          = 0,  /* unlimited */
};

/* TX scheduling state */
static uint32_t tx_count = 0;              /**< Number of TXs sent in current surface event */
static uint32_t last_tx_tick = 0;          /**< Tick of last TX */
static uint32_t current_interval_ms = 0;   /**< Current interval between TXs in ms */
static bool     surface_tx_pending = false; /**< Immediate TX needed on surface detection */

/* MAC profile config: BASIC single TX per message */
static struct KNS_MAC_BLIND_usrCfg_t prflBlindUserCfg = {
	.retx_nb = 0,
	.retx_period_s = 60,
	.nb_parrallel_msg = 1
};

/* ---- Helpers ---- */

static uint32_t compute_next_interval_ms(uint32_t n)
{
	/* T_n = T_initial * (1 + growth/100)^n
	 * Computed iteratively to avoid floating point.
	 * Each step: interval = interval * (100 + growth) / 100
	 */
	uint32_t interval_ms = (uint32_t)tx_cfg.tx_initial_interval_s * 1000;
	for (uint32_t i = 0; i < n; i++) {
		interval_ms = interval_ms * (100 + tx_cfg.tx_growth_percent) / 100;
		/* Cap to max */
		if (interval_ms > (uint32_t)tx_cfg.tx_max_interval_s * 1000) {
			interval_ms = (uint32_t)tx_cfg.tx_max_interval_s * 1000;
			break;
		}
	}
	return interval_ms;
}

static void reset_tx_scheduling(void)
{
	tx_count = 0;
	last_tx_tick = 0;
	current_interval_ms = 0;
	surface_tx_pending = false;
}

static void build_tx_payload(struct KNS_MAC_appEvt_t *appEvt)
{
	struct KNS_CFG_radio_t device_radio_cfg;
	kns_assert(KNS_CFG_getRadioInfo(&device_radio_cfg) == KNS_STATUS_OK);

	appEvt->id = KNS_MAC_SEND_DATA;
	appEvt->data_ctxt.sf = KNS_SF_NO_SERVICE;

	/* Build payload: [state(1B)][adc(2B)][tx_count(1B)] + padding */
	MGR_SWS_State_t state = MGR_SWS_getState();
	uint16_t adc = MGR_SWS_getLastADC();

	appEvt->data_ctxt.usrdata[0] = (uint8_t)state;
	appEvt->data_ctxt.usrdata[1] = (uint8_t)(adc >> 8);
	appEvt->data_ctxt.usrdata[2] = (uint8_t)(adc & 0xFF);
	appEvt->data_ctxt.usrdata[3] = (uint8_t)(tx_count & 0xFF);

	/* Fill rest with zeros */
	for (uint16_t i = 4; i < KNS_MAC_USRDATA_MAXLEN; i++)
		appEvt->data_ctxt.usrdata[i] = 0;

	/* Set bitlen based on modulation */
	switch (device_radio_cfg.modulation) {
	case KNS_TX_MOD_LDA2:
		appEvt->data_ctxt.usrdata_bitlen = 192;
		break;
	case KNS_TX_MOD_LDA2L:
		appEvt->data_ctxt.usrdata_bitlen = 196;
		break;
	case KNS_TX_MOD_VLDA4:
		appEvt->data_ctxt.usrdata_bitlen = 24;
		break;
	case KNS_TX_MOD_LDK:
		appEvt->data_ctxt.usrdata_bitlen = 128;
		break;
	default:
		kns_assert(0);
		break;
	}
}

static void start_mac_profile(void)
{
	enum KNS_status_t status;
	struct KNS_MAC_appEvt_t appEvt;

	appEvt.id = KNS_MAC_INIT;
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BLIND;
	appEvt.init_prfl_ctxt.blindCfg = prflBlindUserCfg;

	status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
	if (status != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[TRACKER] MAC init failed: 0x%x\r\n", status);
		kns_assert(0);
	}
}

static bool process_mac_events(void)
{
	enum KNS_status_t status;
	struct KNS_MAC_srvcEvt_t srvcEvt;
	bool got_event = false;

	for (status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);
	     status != KNS_STATUS_QEMPTY;
	     status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt)) {
		if (status != KNS_STATUS_OK)
			continue;

		got_event = true;

		switch (srvcEvt.id) {
		case KNS_MAC_OK:
			if (tracker_state == TRACKER_WAIT_MAC_READY) {
				MGR_LOG_DEBUG("[TRACKER] MAC ready\r\n");
				tracker_state = TRACKER_MONITORING;
			} else if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
				MGR_LOG_DEBUG("[TRACKER] TX accepted\r\n");
			}
			break;

		case KNS_MAC_TX_DONE:
			MGR_LOG_DEBUG("[TRACKER] TX done (#%lu)\r\n", tx_count);
			if (tracker_state == TRACKER_WAIT_TX_DONE)
				tracker_state = TRACKER_MONITORING;
			break;

		case KNS_MAC_TX_TIMEOUT:
			MGR_LOG_DEBUG("[TRACKER] TX timeout\r\n");
			if (tracker_state == TRACKER_WAIT_TX_DONE)
				tracker_state = TRACKER_MONITORING;
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[TRACKER] MAC error: %d\r\n", srvcEvt.status);
			if (tracker_state == TRACKER_WAIT_TX_DONE)
				tracker_state = TRACKER_MONITORING;
			else if (tracker_state == TRACKER_WAIT_MAC_READY)
				tracker_state = TRACKER_INIT_MAC; /* retry */
			break;

		default:
			break;
		}
	}

	return got_event;
}

/* ---- Public API ---- */

void KNS_APP_tracker_init(void)
{
	tracker_state = TRACKER_INIT_MAC;
	reset_tx_scheduling();
	MGR_LOG_DEBUG("[TRACKER] Init: interval=%us growth=%u%% max=%us\r\n",
		tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent, tx_cfg.tx_max_interval_s);
}

void KNS_APP_tracker_loop(void)
{
	/* Always run SWS measurement */
	MGR_SWS_task();

	switch (tracker_state) {
	case TRACKER_INIT_MAC:
		start_mac_profile();
		tracker_state = TRACKER_WAIT_MAC_READY;
		return;

	case TRACKER_WAIT_MAC_READY:
		process_mac_events();
		return;

	case TRACKER_MONITORING:
	{
		/* Process any pending MAC events */
		process_mac_events();

		MGR_SWS_State_t sws_state = MGR_SWS_getState();

		/* Check for surface detection */
		if (MGR_SWS_stateChanged()) {
			if (sws_state == MGR_SWS_STATE_SURFACE) {
				/* Surface detected! Schedule immediate TX */
				MGR_LOG_DEBUG("[TRACKER] Surface detected, starting TX\r\n");
				reset_tx_scheduling();
				surface_tx_pending = true;
			} else {
				/* Went underwater, stop TX scheduling */
				MGR_LOG_DEBUG("[TRACKER] Underwater, stopping TX\r\n");
				reset_tx_scheduling();
			}
		}

		/* TX scheduling logic */
		if (sws_state == MGR_SWS_STATE_SURFACE) {
			bool should_tx = false;

			if (surface_tx_pending) {
				/* Immediate first TX */
				should_tx = true;
				surface_tx_pending = false;
			} else if (tx_count > 0 && current_interval_ms > 0) {
				/* Check if interval has elapsed */
				uint32_t elapsed = HAL_GetTick() - last_tx_tick;
				if (elapsed >= current_interval_ms)
					should_tx = true;
			}

			/* Check max TX count */
			if (should_tx && tx_cfg.tx_max_count > 0 && tx_count >= tx_cfg.tx_max_count)
				should_tx = false;

			if (should_tx) {
				tracker_state = TRACKER_SURFACE_TX;
				/* Fall through to SURFACE_TX */
			} else {
				return;
			}
		} else {
			return;
		}
	}
	/* Fall through */

	case TRACKER_SURFACE_TX:
	{
		struct KNS_MAC_appEvt_t appEvt;
		build_tx_payload(&appEvt);

		enum KNS_status_t status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
		if (status == KNS_STATUS_OK) {
			MGR_LOG_DEBUG("[TRACKER] TX #%lu sent (interval=%lums)\r\n",
				tx_count, current_interval_ms);

			last_tx_tick = HAL_GetTick();
			current_interval_ms = compute_next_interval_ms(tx_count);
			tx_count++;

			tracker_state = TRACKER_WAIT_TX_DONE;
		} else {
			MGR_LOG_DEBUG("[TRACKER] TX push failed: 0x%x\r\n", status);
			tracker_state = TRACKER_MONITORING;
		}
		return;
	}

	case TRACKER_WAIT_TX_DONE:
		process_mac_events();
		return;

	default:
		tracker_state = TRACKER_INIT_MAC;
		break;
	}
}

KNS_APP_TrackerTxCfg_t KNS_APP_tracker_getTxCfg(void)
{
	return tx_cfg;
}

void KNS_APP_tracker_setTxCfg(const KNS_APP_TrackerTxCfg_t *cfg)
{
	if (cfg) {
		tx_cfg = *cfg;
		MGR_LOG_DEBUG("[TRACKER] TX config: interval=%us growth=%u%% max=%us count=%u\r\n",
			tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
			tx_cfg.tx_max_interval_s, tx_cfg.tx_max_count);
	}
}
