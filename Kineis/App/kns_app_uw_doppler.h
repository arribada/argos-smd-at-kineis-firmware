/**
 * @file    kns_app_uw_doppler.h
 * @brief   UW_DOPPLER application - Underwater/surface detection with BLIND-DOPPLER TX
 *
 * Monitors water conductivity via SWS (Salt Water Switch) and transmits satellite
 * messages when surface is detected using an incremental interval scheduling:
 *   T_n = T_initial * (1 + growth/100)^n
 *
 * The first TX is immediate upon surface detection, then intervals grow by the
 * configured percentage between each subsequent transmission.
 *
 * Deploy mode:
 *   - AT+DEPLOY=1 + AT+SAVE: marks device as deployed (TX enabled)
 *   - When deployed: UART listens for 5s at each boot (boot window).
 *     If no AT command is received, UART is fully de-initialized to save power.
 *     If an AT command is received during the window, UART stays active.
 *   - AT+DEPLOY=0 + AT+SAVE: returns to config mode (UART always active, no TX)
 */

#ifndef KNS_APP_UW_DOPPLER_H
#define KNS_APP_UW_DOPPLER_H

#include <stdint.h>
#include <stdbool.h>

/* ---- BLIND-DOPPLER TX Config ---- */

typedef struct {
	uint16_t tx_initial_interval_s; /**< Interval before 2nd TX after surface (default 10s) */
	uint8_t  tx_growth_percent;     /**< Interval growth % between TXs (default 10) */
	uint16_t tx_max_interval_s;     /**< Max interval cap (default 180s) */
	uint8_t  tx_max_count;          /**< Max TX count per surface event (0 = unlimited) */
	uint8_t  tx_jitter_percent;     /**< Random +/-% applied per TX interval (default 10, 0=off) */
	uint16_t tx_cooldown_s;         /**< Global post-TX quiet time (default 60s, 0=off).
	                                     Survives surface/dive cycles so a turtle that
	                                     resurfaces immediately can't burst-TX. */
} KNS_APP_UwDopplerTxCfg_t;

/** @brief Low-battery (LB) mode config.
 *
 * When VBAT drops below lb_enter_mV, the TX scheduler switches to slower /
 * fewer-TX values (lb_tx_*). It exits LB mode only above lb_exit_mV
 * (hysteresis), so a transient droop during TX doesn't oscillate the mode.
 *
 * Set lb_enter_mV=0 to disable LB mode entirely.
 */
typedef struct {
	uint16_t lb_enter_mV;        /**< Enter LB mode at this voltage (0=disabled) */
	uint16_t lb_exit_mV;         /**< Exit LB mode above this voltage (must > enter_mV) */
	uint16_t lb_tx_interval_s;   /**< Replaces tx_initial_interval_s while in LB mode */
	uint16_t lb_tx_max_s;        /**< Replaces tx_max_interval_s while in LB mode */
	uint8_t  lb_tx_max_count;    /**< Replaces tx_max_count while in LB mode */
	uint8_t  _pad;
} KNS_APP_UwDopplerLbCfg_t;

/* ---- API ---- */

/** @brief Initialize UW_DOPPLER application and MAC profile */
void KNS_APP_uw_doppler_init(void);

/** @brief Main UW_DOPPLER loop - register with KNS_OS as APP task */
void KNS_APP_uw_doppler_loop(void);

/** @brief Get current TX config */
KNS_APP_UwDopplerTxCfg_t KNS_APP_uw_doppler_getTxCfg(void);

/** @brief Set TX config */
void KNS_APP_uw_doppler_setTxCfg(const KNS_APP_UwDopplerTxCfg_t *cfg);

/** @brief Get deploy mode (0=config, 1=deployed) */
uint8_t KNS_APP_uw_doppler_getDeployMode(void);

/**
 * @brief Set deploy mode
 *
 * When deployed (mode=1), TX is enabled and UART is disabled after
 * a 5s boot window unless an AT command is received during the window.
 * When not deployed (mode=0), SWS runs but no TX, UART always active.
 * Must call AT+SAVE to persist.
 *
 * @param mode 0=config mode, 1=deployed
 */
void KNS_APP_uw_doppler_setDeployMode(uint8_t mode);

/** @brief Restore SWS baselines from retention RAM (call after MGR_SWS_init()) */
void KNS_APP_uw_doppler_restoreSwsBaselines(void);

/** @brief Get current state-machine state (numeric, for AT+STATUS).
 *  Avoids exporting the enum: callers only need the integer for reporting. */
uint8_t KNS_APP_uw_doppler_getStateRaw(void);

/** @brief Current TX count since boot (in current surface event window). */
uint32_t KNS_APP_uw_doppler_getTxCountSession(void);

/** @brief LB mode getters / setters */
KNS_APP_UwDopplerLbCfg_t KNS_APP_uw_doppler_getLbCfg(void);
void KNS_APP_uw_doppler_setLbCfg(const KNS_APP_UwDopplerLbCfg_t *cfg);
bool KNS_APP_uw_doppler_isLbActive(void);

/**
 * @brief Sprint 4: trigger a forced TX burst (test / validation).
 *
 * Queues `count` test transmissions that bypass the rate limiter, the post-TX
 * cooldown, the error backoff, the surface check and the deploy_mode gate.
 * Each TX still goes through the standard MAC/PA path so it really hits the
 * antenna — useful for at-deployment validation and RF debugging.
 *
 * The MIN_INTER_TX_INTERVAL_MS safety floor (5 s) is still honored to protect
 * the PA from back-to-back inrush.
 *
 * @param count  Number of TXs to send (clamped to [1..10]).
 * @return       The clamped count actually queued.
 */
uint8_t KNS_APP_uw_doppler_startTestBurst(uint8_t count);

/** @brief How many test TXs are still pending in the current burst. */
uint8_t KNS_APP_uw_doppler_getTestBurstRemaining(void);

#endif /* KNS_APP_UW_DOPPLER_H */
