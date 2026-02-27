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
 */

#ifndef KNS_APP_UW_DOPPLER_H
#define KNS_APP_UW_DOPPLER_H

#include <stdint.h>

/* ---- BLIND-DOPPLER TX Config ---- */

typedef struct {
	uint16_t tx_initial_interval_s; /**< Interval before 2nd TX after surface (default 10s) */
	uint8_t  tx_growth_percent;     /**< Interval growth % between TXs (default 10) */
	uint16_t tx_max_interval_s;     /**< Max interval cap (default 600s) */
	uint8_t  tx_max_count;          /**< Max TX count per surface event (0 = unlimited) */
} KNS_APP_UwDopplerTxCfg_t;

/* ---- API ---- */

/** @brief Initialize UW_DOPPLER application and MAC profile */
void KNS_APP_uw_doppler_init(void);

/** @brief Main UW_DOPPLER loop - register with KNS_OS as APP task */
void KNS_APP_uw_doppler_loop(void);

/** @brief Get current TX config */
KNS_APP_UwDopplerTxCfg_t KNS_APP_uw_doppler_getTxCfg(void);

/** @brief Set TX config */
void KNS_APP_uw_doppler_setTxCfg(const KNS_APP_UwDopplerTxCfg_t *cfg);

/** @brief Get deploy mode (1=deployed, 0=not deployed) */
uint8_t KNS_APP_uw_doppler_getDeployMode(void);

/** @brief Set deploy mode (1=deployed, 0=not deployed) */
void KNS_APP_uw_doppler_setDeployMode(uint8_t mode);

#endif /* KNS_APP_UW_DOPPLER_H */
