/**
 * @file    kns_app_tracker.h
 * @brief   TRACKER application - Underwater/surface detection with BLIND-DOPPLER TX
 *
 * Monitors water conductivity via SWS (Salt Water Switch) and transmits satellite
 * messages when surface is detected using an incremental interval scheduling:
 *   T_n = T_initial * (1 + growth/100)^n
 *
 * The first TX is immediate upon surface detection, then intervals grow by the
 * configured percentage between each subsequent transmission.
 */

#ifndef KNS_APP_TRACKER_H
#define KNS_APP_TRACKER_H

#include <stdint.h>

/* ---- BLIND-DOPPLER TX Config ---- */

typedef struct {
	uint16_t tx_initial_interval_s; /**< Interval before 2nd TX after surface (default 10s) */
	uint8_t  tx_growth_percent;     /**< Interval growth % between TXs (default 10) */
	uint16_t tx_max_interval_s;     /**< Max interval cap (default 600s) */
	uint8_t  tx_max_count;          /**< Max TX count per surface event (0 = unlimited) */
} KNS_APP_TrackerTxCfg_t;

/* ---- API ---- */

/** @brief Initialize tracker application and MAC profile */
void KNS_APP_tracker_init(void);

/** @brief Main tracker loop - register with KNS_OS as APP task */
void KNS_APP_tracker_loop(void);

/** @brief Get current TX config */
KNS_APP_TrackerTxCfg_t KNS_APP_tracker_getTxCfg(void);

/** @brief Set TX config */
void KNS_APP_tracker_setTxCfg(const KNS_APP_TrackerTxCfg_t *cfg);

#endif /* KNS_APP_TRACKER_H */
