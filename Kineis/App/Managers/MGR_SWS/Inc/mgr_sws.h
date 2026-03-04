/**
 * @file    mgr_sws.h
 * @brief   Salt Water Switch manager - Adaptive underwater/surface detection
 *
 * Implements biofouling-adaptive detection algorithm with:
 * - Moving average filter + trend/variance analysis
 * - 3-tier rapid transition detection (T1/T2/T3)
 * - Dynamic EMA water baseline + adaptive air recalibration
 * - Max dive timeout safety + surface lockout
 *
 * Hardware: PA12 = sensor power control, PA11 = ADC_IN7 input
 * ADC: 12-bit (0-4095), adapted from 14-bit spec (values ÷4)
 */

#ifndef __MGR_SWS_H__
#define __MGR_SWS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- State ---- */

typedef enum {
	MGR_SWS_STATE_UNKNOWN    = 0,
	MGR_SWS_STATE_SURFACE    = 1,
	MGR_SWS_STATE_UNDERWATER = 2,
} MGR_SWS_State_t;

/* ---- Configuration ---- */

typedef struct {
	uint16_t threshold_min;          /**< Min valid ADC value (default 13 = 50/4) */
	uint16_t threshold_max;          /**< Max valid ADC value (default 2000 = 8000/4) */
	uint16_t initial_air_baseline;   /**< Initial air baseline (default 50 = 200/4) */
	uint16_t initial_water_baseline; /**< Initial water baseline (default 750 = 3000/4) */
	uint32_t test_interval_ms;       /**< Measurement interval in ms (default 1000) */
	uint32_t max_dive_time_s;        /**< Max dive time before forced surface (default 7200) */
	uint32_t min_surface_time_s;     /**< Min surface time before re-submersion (default 10) */
	bool     enabled;                /**< SWS detection enabled */
} MGR_SWS_Config_t;

/* ---- API ---- */

/** @brief Initialize SWS manager, GPIO PA12, and ADC */
void MGR_SWS_init(void);

/** @brief Periodic task - call from OS scheduler or app loop */
void MGR_SWS_task(void);

/** @brief Get current detection state */
MGR_SWS_State_t MGR_SWS_getState(void);

/** @brief Get last raw ADC reading */
uint16_t MGR_SWS_getLastADC(void);

/** @brief Get current detection config */
MGR_SWS_Config_t MGR_SWS_getConfig(void);

/** @brief Set detection config */
void MGR_SWS_setConfig(const MGR_SWS_Config_t *config);

/** @brief Force immediate measurement (bypass interval timer) */
void MGR_SWS_forceMeasurement(void);

/** @brief Check if state changed since last call (auto-resets flag) */
bool MGR_SWS_stateChanged(void);

/** @brief Get current adapted air baseline (for retention across resets) */
uint16_t MGR_SWS_getAirBaseline(void);

/** @brief Get current adapted water baseline (for retention across resets) */
uint16_t MGR_SWS_getWaterBaseline(void);

/** @brief Restore adapted baselines after a warm reset
 *  @param air   Previously saved air baseline (0 = don't restore)
 *  @param water Previously saved water baseline (0 = don't restore)
 */
void MGR_SWS_restoreBaselines(uint16_t air, uint16_t water);

#ifdef __cplusplus
}
#endif

#endif /* __MGR_SWS_H__ */
