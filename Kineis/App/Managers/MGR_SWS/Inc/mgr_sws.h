/**
 * @file    mgr_sws.h
 * @brief   Salt Water Switch manager - 5-level adaptive underwater/surface detection
 *
 * Implements 5-level surface detection algorithm (port of Linkit v4 hardened):
 * - L1: Instant drop from previous raw (4%, 1 sample)
 * - L2: Consecutive 2-sample raw drops (3% cumulative, each step >=2%)
 * - L3: MA3 trend (3+ decreases, 4% total)
 * - L4: Absolute water baseline drop (8%)
 * - L5: Dive peak safety net (10%, >10s gate)
 * - Proximity guard adaptive (95% normal / 99% biofouling)
 * - Dynamic threshold ratio, 4% hysteresis, AIR_BASELINE_FLOOR
 * - Anti-spike observed_peak filter, dive timeout escalation
 * - Stuck-state recovery (death-spiral), continuous coherence check
 * - Adaptive sample delay (RC charge tuning)
 *
 * Hardware: PA12 = sensor power control, PA11 = ADC_IN7 input
 * ADC: 12-bit (0-4095), adapted from 14-bit Linkit spec (values /4)
 *
 * Polling: distinct intervals for SURFACE state (less frequent, save power)
 *          and UNDERWATER state (frequent, fast surface detection).
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
	uint16_t threshold_min;              /**< Min valid ADC value (default 0) */
	uint16_t threshold_max;              /**< Max valid ADC value (default 2000 = 8000/4) */
	uint16_t initial_air_baseline;       /**< Initial air baseline (default 50 = 200/4) */
	uint16_t initial_water_baseline;     /**< Initial water baseline (default 750 = 3000/4) */
	uint32_t test_interval_surface_ms;   /**< Measurement interval at SURFACE (default 5000) */
	uint32_t test_interval_underwater_ms;/**< Measurement interval UNDERWATER (default 1000) */
	uint32_t max_dive_time_s;            /**< Max dive time before forced surface (default 7200) */
	uint32_t min_surface_time_s;         /**< Min surface time before re-submersion (default 10) */
	uint16_t sample_delay_min_us;        /**< Adaptive RC charge delay floor (default 200) */
	uint16_t sample_delay_max_us;        /**< Adaptive RC charge delay ceiling (default 5000) */
	uint16_t sample_delay_default_us;    /**< Starting RC charge delay (default 1000) */
	bool     enabled;                    /**< SWS detection enabled */
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

/** @brief Get the configured sampling interval (ms) for SURFACE state.
 *  The event-driven LPM scheduler uses this to compute the next wake. */
uint32_t MGR_SWS_getSurfIntervalMs(void);

/** @brief Get the configured sampling interval (ms) for UNDERWATER state. */
uint32_t MGR_SWS_getUWIntervalMs(void);

/** @brief Force immediate measurement (bypass interval timer) */
void MGR_SWS_forceMeasurement(void);

/** @brief Check if state changed since last call (auto-resets flag) */
bool MGR_SWS_stateChanged(void);

/** @brief Get current adapted air baseline (for retention/flash persistence) */
uint16_t MGR_SWS_getAirBaseline(void);

/** @brief Get current adapted water baseline (for retention/flash persistence) */
uint16_t MGR_SWS_getWaterBaseline(void);

/** @brief Restore adapted baselines after a reset
 *  @param air   Previously saved air baseline (0 = don't restore)
 *  @param water Previously saved water baseline (0 = don't restore)
 */
void MGR_SWS_restoreBaselines(uint16_t air, uint16_t water);

/** @brief Get observed peak ADC value (for retention/flash persistence) */
uint16_t MGR_SWS_getObservedPeak(void);

/** @brief Restore observed peak ADC from flash/retention RAM */
void MGR_SWS_restoreObservedPeak(uint16_t peak);

/** @brief Get current adaptive sample delay in microseconds (for diagnostics) */
uint16_t MGR_SWS_getSampleDelayUs(void);

/** @brief Get current dynamic threshold (for diagnostics) */
uint16_t MGR_SWS_getThreshold(void);

/** @brief Check if calibration data has changed since last clear (for debounced flash save).
 *         Returns true if any baseline/peak update happened.
 */
bool MGR_SWS_calibDirty(void);

/** @brief Clear calibration-dirty flag (call after persisting to flash) */
void MGR_SWS_clearCalibDirty(void);

/** @brief Configure SWS GPIO for low-power STOP mode (analog input = no leakage) */
void MGR_SWS_enterLowPower(void);

/** @brief Restore SWS GPIO after STOP mode wakeup */
void MGR_SWS_exitLowPower(void);

/* ---- Sensor fault detection (Sprint 3) ----------------------------------- */

/** Bitfield of detected SWS sensor faults. 0 = healthy. */
#define MGR_SWS_FAULT_NONE          0x00u
#define MGR_SWS_FAULT_STUCK         0x01u  /**< Same ADC value for N consecutive reads */
#define MGR_SWS_FAULT_OUT_OF_RANGE  0x02u  /**< ADC rails low (<5) or high (>4090) */
#define MGR_SWS_FAULT_NO_VARIANCE   0x04u  /**< Variance over window too low (dead sensor) */

/**
 * @brief Get the OR'ed bitfield of currently detected sensor faults.
 * @return One of MGR_SWS_FAULT_* (NONE if all checks pass).
 */
uint8_t MGR_SWS_getFault(void);

#ifdef __cplusplus
}
#endif

#endif /* __MGR_SWS_H__ */
