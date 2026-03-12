/**
 * @file    kns_app_doppler.h
 * @brief   DOPPLER application - Periodic satellite message transmitter
 *
 * Simple periodic transmitter with two operating modes:
 *   - With MCU_DONE (TPL5111): power-cycled by external timer, modulo-based scheduling
 *   - Without MCU_DONE: RTC wakeup timer between sequences, deepest LPM
 *
 * A "sequence" = N messages sent with a fixed interval between each.
 * Compatible with all boards (SMD_STDALONE, SMD_PA, SMD_NOPA, SMD_OP).
 *
 * Deploy mode:
 *   - AT+DEPLOY=1 + AT+SAVE: marks device as deployed
 *   - When deployed: UART listens for 5s at each boot (boot window).
 *     If no AT command is received, UART is fully de-initialized to save power.
 *     If an AT command is received during the window, UART stays active.
 *   - AT+DEPLOY=0 + AT+SAVE: returns to config mode (UART always active)
 */

#ifndef KNS_APP_DOPPLER_H
#define KNS_APP_DOPPLER_H

#include <stdbool.h>
#include <stdint.h>

/* ---- DOPPLER TX Config ---- */

typedef struct {
	uint8_t  msg_count;           /**< Number of messages per sequence (1-255) */
	uint32_t msg_interval_s;      /**< Fixed interval between messages in a sequence (seconds) */
	uint32_t sequence_interval_s; /**< Time between two sequences (seconds) */
	uint32_t tpl_interval_s;      /**< TPL5111 hardware timer period (seconds, 0 = no TPL) */
} KNS_APP_DopplerCfg_t;

/* ---- API ---- */

/** @brief Initialize DOPPLER application */
void KNS_APP_doppler_init(void);

/** @brief Main DOPPLER loop - register with KNS_OS as APP task */
void KNS_APP_doppler_loop(void);

/** @brief Get current config (copy) */
KNS_APP_DopplerCfg_t KNS_APP_doppler_getCfg(void);

/**
 * @brief Set config with validation
 *
 * Validates parameters before applying:
 *   - msg_count, msg_interval_s, sequence_interval_s must be > 0
 *   - msg_interval_s must not overflow when converted to ms (max ~4294967s)
 *   - For BLIND profile: msg_interval_s must fit in uint16_t (max 65535s)
 *
 * @param cfg Pointer to new config
 * @return true on success, false if validation failed (config unchanged)
 */
bool KNS_APP_doppler_setCfg(const KNS_APP_DopplerCfg_t *cfg);

/**
 * @brief Save current config to NVM flash
 *
 * Only writes if config has been modified since last save (dirty flag).
 *
 * @return true on success or if no write needed, false on flash error
 */
bool KNS_APP_doppler_nvmSave(void);

/**
 * @brief Mark NVM config as dirty (needs save)
 *
 * Called when a setting that is persisted in NVM changes outside of setCfg(),
 * e.g. LED mode changed via AT+LED.
 */
void KNS_APP_doppler_markDirty(void);

/** @brief Get deploy mode (0=config, 1=deployed) */
uint8_t KNS_APP_doppler_getDeployMode(void);

/**
 * @brief Set deploy mode
 *
 * When deployed (mode=1), UART is disabled after a 5s boot window
 * unless an AT command is received during the window.
 * Must call KNS_APP_doppler_nvmSave() to persist.
 *
 * @param mode 0=config mode, 1=deployed
 */
void KNS_APP_doppler_setDeployMode(uint8_t mode);

#endif /* KNS_APP_DOPPLER_H */
