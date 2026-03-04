/**
 * @file    mgr_bat.h
 * @brief   Battery monitoring via internal VBAT/3 ADC channel
 *
 * Reads battery voltage using the STM32WL55 internal VBAT ADC channel.
 * Optionally enables VBAT_EN (PB9) for external measurement circuit.
 *
 * Note: PB13 (VBAT_ADC in BSP) is NOT an ADC pin on STM32WL55
 * (it's COMP1_INP only). We use the internal ADC_CHANNEL_VBAT instead.
 */

#ifndef MGR_BAT_H
#define MGR_BAT_H

#include <stdint.h>
#include <stdbool.h>

/** @brief Default minimum voltage for TX (mV). Below this, TX is inhibited. */
#define MGR_BAT_DEFAULT_MIN_TX_MV  2800

/** @brief Initialize battery monitoring (configure VBAT_EN pin) */
void MGR_BAT_init(void);

/**
 * @brief Read battery voltage
 * @return Voltage in millivolts (e.g. 3300 for 3.3V)
 */
uint16_t MGR_BAT_readVoltage_mV(void);

/**
 * @brief Estimate battery level
 * @return 0-100 percentage
 */
uint8_t MGR_BAT_getLevel(void);

/**
 * @brief Check if battery voltage is above minimum TX threshold
 * @return true if voltage is sufficient for TX (or if VBAT ADC unavailable)
 */
bool MGR_BAT_isTxAllowed(void);

/** @brief Get minimum TX voltage threshold in mV */
uint16_t MGR_BAT_getMinTxVoltage_mV(void);

/** @brief Set minimum TX voltage threshold in mV (0 = disabled) */
void MGR_BAT_setMinTxVoltage_mV(uint16_t min_mV);

#endif /* MGR_BAT_H */
