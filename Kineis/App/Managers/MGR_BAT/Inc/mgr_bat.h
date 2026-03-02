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

#endif /* MGR_BAT_H */
