/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_misc.h
 * @brief   MCU wrappers for miscellaneous features
 * @author  shamard
 * @date    Creation 2022/11/18
 * @version 1.0
 * @note
 */

/**
 * @page mcu_misc_page MCU wrappers: MISCellaneous utilities
 *
 * So far, the RF DRIVER needs some miscellaneous utilies such as:
 * * turn the externam PA ON/OFF
 * * get RF HW settings to configure the output power properly
 */

/**
 * @addtogroup MCU_WRAPPERS
 * @brief MCU wrapper used for extra little features
 * @{
 */

#ifndef MCU_MISC_H_
#define MCU_MISC_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "kns_types.h"

#define MCU_PA_BOOTDELAY_MS 250  /* extended from 100ms: gives TPS22904 soft-start + PA bias + supply recovery extra margin against brownout */
/* Structures ----------------------------------------------------------------------------------- */

/** @attention Structure below is platform specific, it may not fit your needs
 * @attention on KRD_MP, KRD_LP, DO NOT CHANGE this structure as the Kineis stack use using same
 * format in its RF driver.
 * */
struct rfSettings_t {
	bool isRficHPA;        // if true use HPA of the STM32WL55 chip, else use LPA
	int8_t rficPaLevel;    // output power of the RFIC internal PA in dBm
	int8_t externalPaGain; // output gain of the external PA in dB
};

/* Function declaration -------------------------------------------------------------*/

/**
 * @brief Power ON external PA
 *
 * In case the HW design contains some external PA, this API is called by Kineis stack to
 * power it ON during transmit process
 */
void MCU_MISC_turn_on_pa();

/**
 * @brief Power OFF external PA
 *
 * In case the HW design contains some external PA, this API is called by Kineis stack to
 * power it OFF during transmit process
 */
void MCU_MISC_turn_off_pa();

/**
 * @brief get HW settings of the platform concerning RF
 *
 * This API is called by Kineis stack to get RF HW settings to reach an expected output power level.
 *
 * This is needed to properly set and configure internal/external PA.
 *
 * As the settings are specific to HW design, the list of parameters depends of it. E.g. on KRD:
 * struct rfSettings_t {
 *	bool isRficHPA;        // if true use HPA of the STM32WL55 chip, else use LPA
 *	int8_t rficPaLevel;    // output power of the RFIC internal PA in dBm
 *	int8_t externalPaGain; // output gain of the external PA in dB
 * };
 *
 * @attention The corresponding driver in Kineis stack will check HW settings are coherent:
 * * rficPaLevel shall fit in the capabilities of the platform. E.g.:
 *     * KRD_MP: RFIC_PWR_MIN =-17 dBm, RFIC_PWR_MAX = 15 dBm
 *     * KRD_LP: RFIC_PWR_MIN =-9 dBm, RFIC_PWR_MAX = 22 dBm
 * * The sum of all PA levels shall be equal to the requested input RF level. E.g. in case of KRD:
 *       rficPaLevel + externalPaGain == rf_level_dBm
 *     * KRD_MP: RF_LEVEL_MIN =19 dBm, RF_LEVEL_MAX = 51 dBm (out of spec)
 *     * KRD_LP: RF_LEVEL_MIN =-9 dBm, RF_LEVEL_MAX = 22 dBm
 *
 * @param[in]     rf_level_dBm Expected output RF level in dBm
 * @param[in,out] rfSettings   Pointer to struct containing settings. Format is void * to be generic
 * to any platform. Both calle and caller are said to know the real format depending on the platform
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t MCU_MISC_getSettingsHwRf(int8_t rf_level_dBm, void* rfSettings);

/**
 * @brief Force enable or disable the TCXO.
 *
 * This function forces the state of the Temperature Compensated Crystal Oscillator (TCXO).
 * When the TCXO is forced to be enabled, it will be activated; when forced to be disabled,
 * it will be turned off regardless of automatic control.
 *
 * @param[in] enable Set to true to force-enable the TCXO, or false to force-disable it.
 */
void MCU_MISC_TCXO_Force_State(bool enable);

/**
 * @brief Set the warmup time for the TCXO.
 *
 * This function configures the warmup duration required by the TCXO after it is enabled.
 * The warmup time ensures that the oscillator stabilizes before being used.
 *
 * @param[in] time_ms Warmup time in milliseconds.
 */
void MCU_MISC_TCXO_set_warmup(uint32_t time_ms);

/**
 * @brief Retrieve the current TCXO warmup time.
 *
 * This function retrieves the current warmup time setting for the TCXO.
 *
 * @param[out] time_ms Pointer to a variable where the warmup time (in milliseconds) will be stored.
 */
void MCU_MISC_TCXO_get_warmup(uint32_t *time_ms);

#endif /* MCU_MISC_H_ */

/**
 * @}
 */
