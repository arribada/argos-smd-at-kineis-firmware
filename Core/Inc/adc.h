/**
 * @file    adc.h
 * @brief   ADC peripheral driver for SWS analog measurement
 *
 * Configures ADC on PA11 (ADC_IN7) for salt water switch conductivity reading.
 * Used in TRACKER mode for underwater/surface detection.
 */

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wlxx_hal.h"
#include <stdbool.h>

/** ADC handle (defined in adc.c) */
extern ADC_HandleTypeDef hadc;

/** @brief Initialize ADC peripheral for SWS measurement on PA11 (ADC_IN7) */
void MX_ADC_Init(void);

/** @brief De-initialize ADC peripheral (for low power mode) */
void MX_ADC_DeInit(void);

/** @brief Read single ADC conversion from PA11
 *  @return Raw ADC value (0-4095 for 12-bit) or 0 on error (legacy; prefer
 *          ADC_ReadValueChecked to distinguish a failure from a genuine 0)
 */
uint32_t ADC_ReadValue(void);

/** @brief Read single ADC conversion, reporting HAL success separately.
 *  @param out  receives the raw value on success (unchanged on failure)
 *  @return true if the conversion succeeded, false on any HAL error
 */
bool ADC_ReadValueChecked(uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
