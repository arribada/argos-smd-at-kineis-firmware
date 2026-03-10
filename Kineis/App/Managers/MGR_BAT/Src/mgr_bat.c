/**
 * @file    mgr_bat.c
 * @brief   Battery monitoring via internal VBAT/3 ADC channel
 *
 * Uses the STM32WL55 internal ADC_CHANNEL_VBAT which reads VBAT/3.
 * VBAT_EN (PB9) is driven HIGH before measurement for external circuit.
 *
 * Measurement sequence:
 *   1. Enable VBAT_EN (PB9 HIGH) + 2ms settle
 *   2. Read VREFINT (factory calibrated at 3300mV)
 *   3. Read VBAT/3 channel
 *   4. Compute: VDDA = 3300 * CAL / raw_vref
 *   5. Compute: VBAT = raw_vbat * VDDA * 3 / 4095
 *   6. Disable VBAT_EN (PB9 LOW)
 *   7. Restore ADC to SWS channel (ADC_CHANNEL_7)
 *
 * On ADC failure, returns the last known good reading (not 0).
 */

/**
 * @addtogroup MGR_BAT
 * @{
 */

#include "mgr_bat.h"
#include "main.h"
#include "adc.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"

#if defined(BSP_HAS_VBAT_ADC)

/* VREFINT calibration: use HAL defines if available, else fallback */
#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR  ((uint16_t *)0x1FFF75AAUL)
#endif
#define MGR_BAT_VREFINT_CAL_MV  3300  /* mV reference for factory calibration (STM32WL55) */

/* Last known good reading — returned on ADC failure instead of 0 */
static uint16_t last_good_vbat_mV = 0;

/* Minimum voltage threshold for TX (0 = disabled) */
static uint16_t min_tx_voltage_mV = MGR_BAT_DEFAULT_MIN_TX_MV;

void MGR_BAT_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB9 = VBAT_EN: output push-pull, start LOW (disabled) */
	GPIO_InitStruct.Pin = VBAT_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VBAT_EN_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);
}

uint16_t MGR_BAT_readVoltage_mV(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	uint32_t raw_vbat = 0;
	uint32_t raw_vref = 0;

	/* Enable external VBAT measurement circuit */
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_SET);
	/* Busy-wait ~2ms for voltage settle (SysTick-independent) */
	{ volatile uint32_t d = 2 * 8000; while (d--) __NOP(); }

	/* Read VREFINT first for accurate voltage calculation */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		goto cleanup;

	if (HAL_ADC_Start(&hadc) == HAL_OK) {
		if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
			raw_vref = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
	}

	/* Read VBAT/3 internal channel */
	sConfig.Channel = ADC_CHANNEL_VBAT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		goto cleanup;

	if (HAL_ADC_Start(&hadc) == HAL_OK) {
		if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
			raw_vbat = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
	}

cleanup:
	/* ALWAYS restore ADC to SWS channel (ADC_CHANNEL_7), even on error paths.
	 * Without this, SWS reads the wrong channel after a BAT ADC failure. */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	/* Disable external VBAT measurement circuit */
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);

	if (raw_vref == 0 || raw_vbat == 0) {
		MGR_LOG_DEBUG("[BAT] ADC read failed, using cached %umV\r\n",
			last_good_vbat_mV);
		return last_good_vbat_mV;
	}

	/* Calculate actual VDDA from VREFINT reading:
	 * VDDA = MGR_BAT_VREFINT_CAL_MV * VREFINT_CAL / raw_vref
	 */
	uint16_t vrefint_cal = *VREFINT_CAL_ADDR;
	uint32_t vdda_mV = (uint32_t)MGR_BAT_VREFINT_CAL_MV * vrefint_cal / raw_vref;

	/* VBAT = raw_vbat * VDDA / 4095 * 3 (internal /3 divider) */
	uint32_t vbat_mV = raw_vbat * vdda_mV * 3 / 4095;

	MGR_LOG_DEBUG("[BAT] raw_vref=%lu raw_vbat=%lu vdda=%lumV vbat=%lumV\r\n",
		raw_vref, raw_vbat, vdda_mV, vbat_mV);

	last_good_vbat_mV = (uint16_t)vbat_mV;
	return (uint16_t)vbat_mV;
}

uint8_t MGR_BAT_getLevel(void)
{
	uint16_t mV = MGR_BAT_readVoltage_mV();

	/* Simple linear mapping:
	 * 3600mV+ = 100%, 3000mV = 0% (LiPo typical range)
	 */
	if (mV >= 3600) return 100;
	if (mV <= 3000) return 0;
	return (uint8_t)((mV - 3000) * 100 / 600);
}

bool MGR_BAT_isTxAllowed(void)
{
	return MGR_BAT_isTxAllowedAt(MGR_BAT_readVoltage_mV());
}

bool MGR_BAT_isTxAllowedAt(uint16_t mV)
{
	if (min_tx_voltage_mV == 0)
		return true;  /* Threshold disabled */
	if (mV == 0)
		return true;  /* ADC failed, allow TX (don't block on sensor failure) */
	return (mV >= min_tx_voltage_mV);
}

uint16_t MGR_BAT_getMinTxVoltage_mV(void)
{
	return min_tx_voltage_mV;
}

void MGR_BAT_setMinTxVoltage_mV(uint16_t min_mV)
{
	min_tx_voltage_mV = min_mV;
	MGR_LOG_DEBUG("[BAT] Min TX voltage: %umV\r\n", min_mV);
}

#else /* No VBAT ADC */

void MGR_BAT_init(void) {}
uint16_t MGR_BAT_readVoltage_mV(void) { return 0; }
uint8_t MGR_BAT_getLevel(void) { return 0; }
bool MGR_BAT_isTxAllowed(void) { return true; }
bool MGR_BAT_isTxAllowedAt(uint16_t mV) { (void)mV; return true; }
uint16_t MGR_BAT_getMinTxVoltage_mV(void) { return 0; }
void MGR_BAT_setMinTxVoltage_mV(uint16_t min_mV) { (void)min_mV; }

#endif /* BSP_HAS_VBAT_ADC */

/**
 * @}
 */
