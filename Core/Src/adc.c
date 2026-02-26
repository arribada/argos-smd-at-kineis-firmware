/**
 * @file    adc.c
 * @brief   ADC peripheral driver for SWS analog measurement
 *
 * Configures ADC on PA11 (ADC_IN7) for salt water switch conductivity reading.
 * 12-bit resolution, software trigger, single conversion mode.
 */

#include "adc.h"

ADC_HandleTypeDef hadc;

void MX_ADC_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc.Instance = ADC;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.NbrOfConversion = 1;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;

	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		return;
	}

	/* Run ADC calibration for better accuracy */
	HAL_ADCEx_Calibration_Start(&hadc);

	/* Configure channel 7 (PA11) */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		return;
	}
}

void MX_ADC_DeInit(void)
{
	HAL_ADC_DeInit(&hadc);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (adcHandle->Instance == ADC) {
		__HAL_RCC_ADC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* PA11 = ADC_IN7 (SWS analog input) */
		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
	if (adcHandle->Instance == ADC) {
		__HAL_RCC_ADC_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
	}
}

uint32_t ADC_ReadValue(void)
{
	uint32_t raw_value = 0;

	if (HAL_ADC_Start(&hadc) != HAL_OK) {
		return 0;
	}

	if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK) {
		HAL_ADC_Stop(&hadc);
		return 0;
	}

	raw_value = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	return raw_value;
}
