// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_misc.c
 * @brief   File to implement miscellaneous features for a specific design
 * @author  Kineis
 * @date    Creation 2023/07/06
 * @version 1.0
 * @note
 */

/**
 * @addtogroup MCU_MISC
 * @{
 */

#include <stdbool.h>
#include <stdio.h>  /* snprintf for direct UART diagnostic in TCXO_Force_State */
#include "mcu_misc.h"
#include "main.h"
#include "mgr_log.h"
#include "usart.h"
#if defined(SMD_STDALONE)
#include "mgr_led.h"  /* MGR_LED_off() — shed LED current right before PA inrush */
#endif

/* Defines -------------------------------------------------------------------------------------- */

/** @note Define a delay macro which does not depend on systick interrupt in a way to be able to run
 * from any ISR (e.g. turn ON PA wrapper below will be called from SUBGHZ ISR during continuous
 * modulated wave)
 *
 * @attention at typical 32MHz clock and 4 instruction cycle per loop, max value is 536870 ms
 *
 * NOP             : 1 cycle
 * SUBS R3, #1     : 1 cycle
 * CMP  R3, #0     : 1 cycle
 * BNE  <ADDR>     : 1 cycle
 */
//#define DELAY_MS(time_ms) HAL_Delay(time_ms)
static uint32_t tcxo_warmup_time_ms = 2000;
extern uint32_t SystemCoreClock;
#define FOR_LOOP_CYCLE_NB 4
#define DELAY_MS(time_ms) \
{ \
	uint32_t _nb_for_loop_per_ms = SystemCoreClock / 1000 / FOR_LOOP_CYCLE_NB; \
	for (uint32_t _count = (uint32_t)((uint64_t)(time_ms) * _nb_for_loop_per_ms); \
		_count > 0 ; \
		_count--) __NOP(); \
}

/* Functions ------------------------------------------------------------------------------------ */

/** @brief Additional delay after PA enable for TCXO stabilization with GR5504 load.
 *  The PA draws significant current which can cause TCXO drift. This delay allows
 *  the power supply and TCXO to stabilize after the initial PA boot current surge.
 *  Extended from 50ms to 200ms: with TPS22904 load switch + only ~3 µF reservoir
 *  cap on PA rail, VSYS needs more time to fully recover and let chip + TCXO
 *  settle before MAC stack starts the RF transmission. */
#define MCU_PA_TCXO_STABILIZATION_MS  200

/** @brief Pre-charge delay BEFORE driving PSU_EN HIGH.
 *  Idle time to make sure VSYS is rock-solid and TPS63901 is in steady-state
 *  before the inrush event. Costs ~5 ms per TX. */
#define MCU_PA_PRECHARGE_DELAY_MS  5

void MCU_MISC_turn_on_pa()
{
	/** @attention this code may run under ISR, especially during continuous modulated wave */
#if defined(SMD_PA) || defined(SMD_STDALONE) || defined(SMD_OP)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Direct sync UART trace — runs even under ISR so we know exactly when
	 * MAC calls into PA and how far we get before any crash. */
	{
		static const char _pa_enter[] = "\r\n[PA-TRACE] turn_on_pa ENTER\r\n";
		if (hlpuart1.gState != HAL_UART_STATE_RESET)
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)_pa_enter,
				sizeof(_pa_enter) - 1, 100);
	}
	MGR_LOG_DEBUG("[PA] turn_on_pa\r\n");

#ifdef USE_SMPS_BYPASS_TX
	/* Switch SMPS to bypass mode (LDO) to reduce switching noise on TCXO.
	 * This improves TCXO stability during TX at the cost of slightly higher power consumption.
	 * The SubGHz radio may also control SMPS, but we force bypass here for PA startup. */
	HAL_PWREx_SMPS_SetMode(PWR_SMPS_BYPASS);
	DELAY_MS(1);  /* Wait for LDO to stabilize */
#endif

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_RESET);
#if !defined(SMD_STDALONE)
	/* SMD_STDALONE: PC1 is wired to TPS63901 SEL (VSYS regulator setpoint).
	 * Driving it would re-program the buck-boost output while the PA pulls
	 * its inrush — guaranteed brownout. R11 (10M pull-up to VBAT) holds SEL
	 * HIGH naturally, so leave PC1 high-Z (analog) on this board. */
	HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
#endif

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = PA_PSU_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PA_PSU_EN_GPIO_Port, &GPIO_InitStruct);

#if !defined(SMD_STDALONE)
	GPIO_InitStruct.Pin = PA_PSU_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PA_PSU_SEL_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
	DELAY_MS(10);
#endif

#if defined(SMD_STDALONE)
	/* Shed every non-essential consumer right before the PA inrush. The RGB
	 * LED sinks ~5-15 mA per color and TPS63901 is only rated 400 mA — the PA
	 * peak alone is already eating most of the budget. */
	MGR_LED_off();
#endif

	/* Pre-charge wait: give VSYS time to be fully steady before the inrush. */
	DELAY_MS(MCU_PA_PRECHARGE_DELAY_MS);
	{
		/* Single-char trace just before — tiniest UART payload so it physically
		 * leaves the chip in microseconds, telling us we reached this point. */
		static const char _pa_pre[] = "[<\r\n";
		if (hlpuart1.gState != HAL_UART_STATE_RESET)
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)_pa_pre,
				sizeof(_pa_pre) - 1, 50);
	}

	HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_SET);

	{
		/* Single-char trace right after — if this comes out, the chip survived
		 * the PA_PSU_EN HIGH transition. If absent → brownout during inrush. */
		static const char _pa_post[] = "[>\r\n";
		if (hlpuart1.gState != HAL_UART_STATE_RESET)
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)_pa_post,
				sizeof(_pa_post) - 1, 50);
	}

	/* Boot delay: TPS22904 soft-start + PA bias-up + supply recovery */
	DELAY_MS(MCU_PA_BOOTDELAY_MS);

	/* Additional delay for TCXO stabilization after PA startup.
	 * Power supply needs to stabilize before RF transmission. */
	DELAY_MS(MCU_PA_TCXO_STABILIZATION_MS);
	{
		static const char _pa_done[] = "[PA-TRACE] turn_on_pa DONE\r\n";
		if (hlpuart1.gState != HAL_UART_STATE_RESET)
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)_pa_done,
				sizeof(_pa_done) - 1, 100);
	}
#endif
}

void MCU_MISC_turn_off_pa()
{
	/** @attention this code may run under ISR, especially during continuous modulated wave */
#if defined(SMD_PA) || defined(SMD_STDALONE) || defined(SMD_OP)
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	MGR_LOG_DEBUG("[PA] turn_off_pa\r\n");
	/* Disable PA: drive PSU_EN LOW actively to ensure PA stays off.
	 * Keep pins as OUTPUT_PP (not analog) so pull-up/pull-down are effective
	 * and the pin actively drives the level. */
	HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = PA_PSU_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PA_PSU_EN_GPIO_Port, &GPIO_InitStruct);

#if !defined(SMD_STDALONE)
	GPIO_InitStruct.Pin = PA_PSU_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PA_PSU_SEL_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
#endif

#ifdef USE_SMPS_BYPASS_TX
	/* Restore SMPS to step-down mode for better power efficiency after TX.
	 * Small delay to ensure PA is fully off before switching regulator mode. */
	DELAY_MS(5);
	HAL_PWREx_SMPS_SetMode(PWR_SMPS_STEP_DOWN);
#endif
#endif
}

enum KNS_status_t MCU_MISC_getSettingsHwRf(int8_t rf_level_dBm, void* rfSettings)
{
	struct rfSettings_t *settings = (struct rfSettings_t *)rfSettings;
	int8_t rficPaLevel, rficPaLevel_min, rficPaLevel_max;


#if defined(SMD_PA) || defined(SMD_STDALONE) || defined(SMD_OP)
	rficPaLevel_min = -17; // dBm
	rficPaLevel_max = 15;  // dBm

	settings->isRficHPA = false;
	settings->externalPaGain = 36;
#elif defined(SMD_NOPA)
	rficPaLevel_min = -9; // dBm
	rficPaLevel_max = 22; // dBm

	settings->isRficHPA = true;
	settings->externalPaGain = 0;
#else
#error "Unknown board type for RF settings"
#endif
	rficPaLevel = rf_level_dBm - settings->externalPaGain;

	MGR_LOG_VERBOSE("[MCU_MISC] RFIC PA select (0:LPA, 1:HPA) = %d\r\n",
			settings->isRficHPA);
	MGR_LOG_VERBOSE("[MCU_MISC] RFIC PA level=%d dBm, external PA gain = %d dB\r\n",
			rficPaLevel,
			settings->externalPaGain);

	/* check settings */
	if ((rficPaLevel >= rficPaLevel_min) && (rficPaLevel <= rficPaLevel_max))
		settings->rficPaLevel = rficPaLevel;
	else
		return KNS_STATUS_BAD_SETTING;

	return KNS_STATUS_OK;
}

void MCU_MISC_TCXO_Force_State(bool enable)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* Get the Oscillators configuration according to the internal RCC registers */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
    RCC_OscInitStruct.OscillatorType |= RCC_OSCILLATORTYPE_HSE;

    if (enable) {
        RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
    }
    else {
        RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    }

    HAL_StatusTypeDef st = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (st != HAL_OK) {
        /* Direct synchronous UART: bypass MGR_LOG so the diagnostic survives.
         * Previously: Error_Handler() -> reset. That masked the actual cause
         * since the reset wiped any context. Logging without reset lets the
         * MAC stack continue (it has its own TCXO control internally) and
         * tells us whether this call is the culprit for the silent crashes. */
        extern UART_HandleTypeDef hlpuart1;
        static char tcxo_buf[80];
        int tcxo_n = snprintf(tcxo_buf, sizeof(tcxo_buf),
            "\r\n!!! TCXO_Force_State(%d) HAL_RCC_OscConfig=%d tick=%lu !!!\r\n",
            (int)enable, (int)st, (unsigned long)HAL_GetTick());
        if (tcxo_n > 0 && hlpuart1.gState != HAL_UART_STATE_RESET)
            HAL_UART_Transmit(&hlpuart1, (uint8_t *)tcxo_buf,
                (uint16_t)tcxo_n, 200);
        MGR_LOG_DEBUG("[MCU_MISC] TCXO_Force_State(%d) failed: HAL=%d\r\n",
            (int)enable, (int)st);
    }
    return;
}
void MCU_MISC_TCXO_set_warmup(uint32_t time_ms) {
	tcxo_warmup_time_ms = time_ms;
    return;
}
void MCU_MISC_TCXO_get_warmup(uint32_t *time_ms) {
	*time_ms = tcxo_warmup_time_ms;
	return;
}

/**
 * @}
 */
