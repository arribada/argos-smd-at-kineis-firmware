/**
 * @file    mgr_led.c
 * @brief   LED RGB driver implementation
 *
 * Active LOW: GPIO_PIN_RESET = LED on, GPIO_PIN_SET = LED off
 */

#include "mgr_led.h"
#include "main.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"

#if defined(BSP_HAS_LED_RGB) && BSP_HAS_LED_RGB

/* ---- Private state ---- */

static MGR_LED_Mode_t led_mode = MGR_LED_MODE_ON;
static uint32_t mode_start_tick = 0;

/* Blink state */
static bool blinking = false;
static MGR_LED_Color_t blink_color = MGR_LED_OFF;
static uint8_t blink_total = 0;
static uint8_t blink_count = 0;
static uint16_t blink_on_ms = 0;
static uint16_t blink_off_ms = 0;
static uint32_t blink_tick = 0;
static bool blink_phase_on = false;

/* ---- Helpers ---- */

/** @brief Force LED GPIO pins back to push-pull output mode.
 *  Needed in case LPM or other peripheral reconfigured them.
 */
static void led_reinit_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = LED_RED_Pin;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_BLUE_Pin;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);
}

static void led_set_raw(bool r, bool g, bool b)
{
	/* Active LOW: RESET = on, SET = off */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   r ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  b ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void led_apply_color(MGR_LED_Color_t color)
{
	switch (color) {
	case MGR_LED_OFF:     led_set_raw(false, false, false); break;
	case MGR_LED_WHITE:   led_set_raw(true,  true,  true);  break;
	case MGR_LED_RED:     led_set_raw(true,  false, false); break;
	case MGR_LED_GREEN:   led_set_raw(false, true,  false); break;
	case MGR_LED_BLUE:    led_set_raw(false, false, true);  break;
	case MGR_LED_VIOLET:  led_set_raw(true,  false, true);  break;
	case MGR_LED_CYAN:    led_set_raw(false, true,  true);  break;
	case MGR_LED_YELLOW:  led_set_raw(true,  true,  false); break;
	default:              led_set_raw(false, false, false); break;
	}
}

static bool is_mode_active(void)
{
	if (led_mode == MGR_LED_MODE_OFF)
		return false;
	if (led_mode == MGR_LED_MODE_24H) {
		if ((HAL_GetTick() - mode_start_tick) > 86400000UL) {
			led_mode = MGR_LED_MODE_OFF;
			return false;
		}
	}
	return true;
}

/* ---- Public API ---- */

void MGR_LED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* All LEDs off at init (HIGH = off for active LOW) */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  GPIO_PIN_SET);

	/* PA1 = LED_RED */
	GPIO_InitStruct.Pin = LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	/* PB4 = LED_GREEN, PB5 = LED_BLUE */
	GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_BLUE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	mode_start_tick = HAL_GetTick();
}

void MGR_LED_set(MGR_LED_Color_t color)
{
	blinking = false;
	if (is_mode_active()) {
		led_reinit_gpio();
		led_apply_color(color);
	}
}

void MGR_LED_blink(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
	if (!is_mode_active()) {
		blinking = false;
		return;
	}

	/* Re-init GPIOs to ensure they are configured as outputs */
	led_reinit_gpio();

	MGR_LOG_DEBUG("[LED] blink color=%u count=%u on=%u off=%u\r\n",
		(unsigned)color, (unsigned)count, (unsigned)on_ms, (unsigned)off_ms);

	blink_color = color;
	blink_total = count;
	blink_count = 0;
	blink_on_ms = on_ms;
	blink_off_ms = off_ms;
	blink_tick = HAL_GetTick();
	blink_phase_on = true;
	blinking = true;

	led_apply_color(color);
}

void MGR_LED_off(void)
{
	blinking = false;
	led_apply_color(MGR_LED_OFF);
}

void MGR_LED_task(void)
{
	if (!blinking)
		return;

	if (!is_mode_active()) {
		MGR_LED_off();
		return;
	}

	uint32_t elapsed = HAL_GetTick() - blink_tick;

	if (blink_phase_on) {
		if (elapsed >= blink_on_ms) {
			/* Turn off */
			led_apply_color(MGR_LED_OFF);
			blink_phase_on = false;
			blink_tick = HAL_GetTick();
			blink_count++;

			/* Check if done */
			if (blink_total > 0 && blink_count >= blink_total) {
				blinking = false;
				return;
			}
		}
	} else {
		if (elapsed >= blink_off_ms) {
			/* Turn on */
			led_apply_color(blink_color);
			blink_phase_on = true;
			blink_tick = HAL_GetTick();
		}
	}
}

bool MGR_LED_isBlinkDone(void)
{
	return !blinking;
}

MGR_LED_Mode_t MGR_LED_getMode(void)
{
	return led_mode;
}

void MGR_LED_setMode(MGR_LED_Mode_t mode)
{
	led_mode = mode;
	mode_start_tick = HAL_GetTick();
	if (mode == MGR_LED_MODE_OFF) {
		MGR_LED_off();
	}
}

void MGR_LED_bootTest(void)
{
	led_reinit_gpio();

	/* Log GPIO register state for debugging */
	uint32_t a_moder = GPIOA->MODER;
	uint32_t b_moder = GPIOB->MODER;
	MGR_LOG_DEBUG("[LED] GPIO MODER: A=0x%08lX B=0x%08lX\r\n",
		(unsigned long)a_moder, (unsigned long)b_moder);
	MGR_LOG_DEBUG("[LED] Pins: RED=PA%u GREEN=PB%u BLUE=PB%u\r\n",
		(unsigned)1, (unsigned)4, (unsigned)5);

	/* PA1 MODER bits [3:2]: should be 01 (output) */
	MGR_LOG_DEBUG("[LED] PA1  MODER[3:2]=%lu (expect 1=output)\r\n",
		(unsigned long)((a_moder >> 2) & 0x3));
	/* PB4 MODER bits [9:8]: should be 01 (output) */
	MGR_LOG_DEBUG("[LED] PB4  MODER[9:8]=%lu (expect 1=output)\r\n",
		(unsigned long)((b_moder >> 8) & 0x3));
	/* PB5 MODER bits [11:10]: should be 01 (output) */
	MGR_LOG_DEBUG("[LED] PB5 MODER[11:10]=%lu (expect 1=output)\r\n",
		(unsigned long)((b_moder >> 10) & 0x3));

	/* Step 1: PA1 only (should be RED) */
	led_set_raw(false, false, false);  /* all off first */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);  /* PA1 LOW = on */
	MGR_LOG_DEBUG("[LED] TEST: PA1=LOW only -> should be RED\r\n");
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);    /* off */

	/* Step 2: PB4 only (should be GREEN) */
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);  /* PB4 LOW = on */
	MGR_LOG_DEBUG("[LED] TEST: PB4=LOW only -> should be GREEN\r\n");
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);    /* off */

	/* Step 3: PB5 only (should be BLUE) */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);  /* PB5 LOW = on */
	MGR_LOG_DEBUG("[LED] TEST: PB5=LOW only -> should be BLUE\r\n");
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);    /* off */

	/* Step 4: All 3 on (should be WHITE) */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  GPIO_PIN_RESET);
	MGR_LOG_DEBUG("[LED] TEST: ALL=LOW -> should be WHITE\r\n");
	MGR_LOG_DEBUG("[LED] ODR: A=0x%04lX B=0x%04lX\r\n",
		(unsigned long)(GPIOA->ODR & 0xFFFF),
		(unsigned long)(GPIOB->ODR & 0xFFFF));
	HAL_Delay(500);

	/* All off */
	led_set_raw(false, false, false);
	MGR_LOG_DEBUG("[LED] TEST: done\r\n");
}

#else /* No LED RGB */

void MGR_LED_init(void) {}
void MGR_LED_set(MGR_LED_Color_t color) { (void)color; }
void MGR_LED_blink(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms, uint16_t off_ms)
{ (void)color; (void)count; (void)on_ms; (void)off_ms; }
void MGR_LED_off(void) {}
void MGR_LED_task(void) {}
bool MGR_LED_isBlinkDone(void) { return true; }
MGR_LED_Mode_t MGR_LED_getMode(void) { return MGR_LED_MODE_OFF; }
void MGR_LED_setMode(MGR_LED_Mode_t mode) { (void)mode; }
void MGR_LED_bootTest(void) {}

#endif /* BSP_HAS_LED_RGB */
