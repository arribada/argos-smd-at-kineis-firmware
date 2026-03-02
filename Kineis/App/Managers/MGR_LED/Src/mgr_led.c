/**
 * @file    mgr_led.c
 * @brief   LED RGB driver implementation for SMD_STDALONE board
 *
 * Active LOW: GPIO_PIN_RESET = LED on, GPIO_PIN_SET = LED off.
 * Pins: PA1=RED, PB4=GREEN, PB5=BLUE.
 *
 * Supports solid color display and non-blocking blink sequences.
 * MGR_LED_task() must be called from the main loop to update blink timing.
 *
 * LED modes:
 *   - OFF (0): LED always off
 *   - ON  (1): LED enabled (state-dependent colors in UW_DOPPLER)
 *   - 24H (2): LED enabled for 24 hours then auto-off
 */

/**
 * @addtogroup MGR_LED
 * @{
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

	/* Init each LED individually (pins may be on different ports) */
	GPIO_InitStruct.Pin = LED_RED_Pin;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_BLUE_Pin;
	HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);
}

static void led_set_raw(bool r, bool g, bool b)
{
#if defined(BSP_LED_ACTIVE_HIGH) && BSP_LED_ACTIVE_HIGH
	/* Active HIGH (common cathode): SET = on, RESET = off */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   r ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  b ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
	/* Active LOW (common anode): RESET = on, SET = off */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   r ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  b ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
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

	/* All LEDs off at init */
#if defined(BSP_LED_ACTIVE_HIGH) && BSP_LED_ACTIVE_HIGH
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  GPIO_PIN_RESET);
#else
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,   LED_RED_Pin,   GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,  LED_BLUE_Pin,  GPIO_PIN_SET);
#endif

	/* Init each LED individually (pins may be on different ports) */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = LED_RED_Pin;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_BLUE_Pin;
	HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

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
	led_set_raw(false, false, false);

	/* Cycle each color individually for pin-to-color identification */
	MGR_LOG_DEBUG("[LED] Boot test: RED (PA1)\r\n");
	led_set_raw(true, false, false);
	HAL_Delay(400);

	MGR_LOG_DEBUG("[LED] Boot test: GREEN (PB4)\r\n");
	led_set_raw(false, true, false);
	HAL_Delay(400);

	MGR_LOG_DEBUG("[LED] Boot test: BLUE (PB5)\r\n");
	led_set_raw(false, false, true);
	HAL_Delay(400);

	MGR_LOG_DEBUG("[LED] Boot test: WHITE (all)\r\n");
	led_set_raw(true, true, true);
	HAL_Delay(400);

	led_set_raw(false, false, false);
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

/**
 * @}
 */
