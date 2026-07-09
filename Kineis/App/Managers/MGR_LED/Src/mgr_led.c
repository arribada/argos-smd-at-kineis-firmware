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

static MGR_LED_Mode_t led_mode = MGR_LED_MODE_24H; /* visible for commissioning day, then dark */
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
/* True when the in-flight blink was started via a *Forced variant. The forced
 * gesture/AT feedback (WAKE green, CONFIG blue, SHUTDOWN red, WAIT_CONFIRM)
 * must stay visible even when AT+LED=0 / the 24H window has set led_mode=OFF.
 * blink_impl bypasses the mode gate at START, but MGR_LED_task() re-checks
 * is_mode_active() every loop and used to silence the forced blink on the very
 * next tick (symptom: the DETECT WHITE solid shows but the SWITCH/SHUTDOWN
 * blinks go dark). Storing the bypass here lets the task honor it too. */
static bool blink_bypass = false;

/* Soft-PWM state for composite colours (WHITE/VIOLET/CYAN/YELLOW).
 *
 * HW context: SMD_STDALONE has a common-anode RGB LED with a SINGLE anode
 * current-limit resistor. When two cathodes are pulled low, the supply node
 * clamps to Vbus - Vf_red (~1.3 V on a 3V3 rail), which is too low to drive
 * GREEN/BLUE (Vf ≈ 3.2 V). The result: a "WHITE" command shows as RED only.
 *
 * Workaround: time-multiplex the channels at 1 ms per slot via SysTick. At
 * 333 Hz total / 111 Hz per channel the eye integrates as continuous colour.
 * The PWM runs in SysTick ISR (negligible cost: a load, compare, and one
 * GPIO BSRR write per ms).
 *
 * Solid R/G/B keeps the direct-drive fast path: no PWM needed when only one
 * cathode is active. */
static volatile MGR_LED_Color_t soft_color = MGR_LED_OFF;
static volatile uint8_t          soft_slot  = 0;

/* ---- Helpers ---- */

/** @brief (Re)configure LED GPIO pins as push-pull outputs.
 *  HAL_GPIO_Init is idempotent and cheap; calling it unconditionally on
 *  every set/blink avoids stale-flag bugs after LPM transitions that may
 *  have reconfigured the pins to analog.
 */
static void led_reinit_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = LED_RED_Pin;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_BLUE_Pin;
	HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);
}

static void led_set_raw(bool r, bool g, bool b)
{
	/* Direct-pin diag (2026-06-05) confirmed PA1=RED, PB4=GREEN, PB5=BLUE
	 * match the BSP header. No swap needed. */
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

/** @brief Return true if `color` is a composite that requires soft-PWM
 *  (multiple cathodes; the HW can't drive them simultaneously). */
static bool color_needs_soft_pwm(MGR_LED_Color_t color)
{
	return (color == MGR_LED_WHITE)  ||
	       (color == MGR_LED_VIOLET) ||
	       (color == MGR_LED_CYAN)   ||
	       (color == MGR_LED_YELLOW);
}

static void led_apply_color(MGR_LED_Color_t color)
{
	/* Composite colours: enable soft-PWM (driven by MGR_LED_softTick()
	 * from SysTick at 1 ms cadence). Direct-drive only the FIRST slot
	 * here; the ISR rotates through the remaining slots. */
	if (color_needs_soft_pwm(color)) {
		soft_color = color;
		soft_slot  = 0;
		/* Slot 0 of every composite includes RED. Drive it now so
		 * the user sees light immediately (before the next ISR tick). */
		switch (color) {
		case MGR_LED_WHITE:  led_set_raw(true, false, false); break;
		case MGR_LED_VIOLET: led_set_raw(true, false, false); break;
		case MGR_LED_CYAN:   led_set_raw(false, true, false); break;
		case MGR_LED_YELLOW: led_set_raw(true, false, false); break;
		default: break;
		}
		return;
	}

	/* Solid single-channel colour or OFF: stop the soft-PWM rotator and
	 * drive the GPIOs directly. */
	soft_color = MGR_LED_OFF;
	switch (color) {
	case MGR_LED_OFF:    led_set_raw(false, false, false); break;
	case MGR_LED_RED:    led_set_raw(true,  false, false); break;
	case MGR_LED_GREEN:  led_set_raw(false, true,  false); break;
	case MGR_LED_BLUE:   led_set_raw(false, false, true);  break;
	default:             led_set_raw(false, false, false); break;
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
	led_reinit_gpio();

	mode_start_tick = HAL_GetTick();
}

/* Internal set: the bypass flag skips is_mode_active() check. Used by the
 * "Forced" public variant for gesture FSM + AT command feedback which must
 * remain visible even when AT+LED=0 has silenced the deployment status
 * LEDs (TX-in-flight, surface/UW transitions, low-battery indicator). */
static void set_impl(MGR_LED_Color_t color, bool bypass)
{
	blinking = false;
	if (bypass || is_mode_active()) {
		led_reinit_gpio();
		led_apply_color(color);
	}
}

void MGR_LED_set(MGR_LED_Color_t color)
{
	set_impl(color, false);
}

void MGR_LED_setForced(MGR_LED_Color_t color)
{
	set_impl(color, true);
}

static void blink_impl(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms,
                       uint16_t off_ms, bool bypass)
{
	if (!bypass && !is_mode_active()) {
		blinking = false;
		return;
	}

	/* Remember whether this blink is forced so MGR_LED_task() keeps it
	 * alive even when led_mode is OFF/expired. Set before the idempotent
	 * early-return so a re-issue updates the bypass semantics too. */
	blink_bypass = bypass;

	/* Idempotent: a blink already in flight with the SAME parameters
	 * is left alone. Without this, callers that re-issue the same
	 * MGR_LED_blink() each main-loop iteration (e.g. the CONFIG-mode
	 * indicator) would reset the blink phase every iteration → LED
	 * stuck on. Note: an externally interrupted blink (blinking==false)
	 * IS restarted, which is exactly what we want after e.g. AT+DIAG
	 * turns the LED off mid-cycle. */
	if (blinking &&
	    blink_color == color &&
	    blink_total == count &&
	    blink_on_ms == on_ms &&
	    blink_off_ms == off_ms)
		return;

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

void MGR_LED_blink(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms,
                   uint16_t off_ms)
{
	blink_impl(color, count, on_ms, off_ms, false);
}

void MGR_LED_blinkForced(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms,
                         uint16_t off_ms)
{
	blink_impl(color, count, on_ms, off_ms, true);
}

void MGR_LED_off(void)
{
	blinking = false;
	led_apply_color(MGR_LED_OFF);
}

/* ---- Soft-PWM weighted schedules ----
 *
 * Each composite colour gets a SCHEDULE: a circular array of (R, G, B)
 * slot states. The SysTick ISR advances one slot per ms; the slot array
 * length therefore controls the perceived chromatic balance.
 *
 * Why weighted instead of round-robin? Two compounding asymmetries:
 *   1. HW: the common-anode resistor drops less voltage across RED (Vf
 *      ~2.0 V) than GREEN/BLUE (Vf ~3.0 V), so RED gets ~4× the current
 *      and is much brighter at equal duty.
 *   2. Eye: relative photopic sensitivity is ~30/59/11 for R/G/B (CIE Y).
 *      Green dominates perception even at equal radiometric power.
 *
 * Net: equal R/G/B duty looks GREEN-RED-tinted, never neutral white. The
 * tables below were calibrated empirically on the SMD_STDALONE board.
 *
 * Slot encoding: bit0=R, bit1=G, bit2=B. 0x00 = OFF slot (useful to dim
 * a colour without changing its hue).
 *
 * Total slot count = full cycle in ms. Keep ≤16 (62.5 Hz) for flicker-free
 * perception; smaller = higher refresh, less duty granularity.
 */
#define SLOT_R 0x01u
#define SLOT_G 0x02u
#define SLOT_B 0x04u

/* WHITE: pull GREEN down hard, bias toward R+B to counter green dominance.
 * 8 slots, R:3 G:1 B:3 OFF:1 → ~125 Hz. Tunable. */
static const uint8_t soft_white_sched[] = {
	SLOT_R, SLOT_R, SLOT_R,
	SLOT_G,
	SLOT_B, SLOT_B, SLOT_B,
	0u,  /* dimming slot */
};

/* VIOLET: equal R and B, no green at all. 4 slots → 250 Hz. */
static const uint8_t soft_violet_sched[] = {
	SLOT_R, SLOT_R,
	SLOT_B, SLOT_B,
};

/* CYAN: heavy BLUE, light GREEN — without this it looks lime/teal because
 * of the green sensitivity bias. 4 slots → 250 Hz. */
static const uint8_t soft_cyan_sched[] = {
	SLOT_G,
	SLOT_B, SLOT_B, SLOT_B,
};

/* YELLOW: more GREEN than RED to compensate for the RED brightness
 * advantage (otherwise it looks orange / pale-white). One OFF slot
 * tames the overall brightness so the warning isn't blinding. 5 slots. */
static const uint8_t soft_yellow_sched[] = {
	SLOT_R,
	SLOT_G, SLOT_G,
	0u,
	SLOT_R | 0u,  /* hold pattern symmetry */
};

/* Lookup helper: returns the schedule table + length for a composite. */
static bool soft_schedule_get(MGR_LED_Color_t c,
                              const uint8_t **out_sched, uint8_t *out_len)
{
	switch (c) {
	case MGR_LED_WHITE:
		*out_sched = soft_white_sched;
		*out_len   = (uint8_t)(sizeof(soft_white_sched) /
		                       sizeof(soft_white_sched[0]));
		return true;
	case MGR_LED_VIOLET:
		*out_sched = soft_violet_sched;
		*out_len   = (uint8_t)(sizeof(soft_violet_sched) /
		                       sizeof(soft_violet_sched[0]));
		return true;
	case MGR_LED_CYAN:
		*out_sched = soft_cyan_sched;
		*out_len   = (uint8_t)(sizeof(soft_cyan_sched) /
		                       sizeof(soft_cyan_sched[0]));
		return true;
	case MGR_LED_YELLOW:
		*out_sched = soft_yellow_sched;
		*out_len   = (uint8_t)(sizeof(soft_yellow_sched) /
		                       sizeof(soft_yellow_sched[0]));
		return true;
	default:
		return false;
	}
}

/* Called from SysTick ISR (1 ms cadence). Advances the soft-PWM slot for
 * composite colours. Safe in ISR: only single-cycle GPIO BSRR writes via
 * HAL_GPIO_WritePin → GPIOx->BSRR = pin. */
void MGR_LED_softTick(void)
{
	const MGR_LED_Color_t c = soft_color;
	if (c == MGR_LED_OFF)
		return;

	const uint8_t *sched;
	uint8_t len;
	if (!soft_schedule_get(c, &sched, &len)) {
		/* Invalid composite colour: drop back to OFF. */
		soft_color = MGR_LED_OFF;
		return;
	}

	uint8_t slot = (uint8_t)((soft_slot + 1u) % len);
	soft_slot = slot;

	const uint8_t mask = sched[slot];
	led_set_raw((mask & SLOT_R) != 0u,
	            (mask & SLOT_G) != 0u,
	            (mask & SLOT_B) != 0u);
}

void MGR_LED_task(void)
{
	if (!blinking)
		return;

	/* Forced blinks (gesture / AT feedback) bypass the mode gate — they must
	 * remain visible even when AT+LED=0 silenced the deployment status LEDs.
	 * Only a NON-forced blink is silenced when led_mode is OFF/24H-expired. */
	if (!blink_bypass && !is_mode_active()) {
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

/** @brief Quick HW sanity check: cycle RED -> GREEN -> BLUE then OFF.
 *  Proves the LED + GPIO chain works independently of the app state machine
 *  and the NVM-loaded led_mode setting. Total ~1.2s, blocking.
 *  Physical wiring on SMD_STDALONE confirmed 2026-06-05:
 *    PA1 -> RED, PB4 -> GREEN, PB5 -> BLUE (matches BSP header).
 */
void MGR_LED_bootTest(void)
{
	led_reinit_gpio();
	led_set_raw(false, false, false);

	led_apply_color(MGR_LED_RED);    HAL_Delay(400);
	led_apply_color(MGR_LED_GREEN);  HAL_Delay(400);
	led_apply_color(MGR_LED_BLUE);   HAL_Delay(400);
	led_apply_color(MGR_LED_OFF);
}

#else /* No LED RGB */

void MGR_LED_init(void) {}
void MGR_LED_set(MGR_LED_Color_t color) { (void)color; }
void MGR_LED_setForced(MGR_LED_Color_t color) { (void)color; }
void MGR_LED_blink(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms, uint16_t off_ms)
{ (void)color; (void)count; (void)on_ms; (void)off_ms; }
void MGR_LED_blinkForced(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms, uint16_t off_ms)
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
