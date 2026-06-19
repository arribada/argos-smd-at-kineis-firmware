/**
 * @file    mgr_led.h
 * @brief   LED RGB driver for SMD_STDALONE board
 *
 * Active LOW: GPIO_PIN_RESET = LED on, GPIO_PIN_SET = LED off
 * Pins: PA1 = RED, PB4 = GREEN, PB5 = BLUE
 */

#ifndef MGR_LED_H
#define MGR_LED_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	MGR_LED_OFF = 0,
	MGR_LED_WHITE,
	MGR_LED_RED,
	MGR_LED_GREEN,
	MGR_LED_BLUE,
	MGR_LED_VIOLET,
	MGR_LED_CYAN,
	MGR_LED_YELLOW,
} MGR_LED_Color_t;

typedef enum {
	MGR_LED_MODE_OFF  = 0,  /**< LED always off */
	MGR_LED_MODE_ON   = 1,  /**< LED enabled */
	MGR_LED_MODE_24H  = 2,  /**< LED enabled for 24h then auto-off */
} MGR_LED_Mode_t;

/** @brief Initialize LED GPIO pins */
void MGR_LED_init(void);

/** @brief Set solid color
 *
 *  @note On SMD_STDALONE the RGB LED has a single anode current-limit
 *  resistor. Driving multiple cathodes simultaneously only lights RED
 *  (lowest Vf) — composite colours (WHITE/VIOLET/CYAN/YELLOW) appear as RED
 *  to the eye. Use single-colour codes (R/G/B) for reliable display.
 */
void MGR_LED_set(MGR_LED_Color_t color);

/** @brief Start non-blocking blink sequence
 *  @param color Color to blink
 *  @param count Number of blinks (0 = infinite)
 *  @param on_ms On time in ms
 *  @param off_ms Off time in ms
 */
void MGR_LED_blink(MGR_LED_Color_t color, uint8_t count, uint16_t on_ms, uint16_t off_ms);

/** @brief Forced variants — bypass the `led_mode = OFF` gate.
 *
 * Use these for events the operator MUST see regardless of the deployment
 * LED policy:
 *   - gesture FSM feedback (magnet detection, mode switch confirmation)
 *   - AT command responses (AT+DIAG cycle, commissioning indicators)
 *   - boot-time wake blink
 *
 * The plain MGR_LED_set / MGR_LED_blink remain gated and silence
 * themselves when AT+LED=0 has muted the deployment status indicators
 * (TX-in-flight, SWS surface/UW transitions, low-battery hint). */
void MGR_LED_setForced(MGR_LED_Color_t color);
void MGR_LED_blinkForced(MGR_LED_Color_t color, uint8_t count,
                         uint16_t on_ms, uint16_t off_ms);

/** @brief Turn off LED and cancel any blink sequence */
void MGR_LED_off(void);

/** @brief Call from main loop to update blink timing */
void MGR_LED_task(void);

/** @brief Soft-PWM tick for composite colours (WHITE/VIOLET/CYAN/YELLOW).
 *
 * MUST be called every 1 ms — wire it into SysTick_Handler() after
 * HAL_IncTick(). Single-channel colours (R/G/B/OFF) are direct-drive and
 * do not need this tick. Safe in ISR context (single BSRR writes only).
 *
 * Without this, WHITE/VIOLET/CYAN/YELLOW silently degrade to RED on a
 * common-anode RGB with a single anode current-limit resistor.
 */
void MGR_LED_softTick(void);

/** @brief Check if blink sequence has finished */
bool MGR_LED_isBlinkDone(void);

/** @brief Get current LED mode */
MGR_LED_Mode_t MGR_LED_getMode(void);

/** @brief Set LED mode */
void MGR_LED_setMode(MGR_LED_Mode_t mode);

/** @brief Boot color test: cycles R, G, B, WHITE individually (blocking, 500ms each)
 *  Logs GPIO register state for each step. Use to verify pin-to-color mapping.
 */
void MGR_LED_bootTest(void);

#endif /* MGR_LED_H */
