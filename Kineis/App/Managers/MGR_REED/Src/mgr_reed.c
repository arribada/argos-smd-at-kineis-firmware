/**
 * @file    mgr_reed.c
 * @brief   Reed switch (EXTI interrupt) and power latch driver for SMD_STDALONE
 *
 * Reed: PB6, active HIGH (HIGH = magnet present), EXTI on both edges.
 * Power latch: PB7, HIGH = keep board powered.
 *
 * The EXTI ISR records timestamps for rising/falling edges.
 * The application reads events via MGR_REED_getEvent() and gets
 * hold duration via MGR_REED_getLastHoldDuration_ms() to decide
 * which action to trigger based on how long the magnet was held.
 *
 * Usage in UW_DOPPLER:
 *   - Magnet ON: starts hold timer
 *   - Magnet OFF (<10s): no action (short contact)
 *   - Magnet held >10s: triggers shutdown sequence (blink + NVM save + power off)
 */

/**
 * @addtogroup MGR_REED
 * @{
 */

#include "mgr_reed.h"
#include "main.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"

#if defined(BSP_HAS_REED_SWITCH) && BSP_HAS_REED_SWITCH

/* ---- Private state ---- */

#define REED_DEBOUNCE_MS  50  /**< Ignore edges within this interval */

/* Event ring buffer: ISR pushes, getEvent() pops.
 * Size must be power-of-2 for mask trick. 4 entries is enough
 * (only 2 event types, 50ms debounce = max ~20 events/s). */
#define EVT_BUF_SIZE  4
#define EVT_BUF_MASK  (EVT_BUF_SIZE - 1)

static volatile MGR_REED_Event_t evt_buf[EVT_BUF_SIZE];
static volatile uint8_t evt_head = 0;  /**< Next write index (ISR only) */
static volatile uint8_t evt_tail = 0;  /**< Next read index (main only) */

static volatile uint32_t rising_tick = 0;       /**< Tick when magnet was detected */
static volatile uint32_t last_hold_ms = 0;      /**< Duration of last completed hold */
static volatile uint32_t last_edge_tick = 0;    /**< Tick of last edge (for debounce) */

/* ---- Public API ---- */

void MGR_REED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB7 = PWR_LATCH: drive HIGH immediately to keep board powered */
	HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = PWR_LATCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PWR_LATCH_GPIO_Port, &GPIO_InitStruct);

	/* PB6 = REED_MCU: EXTI on both edges, pull-down (active HIGH) */
	GPIO_InitStruct.Pin = REED_MCU_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(REED_MCU_GPIO_Port, &GPIO_InitStruct);

	/* Enable EXTI9_5 interrupt (covers EXTI lines 5-9, PB6 = line 6) */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

bool MGR_REED_isMagnetPresent(void)
{
	return (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin) == GPIO_PIN_SET);
}

MGR_REED_Event_t MGR_REED_getEvent(void)
{
	/* Lock-free SPSC: tail only modified here (main context) */
	if (evt_tail == evt_head)
		return MGR_REED_EVT_NONE;

	MGR_REED_Event_t evt = evt_buf[evt_tail & EVT_BUF_MASK];
	evt_tail++;  /* Atomic 8-bit write on Cortex-M */
	return evt;
}

uint32_t MGR_REED_getLastHoldDuration_ms(void)
{
	return last_hold_ms;
}

void MGR_REED_latchPower(void)
{
	HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_SET);
}

void MGR_REED_releasePower(void)
{
	HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_RESET);
}

/* ---- EXTI ISR ---- */

/**
 * @brief EXTI lines 5-9 interrupt handler
 * @note  Overrides weak default from startup_stm32wl55xx_cm4.s
 */
void EXTI9_5_IRQHandler(void)
{
	/* Service all EXTI lines 5-9 to prevent lockup if other sources trigger */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

/**
 * @brief HAL GPIO EXTI callback - called from HAL_GPIO_EXTI_IRQHandler
 * @note  Overrides weak default from HAL driver
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin != REED_MCU_Pin)
		return;

	uint32_t now = HAL_GetTick();

	/* Debounce: ignore edges too close together */
	if ((now - last_edge_tick) < REED_DEBOUNCE_MS) {
		return;
	}
	last_edge_tick = now;

	MGR_REED_Event_t evt;

	if (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin) == GPIO_PIN_SET) {
		/* Rising edge: magnet detected */
		rising_tick = now;
		evt = MGR_REED_EVT_MAGNET_ON;
	} else {
		/* Falling edge: magnet removed */
		if (rising_tick > 0)
			last_hold_ms = now - rising_tick;
		else
			last_hold_ms = 0;  /* No valid rising edge recorded */
		rising_tick = 0;
		evt = MGR_REED_EVT_MAGNET_OFF;
	}

	/* Push into ring buffer (drop oldest if full) */
	if ((uint8_t)(evt_head - evt_tail) < EVT_BUF_SIZE) {
		evt_buf[evt_head & EVT_BUF_MASK] = evt;
		evt_head++;
	}
}

#else /* No reed switch */

void MGR_REED_init(void) {}
bool MGR_REED_isMagnetPresent(void) { return false; }
MGR_REED_Event_t MGR_REED_getEvent(void) { return MGR_REED_EVT_NONE; }
uint32_t MGR_REED_getLastHoldDuration_ms(void) { return 0; }
void MGR_REED_latchPower(void) {}
void MGR_REED_releasePower(void) {}

#endif /* BSP_HAS_REED_SWITCH */

/**
 * @}
 */
