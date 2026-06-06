/**
 * @file    mgr_reed.c
 * @brief   Reed switch (EXTI) + power latch driver for SMD_STDALONE.
 *
 * Hardware: PB6 = reed, active HIGH (magnet → HIGH via reed-to-VBAT path with
 * internal pull-down on PB6). PB7 = PWR_LATCH, HIGH = board powered.
 *
 * EXTI on both edges. ISR runs a single-producer / single-consumer ring
 * buffer (this file is the only writer; MGR_REED_getEvent is the only reader).
 *
 * Hardening choices made for sealed-capsule deployment:
 *  - Buffer 8 entries (was 4): copes with chatter/double-tap bursts; debounce
 *    is 50 ms so absolute max producer rate is ~20 ev/s, but a flurry of
 *    edges right after the debounce window can still pile 4-6 in <200 ms.
 *  - Reader pops by reading into a LOCAL first, then incrementing tail —
 *    closes the torn-read window if a preempting IRQ overwrites the same
 *    slot between the buf read and the tail increment.
 *  - EXTI9_5 handler dispatches ONLY pin 6 (other lines are not configured
 *    for EXTI in this build; if another peripheral ever does, give it its
 *    own dispatch, do not let it pile through this callback).
 *  - All ISR-shared state is `volatile` and accessed as 32-bit single-word
 *    loads/stores (atomic on Cortex-M4).
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

#define REED_DEBOUNCE_MS  50u  /**< Ignore edges within this interval */

/* Event ring buffer. Power-of-two for mask trick. 8 entries: 2 x debounce
 * window worth of edges, gives headroom under bounce. */
#define EVT_BUF_SIZE  8u
#define EVT_BUF_MASK  (EVT_BUF_SIZE - 1u)

_Static_assert((EVT_BUF_SIZE & EVT_BUF_MASK) == 0u,
               "EVT_BUF_SIZE must be a power of two");

static volatile MGR_REED_Event_t evt_buf[EVT_BUF_SIZE];
static volatile uint8_t evt_head = 0;  /**< Next write index (ISR only) */
static volatile uint8_t evt_tail = 0;  /**< Next read index (main only) */

static volatile uint32_t rising_tick = 0;     /**< Tick of last rising edge */
static volatile uint32_t last_hold_ms = 0;    /**< Duration of last completed hold */
static volatile uint32_t last_edge_tick = 0;  /**< Tick of last accepted edge */

/* ---- Public API ---- */

void MGR_REED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB7 = PWR_LATCH: drive HIGH BEFORE configuring as output so the pin
	 * never transitions LOW→HIGH (output starts at the level we just set). */
	HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = PWR_LATCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PWR_LATCH_GPIO_Port, &GPIO_InitStruct);

	/* PB6 = REED_MCU: EXTI on both edges, pull-down (active HIGH). */
	GPIO_InitStruct.Pin = REED_MCU_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(REED_MCU_GPIO_Port, &GPIO_InitStruct);

	/* Drop any pending edge that fired while the GPIO was being configured.
	 * Without this, the very first re-init after a wake can deliver a phantom
	 * event for an edge the application never saw. */
	__HAL_GPIO_EXTI_CLEAR_IT(REED_MCU_Pin);

	/* If the magnet is already present at init (boot with magnet held),
	 * synthesize a MAGNET_ON event so the FSM doesn't sit IDLE waiting for
	 * an edge that will never fire. */
	if (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin) == GPIO_PIN_SET) {
		rising_tick = HAL_GetTick();
		last_edge_tick = rising_tick;
		evt_buf[evt_head & EVT_BUF_MASK] = MGR_REED_EVT_MAGNET_ON;
		evt_head++;
	}

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

bool MGR_REED_isMagnetPresent(void)
{
	return (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin) == GPIO_PIN_SET);
}

MGR_REED_Event_t MGR_REED_getEvent(void)
{
	/* SPSC pop. Snapshot head locally (volatile fetch). If empty, bail. */
	const uint8_t head_snap = evt_head;
	uint8_t tail_snap = evt_tail;
	if (tail_snap == head_snap)
		return MGR_REED_EVT_NONE;

	/* Read slot into a local FIRST, then advance tail. Order matters: if
	 * advancing tail were done first, a writer could overwrite the slot we
	 * are about to read. Reading first guarantees we get a stable value. */
	MGR_REED_Event_t evt = evt_buf[tail_snap & EVT_BUF_MASK];
	evt_tail = tail_snap + 1u;  /* atomic single-byte store on Cortex-M */
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

/* Diagnostic counters — useful when the reed is wired through a flaky
 * harness and you need to confirm the EXTI is actually firing. Read by the
 * heartbeat in kns_app_uw_doppler.c. Single-writer (ISR) / single-reader
 * (main loop) on aligned uint32_t → atomic. */
volatile uint32_t g_reed_isr_count       = 0;
volatile uint32_t g_reed_isr_last_state  = 0;
volatile uint32_t g_reed_isr_pin_at_call = 0;

/**
 * @brief EXTI lines 5-9 interrupt handler. Only PB6 is configured for EXTI
 *        in this build, so dispatch is narrowed accordingly.
 * @note  Overrides the weak default from startup_stm32wl55xx_cm4.s.
 */
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(REED_MCU_Pin);
}

/**
 * @brief HAL GPIO EXTI callback — invoked from HAL_GPIO_EXTI_IRQHandler.
 * @note  Overrides the weak default in the HAL.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	g_reed_isr_count++;
	g_reed_isr_pin_at_call = GPIO_Pin;

	if (GPIO_Pin != REED_MCU_Pin)
		return;

	const uint32_t now = HAL_GetTick();
	const uint32_t pin_state =
	    (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin) == GPIO_PIN_SET) ? 1u : 0u;
	g_reed_isr_last_state = pin_state;

	/* Debounce: ignore edges within the debounce window. uint32_t subtraction
	 * wraps cleanly at 49.7 days uptime. */
	if ((now - last_edge_tick) < REED_DEBOUNCE_MS)
		return;
	last_edge_tick = now;

	MGR_REED_Event_t evt;
	if (pin_state) {
		rising_tick = now;
		evt = MGR_REED_EVT_MAGNET_ON;
	} else {
		const uint32_t r = rising_tick;
		last_hold_ms = (r != 0u) ? (now - r) : 0u;
		rising_tick = 0u;
		evt = MGR_REED_EVT_MAGNET_OFF;
	}

	/* Push into the ring buffer. If full, drop the new event (preserve
	 * order of older events the FSM hasn't seen yet). */
	const uint8_t depth = (uint8_t)(evt_head - evt_tail);
	if (depth < EVT_BUF_SIZE) {
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
