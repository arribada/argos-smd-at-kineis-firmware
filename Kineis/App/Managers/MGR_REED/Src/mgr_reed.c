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

/* Poll-based debouncer. Replaces the old ISR-driven approach which couldn't
 * cope with Hall-sensor hysteresis: after the operator removes the magnet
 * the Hall output may take hundreds of ms to settle (slow analog comparator
 * crossing), generating spurious edges and a polled-HIGH state that lingers
 * past release. The poll/confirm scheme below requires N consecutive samples
 * in the same state before declaring a transition — Hall settling artefacts
 * are absorbed.
 *
 * The ISR is kept for STOP2 wake (PB6 is not a WL55 WKUP pin, but EXTI lines
 * can still wake from STOP modes) and as a diagnostic counter, but it no
 * longer pushes events itself. */
#define REED_POLL_INTERVAL_MS    10u   /**< Sample cadence in main loop */
/* Asymmetric debouncing.
 *
 * ON path  (LOW → HIGH): the magnet just appeared. Clean snap edges, no
 * hysteresis at this side. A short confirmation (50 ms = 5 samples) keeps
 * the gesture feeling snappy — the operator presses, the LED lights up
 * almost immediately.
 *
 * OFF path (HIGH → LOW): the magnet was removed. Hall hysteresis can hold
 * HIGH for up to ~200 ms after release, with brief LOW dips at the
 * threshold crossing. A long confirmation (200 ms = 20 samples) absorbs
 * those dips so we don't fire a spurious MAGNET_OFF mid-gesture. */
#define REED_DEBOUNCE_ON_SAMPLES   5u   /**< 5 * 10 ms = 50 ms */
#define REED_DEBOUNCE_OFF_SAMPLES 20u   /**< 20 * 10 ms = 200 ms */

/* Event ring buffer. Power-of-two for mask trick. 8 entries: 2 x debounce
 * window worth of edges, gives headroom under bounce. */
#define EVT_BUF_SIZE  8u
#define EVT_BUF_MASK  (EVT_BUF_SIZE - 1u)

_Static_assert((EVT_BUF_SIZE & EVT_BUF_MASK) == 0u,
               "EVT_BUF_SIZE must be a power of two");

static volatile MGR_REED_Event_t evt_buf[EVT_BUF_SIZE];
static volatile uint8_t evt_head = 0;  /**< Next write index (poll only) */
static volatile uint8_t evt_tail = 0;  /**< Next read index (main only) */

static volatile uint32_t rising_tick = 0;     /**< Tick of last rising edge */
static volatile uint32_t last_hold_ms = 0;    /**< Duration of last completed hold */
static volatile uint32_t last_edge_tick = 0;  /**< Tick of last confirmed edge */

/* Debouncer state. Confirmed-state is what the rest of the firmware sees
 * via MGR_REED_isMagnetPresent. The candidate-state is the raw pin
 * reading; it must match the candidate for N consecutive polls before
 * being promoted to confirmed. */
static uint32_t s_last_poll_tick   = 0;
static uint8_t  s_debounce_counter = 0;
static bool     s_confirmed_state  = false;  /**< false = no magnet */
static bool     s_candidate_state  = false;

/* EXTI vector follows the reed pin: PB3 (WKUP3 bench mod) is on the
 * dedicated EXTI3 line, default PB6 shares EXTI9_5. */
#if defined(BSP_REED_ON_WKUP3)
#define REED_EXTI_IRQn  EXTI3_IRQn
#else
#define REED_EXTI_IRQn  EXTI9_5_IRQn
#endif

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

	/* Seed the debouncer with the current pin state — no event synthesised
	 * here (the poll loop will publish an MAGNET_ON event ~200 ms later if
	 * the magnet is really present, after confirming stability). Treating
	 * the very first reading as authoritative was the bug behind phantom
	 * boot-time MAGNET_ON events on a Hall whose output drifts briefly
	 * after power-up. */
	const bool init_pin = (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin)
	                       == GPIO_PIN_SET);
	s_confirmed_state  = false;       /* assume no magnet, debouncer corrects */
	s_candidate_state  = init_pin;
	s_debounce_counter = 0;
	s_last_poll_tick   = HAL_GetTick();
	evt_head = 0;
	evt_tail = 0;
	rising_tick = 0;
	last_edge_tick = 0;

	HAL_NVIC_SetPriority(REED_EXTI_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(REED_EXTI_IRQn);
}

/* Poll the pin and update the debouncer state. Called lazily from
 * getEvent / isMagnetPresent so callers don't need to add a task entry.
 * Cheap (one GPIO read + counter update) and rate-limited to
 * REED_POLL_INTERVAL_MS. */
static void reed_poll(void)
{
	const uint32_t now = HAL_GetTick();
	if ((uint32_t)(now - s_last_poll_tick) < REED_POLL_INTERVAL_MS)
		return;
	s_last_poll_tick = now;

	const bool raw = (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin)
	                  == GPIO_PIN_SET);

	if (raw == s_confirmed_state) {
		/* Same as confirmed → no transition pending. */
		s_candidate_state  = raw;
		s_debounce_counter = 0;
		return;
	}

	if (raw != s_candidate_state) {
		/* Candidate changed mid-debounce — restart the counter on the
		 * new candidate. This is the path that absorbs bounce: a glitch
		 * back to the old state resets the count, so the change only
		 * confirms on a sustained N-sample window. */
		s_candidate_state  = raw;
		s_debounce_counter = 1u;
		return;
	}

	s_debounce_counter++;
	/* Asymmetric threshold: short for LOW→HIGH (snappy gesture detection),
	 * long for HIGH→LOW (Hall hysteresis filter). */
	const uint8_t needed = raw ? REED_DEBOUNCE_ON_SAMPLES
	                           : REED_DEBOUNCE_OFF_SAMPLES;
	if (s_debounce_counter < needed)
		return;

	/* Confirmed transition. Publish the event + bookkeep timing. */
	s_confirmed_state  = raw;
	s_debounce_counter = 0;

	MGR_REED_Event_t evt;
	if (raw) {
		rising_tick = now;
		evt = MGR_REED_EVT_MAGNET_ON;
	} else {
		const uint32_t r = rising_tick;
		last_hold_ms = (r != 0u) ? (now - r) : 0u;
		rising_tick = 0u;
		evt = MGR_REED_EVT_MAGNET_OFF;
	}
	last_edge_tick = now;

	const uint8_t depth = (uint8_t)(evt_head - evt_tail);
	if (depth < EVT_BUF_SIZE) {
		evt_buf[evt_head & EVT_BUF_MASK] = evt;
		evt_head++;
	}
}

bool MGR_REED_isMagnetPresent(void)
{
	reed_poll();
	return s_confirmed_state;
}

bool MGR_REED_isDebouncing(void)
{
	/* True while the candidate state differs from the confirmed state —
	 * i.e. a transition is being polled but has not yet accumulated the
	 * required consecutive-sample count. The LPM client uses this to keep
	 * the chip awake long enough for the debouncer to converge; without
	 * the check, a STOP2 cycle could re-enter mid-debounce (debouncer
	 * accumulates ~1 sample per wake, so the magnet edge never confirms
	 * → no MAGNET_ON event → gesture LED never fires). */
	return s_candidate_state != s_confirmed_state;
}

MGR_REED_Event_t MGR_REED_getEvent(void)
{
	/* Poll the debouncer first so a caller that hammers getEvent()
	 * naturally drives the sampling. Rate-limited internally. */
	reed_poll();

	const uint8_t head_snap = evt_head;
	uint8_t tail_snap = evt_tail;
	if (tail_snap == head_snap)
		return MGR_REED_EVT_NONE;

	MGR_REED_Event_t evt = evt_buf[tail_snap & EVT_BUF_MASK];
	evt_tail = tail_snap + 1u;
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
 * @brief EXTI interrupt handler for the reed line. Only the reed pin is
 *        configured for EXTI in this build, so dispatch is narrowed
 *        accordingly: EXTI3 for the PB3/WKUP3 bench mod, EXTI9_5 for the
 *        default PB6 wiring.
 * @note  Overrides the weak default from startup_stm32wl55xx_cm4.s.
 */
#if defined(BSP_REED_ON_WKUP3)
void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(REED_MCU_Pin);
}
#else
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(REED_MCU_Pin);
}
#endif

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

	/* Snapshot raw pin state for the heartbeat — the actual event
	 * push happens in the poll-based debouncer (reed_poll). Keeping the
	 * ISR enabled is what wakes the chip from STOP2 on a magnet edge;
	 * doing the debounce in the main loop guarantees Hall-hysteresis
	 * settling artefacts cannot inject spurious events. */
	g_reed_isr_last_state =
	    (HAL_GPIO_ReadPin(REED_MCU_GPIO_Port, REED_MCU_Pin) == GPIO_PIN_SET) ? 1u : 0u;
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
