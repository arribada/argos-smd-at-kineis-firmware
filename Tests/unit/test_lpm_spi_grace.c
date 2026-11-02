/**
 * @file    test_lpm_spi_grace.c
 * @brief   STOP-over-SPI gating (Kineis/Lpm/Inc/lpm_spi_gate.h, real code).
 *
 * The grace window is armed on every STOP wake and at the end of every SPI
 * transaction; while it runs, or while the host is mid-frame, the GUI idle
 * loop must not enter STOP (it de-inits SPI1 and loses the pipelined response).
 *
 * SLEEP keeps SPI1 running, but a frame must still wake the CPU and the
 * driver's end-of-transaction test needs HAL_GetTick to advance.
 */

#include "test_framework.h"
#include "lpm_spi_gate.h"

static volatile uint32_t deadline;

static void arm(uint32_t now)     { deadline = LPM_spiGraceDeadline(now, LPM_SPI_STOP_GRACE_MS); }
static bool active(uint32_t now)  { return LPM_spiGraceActive(&deadline, now); }

static void test_fresh_boot_inactive(void)
{
	TEST_START("fresh boot: disarmed window never blocks");
	deadline = 0u;
	ASSERT_FALSE(active(0u));
	ASSERT_FALSE(active(0x80000000u));
	TEST_PASS();
}

static void test_window_bounds(void)
{
	TEST_START("active from arm to deadline-1, inactive at the deadline");
	arm(10000u);
	ASSERT_TRUE(active(10000u));
	ASSERT_TRUE(active(10000u + LPM_SPI_STOP_GRACE_MS - 1u));
	ASSERT_FALSE(active(10000u + LPM_SPI_STOP_GRACE_MS));
	TEST_PASS();
}

static void test_expiry_clears(void)
{
	TEST_START("expired window is cleared, cannot re-engage 2^31 ms later");
	arm(10000u);
	ASSERT_FALSE(active(10000u + LPM_SPI_STOP_GRACE_MS));
	ASSERT_EQ(0u, deadline);
	ASSERT_FALSE(active(10000u + 0x80000000u));
	TEST_PASS();
}

static void test_wrap_safe(void)
{
	TEST_START("window spanning the 32-bit tick wrap");
	uint32_t base = 0xFFFFFFFFu - 100u;
	arm(base);
	ASSERT_TRUE(active(base + 100u));
	ASSERT_TRUE(active(base + LPM_SPI_STOP_GRACE_MS - 1u));
	ASSERT_FALSE(active(base + LPM_SPI_STOP_GRACE_MS));
	TEST_PASS();
}

static void test_zero_deadline_sentinel(void)
{
	TEST_START("deadline landing on 0 is nudged to 1 (0 = disarmed)");
	arm(0u - LPM_SPI_STOP_GRACE_MS);
	ASSERT_EQ(1u, deadline);
	ASSERT_TRUE(active(0u - 1u));
	TEST_PASS();
}

static void test_host_session_never_stops(void)
{
	TEST_START("host session: command, poll at +30 ms, next command: no STOP between");
	uint32_t t = 50000u;
	arm(t);                                    /* command frame ends */
	ASSERT_TRUE(LPM_spiStopGate(active(t + 30u), false, false, false));   /* response armed */
	arm(t + 30u);                              /* poll transaction ends */
	ASSERT_TRUE(LPM_spiStopGate(active(t + 250u), false, false, false));  /* next command gap */
	arm(t + 250u);                             /* next command */
	ASSERT_FALSE(LPM_spiStopGate(active(t + 250u + LPM_SPI_STOP_GRACE_MS), false, false, false));
	TEST_PASS();
}

static void test_gate_conditions(void)
{
	TEST_START("each host-activity condition blocks STOP on its own");
	ASSERT_FALSE(LPM_spiStopGate(false, false, false, false));
	ASSERT_TRUE(LPM_spiStopGate(true, false, false, false));   /* grace window */
	ASSERT_TRUE(LPM_spiStopGate(false, true, false, false));   /* NSS low: frame in flight */
	ASSERT_TRUE(LPM_spiStopGate(false, false, true, false));   /* bytes not processed */
	ASSERT_TRUE(LPM_spiStopGate(false, false, false, true));   /* command pending */
	TEST_PASS();
}

/*
 * SLEEP model: the host clocks one frame at t=0, then stays quiet. The DMA
 * raises an IRQ from 32 bytes (half-transfer); the CPU also wakes on the 1 ms
 * tick when SLEEP keeps it. Awake, a loop pass mirrors the driver's
 * end-of-transaction test (count stable for 3 ms of HAL_GetTick) and the real
 * gate. Returns the wall-clock ms at which the frame is processed, or NEVER.
 */
#define NEVER         0xFFFFFFFFu
#define RX_STABLE_MS  3u
#define DMA_HT_BYTES  32u

static uint32_t sleep_latency_ms(uint32_t frame_len, bool keeps_tick, bool gate_rx)
{
	volatile uint32_t grace = 0u;
	uint32_t tick = 0u;
	uint32_t stable_since = 0u;
	bool dma_irq = frame_len >= DMA_HT_BYTES;
	bool awake = false;
	bool seen = false;

	for (uint32_t us = 0u; us < 10000000u; us += 100u) {
		bool ms_edge = (us != 0u) && (us % 1000u == 0u);

		if (ms_edge && (awake || keeps_tick))
			tick++;
		if (!awake) {
			if (dma_irq)
				dma_irq = false;
			else if (!(ms_edge && keeps_tick))
				continue;
			awake = true;
		}
		if (!seen) {
			seen = true;
			stable_since = tick;
		} else if (tick - stable_since >= RX_STABLE_MS) {
			return us / 1000u;
		}
		if (!LPM_spiStopGate(LPM_spiGraceActive(&grace, tick), false, gate_rx, false))
			awake = false;
	}
	return NEVER;
}

static void test_sleep_short_frame(void)
{
	TEST_START("SLEEP: a frame below the DMA half-transfer is processed within 4 ms");
	ASSERT_TRUE(sleep_latency_ms(12u, LPM_SPI_SLEEP_KEEPS_TICK, true) <= 4u);
	ASSERT_TRUE(sleep_latency_ms(12u, LPM_SPI_SLEEP_KEEPS_TICK, false) <= 4u);
	TEST_PASS();
}

static void test_sleep_full_frame(void)
{
	TEST_START("SLEEP: a 64-byte frame is processed within 4 ms");
	ASSERT_TRUE(sleep_latency_ms(64u, LPM_SPI_SLEEP_KEEPS_TICK, true) <= 4u);
	ASSERT_TRUE(sleep_latency_ms(64u, LPM_SPI_SLEEP_KEEPS_TICK, false) <= 4u);
	TEST_PASS();
}

static void test_sleep_suspended_tick_starves(void)
{
	TEST_START("SLEEP with a suspended tick: frames are never processed");
	ASSERT_EQ(NEVER, sleep_latency_ms(12u, false, true));   /* no IRQ, no tick */
	ASSERT_EQ(NEVER, sleep_latency_ms(64u, false, false));  /* tick frozen after the DMA IRQ */
	TEST_PASS();
}

int main(void)
{
	printf("\n=== STOP-over-SPI gating ===\n");
	test_fresh_boot_inactive();
	test_window_bounds();
	test_expiry_clears();
	test_wrap_safe();
	test_zero_deadline_sentinel();
	test_host_session_never_stops();
	test_gate_conditions();
	test_sleep_short_frame();
	test_sleep_full_frame();
	test_sleep_suspended_tick_starves();

	printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
	return tests_failed ? 1 : 0;
}
