/**
 * @file    test_lpm_spi_grace.c
 * @brief   STOP-over-SPI gating (Kineis/Lpm/Inc/lpm_spi_gate.h, real code).
 *
 * The grace window is armed on every STOP wake and at the end of every SPI
 * transaction; while it runs, or while the host is mid-frame, the GUI idle
 * loop must not enter STOP (it de-inits SPI1 and loses the pipelined response).
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

	printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
	return tests_failed ? 1 : 0;
}
