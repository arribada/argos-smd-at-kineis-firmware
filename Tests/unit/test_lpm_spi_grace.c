/**
 * @file    test_lpm_spi_grace.c
 * @brief   Unit tests for the STOP-over-SPI grace window (wrap-safe tick logic)
 *
 * Mirrors lpm.c LPM_spiStopGraceActive() + the arm in LPM_stop_exit():
 *   arm:    spi_stop_grace_until_tick = HAL_GetTick() + SPI_STOP_GRACE_MS;
 *   active: (int32_t)(spi_stop_grace_until_tick - HAL_GetTick()) > 0;
 *
 * The grace window keeps the GUI idle loop from re-entering STOP for
 * SPI_STOP_GRACE_MS after a STOP wake, so the host can complete a retried
 * multi-phase SPI transaction (e.g. WRITE_LPM=0x00) before the slave sleeps
 * again. Keep SPI_STOP_GRACE_MS below in sync with lpm.c.
 */

#include "test_framework.h"

#define SPI_STOP_GRACE_MS 500u

/* Mirror of the file-static deadline in lpm.c. */
static uint32_t grace_until;

static void grace_arm(uint32_t now)        { grace_until = now + SPI_STOP_GRACE_MS; }
static bool    grace_active(uint32_t now)  { return (int32_t)(grace_until - now) > 0; }

static void test_fresh_boot_inactive(void)
{
	/* No arm yet: deadline is 0. Must NOT report active on a fresh boot,
	 * otherwise the idle loop would refuse to ever enter STOP. */
	grace_until = 0;
	ASSERT_FALSE(grace_active(0u));
	ASSERT_FALSE(grace_active(1000u));
	ASSERT_FALSE(grace_active(0x80000000u));
	TEST_PASS();
}

static void test_active_right_after_arm(void)
{
	grace_arm(10000u);
	ASSERT_TRUE(grace_active(10000u));        /* same tick as the wake */
	ASSERT_TRUE(grace_active(10001u));
	TEST_PASS();
}

static void test_active_within_window(void)
{
	grace_arm(10000u);
	ASSERT_TRUE(grace_active(10000u + SPI_STOP_GRACE_MS - 1u)); /* last live tick */
	TEST_PASS();
}

static void test_inactive_at_and_after_deadline(void)
{
	grace_arm(10000u);
	ASSERT_FALSE(grace_active(10000u + SPI_STOP_GRACE_MS));      /* deadline reached */
	ASSERT_FALSE(grace_active(10000u + SPI_STOP_GRACE_MS + 50u));
	TEST_PASS();
}

static void test_wrap_safe(void)
{
	/* Wake near the 32-bit tick wrap: deadline overflows past 0. The signed
	 * difference must still resolve the ordering correctly. */
	uint32_t base = 0xFFFFFFFFu - 100u;       /* +500 wraps to ~399 */
	grace_arm(base);
	ASSERT_TRUE(grace_active(base));          /* before deadline despite wrap */
	ASSERT_TRUE(grace_active(base + 100u));   /* still before (now at wrap point) */
	ASSERT_FALSE(grace_active(base + SPI_STOP_GRACE_MS));       /* at deadline */
	ASSERT_FALSE(grace_active(base + SPI_STOP_GRACE_MS + 10u)); /* past deadline */
	TEST_PASS();
}

static void test_rearm_extends_window(void)
{
	/* A second STOP wake mid-window must push the deadline out (each wake
	 * grants a fresh full window). */
	grace_arm(10000u);
	grace_arm(10300u);                        /* re-armed 300 ticks later */
	ASSERT_TRUE(grace_active(10700u));        /* still live off the 2nd arm */
	ASSERT_FALSE(grace_active(10800u));       /* 10300+500 = 10800 deadline */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("STOP-over-SPI grace window");
	RUN_TEST(test_fresh_boot_inactive);
	RUN_TEST(test_active_right_after_arm);
	RUN_TEST(test_active_within_window);
	RUN_TEST(test_inactive_at_and_after_deadline);
	RUN_TEST(test_wrap_safe);
	RUN_TEST(test_rearm_extends_window);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
