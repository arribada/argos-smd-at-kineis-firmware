/**
 * @file    test_cooldown_persist.c
 * @brief   Unit tests for Phase 5A: cooldown persistance across reset
 *
 * Verifies the RTC-delta math that falsifies cooldown_ref_tick after a
 * reset. The chip stores `rtc_seconds_at_cooldown_stamp` in TAMP backup,
 * then on boot reconstructs an equivalent tick offset.
 */

#include "test_framework.h"

#define COOLDOWN_BKP_MAGIC  0x434C4400u  /* "CLD\0" */

/* Mirror of cooldown_restore logic from kns_app_uw_doppler.c.
 * Returns the falsified cooldown_ref_tick. */
static uint32_t cooldown_restore_calc(uint32_t saved_s, uint32_t now_s,
                                       uint32_t boot_tick)
{
	uint32_t elapsed_s;
	if (now_s >= saved_s)
		elapsed_s = now_s - saved_s;
	else
		elapsed_s = (86400u - saved_s) + now_s;  /* 1-day wrap */
	uint32_t elapsed_ms = elapsed_s * 1000u;
	return boot_tick - elapsed_ms;  /* unsigned wrap is intentional */
}

void test_zero_elapsed(void)
{
	TEST_START("Zero elapsed: ref_tick == boot_tick");
	uint32_t ref = cooldown_restore_calc(100, 100, 1000);
	ASSERT_EQ(1000u, ref);
	TEST_PASS();
}

void test_30s_elapsed(void)
{
	TEST_START("30 s elapsed: since_ref reflects 30000 ms");
	uint32_t boot_tick = 1500;       /* 1.5 s into post-boot */
	uint32_t ref = cooldown_restore_calc(100, 130, boot_tick);
	/* boot_tick - ref = 30000 ms (the elapsed time before reset) */
	uint32_t since_ref = boot_tick - ref;
	ASSERT_EQ(30000u, since_ref);
	TEST_PASS();
}

void test_60s_cooldown_satisfied_after_70s_elapsed(void)
{
	TEST_START("60s cooldown satisfied if 70s elapsed across reset");
	uint32_t boot_tick = 500;
	uint32_t ref = cooldown_restore_calc(0, 70, boot_tick);
	uint32_t since_ref = boot_tick - ref;
	ASSERT_TRUE(since_ref >= 60000u);
	TEST_PASS();
}

void test_60s_cooldown_blocked_after_10s_elapsed(void)
{
	TEST_START("60s cooldown blocked if only 10s elapsed across reset");
	uint32_t boot_tick = 500;
	uint32_t ref = cooldown_restore_calc(0, 10, boot_tick);
	uint32_t since_ref = boot_tick - ref;
	ASSERT_FALSE(since_ref >= 60000u);
	TEST_PASS();
}

void test_day_wrap(void)
{
	TEST_START("RTC day-wrap handled correctly");
	/* saved=86390 (10 s before midnight), now=20 (20 s after midnight)
	 * elapsed should be 30 s, not (20 - 86390) = negative */
	uint32_t boot_tick = 1000;
	uint32_t ref = cooldown_restore_calc(86390, 20, boot_tick);
	uint32_t since_ref = boot_tick - ref;
	ASSERT_EQ(30000u, since_ref);
	TEST_PASS();
}

void test_magic_validation(void)
{
	TEST_START("Magic word validates the persisted stamp");
	/* Simulate: only the upper 24 bits are checked.
	 *   0x434C4400 matches  -> valid
	 *   0x00000000          -> invalid (uninitialized)
	 *   0x434C44FF matches  -> valid (lower byte is reserved/ignored) */
	ASSERT_TRUE((0x434C4400u & 0xFFFFFF00u) == COOLDOWN_BKP_MAGIC);
	ASSERT_FALSE((0x00000000u & 0xFFFFFF00u) == COOLDOWN_BKP_MAGIC);
	ASSERT_TRUE((0x434C44FFu & 0xFFFFFF00u) == COOLDOWN_BKP_MAGIC);
	TEST_PASS();
}

int main(void)
{
	printf("=== Cooldown persistance tests (Phase 5A) ===\n");

	test_zero_elapsed();
	test_30s_elapsed();
	test_60s_cooldown_satisfied_after_70s_elapsed();
	test_60s_cooldown_blocked_after_10s_elapsed();
	test_day_wrap();
	test_magic_validation();

	printf("\n%d/%d tests passed (%d failed)\n",
	       tests_passed, tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
