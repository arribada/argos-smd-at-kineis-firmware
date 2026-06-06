/**
 * @file    test_selftest_mask.c
 * @brief   Unit tests for Phase E: boot self-test fail mask
 *
 * Verifies the mask bits set correctly for each input state, and that the
 * LED indication selection picks the most-severe failure.
 */

#include "test_framework.h"

#define SELFTEST_FAIL_BAT_LOW       (1u << 0)
#define SELFTEST_FAIL_BAT_CRITICAL  (1u << 1)
#define SELFTEST_FAIL_RF_DEAD       (1u << 2)
#define SELFTEST_FAIL_SWS_FAULT     (1u << 3)
#define SELFTEST_FAIL_NO_DEPLOY     (1u << 4)

typedef enum {
	LED_GREEN,    /* all OK + deployed */
	LED_BLUE,     /* all OK + not deployed */
	LED_YELLOW,   /* BAT low */
	LED_RED,      /* BAT critical */
	LED_VIOLET,   /* RF dead / SWS fault */
} LedIndication_t;

/* Mirror of build_mask + select_indication from kns_app_uw_doppler.c */
static uint16_t selftest_build_mask(uint16_t vbat_mv, bool rf_ok,
                                     bool sws_fault, bool deployed)
{
	uint16_t mask = 0;
	if (vbat_mv > 0) {
		if (vbat_mv < 2700)
			mask |= SELFTEST_FAIL_BAT_CRITICAL;
		else if (vbat_mv < 3000)
			mask |= SELFTEST_FAIL_BAT_LOW;
	}
	if (!rf_ok)        mask |= SELFTEST_FAIL_RF_DEAD;
	if (sws_fault)     mask |= SELFTEST_FAIL_SWS_FAULT;
	if (!deployed)     mask |= SELFTEST_FAIL_NO_DEPLOY;
	return mask;
}

static LedIndication_t selftest_select_led(uint16_t mask)
{
	if (mask & (SELFTEST_FAIL_RF_DEAD | SELFTEST_FAIL_SWS_FAULT))
		return LED_VIOLET;
	if (mask & SELFTEST_FAIL_BAT_CRITICAL) return LED_RED;
	if (mask & SELFTEST_FAIL_BAT_LOW)      return LED_YELLOW;
	if (mask & SELFTEST_FAIL_NO_DEPLOY)    return LED_BLUE;
	return LED_GREEN;
}

void test_all_ok_deployed_green(void)
{
	TEST_START("All OK + deployed -> GREEN");
	uint16_t m = selftest_build_mask(3500, /*rf_ok*/true, /*sws_fault*/false, /*deployed*/true);
	ASSERT_EQ(0, m);
	ASSERT_EQ(LED_GREEN, selftest_select_led(m));
	TEST_PASS();
}

void test_all_ok_not_deployed_blue(void)
{
	TEST_START("All OK + not deployed -> BLUE");
	uint16_t m = selftest_build_mask(3500, true, false, false);
	ASSERT_EQ(SELFTEST_FAIL_NO_DEPLOY, m);
	ASSERT_EQ(LED_BLUE, selftest_select_led(m));
	TEST_PASS();
}

void test_bat_low_yellow(void)
{
	TEST_START("BAT=2800mV (low) + deployed -> YELLOW");
	uint16_t m = selftest_build_mask(2800, true, false, true);
	ASSERT_EQ(SELFTEST_FAIL_BAT_LOW, m);
	ASSERT_EQ(LED_YELLOW, selftest_select_led(m));
	TEST_PASS();
}

void test_bat_critical_red(void)
{
	TEST_START("BAT=2600mV (critical) -> RED");
	uint16_t m = selftest_build_mask(2600, true, false, true);
	ASSERT_TRUE(m & SELFTEST_FAIL_BAT_CRITICAL);
	ASSERT_EQ(LED_RED, selftest_select_led(m));
	TEST_PASS();
}

void test_rf_dead_violet(void)
{
	TEST_START("RF dead -> VIOLET (overrides battery)");
	uint16_t m = selftest_build_mask(3500, /*rf_ok*/false, false, true);
	ASSERT_TRUE(m & SELFTEST_FAIL_RF_DEAD);
	ASSERT_EQ(LED_VIOLET, selftest_select_led(m));
	TEST_PASS();
}

void test_sws_fault_violet(void)
{
	TEST_START("SWS fault -> VIOLET");
	uint16_t m = selftest_build_mask(3500, true, /*sws_fault*/true, true);
	ASSERT_TRUE(m & SELFTEST_FAIL_SWS_FAULT);
	ASSERT_EQ(LED_VIOLET, selftest_select_led(m));
	TEST_PASS();
}

void test_rf_overrides_bat_critical(void)
{
	TEST_START("RF dead + BAT critical -> VIOLET (most severe wins)");
	uint16_t m = selftest_build_mask(2400, false, false, true);
	ASSERT_TRUE(m & SELFTEST_FAIL_RF_DEAD);
	ASSERT_TRUE(m & SELFTEST_FAIL_BAT_CRITICAL);
	ASSERT_EQ(LED_VIOLET, selftest_select_led(m));
	TEST_PASS();
}

void test_bat_zero_ignored(void)
{
	TEST_START("BAT=0 (read failure) does NOT set any BAT mask bit");
	uint16_t m = selftest_build_mask(0, true, false, true);
	ASSERT_FALSE(m & SELFTEST_FAIL_BAT_LOW);
	ASSERT_FALSE(m & SELFTEST_FAIL_BAT_CRITICAL);
	TEST_PASS();
}

void test_threshold_boundary_3000(void)
{
	TEST_START("BAT=3000mV exactly is OK (boundary)");
	uint16_t m = selftest_build_mask(3000, true, false, true);
	ASSERT_FALSE(m & SELFTEST_FAIL_BAT_LOW);
	ASSERT_FALSE(m & SELFTEST_FAIL_BAT_CRITICAL);
	TEST_PASS();
}

void test_threshold_boundary_2700(void)
{
	TEST_START("BAT=2700mV is BAT_LOW (boundary)");
	uint16_t m = selftest_build_mask(2700, true, false, true);
	ASSERT_TRUE(m & SELFTEST_FAIL_BAT_LOW);
	ASSERT_FALSE(m & SELFTEST_FAIL_BAT_CRITICAL);
	TEST_PASS();
}

int main(void)
{
	printf("=== Self-test fail-mask tests (Phase E) ===\n");

	test_all_ok_deployed_green();
	test_all_ok_not_deployed_blue();
	test_bat_low_yellow();
	test_bat_critical_red();
	test_rf_dead_violet();
	test_sws_fault_violet();
	test_rf_overrides_bat_critical();
	test_bat_zero_ignored();
	test_threshold_boundary_3000();
	test_threshold_boundary_2700();

	printf("\n%d/%d tests passed (%d failed)\n",
	       tests_passed, tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
