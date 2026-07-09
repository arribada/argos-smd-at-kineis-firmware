/**
 * @file    test_boot_loop_escalation.c
 * @brief   Regression spec for the UW_DOPPLER boot-loop escalation thresholds
 *          after the 2026-07-08 "recovery leniency" change (②): factory reset
 *          5 -> 8, permanent-off 10 -> 16.
 *
 * The production logic lives in kns_app_uw_doppler.c (boot_loop_handle /
 * boot_loop_mark_success) against SRAM2 retention + the STM32 HAL, so it cannot
 * be linked directly here. These tests MIRROR the escalation state machine and
 * lock the new contract:
 *   - factory reset fires exactly at BOOT_FAIL_FACTORY_RESET (8) consecutive
 *     pre-MONITORING failures, ONCE only;
 *   - permanent-off fires at BOOT_FAIL_PERMANENT_OFF (16);
 *   - a cold reset (BOR/PIN/OBL) or a boot that reaches MONITORING clears the
 *     counter, so a healthy tag never escalates.
 */

#include "test_framework.h"

#define BOOT_FAIL_FACTORY_RESET    8
#define BOOT_FAIL_PERMANENT_OFF    16

enum { ACT_NONE = 0, ACT_FACTORY_RESET, ACT_PERMANENT_OFF };

/* Mirror of the retention block (the fields the escalation reads). */
static uint16_t consecutive_failures;
static uint8_t  factory_reset_attempted;
static uint8_t  boot_in_progress;

/* Observability for the tests. */
static int last_action;
static int factory_reset_calls;
static int permanent_off_calls;

/* Mirror of boot_loop_handle(): `cold` = this boot's reset cause is a
 * BOR/PIN/OBL (user-initiated / option-byte), which clears the counter. */
static void boot_handle(int cold)
{
	last_action = ACT_NONE;

	if (cold) {
		consecutive_failures = 0;
		factory_reset_attempted = 0;
		boot_in_progress = 1;
		return;
	}

	if (!boot_in_progress) {
		/* Previous boot reached MONITORING — reset from steady state, not a
		 * fault. Don't penalise it. */
		boot_in_progress = 1;
		return;
	}

	/* Previous boot died before MONITORING — count it. */
	if (consecutive_failures < 0xFFFF)
		consecutive_failures++;
	boot_in_progress = 1;

	if (consecutive_failures >= BOOT_FAIL_PERMANENT_OFF) {
		last_action = ACT_PERMANENT_OFF;
		permanent_off_calls++;
		return;
	}
	if (consecutive_failures >= BOOT_FAIL_FACTORY_RESET &&
	    !factory_reset_attempted) {
		last_action = ACT_FACTORY_RESET;
		factory_reset_attempted = 1;
		factory_reset_calls++;
	}
}

/* Mirror of boot_loop_mark_success(): MONITORING reached. */
static void mark_success(void)
{
	consecutive_failures = 0;
	factory_reset_attempted = 0;
	boot_in_progress = 0;
}

static void reset_state(void)
{
	/* Post first-init state: boot_in_progress set, counters clear. */
	consecutive_failures = 0;
	factory_reset_attempted = 0;
	boot_in_progress = 1;
	last_action = ACT_NONE;
	factory_reset_calls = 0;
	permanent_off_calls = 0;
}

/* ---- Tests ---- */

/** No escalation before 8 consecutive pre-MONITORING failures. */
void test_no_action_before_eight(void)
{
	reset_state();
	for (int i = 0; i < 7; i++) {              /* 7 failed boots */
		boot_handle(0);
		ASSERT_EQ(ACT_NONE, last_action);
	}
	ASSERT_EQ(7, (long)consecutive_failures);
	ASSERT_EQ(0, (long)factory_reset_calls);
	TEST_PASS();
}

/** Factory reset fires exactly on the 8th failure. */
void test_factory_reset_at_eight(void)
{
	reset_state();
	for (int i = 0; i < 7; i++)
		boot_handle(0);
	boot_handle(0);                             /* 8th failure */
	ASSERT_EQ(ACT_FACTORY_RESET, last_action);
	ASSERT_EQ(8, (long)consecutive_failures);
	ASSERT_EQ(1, (long)factory_reset_calls);
	TEST_PASS();
}

/** Factory reset is done ONCE — failures 9..15 do not re-trigger it. */
void test_factory_reset_once_only(void)
{
	reset_state();
	for (int i = 0; i < 8; i++)
		boot_handle(0);                         /* reaches factory reset */
	ASSERT_EQ(1, (long)factory_reset_calls);
	for (int i = 9; i <= 15; i++) {             /* failures 9..15 */
		boot_handle(0);
		ASSERT_EQ(ACT_NONE, last_action);       /* no second factory reset, no perm-off yet */
	}
	ASSERT_EQ(1, (long)factory_reset_calls);
	ASSERT_EQ(15, (long)consecutive_failures);
	TEST_PASS();
}

/** Permanent-off fires at 16 consecutive failures. */
void test_permanent_off_at_sixteen(void)
{
	reset_state();
	for (int i = 0; i < 15; i++)
		boot_handle(0);
	ASSERT_EQ(0, (long)permanent_off_calls);
	boot_handle(0);                             /* 16th failure */
	ASSERT_EQ(ACT_PERMANENT_OFF, last_action);
	ASSERT_EQ(16, (long)consecutive_failures);
	ASSERT_EQ(1, (long)permanent_off_calls);
	TEST_PASS();
}

/** A cold reset (NRST / power-on / OBL) clears the counter mid-streak. */
void test_cold_reset_clears_counter(void)
{
	reset_state();
	for (int i = 0; i < 5; i++)
		boot_handle(0);                         /* 5 failures accumulated */
	ASSERT_EQ(5, (long)consecutive_failures);
	boot_handle(1);                             /* cold reset */
	ASSERT_EQ(0, (long)consecutive_failures);
	ASSERT_EQ(ACT_NONE, last_action);
	TEST_PASS();
}

/** Reaching MONITORING resets the counter; a later single fault never escalates. */
void test_monitoring_success_resets(void)
{
	reset_state();
	for (int i = 0; i < 6; i++)
		boot_handle(0);                         /* 6 failures */
	mark_success();                             /* MONITORING reached */
	ASSERT_EQ(0, (long)consecutive_failures);
	/* A reboot from steady state: prev reached MONITORING (boot_in_progress
	 * was cleared) so it is NOT counted. */
	boot_handle(0);
	ASSERT_EQ(0, (long)consecutive_failures);
	ASSERT_EQ(ACT_NONE, last_action);
	TEST_PASS();
}

/** A healthy tag that reaches MONITORING every boot never escalates, even over
 *  many reboots (e.g. dev reflashes / AT+RESET from steady state). */
void test_healthy_reboots_never_escalate(void)
{
	reset_state();
	for (int i = 0; i < 50; i++) {
		boot_handle(0);                         /* boot */
		mark_success();                         /* reaches MONITORING */
		ASSERT_EQ(ACT_NONE, last_action);
	}
	ASSERT_EQ(0, (long)consecutive_failures);
	ASSERT_EQ(0, (long)factory_reset_calls);
	ASSERT_EQ(0, (long)permanent_off_calls);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER boot-loop escalation (8 / 16)");

	RUN_TEST(test_no_action_before_eight);
	RUN_TEST(test_factory_reset_at_eight);
	RUN_TEST(test_factory_reset_once_only);
	RUN_TEST(test_permanent_off_at_sixteen);
	RUN_TEST(test_cold_reset_clears_counter);
	RUN_TEST(test_monitoring_success_resets);
	RUN_TEST(test_healthy_reboots_never_escalate);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
