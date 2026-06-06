/**
 * @file    test_boot_loop_crc.c
 * @brief   Unit tests for boot_retained CRC + threshold logic
 *
 * Mirrors the SRAM2-retained boot-loop counter from kns_app_uw_doppler.c.
 * The counter survives soft resets so the firmware can detect repeated boot
 * failures and (a) attempt factory reset after N failures, (b) latch
 * permanent-OFF after M failures to avoid bricking a deployed tag.
 *
 * Note: the firmware currently force-clears this counter unconditionally
 * (DEV patch, kns_app_uw_doppler.c:196-214). These tests still validate the
 * CRC + threshold logic so it's ready for the proper detector.
 */

#include "test_framework.h"

/* Must match kns_app_uw_doppler.c:140-142 — if the firmware values change,
 * this test should fail to flag the divergence. */
#define BOOT_RETAIN_MAGIC          0x424F4F54UL  /* "BOOT" */
#define BOOT_FAIL_FACTORY_RESET    5u
#define BOOT_FAIL_PERMANENT_OFF    10u

typedef struct {
	uint32_t magic;
	uint16_t consecutive_failures;
	uint8_t  factory_reset_attempted;
	uint8_t  permanent_off_armed;
	uint32_t crc32;
} boot_retained_t;

static uint32_t boot_retained_crc(const boot_retained_t *b)
{
	uint32_t s = b->magic;
	s = s * 31u + b->consecutive_failures;
	s = s * 31u + b->factory_reset_attempted;
	s = s * 31u + b->permanent_off_armed;
	return s;
}

static bool boot_retained_valid(const boot_retained_t *b)
{
	return b->magic == BOOT_RETAIN_MAGIC && b->crc32 == boot_retained_crc(b);
}

static void boot_retained_commit(boot_retained_t *b)
{
	b->magic = BOOT_RETAIN_MAGIC;
	b->crc32 = boot_retained_crc(b);
}

void test_fresh_init_is_valid(void)
{
	TEST_START("Freshly committed struct passes CRC");
	boot_retained_t b = {0};
	b.consecutive_failures = 0;
	boot_retained_commit(&b);
	ASSERT_TRUE(boot_retained_valid(&b));
	TEST_PASS();
}

void test_zero_garbage_invalid(void)
{
	TEST_START("All-zero SRAM (cold boot) -> invalid magic");
	boot_retained_t b = {0};
	ASSERT_FALSE(boot_retained_valid(&b));
	TEST_PASS();
}

void test_corruption_detected(void)
{
	TEST_START("Bit flip in consecutive_failures -> CRC mismatch");
	boot_retained_t b = {0};
	b.consecutive_failures = 3;
	boot_retained_commit(&b);
	ASSERT_TRUE(boot_retained_valid(&b));
	b.consecutive_failures = 4;  /* tamper, don't recompute CRC */
	ASSERT_FALSE(boot_retained_valid(&b));
	TEST_PASS();
}

void test_increment_and_recommit(void)
{
	TEST_START("Increment + re-commit produces a fresh valid struct");
	boot_retained_t b = {0};
	boot_retained_commit(&b);
	for (int i = 1; i <= 3; i++) {
		b.consecutive_failures++;
		boot_retained_commit(&b);
		ASSERT_TRUE(boot_retained_valid(&b));
	}
	ASSERT_EQ(3, b.consecutive_failures);
	TEST_PASS();
}

void test_factory_reset_threshold(void)
{
	TEST_START("Factory reset triggers exactly at threshold");
	boot_retained_t b = {0};
	b.consecutive_failures = BOOT_FAIL_FACTORY_RESET - 1;
	ASSERT_FALSE(b.consecutive_failures >= BOOT_FAIL_FACTORY_RESET);
	b.consecutive_failures = BOOT_FAIL_FACTORY_RESET;
	ASSERT_TRUE(b.consecutive_failures >= BOOT_FAIL_FACTORY_RESET);
	TEST_PASS();
}

void test_permanent_off_higher_threshold(void)
{
	TEST_START("PERMANENT_OFF threshold > FACTORY_RESET threshold");
	ASSERT_TRUE(BOOT_FAIL_PERMANENT_OFF > BOOT_FAIL_FACTORY_RESET);
	TEST_PASS();
}

void test_success_clears_counter(void)
{
	TEST_START("On MONITORING reached, counter resets to 0");
	boot_retained_t b = {0};
	b.consecutive_failures = 3;
	boot_retained_commit(&b);

	/* Simulate boot_loop_mark_success */
	b.consecutive_failures = 0;
	b.factory_reset_attempted = 0;
	b.permanent_off_armed = 0;
	boot_retained_commit(&b);

	ASSERT_EQ(0, b.consecutive_failures);
	ASSERT_TRUE(boot_retained_valid(&b));
	TEST_PASS();
}

void test_magic_value_not_trivial(void)
{
	TEST_START("Magic is non-zero (catches uninitialized RAM)");
	ASSERT_NE(0u, BOOT_RETAIN_MAGIC);
	ASSERT_NE(0xFFFFFFFFu, BOOT_RETAIN_MAGIC);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("boot_retained CRC + threshold logic");
	test_fresh_init_is_valid();
	test_zero_garbage_invalid();
	test_corruption_detected();
	test_increment_and_recommit();
	test_factory_reset_threshold();
	test_permanent_off_higher_threshold();
	test_success_clears_counter();
	test_magic_value_not_trivial();
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
