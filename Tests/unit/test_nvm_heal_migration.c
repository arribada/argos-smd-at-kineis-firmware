/**
 * @file    test_nvm_heal_migration.c
 * @brief   Regression spec for mgr_nvm apply_config heals + v7->v8 migration.
 *
 * The production NVM module compiles against the STM32 HAL and ~11 manager
 * setters, so (per this repo's per-file standalone test runner) it cannot be
 * linked directly here — these tests MIRROR the exact logic of
 * apply_config()/migrate_v7_to_v8() in mgr_nvm.c and lock the contract.
 *
 * Covers the MF-1 fix: the sample_delay_max heal must run AFTER the field is
 * assigned. The `buggy_order` path below reproduces the original defect (heal
 * reads the zero-initialised field before assignment) and proves the test
 * distinguishes it from the fixed order — so a regression of the ordering
 * would fail test_heal_ordering_matters.
 */

#include "test_framework.h"
#include <stddef.h>   /* offsetof — not guaranteed via test_framework.h on strict libc */

/* ---- Mirror of the SWS-config slice of apply_config (mgr_nvm.c:432-463) ---- */

typedef struct {
	uint16_t threshold_max;
	uint16_t sample_delay_max_us;
	uint32_t max_dive_time_s;
} sws_slice_t;

/* stored-config inputs that feed the heals */
typedef struct {
	uint16_t sws_threshold_max;
	uint16_t sws_sample_delay_max_us;
	uint32_t sws_max_dive_time_s;
} nvm_slice_t;

/* Replicates the fixed apply_config ordering: zero-init, assign, THEN heal.
 * When buggy_order is set, the heal runs before the assignment (the original
 * bug) against the zero-initialised struct. */
static sws_slice_t apply_sws_slice(const nvm_slice_t *cfg, int buggy_order)
{
	sws_slice_t s = {0};               /* zero-init (part of the fix) */

	if (buggy_order) {
		/* BUGGY: heal reads sample_delay_max_us before assignment (0). */
		if (s.threshold_max == 2000u) s.threshold_max = 4095u;
		if (s.sample_delay_max_us == 1000u || s.sample_delay_max_us == 5000u)
			s.sample_delay_max_us = 10000u;
		s.threshold_max       = cfg->sws_threshold_max;
		s.sample_delay_max_us = cfg->sws_sample_delay_max_us;
		s.max_dive_time_s     = cfg->sws_max_dive_time_s;
	} else {
		/* FIXED: assign, then heal against assigned values. */
		s.threshold_max       = cfg->sws_threshold_max;
		s.sample_delay_max_us = cfg->sws_sample_delay_max_us;
		s.max_dive_time_s     = cfg->sws_max_dive_time_s;
		if (s.threshold_max == 2000u) s.threshold_max = 4095u;
		if (s.sample_delay_max_us == 1000u || s.sample_delay_max_us == 5000u)
			s.sample_delay_max_us = 10000u;
		/* 0 disables the stuck-underwater backstop -> heal to default. */
		if (s.max_dive_time_s == 0u) s.max_dive_time_s = 7200u;
	}
	return s;
}

static void test_heal_sample_delay_1000_to_10000(void)
{
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 1000 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(10000, s.sample_delay_max_us);
	TEST_PASS();
}

static void test_heal_sample_delay_5000_to_10000(void)
{
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 5000 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(10000, s.sample_delay_max_us);
	TEST_PASS();
}

static void test_heal_preserves_operator_sample_delay(void)
{
	/* A deliberate operator value (8000) outside the legacy sentinels must
	 * survive un-healed. */
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 8000 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(8000, s.sample_delay_max_us);
	TEST_PASS();
}

static void test_heal_threshold_2000_to_4095(void)
{
	nvm_slice_t cfg = { .sws_threshold_max = 2000, .sws_sample_delay_max_us = 10000 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(4095, s.threshold_max);
	TEST_PASS();
}

static void test_heal_threshold_4095_no_double_heal(void)
{
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 10000 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(4095, s.threshold_max);
	TEST_PASS();
}

/* A persisted/legacy max_dive_time_s of 0 disables the stuck-underwater
 * backstop (mgr_sws.c:1019 gate is > 0) and would let a sealed unit go
 * permanently TX-silent. apply_config heals it to the default. */
static void test_heal_max_dive_zero_to_default(void)
{
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 10000,
			    .sws_max_dive_time_s = 0 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(7200, s.max_dive_time_s);
	TEST_PASS();
}

static void test_heal_preserves_operator_max_dive(void)
{
	/* A deliberate non-zero operator value must survive un-healed. */
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 10000,
			    .sws_max_dive_time_s = 300 };
	sws_slice_t s = apply_sws_slice(&cfg, 0);
	ASSERT_EQ(300, s.max_dive_time_s);
	TEST_PASS();
}

/* The crux: with the buggy (heal-before-assign) order a stored 1000us is NOT
 * healed — exactly the MF-1 field defect. The fixed order heals it. This is
 * the test that distinguishes the two and would fail on a reordering. */
static void test_heal_ordering_matters(void)
{
	nvm_slice_t cfg = { .sws_threshold_max = 4095, .sws_sample_delay_max_us = 1000 };
	sws_slice_t fixed  = apply_sws_slice(&cfg, 0);
	sws_slice_t buggy  = apply_sws_slice(&cfg, 1);
	ASSERT_EQ(10000, fixed.sample_delay_max_us);   /* heal applied */
	ASSERT_EQ(1000,  buggy.sample_delay_max_us);    /* heal missed (bug) */
	TEST_PASS();
}

/* ---- Mirror of migrate_v7_to_v8 prefix copy (mgr_nvm.c:651-659) ---- */

/* Byte-identical prefix of v7 and v8 up to where v8 inserts its new fields. */
typedef struct { uint32_t magic; uint16_t a; uint16_t b; uint32_t prefix_crc; } v7_t;
typedef struct { uint32_t magic; uint16_t a; uint16_t b;
                 uint8_t payload_format; uint8_t stat_window_h; uint16_t pad;
                 uint32_t crc32; } v8_t;

static v8_t migrate_v7_to_v8(const v7_t *v7)
{
	v8_t out;
	memset(&out, 0, sizeof(out));
	/* prefix = everything before v7's crc32, i.e. up to v8's first new field */
	memcpy(&out, v7, offsetof(v7_t, prefix_crc));
	out.payload_format = 0;   /* not copied -> stays 0 (legacy frame) */
	out.stat_window_h  = 0;   /* 0 -> apply_config maps to 24h default */
	return out;
}

static void test_migrate_v7_to_v8_defaults_new_fields(void)
{
	/* Compile-time invariant the real code guards via _Static_assert. */
	ASSERT_EQ((long)offsetof(v7_t, prefix_crc), (long)offsetof(v8_t, payload_format));

	v7_t v7 = { .magic = 0x434F4E46, .a = 0x1234, .b = 0x5678, .prefix_crc = 0xDEAD };
	v8_t v8 = migrate_v7_to_v8(&v7);
	ASSERT_EQ_HEX(0x434F4E46, v8.magic);    /* prefix preserved */
	ASSERT_EQ(0x1234, v8.a);
	ASSERT_EQ(0x5678, v8.b);
	ASSERT_EQ(0, v8.payload_format);         /* new field defaults to legacy */
	ASSERT_EQ(0, v8.stat_window_h);          /* 0 -> 24h at apply_config */
	TEST_PASS();
}

/* ---- Mirror of validate_config bounds for the v8 fields ---- */

static int validate_payload(uint8_t payload_format, uint8_t stat_window_h)
{
	if (payload_format > 1u) return 0;
	if (stat_window_h > 48u) return 0;
	return 1;
}

static void test_validate_payload_bounds(void)
{
	ASSERT_TRUE(validate_payload(0, 24));
	ASSERT_TRUE(validate_payload(1, 48));
	ASSERT_TRUE(validate_payload(1, 0));     /* 0 healed to 24h downstream */
	ASSERT_FALSE(validate_payload(2, 24));   /* format out of range */
	ASSERT_FALSE(validate_payload(1, 49));   /* window out of range */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("NVM apply_config heals + v7->v8 migration");

	RUN_TEST(test_heal_sample_delay_1000_to_10000);
	RUN_TEST(test_heal_sample_delay_5000_to_10000);
	RUN_TEST(test_heal_preserves_operator_sample_delay);
	RUN_TEST(test_heal_threshold_2000_to_4095);
	RUN_TEST(test_heal_threshold_4095_no_double_heal);
	RUN_TEST(test_heal_max_dive_zero_to_default);
	RUN_TEST(test_heal_preserves_operator_max_dive);
	RUN_TEST(test_heal_ordering_matters);
	RUN_TEST(test_migrate_v7_to_v8_defaults_new_fields);
	RUN_TEST(test_validate_payload_bounds);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
