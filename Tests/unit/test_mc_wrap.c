/**
 * @file    test_mc_wrap.c
 * @brief   Regression spec for the Layer-1 9-bit wrap fix in MCU_NVM_setMC.
 *
 * Mirrors the WL classification of MCU_NVM_setMC after the fix: distances are
 * computed on the 9-bit ring so the 511->0 rolling wrap is a normal forward
 * step (one WL slot program, NO page-0 erase), not a destructive rewrite.
 * Counts increment_msg_counter (WL slot, no erase) vs set_msg_counter (page-0
 * erase) calls — the old &0xFFFF compare fired set_msg_counter every 512 TX.
 */

#include "test_framework.h"

#define MC_SMALL_STEP 64u

static uint64_t shadow;
static int inc_count;    /* MCU_FLASH_increment_msg_counter (WL slot, no erase) */
static int set_count;    /* MCU_FLASH_set_msg_counter (page-0 erase) */

/* exact mirror of the patched MCU_NVM_setMC wear-leveling math */
static void setMC(uint16_t mcTmp)
{
	mcTmp = (uint16_t)(mcTmp & 0x1FFu);
	const uint16_t flash_lo = (uint16_t)(shadow & 0x1FFu);
	const uint16_t fwd = (uint16_t)((mcTmp - flash_lo) & 0x1FFu);

	if (fwd == 0u)
		return;
	if (fwd <= MC_SMALL_STEP) {
		for (uint16_t i = 0; i < fwd; i++) { inc_count++; shadow++; }
		return;
	}
	if ((uint16_t)((flash_lo - mcTmp) & 0x1FFu) <= MC_SMALL_STEP)
		return;                                    /* rollback: RAM only */
	shadow = (shadow & ~0x1FFull) + 0x200u + (uint64_t)mcTmp;   /* far jump */
	set_count++;
}

static void reset_at(uint64_t s) { shadow = s; inc_count = 0; set_count = 0; }

static void test_wrap_511_to_0_is_forward(void)
{
	reset_at(511);
	setMC(0);                       /* lib post-TX +1 wraps 511 -> 0 */
	ASSERT_EQ(1, inc_count);        /* one WL slot... */
	ASSERT_EQ(0, set_count);        /* ...NO page-0 erase (was the bug) */
	ASSERT_EQ(512, (long)shadow);   /* shadow stays monotonic */
	TEST_PASS();
}

static void test_post_wrap_cadence(void)
{
	reset_at(512);
	for (int i = 0; i < 600; i++) {
		uint16_t next = (uint16_t)(((shadow & 0x1FFu) + 1u) & 0x1FFu);
		setMC(next);
	}
	ASSERT_EQ(600, inc_count);
	ASSERT_EQ(0, set_count);        /* 600 TX across two more wraps, zero erases */
	TEST_PASS();
}

static void test_legacy_high_shadow_forward(void)
{
	reset_at(5000);                 /* old non-clamped deploy; 5000 & 0x1FF = 392 */
	setMC((uint16_t)((5000u & 0x1FFu) + 1u));   /* lib +1 = 393 */
	ASSERT_EQ(1, inc_count);
	ASSERT_EQ(0, set_count);
	TEST_PASS();
}

static void test_rollback_ram_only(void)
{
	reset_at(100);
	setMC(99);                      /* per-sequence rollback (<=64) */
	ASSERT_EQ(0, inc_count);
	ASSERT_EQ(0, set_count);
	ASSERT_EQ(100, (long)shadow);   /* high-water unchanged */
	TEST_PASS();
}

static void test_far_jump_residue_and_monotonic(void)
{
	reset_at(10);
	setMC(400);                     /* operator far jump (in-ring) */
	ASSERT_EQ(1, set_count);
	ASSERT_EQ(400, (long)(shadow & 0x1FFu));   /* residue == requested MC */
	ASSERT_TRUE(shadow > 10);                   /* strictly monotonic */
	TEST_PASS();
}

static void test_far_jump_high_shadow(void)
{
	/* scenario-4 coverage gap: far jump from a HIGH legacy shadow must still
	 * land residue == mcTmp and never rewind. */
	reset_at(5000);
	setMC(50);
	ASSERT_EQ(1, set_count);
	ASSERT_EQ(50, (long)(shadow & 0x1FFu));
	ASSERT_TRUE(shadow > 5000);
	TEST_PASS();
}

static void test_plus_one_from_every_residue(void)
{
	/* +1 step from EVERY residue 0..511 must be a forward WL slot, never a
	 * page-0 erase — guards the ring-distance comparison against regression. */
	for (uint16_t r = 0; r <= 511; r++) {
		reset_at(r);
		setMC((uint16_t)((r + 1u) & 0x1FFu));
		ASSERT_EQ(1, inc_count);
		ASSERT_EQ(0, set_count);
	}
	TEST_PASS();
}

static void test_forward_boundary_64_65(void)
{
	reset_at(0);
	setMC(64);                      /* fwd == MC_SMALL_STEP -> forward */
	ASSERT_EQ(64, inc_count);
	ASSERT_EQ(0, set_count);

	reset_at(0);
	setMC(65);                      /* fwd > MC_SMALL_STEP -> far jump */
	ASSERT_EQ(0, inc_count);
	ASSERT_EQ(1, set_count);
	TEST_PASS();
}

static void test_rollback_boundary_64_65(void)
{
	reset_at(100);
	setMC(36);                      /* rollback == 64 -> RAM only */
	ASSERT_EQ(0, inc_count);
	ASSERT_EQ(0, set_count);

	reset_at(100);
	setMC(35);                      /* rollback == 65 -> far jump */
	ASSERT_EQ(1, set_count);
	TEST_PASS();
}

/* read_msg_counter() = overflow*1024 + WL_index; on-air MC = value & 0x1FF.
 * A page-0 brownout blanks the overflow word (-> 0) but the WL slot pages
 * (the index) survive on a separate page. This proves losing the overflow
 * count does NOT change the on-air 9-bit MC, so MGR_CRED need not restore it. */
static uint16_t onair(uint64_t overflow, uint16_t index)
{
	return (uint16_t)(((overflow * 1024u) + index) & 0x1FFu);
}

static void test_of_loss_preserves_onair_residue(void)
{
	uint16_t idxs[] = { 0, 1, 200, 511, 512, 1023 };
	for (unsigned i = 0; i < sizeof(idxs)/sizeof(idxs[0]); i++)
		ASSERT_EQ(onair(5, idxs[i]), onair(0, idxs[i]));   /* OF=5 lost -> 0, residue same */

	/* and the full continuing sequence is byte-identical across a brownout */
	uint64_t of_a = 5, of_b = 0; uint16_t idx_a = 200, idx_b = 200;
	for (int n = 0; n < 1500; n++) {
		ASSERT_EQ(onair(of_a, idx_a), onair(of_b, idx_b));
		if (++idx_a >= 1024) { idx_a = 0; of_a++; }
		if (++idx_b >= 1024) { idx_b = 0; of_b++; }
	}
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("MC 9-bit wrap (Layer 1)");
	RUN_TEST(test_wrap_511_to_0_is_forward);
	RUN_TEST(test_post_wrap_cadence);
	RUN_TEST(test_legacy_high_shadow_forward);
	RUN_TEST(test_rollback_ram_only);
	RUN_TEST(test_far_jump_residue_and_monotonic);
	RUN_TEST(test_far_jump_high_shadow);
	RUN_TEST(test_plus_one_from_every_residue);
	RUN_TEST(test_forward_boundary_64_65);
	RUN_TEST(test_rollback_boundary_64_65);
	RUN_TEST(test_of_loss_preserves_onair_residue);
	TEST_SUITE_END();
	TEST_SUMMARY();
}
