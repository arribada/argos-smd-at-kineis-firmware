/**
 * @file    test_mc_9bit.c
 * @brief   9-bit Message Counter clamp / modulo invariant.
 *
 * The Argos/Kineis MC is a 9-bit field (0..511). The closed libkineis uses the
 * raw getMC value for the CURRENT frame, writing it BOTH as a 9-bit header AND
 * as the 16-bit AES-CTR counter (it only masks & 0x1ff when storing the *next*
 * value). So any MC > 511 wraps in the header (e.g. 512 -> 0) but not in the
 * AES counter, and the ground segment decrypts the payload to garbage.
 *
 * Defence: the operator AT+MC handler folds the value mod 512
 * (mgr_at_cmd_list_general.c) and MCU_NVM_get/setMC clamp & 0x1ff
 * (mcu_nvm.c). These tests mirror and lock that invariant.
 */

#include "test_framework.h"

/* ---- Mirror of MCU_NVM_get/setMC clamp (mcu_nvm.c) ---- */
static uint16_t stored_mc;
static void     nvm_setMC(uint16_t v) { stored_mc = (uint16_t)(v & 0x1FFu); }
static uint16_t nvm_getMC(void)       { return (uint16_t)(stored_mc & 0x1FFu); }

/* ---- Mirror of the AT+MC operator fold (mgr_at_cmd_list_general.c) ---- */
static uint16_t at_mc_fold(long mc_in) { return (uint16_t)(mc_in % 512); }

/** In-range values (0..511) pass through unchanged. */
void test_in_range_unchanged(void)
{
	for (uint16_t v = 0; v <= 511u; v = (uint16_t)(v + 73u)) {
		nvm_setMC(v);
		ASSERT_EQ(v, nvm_getMC());
	}
	nvm_setMC(511); ASSERT_EQ(511, nvm_getMC());
	TEST_PASS();
}

/** Values > 511 fold mod 512 — never a raw value the lib could corrupt. */
void test_over_511_folds(void)
{
	nvm_setMC(512);   ASSERT_EQ(0,   nvm_getMC());   /* the corruption boundary */
	nvm_setMC(513);   ASSERT_EQ(1,   nvm_getMC());
	nvm_setMC(600);   ASSERT_EQ(88,  nvm_getMC());   /* 600 - 512            */
	nvm_setMC(1023);  ASSERT_EQ(511, nvm_getMC());   /* last before 2nd wrap */
	nvm_setMC(1024);  ASSERT_EQ(0,   nvm_getMC());   /* 2 * 512              */
	nvm_setMC(65535); ASSERT_EQ(511, nvm_getMC());   /* 0xFFFF & 0x1FF       */
	TEST_PASS();
}

/** Whatever is stored, getMC always hands the lib a valid 9-bit value. */
void test_getmc_always_9bit(void)
{
	for (long v = 0; v <= 65535; v += 257) {
		nvm_setMC((uint16_t)v);
		ASSERT_TRUE(nvm_getMC() <= 511u);
	}
	TEST_PASS();
}

/** The operator fold and the NVM clamp agree: AT+MC echo == frame value. */
void test_at_mc_fold_matches_nvm(void)
{
	ASSERT_EQ(0,   at_mc_fold(512));
	ASSERT_EQ(88,  at_mc_fold(600));
	ASSERT_EQ(511, at_mc_fold(65535));
	ASSERT_EQ(300, at_mc_fold(300));

	/* What the operator gets back (fold) is exactly what the frame ships
	 * (nvm clamp of the same input) — no silent divergence. */
	for (long in = 0; in <= 2000; in += 137) {
		nvm_setMC((uint16_t)in);
		ASSERT_EQ(at_mc_fold(in), nvm_getMC());
	}
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("MC 9-bit clamp / modulo invariant");

	RUN_TEST(test_in_range_unchanged);
	RUN_TEST(test_over_511_folds);
	RUN_TEST(test_getmc_always_9bit);
	RUN_TEST(test_at_mc_fold_matches_nvm);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
