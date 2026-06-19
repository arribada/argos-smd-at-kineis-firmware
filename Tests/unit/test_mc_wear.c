/**
 * @file    test_mc_wear.c
 * @brief   Flash-wear semantics of the MC RAM-cache + high-water mark
 *          (MCU_NVM_get/setMC in mcu_nvm.c).
 *
 * The closed lib persists the MC as getMC + setMC(mc+1) after every TX and
 * the app rolls it back (setMC) before every push of a sequence. The old
 * passthrough implementation made every setMC a 5-page-erase rewrite —
 * lethal for the 10k-cycle flash within weeks at deployment TX rates.
 * These tests pin down the wear contract of the fixed implementation:
 *   - per 3-TX sequence: exactly ONE wear-slot program, ZERO erases
 *   - rollback (sequence MC reuse) never touches flash
 *   - flash high-water mark guarantees no MC repetition after a crash
 *   - operator jumps still perform the genuine rewrite
 */

#include "test_framework.h"

/*******************************************************************************
 * MOCKED FLASH PRIMITIVES (counters = the wear contract)
 ******************************************************************************/

enum { ST_OK = 0, ST_ERR = 1 };

static uint64_t flash_counter;      /* full wear-leveled counter value */
static int n_slot_programs;         /* increment_wear_counter calls */
static int n_full_rewrites;         /* set_wear_counter calls (5 erases each) */

static uint64_t MCU_FLASH_read_msg_counter(void) { return flash_counter; }
static int MCU_FLASH_increment_msg_counter(void)
{
	flash_counter++;
	n_slot_programs++;
	return ST_OK;
}
static int MCU_FLASH_set_msg_counter(uint64_t v)
{
	flash_counter = v;
	n_full_rewrites++;
	return ST_OK;
}

/*******************************************************************************
 * LOGIC UNDER TEST (mirrored from mcu_nvm.c)
 ******************************************************************************/

#define MC_SMALL_STEP 64u

static uint16_t message_counter;
static int      mc_cache_valid;
static uint64_t mc_flash_shadow;

static void mc_seed_cache(void)
{
	if (!mc_cache_valid) {
		mc_flash_shadow = MCU_FLASH_read_msg_counter();
		message_counter = (uint16_t)(mc_flash_shadow & 0xFFFF);
		mc_cache_valid = 1;
	}
}

static int MCU_NVM_getMC(uint16_t *mc)
{
	mc_seed_cache();
	*mc = message_counter;
	return ST_OK;
}

static int MCU_NVM_setMC(uint16_t mcTmp)
{
	mc_seed_cache();
	message_counter = mcTmp;

	const uint16_t flash_lo = (uint16_t)(mc_flash_shadow & 0xFFFF);
	const uint16_t fwd = (uint16_t)(mcTmp - flash_lo);

	if (fwd == 0u)
		return ST_OK;
	if (fwd <= MC_SMALL_STEP) {
		for (uint16_t i = 0; i < fwd; i++) {
			if (MCU_FLASH_increment_msg_counter() != ST_OK)
				return ST_ERR;
			mc_flash_shadow++;
		}
		return ST_OK;
	}
	if ((uint16_t)(flash_lo - mcTmp) <= MC_SMALL_STEP)
		return ST_OK;

	mc_flash_shadow = (uint64_t)mcTmp;
	return MCU_FLASH_set_msg_counter((uint64_t)mcTmp);
}

/*******************************************************************************
 * HELPERS — replay the real call patterns
 ******************************************************************************/

static void reset_all(uint64_t flash_init)
{
	flash_counter = flash_init;
	n_slot_programs = 0;
	n_full_rewrites = 0;
	message_counter = 0;
	mc_cache_valid = 0;
	mc_flash_shadow = 0;
}

/* The lib's post-TX persistence: getMC + setMC(mc+1). */
static void lib_post_tx(void)
{
	uint16_t mc;
	MCU_NVM_getMC(&mc);
	MCU_NVM_setMC((uint16_t)(mc + 1u));
}

/* The app's pre-push hook: force the session MC. */
static void app_pre_push(uint16_t session_mc)
{
	MCU_NVM_setMC(session_mc);
}

/*******************************************************************************
 * TEST CASES
 ******************************************************************************/

/** A full 3-TX sequence costs exactly ONE slot program and ZERO rewrites. */
void test_sequence_wear_budget(void)
{
	reset_all(147);
	uint16_t session;
	MCU_NVM_getMC(&session);          /* app snapshot at tx_count==0 */
	ASSERT_EQ(147, session);

	app_pre_push(session); lib_post_tx();   /* TX #0 */
	app_pre_push(session); lib_post_tx();   /* TX #1 (rollback) */
	app_pre_push(session); lib_post_tx();   /* TX #2 (rollback) */

	ASSERT_EQ(1, n_slot_programs);
	ASSERT_EQ(0, n_full_rewrites);
	ASSERT_EQ(148, (long)flash_counter);    /* high-water = next sequence */
	TEST_PASS();
}

/** Two sequences: MC values 147 then 148 on air, 2 slot programs total. */
void test_two_sequences(void)
{
	reset_all(147);
	uint16_t s1, s2;

	MCU_NVM_getMC(&s1);
	app_pre_push(s1); lib_post_tx();
	app_pre_push(s1); lib_post_tx();
	app_pre_push(s1); lib_post_tx();

	MCU_NVM_getMC(&s2);               /* next sequence snapshot */
	ASSERT_EQ(147, s1);
	ASSERT_EQ(148, s2);
	app_pre_push(s2); lib_post_tx();

	ASSERT_EQ(2, n_slot_programs);
	ASSERT_EQ(0, n_full_rewrites);
	TEST_PASS();
}

/** Crash mid-sequence: reboot reads the high-water mark — the crashed
 *  sequence's MC can never be repeated on air. */
void test_no_repeat_after_crash(void)
{
	reset_all(147);
	uint16_t session;
	MCU_NVM_getMC(&session);          /* 147 */
	app_pre_push(session); lib_post_tx();   /* TX #0 done, flash = 148 */
	app_pre_push(session);            /* TX #1 about to fly... crash! */

	/* Reboot: fresh cache, seeded from flash. */
	mc_cache_valid = 0;
	uint16_t after;
	MCU_NVM_getMC(&after);
	ASSERT_EQ(148, after);            /* != 147 -> no repetition */
	TEST_PASS();
}

/** uint16 wrap: 0xFFFF -> 0x0000 is a normal +1 slot program. */
void test_wrap_is_small_step(void)
{
	reset_all(0xFFFF);
	uint16_t mc;
	MCU_NVM_getMC(&mc);
	ASSERT_EQ(0xFFFF, mc);
	lib_post_tx();                    /* setMC(0x0000) */
	ASSERT_EQ(1, n_slot_programs);
	ASSERT_EQ(0, n_full_rewrites);
	MCU_NVM_getMC(&mc);
	ASSERT_EQ(0, mc);
	TEST_PASS();
}

/** Operator jump far forward: genuine full rewrite, exactly once. */
void test_operator_jump_forward(void)
{
	reset_all(147);
	MCU_NVM_setMC(5000);
	ASSERT_EQ(0, n_slot_programs);
	ASSERT_EQ(1, n_full_rewrites);
	ASSERT_EQ(5000, (long)flash_counter);
	TEST_PASS();
}

/** Operator reset to 0 (rollback beyond MC_SMALL_STEP): full rewrite. */
void test_operator_reset_zero(void)
{
	reset_all(5000);
	MCU_NVM_setMC(0);
	ASSERT_EQ(1, n_full_rewrites);
	ASSERT_EQ(0, (long)flash_counter);
	uint16_t mc;
	MCU_NVM_getMC(&mc);
	ASSERT_EQ(0, mc);
	TEST_PASS();
}

/** Redundant setMC with the persisted value: zero flash traffic. */
void test_noop_setmc(void)
{
	reset_all(147);
	MCU_NVM_setMC(147);
	MCU_NVM_setMC(147);
	ASSERT_EQ(0, n_slot_programs);
	ASSERT_EQ(0, n_full_rewrites);
	TEST_PASS();
}

/** One year at 100 TX/day in 3-TX sequences: wear stays trivial. */
void test_one_year_wear(void)
{
	reset_all(0);
	for (int day = 0; day < 365; day++) {
		for (int seq = 0; seq < 33; seq++) {    /* ~100 TX/day */
			uint16_t s;
			MCU_NVM_getMC(&s);
			app_pre_push(s); lib_post_tx();
			app_pre_push(s); lib_post_tx();
			app_pre_push(s); lib_post_tx();
		}
	}
	/* 12045 slot programs = ~12 WL-area wraps = ~12 erases/year versus
	 * 10k endurance. Old implementation: 36500 setMC * 5 page erases. */
	ASSERT_EQ(365 * 33, n_slot_programs);
	ASSERT_EQ(0, n_full_rewrites);
	TEST_PASS();
}

/*******************************************************************************
 * MAIN
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("MC flash wear: RAM cache + high-water mark");

	RUN_TEST(test_sequence_wear_budget);
	RUN_TEST(test_two_sequences);
	RUN_TEST(test_no_repeat_after_crash);
	RUN_TEST(test_wrap_is_small_step);
	RUN_TEST(test_operator_jump_forward);
	RUN_TEST(test_operator_reset_zero);
	RUN_TEST(test_noop_setmc);
	RUN_TEST(test_one_year_wear);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
