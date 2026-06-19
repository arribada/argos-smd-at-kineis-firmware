/**
 * @file    test_seq_mc.c
 * @brief   Unit tests for UW_DOPPLER per-sequence Message Counter and
 *          sequence-restart timer.
 *
 * Mirrors the logic in kns_app_uw_doppler.c:
 *   - MC snapshot at tx_count==0 (sequence start), same MC pushed for every
 *     TX of the sequence via KNS_CFG_setMC, lib auto-increments the
 *     persisted value after each TX so the next sequence naturally starts
 *     at session_mc+1 with no explicit add.
 *   - Sequence restart: cap reached + tx_seq_restart_s elapsed since the
 *     last TX -> new sequence (tx_count reset, MC bumps via re-snapshot).
 *   - Boot-as-first-surface-event one-shot.
 */

#include "test_framework.h"

/*******************************************************************************
 * MOCKED LIB STATE (KNS_CFG_get/setMC behaviour incl. auto-increment)
 ******************************************************************************/

static uint16_t lib_persisted_mc;   /* what KNS_CFG_getMC returns */

static int KNS_CFG_getMC(uint16_t *mc) { *mc = lib_persisted_mc; return 0; }
static void KNS_CFG_setMC(uint16_t mc) { lib_persisted_mc = mc; }
/* The closed lib increments its persisted MC after each successful TX. */
static void lib_tx_complete(void) { lib_persisted_mc++; }

/*******************************************************************************
 * LOGIC UNDER TEST (mirrored from kns_app_uw_doppler.c)
 ******************************************************************************/

typedef struct {
	uint16_t session_mc;
	uint8_t  initialised;
} mc_retained_t;

static mc_retained_t mc_retained;
static uint32_t tx_count;
static uint32_t last_tx_tick;
static uint16_t tx_seq_restart_s;
static uint8_t  tx_max_count;

static void mc_boot_seed(void)
{
	uint16_t mc;
	if (KNS_CFG_getMC(&mc) == 0) {
		mc_retained.session_mc = mc;
		mc_retained.initialised = 1;
	}
}

/* Pre-TX hook: snapshot at sequence start, then force the session MC into
 * the lib before every push (mirrors kns_app_uw_doppler.c:2282-2294). */
static uint16_t mc_before_push(void)
{
	if (tx_count == 0) {
		uint16_t mc_now;
		if (KNS_CFG_getMC(&mc_now) == 0)
			mc_retained.session_mc = mc_now;
	}
	KNS_CFG_setMC(mc_retained.session_mc);
	return mc_retained.session_mc;
}

static void tx_send_one(void)
{
	(void)mc_before_push();
	lib_tx_complete();      /* lib persists session_mc + 1 */
	tx_count++;
	last_tx_tick = 1;       /* non-zero marker, real tick set by caller */
}

static void reset_tx_scheduling(void) { tx_count = 0; }

/* Sequence-restart condition (kns_app_uw_doppler.c:2093). */
static int seq_restart_due(uint32_t now_ms)
{
	return tx_seq_restart_s > 0 &&
	       tx_max_count > 0 && tx_count >= tx_max_count &&
	       last_tx_tick > 0 &&
	       (now_ms - last_tx_tick) >= ((uint32_t)tx_seq_restart_s * 1000u);
}

/*******************************************************************************
 * BOOT-FIRST-SEQUENCE ONE-SHOT (kns_app_uw_doppler.c MONITORING)
 ******************************************************************************/

enum { SWS_UNKNOWN = 0, SWS_SURFACE = 1, SWS_UNDERWATER = 2 };

static int boot_first_seq_pending;
static int surface_tx_pending;
static int wake_from_standby;

static void boot_first_seq_tick(int sws_state)
{
	if (boot_first_seq_pending && sws_state != SWS_UNKNOWN) {
		boot_first_seq_pending = 0;
		if (sws_state == SWS_SURFACE && !surface_tx_pending &&
		    !wake_from_standby) {
			reset_tx_scheduling();
			surface_tx_pending = 1;
		}
	}
}

/*******************************************************************************
 * TEST CASES
 ******************************************************************************/

/** All TXs of one sequence carry the same MC. */
void test_same_mc_within_sequence(void)
{
	lib_persisted_mc = 100;
	tx_count = 0;
	mc_boot_seed();

	uint16_t mc1 = mc_before_push(); lib_tx_complete(); tx_count++;
	uint16_t mc2 = mc_before_push(); lib_tx_complete(); tx_count++;
	uint16_t mc3 = mc_before_push(); lib_tx_complete(); tx_count++;

	ASSERT_EQ(100, mc1);
	ASSERT_EQ(100, mc2);
	ASSERT_EQ(100, mc3);
	TEST_PASS();
}

/** A new sequence bumps MC by exactly 1 (lib auto-increment, no double add). */
void test_new_sequence_increments_mc_once(void)
{
	lib_persisted_mc = 100;
	tx_count = 0;
	mc_boot_seed();

	/* Sequence 1: 3 TX */
	tx_send_one(); tx_send_one(); tx_send_one();
	ASSERT_EQ(101, lib_persisted_mc);  /* lib persisted session+1 */

	/* Sequence 2 (UW->SURF or seq-restart): snapshot picks up 101 */
	reset_tx_scheduling();
	uint16_t mc_seq2 = mc_before_push();
	ASSERT_EQ(101, mc_seq2);
	TEST_PASS();
}

/** Cold boot seeds session MC from the lib-persisted NVM value. */
void test_boot_seed_from_lib(void)
{
	lib_persisted_mc = 4242;
	mc_retained.session_mc = 0;
	mc_retained.initialised = 0;
	mc_boot_seed();
	ASSERT_EQ(4242, mc_retained.session_mc);
	ASSERT_EQ(1, mc_retained.initialised);
	TEST_PASS();
}

/** MC wraps at uint16 without UB. */
void test_mc_wrap_uint16(void)
{
	lib_persisted_mc = 0xFFFF;
	tx_count = 0;
	mc_boot_seed();
	tx_send_one();
	ASSERT_EQ(0, lib_persisted_mc);   /* 0xFFFF + 1 wraps */
	TEST_PASS();
}

/** Restart timer: fires only after cap reached AND delay elapsed. */
void test_seq_restart_needs_cap_and_delay(void)
{
	tx_seq_restart_s = 45;
	tx_max_count = 3;
	tx_count = 3;
	last_tx_tick = 10000;

	ASSERT_EQ(0, seq_restart_due(10000 + 44999));
	ASSERT_EQ(1, seq_restart_due(10000 + 45000));
	TEST_PASS();
}

/** Restart timer disabled when seq_restart_s == 0. */
void test_seq_restart_disabled(void)
{
	tx_seq_restart_s = 0;
	tx_max_count = 3;
	tx_count = 3;
	last_tx_tick = 10000;
	ASSERT_EQ(0, seq_restart_due(10000 + 999999));
	TEST_PASS();
}

/** Restart timer inert while the sequence is still running (under cap). */
void test_seq_restart_not_during_sequence(void)
{
	tx_seq_restart_s = 45;
	tx_max_count = 3;
	tx_count = 2;              /* sequence not finished */
	last_tx_tick = 10000;
	ASSERT_EQ(0, seq_restart_due(10000 + 100000));
	TEST_PASS();
}

/** Restart timer inert before any TX happened (last_tx_tick == 0). */
void test_seq_restart_needs_a_tx_first(void)
{
	tx_seq_restart_s = 45;
	tx_max_count = 3;
	tx_count = 3;
	last_tx_tick = 0;
	ASSERT_EQ(0, seq_restart_due(999999));
	TEST_PASS();
}

/** Boot at SURFACE starts a sequence (one-shot). */
void test_boot_at_surface_starts_sequence(void)
{
	boot_first_seq_pending = 1;
	surface_tx_pending = 0;
	wake_from_standby = 0;
	tx_count = 99;

	boot_first_seq_tick(SWS_SURFACE);
	ASSERT_EQ(1, surface_tx_pending);
	ASSERT_EQ(0, tx_count);           /* reset_tx_scheduling ran */
	ASSERT_EQ(0, boot_first_seq_pending);
	TEST_PASS();
}

/** UNKNOWN state does not consume the one-shot. */
void test_boot_oneshot_waits_for_known_state(void)
{
	boot_first_seq_pending = 1;
	surface_tx_pending = 0;
	wake_from_standby = 0;

	boot_first_seq_tick(SWS_UNKNOWN);
	ASSERT_EQ(1, boot_first_seq_pending);
	ASSERT_EQ(0, surface_tx_pending);
	TEST_PASS();
}

/** Boot UNDERWATER consumes the one-shot without TX (normal surfacing
 *  detection takes over later). */
void test_boot_underwater_no_sequence(void)
{
	boot_first_seq_pending = 1;
	surface_tx_pending = 0;
	wake_from_standby = 0;

	boot_first_seq_tick(SWS_UNDERWATER);
	ASSERT_EQ(0, boot_first_seq_pending);
	ASSERT_EQ(0, surface_tx_pending);
	TEST_PASS();
}

/** STANDBY duty-cycle wake must NOT fire a boot sequence. */
void test_standby_wake_no_boot_sequence(void)
{
	boot_first_seq_pending = 1;
	surface_tx_pending = 0;
	wake_from_standby = 1;

	boot_first_seq_tick(SWS_SURFACE);
	ASSERT_EQ(0, boot_first_seq_pending);
	ASSERT_EQ(0, surface_tx_pending);
	TEST_PASS();
}

/** Second consume is a no-op even at SURFACE (one-shot semantics). */
void test_boot_oneshot_is_single_use(void)
{
	boot_first_seq_pending = 1;
	surface_tx_pending = 0;
	wake_from_standby = 0;

	boot_first_seq_tick(SWS_SURFACE);
	surface_tx_pending = 0;            /* sequence consumed elsewhere */
	boot_first_seq_tick(SWS_SURFACE);  /* must not re-arm */
	ASSERT_EQ(0, surface_tx_pending);
	TEST_PASS();
}

/*******************************************************************************
 * MAIN
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER per-sequence MC + seq-restart timer");

	RUN_TEST(test_same_mc_within_sequence);
	RUN_TEST(test_new_sequence_increments_mc_once);
	RUN_TEST(test_boot_seed_from_lib);
	RUN_TEST(test_mc_wrap_uint16);
	RUN_TEST(test_seq_restart_needs_cap_and_delay);
	RUN_TEST(test_seq_restart_disabled);
	RUN_TEST(test_seq_restart_not_during_sequence);
	RUN_TEST(test_seq_restart_needs_a_tx_first);
	RUN_TEST(test_boot_at_surface_starts_sequence);
	RUN_TEST(test_boot_oneshot_waits_for_known_state);
	RUN_TEST(test_boot_underwater_no_sequence);
	RUN_TEST(test_standby_wake_no_boot_sequence);
	RUN_TEST(test_boot_oneshot_is_single_use);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
