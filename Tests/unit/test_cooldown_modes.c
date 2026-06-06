/**
 * @file    test_cooldown_modes.c
 * @brief   Unit tests for Phase 4C: Post-TX cooldown trigger modes
 *
 * Three modes:
 *   AT_LAST_TX    (0) = cooldown stamped on every TX_DONE (default)
 *   AFTER_DIVE    (1) = cooldown stamped when entering UNDERWATER
 *   AFTER_BURST   (2) = cooldown stamped when surface burst max reached
 *
 * Verifies the gating logic decides correctly whether to TX, by
 * faithfully reproducing the bool/timing logic from kns_app_uw_doppler.c
 * without the rest of the firmware.
 */

#include "test_framework.h"

/* Mirror of the relevant enum + cfg fields */
typedef enum {
	COOLDOWN_AT_LAST_TX  = 0,
	COOLDOWN_AFTER_DIVE  = 1,
	COOLDOWN_AFTER_BURST = 2,
} CooldownTrigger_t;

#define MIN_INTER_TX_INTERVAL_MS  5000u

/* Mirror of cooldown gate logic from kns_app_uw_doppler.c */
static bool cooldown_gate_ok(uint32_t now_tick,
                              uint32_t cooldown_ref_tick,
                              uint32_t last_tx_tick,
                              uint32_t cooldown_ms,
                              uint32_t first_tx_random_offset_ms,
                              uint8_t  cooldown_trigger,
                              bool     lb_active,
                              uint16_t tx_initial_interval_s)
{
	uint32_t since_last_tx = now_tick - last_tx_tick;
	uint32_t since_cooldown_ref = now_tick - cooldown_ref_tick;
	uint32_t configured_min_ms = (uint32_t)tx_initial_interval_s * 1000u;
	uint32_t first_tx_floor_ms = MIN_INTER_TX_INTERVAL_MS;
	if (configured_min_ms > first_tx_floor_ms)
		first_tx_floor_ms = configured_min_ms;

	bool enforce_cooldown_on_first =
		lb_active ||
		cooldown_trigger == COOLDOWN_AFTER_DIVE ||
		cooldown_trigger == COOLDOWN_AFTER_BURST;
	bool cooldown_ok = !enforce_cooldown_on_first
		|| cooldown_ms == 0
		|| since_cooldown_ref >= cooldown_ms;

	uint32_t first_tx_min_ms = first_tx_floor_ms + first_tx_random_offset_ms;
	return cooldown_ok && since_last_tx >= first_tx_min_ms;
}

/* AT_LAST_TX: cooldown gate bypassed for first-TX-of-surface, only the
 * inter-TX floor matters. */
void test_at_last_tx_bypasses_cooldown(void)
{
	TEST_START("AT_LAST_TX bypasses cooldown for first-TX");
	/* 30 s since last_tx, cooldown=60s -> with AT_LAST_TX, gate OK
	 * because inter-TX floor (5s) is met and cooldown is ignored. */
	ASSERT_TRUE(cooldown_gate_ok(
		/* now */            30000u,
		/* cooldown_ref */   0u,
		/* last_tx */        0u,
		/* cooldown_ms */    60000u,
		/* random_offset */  0u,
		COOLDOWN_AT_LAST_TX,
		/* lb_active */      false,
		/* initial_s */      10));
	TEST_PASS();
}

/* AFTER_DIVE: cooldown enforced on first-TX-of-surface. If
 * since_cooldown_ref < cooldown_ms, gate is BLOCKED. */
void test_after_dive_enforces_cooldown(void)
{
	TEST_START("AFTER_DIVE enforces cooldown");
	ASSERT_FALSE(cooldown_gate_ok(
		/* now */            30000u,   /* 30s since dive */
		/* cooldown_ref */   0u,
		/* last_tx */        0u,
		/* cooldown_ms */    60000u,   /* 60s cooldown */
		/* random_offset */  0u,
		COOLDOWN_AFTER_DIVE,
		false, 10));
	TEST_PASS();
}

/* AFTER_DIVE: once cooldown elapsed, gate OK. */
void test_after_dive_unblocks_after_cooldown(void)
{
	TEST_START("AFTER_DIVE unblocks after cooldown");
	ASSERT_TRUE(cooldown_gate_ok(
		/* now */            70000u,   /* 70s since dive */
		/* cooldown_ref */   0u,
		/* last_tx */        0u,
		/* cooldown_ms */    60000u,
		/* random_offset */  0u,
		COOLDOWN_AFTER_DIVE,
		false, 10));
	TEST_PASS();
}

/* AFTER_BURST: same gating semantics as AFTER_DIVE */
void test_after_burst_enforces_cooldown(void)
{
	TEST_START("AFTER_BURST enforces cooldown");
	ASSERT_FALSE(cooldown_gate_ok(
		15000u, 0u, 0u, 60000u, 0u,
		COOLDOWN_AFTER_BURST, false, 10));
	TEST_PASS();
}

/* LB mode forces cooldown even with AT_LAST_TX trigger */
void test_lb_active_forces_cooldown(void)
{
	TEST_START("LB mode forces cooldown even on AT_LAST_TX");
	ASSERT_FALSE(cooldown_gate_ok(
		15000u, 0u, 0u, 60000u, 0u,
		COOLDOWN_AT_LAST_TX, /* lb_active */ true, 10));
	TEST_PASS();
}

/* cooldown_ms = 0 disables the cooldown entirely */
void test_cooldown_zero_always_passes(void)
{
	TEST_START("cooldown_ms=0 always passes");
	/* AFTER_DIVE with cooldown=0 -> gate OK (only inter-TX floor) */
	ASSERT_TRUE(cooldown_gate_ok(
		6000u, 0u, 0u, 0u, 0u,
		COOLDOWN_AFTER_DIVE, false, 5));
	TEST_PASS();
}

/* Inter-TX floor blocks even with cooldown OK */
void test_inter_tx_floor_blocks(void)
{
	TEST_START("Inter-TX floor blocks under MIN_INTER_TX_INTERVAL_MS");
	/* Only 1s since last_tx but cooldown OK -> still blocked by 5s floor */
	ASSERT_FALSE(cooldown_gate_ok(
		1000u, 0u, 0u, 0u, 0u,
		COOLDOWN_AT_LAST_TX, false, 10));
	TEST_PASS();
}

/* Random offset gets added to the floor */
void test_random_offset_adds_to_floor(void)
{
	TEST_START("Random offset adds to first-TX floor");
	/* floor 10s + 1.5s random = 11.5s -> 10s since_last_tx is NOT enough */
	ASSERT_FALSE(cooldown_gate_ok(
		10500u, 0u, 0u, 0u, 1500u,
		COOLDOWN_AT_LAST_TX, false, 10));
	/* But 12s is */
	ASSERT_TRUE(cooldown_gate_ok(
		12000u, 0u, 0u, 0u, 1500u,
		COOLDOWN_AT_LAST_TX, false, 10));
	TEST_PASS();
}

int main(void)
{
	printf("=== Cooldown trigger mode tests (Phase 4C) ===\n");

	test_at_last_tx_bypasses_cooldown();
	test_after_dive_enforces_cooldown();
	test_after_dive_unblocks_after_cooldown();
	test_after_burst_enforces_cooldown();
	test_lb_active_forces_cooldown();
	test_cooldown_zero_always_passes();
	test_inter_tx_floor_blocks();
	test_random_offset_adds_to_floor();

	printf("\n%d/%d tests passed (%d failed)\n",
	       tests_passed, tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
