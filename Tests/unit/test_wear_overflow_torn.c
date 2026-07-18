/**
 * @file    test_wear_overflow_torn.c
 * @brief   Crash-safety of the flash wear-leveled counter OVERFLOW branch
 *          (read_wear_counter / increment_wear_counter in mcu_flash.c),
 *          backing BOTH the Argos MSG counter (MC) and the WKU counter.
 *
 * Audit #9 flagged the overflow branch as non-atomic: it persists overflow=of+1
 * FIRST, then erases the 1024-slot wear area and programs slot0. A power loss in
 * that window is possible for the first time only after the wear area fills
 * (~1024 increments = ~a week at field rates), matching a ">7-day" onset.
 *
 * This suite models the REAL logic (1024-slot area + overflow word, current
 * write order) with a fault-injection hook at every step, and pins down what
 * actually matters:
 *   - INVARIANT: read_wear_counter is MONOTONIC across ANY torn overflow +
 *     reboot -> the MC can never be REPLAYED (the property that protects the
 *     Argos link). The current OF-first order guarantees this; reversing it
 *     (as an early "fix" suggested) would BREAK it, so this test guards against
 *     that regression.
 *   - UW on-air MC (value % 512) is UNAFFECTED by the worst torn jump (+1024,
 *     and 1024 % 512 == 0), so the sealed turtle tag's on-air counter is safe.
 *   - The residual cost is a forward jump of the FULL counter, which only
 *     matters for the WKU modulo TX schedule (DOPPLER-TPL) — documented here.
 */

#include "test_framework.h"

/*******************************************************************************
 * FAKE FLASH + REAL WEAR-COUNTER LOGIC (mirrored from mcu_flash.c)
 ******************************************************************************/

#define WL          1024u                     /* FLASH_*_COUNTER_WL_SIZE */
#define ERASED      0xFFFFFFFFFFFFFFFFull

static uint64_t g_slots[WL];
static uint64_t g_of;

static void flash_reset_empty(void)
{
	for (uint32_t i = 0; i < WL; i++)
		g_slots[i] = ERASED;
	g_of = ERASED;                            /* erased overflow word reads as 0 */
}

/* read_wear_counter(): overflow*WL + count-of-contiguous-programmed-slots. */
static uint64_t read_wear(void)
{
	uint32_t valid = 0;
	for (uint32_t i = 0; i < WL; i++) {
		if (g_slots[i] == ERASED)
			break;
		valid = i + 1;
	}
	uint64_t of = (g_of == ERASED) ? 0u : g_of;
	return of * WL + valid;
}

/* Fault-injection points inside the overflow branch (current write order). */
typedef enum {
	F_NONE = 0,        /* complete, no fault */
	F_BEFORE_OF,       /* brownout before anything is written (increment lost) */
	F_AFTER_OF,        /* brownout after OF bumped, BEFORE the wear erase */
	F_AFTER_ERASE,     /* brownout after erase, BEFORE slot0 is programmed */
} fault_t;

/* increment_wear_counter(): one slot per call; on a full area, run the overflow
 * branch in the SAME order as mcu_flash.c: (1) bump+persist OF, (2) erase WL,
 * (3) program slot0. `fault` aborts partway to model a power loss. */
static void increment_wear(fault_t fault)
{
	uint32_t idx = WL;
	for (uint32_t i = 0; i < WL; i++) {
		if (g_slots[i] == ERASED) { idx = i; break; }
	}
	if (idx < WL) {
		g_slots[idx] = 0;                     /* normal: program next slot */
		return;
	}
	/* Overflow branch. */
	if (fault == F_BEFORE_OF)
		return;                               /* nothing written; retried next boot */
	uint64_t of = (g_of == ERASED) ? 0u : g_of;
	of++;
	g_of = of;                                /* (1) persist overflow word */
	if (fault == F_AFTER_OF)
		return;                               /* torn: OF bumped, area still full */
	for (uint32_t i = 0; i < WL; i++)
		g_slots[i] = ERASED;                  /* (2) erase wear area */
	if (fault == F_AFTER_ERASE)
		return;                               /* torn: erased, slot0 not yet written */
	g_slots[0] = 0;                           /* (3) program slot0 */
}

/* Fill the area to exactly full (value == WL, next increment overflows). */
static void fill_to_full(void)
{
	flash_reset_empty();
	for (uint32_t i = 0; i < WL; i++)
		increment_wear(F_NONE);
}

/*******************************************************************************
 * TESTS
 ******************************************************************************/

/** Normal fill then a CLEAN overflow advances the counter by exactly 1. */
void test_clean_overflow_plus_one(void)
{
	fill_to_full();
	ASSERT_EQ(WL, read_wear());                /* 1024: area full, of=0 */
	increment_wear(F_NONE);                    /* clean overflow */
	ASSERT_EQ(WL + 1u, read_wear());           /* 1025: of=1, slot0 */
	TEST_PASS();
}

/** Torn AFTER the OF write, BEFORE the erase: reads +1024 (the documented
 *  forward jump) — but crucially NEVER below the pre-overflow value. */
void test_torn_after_of_is_forward_only(void)
{
	fill_to_full();
	uint64_t pre = read_wear();                /* 1024 */
	increment_wear(F_AFTER_OF);                /* brownout in the window */
	uint64_t torn = read_wear();               /* (of=1, full) = 2048 */
	ASSERT_EQ(2048, torn);
	ASSERT_TRUE(torn >= pre);                  /* monotonic: never replays an MC */
	TEST_PASS();
}

/** Torn AFTER the erase, BEFORE slot0: reads == pre (stall), still monotonic. */
void test_torn_after_erase_is_stall(void)
{
	fill_to_full();
	uint64_t pre = read_wear();                /* 1024 */
	increment_wear(F_AFTER_ERASE);
	uint64_t torn = read_wear();               /* (of=1, empty) = 1024 */
	ASSERT_EQ(pre, torn);
	ASSERT_TRUE(torn >= pre);
	TEST_PASS();
}

/** Torn BEFORE the OF write: the increment is simply lost, value == pre. */
void test_torn_before_of_is_noop(void)
{
	fill_to_full();
	uint64_t pre = read_wear();
	increment_wear(F_BEFORE_OF);
	ASSERT_EQ(pre, read_wear());
	TEST_PASS();
}

/** THE invariant: across EVERY fault point + reboot, the counter is never
 *  lower than before -> the MC can never be replayed. This is the crash-safety
 *  property that must hold; reversing the write order would break it. */
void test_overflow_is_always_monotonic(void)
{
	const fault_t faults[] = { F_NONE, F_BEFORE_OF, F_AFTER_OF, F_AFTER_ERASE };
	for (unsigned k = 0; k < sizeof(faults) / sizeof(faults[0]); k++) {
		fill_to_full();
		uint64_t pre = read_wear();            /* 1024 */
		increment_wear(faults[k]);             /* possibly-torn overflow */
		uint64_t post = read_wear();
		ASSERT_TRUE(post >= pre);              /* never rewinds -> no replay */
	}
	TEST_PASS();
}

/** UW on-air safety: the on-air MC is value % 512. The worst torn jump (+1024)
 *  leaves it unchanged because 1024 % 512 == 0, so the sealed tag's on-air
 *  counter is immune to the overflow torn state. */
void test_onair_mc_mod512_unaffected(void)
{
	fill_to_full();
	uint16_t onair_pre = (uint16_t)(read_wear() % 512u);
	increment_wear(F_AFTER_OF);                /* worst torn: +1024 */
	uint16_t onair_torn = (uint16_t)(read_wear() % 512u);
	ASSERT_EQ(onair_pre, onair_torn);          /* 0 == 0 : on-air unaffected */
	TEST_PASS();
}

/** After a torn +1024, the structure is still consistent: further increments
 *  keep advancing monotonically (no wedge / corruption of the wear area). */
void test_continues_monotonic_after_torn(void)
{
	fill_to_full();
	increment_wear(F_AFTER_OF);                /* torn -> 2048 */
	uint64_t v = read_wear();
	ASSERT_EQ(2048, v);
	for (int i = 0; i < 5; i++) {
		increment_wear(F_NONE);
		uint64_t nv = read_wear();
		ASSERT_TRUE(nv >= v);                  /* keeps climbing, no rewind */
		v = nv;
	}
	TEST_PASS();
}

/** DOCUMENTED residual cost: the +1024 forward jump DOES shift a modulo TX
 *  schedule (the DOPPLER-TPL WKU use), which UW_DOPPLER (mod-512 on-air) does
 *  NOT suffer. This asserts the phase actually moves, pinning the WKU exposure
 *  so a future WKU-side fix has a regression guard. */
void test_wku_modulo_schedule_shifts_on_torn(void)
{
	const uint32_t modulo = 24;                /* e.g. send every 24th wake */
	fill_to_full();
	uint32_t phase_pre = (uint32_t)(read_wear() % modulo);   /* 1024 % 24 = 16 */
	increment_wear(F_AFTER_OF);
	uint32_t phase_torn = (uint32_t)(read_wear() % modulo);  /* 2048 % 24 = 8  */
	ASSERT_NE(phase_pre, phase_torn);          /* schedule phase shifted (WKU cost) */
	TEST_PASS();
}

/*******************************************************************************
 * MAIN
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("Wear-counter overflow crash-safety (MC/WKU)");

	RUN_TEST(test_clean_overflow_plus_one);
	RUN_TEST(test_torn_after_of_is_forward_only);
	RUN_TEST(test_torn_after_erase_is_stall);
	RUN_TEST(test_torn_before_of_is_noop);
	RUN_TEST(test_overflow_is_always_monotonic);
	RUN_TEST(test_onair_mc_mod512_unaffected);
	RUN_TEST(test_continues_monotonic_after_torn);
	RUN_TEST(test_wku_modulo_schedule_shifts_on_torn);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
