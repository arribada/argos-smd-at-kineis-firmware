/**
 * @file    test_tx_interval.c
 * @brief   Unit tests for UW_DOPPLER TX interval computation
 *
 * Tests the exponential backoff TX scheduling:
 *   T_n = T_initial * (1 + growth/100)^n
 *
 * Verifies correct interval growth, max capping, overflow protection
 * (uint64_t intermediate), and edge cases.
 */

#include "test_framework.h"

/*******************************************************************************
 * TX CONFIG (from kns_app_uw_doppler.h)
 ******************************************************************************/

typedef struct {
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint16_t tx_max_interval_s;
	uint8_t  tx_max_count;
} TxCfg_t;

/* Module-level config (like the static in kns_app_uw_doppler.c) */
static TxCfg_t tx_cfg;

/*******************************************************************************
 * FUNCTION UNDER TEST (copied from kns_app_uw_doppler.c)
 ******************************************************************************/

static uint32_t compute_next_interval_ms(uint32_t n)
{
	uint32_t interval_ms = (uint32_t)tx_cfg.tx_initial_interval_s * 1000;
	uint32_t max_ms = (uint32_t)tx_cfg.tx_max_interval_s * 1000;
	for (uint32_t i = 0; i < n; i++) {
		uint64_t next = (uint64_t)interval_ms * (100 + tx_cfg.tx_growth_percent) / 100;
		if (next >= max_ms) {
			interval_ms = max_ms;
			break;
		}
		interval_ms = (uint32_t)next;
	}
	return interval_ms;
}

/*******************************************************************************
 * TEST CASES
 ******************************************************************************/

/** n=0: returns initial interval (no growth applied) */
void test_n0_returns_initial(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 50;
	tx_cfg.tx_max_interval_s = 600;

	ASSERT_EQ(10000, compute_next_interval_ms(0));
	TEST_PASS();
}

/** n=1: initial * (100+growth)/100 */
void test_n1_single_growth(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 10;
	tx_cfg.tx_max_interval_s = 600;

	/* 10000 * 110 / 100 = 11000 */
	ASSERT_EQ(11000, compute_next_interval_ms(1));
	TEST_PASS();
}

/** n=2: two growth steps */
void test_n2_double_growth(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 10;
	tx_cfg.tx_max_interval_s = 600;

	/* 10000 * 1.1 * 1.1 = 12100 */
	ASSERT_EQ(12100, compute_next_interval_ms(2));
	TEST_PASS();
}

/** growth=0: interval stays constant regardless of n */
void test_zero_growth(void)
{
	tx_cfg.tx_initial_interval_s = 30;
	tx_cfg.tx_growth_percent = 0;
	tx_cfg.tx_max_interval_s = 600;

	ASSERT_EQ(30000, compute_next_interval_ms(0));
	ASSERT_EQ(30000, compute_next_interval_ms(1));
	ASSERT_EQ(30000, compute_next_interval_ms(10));
	ASSERT_EQ(30000, compute_next_interval_ms(100));
	TEST_PASS();
}

/** Large n: interval caps at max */
void test_caps_at_max(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 50;
	tx_cfg.tx_max_interval_s = 60;

	/* 10s * 1.5^n should quickly exceed 60s max */
	uint32_t result = compute_next_interval_ms(100);
	ASSERT_EQ(60000, result);
	TEST_PASS();
}

/** Verify growth sequence with 50% growth */
void test_growth_sequence_50pct(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 50;
	tx_cfg.tx_max_interval_s = 600;

	ASSERT_EQ(10000, compute_next_interval_ms(0));  /* 10s */
	ASSERT_EQ(15000, compute_next_interval_ms(1));  /* 15s */
	ASSERT_EQ(22500, compute_next_interval_ms(2));  /* 22.5s */
	ASSERT_EQ(33750, compute_next_interval_ms(3));  /* 33.75s */
	TEST_PASS();
}

/** Exact cap: when result exactly equals max */
void test_exact_cap(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 100;  /* Double each time */
	tx_cfg.tx_max_interval_s = 40;

	/* n=0: 10s, n=1: 20s, n=2: 40s (=max), n=3: still 40s */
	ASSERT_EQ(10000, compute_next_interval_ms(0));
	ASSERT_EQ(20000, compute_next_interval_ms(1));
	ASSERT_EQ(40000, compute_next_interval_ms(2));
	ASSERT_EQ(40000, compute_next_interval_ms(3));
	TEST_PASS();
}

/** Overflow protection: large initial * large growth with uint64 intermediate */
void test_overflow_protection(void)
{
	tx_cfg.tx_initial_interval_s = 65535;  /* Max uint16 */
	tx_cfg.tx_growth_percent = 255;        /* Max uint8 */
	tx_cfg.tx_max_interval_s = 65535;

	/* Without uint64, 65535000 * 355 would overflow uint32.
	 * With uint64, it's capped to max (65535000). */
	uint32_t result = compute_next_interval_ms(1);
	/* 65535 * 1000 * (100+255) / 100 = 65535000 * 355 / 100 = 232649250
	 * But max = 65535*1000 = 65535000, so capped */
	ASSERT_EQ(65535000, result);
	TEST_PASS();
}

/** Small growth percentage: 1% */
void test_one_percent_growth(void)
{
	tx_cfg.tx_initial_interval_s = 100;
	tx_cfg.tx_growth_percent = 1;
	tx_cfg.tx_max_interval_s = 600;

	/* 100000 * 101 / 100 = 101000 */
	ASSERT_EQ(101000, compute_next_interval_ms(1));
	/* 101000 * 101 / 100 = 102010 */
	ASSERT_EQ(102010, compute_next_interval_ms(2));
	TEST_PASS();
}

/** Initial interval = 1s (minimum practical) */
void test_one_second_initial(void)
{
	tx_cfg.tx_initial_interval_s = 1;
	tx_cfg.tx_growth_percent = 100;
	tx_cfg.tx_max_interval_s = 600;

	ASSERT_EQ(1000,  compute_next_interval_ms(0));
	ASSERT_EQ(2000,  compute_next_interval_ms(1));
	ASSERT_EQ(4000,  compute_next_interval_ms(2));
	ASSERT_EQ(8000,  compute_next_interval_ms(3));
	ASSERT_EQ(16000, compute_next_interval_ms(4));
	TEST_PASS();
}

/** Default config: 10s initial, 10% growth, 600s max */
void test_default_config(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 10;
	tx_cfg.tx_max_interval_s = 600;

	uint32_t prev = compute_next_interval_ms(0);
	ASSERT_EQ(10000, prev);

	/* Verify monotonically increasing */
	for (uint32_t n = 1; n < 50; n++) {
		uint32_t cur = compute_next_interval_ms(n);
		ASSERT_TRUE(cur >= prev);
		ASSERT_TRUE(cur <= 600000);
		prev = cur;
	}

	/* Should eventually cap at max */
	ASSERT_EQ(600000, compute_next_interval_ms(100));
	TEST_PASS();
}

/** High n values don't cause infinite loop or excessive runtime */
void test_high_n_terminates(void)
{
	tx_cfg.tx_initial_interval_s = 10;
	tx_cfg.tx_growth_percent = 10;
	tx_cfg.tx_max_interval_s = 600;

	/* Once capped, the break statement exits the loop early */
	uint32_t result = compute_next_interval_ms(10000);
	ASSERT_EQ(600000, result);
	TEST_PASS();
}

/** growth=100%: interval doubles each step */
void test_doubling(void)
{
	tx_cfg.tx_initial_interval_s = 5;
	tx_cfg.tx_growth_percent = 100;
	tx_cfg.tx_max_interval_s = 600;

	ASSERT_EQ(5000,   compute_next_interval_ms(0));
	ASSERT_EQ(10000,  compute_next_interval_ms(1));
	ASSERT_EQ(20000,  compute_next_interval_ms(2));
	ASSERT_EQ(40000,  compute_next_interval_ms(3));
	ASSERT_EQ(80000,  compute_next_interval_ms(4));
	ASSERT_EQ(160000, compute_next_interval_ms(5));
	ASSERT_EQ(320000, compute_next_interval_ms(6));
	ASSERT_EQ(600000, compute_next_interval_ms(7));  /* Capped */
	TEST_PASS();
}

/** When initial > max, first call still returns initial (n=0) */
void test_initial_greater_than_max(void)
{
	tx_cfg.tx_initial_interval_s = 100;
	tx_cfg.tx_growth_percent = 10;
	tx_cfg.tx_max_interval_s = 50;

	/* n=0 returns initial regardless (no growth applied) */
	ASSERT_EQ(100000, compute_next_interval_ms(0));
	/* n=1 should cap at max since result > max */
	ASSERT_EQ(50000, compute_next_interval_ms(1));
	TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("TX Interval Computation Tests");

	RUN_TEST(test_n0_returns_initial);
	RUN_TEST(test_n1_single_growth);
	RUN_TEST(test_n2_double_growth);
	RUN_TEST(test_zero_growth);
	RUN_TEST(test_caps_at_max);
	RUN_TEST(test_growth_sequence_50pct);
	RUN_TEST(test_exact_cap);
	RUN_TEST(test_overflow_protection);
	RUN_TEST(test_one_percent_growth);
	RUN_TEST(test_one_second_initial);
	RUN_TEST(test_default_config);
	RUN_TEST(test_high_n_terminates);
	RUN_TEST(test_doubling);
	RUN_TEST(test_initial_greater_than_max);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
