/**
 * @file    test_lb_hysteresis.c
 * @brief   Low-battery (LB) mode hysteresis. Mirrors lb_update()
 *          (kns_app_uw_doppler.c ~3234) and the LB cadence/cap selection.
 *
 * Contract:
 *  - lb_enter_mV == 0 => LB disabled, never active.
 *  - enter when (!active && bat>0 && bat < lb_enter_mV).
 *  - exit only when (active && bat > lb_exit_mV)  [exit > enter => no flap].
 *  - bat == 0 (ADC failure) never CHANGES the latch (a failed read must not
 *    force LB on or off).
 *  - when active, TX cadence/cap switch to the LB variants; TX is NEVER blocked.
 */

#include "test_framework.h"

/* ---- Mirrored LB config + state ---- */
typedef struct {
	uint16_t lb_enter_mV;
	uint16_t lb_exit_mV;
	uint16_t lb_tx_interval_s;
	uint16_t lb_tx_max_s;
	uint8_t  lb_tx_max_count;
} LbCfg;

typedef struct {
	uint16_t tx_initial_interval_s;
	uint16_t tx_max_interval_s;
	uint8_t  tx_max_count;
} TxCfg;

static LbCfg lb_cfg;
static TxCfg tx_cfg;
static bool  lb_active;
static int   enter_events, exit_events;   /* EVT_LB_ENTER / EVT_LB_EXIT stand-ins */

/* Mirror of lb_update() (kns_app_uw_doppler.c:3234-3252). */
static bool lb_update(uint16_t bat_mV)
{
	if (lb_cfg.lb_enter_mV == 0) {
		lb_active = false;
		return false;
	}
	if (!lb_active && bat_mV > 0 && bat_mV < lb_cfg.lb_enter_mV) {
		lb_active = true;
		enter_events++;
	} else if (lb_active && bat_mV > lb_cfg.lb_exit_mV) {
		lb_active = false;
		exit_events++;
	}
	return lb_active;
}

/* The "mode change": which cap/interval the scheduler uses (mirror of the
 * lb_active ? lb_* : tx_* selection at the TX gate + compute_next_interval_ms). */
static uint8_t  active_cap(void)      { return lb_active ? lb_cfg.lb_tx_max_count : tx_cfg.tx_max_count; }
static uint16_t active_interval_s(void){ return lb_active ? lb_cfg.lb_tx_interval_s : tx_cfg.tx_initial_interval_s; }

static void setup(void)
{
	lb_cfg.lb_enter_mV = 2900; lb_cfg.lb_exit_mV = 3100;
	lb_cfg.lb_tx_interval_s = 60; lb_cfg.lb_tx_max_s = 600; lb_cfg.lb_tx_max_count = 3;
	tx_cfg.tx_initial_interval_s = 10; tx_cfg.tx_max_interval_s = 180; tx_cfg.tx_max_count = 3;
	lb_active = false;
	enter_events = exit_events = 0;
}

/* ---- Tests ---- */

/** Disabled by default (enter==0): never activates whatever the voltage. */
void test_disabled_never_activates(void)
{
	setup();
	lb_cfg.lb_enter_mV = 0;               /* default */
	lb_update(2000);                      /* very low */
	ASSERT_FALSE(lb_active);
	lb_update(4000);
	ASSERT_FALSE(lb_active);
	ASSERT_EQ(0, enter_events);
	TEST_PASS();
}

/** Enters below lb_enter_mV, and switches the cadence/cap (the mode change). */
void test_enter_switches_mode(void)
{
	setup();
	ASSERT_EQ((long)tx_cfg.tx_initial_interval_s, (long)active_interval_s());  /* normal */
	lb_update(2850);                      /* < 2900 */
	ASSERT_TRUE(lb_active);
	ASSERT_EQ((long)lb_cfg.lb_tx_interval_s, (long)active_interval_s());       /* slowed */
	ASSERT_EQ((long)lb_cfg.lb_tx_max_count, (long)active_cap());
	TEST_PASS();
}

/** Hysteresis: once active, a reading between enter and exit does NOT exit;
 *  only a reading above lb_exit_mV clears LB. No flapping. */
void test_hysteresis_band_holds(void)
{
	setup();
	lb_update(2850); ASSERT_TRUE(lb_active);      /* enter */
	lb_update(2950); ASSERT_TRUE(lb_active);      /* in band (2900..3100) -> hold */
	lb_update(3050); ASSERT_TRUE(lb_active);      /* still in band -> hold */
	lb_update(3150); ASSERT_FALSE(lb_active);     /* above exit -> leave */
	ASSERT_EQ(1, enter_events);
	ASSERT_EQ(1, exit_events);
	TEST_PASS();
}

/** A pack oscillating around lb_enter_mV must not flap the mode. */
void test_no_flap_around_enter(void)
{
	setup();
	lb_update(2850); ASSERT_TRUE(lb_active);      /* enter once */
	for (int i = 0; i < 20; i++) {
		lb_update(2850);                          /* below enter */
		lb_update(2950);                          /* in band */
		ASSERT_TRUE(lb_active);                   /* never exits in the band */
	}
	ASSERT_EQ(1, enter_events);                   /* entered exactly once */
	ASSERT_EQ(0, exit_events);                    /* never flapped out */
	TEST_PASS();
}

/** ADC failure (bat==0) never changes the latch either way. */
void test_adc_fail_preserves_state(void)
{
	setup();
	lb_update(0);  ASSERT_FALSE(lb_active);       /* was inactive -> stays */
	lb_update(2850); ASSERT_TRUE(lb_active);      /* enter */
	lb_update(0);  ASSERT_TRUE(lb_active);        /* failed read -> stays active */
	ASSERT_EQ(1, enter_events);
	ASSERT_EQ(0, exit_events);
	TEST_PASS();
}

/** LB never blocks TX: the mode only reshapes cadence/cap. Even active, the cap
 *  is > 0 (TXs still happen) — there is no "0 TX" inhibit. */
void test_lb_never_blocks_tx(void)
{
	setup();
	lb_update(2850);
	ASSERT_TRUE(lb_active);
	ASSERT_TRUE(active_cap() > 0);                /* still transmits, just capped */
	ASSERT_TRUE(active_interval_s() >= tx_cfg.tx_initial_interval_s); /* slower, not off */
	TEST_PASS();
}

/** Full sag-then-recover cycle: enter on the way down, hold through the band,
 *  exit only on genuine recovery, back to normal cadence. */
void test_full_sag_recover_cycle(void)
{
	setup();
	uint16_t profile[] = { 3300, 3050, 2950, 2880, 2850, 2900, 3000, 3090, 3120, 3300 };
	int active_seen = 0;
	for (unsigned i = 0; i < sizeof(profile)/sizeof(profile[0]); i++) {
		lb_update(profile[i]);
		if (lb_active) active_seen++;
	}
	ASSERT_FALSE(lb_active);                      /* recovered by the end (3300>3100) */
	ASSERT_EQ(1, enter_events);
	ASSERT_EQ(1, exit_events);
	ASSERT_TRUE(active_seen > 0);                 /* was active during the sag */
	ASSERT_EQ((long)tx_cfg.tx_initial_interval_s, (long)active_interval_s()); /* normal again */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("LB mode hysteresis");

	RUN_TEST(test_disabled_never_activates);
	RUN_TEST(test_enter_switches_mode);
	RUN_TEST(test_hysteresis_band_holds);
	RUN_TEST(test_no_flap_around_enter);
	RUN_TEST(test_adc_fail_preserves_state);
	RUN_TEST(test_lb_never_blocks_tx);
	RUN_TEST(test_full_sag_recover_cycle);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
