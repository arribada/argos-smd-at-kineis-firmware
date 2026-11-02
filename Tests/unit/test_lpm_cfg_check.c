/**
 * @file    test_lpm_cfg_check.c
 * @brief   Stale-SRAM2 detection at boot (Kineis/Lpm/Inc/lpm_cfg_check.h, real code).
 *
 * lpm_config (bitmap + LPM callbacks) lives in SRAM2, which survives resets.
 * After a reflash or a DFU that followed an LPM entry, the new image must not
 * run the previous image's callbacks: the boot cold-starts SRAM2 instead.
 */

#include "test_framework.h"
#include "lpm_cfg_check.h"

/* Two images' worth of LPM callbacks. */
static void a_sleep_enter(void) {}
static void a_sleep_exit(void) {}
static void a_stop_enter(void) {}
static void a_stop_exit(void) {}
static void a_standby_enter(void) {}
static void a_shutdown_enter(void) {}
static void b_stop_enter(void) {}

static const struct MgrLpm_EnvConfig_t image_a = {
	.allowedLPMbitmap  = LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP,
	.fp_sleep_enter    = a_sleep_enter,
	.fp_sleep_exit     = a_sleep_exit,
	.fp_stop_enter     = a_stop_enter,
	.fp_stop_exit      = a_stop_exit,
	.fp_standby_enter  = a_standby_enter,
	.fp_shutdown_enter = NULL,
};

static void test_same_image_kept(void)
{
	TEST_START("same image: callbacks match, host-changed bitmap is kept");
	struct MgrLpm_EnvConfig_t ram = image_a;
	ram.allowedLPMbitmap = LOW_POWER_MODE_SLEEP;   /* host capped it (AT+LPM / WRITE_LPM) */
	ASSERT_TRUE(LPM_cfgCallbacksMatch(&ram, &image_a));
	ASSERT_FALSE(LPM_sram2Stale(LOW_POWER_MODE_STOP, LPM_cfgCallbacksMatch(&ram, &image_a)));
	ASSERT_FALSE(LPM_sram2Stale(LOW_POWER_MODE_STANDBY, true));
	TEST_PASS();
}

static void test_any_moved_callback_is_stale(void)
{
	TEST_START("any callback from another image is detected");
	void (*const other)(void) = b_stop_enter;
	for (int field = 0; field < 6; field++) {
		struct MgrLpm_EnvConfig_t ram = image_a;
		switch (field) {
		case 0: ram.fp_sleep_enter = other; break;
		case 1: ram.fp_sleep_exit = other; break;
		case 2: ram.fp_stop_enter = other; break;
		case 3: ram.fp_stop_exit = other; break;
		case 4: ram.fp_standby_enter = other; break;
		default: ram.fp_shutdown_enter = other; break;
		}
		ASSERT_FALSE(LPM_cfgCallbacksMatch(&ram, &image_a));
	}
	TEST_PASS();
}

static void test_shutdown_callback_presence(void)
{
	TEST_START("LPM=SHUTDOWN image over a STOP image (shutdown callback NULL vs set)");
	struct MgrLpm_EnvConfig_t ram = image_a;
	ram.fp_shutdown_enter = a_shutdown_enter;
	ASSERT_FALSE(LPM_cfgCallbacksMatch(&ram, &image_a));
	TEST_PASS();
}

static void test_boot_decision(void)
{
	TEST_START("boot: cold-start SRAM2 on mismatch after NONE/SLEEP/STOP/STANDBY");
	ASSERT_TRUE(LPM_sram2Stale(LOW_POWER_MODE_NONE, false));     /* POR: random SRAM2 */
	ASSERT_TRUE(LPM_sram2Stale(LOW_POWER_MODE_SLEEP, false));
	ASSERT_TRUE(LPM_sram2Stale(LOW_POWER_MODE_STOP, false));     /* reflash/DFU after STOP */
	ASSERT_TRUE(LPM_sram2Stale(LOW_POWER_MODE_STANDBY, false));
	TEST_PASS();
}

static void test_shutdown_wake_untouched(void)
{
	TEST_START("SHUTDOWN wake: SRAM2 is lost but the boot re-inits it, mode kept");
	ASSERT_FALSE(LPM_sram2Stale(LOW_POWER_MODE_SHUTDOWN, false));
	ASSERT_FALSE(LPM_sram2Stale(LOW_POWER_MODE_SHUTDOWN, true));
	TEST_PASS();
}

int main(void)
{
	printf("\n=== Stale SRAM2 lpm_config at boot ===\n");
	test_same_image_kept();
	test_any_moved_callback_is_stale();
	test_shutdown_callback_presence();
	test_boot_decision();
	test_shutdown_wake_untouched();

	printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
	return tests_failed ? 1 : 0;
}
