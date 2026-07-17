/**
 * @file    test_mgr_err_bkp_map.c
 * @brief   MGR_ERR TAMP backup registers never alias another owner.
 *
 * Owners of the TAMP backup registers (BKPnR at 0x4000B100 + 4n):
 *  - BKP0R-BKP5R: linker .rtc_bkpr (mcu_tim timer[], lpm_ctxt, Argos MC;
 *    0x18 bytes in the current map). BKP0R is also the bootloader DFU flag.
 *  - BKP10R: MGR_GESTURE persisted mode.
 *  - BKP11R: MGR_LPM_UW soft-off wake magic.
 * MGR_ERR used BKP2R-BKP7R, so MGR_ERR_log() overwrote the live MC and
 * lpm_ctxt at runtime. It now uses BKP12R-BKP17R, plus BKP6R for the
 * fault-streak counter.
 */

#include "test_framework.h"
#include "mgr_err.h"

#define TAMP_BKP_BASE      0x4000B100UL
#define BKP(n)             (TAMP_BKP_BASE + 4UL * (n))
#define LINKER_BKPR_END    BKP(6)   /* .rtc_bkpr end (_ertc_bkpr) */

static const uint32_t err_regs[] = {
	TAMP_BKP12R_ADDR, TAMP_BKP13R_ADDR, TAMP_BKP14R_ADDR,
	TAMP_BKP15R_ADDR, TAMP_BKP16R_ADDR, TAMP_BKP17R_ADDR,
};
#define N_ERR_REGS (sizeof(err_regs) / sizeof(err_regs[0]))

static void test_addresses_are_bkp12_to_17(void)
{
	size_t i;

	TEST_START("MGR_ERR uses BKP12R..BKP17R, contiguous");
	for (i = 0; i < N_ERR_REGS; i++)
		ASSERT_EQ_HEX(BKP(12 + i), err_regs[i]);
	TEST_PASS();
}

static void test_no_alias_with_other_owners(void)
{
	size_t i;

	TEST_START("No overlap with .rtc_bkpr, MGR_GESTURE or MGR_LPM_UW");
	for (i = 0; i < N_ERR_REGS; i++) {
		ASSERT_TRUE(err_regs[i] >= LINKER_BKPR_END);
		ASSERT_TRUE(err_regs[i] != BKP(10));
		ASSERT_TRUE(err_regs[i] != BKP(11));
		ASSERT_TRUE(err_regs[i] <= BKP(31));
	}
	TEST_PASS();
}

static void test_fault_streak_register(void)
{
	size_t i;

	TEST_START("Fault streak at BKP6R, clear of every other owner");
	ASSERT_EQ_HEX(BKP(6), TAMP_BKP6R_ADDR);
	ASSERT_TRUE(TAMP_BKP6R_ADDR >= LINKER_BKPR_END);
	ASSERT_TRUE(TAMP_BKP6R_ADDR != BKP(10));
	ASSERT_TRUE(TAMP_BKP6R_ADDR != BKP(11));
	for (i = 0; i < N_ERR_REGS; i++)
		ASSERT_TRUE(err_regs[i] != TAMP_BKP6R_ADDR);
	TEST_PASS();
}

int main(void)
{
	printf("\n=== MGR_ERR backup register map ===\n");
	test_addresses_are_bkp12_to_17();
	test_no_alias_with_other_owners();
	test_fault_streak_register();

	printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
	return tests_failed ? 1 : 0;
}
