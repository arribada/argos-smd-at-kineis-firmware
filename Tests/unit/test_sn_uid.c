/**
 * @file    test_sn_uid.c
 * @brief   UID-derived serial number: format, determinism, uniqueness.
 *
 * Mirrors MCU_NVM_getSN() (mcu_nvm.c): SN = "SMD" + 3 hex (lot^wafer words
 * folded to 12 bits) + 8 hex (die X/Y verbatim), exactly 14 chars, no NUL
 * in the output contract.
 */

#include "test_framework.h"

#define DEVICE_SN_LENGTH 14

/* Injectable UID words (the real code reads UID_BASE). */
static uint32_t uid_xy, uid_wafer, uid_lot;

static int sn_build(uint8_t sn[])
{
	const uint32_t mix = uid_wafer ^ uid_lot;
	const uint32_t fold12 = (mix ^ (mix >> 12) ^ (mix >> 24)) & 0xFFFu;

	char buf[DEVICE_SN_LENGTH + 2];
	const int n = snprintf(buf, sizeof(buf), "SMD%03lX%08lX",
			       (unsigned long)fold12, (unsigned long)uid_xy);
	if (n != DEVICE_SN_LENGTH)
		return -1;
	memcpy(sn, buf, DEVICE_SN_LENGTH);
	return 0;
}

static void test_sn_is_14_chars_with_prefix(void)
{
	uint8_t sn[DEVICE_SN_LENGTH + 1] = {0};
	uid_xy = 0x0012001Fu; uid_wafer = 0x38323436u; uid_lot = 0x57334A31u;
	ASSERT_EQ(0, sn_build(sn));
	sn[DEVICE_SN_LENGTH] = '\0';
	ASSERT_EQ(14, (long)strlen((char *)sn));
	ASSERT_TRUE(sn[0] == 'S' && sn[1] == 'M' && sn[2] == 'D');
	/* All payload chars are uppercase hex */
	for (int i = 3; i < DEVICE_SN_LENGTH; i++)
		ASSERT_TRUE((sn[i] >= '0' && sn[i] <= '9') ||
		            (sn[i] >= 'A' && sn[i] <= 'F'));
	TEST_PASS();
}

static void test_sn_deterministic(void)
{
	uint8_t a[DEVICE_SN_LENGTH], b[DEVICE_SN_LENGTH];
	uid_xy = 0xDEADBEEFu; uid_wafer = 0x12345678u; uid_lot = 0x9ABCDEF0u;
	ASSERT_EQ(0, sn_build(a));
	ASSERT_EQ(0, sn_build(b));
	ASSERT_MEM_EQ(a, b, DEVICE_SN_LENGTH);
	TEST_PASS();
}

static void test_sn_differs_with_die_xy(void)
{
	uint8_t a[DEVICE_SN_LENGTH], b[DEVICE_SN_LENGTH];
	uid_wafer = 0x38323436u; uid_lot = 0x57334A31u;
	uid_xy = 0x00120010u; ASSERT_EQ(0, sn_build(a));
	uid_xy = 0x00120011u; ASSERT_EQ(0, sn_build(b));
	ASSERT_TRUE(memcmp(a, b, DEVICE_SN_LENGTH) != 0);
	TEST_PASS();
}

static void test_sn_differs_with_lot(void)
{
	uint8_t a[DEVICE_SN_LENGTH], b[DEVICE_SN_LENGTH];
	uid_xy = 0x0012001Fu; uid_wafer = 0x38323436u;
	uid_lot = 0x57334A31u; ASSERT_EQ(0, sn_build(a));
	uid_lot = 0x57334A32u; ASSERT_EQ(0, sn_build(b));
	ASSERT_TRUE(memcmp(a, b, DEVICE_SN_LENGTH) != 0);
	TEST_PASS();
}

static void test_sn_fold_uses_high_lot_bits(void)
{
	/* A change ONLY in the top nibble of the lot word must still move
	 * the serial (every UID bit contributes via the 12-bit fold). */
	uint8_t a[DEVICE_SN_LENGTH], b[DEVICE_SN_LENGTH];
	uid_xy = 0x0012001Fu; uid_wafer = 0x38323436u;
	uid_lot = 0x07334A31u; ASSERT_EQ(0, sn_build(a));
	uid_lot = 0xF7334A31u; ASSERT_EQ(0, sn_build(b));
	ASSERT_TRUE(memcmp(a, b, DEVICE_SN_LENGTH) != 0);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UID-derived serial number");

	RUN_TEST(test_sn_is_14_chars_with_prefix);
	RUN_TEST(test_sn_deterministic);
	RUN_TEST(test_sn_differs_with_die_xy);
	RUN_TEST(test_sn_differs_with_lot);
	RUN_TEST(test_sn_fold_uses_high_lot_bits);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
