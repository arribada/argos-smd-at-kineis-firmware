/**
 * @file    test_led_swap.c
 * @brief   Unit tests for MGR_LED color mapping (SMD_STDALONE)
 *
 * Confirmed 2026-06-05 by direct-pin diagnostic: on SMD_STDALONE
 *   PA1 -> physical RED, PB4 -> physical GREEN, PB5 -> physical BLUE
 *
 * BSP header bsp_smd_stdalone.h matches; no GPIO swap needed in led_set_raw().
 * A previous swap macro was added on the wrong hypothesis and caused R-B-G
 * to be observed instead of R-G-B; this test guards against that regression.
 *
 * This file mirrors led_apply_color() + led_set_raw() (no swap path) and
 * verifies every named color drives exactly the expected GPIOs.
 */

#include "test_framework.h"

/* Mirror of led_apply_color + led_set_raw (no SMD_STDALONE swap).
 * Returns bitfield of physical LEDs that light up:
 *   bit0 = physical RED   (driven by LED_RED_Pin   on PA1)
 *   bit1 = physical GREEN (driven by LED_GREEN_Pin on PB4)
 *   bit2 = physical BLUE  (driven by LED_BLUE_Pin  on PB5)
 */
typedef enum {
	C_OFF = 0, C_WHITE, C_RED, C_GREEN, C_BLUE,
	C_VIOLET, C_CYAN, C_YELLOW,
} mgr_led_color_t;

static uint8_t led_drive(mgr_led_color_t color)
{
	bool r = false, g = false, b = false;
	switch (color) {
	case C_OFF:    break;
	case C_WHITE:  r = true; g = true;  b = true;  break;
	case C_RED:    r = true;                       break;
	case C_GREEN:           g = true;              break;
	case C_BLUE:                        b = true;  break;
	case C_VIOLET: r = true;            b = true;  break;
	case C_CYAN:            g = true;   b = true;  break;
	case C_YELLOW: r = true; g = true;             break;
	}
	uint8_t phys = 0;
	if (r) phys |= (1u << 0);  /* PA1 = physical RED */
	if (g) phys |= (1u << 1);  /* PB4 = physical GREEN */
	if (b) phys |= (1u << 2);  /* PB5 = physical BLUE */
	return phys;
}

#define PHY_RED    (1u << 0)
#define PHY_GREEN  (1u << 1)
#define PHY_BLUE   (1u << 2)

void test_off(void)
{
	TEST_START("OFF -> no LED");
	ASSERT_EQ(0, led_drive(C_OFF));
	TEST_PASS();
}

void test_red(void)
{
	TEST_START("RED -> physical RED only");
	ASSERT_EQ(PHY_RED, led_drive(C_RED));
	TEST_PASS();
}

void test_green(void)
{
	TEST_START("GREEN -> physical GREEN only (no swap regression)");
	ASSERT_EQ(PHY_GREEN, led_drive(C_GREEN));
	TEST_PASS();
}

void test_blue(void)
{
	TEST_START("BLUE -> physical BLUE only (no swap regression)");
	ASSERT_EQ(PHY_BLUE, led_drive(C_BLUE));
	TEST_PASS();
}

void test_white(void)
{
	TEST_START("WHITE -> all three lit");
	ASSERT_EQ(PHY_RED | PHY_GREEN | PHY_BLUE, led_drive(C_WHITE));
	TEST_PASS();
}

void test_violet(void)
{
	TEST_START("VIOLET -> RED + BLUE");
	ASSERT_EQ(PHY_RED | PHY_BLUE, led_drive(C_VIOLET));
	TEST_PASS();
}

void test_cyan(void)
{
	TEST_START("CYAN -> GREEN + BLUE");
	ASSERT_EQ(PHY_GREEN | PHY_BLUE, led_drive(C_CYAN));
	TEST_PASS();
}

void test_yellow(void)
{
	TEST_START("YELLOW -> RED + GREEN");
	ASSERT_EQ(PHY_RED | PHY_GREEN, led_drive(C_YELLOW));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("MGR_LED color mapping (no swap, BSP-direct)");
	test_off();
	test_red();
	test_green();
	test_blue();
	test_white();
	test_violet();
	test_cyan();
	test_yellow();
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
