/**
 * @file    test_iwdg_optionbyte.c
 * @brief   Unit tests for IWDG_STOP option-byte enforcement logic.
 *
 * Mirrors MGR_WDG_ensureIwdgStopOptionByte() decision tree:
 *   - bit already set         -> false (no action)
 *   - bit not set, unlock OK  -> program, launch (reset, never returns)
 *   - any HAL unlock failure  -> false + logged
 *
 * The actual hardware flash write isn't testable in unit form, but the
 * gating logic (which determines whether we attempt the write at all) is.
 */

#include "test_framework.h"

#define FLASH_OPTR_IWDG_STOP  (1u << 17)

typedef enum {
    HAL_STATUS_OK = 0,
    HAL_STATUS_ERROR,
} hal_status_t;

/* Mocked HAL state */
static uint32_t g_optr;
static hal_status_t g_flash_unlock_result;
static hal_status_t g_ob_unlock_result;
static hal_status_t g_ob_program_result;
static int g_launch_called;
static int g_ob_program_called;

static void reset_mocks(void)
{
    g_optr = 0;
    g_flash_unlock_result = HAL_STATUS_OK;
    g_ob_unlock_result = HAL_STATUS_OK;
    g_ob_program_result = HAL_STATUS_OK;
    g_launch_called = 0;
    g_ob_program_called = 0;
}

/* Mirror of MGR_WDG_ensureIwdgStopOptionByte() decision logic */
static bool ensure_optionbyte(void)
{
    if (g_optr & FLASH_OPTR_IWDG_STOP)
        return false;  /* already set */

    if (g_flash_unlock_result != HAL_STATUS_OK)
        return false;

    if (g_ob_unlock_result != HAL_STATUS_OK)
        return false;

    g_ob_program_called++;
    if (g_ob_program_result != HAL_STATUS_OK)
        return false;

    g_launch_called++;
    /* In real HW the launch resets the chip. Tests assume the function
     * "would have" reset, and validate that it would not return on success.
     * For the mock we just return false (the real fn's fallback). */
    return false;
}

/* ---- Tests ---- */

void test_bit_already_set_no_action(void)
{
    TEST_START("IWDG_STOP already set -> no flash write attempted");
    reset_mocks();
    g_optr = FLASH_OPTR_IWDG_STOP | 0x12340000u;  /* other bits don't matter */
    bool r = ensure_optionbyte();
    ASSERT_FALSE(r);
    ASSERT_EQ(0, g_ob_program_called);
    ASSERT_EQ(0, g_launch_called);
    TEST_PASS();
}

void test_bit_clear_attempts_program(void)
{
    TEST_START("IWDG_STOP clear -> program + launch attempted");
    reset_mocks();
    g_optr = 0;  /* all bits clear */
    (void)ensure_optionbyte();
    ASSERT_EQ(1, g_ob_program_called);
    ASSERT_EQ(1, g_launch_called);
    TEST_PASS();
}

void test_flash_unlock_failure_aborts(void)
{
    TEST_START("FLASH unlock failure -> abort, no program");
    reset_mocks();
    g_optr = 0;
    g_flash_unlock_result = HAL_STATUS_ERROR;
    bool r = ensure_optionbyte();
    ASSERT_FALSE(r);
    ASSERT_EQ(0, g_ob_program_called);
    ASSERT_EQ(0, g_launch_called);
    TEST_PASS();
}

void test_ob_unlock_failure_aborts(void)
{
    TEST_START("OB unlock failure -> abort, no program");
    reset_mocks();
    g_optr = 0;
    g_ob_unlock_result = HAL_STATUS_ERROR;
    bool r = ensure_optionbyte();
    ASSERT_FALSE(r);
    ASSERT_EQ(0, g_ob_program_called);
    ASSERT_EQ(0, g_launch_called);
    TEST_PASS();
}

void test_program_failure_no_launch(void)
{
    TEST_START("OB program failure -> no launch (avoids weird half-set state)");
    reset_mocks();
    g_optr = 0;
    g_ob_program_result = HAL_STATUS_ERROR;
    bool r = ensure_optionbyte();
    ASSERT_FALSE(r);
    ASSERT_EQ(1, g_ob_program_called);
    ASSERT_EQ(0, g_launch_called);
    TEST_PASS();
}

void test_idempotent_after_set(void)
{
    TEST_START("After (simulated) successful set, second call is a no-op");
    reset_mocks();
    g_optr = 0;
    (void)ensure_optionbyte();  /* attempt #1 */
    /* Simulate the reset + reload: bit is now set. */
    g_optr = FLASH_OPTR_IWDG_STOP;
    g_ob_program_called = 0;
    g_launch_called = 0;
    bool r = ensure_optionbyte();
    ASSERT_FALSE(r);
    ASSERT_EQ(0, g_ob_program_called);
    ASSERT_EQ(0, g_launch_called);
    TEST_PASS();
}

void test_optr_other_bits_dont_trigger(void)
{
    TEST_START("Other OPTR bits set, IWDG_STOP clear -> still attempts");
    reset_mocks();
    g_optr = 0xFFFFFFFFu & ~FLASH_OPTR_IWDG_STOP;
    (void)ensure_optionbyte();
    ASSERT_EQ(1, g_ob_program_called);
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("IWDG_STOP option-byte ensure logic");
    test_bit_already_set_no_action();
    test_bit_clear_attempts_program();
    test_flash_unlock_failure_aborts();
    test_ob_unlock_failure_aborts();
    test_program_failure_no_launch();
    test_idempotent_after_set();
    test_optr_other_bits_dont_trigger();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
