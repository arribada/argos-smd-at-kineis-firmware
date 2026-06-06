/**
 * @file    test_shutdown_sequence.c
 * @brief   Unit tests for UW_DOPPLER SHUTDOWN entry sequence
 *
 * Two call sites in kns_app_uw_doppler.c trigger SHUTDOWN on SMD_STDALONE:
 *   1. boot_loop_handle() PERMANENT_OFF (10 consecutive boot failures)
 *   2. enter_shutdown() via magnet 6 s gesture (SHUTDOWN_BLINK state)
 *
 * Both must run a specific sequence before LPM_shutdownNow() so the chip
 * cuts power cleanly and reboots correctly on the next reed wake:
 *
 *   a) Persist app state to flash (NVM_save) — magnet path only.
 *   b) Release the PWR_LATCH (drive PB7 LOW). Done BEFORE the LPM teardown
 *      because if the latch released happens after EnablePullDown, the
 *      pin transition order is wrong and the latch may not release.
 *   c) LPM_shutdownNow() — teardown ADC, GPIO to analog, pull-down PB7
 *      (maintain LOW in SHUTDOWN), arm RTC wake-up, clear PWR flags,
 *      then EnterSHUTDOWNMode.
 *
 * Tests mock the side-effects with counters + sequence tracking so we can
 * verify the ordering without touching HAL.
 */

#include "test_framework.h"

/* ---- Mock framework: log every "side effect" with its order ---- */

#define MOCK_LOG_SIZE 32
static const char *mock_log[MOCK_LOG_SIZE];
static int         mock_log_n;

static void mock_reset(void) { mock_log_n = 0; }
static void mock_log_add(const char *tag) {
    if (mock_log_n < MOCK_LOG_SIZE)
        mock_log[mock_log_n++] = tag;
}

static bool mock_contains(const char *tag) {
    for (int i = 0; i < mock_log_n; i++)
        if (strcmp(mock_log[i], tag) == 0) return true;
    return false;
}

static int mock_index_of(const char *tag) {
    for (int i = 0; i < mock_log_n; i++)
        if (strcmp(mock_log[i], tag) == 0) return i;
    return -1;
}

/* ---- Stubs for the firmware functions ---- */

static void STUB_MGR_WDG_refresh(void) { mock_log_add("WDG_refresh"); }
static bool STUB_MGR_NVM_save(void)    { mock_log_add("NVM_save"); return true; }
static void STUB_REED_releasePower(void){ mock_log_add("REED_releasePower"); }
static void STUB_EVTLOG_log_SHUTDOWN(void){ mock_log_add("EVT_SHUTDOWN"); }
static void STUB_EVTLOG_log_FACTORY_RESET(void){ mock_log_add("EVT_FACTORY_RESET"); }
static uint32_t last_wake_seconds_armed = 0;

static void STUB_LPM_shutdownWithAutoWake(uint32_t wake_seconds) {
    last_wake_seconds_armed = wake_seconds;
    /* LPM_shutdownWithAutoWake performs:
     *   teardown → optional RTC arm → flag clear → enter */
    mock_log_add("LPM_teardown_ADC");
    mock_log_add("LPM_GPIO_analog");
    mock_log_add("LPM_pulldown_PB7_latch");
    mock_log_add("LPM_configWakeUpRtc");
    mock_log_add("LPM_configWakeUpPins");
    if (wake_seconds > 0)
        mock_log_add("RTC_wakeup_armed");
    mock_log_add("LPM_PWR_flag_clear");
    mock_log_add("HAL_EnterSHUTDOWN");
}

static void STUB_LPM_shutdownNow(void) {
    STUB_LPM_shutdownWithAutoWake(0);
}

/* ---- Mirrors of the firmware call sites ---- */

static void mirror_magnet_shutdown(void)
{
    /* enter_shutdown() in kns_app_uw_doppler.c:628 (after my refactor). */
    STUB_EVTLOG_log_SHUTDOWN();
    STUB_MGR_WDG_refresh();
    STUB_MGR_NVM_save();
    STUB_REED_releasePower();
    STUB_LPM_shutdownNow();
}

static void mirror_bootloop_shutdown(void)
{
    /* boot_loop_handle() PERMANENT_OFF branch: now uses 24 h auto-wake
     * so sealed capsule never goes truly off forever. */
    STUB_EVTLOG_log_FACTORY_RESET();
    STUB_REED_releasePower();
    STUB_LPM_shutdownWithAutoWake(24u * 3600u);
}

/* ---- Magnet 6 s shutdown sequence ---- */

void test_magnet_logs_shutdown_event(void)
{
    TEST_START("Magnet shutdown logs EVT_SHUTDOWN");
    mock_reset();
    mirror_magnet_shutdown();
    ASSERT_TRUE(mock_contains("EVT_SHUTDOWN"));
    TEST_PASS();
}

void test_magnet_saves_nvm_before_power_release(void)
{
    TEST_START("Magnet shutdown: NVM_save BEFORE releasePower");
    mock_reset();
    mirror_magnet_shutdown();
    int nvm_idx = mock_index_of("NVM_save");
    int rel_idx = mock_index_of("REED_releasePower");
    ASSERT_TRUE(nvm_idx >= 0);
    ASSERT_TRUE(rel_idx >= 0);
    ASSERT_TRUE(nvm_idx < rel_idx);
    TEST_PASS();
}

void test_magnet_refreshes_wdg_before_nvm(void)
{
    TEST_START("Magnet shutdown: WDG_refresh BEFORE NVM_save (flash takes time)");
    mock_reset();
    mirror_magnet_shutdown();
    int wdg_idx = mock_index_of("WDG_refresh");
    int nvm_idx = mock_index_of("NVM_save");
    ASSERT_TRUE(wdg_idx < nvm_idx);
    TEST_PASS();
}

void test_magnet_releases_power_before_lpm_teardown(void)
{
    TEST_START("Magnet shutdown: releasePower BEFORE LPM_shutdownNow teardown");
    mock_reset();
    mirror_magnet_shutdown();
    int rel_idx = mock_index_of("REED_releasePower");
    int lpm_idx = mock_index_of("LPM_teardown_ADC");
    ASSERT_TRUE(rel_idx < lpm_idx);
    TEST_PASS();
}

void test_magnet_enters_hal_shutdown_last(void)
{
    TEST_START("Magnet shutdown: HAL_EnterSHUTDOWN is the LAST step");
    mock_reset();
    mirror_magnet_shutdown();
    int hal_idx = mock_index_of("HAL_EnterSHUTDOWN");
    ASSERT_EQ(mock_log_n - 1, hal_idx);
    TEST_PASS();
}

/* ---- Boot loop PERMANENT_OFF sequence ---- */

void test_bootloop_logs_factory_reset(void)
{
    TEST_START("Boot loop PERMANENT_OFF logs EVT_FACTORY_RESET");
    mock_reset();
    mirror_bootloop_shutdown();
    ASSERT_TRUE(mock_contains("EVT_FACTORY_RESET"));
    TEST_PASS();
}

void test_bootloop_releases_power(void)
{
    TEST_START("Boot loop PERMANENT_OFF releases power latch");
    mock_reset();
    mirror_bootloop_shutdown();
    ASSERT_TRUE(mock_contains("REED_releasePower"));
    TEST_PASS();
}

void test_bootloop_no_nvm_save(void)
{
    TEST_START("Boot loop PERMANENT_OFF does NOT save NVM (flash may be bad)");
    mock_reset();
    mirror_bootloop_shutdown();
    /* Boot loop fires because the chip is in a bad state — touching flash
     * is risky. NVM save is intentionally NOT in this path. */
    ASSERT_FALSE(mock_contains("NVM_save"));
    TEST_PASS();
}

void test_bootloop_clears_pwr_flags(void)
{
    TEST_START("Boot loop PERMANENT_OFF clears PWR flags via LPM_shutdownNow");
    mock_reset();
    mirror_bootloop_shutdown();
    ASSERT_TRUE(mock_contains("LPM_PWR_flag_clear"));
    TEST_PASS();
}

void test_both_paths_enter_hal_shutdown(void)
{
    TEST_START("Both paths terminate with HAL_EnterSHUTDOWN");
    mock_reset();
    mirror_magnet_shutdown();
    ASSERT_TRUE(mock_contains("HAL_EnterSHUTDOWN"));

    mock_reset();
    mirror_bootloop_shutdown();
    ASSERT_TRUE(mock_contains("HAL_EnterSHUTDOWN"));
    TEST_PASS();
}

void test_bootloop_arms_24h_auto_wake(void)
{
    TEST_START("Boot-loop PERMANENT_OFF arms 24h RTC auto-wake (sealed-safe)");
    mock_reset();
    last_wake_seconds_armed = 0;
    mirror_bootloop_shutdown();
    ASSERT_TRUE(mock_contains("RTC_wakeup_armed"));
    ASSERT_EQ((long)(24L * 3600L), (long)last_wake_seconds_armed);
    TEST_PASS();
}

void test_magnet_no_auto_wake_by_default(void)
{
    TEST_START("Magnet 6s shutdown does NOT auto-wake by default (user choice)");
    mock_reset();
    last_wake_seconds_armed = 0xDEADBEEF;
    mirror_magnet_shutdown();
    /* mirror_magnet_shutdown uses LPM_shutdownNow which is 0 wake_seconds. */
    ASSERT_FALSE(mock_contains("RTC_wakeup_armed"));
    ASSERT_EQ(0L, (long)last_wake_seconds_armed);
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("UW_DOPPLER SHUTDOWN entry sequences");
    test_magnet_logs_shutdown_event();
    test_magnet_saves_nvm_before_power_release();
    test_magnet_refreshes_wdg_before_nvm();
    test_magnet_releases_power_before_lpm_teardown();
    test_magnet_enters_hal_shutdown_last();
    test_bootloop_logs_factory_reset();
    test_bootloop_releases_power();
    test_bootloop_no_nvm_save();
    test_bootloop_clears_pwr_flags();
    test_both_paths_enter_hal_shutdown();
    test_bootloop_arms_24h_auto_wake();
    test_magnet_no_auto_wake_by_default();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
