/**
 * @file    test_boot_loop_time.c
 * @brief   Unit tests for the TIME-based boot-loop detector.
 *
 * The detector uses a `boot_in_progress` flag in retention RAM:
 *   - SET at boot
 *   - CLEARED on MONITORING reached (boot_loop_mark_success)
 *
 * Logic:
 *   - First boot (struct invalid): initialize, set boot_in_progress, no count
 *   - BOR/PIN reset: clear counter (user-initiated), set boot_in_progress
 *   - SW reset + previous boot did NOT mark success (boot_in_progress still 1):
 *       count as failure
 *   - SW reset + previous boot DID mark success (boot_in_progress = 0):
 *       don't count — this is a steady-state reset (AT+RESET, late fault)
 *   - At FACTORY_RESET threshold: wipe NVM
 *   - At PERMANENT_OFF threshold: SHUTDOWN with 24h auto-wake
 *
 * Replaces the previous count-every-reset scheme that pushed the chip into
 * permanent SHUTDOWN after JLink reflashes.
 */

#include "test_framework.h"

#define BOOT_RETAIN_MAGIC          0x424F4F54UL  /* "BOOT" */
#define BOOT_FAIL_FACTORY_RESET    5u
#define BOOT_FAIL_PERMANENT_OFF    10u

#define RCC_CSR_BORRSTF  (1u << 27)
#define RCC_CSR_PINRSTF  (1u << 28)
#define RCC_CSR_SFTRSTF  (1u << 23)
#define RCC_CSR_IWDGRSTF (1u << 25)

typedef struct {
    uint32_t magic;
    uint16_t consecutive_failures;
    uint8_t  factory_reset_attempted;
    uint8_t  permanent_off_armed;
    uint8_t  boot_in_progress;
    uint8_t  _pad[3];
    uint32_t crc32;
} boot_retained_t;

/* Effect tracking — what handle() does this boot. */
typedef enum {
    EFFECT_NONE,
    EFFECT_INIT_FRESH,
    EFFECT_CLEAR_ON_COLD_RESET,
    EFFECT_PREVIOUS_OK_NO_COUNT,
    EFFECT_COUNT_FAILURE,
    EFFECT_FACTORY_RESET,
    EFFECT_PERMANENT_OFF_WITH_AUTO_WAKE,
} effect_t;

static uint32_t crc_of(const boot_retained_t *b)
{
    uint32_t s = b->magic;
    s = s * 31u + b->consecutive_failures;
    s = s * 31u + b->factory_reset_attempted;
    s = s * 31u + b->permanent_off_armed;
    s = s * 31u + b->boot_in_progress;
    return s;
}

static bool valid(const boot_retained_t *b)
{
    return b->magic == BOOT_RETAIN_MAGIC && b->crc32 == crc_of(b);
}

static void commit(boot_retained_t *b)
{
    b->magic = BOOT_RETAIN_MAGIC;
    b->crc32 = crc_of(b);
}

static effect_t boot_loop_handle_mirror(boot_retained_t *b, uint32_t csr)
{
    if (!valid(b)) {
        memset(b, 0, sizeof(*b));
        b->boot_in_progress = 1;
        commit(b);
        return EFFECT_INIT_FRESH;
    }
    if (csr & (RCC_CSR_BORRSTF | RCC_CSR_PINRSTF)) {
        b->consecutive_failures = 0;
        b->factory_reset_attempted = 0;
        b->permanent_off_armed = 0;
        b->boot_in_progress = 1;
        commit(b);
        return EFFECT_CLEAR_ON_COLD_RESET;
    }
    if (!b->boot_in_progress) {
        b->boot_in_progress = 1;
        commit(b);
        return EFFECT_PREVIOUS_OK_NO_COUNT;
    }
    if (b->consecutive_failures < 0xFFFFu)
        b->consecutive_failures++;
    b->boot_in_progress = 1;
    commit(b);

    if (b->consecutive_failures >= BOOT_FAIL_PERMANENT_OFF) {
        b->permanent_off_armed = 1;
        commit(b);
        return EFFECT_PERMANENT_OFF_WITH_AUTO_WAKE;
    }
    if (b->consecutive_failures >= BOOT_FAIL_FACTORY_RESET &&
        !b->factory_reset_attempted) {
        b->factory_reset_attempted = 1;
        commit(b);
        return EFFECT_FACTORY_RESET;
    }
    return EFFECT_COUNT_FAILURE;
}

static void mark_success(boot_retained_t *b)
{
    b->consecutive_failures = 0;
    b->factory_reset_attempted = 0;
    b->boot_in_progress = 0;
    commit(b);
}

/* ---- Tests ---- */

void test_first_boot_initializes(void)
{
    TEST_START("Fresh SRAM2 -> initialize, no failure count");
    boot_retained_t b;
    memset(&b, 0xFF, sizeof(b));  /* uninitialized */
    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_BORRSTF | RCC_CSR_PINRSTF);
    ASSERT_EQ(EFFECT_INIT_FRESH, e);
    ASSERT_EQ(0, b.consecutive_failures);
    ASSERT_EQ(1, b.boot_in_progress);
    TEST_PASS();
}

void test_cold_reset_clears_counter(void)
{
    TEST_START("BOR/PIN reset clears previous failure counter");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.consecutive_failures = 4;
    b.boot_in_progress = 1;
    commit(&b);

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_BORRSTF);
    ASSERT_EQ(EFFECT_CLEAR_ON_COLD_RESET, e);
    ASSERT_EQ(0, b.consecutive_failures);
    TEST_PASS();
}

void test_steady_state_reset_no_count(void)
{
    TEST_START("Reset after success (boot_in_progress=0) does NOT count");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    /* Previous boot reached MONITORING and cleared boot_in_progress. */
    b.consecutive_failures = 0;
    b.boot_in_progress = 0;
    commit(&b);

    /* Software reset (e.g. AT+RESET after hours of uptime). */
    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
    ASSERT_EQ(EFFECT_PREVIOUS_OK_NO_COUNT, e);
    ASSERT_EQ(0, b.consecutive_failures);
    TEST_PASS();
}

void test_fail_before_monitoring_counts(void)
{
    TEST_START("SW reset + previous boot didn't reach MONITORING -> count");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.consecutive_failures = 0;
    b.boot_in_progress = 1;  /* previous boot died */
    commit(&b);

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
    ASSERT_EQ(EFFECT_COUNT_FAILURE, e);
    ASSERT_EQ(1, b.consecutive_failures);
    TEST_PASS();
}

void test_factory_reset_at_threshold(void)
{
    TEST_START("5th failure triggers FACTORY_RESET");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.consecutive_failures = BOOT_FAIL_FACTORY_RESET - 1;
    b.boot_in_progress = 1;
    commit(&b);

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
    ASSERT_EQ(EFFECT_FACTORY_RESET, e);
    ASSERT_EQ(1, b.factory_reset_attempted);
    TEST_PASS();
}

void test_factory_reset_only_once(void)
{
    TEST_START("Factory reset triggers only once (idempotent)");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.consecutive_failures = BOOT_FAIL_FACTORY_RESET;
    b.factory_reset_attempted = 1;  /* already done */
    b.boot_in_progress = 1;
    commit(&b);

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
    /* Counter increments (to 6) but no factory reset re-triggered. */
    ASSERT_EQ(EFFECT_COUNT_FAILURE, e);
    ASSERT_EQ(BOOT_FAIL_FACTORY_RESET + 1, b.consecutive_failures);
    TEST_PASS();
}

void test_permanent_off_triggers_auto_wake(void)
{
    TEST_START("10th failure -> SHUTDOWN with 24h auto-wake (NOT off forever)");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.consecutive_failures = BOOT_FAIL_PERMANENT_OFF - 1;
    b.factory_reset_attempted = 1;  /* already done */
    b.boot_in_progress = 1;
    commit(&b);

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
    ASSERT_EQ(EFFECT_PERMANENT_OFF_WITH_AUTO_WAKE, e);
    ASSERT_EQ(1, b.permanent_off_armed);
    TEST_PASS();
}

void test_monitoring_success_clears_flag(void)
{
    TEST_START("On MONITORING reached, boot_in_progress is cleared");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.consecutive_failures = 3;
    b.boot_in_progress = 1;
    commit(&b);

    mark_success(&b);
    ASSERT_EQ(0, b.boot_in_progress);
    ASSERT_EQ(0, b.consecutive_failures);
    TEST_PASS();
}

void test_dev_reflash_does_not_lock_chip(void)
{
    TEST_START("Simulate 15 dev reflashes (success each time) -> never PERMANENT_OFF");
    boot_retained_t b;
    memset(&b, 0xFF, sizeof(b));

    /* First boot: init. */
    boot_loop_handle_mirror(&b, RCC_CSR_BORRSTF | RCC_CSR_PINRSTF);
    mark_success(&b);

    /* 15 reflashes: each reflashes triggers SFTRSTF. Previous boot reached
     * MONITORING and cleared boot_in_progress, so no count. */
    for (int i = 0; i < 15; i++) {
        effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
        ASSERT_EQ(EFFECT_PREVIOUS_OK_NO_COUNT, e);
        mark_success(&b);
    }
    ASSERT_EQ(0, b.consecutive_failures);
    ASSERT_EQ(0, b.permanent_off_armed);
    TEST_PASS();
}

void test_genuine_failure_loop_escalates(void)
{
    TEST_START("Real failure loop (never reach MONITORING) escalates correctly");
    boot_retained_t b;
    memset(&b, 0xFF, sizeof(b));

    /* First boot: init. boot_in_progress = 1, never cleared. */
    boot_loop_handle_mirror(&b, RCC_CSR_BORRSTF | RCC_CSR_PINRSTF);

    /* SW reset 5 times (each previous boot failed). */
    int factory_reset_seen = 0;
    int permanent_off_seen = 0;
    for (int i = 0; i < 12; i++) {
        effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
        if (e == EFFECT_FACTORY_RESET) factory_reset_seen++;
        if (e == EFFECT_PERMANENT_OFF_WITH_AUTO_WAKE) permanent_off_seen++;
    }
    ASSERT_EQ(1, factory_reset_seen);
    ASSERT_TRUE(permanent_off_seen >= 1);
    TEST_PASS();
}

void test_iwdg_reset_counted_as_failure(void)
{
    TEST_START("IWDG reset before MONITORING is counted as a failure");
    boot_retained_t b;
    memset(&b, 0, sizeof(b));
    b.boot_in_progress = 1;  /* previous boot didn't finish */
    commit(&b);

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_IWDGRSTF);
    ASSERT_EQ(EFFECT_COUNT_FAILURE, e);
    ASSERT_EQ(1, b.consecutive_failures);
    TEST_PASS();
}

void test_corrupted_struct_reinit(void)
{
    TEST_START("Corrupted retention struct re-initializes safely");
    boot_retained_t b;
    memset(&b, 0xAA, sizeof(b));  /* garbage */
    /* magic + CRC won't match. */

    effect_t e = boot_loop_handle_mirror(&b, RCC_CSR_SFTRSTF);
    ASSERT_EQ(EFFECT_INIT_FRESH, e);
    ASSERT_EQ(0, b.consecutive_failures);
    ASSERT_TRUE(valid(&b));
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("Boot-loop TIME-based detector + sealed-deployment recovery");
    test_first_boot_initializes();
    test_cold_reset_clears_counter();
    test_steady_state_reset_no_count();
    test_fail_before_monitoring_counts();
    test_factory_reset_at_threshold();
    test_factory_reset_only_once();
    test_permanent_off_triggers_auto_wake();
    test_monitoring_success_clears_flag();
    test_dev_reflash_does_not_lock_chip();
    test_genuine_failure_loop_escalates();
    test_iwdg_reset_counted_as_failure();
    test_corrupted_struct_reinit();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
