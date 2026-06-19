/**
 * @file    test_crash_forensics.c
 * @brief   Unit tests for MGR_ERR crash_info struct (CRC + replay protocol).
 *
 * The crash_info struct lives in SRAM2 retention RAM. It's written by the
 * fault handlers (after capturing the Cortex-M stacked frame + SCB fault
 * regs) and read once on the next boot by KNS_APP_uw_doppler_init() before
 * being cleared. The CRC guards against partial writes / first-power-on
 * garbage so we don't emit a spurious post-mortem trace.
 *
 * Tests mirror the struct + CRC + protocol from MGR_ERR mgr_err.c.
 */

#include "test_framework.h"
#include <stddef.h>

#define CRASH_MAGIC  0xC4A5417FUL

typedef struct {
    uint32_t magic;
    uint8_t  fault_type;
    uint8_t  app_state;
    uint16_t _pad;
    uint32_t hfsr;
    uint32_t cfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t pc;
    uint32_t lr;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t xpsr;
    uint32_t tick;
    uint32_t crc;
} crash_info_t;

static uint32_t crash_crc(const crash_info_t *c)
{
    const uint32_t *p = (const uint32_t *)c;
    size_t n = (offsetof(crash_info_t, crc)) / 4;
    uint32_t s = 0;
    for (size_t i = 0; i < n; i++)
        s = s * 31u + p[i];
    return s;
}

static void capture(crash_info_t *c, uint32_t *frame,
                    uint8_t fault_type, uint8_t app_state)
{
    c->magic      = CRASH_MAGIC;
    c->fault_type = fault_type;
    c->app_state  = app_state;
    c->_pad       = 0;
    c->hfsr       = 0x40000000u;  /* simulated FORCED */
    c->cfsr       = 0x00020000u;  /* INVSTATE */
    c->bfar       = 0xE000ED38u;
    c->mmfar      = 0xE000ED34u;
    if (frame) {
        c->r0 = frame[0]; c->r1 = frame[1]; c->r2 = frame[2]; c->r3 = frame[3];
        c->r12 = frame[4]; c->lr = frame[5]; c->pc = frame[6]; c->xpsr = frame[7];
    } else {
        c->r0 = c->r1 = c->r2 = c->r3 = 0;
        c->r12 = c->lr = c->pc = c->xpsr = 0;
    }
    c->tick = 12345;
    c->crc  = crash_crc(c);
}

static bool has_valid(const crash_info_t *c)
{
    return c->magic == CRASH_MAGIC && c->crc == crash_crc(c);
}

static bool take(crash_info_t *retained, crash_info_t *out)
{
    if (!has_valid(retained)) return false;
    if (out) *out = *retained;
    retained->magic = 0;
    retained->crc   = 0;
    return true;
}

/* ---- Tests ---- */

void test_fresh_sram_no_crash(void)
{
    TEST_START("Fresh SRAM (all zero / garbage) -> hasRetainedCrash false");
    crash_info_t c;
    memset(&c, 0, sizeof(c));
    ASSERT_FALSE(has_valid(&c));

    memset(&c, 0xFF, sizeof(c));
    ASSERT_FALSE(has_valid(&c));

    memset(&c, 0xAA, sizeof(c));
    ASSERT_FALSE(has_valid(&c));
    TEST_PASS();
}

void test_capture_valid(void)
{
    TEST_START("After capture, struct is valid");
    crash_info_t c;
    uint32_t frame[8] = {1,2,3,4, 12, 0xDEAD, 0x08001234, 0x21000000};
    capture(&c, frame, 1 /*ERR_HARDFAULT*/, 4 /*MONITORING*/);
    ASSERT_TRUE(has_valid(&c));
    ASSERT_EQ(0x08001234, c.pc);
    ASSERT_EQ(0xDEAD, c.lr);
    TEST_PASS();
}

void test_take_clears(void)
{
    TEST_START("take() copies + clears so next call returns false");
    crash_info_t retained, out;
    uint32_t frame[8] = {1,2,3,4, 12, 0xBAD0, 0x08002000, 0};
    capture(&retained, frame, 2 /*ERR_BUSFAULT*/, 5);
    ASSERT_TRUE(take(&retained, &out));
    ASSERT_EQ(2, out.fault_type);
    ASSERT_EQ(0x08002000, out.pc);
    ASSERT_FALSE(take(&retained, NULL));
    TEST_PASS();
}

void test_corruption_detected(void)
{
    TEST_START("Bit flip in PC after capture invalidates CRC");
    crash_info_t c;
    uint32_t frame[8] = {0};
    frame[6] = 0x08001000;
    capture(&c, frame, 1, 4);
    ASSERT_TRUE(has_valid(&c));
    c.pc ^= 1;  /* tamper without recomputing CRC */
    ASSERT_FALSE(has_valid(&c));
    TEST_PASS();
}

void test_magic_alone_insufficient(void)
{
    TEST_START("Magic correct but CRC wrong -> invalid");
    crash_info_t c;
    memset(&c, 0, sizeof(c));
    c.magic = CRASH_MAGIC;
    c.crc   = 0;  /* deliberately wrong */
    ASSERT_FALSE(has_valid(&c));
    TEST_PASS();
}

void test_take_passes_full_struct(void)
{
    TEST_START("take() out struct exposes all captured fields");
    crash_info_t retained, out;
    uint32_t frame[8] = {0xDEADBEEF, 1, 2, 3, 0x12, 0xAAAA, 0x08003000, 0x21000000};
    capture(&retained, frame, 4 /*ERR_USAGEFAULT*/, 6);
    ASSERT_TRUE(take(&retained, &out));
    ASSERT_EQ(4, out.fault_type);
    ASSERT_EQ(6, out.app_state);
    ASSERT_EQ(0x40000000, out.hfsr);
    ASSERT_EQ(0x00020000, out.cfsr);
    ASSERT_EQ(0xDEADBEEF, out.r0);
    ASSERT_EQ(0xAAAA, out.lr);
    ASSERT_EQ(0x08003000, out.pc);
    TEST_PASS();
}

void test_no_frame_safe(void)
{
    TEST_START("capture(NULL frame) sets PC/LR to 0 safely (no UB)");
    crash_info_t c;
    capture(&c, NULL, 5 /*ERR_NMI*/, 0);
    ASSERT_TRUE(has_valid(&c));
    ASSERT_EQ(0u, c.pc);
    ASSERT_EQ(0u, c.lr);
    TEST_PASS();
}

void test_multiple_captures_overwrite(void)
{
    TEST_START("Successive capture()s overwrite — only latest is replayed");
    crash_info_t retained, out;
    uint32_t f1[8] = {0,0,0,0,0,0, 0x1000, 0};
    uint32_t f2[8] = {0,0,0,0,0,0, 0x2000, 0};
    capture(&retained, f1, 1, 0);
    capture(&retained, f2, 1, 0);  /* second fault overwrites first */
    ASSERT_TRUE(take(&retained, &out));
    ASSERT_EQ(0x2000, out.pc);
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("HardFault forensics (crash_info CRC + replay)");
    test_fresh_sram_no_crash();
    test_capture_valid();
    test_take_clears();
    test_corruption_detected();
    test_magic_alone_insufficient();
    test_take_passes_full_struct();
    test_no_frame_safe();
    test_multiple_captures_overwrite();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
