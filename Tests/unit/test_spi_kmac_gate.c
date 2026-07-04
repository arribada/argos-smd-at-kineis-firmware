/**
 * @file    test_spi_kmac_gate.c
 * @brief   Contract tests for the SPI BLIND-config gate and the CMD_READ_TX_HDLR
 *          wire format.
 *
 * Mirrors two contracts added on the feat-blind branch:
 *  1. bMGR_SPI_CMD_WRITEKMAC_cmd copies the 7-byte BLIND ctx from the A+ payload
 *     ONLY when ALL THREE gates pass (mgr_spi_cmd_list_mac.c):
 *       - profile id == BLIND (BASIC/NONE keep blindCfg = {0} as before)
 *       - A+ protocol frame (legacy rx->size is the RAW DMA clock count — a
 *         fixed-size-transaction master would otherwise copy bus padding)
 *       - rx->size >= 2 + 7 (cmd byte + id + full ctx actually present)
 *  2. bMGR_SPI_CMD_READTXHDLR_cmd returns the handler as int16 LITTLE-ENDIAN,
 *     2 bytes (memcpy — same convention as CMD_READ_MC).
 *
 * Standalone mirror test (the harness compiles each test alone, -Iunit).
 */

#include "test_framework.h"

/* Mirror of the profile ids used by the gate (kns_mac_prfl_cfg.h order,
 * static-asserted in mgr_at_cmd_common.c: NONE=0, BASIC=1, BLIND=2). */
#define PRFL_NONE  0
#define PRFL_BASIC 1
#define PRFL_BLIND 2

#define BLIND_CTX_SIZE 7u  /* sizeof(struct KNS_MAC_BLIND_usrCfg_t), packed */

/* Mirror of the firmware triple gate. */
static int gate_allows_copy(int id, int is_legacy, unsigned rx_size)
{
    return id == PRFL_BLIND &&
           !is_legacy &&
           rx_size >= 2u + BLIND_CTX_SIZE;
}

void test_gate_blind_aplus_full_ctx_copies(void)
{
    TEST_START("BLIND + A+ + full ctx (size 9) -> copy");
    ASSERT_TRUE(gate_allows_copy(PRFL_BLIND, 0, 9));
    TEST_PASS();
}

void test_gate_basic_never_copies(void)
{
    TEST_START("BASIC/NONE never copy, even with a long A+ payload");
    ASSERT_TRUE(!gate_allows_copy(PRFL_BASIC, 0, 9));
    ASSERT_TRUE(!gate_allows_copy(PRFL_BASIC, 0, 64));
    ASSERT_TRUE(!gate_allows_copy(PRFL_NONE, 0, 64));
    TEST_PASS();
}

void test_gate_legacy_never_copies(void)
{
    TEST_START("legacy mode never copies (raw DMA size = padding risk)");
    /* A legacy master clocking a fixed 64-byte transaction passes the size
     * check — the legacy gate is what protects blindCfg from bus padding. */
    ASSERT_TRUE(!gate_allows_copy(PRFL_BLIND, 1, 64));
    ASSERT_TRUE(!gate_allows_copy(PRFL_BASIC, 1, 64));
    TEST_PASS();
}

void test_gate_short_payload_never_copies(void)
{
    TEST_START("A+ BLIND with id only or truncated ctx -> no copy");
    ASSERT_TRUE(!gate_allows_copy(PRFL_BLIND, 0, 2));  /* id only */
    ASSERT_TRUE(!gate_allows_copy(PRFL_BLIND, 0, 8));  /* 6 of 7 ctx bytes */
    TEST_PASS();
}

/* Mirror of the READTXHDLR response packing (memcpy of int16 on a
 * little-endian target — CM4 and the test hosts are both LE). */
static void pack_hdlr(int16_t hdlr, uint8_t out[2])
{
    memcpy(out, &hdlr, sizeof(hdlr));
}

void test_readtxhdlr_wire_format_le(void)
{
    TEST_START("CMD_READ_TX_HDLR returns int16 little-endian");
    uint8_t b[2];

    pack_hdlr(-1, b);          /* no TX yet */
    ASSERT_EQ(0xFF, b[0]);
    ASSERT_EQ(0xFF, b[1]);

    pack_hdlr(5, b);
    ASSERT_EQ(0x05, b[0]);
    ASSERT_EQ(0x00, b[1]);

    pack_hdlr(0x7FFF, b);      /* int16 max */
    ASSERT_EQ(0xFF, b[0]);
    ASSERT_EQ(0x7F, b[1]);
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("SPI WRITE_KMAC BLIND gate + READ_TX_HDLR format");
    test_gate_blind_aplus_full_ctx_copies();
    test_gate_basic_never_copies();
    test_gate_legacy_never_copies();
    test_gate_short_payload_never_copies();
    test_readtxhdlr_wire_format_le();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
