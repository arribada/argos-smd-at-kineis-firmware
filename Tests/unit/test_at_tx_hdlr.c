/**
 * @file    test_at_tx_hdlr.c
 * @brief   Contract test for the +TX response format with the appended Kineis
 *          frame handler.
 *
 * SMD appends the frame handler at the END of the +TX line:
 *     +TX=<code>,<data>,<hdlr>
 * This is a DELIBERATE divergence from the KIM reference (krd_fw v11.1.0), which
 * puts the handler FIRST (+TX=<hdlr>,<code>,<data>) and thereby breaks backward
 * compatibility. Appending keeps <code> and <data> in their original positions,
 * so a legacy host that parses only the first two fields is unaffected.
 *
 * Mirrors the format built in mgr_at_cmd_common.c bMGR_AT_CMD_sendResponse
 * (ATCMD_RSP_TXOK and the TX/RX error branch). Standalone by design — the test
 * harness compiles each test alone (-Iunit), like test_at_dispatch.c.
 */

#include "test_framework.h"
#include <stdio.h>

/* Mirror of the real success/error +TX formatters (mgr_at_cmd_common.c:72,95). */
static void fmt_txok(char *buf, size_t n, const char *data_hex, int hdlr)
{
    snprintf(buf, n, "+TX=0,%s,%d\r\n", data_hex, hdlr);
}
static void fmt_txerr(char *buf, size_t n, int err, const char *data_hex, int hdlr)
{
    snprintf(buf, n, "+TX=%d,%s,%d\r\n", err, data_hex, hdlr);
}

/* A legacy host parser that only knows +TX=<code>,<data> — reads the first two
 * comma fields and ignores any trailing field. Returns code, copies data. */
static int legacy_parse(const char *line, char *data_out, size_t data_n)
{
    int code = -1;
    /* Expect "+TX=" prefix */
    if (strncmp(line, "+TX=", 4) != 0) return -999;
    const char *p = line + 4;
    code = atoi(p);                         /* field 0 = code (unchanged) */
    const char *c1 = strchr(p, ',');
    if (!c1) return -998;
    const char *d = c1 + 1;                 /* field 1 = data (unchanged) */
    size_t i = 0;
    while (d[i] && d[i] != ',' && i < data_n - 1) { data_out[i] = d[i]; i++; }
    data_out[i] = '\0';
    return code;
}

void test_txok_format_has_handler_last(void)
{
    TEST_START("+TX success = +TX=0,<data>,<hdlr> (handler appended last)");
    char buf[64];
    fmt_txok(buf, sizeof(buf), "11223344", 5);
    ASSERT_TRUE(strcmp(buf, "+TX=0,11223344,5\r\n") == 0);
    TEST_PASS();
}

void test_txerr_format_has_handler_last(void)
{
    TEST_START("+TX error = +TX=<err>,<data>,<hdlr>");
    char buf[64];
    fmt_txerr(buf, sizeof(buf), 3, "AABB", 7);   /* 3 = KNS_STATUS_TIMEOUT */
    ASSERT_TRUE(strcmp(buf, "+TX=3,AABB,7\r\n") == 0);
    TEST_PASS();
}

void test_legacy_parser_still_reads_code_and_data(void)
{
    TEST_START("legacy <code>,<data> parser unaffected by trailing handler");
    char buf[64], data[32];
    fmt_txok(buf, sizeof(buf), "DEADBEEF", 42);
    int code = legacy_parse(buf, data, sizeof(data));
    ASSERT_EQ(0, code);                           /* code still field 0 */
    ASSERT_TRUE(strcmp(data, "DEADBEEF") == 0);   /* data still field 1 */
    TEST_PASS();
}

void test_handler_negative_default(void)
{
    TEST_START("handler defaults to -1 when none captured");
    char buf[64];
    fmt_txok(buf, sizeof(buf), "00", -1);
    ASSERT_TRUE(strcmp(buf, "+TX=0,00,-1\r\n") == 0);
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("AT +TX handler appended (backward-compatible)");
    test_txok_format_has_handler_last();
    test_txerr_format_has_handler_last();
    test_legacy_parser_still_reads_code_and_data();
    test_handler_negative_default();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
