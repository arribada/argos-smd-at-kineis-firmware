/**
 * @file    test_at_dispatch.c
 * @brief   Unit tests for AT command prefix dispatch (longest-match).
 *
 * Guards against the regression where "break on first prefix match"
 * shadowed longer commands with the same prefix.
 *
 * Real bug confirmed live 2026-06-05: in builds with both AT+TX (5 chars)
 * and AT+TXCFG / AT+TXSTATS (longer prefix), AT+TX was listed first in the
 * table → matched first → terminator check ('C' or 'S' not in {'=','\r','\n'})
 * → return UNKNOWN_AT_CMD. AT+TXCFG and AT+TXSTATS were completely unreachable.
 *
 * Fix in mgr_at_cmd.c: scan ALL entries and pick the LONGEST matching prefix.
 */

#include "test_framework.h"

/* Mirror of atcmd_desc_t (mgr_at_cmd_list.h) */
typedef struct {
    const char *name;
    uint8_t len;
} atcmd_t;

/* Mirror of the real AT command table with the realistic ordering that
 * triggered the original bug (AT+TX listed BEFORE AT+TXCFG/STATS). */
static const atcmd_t TABLE[] = {
    { "AT+VERSION",    10 },
    { "AT+PING",        7 },
    { "AT+TX",          5 },   /* general — short, listed first */
    { "AT+SWSFORCE",   11 },
    { "AT+SWSCFG",      9 },
    { "AT+SWS",         6 },
    { "AT+TXCFG",       8 },   /* longer, listed AFTER AT+TX */
    { "AT+TXSTATS",    10 },
    { "AT+TEST",        7 },
    { "AT+SAVE_RCONF", 13 },
    { "AT+SAVE",        7 },
    { "AT+RATECLEAR",  12 },
    { "AT+RATECFG",    10 },
    { "AT+RATE",        7 },
};
#define TABLE_LEN (sizeof(TABLE) / sizeof(TABLE[0]))

/* Mirror of the FIXED dispatcher: longest prefix match. */
static int dispatch(const char *input)
{
    int best_idx = -1;
    uint8_t best_len = 0;
    for (size_t i = 0; i < TABLE_LEN; i++) {
        if (TABLE[i].len <= best_len) continue;
        if (strncmp(TABLE[i].name, input, TABLE[i].len) == 0) {
            best_idx = (int)i;
            best_len = TABLE[i].len;
        }
    }
    return best_idx;
}

/* Find by name for assertion convenience. */
static int idx_of(const char *name)
{
    for (size_t i = 0; i < TABLE_LEN; i++)
        if (strcmp(TABLE[i].name, name) == 0) return (int)i;
    return -1;
}

void test_short_command_matches(void)
{
    TEST_START("AT+TX action mode resolves to AT+TX");
    ASSERT_EQ(idx_of("AT+TX"), dispatch("AT+TX\r\n"));
    TEST_PASS();
}

void test_short_with_param_matches(void)
{
    TEST_START("AT+TX=11223344,0x02 resolves to AT+TX (not TXCFG/STATS)");
    ASSERT_EQ(idx_of("AT+TX"), dispatch("AT+TX=11223344,0x02"));
    TEST_PASS();
}

void test_long_command_not_shadowed_by_short_prefix(void)
{
    TEST_START("AT+TXCFG=? resolves to AT+TXCFG (NOT shadowed by AT+TX)");
    ASSERT_EQ(idx_of("AT+TXCFG"), dispatch("AT+TXCFG=?"));
    TEST_PASS();
}

void test_longer_command_not_shadowed(void)
{
    TEST_START("AT+TXSTATS=? resolves to AT+TXSTATS (NOT shadowed by AT+TX)");
    ASSERT_EQ(idx_of("AT+TXSTATS"), dispatch("AT+TXSTATS=?"));
    TEST_PASS();
}

void test_swsforce_not_shadowed_by_sws(void)
{
    TEST_START("AT+SWSFORCE resolves to AT+SWSFORCE (NOT shadowed by AT+SWS)");
    ASSERT_EQ(idx_of("AT+SWSFORCE"), dispatch("AT+SWSFORCE\r\n"));
    TEST_PASS();
}

void test_swscfg_not_shadowed_by_sws(void)
{
    TEST_START("AT+SWSCFG=? resolves to AT+SWSCFG (NOT shadowed by AT+SWS)");
    ASSERT_EQ(idx_of("AT+SWSCFG"), dispatch("AT+SWSCFG=?"));
    TEST_PASS();
}

void test_save_rconf_not_shadowed_by_save(void)
{
    TEST_START("AT+SAVE_RCONF resolves to AT+SAVE_RCONF (NOT shadowed by AT+SAVE)");
    ASSERT_EQ(idx_of("AT+SAVE_RCONF"), dispatch("AT+SAVE_RCONF\r\n"));
    TEST_PASS();
}

void test_rateclear_not_shadowed_by_rate(void)
{
    TEST_START("AT+RATECLEAR resolves to AT+RATECLEAR");
    ASSERT_EQ(idx_of("AT+RATECLEAR"), dispatch("AT+RATECLEAR\r\n"));
    TEST_PASS();
}

void test_ratecfg_not_shadowed_by_rate(void)
{
    TEST_START("AT+RATECFG=? resolves to AT+RATECFG");
    ASSERT_EQ(idx_of("AT+RATECFG"), dispatch("AT+RATECFG=?"));
    TEST_PASS();
}

void test_unknown_returns_minus1(void)
{
    TEST_START("Unknown command returns -1");
    ASSERT_EQ(-1, dispatch("AT+NOTACOMMAND\r\n"));
    TEST_PASS();
}

void test_partial_prefix_not_matched(void)
{
    TEST_START("AT+TXC (5 chars) doesn't accidentally match AT+TX without terminator check");
    /* The dispatcher returns the index — the terminator check happens
     * elsewhere. But "AT+TXC" first 5 chars are "AT+TX" so AT+TX matches
     * here. The CALLER then checks the char at offset 5 ('C') and rejects. */
    int idx = dispatch("AT+TXC\r\n");
    ASSERT_EQ(idx_of("AT+TX"), idx);
    /* Now simulate the terminator check that the real code does. */
    const char *input = "AT+TXC\r\n";
    char term = input[TABLE[idx].len];
    ASSERT_TRUE(term != '=' && term != '\r' && term != '\n');
    /* The real code would then set ATcmdIndex = UNKNOWN. */
    TEST_PASS();
}

void test_test_command_matches(void)
{
    TEST_START("AT+TEST=5 resolves to AT+TEST (not shadowed by AT+TX since 'E' != 'X')");
    ASSERT_EQ(idx_of("AT+TEST"), dispatch("AT+TEST=5"));
    TEST_PASS();
}

int main(void)
{
    TEST_SUITE_START("AT command longest-prefix dispatch");
    test_short_command_matches();
    test_short_with_param_matches();
    test_long_command_not_shadowed_by_short_prefix();
    test_longer_command_not_shadowed();
    test_swsforce_not_shadowed_by_sws();
    test_swscfg_not_shadowed_by_sws();
    test_save_rconf_not_shadowed_by_save();
    test_rateclear_not_shadowed_by_rate();
    test_ratecfg_not_shadowed_by_rate();
    test_unknown_returns_minus1();
    test_partial_prefix_not_matched();
    test_test_command_matches();
    TEST_SUITE_END();
    return tests_failed ? 1 : 0;
}
