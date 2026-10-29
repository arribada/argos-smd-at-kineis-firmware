/**
 * @file    test_at_tx_queue.c
 * @brief   AT+TX user-data queue: every rejected command must give its slot back.
 *
 * Compiles the REAL firmware sources (user_data.c, mgr_at_cmd_common.c,
 * mgr_at_cmd_list_user_data.c, strutil_lib.c) against host stubs of the HAL,
 * the Kineis queues and the UART console. Include paths come from
 * test_at_tx_queue.flags.
 *
 * Bug guarded: bMGR_AT_CMD_handleNewTxData reserves a queue slot before
 * parsing; the rejection paths returned without releasing it, so 4 rejected
 * AT+TX locked the queue (+ERROR=1101) until a power cycle.
 */

#include "test_framework.h"
#include <stdarg.h>

#include "../../Kineis/App/Libs/USERDATA/Src/user_data.c"
#include "../../Kineis/App/Libs/STRUTIL/Src/strutil_lib.c"
#include "../../Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd_common.c"
#include "../../Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd_list_user_data.c"

/* ---- Host stubs -------------------------------------------------------- */

/* No USERDATA client is registered in the firmware (USERDATA_RAT_MAX == 0);
 * the firmware build drops the loops at -O1, the -O0 host build needs it. */
struct sUserDataClientCb_t sUserDataClientCb[1];

static char g_out[4096];
static size_t g_out_len;
static int g_assert_count;
static int g_push_count;
static enum KNS_status_t g_push_status;
static struct KNS_MAC_appEvt_t g_last_push;
static bool g_pop_pending;
static struct KNS_MAC_srvcEvt_t g_pop_evt;
static enum KNS_tx_mod_t g_modulation;

void KNS_CS_enter(void) {}
void KNS_CS_exit(void) {}
void HAL_Delay(uint32_t Delay) { (void)Delay; }
void MCU_MISC_TCXO_Force_State(bool enable) { (void)enable; }
void MCU_MISC_TCXO_get_warmup(uint32_t *time_ms) { *time_ms = 0; }
void MCU_MISC_turn_off_pa(void) {}

void kns_assert_failed(uint8_t *file, uint32_t line)
{
    printf("\n    kns_assert at %s:%u\n", (const char *)file, (unsigned)line);
    g_assert_count++;
}

void MCU_AT_CONSOLE_send(const char *format, ...)
{
    va_list ap;
    int n;

    va_start(ap, format);
    n = vsnprintf(g_out + g_out_len, sizeof(g_out) - g_out_len, format, ap);
    va_end(ap);
    if (n > 0 && g_out_len + (size_t)n < sizeof(g_out))
        g_out_len += (size_t)n;
}

void MCU_AT_CONSOLE_send_dataBuf(uint8_t *pu8_inDataBuff, uint16_t u16_dataLenBit)
{
    uint16_t i;

    for (i = 0; i < (uint16_t)((u16_dataLenBit + 7u) / 8u); i++)
        MCU_AT_CONSOLE_send("%02X", pu8_inDataBuff[i]);
}

enum KNS_status_t KNS_Q_push(enum KNS_Q_handle_t qHandle, void *qItem)
{
    if (qHandle == KNS_Q_DL_APP2MAC && g_push_status == KNS_STATUS_OK) {
        g_push_count++;
        memcpy(&g_last_push, qItem, sizeof(g_last_push));
    }
    return g_push_status;
}

enum KNS_status_t KNS_Q_pop(enum KNS_Q_handle_t qHandle, void *qItem)
{
    if (qHandle != KNS_Q_UL_MAC2APP || !g_pop_pending)
        return KNS_STATUS_QEMPTY;
    memcpy(qItem, &g_pop_evt, sizeof(g_pop_evt));
    g_pop_pending = false;
    return KNS_STATUS_OK;
}

enum KNS_status_t KNS_CFG_getRadioInfo(struct KNS_CFG_radio_t *radio_cfg)
{
    memset(radio_cfg, 0, sizeof(*radio_cfg));
    radio_cfg->modulation = g_modulation;
    return KNS_STATUS_OK;
}

/* ---- Helpers ----------------------------------------------------------- */

static void reset_all(void)
{
    uint8_t i;

    for (i = 0; i < USERDATA_TX_FIFO_SIZE; i++)
        sUserDataTxFifoBuf[i] = sUserDataTxEltDflt;
    sUserDataTxFifo.spFirst = NULL;

    g_out[0] = '\0';
    g_out_len = 0;
    g_assert_count = 0;
    g_push_count = 0;
    g_push_status = KNS_STATUS_OK;
    g_pop_pending = false;
    g_modulation = KNS_TX_MOD_LDA2;
}

static void clear_out(void)
{
    g_out[0] = '\0';
    g_out_len = 0;
}

static bool out_has(const char *s)
{
    return strstr(g_out, s) != NULL;
}

/* Slots not held by anyone: the direct measure of a leak. */
static int free_slots(void)
{
    int n = 0;
    uint8_t i;

    for (i = 0; i < USERDATA_TX_FIFO_SIZE; i++)
        if (!sUserDataTxFifoBuf[i].bIsToBeTransmit)
            n++;
    return n;
}

static bool at_tx(const char *cmd)
{
    char buf[128];

    snprintf(buf, sizeof(buf), "%s", cmd);
    return bMGR_AT_CMD_TX_cmd((uint8_t *)buf, ATCMD_ACTION_MODE);
}

static void mac_tx_done(int16_t hdlr)
{
    memset(&g_pop_evt, 0, sizeof(g_pop_evt));
    g_pop_evt.id = KNS_MAC_TX_DONE;
    g_pop_evt.tx_ctxt.frm_hdlr = hdlr;
    g_pop_pending = true;
    (void)MGR_AT_CMD_macEvtProcess();
}

static char g_err1100[24], g_err1101[24], g_err1201[24], g_err1203[24], g_err300[24];

/* ---- Tests ------------------------------------------------------------- */

static void test_error_codes_pinned(void)
{
    TEST_START("error codes reported to the host are unchanged");
    ASSERT_EQ(1100, ERROR_INVALID_USER_DATA_LENGTH);
    ASSERT_EQ(1101, ERROR_DATA_QUEUE_FULL);
    ASSERT_EQ(1201, ERROR_MISSING_PARAMETERS);
    ASSERT_EQ(1203, ERROR_UNKNOWN_AT_CMD);
    ASSERT_EQ(4, USERDATA_TX_FIFO_SIZE);
    TEST_PASS();
}

static void test_valid_tx_holds_slot_until_tx_done(void)
{
    TEST_START("valid AT+TX keeps its slot until TX_DONE, then +TX frees it");
    reset_all();
    ASSERT_TRUE(at_tx("AT+TX=11223344"));
    ASSERT_EQ(1, g_push_count);
    ASSERT_EQ(32, g_last_push.data_ctxt.usrdata_bitlen);
    ASSERT_EQ(3, free_slots());
    ASSERT_EQ(1, USERDATA_txFifoGetCount());
    ASSERT_FALSE(out_has("+ERROR"));

    mac_tx_done(7);
    ASSERT_TRUE(out_has("+TX=0,11223344,7\r\n"));
    ASSERT_EQ(4, free_slots());
    ASSERT_EQ(0, USERDATA_txFifoGetCount());
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_non_hex_payload_releases_slot(void)
{
    int i;

    TEST_START("4x non-hex payload (+ERROR=1201) then valid AT+TX is accepted");
    reset_all();
    for (i = 0; i < 4; i++) {
        clear_out();
        ASSERT_FALSE(at_tx("AT+TX=zz"));
        ASSERT_TRUE(out_has(g_err1201));
        ASSERT_EQ(4, free_slots());
    }
    clear_out();
    ASSERT_TRUE(at_tx("AT+TX=11223344"));
    ASSERT_FALSE(out_has(g_err1101));
    ASSERT_EQ(1, g_push_count);
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_empty_payload_releases_slot(void)
{
    int i;

    TEST_START("4x empty payload (+ERROR=1201) then valid AT+TX is accepted");
    reset_all();
    for (i = 0; i < 4; i++) {
        clear_out();
        ASSERT_FALSE(at_tx("AT+TX="));
        ASSERT_TRUE(out_has(g_err1201));
    }
    ASSERT_EQ(4, free_slots());
    clear_out();
    ASSERT_TRUE(at_tx("AT+TX=AABBCCDD,0x00"));
    ASSERT_FALSE(out_has(g_err1101));
    ASSERT_EQ(1, g_push_count);
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_oversized_payload_releases_slot(void)
{
    int i;

    TEST_START("4x payload too long for LDK (+ERROR=1100) then valid AT+TX");
    reset_all();
    g_modulation = KNS_TX_MOD_LDK;
    for (i = 0; i < 4; i++) {
        clear_out();
        /* 17 bytes = 136 bits > 128-bit LDK frame */
        ASSERT_FALSE(at_tx("AT+TX=0102030405060708090A0B0C0D0E0F1011"));
        ASSERT_TRUE(out_has(g_err1100));
    }
    ASSERT_EQ(4, free_slots());
    clear_out();
    ASSERT_TRUE(at_tx("AT+TX=0102030405060708090A0B0C0D0E0F10"));
    ASSERT_FALSE(out_has("+ERROR"));
    ASSERT_EQ(128, g_last_push.data_ctxt.usrdata_bitlen);
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_lda2l_pads_to_196(void)
{
    TEST_START("LDA2L: AT+TX padded to the 196-bit frame the encoder accepts");
    reset_all();
    g_modulation = KNS_TX_MOD_LDA2L;
    ASSERT_TRUE(at_tx("AT+TX=11223344"));
    ASSERT_EQ(196, g_last_push.data_ctxt.usrdata_bitlen);
    clear_out();
    /* 24 bytes = 192 bits: max full-byte payload, still a 196-bit frame */
    ASSERT_TRUE(at_tx("AT+TX=0102030405060708090A0B0C0D0E0F101112131415161718"));
    ASSERT_FALSE(out_has("+ERROR"));
    ASSERT_EQ(196, g_last_push.data_ctxt.usrdata_bitlen);
    clear_out();
    /* 25 bytes = 200 bits > 196 */
    ASSERT_FALSE(at_tx("AT+TX=0102030405060708090A0B0C0D0E0F10111213141516171819"));
    ASSERT_TRUE(out_has(g_err1100));
    ASSERT_EQ(2, free_slots());
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_other_modulations_padding_unchanged(void)
{
    TEST_START("LDA2 / VLDA4 padding unchanged (32-bit step, 24-bit frame)");
    reset_all();
    g_modulation = KNS_TX_MOD_LDA2;
    ASSERT_TRUE(at_tx("AT+TX=1122334455"));
    ASSERT_EQ(64, g_last_push.data_ctxt.usrdata_bitlen);
    g_modulation = KNS_TX_MOD_VLDA4;
    ASSERT_TRUE(at_tx("AT+TX=112233"));
    ASSERT_EQ(24, g_last_push.data_ctxt.usrdata_bitlen);
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_mac_queue_full_releases_and_unlinks(void)
{
    TEST_START("MAC queue push failure: slot freed and unlinked from FIFO");
    reset_all();
    g_push_status = KNS_STATUS_QFULL;
    ASSERT_FALSE(at_tx("AT+TX=11223344"));
    ASSERT_TRUE(out_has(g_err300));
    ASSERT_EQ(4, free_slots());
    ASSERT_EQ(0, USERDATA_txFifoGetCount());
    ASSERT_TRUE(USERDATA_txFifoGetFirst() == NULL);

    g_push_status = KNS_STATUS_OK;
    clear_out();
    ASSERT_TRUE(at_tx("AT+TX=11223344"));
    ASSERT_EQ(1, USERDATA_txFifoGetCount());
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_queue_full_behaviour_unchanged(void)
{
    int i;

    TEST_START("4 pending valid TX: 5th gets 1101, TX_DONE makes room again");
    reset_all();
    for (i = 0; i < 4; i++)
        ASSERT_TRUE(at_tx("AT+TX=11223344"));
    ASSERT_EQ(0, free_slots());
    ASSERT_EQ(4, USERDATA_txFifoGetCount());

    clear_out();
    ASSERT_FALSE(at_tx("AT+TX=55667788"));
    ASSERT_TRUE(out_has(g_err1101));
    ASSERT_EQ(4, USERDATA_txFifoGetCount());
    ASSERT_EQ(4, g_push_count);

    mac_tx_done(1);
    ASSERT_EQ(1, free_slots());
    clear_out();
    ASSERT_TRUE(at_tx("AT+TX=55667788"));
    ASSERT_FALSE(out_has("+ERROR"));
    ASSERT_EQ(0, free_slots());
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_rejection_does_not_disturb_pending(void)
{
    TEST_START("rejected AT+TX between valid ones leaves pending data intact");
    reset_all();
    ASSERT_TRUE(at_tx("AT+TX=A1A2A3A4"));
    ASSERT_TRUE(at_tx("AT+TX=B1B2B3B4"));
    ASSERT_FALSE(at_tx("AT+TX=zz"));
    ASSERT_TRUE(at_tx("AT+TX=C1C2C3C4"));
    ASSERT_EQ(3, USERDATA_txFifoGetCount());
    ASSERT_EQ(1, free_slots());

    clear_out();
    mac_tx_done(1);
    ASSERT_TRUE(out_has("+TX=0,A1A2A3A4,1\r\n"));
    clear_out();
    mac_tx_done(2);
    ASSERT_TRUE(out_has("+TX=0,B1B2B3B4,2\r\n"));
    clear_out();
    mac_tx_done(3);
    ASSERT_TRUE(out_has("+TX=0,C1C2C3C4,3\r\n"));
    ASSERT_EQ(4, free_slots());
    ASSERT_EQ(0, USERDATA_txFifoGetCount());
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_mixed_fill_then_1101(void)
{
    TEST_START("3 pending + 1 rejected + 1 valid fills 4, next one gets 1101");
    reset_all();
    ASSERT_TRUE(at_tx("AT+TX=11111111"));
    ASSERT_TRUE(at_tx("AT+TX=22222222"));
    ASSERT_TRUE(at_tx("AT+TX=33333333"));
    ASSERT_FALSE(at_tx("AT+TX="));
    ASSERT_TRUE(at_tx("AT+TX=44444444"));
    ASSERT_EQ(0, free_slots());
    clear_out();
    ASSERT_FALSE(at_tx("AT+TX=55555555"));
    ASSERT_TRUE(out_has(g_err1101));
    ASSERT_EQ(0, g_assert_count);
    TEST_PASS();
}

static void test_status_mode_reserves_nothing(void)
{
    char buf[] = "AT+TX=?";

    TEST_START("AT+TX=? is refused (+ERROR=1203) without touching the queue");
    reset_all();
    ASSERT_FALSE(bMGR_AT_CMD_TX_cmd((uint8_t *)buf, ATCMD_STATUS_MODE));
    ASSERT_TRUE(out_has(g_err1203));
    ASSERT_EQ(4, free_slots());
    ASSERT_EQ(0, g_push_count);
    TEST_PASS();
}

int main(void)
{
    snprintf(g_err1100, sizeof(g_err1100), "+ERROR=%d\r\n", (int)ERROR_INVALID_USER_DATA_LENGTH);
    snprintf(g_err1101, sizeof(g_err1101), "+ERROR=%d\r\n", (int)ERROR_DATA_QUEUE_FULL);
    snprintf(g_err1201, sizeof(g_err1201), "+ERROR=%d\r\n", (int)ERROR_MISSING_PARAMETERS);
    snprintf(g_err1203, sizeof(g_err1203), "+ERROR=%d\r\n", (int)ERROR_UNKNOWN_AT_CMD);
    snprintf(g_err300, sizeof(g_err300), "+ERROR=%d\r\n", (int)KNS_STATUS_QFULL);

    printf("\n=== AT+TX queue slot release ===\n");
    test_error_codes_pinned();
    test_valid_tx_holds_slot_until_tx_done();
    test_non_hex_payload_releases_slot();
    test_empty_payload_releases_slot();
    test_oversized_payload_releases_slot();
    test_lda2l_pads_to_196();
    test_other_modulations_padding_unchanged();
    test_mac_queue_full_releases_and_unlinks();
    test_queue_full_behaviour_unchanged();
    test_rejection_does_not_disturb_pending();
    test_mixed_fill_then_1101();
    test_status_mode_reserves_nothing();

    printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
    return tests_failed ? 1 : 0;
}
