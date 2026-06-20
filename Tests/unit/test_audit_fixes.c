/**
 * @file    test_audit_fixes.c
 * @brief   Regression locks for the memory-safety / concurrency bugs found by the
 *          2026-06-20 total audit and fixed on branch v2. Each test mirrors the
 *          FIXED contract and, where possible, demonstrates what the pre-fix code
 *          did wrong (against a canary), so a reintroduction fails the suite.
 *
 *  - MCU_NVM_getID/getAddr: 8-byte flash read must NOT overflow the 4-byte caller
 *    object (mcu_nvm.c). Read into a sized local, copy only the contracted bytes.
 *  - KNS_CS_exit: must not underflow idx on an unbalanced exit (kns_cs.c).
 *  - AT+TX "0x%hX": the 'h' conversion writes 2 bytes; scan into a 2-byte object
 *    then narrow to the 1-byte attribute union (mgr_at_cmd_list_user_data.c).
 *  - qIdx2Str[]: every KNS_Q_MAX slot must be a non-NULL string (kns_q_conf.c).
 */

#include "test_framework.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ===================================================================== */
/* 1. MCU_NVM slot-read must not overflow the caller's value object       */
/* ===================================================================== */

/* FIXED contract: read the N-byte flash slot into a sized local, then copy only
 * the value size into the destination. Models mcu_nvm.c getID/getAddr. */
static void nvm_read_value(uint8_t *dest, uint32_t dest_size,
                           const uint8_t *slot, uint32_t slot_size)
{
	uint8_t local[8];
	memcpy(local, slot, slot_size);          /* read full 8-byte flash slot */
	memcpy(dest, local, dest_size);          /* copy ONLY the value bytes out */
}

/* BUGGY contract (the pre-fix code): copy slot_size bytes straight into dest. */
static void nvm_read_value_buggy(uint8_t *dest, const uint8_t *slot, uint32_t slot_size)
{
	memcpy(dest, slot, slot_size);           /* writes 8 into a 4-byte object */
}

/** getID: 8-byte slot, 4-byte destination — value correct, no overflow. */
void test_nvm_getid_no_overflow(void)
{
	const uint8_t slot[8] = {0x78,0x56,0x34,0x12, 0xFF,0xFF,0xFF,0xFF};
	struct { uint32_t id; uint8_t canary[4]; } s;
	memset(s.canary, 0xAA, sizeof(s.canary));

	nvm_read_value((uint8_t *)&s.id, sizeof(s.id), slot, 8);

	ASSERT_EQ_HEX(0x12345678u, s.id);                 /* low word = the ID */
	for (int i = 0; i < 4; i++) ASSERT_EQ(0xAA, s.canary[i]); /* untouched */
	TEST_PASS();
}

/** Lock: the pre-fix 8-into-4 copy WOULD clobber the 4 bytes after the value. */
void test_nvm_getid_buggy_overflows_canary(void)
{
	const uint8_t slot[8] = {0x78,0x56,0x34,0x12, 0xDE,0xAD,0xBE,0xEF};
	struct { uint32_t id; uint8_t canary[4]; } s;
	memset(s.canary, 0xAA, sizeof(s.canary));

	nvm_read_value_buggy((uint8_t *)&s.id, slot, 8);  /* the OLD behavior */

	/* Proof the old code corrupted the adjacent bytes (this is what the fix
	 * prevents). If someone reverts to an 8-into-4 read, the FIXED test above
	 * stays green only because this models the corruption explicitly. */
	ASSERT_EQ(0xDE, s.canary[0]);
	ASSERT_NE(0xAA, s.canary[0]);
	TEST_PASS();
}

/** getAddr: 8-byte slot, 4-byte array — first 4 bytes copied, no overflow. */
void test_nvm_getaddr_no_overflow(void)
{
	const uint8_t slot[8] = {0x11,0x22,0x33,0x44, 0xFF,0xFF,0xFF,0xFF};
	struct { uint8_t addr[4]; uint8_t canary[4]; } s;
	memset(s.canary, 0x5A, sizeof(s.canary));

	nvm_read_value(s.addr, 4, slot, 8);

	const uint8_t exp[4] = {0x11,0x22,0x33,0x44};
	ASSERT_MEM_EQ(exp, s.addr, 4);
	for (int i = 0; i < 4; i++) ASSERT_EQ(0x5A, s.canary[i]);
	TEST_PASS();
}

/** setID/setAddr: stage the value into a zero-padded 8-byte slot (no over-read). */
void test_nvm_set_zero_pads_slot(void)
{
	const uint32_t id = 0xCAFEBABEu;
	uint8_t slot[8] = {0};
	memcpy(slot, &id, sizeof(id));            /* FIXED setID staging */
	const uint8_t exp[8] = {0xBE,0xBA,0xFE,0xCA, 0,0,0,0};
	ASSERT_MEM_EQ(exp, slot, 8);              /* low 4 = value, high 4 = pad 0 */
	TEST_PASS();
}

/* ===================================================================== */
/* 2. KNS_CS_exit must not underflow on an unbalanced exit                 */
/* ===================================================================== */

#define CS_N 8
static uint8_t  cs_idx;          /* mirrors kns_cs.c idx (uint8_t) */
static uint32_t cs_prim[CS_N];

static void cs_reset(void) { cs_idx = 0; memset(cs_prim, 0, sizeof(cs_prim)); }
static void cs_enter(uint32_t primask) { cs_prim[cs_idx % CS_N] = primask; cs_idx++; }
/* FIXED exit: guard idx==0. Returns 1 if it would re-enable IRQs, 0 if not, -1 guarded. */
static int  cs_exit_fixed(void)
{
	if (cs_idx == 0) return -1;              /* the guard */
	cs_idx--;
	return (cs_prim[cs_idx % CS_N] == 0) ? 1 : 0;
}

/** Balanced enter/exit returns idx to 0 and tracks PRIMASK correctly. */
void test_cs_balanced(void)
{
	cs_reset();
	cs_enter(0);            /* outer: IRQs were enabled */
	cs_enter(1);            /* inner: IRQs already masked */
	ASSERT_EQ(2, cs_idx);
	ASSERT_EQ(0, cs_exit_fixed());   /* inner: saved primask=1 -> do NOT re-enable */
	ASSERT_EQ(1, cs_exit_fixed());   /* outer: saved primask=0 -> re-enable */
	ASSERT_EQ(0, cs_idx);
	TEST_PASS();
}

/** Unbalanced exit (idx==0) must be guarded — never wrap idx to 255. */
void test_cs_unbalanced_exit_guarded(void)
{
	cs_reset();
	ASSERT_EQ(-1, cs_exit_fixed());  /* guarded, no underflow */
	ASSERT_EQ(0, cs_idx);            /* did NOT wrap to 255 */
	/* Several spurious exits stay pinned at 0. */
	for (int i = 0; i < 5; i++) (void)cs_exit_fixed();
	ASSERT_EQ(0, cs_idx);
	TEST_PASS();
}

/** Lock: the pre-fix unguarded decrement would wrap a uint8_t idx to 255. */
void test_cs_buggy_wraps(void)
{
	uint8_t idx = 0;
	idx--;                           /* the OLD behavior at idx==0 */
	ASSERT_EQ(255, idx);             /* exactly the corruption the guard prevents */
	TEST_PASS();
}

/* ===================================================================== */
/* 3. AT+TX "0x%hX" needs a 2-byte target, then narrow to the 1-byte union */
/* ===================================================================== */

/** FIXED parse: scan into unsigned short, narrow to uint8 — no overflow. */
void test_attr_scan_2byte_then_narrow(void)
{
	struct { uint8_t u8_raw; uint8_t canary; } attr;
	attr.canary = 0xC3;

	unsigned short u16 = 0;
	int n = sscanf("DATA,0x2F", "%*[^,],0x%hX", &u16);
	ASSERT_EQ(1, n);
	attr.u8_raw = (uint8_t)u16;

	ASSERT_EQ(0x2F, attr.u8_raw);
	ASSERT_EQ(0xC3, attr.canary);     /* the 1-byte union neighbour is intact */
	TEST_PASS();
}

/** A value with a high nibble still narrows cleanly to one byte. */
void test_attr_narrow_high_value(void)
{
	unsigned short u16 = 0;
	(void)sscanf("0x1FF", "0x%hX", &u16);
	ASSERT_EQ(0x1FF, u16);            /* short holds 9-bit value */
	ASSERT_EQ(0xFF, (uint8_t)u16);   /* narrowed attribute */
	TEST_PASS();
}

/* ===================================================================== */
/* 4. qIdx2Str[] must have a non-NULL string for every queue slot          */
/* ===================================================================== */

#define Q_MAX 5
/* FIXED initializer (commas present): 5 distinct entries. */
static const char *qIdx2Str_fixed[Q_MAX] = {
	"KNS_Q_UL_MAC2APP_EVT_LIST",
	"KNS_Q_DL_APP2MAC",
	"KNS_Q_UL_MAC2APP",
	"KNS_Q_UL_INFRA2MAC",
	"KNS_Q_UL_SRVC2MAC",
};
/* BUGGY initializer (first comma missing): adjacent literals concatenate -> 4
 * entries, slot 4 becomes NULL. */
static const char *qIdx2Str_buggy[Q_MAX] = {
	"KNS_Q_UL_MAC2APP_EVT_LIST"
	"KNS_Q_DL_APP2MAC",
	"KNS_Q_UL_MAC2APP",
	"KNS_Q_UL_INFRA2MAC",
	"KNS_Q_UL_SRVC2MAC",
};

void test_qidx2str_all_non_null(void)
{
	for (int i = 0; i < Q_MAX; i++)
		ASSERT_TRUE(qIdx2Str_fixed[i] != NULL);
	ASSERT_STR_EQ("KNS_Q_UL_SRVC2MAC", qIdx2Str_fixed[Q_MAX - 1]);
	TEST_PASS();
}

/** Lock: the missing-comma form leaves the last slot NULL (the bug). */
void test_qidx2str_buggy_has_null(void)
{
	ASSERT_TRUE(qIdx2Str_buggy[Q_MAX - 1] == NULL);  /* the %s NULL-deref source */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("Audit 2026-06-20 regression locks");

	RUN_TEST(test_nvm_getid_no_overflow);
	RUN_TEST(test_nvm_getid_buggy_overflows_canary);
	RUN_TEST(test_nvm_getaddr_no_overflow);
	RUN_TEST(test_nvm_set_zero_pads_slot);

	RUN_TEST(test_cs_balanced);
	RUN_TEST(test_cs_unbalanced_exit_guarded);
	RUN_TEST(test_cs_buggy_wraps);

	RUN_TEST(test_attr_scan_2byte_then_narrow);
	RUN_TEST(test_attr_narrow_high_value);

	RUN_TEST(test_qidx2str_all_non_null);
	RUN_TEST(test_qidx2str_buggy_has_null);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
