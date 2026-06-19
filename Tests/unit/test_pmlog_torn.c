/**
 * @file    test_pmlog_torn.c
 * @brief   Regression spec for the PMLOG torn-write fix (SF-1).
 *
 * Mirrors the slot state machine of mgr_pmlog.c after the magic-last fix:
 *   - magic (dw0) is programmed LAST, so a torn write leaves the slot's magic
 *     erased (EMPTY) and dw1 programmed → "torn", not "valid".
 *   - a slot is reusable only if BOTH dwords are erased (truly-empty).
 *   - MGR_PMLOG_log skips torn slots and only erases the page on a genuine
 *     wrap — a single torn write must NOT wipe prior valid entries.
 */

#include "test_framework.h"

#define MAX_ENTRIES 8u
#define MAGIC_VALID 0x504D4C45u
#define MAGIC_EMPTY 0xFFFFFFFFu
#define DW1_ERASED  0xFFFFFFFFFFFFFFFFull

typedef struct { uint32_t magic; uint64_t dw1; } slot_t;
static slot_t page[MAX_ENTRIES];
static uint16_t s_next_slot, s_valid_count, s_next_seq;
static int erase_count;

/* inject a torn write at a chosen call index */
static int program_calls;
static int tear_at_call = -1;

static void erase_page(void)
{
	for (uint16_t i = 0; i < MAX_ENTRIES; i++) {
		page[i].magic = MAGIC_EMPTY;
		page[i].dw1   = DW1_ERASED;
	}
	s_next_slot = 0; s_valid_count = 0; s_next_seq = 0;
	erase_count++;
}

static int slot_is_truly_empty(uint16_t i)
{
	return page[i].magic == MAGIC_EMPTY && page[i].dw1 == DW1_ERASED;
}

/* magic-last program; returns 0 on (injected) tear, leaving dw1 written but
 * magic erased — exactly the firmware's torn-write residue. */
static int program_two_dwords(uint16_t i, uint64_t dw1)
{
	int torn = (program_calls == tear_at_call);
	program_calls++;
	page[i].dw1 = dw1;                 /* dw1 written first */
	if (torn) return 0;               /* brownout before magic */
	page[i].magic = MAGIC_VALID;      /* magic last */
	return 1;
}

static void pmlog_log(uint64_t payload)
{
	while (s_next_slot < MAX_ENTRIES && !slot_is_truly_empty(s_next_slot))
		s_next_slot++;
	if (s_next_slot >= MAX_ENTRIES)
		erase_page();
	if (program_two_dwords(s_next_slot, payload)) {
		s_next_slot++; s_valid_count++; if (s_next_seq < 0xFFFFu) s_next_seq++;
	} else {
		s_next_slot++;   /* skip wasted slot, NO page erase */
	}
}

static uint16_t count_valid(void)
{
	uint16_t n = 0;
	for (uint16_t i = 0; i < MAX_ENTRIES; i++)
		if (page[i].magic == MAGIC_VALID) n++;
	return n;
}

static void reset_page(void)
{
	for (uint16_t i = 0; i < MAX_ENTRIES; i++) {
		page[i].magic = MAGIC_EMPTY; page[i].dw1 = DW1_ERASED;
	}
	s_next_slot = 0; s_valid_count = 0; s_next_seq = 0;
	erase_count = 0; program_calls = 0; tear_at_call = -1;
}

static void test_torn_write_does_not_erase_page(void)
{
	reset_page();
	pmlog_log(0x1111);          /* slot 0 OK */
	pmlog_log(0x2222);          /* slot 1 OK */
	tear_at_call = 2;           /* tear the 3rd program (slot 2) */
	pmlog_log(0x3333);          /* torn — slot 2 wasted */
	ASSERT_EQ(0, erase_count);  /* the bug: this used to wipe the page */
	ASSERT_EQ(2, count_valid()); /* slots 0,1 survive */
	TEST_PASS();
}

static void test_next_log_skips_torn_slot(void)
{
	reset_page();
	pmlog_log(0x1111);
	tear_at_call = 1;
	pmlog_log(0x2222);          /* torn at slot 1 */
	tear_at_call = -1;
	pmlog_log(0x3333);          /* must land in slot 2, not retry slot 1 */
	ASSERT_EQ(0, erase_count);
	ASSERT_EQ(MAGIC_EMPTY, page[1].magic);  /* torn slot magic stays erased */
	ASSERT_EQ(MAGIC_VALID, page[2].magic);  /* new entry skipped to slot 2 */
	ASSERT_EQ(2, count_valid());
	TEST_PASS();
}

static void test_genuine_wrap_still_erases(void)
{
	reset_page();
	for (int i = 0; i < (int)MAX_ENTRIES; i++)   /* fill the page */
		pmlog_log((uint64_t)(0xA000 + i));
	ASSERT_EQ(0, erase_count);
	ASSERT_EQ(MAX_ENTRIES, count_valid());
	pmlog_log(0xB000);          /* one past full → wrap → erase + write */
	ASSERT_EQ(1, erase_count);
	ASSERT_EQ(1, count_valid());
	TEST_PASS();
}

static void test_init_skips_torn_slot(void)
{
	/* Simulate a reboot: a torn slot sits among valid ones. The init-equivalent
	 * cursor (first truly-empty) must skip the torn slot. */
	reset_page();
	page[0].magic = MAGIC_VALID; page[0].dw1 = 0x11;
	page[1].magic = MAGIC_EMPTY; page[1].dw1 = 0x22;  /* torn: magic erased, dw1 set */
	page[2].magic = MAGIC_EMPTY; page[2].dw1 = DW1_ERASED;  /* truly empty */
	/* cursor walk (mirrors init + log's skip loop) */
	s_next_slot = 0;
	while (s_next_slot < MAX_ENTRIES && !slot_is_truly_empty(s_next_slot))
		s_next_slot++;
	ASSERT_EQ(2, s_next_slot);   /* skipped slot 0 (valid) and slot 1 (torn) */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("PMLOG torn-write resilience (SF-1)");

	RUN_TEST(test_torn_write_does_not_erase_page);
	RUN_TEST(test_next_log_skips_torn_slot);
	RUN_TEST(test_genuine_wrap_still_erases);
	RUN_TEST(test_init_skips_torn_slot);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
