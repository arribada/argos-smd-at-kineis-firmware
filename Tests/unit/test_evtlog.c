/**
 * @file    test_evtlog.c
 * @brief   Unit tests for MGR_EVTLOG circular event buffer
 *
 * Tests the circular buffer logic: init, log, wrap-around,
 * oldest-first retrieval, clear, and magic validation.
 *
 * HAL dependencies are stubbed out (HAL_GetTick, g_uw_doppler_state_for_err).
 */

#include "test_framework.h"

/*******************************************************************************
 * STUBS FOR HARDWARE DEPENDENCIES
 ******************************************************************************/

static uint32_t stub_tick = 0;
uint32_t HAL_GetTick(void) { return stub_tick; }

volatile uint32_t g_uw_doppler_state_for_err = 0;

/*******************************************************************************
 * EVTLOG DEFINITIONS (copied from mgr_evtlog.h / mgr_evtlog.c)
 ******************************************************************************/

#define MGR_EVTLOG_MAX_ENTRIES  256
#define MGR_EVTLOG_MAGIC        0x4C4F4721  /* "LOG!" */

typedef enum {
	EVT_BOOT = 0,
	EVT_STATE_CHANGE,
	EVT_REED_ON,
	EVT_REED_OFF,
	EVT_SWS_SURFACE,
	EVT_SWS_UNDERWATER,
	EVT_TX_START,
	EVT_TX_DONE,
	EVT_TX_TIMEOUT,
	EVT_MAC_READY,
	EVT_MAC_ERROR,
	EVT_BAT,
	EVT_SHUTDOWN,
	EVT_TIMEOUT,
	EVT_ERROR,
} MGR_EVTLOG_Type_t;

typedef struct {
	uint32_t tick;
	uint8_t  type;
	uint8_t  state;
	uint16_t data;
} MGR_EVTLOG_Entry_t;

/* Buffer struct (no __section__ attribute for test) */
typedef struct {
	uint32_t magic;
	uint16_t head;
	uint16_t count;
	MGR_EVTLOG_Entry_t entries[MGR_EVTLOG_MAX_ENTRIES];
} MGR_EVTLOG_Buffer_t;

static MGR_EVTLOG_Buffer_t evtlog_buf;

/* ---- Functions under test (copied from mgr_evtlog.c) ---- */

static void MGR_EVTLOG_init(void)
{
	if (evtlog_buf.magic != MGR_EVTLOG_MAGIC) {
		evtlog_buf.magic = MGR_EVTLOG_MAGIC;
		evtlog_buf.head  = 0;
		evtlog_buf.count = 0;
	}
}

static void MGR_EVTLOG_log(MGR_EVTLOG_Type_t type, uint16_t data)
{
	MGR_EVTLOG_Entry_t *e = &evtlog_buf.entries[evtlog_buf.head];

	e->tick  = HAL_GetTick();
	e->type  = (uint8_t)type;
	e->state = (uint8_t)g_uw_doppler_state_for_err;
	e->data  = data;

	evtlog_buf.head = (evtlog_buf.head + 1) % MGR_EVTLOG_MAX_ENTRIES;
	if (evtlog_buf.count < MGR_EVTLOG_MAX_ENTRIES)
		evtlog_buf.count++;
}

static uint16_t MGR_EVTLOG_count(void)
{
	return evtlog_buf.count;
}

static const MGR_EVTLOG_Entry_t *MGR_EVTLOG_get(uint16_t index)
{
	if (index >= evtlog_buf.count)
		return NULL;

	uint16_t pos = (evtlog_buf.head + MGR_EVTLOG_MAX_ENTRIES - evtlog_buf.count + index)
			% MGR_EVTLOG_MAX_ENTRIES;
	return &evtlog_buf.entries[pos];
}

static void MGR_EVTLOG_clear(void)
{
	evtlog_buf.head  = 0;
	evtlog_buf.count = 0;
}

/* Helper to reset buffer to garbage state */
static void reset_buffer_garbage(void)
{
	memset(&evtlog_buf, 0xAA, sizeof(evtlog_buf));
}

/* Helper to reset buffer cleanly */
static void reset_buffer_clean(void)
{
	memset(&evtlog_buf, 0, sizeof(evtlog_buf));
	stub_tick = 0;
	g_uw_doppler_state_for_err = 0;
}

/*******************************************************************************
 * TEST CASES
 ******************************************************************************/

/** Entry struct must be exactly 8 bytes for efficient storage */
void test_entry_struct_size(void)
{
	ASSERT_EQ(8, sizeof(MGR_EVTLOG_Entry_t));
	TEST_PASS();
}

/** Buffer struct: 8-byte header + 256*8 = 2056 bytes */
void test_buffer_struct_size(void)
{
	ASSERT_EQ(2056, sizeof(MGR_EVTLOG_Buffer_t));
	TEST_PASS();
}

/** Init with garbage magic should reset buffer */
void test_init_garbage_magic(void)
{
	reset_buffer_garbage();
	MGR_EVTLOG_init();

	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	ASSERT_EQ(0, evtlog_buf.head);
	ASSERT_EQ(0, evtlog_buf.count);
	TEST_PASS();
}

/** Init with valid magic should preserve buffer */
void test_init_valid_magic_preserves(void)
{
	reset_buffer_clean();
	evtlog_buf.magic = MGR_EVTLOG_MAGIC;
	evtlog_buf.head  = 42;
	evtlog_buf.count = 42;

	MGR_EVTLOG_init();

	ASSERT_EQ(42, evtlog_buf.head);
	ASSERT_EQ(42, evtlog_buf.count);
	TEST_PASS();
}

/** Log single entry */
void test_log_single_entry(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	stub_tick = 1000;
	g_uw_doppler_state_for_err = 3;
	MGR_EVTLOG_log(EVT_BOOT, 0x1234);

	ASSERT_EQ(1, MGR_EVTLOG_count());
	ASSERT_EQ(1, evtlog_buf.head);

	const MGR_EVTLOG_Entry_t *e = MGR_EVTLOG_get(0);
	ASSERT_NOT_NULL(e);
	ASSERT_EQ(1000, e->tick);
	ASSERT_EQ(EVT_BOOT, e->type);
	ASSERT_EQ(3, e->state);
	ASSERT_EQ(0x1234, e->data);
	TEST_PASS();
}

/** Log multiple entries, verify FIFO order (get(0) = oldest) */
void test_log_multiple_fifo_order(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 10; i++) {
		stub_tick = (uint32_t)(i * 100);
		g_uw_doppler_state_for_err = (uint32_t)i;
		MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)i);
	}

	ASSERT_EQ(10, MGR_EVTLOG_count());

	/* get(0) = oldest (tick=0), get(9) = newest (tick=900) */
	const MGR_EVTLOG_Entry_t *oldest = MGR_EVTLOG_get(0);
	const MGR_EVTLOG_Entry_t *newest = MGR_EVTLOG_get(9);
	ASSERT_NOT_NULL(oldest);
	ASSERT_NOT_NULL(newest);
	ASSERT_EQ(0, oldest->tick);
	ASSERT_EQ(0, oldest->data);
	ASSERT_EQ(900, newest->tick);
	ASSERT_EQ(9, newest->data);
	TEST_PASS();
}

/** Count saturates at 256 */
void test_count_saturates_at_256(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 300; i++) {
		stub_tick = (uint32_t)i;
		MGR_EVTLOG_log(EVT_BOOT, (uint16_t)i);
	}

	ASSERT_EQ(256, MGR_EVTLOG_count());
	TEST_PASS();
}

/** Wrap-around: after 300 logs, oldest should be entry #44 (300-256) */
void test_wrap_around_oldest_correct(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 300; i++) {
		stub_tick = (uint32_t)(i * 10);
		MGR_EVTLOG_log(EVT_BAT, (uint16_t)i);
	}

	/* Oldest entry is i=44 (first 44 entries were overwritten) */
	const MGR_EVTLOG_Entry_t *oldest = MGR_EVTLOG_get(0);
	ASSERT_NOT_NULL(oldest);
	ASSERT_EQ(44 * 10, oldest->tick);
	ASSERT_EQ(44, oldest->data);

	/* Newest entry is i=299 */
	const MGR_EVTLOG_Entry_t *newest = MGR_EVTLOG_get(255);
	ASSERT_NOT_NULL(newest);
	ASSERT_EQ(299 * 10, newest->tick);
	ASSERT_EQ(299, newest->data);
	TEST_PASS();
}

/** Head wraps correctly */
void test_head_wraps_modulo_256(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 256; i++) {
		MGR_EVTLOG_log(EVT_BOOT, 0);
	}
	ASSERT_EQ(0, evtlog_buf.head);  /* 256 % 256 = 0 */

	MGR_EVTLOG_log(EVT_BOOT, 0);
	ASSERT_EQ(1, evtlog_buf.head);
	TEST_PASS();
}

/** get() returns NULL for out-of-range index */
void test_get_out_of_range(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	MGR_EVTLOG_log(EVT_BOOT, 0);

	ASSERT_TRUE(MGR_EVTLOG_get(0) != NULL);
	ASSERT_TRUE(MGR_EVTLOG_get(1) == NULL);
	ASSERT_TRUE(MGR_EVTLOG_get(256) == NULL);
	ASSERT_TRUE(MGR_EVTLOG_get(65535) == NULL);
	TEST_PASS();
}

/** get() on empty buffer returns NULL */
void test_get_empty_buffer(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	ASSERT_EQ(0, MGR_EVTLOG_count());
	ASSERT_TRUE(MGR_EVTLOG_get(0) == NULL);
	TEST_PASS();
}

/** clear() resets head and count, keeps magic */
void test_clear(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 50; i++) {
		MGR_EVTLOG_log(EVT_BOOT, 0);
	}
	ASSERT_EQ(50, MGR_EVTLOG_count());

	MGR_EVTLOG_clear();

	ASSERT_EQ(0, MGR_EVTLOG_count());
	ASSERT_EQ(0, evtlog_buf.head);
	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	TEST_PASS();
}

/** After clear, can log again starting from 0 */
void test_log_after_clear(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 100; i++) {
		MGR_EVTLOG_log(EVT_BOOT, (uint16_t)i);
	}
	MGR_EVTLOG_clear();

	stub_tick = 9999;
	MGR_EVTLOG_log(EVT_SHUTDOWN, 0xABCD);

	ASSERT_EQ(1, MGR_EVTLOG_count());
	const MGR_EVTLOG_Entry_t *e = MGR_EVTLOG_get(0);
	ASSERT_NOT_NULL(e);
	ASSERT_EQ(9999, e->tick);
	ASSERT_EQ(EVT_SHUTDOWN, e->type);
	ASSERT_EQ(0xABCD, e->data);
	TEST_PASS();
}

/** All event types store correctly */
void test_all_event_types(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	MGR_EVTLOG_Type_t types[] = {
		EVT_BOOT, EVT_STATE_CHANGE, EVT_REED_ON, EVT_REED_OFF,
		EVT_SWS_SURFACE, EVT_SWS_UNDERWATER, EVT_TX_START, EVT_TX_DONE,
		EVT_TX_TIMEOUT, EVT_MAC_READY, EVT_MAC_ERROR, EVT_BAT,
		EVT_SHUTDOWN, EVT_TIMEOUT, EVT_ERROR
	};
	int n = (int)(sizeof(types) / sizeof(types[0]));

	for (int i = 0; i < n; i++) {
		stub_tick = (uint32_t)(i + 1);
		MGR_EVTLOG_log(types[i], (uint16_t)(i * 10));
	}

	ASSERT_EQ(n, MGR_EVTLOG_count());

	for (int i = 0; i < n; i++) {
		const MGR_EVTLOG_Entry_t *e = MGR_EVTLOG_get((uint16_t)i);
		ASSERT_NOT_NULL(e);
		ASSERT_EQ((uint8_t)types[i], e->type);
		ASSERT_EQ((uint16_t)(i * 10), e->data);
	}
	TEST_PASS();
}

/** State field tracks g_uw_doppler_state_for_err */
void test_state_tracking(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	g_uw_doppler_state_for_err = 0;
	MGR_EVTLOG_log(EVT_BOOT, 0);

	g_uw_doppler_state_for_err = 4;
	MGR_EVTLOG_log(EVT_SWS_SURFACE, 0);

	g_uw_doppler_state_for_err = 7;
	MGR_EVTLOG_log(EVT_SHUTDOWN, 0);

	ASSERT_EQ(0, MGR_EVTLOG_get(0)->state);
	ASSERT_EQ(4, MGR_EVTLOG_get(1)->state);
	ASSERT_EQ(7, MGR_EVTLOG_get(2)->state);
	TEST_PASS();
}

/** Exact boundary: fill exactly 256 entries */
void test_exact_full_buffer(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 256; i++) {
		stub_tick = (uint32_t)i;
		MGR_EVTLOG_log(EVT_BAT, (uint16_t)i);
	}

	ASSERT_EQ(256, MGR_EVTLOG_count());

	/* Oldest is i=0, newest is i=255 */
	ASSERT_EQ(0, MGR_EVTLOG_get(0)->data);
	ASSERT_EQ(255, MGR_EVTLOG_get(255)->data);
	TEST_PASS();
}

/** One more than full: oldest entry overwritten */
void test_overwrite_oldest(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	for (int i = 0; i < 257; i++) {
		stub_tick = (uint32_t)i;
		MGR_EVTLOG_log(EVT_BAT, (uint16_t)i);
	}

	ASSERT_EQ(256, MGR_EVTLOG_count());

	/* Entry i=0 was overwritten, oldest is now i=1 */
	ASSERT_EQ(1, MGR_EVTLOG_get(0)->data);
	ASSERT_EQ(256, MGR_EVTLOG_get(255)->data);
	TEST_PASS();
}

/** Simulated warm boot: init preserves entries across "reset" */
void test_warm_boot_preservation(void)
{
	reset_buffer_clean();
	MGR_EVTLOG_init();

	stub_tick = 100;
	MGR_EVTLOG_log(EVT_BOOT, 1);
	stub_tick = 200;
	MGR_EVTLOG_log(EVT_STATE_CHANGE, 4);

	/* Simulate warm reset: don't clear buffer, just re-init */
	MGR_EVTLOG_init();

	/* Previous entries should still be there */
	ASSERT_EQ(2, MGR_EVTLOG_count());
	ASSERT_EQ(100, MGR_EVTLOG_get(0)->tick);
	ASSERT_EQ(200, MGR_EVTLOG_get(1)->tick);

	/* Can log new entries */
	stub_tick = 300;
	MGR_EVTLOG_log(EVT_BOOT, 2);
	ASSERT_EQ(3, MGR_EVTLOG_count());
	ASSERT_EQ(300, MGR_EVTLOG_get(2)->tick);
	TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("MGR_EVTLOG Unit Tests");

	RUN_TEST(test_entry_struct_size);
	RUN_TEST(test_buffer_struct_size);
	RUN_TEST(test_init_garbage_magic);
	RUN_TEST(test_init_valid_magic_preserves);
	RUN_TEST(test_log_single_entry);
	RUN_TEST(test_log_multiple_fifo_order);
	RUN_TEST(test_count_saturates_at_256);
	RUN_TEST(test_wrap_around_oldest_correct);
	RUN_TEST(test_head_wraps_modulo_256);
	RUN_TEST(test_get_out_of_range);
	RUN_TEST(test_get_empty_buffer);
	RUN_TEST(test_clear);
	RUN_TEST(test_log_after_clear);
	RUN_TEST(test_all_event_types);
	RUN_TEST(test_state_tracking);
	RUN_TEST(test_exact_full_buffer);
	RUN_TEST(test_overwrite_oldest);
	RUN_TEST(test_warm_boot_preservation);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
