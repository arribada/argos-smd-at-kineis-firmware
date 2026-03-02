/**
 * @file    test_memory_safety.c
 * @brief   Memory safety and flash integrity tests
 *
 * Validates that:
 * - EVTLOG circular buffer never writes outside its bounds (canary-based)
 * - EVTLOG get() always returns pointers within the buffer
 * - NVM flash writes never exceed the flash page size
 * - Repeated save/load cycles don't accumulate or corrupt data
 * - Repeated log/clear cycles don't corrupt adjacent memory
 * - Buffer head/count fields stay within valid ranges under stress
 *
 * Uses "canary" bytes around allocated structures to detect overflow.
 */

#include "test_framework.h"
#include <stddef.h>

/*******************************************************************************
 * STUBS
 ******************************************************************************/

static uint32_t stub_tick = 0;
uint32_t HAL_GetTick(void) { return stub_tick; }
volatile uint32_t g_uw_doppler_state_for_err = 0;

/*******************************************************************************
 * EVTLOG DEFINITIONS (from mgr_evtlog.h / mgr_evtlog.c)
 ******************************************************************************/

#define MGR_EVTLOG_MAX_ENTRIES  256
#define MGR_EVTLOG_MAGIC        0x4C4F4721

typedef enum {
	EVT_BOOT = 0, EVT_STATE_CHANGE, EVT_REED_ON, EVT_REED_OFF,
	EVT_SWS_SURFACE, EVT_SWS_UNDERWATER, EVT_TX_START, EVT_TX_DONE,
	EVT_TX_TIMEOUT, EVT_MAC_READY, EVT_MAC_ERROR, EVT_BAT,
	EVT_SHUTDOWN, EVT_TIMEOUT, EVT_ERROR,
} MGR_EVTLOG_Type_t;

typedef struct {
	uint32_t tick;
	uint8_t  type;
	uint8_t  state;
	uint16_t data;
} MGR_EVTLOG_Entry_t;

typedef struct {
	uint32_t magic;
	uint16_t head;
	uint16_t count;
	MGR_EVTLOG_Entry_t entries[MGR_EVTLOG_MAX_ENTRIES];
} MGR_EVTLOG_Buffer_t;

/*
 * Canary-guarded buffer: 64 canary bytes before and after the real buffer
 * to detect any out-of-bounds write.
 */
#define CANARY_SIZE  64
#define CANARY_BYTE  0xCD

static struct {
	uint8_t canary_before[CANARY_SIZE];
	MGR_EVTLOG_Buffer_t buf;
	uint8_t canary_after[CANARY_SIZE];
} guarded_evtlog;

/* Pointer alias for convenience */
#define evtlog_buf (guarded_evtlog.buf)

/* EVTLOG functions (from mgr_evtlog.c) */
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

/*******************************************************************************
 * NVM DEFINITIONS (from mgr_nvm.h / mgr_nvm.c)
 ******************************************************************************/

#define NVM_MAGIC   0x434F4E46UL
#define NVM_VERSION 1

typedef struct {
	uint32_t magic;
	uint8_t  version;
	uint8_t  deploy_mode;
	uint8_t  led_mode;
	uint8_t  _pad0;
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint8_t  tx_max_count;
	uint16_t tx_max_interval_s;
	uint8_t  _pad1[2];
	uint16_t sws_threshold_min;
	uint16_t sws_threshold_max;
	uint16_t sws_initial_air_baseline;
	uint16_t sws_initial_water_baseline;
	uint32_t sws_test_interval_ms;
	uint32_t sws_max_dive_time_s;
	uint32_t sws_min_surface_time_s;
	uint8_t  sws_enabled;
	uint8_t  _pad2[3];
	uint32_t crc32;
} NVM_Config_t;

static uint32_t nvm_crc32(const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t crc = 0xFFFFFFFFUL;
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)p[i] << 24;
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80000000UL)
				crc = (crc << 1) ^ 0x04C11DB7UL;
			else
				crc <<= 1;
		}
	}
	return crc;
}

/*
 * Canary-guarded flash page: detect writes beyond page boundary.
 */
#define FLASH_PAGE_SIZE 2048

static struct {
	uint8_t canary_before[CANARY_SIZE];
	uint8_t page[FLASH_PAGE_SIZE];
	uint8_t canary_after[CANARY_SIZE];
} guarded_flash;

/* Track every write: offset and length */
#define MAX_WRITE_LOG 256
static struct {
	size_t offset;
	size_t len;
} write_log[MAX_WRITE_LOG];
static int write_log_count = 0;

typedef int KNS_status_t;
#define KNS_STATUS_OK    0
#define KNS_STATUS_ERROR (-1)

static KNS_status_t MCU_FLASH_read(uint32_t addr, void *dst, size_t len)
{
	(void)addr;
	if (len > FLASH_PAGE_SIZE) return KNS_STATUS_ERROR;
	memcpy(dst, guarded_flash.page, len);
	return KNS_STATUS_OK;
}

static KNS_status_t MCU_FLASH_write(uint32_t addr, const void *src, size_t len)
{
	(void)addr;
	if (len > FLASH_PAGE_SIZE) return KNS_STATUS_ERROR;
	memcpy(guarded_flash.page, src, len);
	if (write_log_count < MAX_WRITE_LOG) {
		write_log[write_log_count].offset = 0;
		write_log[write_log_count].len = len;
		write_log_count++;
	}
	return KNS_STATUS_OK;
}

#define FLASH_NVM_CONFIG_ADDR 0x0803F800UL

/* Simplified NVM stubs */
static uint8_t stub_deploy_mode;
static uint8_t stub_led_mode;
static uint16_t stub_tx_initial_interval_s;
static uint8_t  stub_tx_growth_percent;
static uint16_t stub_tx_max_interval_s;
static uint8_t  stub_tx_max_count;

static bool MGR_NVM_save(void)
{
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.magic   = NVM_MAGIC;
	cfg.version = NVM_VERSION;
	cfg.deploy_mode = stub_deploy_mode;
	cfg.led_mode    = stub_led_mode;
	cfg.tx_initial_interval_s = stub_tx_initial_interval_s;
	cfg.tx_growth_percent     = stub_tx_growth_percent;
	cfg.tx_max_interval_s     = stub_tx_max_interval_s;
	cfg.tx_max_count          = stub_tx_max_count;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;
	return true;
}

static bool MGR_NVM_load(void)
{
	NVM_Config_t cfg;
	if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;
	if (cfg.magic != NVM_MAGIC || cfg.version != NVM_VERSION)
		return false;
	uint32_t computed = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (computed != cfg.crc32)
		return false;
	stub_deploy_mode = cfg.deploy_mode;
	stub_led_mode    = cfg.led_mode;
	stub_tx_initial_interval_s = cfg.tx_initial_interval_s;
	stub_tx_growth_percent     = cfg.tx_growth_percent;
	stub_tx_max_interval_s     = cfg.tx_max_interval_s;
	stub_tx_max_count          = cfg.tx_max_count;
	return true;
}

/*******************************************************************************
 * HELPERS
 ******************************************************************************/

static void init_canaries(void)
{
	memset(guarded_evtlog.canary_before, CANARY_BYTE, CANARY_SIZE);
	memset(guarded_evtlog.canary_after,  CANARY_BYTE, CANARY_SIZE);
	memset(guarded_flash.canary_before,  CANARY_BYTE, CANARY_SIZE);
	memset(guarded_flash.canary_after,   CANARY_BYTE, CANARY_SIZE);
}

static bool check_canary(const uint8_t *canary, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (canary[i] != CANARY_BYTE)
			return false;
	}
	return true;
}

static void reset_all(void)
{
	memset(&guarded_evtlog.buf, 0, sizeof(guarded_evtlog.buf));
	memset(guarded_flash.page, 0xFF, FLASH_PAGE_SIZE);
	stub_tick = 0;
	g_uw_doppler_state_for_err = 0;
	stub_deploy_mode = 0;
	stub_led_mode = 0;
	stub_tx_initial_interval_s = 0;
	stub_tx_growth_percent = 0;
	stub_tx_max_interval_s = 0;
	stub_tx_max_count = 0;
	write_log_count = 0;
	init_canaries();
}

/*******************************************************************************
 * EVTLOG MEMORY SAFETY TESTS
 ******************************************************************************/

/** Fill buffer exactly to 256 entries - canaries untouched */
void test_evtlog_full_fill_no_overflow(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int i = 0; i < 256; i++) {
		stub_tick = (uint32_t)i;
		MGR_EVTLOG_log(EVT_BAT, (uint16_t)i);
	}

	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** Wrap around 3x (768 entries) - canaries untouched */
void test_evtlog_triple_wrap_no_overflow(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int i = 0; i < 768; i++) {
		stub_tick = (uint32_t)i;
		g_uw_doppler_state_for_err = (uint32_t)(i % 8);
		MGR_EVTLOG_log((MGR_EVTLOG_Type_t)(i % 15), (uint16_t)(i & 0xFFFF));
	}

	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	ASSERT_EQ(256, MGR_EVTLOG_count());
	TEST_PASS();
}

/** Stress test: 10000 log entries - no memory corruption */
void test_evtlog_stress_10k_no_corruption(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int i = 0; i < 10000; i++) {
		stub_tick = (uint32_t)i;
		g_uw_doppler_state_for_err = (uint32_t)(i & 0x07);
		MGR_EVTLOG_log((MGR_EVTLOG_Type_t)(i % 15), (uint16_t)(i * 7));
	}

	/* Canaries intact */
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));

	/* Count saturated */
	ASSERT_EQ(256, MGR_EVTLOG_count());

	/* Head is valid */
	ASSERT_TRUE(evtlog_buf.head < MGR_EVTLOG_MAX_ENTRIES);

	/* Magic intact */
	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	TEST_PASS();
}

/** Repeated log/clear cycles - no memory leak or corruption */
void test_evtlog_log_clear_cycles(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int cycle = 0; cycle < 100; cycle++) {
		/* Log some entries */
		for (int i = 0; i < 50; i++) {
			stub_tick = (uint32_t)(cycle * 1000 + i);
			MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)i);
		}
		/* Clear */
		MGR_EVTLOG_clear();
		ASSERT_EQ(0, MGR_EVTLOG_count());
		ASSERT_EQ(0, evtlog_buf.head);
	}

	/* After 100 cycles, canaries still intact */
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	TEST_PASS();
}

/** get() always returns pointers within the buffer bounds */
void test_evtlog_get_pointers_in_bounds(void)
{
	reset_all();
	MGR_EVTLOG_init();

	/* Fill with wrap-around */
	for (int i = 0; i < 300; i++) {
		stub_tick = (uint32_t)i;
		MGR_EVTLOG_log(EVT_BOOT, (uint16_t)i);
	}

	const uint8_t *buf_start = (const uint8_t *)&evtlog_buf.entries[0];
	const uint8_t *buf_end   = (const uint8_t *)&evtlog_buf.entries[MGR_EVTLOG_MAX_ENTRIES];

	for (uint16_t i = 0; i < MGR_EVTLOG_count(); i++) {
		const MGR_EVTLOG_Entry_t *e = MGR_EVTLOG_get(i);
		ASSERT_NOT_NULL(e);
		const uint8_t *ptr = (const uint8_t *)e;
		ASSERT_TRUE(ptr >= buf_start);
		ASSERT_TRUE(ptr < buf_end);
	}

	/* Out-of-range returns NULL */
	ASSERT_TRUE(MGR_EVTLOG_get(256) == NULL);
	ASSERT_TRUE(MGR_EVTLOG_get(65535) == NULL);
	TEST_PASS();
}

/** head and count invariants hold under all conditions */
void test_evtlog_invariants(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int i = 0; i < 500; i++) {
		MGR_EVTLOG_log(EVT_BOOT, 0);

		/* head always in [0, 255] */
		ASSERT_TRUE(evtlog_buf.head < MGR_EVTLOG_MAX_ENTRIES);

		/* count always in [0, 256] */
		ASSERT_TRUE(evtlog_buf.count <= MGR_EVTLOG_MAX_ENTRIES);

		/* count <= head before saturation, == 256 after */
		if (i < 255) {
			ASSERT_EQ((uint16_t)(i + 1), evtlog_buf.count);
		} else {
			ASSERT_EQ(256, evtlog_buf.count);
		}
	}

	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** Simulated warm reboot cycles with logging - no corruption */
void test_evtlog_warm_reboot_stress(void)
{
	reset_all();

	for (int boot = 0; boot < 50; boot++) {
		/* Simulate boot: init preserves or creates fresh */
		MGR_EVTLOG_init();

		/* Log a few events per boot */
		for (int i = 0; i < 10; i++) {
			stub_tick = (uint32_t)(boot * 10000 + i * 100);
			g_uw_doppler_state_for_err = (uint32_t)(boot % 8);
			MGR_EVTLOG_log(EVT_BOOT + (i % 5), (uint16_t)(boot * 10 + i));
		}
	}

	/* 50 boots * 10 events = 500 total, last 256 preserved */
	ASSERT_EQ(256, MGR_EVTLOG_count());
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	TEST_PASS();
}

/*******************************************************************************
 * NVM / FLASH SAFETY TESTS
 ******************************************************************************/

/** NVM save writes exactly sizeof(NVM_Config_t) bytes - no more */
void test_nvm_write_exact_size(void)
{
	reset_all();
	stub_tx_initial_interval_s = 10;
	ASSERT_TRUE(MGR_NVM_save());

	ASSERT_EQ(1, write_log_count);
	ASSERT_EQ(sizeof(NVM_Config_t), write_log[0].len);
	ASSERT_TRUE(write_log[0].len <= FLASH_PAGE_SIZE);
	TEST_PASS();
}

/** NVM save never writes beyond flash page boundary */
void test_nvm_no_flash_overflow(void)
{
	reset_all();
	stub_tx_initial_interval_s = 10;
	ASSERT_TRUE(MGR_NVM_save());

	/* Flash canaries intact */
	ASSERT_TRUE(check_canary(guarded_flash.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** Repeated save cycles - flash canaries intact, no accumulation */
void test_nvm_repeated_save_no_accumulation(void)
{
	reset_all();

	for (int i = 0; i < 100; i++) {
		stub_deploy_mode = (uint8_t)(i & 1);
		stub_tx_initial_interval_s = (uint16_t)(10 + i);
		stub_tx_growth_percent = (uint8_t)(i % 50);
		ASSERT_TRUE(MGR_NVM_save());
	}

	/* Flash canaries intact after 100 writes */
	ASSERT_TRUE(check_canary(guarded_flash.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));

	/* All writes were same size */
	ASSERT_EQ(100, write_log_count);
	for (int i = 0; i < 100; i++) {
		ASSERT_EQ(sizeof(NVM_Config_t), write_log[i].len);
	}

	/* Last save is what we read back */
	ASSERT_TRUE(MGR_NVM_load());
	ASSERT_EQ(109, stub_tx_initial_interval_s);  /* 10 + 99 */
	ASSERT_EQ(49,  stub_tx_growth_percent);       /* 99 % 50 */
	TEST_PASS();
}

/** Save/load round-trip 100 times with varying data - no corruption */
void test_nvm_save_load_stress(void)
{
	reset_all();

	for (int i = 0; i < 100; i++) {
		stub_deploy_mode = (uint8_t)(i % 2);
		stub_led_mode = (uint8_t)(i % 3);
		stub_tx_initial_interval_s = (uint16_t)(5 + i * 3);
		stub_tx_growth_percent = (uint8_t)(i % 100);
		stub_tx_max_interval_s = (uint16_t)(60 + i * 10);
		stub_tx_max_count = (uint8_t)(i % 255);

		ASSERT_TRUE(MGR_NVM_save());

		/* Clear local state */
		stub_deploy_mode = 0xFF;
		stub_led_mode = 0xFF;
		stub_tx_initial_interval_s = 0xFFFF;
		stub_tx_growth_percent = 0xFF;
		stub_tx_max_interval_s = 0xFFFF;
		stub_tx_max_count = 0xFF;

		/* Load back */
		ASSERT_TRUE(MGR_NVM_load());

		/* Verify restored correctly */
		ASSERT_EQ((uint8_t)(i % 2),            stub_deploy_mode);
		ASSERT_EQ((uint8_t)(i % 3),            stub_led_mode);
		ASSERT_EQ((uint16_t)(5 + i * 3),       stub_tx_initial_interval_s);
		ASSERT_EQ((uint8_t)(i % 100),          stub_tx_growth_percent);
		ASSERT_EQ((uint16_t)(60 + i * 10),     stub_tx_max_interval_s);
		ASSERT_EQ((uint8_t)(i % 255),          stub_tx_max_count);
	}

	/* Flash canaries after all cycles */
	ASSERT_TRUE(check_canary(guarded_flash.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** NVM config struct fits within one flash page */
void test_nvm_struct_fits_page(void)
{
	ASSERT_TRUE(sizeof(NVM_Config_t) <= FLASH_PAGE_SIZE);

	/* Should have ample room (struct is 44 bytes, page is 2048) */
	ASSERT_TRUE(sizeof(NVM_Config_t) < 256);
	TEST_PASS();
}

/** Flash page is clean (0xFF) after reset, no residual data leaks */
void test_nvm_reset_no_data_residue(void)
{
	reset_all();

	/* Write sensitive data */
	stub_deploy_mode = 1;
	stub_tx_initial_interval_s = 12345;
	ASSERT_TRUE(MGR_NVM_save());

	/* Verify data is in flash */
	NVM_Config_t *cfg = (NVM_Config_t *)guarded_flash.page;
	ASSERT_EQ(12345, cfg->tx_initial_interval_s);

	/* Reset: fills with 0xFF */
	NVM_Config_t reset_cfg;
	memset(&reset_cfg, 0xFF, sizeof(reset_cfg));
	MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &reset_cfg, sizeof(reset_cfg));

	/* Verify the config region is erased */
	for (size_t i = 0; i < sizeof(NVM_Config_t); i++) {
		ASSERT_EQ(0xFF, guarded_flash.page[i]);
	}

	/* Load should fail (erased) */
	ASSERT_FALSE(MGR_NVM_load());

	ASSERT_TRUE(check_canary(guarded_flash.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** Padding bytes in NVM_Config_t are zeroed (no stack garbage leaks) */
void test_nvm_padding_bytes_zeroed(void)
{
	reset_all();

	/* Poison the stack-like area */
	stub_deploy_mode = 0;
	stub_led_mode = 0;
	stub_tx_initial_interval_s = 100;
	stub_tx_growth_percent = 10;
	stub_tx_max_interval_s = 600;
	stub_tx_max_count = 50;

	ASSERT_TRUE(MGR_NVM_save());

	NVM_Config_t *cfg = (NVM_Config_t *)guarded_flash.page;

	/* Check padding fields are zero (memset in save) */
	ASSERT_EQ(0, cfg->_pad0);
	ASSERT_EQ(0, cfg->_pad1[0]);
	ASSERT_EQ(0, cfg->_pad1[1]);
	ASSERT_EQ(0, cfg->_pad2[0]);
	ASSERT_EQ(0, cfg->_pad2[1]);
	ASSERT_EQ(0, cfg->_pad2[2]);
	TEST_PASS();
}

/*******************************************************************************
 * COMBINED EVTLOG + NVM SAFETY
 ******************************************************************************/

/** Simultaneous EVTLOG and NVM stress - neither corrupts the other */
void test_combined_evtlog_nvm_isolation(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int i = 0; i < 500; i++) {
		/* Log events */
		stub_tick = (uint32_t)(i * 10);
		g_uw_doppler_state_for_err = (uint32_t)(i % 8);
		MGR_EVTLOG_log((MGR_EVTLOG_Type_t)(i % 15), (uint16_t)i);

		/* Periodically save NVM */
		if (i % 50 == 0) {
			stub_tx_initial_interval_s = (uint16_t)(10 + i);
			ASSERT_TRUE(MGR_NVM_save());
		}
	}

	/* Both canary zones intact */
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_flash.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));

	/* EVTLOG integrity */
	ASSERT_EQ(256, MGR_EVTLOG_count());
	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	ASSERT_TRUE(evtlog_buf.head < MGR_EVTLOG_MAX_ENTRIES);

	/* NVM integrity - last save was i=450 */
	ASSERT_TRUE(MGR_NVM_load());
	ASSERT_EQ(460, stub_tx_initial_interval_s); /* 10 + 450 */
	TEST_PASS();
}

/*******************************************************************************
 * FAULT INJECTION TESTS
 *
 * These tests DELIBERATELY introduce bugs and verify that our detection
 * mechanisms catch them. This proves the tests have real detection power
 * and aren't just trivially passing.
 ******************************************************************************/

/** INJECT: Corrupt canary_before → check_canary detects it */
void test_inject_canary_before_corruption(void)
{
	reset_all();

	/* Canary is intact */
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));

	/* Simulate buffer underflow: corrupt one byte in canary_before */
	guarded_evtlog.canary_before[CANARY_SIZE - 1] = 0x00;

	/* Detection: check_canary returns false */
	ASSERT_FALSE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));

	/* Restore for next tests */
	guarded_evtlog.canary_before[CANARY_SIZE - 1] = CANARY_BYTE;
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	TEST_PASS();
}

/** INJECT: Corrupt canary_after → check_canary detects it */
void test_inject_canary_after_corruption(void)
{
	reset_all();
	MGR_EVTLOG_init();

	/* Fill buffer completely */
	for (int i = 0; i < 256; i++)
		MGR_EVTLOG_log(EVT_BOOT, 0);

	/* Canary should be intact after normal operation */
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));

	/* Simulate buffer overflow: corrupt first byte of canary_after */
	guarded_evtlog.canary_after[0] = 0x42;

	/* Detection: check_canary catches it */
	ASSERT_FALSE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** INJECT: Corrupt flash canary → flash overflow detected */
void test_inject_flash_canary_corruption(void)
{
	reset_all();
	ASSERT_TRUE(MGR_NVM_save());

	/* Flash canary intact after normal write */
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));

	/* Simulate flash write overflow: corrupt byte just past the page */
	guarded_flash.canary_after[0] = 0xFF;

	/* Detection: canary check catches it */
	ASSERT_FALSE(check_canary(guarded_flash.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** INJECT: Single bit flip in NVM data → CRC rejects load */
void test_inject_single_bit_flip_detected(void)
{
	reset_all();
	stub_tx_initial_interval_s = 1000;
	stub_tx_growth_percent = 25;
	ASSERT_TRUE(MGR_NVM_save());

	/* Normal load works */
	stub_tx_initial_interval_s = 0;
	ASSERT_TRUE(MGR_NVM_load());
	ASSERT_EQ(1000, stub_tx_initial_interval_s);

	/* INJECT: flip bit 0 of byte 10 in flash */
	guarded_flash.page[10] ^= 0x01;

	/* Detection: CRC mismatch → load fails */
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** INJECT: Flip every single byte position → CRC catches all of them */
void test_inject_every_byte_corruption_detected(void)
{
	reset_all();
	stub_tx_initial_interval_s = 500;
	stub_deploy_mode = 1;
	ASSERT_TRUE(MGR_NVM_save());

	/* Save a clean copy */
	uint8_t clean_page[FLASH_PAGE_SIZE];
	memcpy(clean_page, guarded_flash.page, FLASH_PAGE_SIZE);

	/* For each byte in the NVM config, flip it and verify CRC catches it */
	for (size_t pos = 0; pos < sizeof(NVM_Config_t); pos++) {
		/* Restore clean state */
		memcpy(guarded_flash.page, clean_page, sizeof(NVM_Config_t));

		/* Corrupt this byte */
		guarded_flash.page[pos] ^= 0xFF;

		/* Load must fail (CRC mismatch, wrong magic, or wrong version) */
		ASSERT_FALSE(MGR_NVM_load());
	}

	/* Restore clean: load must succeed */
	memcpy(guarded_flash.page, clean_page, sizeof(NVM_Config_t));
	ASSERT_TRUE(MGR_NVM_load());
	ASSERT_EQ(500, stub_tx_initial_interval_s);
	TEST_PASS();
}

/** INJECT: Wrong magic in NVM → load rejected */
void test_inject_wrong_magic_rejected(void)
{
	reset_all();
	stub_tx_initial_interval_s = 100;
	ASSERT_TRUE(MGR_NVM_save());
	ASSERT_TRUE(MGR_NVM_load());

	/* Overwrite magic with garbage */
	NVM_Config_t *cfg = (NVM_Config_t *)guarded_flash.page;
	cfg->magic = 0xDEADBEEF;

	/* Detection: load fails */
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** INJECT: Wrong version in NVM → load rejected */
void test_inject_wrong_version_rejected(void)
{
	reset_all();
	stub_tx_initial_interval_s = 100;
	ASSERT_TRUE(MGR_NVM_save());

	NVM_Config_t *cfg = (NVM_Config_t *)guarded_flash.page;
	cfg->version = 99;

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** INJECT: Corrupted EVTLOG magic → init resets buffer */
void test_inject_evtlog_bad_magic_resets(void)
{
	reset_all();
	MGR_EVTLOG_init();

	/* Log 100 entries */
	for (int i = 0; i < 100; i++)
		MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)i);
	ASSERT_EQ(100, MGR_EVTLOG_count());

	/* INJECT: corrupt magic (simulates power glitch or SRAM corruption) */
	evtlog_buf.magic = 0xBADBAD;

	/* Re-init detects bad magic and resets */
	MGR_EVTLOG_init();
	ASSERT_EQ(0, MGR_EVTLOG_count());
	ASSERT_EQ(0, evtlog_buf.head);
	ASSERT_EQ_HEX(MGR_EVTLOG_MAGIC, evtlog_buf.magic);
	TEST_PASS();
}

/** INJECT: Corrupted head value → get() still safe (no crash, stays in bounds) */
void test_inject_corrupted_head_safe(void)
{
	reset_all();
	MGR_EVTLOG_init();

	for (int i = 0; i < 50; i++)
		MGR_EVTLOG_log(EVT_BOOT, (uint16_t)i);

	/* INJECT: set head to a value near the boundary */
	evtlog_buf.head = 255;

	/* Log one more entry — should write at index 255 and wrap to 0 */
	MGR_EVTLOG_log(EVT_ERROR, 0xFFFF);
	ASSERT_EQ(0, evtlog_buf.head);  /* Wrapped correctly */

	/* Canaries must still be intact */
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_before, CANARY_SIZE));
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** INJECT: Simulate the bug where head doesn't use modulo (off-by-one overflow) */
void test_inject_no_modulo_causes_overflow(void)
{
	reset_all();
	init_canaries();

	/* Manually set up buffer at the boundary */
	evtlog_buf.magic = MGR_EVTLOG_MAGIC;
	evtlog_buf.head = 255;
	evtlog_buf.count = 255;

	/* Normal log: uses modulo, wraps to 0 — safe */
	MGR_EVTLOG_log(EVT_BOOT, 0);
	ASSERT_EQ(0, evtlog_buf.head);
	ASSERT_TRUE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));

	/* Now simulate the BUG: what if head=256 (no modulo, off by one)?
	 * We manually set head=256 and write — this WOULD corrupt canary_after.
	 */
	evtlog_buf.head = MGR_EVTLOG_MAX_ENTRIES; /* 256 = OUT OF BOUNDS */

	/* Direct write at entries[256] stomps on canary_after */
	MGR_EVTLOG_Entry_t *bad_ptr = &evtlog_buf.entries[evtlog_buf.head];
	bad_ptr->tick = 0xDEADBEEF;
	bad_ptr->type = 0xFF;
	bad_ptr->state = 0xFF;
	bad_ptr->data = 0xFFFF;

	/* Detection: canary_after is now corrupted */
	ASSERT_FALSE(check_canary(guarded_evtlog.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** INJECT: Simulate flash write larger than page → overflow detected */
void test_inject_oversized_flash_write(void)
{
	reset_all();

	/* Normal write: fits in page */
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));

	/* INJECT: write 2048 + 8 bytes into the page (oversized) */
	uint8_t big_data[FLASH_PAGE_SIZE + 8];
	memset(big_data, 0xAA, sizeof(big_data));

	/* Our stub rejects writes > FLASH_PAGE_SIZE */
	KNS_status_t status = MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, big_data, sizeof(big_data));
	ASSERT_EQ(KNS_STATUS_ERROR, status);

	/* Canary should still be intact (write was rejected) */
	ASSERT_TRUE(check_canary(guarded_flash.canary_after, CANARY_SIZE));
	TEST_PASS();
}

/** INJECT: NVM padding not zeroed → data leak would be visible */
void test_inject_padding_leak_detected(void)
{
	reset_all();
	stub_tx_initial_interval_s = 100;
	ASSERT_TRUE(MGR_NVM_save());

	NVM_Config_t *cfg = (NVM_Config_t *)guarded_flash.page;

	/* Padding should be zero after memset in save */
	ASSERT_EQ(0, cfg->_pad0);
	ASSERT_EQ(0, cfg->_pad1[0]);
	ASSERT_EQ(0, cfg->_pad2[0]);

	/* INJECT: simulate what happens if save() DIDN'T use memset
	 * Padding bytes would contain stack garbage */
	cfg->_pad0 = 0xDE;
	cfg->_pad1[0] = 0xAD;
	cfg->_pad2[0] = 0xBE;

	/* These are now non-zero = data leak indicator */
	ASSERT_TRUE(cfg->_pad0 != 0);
	ASSERT_TRUE(cfg->_pad1[0] != 0);
	ASSERT_TRUE(cfg->_pad2[0] != 0);

	/* Also: CRC will now be wrong since we modified the struct */
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("Memory Safety & Flash Integrity Tests");

	/* EVTLOG memory safety */
	RUN_TEST(test_evtlog_full_fill_no_overflow);
	RUN_TEST(test_evtlog_triple_wrap_no_overflow);
	RUN_TEST(test_evtlog_stress_10k_no_corruption);
	RUN_TEST(test_evtlog_log_clear_cycles);
	RUN_TEST(test_evtlog_get_pointers_in_bounds);
	RUN_TEST(test_evtlog_invariants);
	RUN_TEST(test_evtlog_warm_reboot_stress);

	/* NVM / Flash safety */
	RUN_TEST(test_nvm_write_exact_size);
	RUN_TEST(test_nvm_no_flash_overflow);
	RUN_TEST(test_nvm_repeated_save_no_accumulation);
	RUN_TEST(test_nvm_save_load_stress);
	RUN_TEST(test_nvm_struct_fits_page);
	RUN_TEST(test_nvm_reset_no_data_residue);
	RUN_TEST(test_nvm_padding_bytes_zeroed);

	/* Combined isolation */
	RUN_TEST(test_combined_evtlog_nvm_isolation);

	/* Fault injection - prove detection works */
	RUN_TEST(test_inject_canary_before_corruption);
	RUN_TEST(test_inject_canary_after_corruption);
	RUN_TEST(test_inject_flash_canary_corruption);
	RUN_TEST(test_inject_single_bit_flip_detected);
	RUN_TEST(test_inject_every_byte_corruption_detected);
	RUN_TEST(test_inject_wrong_magic_rejected);
	RUN_TEST(test_inject_wrong_version_rejected);
	RUN_TEST(test_inject_evtlog_bad_magic_resets);
	RUN_TEST(test_inject_corrupted_head_safe);
	RUN_TEST(test_inject_no_modulo_causes_overflow);
	RUN_TEST(test_inject_oversized_flash_write);
	RUN_TEST(test_inject_padding_leak_detected);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
