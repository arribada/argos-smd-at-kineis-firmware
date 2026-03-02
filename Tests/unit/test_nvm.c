/**
 * @file    test_nvm.c
 * @brief   Unit tests for MGR_NVM CRC32 computation and config validation
 *
 * Tests the NVM CRC32/MPEG-2 implementation (same as STM32 HW CRC)
 * and the NVM_Config_t struct layout (alignment, size, CRC field offset).
 *
 * Flash and application dependencies are stubbed.
 */

#include "test_framework.h"
#include <stddef.h>

/*******************************************************************************
 * NVM DEFINITIONS (from mgr_nvm.h)
 ******************************************************************************/

#define NVM_MAGIC   0x434F4E46UL  /* "CONF" */
#define NVM_VERSION 1

typedef struct {
	uint32_t magic;
	uint8_t  version;
	uint8_t  deploy_mode;
	uint8_t  led_mode;
	uint8_t  _pad0;
	/* TX config */
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint8_t  tx_max_count;
	uint16_t tx_max_interval_s;
	uint8_t  _pad1[2];
	/* SWS config */
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

/*******************************************************************************
 * CRC32/MPEG-2 (copied from mgr_nvm.c)
 ******************************************************************************/

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

/*******************************************************************************
 * FLASH STUB
 ******************************************************************************/

/* Simulated flash page */
static uint8_t flash_page[2048];

typedef int KNS_status_t;
#define KNS_STATUS_OK    0
#define KNS_STATUS_ERROR (-1)

static bool flash_read_fail = false;
static bool flash_write_fail = false;

static KNS_status_t MCU_FLASH_read(uint32_t addr, void *dst, size_t len)
{
	(void)addr;
	if (flash_read_fail) return KNS_STATUS_ERROR;
	memcpy(dst, flash_page, len);
	return KNS_STATUS_OK;
}

static KNS_status_t MCU_FLASH_write(uint32_t addr, const void *src, size_t len)
{
	(void)addr;
	if (flash_write_fail) return KNS_STATUS_ERROR;
	memcpy(flash_page, src, len);
	return KNS_STATUS_OK;
}

/*******************************************************************************
 * APP STUBS (setters/getters used by MGR_NVM_load/save)
 ******************************************************************************/

typedef struct {
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint16_t tx_max_interval_s;
	uint8_t  tx_max_count;
} TxCfg_t;

typedef struct {
	uint16_t threshold_min;
	uint16_t threshold_max;
	uint16_t initial_air_baseline;
	uint16_t initial_water_baseline;
	uint32_t test_interval_ms;
	uint32_t max_dive_time_s;
	uint32_t min_surface_time_s;
	uint8_t  enabled;
} SwsCfg_t;

static TxCfg_t stub_tx_cfg;
static SwsCfg_t stub_sws_cfg;
static uint8_t stub_deploy_mode;
static uint8_t stub_led_mode;

/*******************************************************************************
 * SIMPLIFIED MGR_NVM_load / MGR_NVM_save (logic under test)
 ******************************************************************************/

#define FLASH_NVM_CONFIG_ADDR 0x0803F800UL  /* Doesn't matter for test */

static bool MGR_NVM_load(void)
{
	NVM_Config_t cfg;

	if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;

	if (cfg.magic != NVM_MAGIC || cfg.version != NVM_VERSION)
		return false;

	uint32_t computed_crc = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (computed_crc != cfg.crc32)
		return false;

	/* Restore config */
	stub_tx_cfg.tx_initial_interval_s = cfg.tx_initial_interval_s;
	stub_tx_cfg.tx_growth_percent     = cfg.tx_growth_percent;
	stub_tx_cfg.tx_max_interval_s     = cfg.tx_max_interval_s;
	stub_tx_cfg.tx_max_count          = cfg.tx_max_count;

	stub_sws_cfg.threshold_min          = cfg.sws_threshold_min;
	stub_sws_cfg.threshold_max          = cfg.sws_threshold_max;
	stub_sws_cfg.initial_air_baseline   = cfg.sws_initial_air_baseline;
	stub_sws_cfg.initial_water_baseline = cfg.sws_initial_water_baseline;
	stub_sws_cfg.test_interval_ms       = cfg.sws_test_interval_ms;
	stub_sws_cfg.max_dive_time_s        = cfg.sws_max_dive_time_s;
	stub_sws_cfg.min_surface_time_s     = cfg.sws_min_surface_time_s;
	stub_sws_cfg.enabled                = cfg.sws_enabled;

	stub_deploy_mode = cfg.deploy_mode;
	stub_led_mode    = cfg.led_mode;

	return true;
}

static bool MGR_NVM_save(void)
{
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.magic   = NVM_MAGIC;
	cfg.version = NVM_VERSION;

	cfg.tx_initial_interval_s = stub_tx_cfg.tx_initial_interval_s;
	cfg.tx_growth_percent     = stub_tx_cfg.tx_growth_percent;
	cfg.tx_max_interval_s     = stub_tx_cfg.tx_max_interval_s;
	cfg.tx_max_count          = stub_tx_cfg.tx_max_count;

	cfg.sws_threshold_min          = stub_sws_cfg.threshold_min;
	cfg.sws_threshold_max          = stub_sws_cfg.threshold_max;
	cfg.sws_initial_air_baseline   = stub_sws_cfg.initial_air_baseline;
	cfg.sws_initial_water_baseline = stub_sws_cfg.initial_water_baseline;
	cfg.sws_test_interval_ms       = stub_sws_cfg.test_interval_ms;
	cfg.sws_max_dive_time_s        = stub_sws_cfg.max_dive_time_s;
	cfg.sws_min_surface_time_s     = stub_sws_cfg.min_surface_time_s;
	cfg.sws_enabled                = stub_sws_cfg.enabled;

	cfg.deploy_mode = stub_deploy_mode;
	cfg.led_mode    = stub_led_mode;

	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));

	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;

	return true;
}

static bool MGR_NVM_reset(void)
{
	NVM_Config_t cfg;
	memset(&cfg, 0xFF, sizeof(cfg));

	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;

	return true;
}

/* Helper to reset test state */
static void reset_test_state(void)
{
	memset(flash_page, 0xFF, sizeof(flash_page));
	memset(&stub_tx_cfg, 0, sizeof(stub_tx_cfg));
	memset(&stub_sws_cfg, 0, sizeof(stub_sws_cfg));
	stub_deploy_mode = 0;
	stub_led_mode = 0;
	flash_read_fail = false;
	flash_write_fail = false;
}

/*******************************************************************************
 * CRC32 TESTS
 ******************************************************************************/

/** CRC32 known test vector: "123456789" */
void test_crc32_known_vector(void)
{
	const uint8_t data[] = "123456789";
	uint32_t crc = nvm_crc32(data, 9);
	ASSERT_EQ_HEX(0x0376E6E7, crc);
	TEST_PASS();
}

/** CRC32 single byte 0x00 */
void test_crc32_single_byte(void)
{
	uint8_t data = 0x00;
	uint32_t crc = nvm_crc32(&data, 1);
	ASSERT_EQ_HEX(0x4E08BFB4, crc);
	TEST_PASS();
}

/** CRC32 consistency: same data → same CRC */
void test_crc32_consistency(void)
{
	uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
	uint32_t crc1 = nvm_crc32(data, sizeof(data));
	uint32_t crc2 = nvm_crc32(data, sizeof(data));
	ASSERT_EQ_HEX(crc1, crc2);
	TEST_PASS();
}

/** CRC32 different data → different CRC */
void test_crc32_different_data(void)
{
	uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
	uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};
	uint32_t crc1 = nvm_crc32(data1, 4);
	uint32_t crc2 = nvm_crc32(data2, 4);
	ASSERT_TRUE(crc1 != crc2);
	TEST_PASS();
}

/** CRC32 empty data → init value */
void test_crc32_empty(void)
{
	uint32_t crc = nvm_crc32(NULL, 0);
	ASSERT_EQ_HEX(0xFFFFFFFF, crc);
	TEST_PASS();
}

/*******************************************************************************
 * NVM_Config_t STRUCT LAYOUT TESTS
 ******************************************************************************/

/** Struct size is 32-bit aligned (all fields uint32/uint16/uint8) */
void test_nvm_config_alignment(void)
{
	ASSERT_EQ(0, sizeof(NVM_Config_t) % 4);
	ASSERT_EQ(44, sizeof(NVM_Config_t));
	TEST_PASS();
}

/** CRC32 field is at the end of the struct */
void test_nvm_crc32_field_at_end(void)
{
	ASSERT_EQ(offsetof(NVM_Config_t, crc32) + sizeof(uint32_t), sizeof(NVM_Config_t));
	TEST_PASS();
}

/** Magic is at offset 0 */
void test_nvm_magic_at_offset_zero(void)
{
	ASSERT_EQ(0, offsetof(NVM_Config_t, magic));
	TEST_PASS();
}

/*******************************************************************************
 * NVM LOAD TESTS
 ******************************************************************************/

/** Load from erased flash (0xFF) → returns false */
void test_load_erased_flash(void)
{
	reset_test_state();
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load with wrong magic → returns false */
void test_load_wrong_magic(void)
{
	reset_test_state();
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.magic = 0xDEADBEEF;
	cfg.version = NVM_VERSION;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	memcpy(flash_page, &cfg, sizeof(cfg));

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load with wrong version → returns false */
void test_load_wrong_version(void)
{
	reset_test_state();
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.magic = NVM_MAGIC;
	cfg.version = 99;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	memcpy(flash_page, &cfg, sizeof(cfg));

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load with corrupted CRC → returns false */
void test_load_bad_crc(void)
{
	reset_test_state();
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.magic = NVM_MAGIC;
	cfg.version = NVM_VERSION;
	cfg.tx_initial_interval_s = 10;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	/* Corrupt one byte after CRC was computed */
	cfg.tx_initial_interval_s = 999;
	memcpy(flash_page, &cfg, sizeof(cfg));

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load with flash read error → returns false */
void test_load_flash_read_error(void)
{
	reset_test_state();
	flash_read_fail = true;
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load valid config → restores all fields */
void test_load_valid_config(void)
{
	reset_test_state();

	/* Prepare a valid config in flash */
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.magic = NVM_MAGIC;
	cfg.version = NVM_VERSION;
	cfg.deploy_mode = 1;
	cfg.led_mode = 2;
	cfg.tx_initial_interval_s = 15;
	cfg.tx_growth_percent = 20;
	cfg.tx_max_interval_s = 300;
	cfg.tx_max_count = 50;
	cfg.sws_threshold_min = 100;
	cfg.sws_threshold_max = 3000;
	cfg.sws_initial_air_baseline = 200;
	cfg.sws_initial_water_baseline = 2500;
	cfg.sws_test_interval_ms = 5000;
	cfg.sws_max_dive_time_s = 3600;
	cfg.sws_min_surface_time_s = 10;
	cfg.sws_enabled = 1;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	memcpy(flash_page, &cfg, sizeof(cfg));

	ASSERT_TRUE(MGR_NVM_load());

	/* Verify all fields were restored */
	ASSERT_EQ(1,    stub_deploy_mode);
	ASSERT_EQ(2,    stub_led_mode);
	ASSERT_EQ(15,   stub_tx_cfg.tx_initial_interval_s);
	ASSERT_EQ(20,   stub_tx_cfg.tx_growth_percent);
	ASSERT_EQ(300,  stub_tx_cfg.tx_max_interval_s);
	ASSERT_EQ(50,   stub_tx_cfg.tx_max_count);
	ASSERT_EQ(100,  stub_sws_cfg.threshold_min);
	ASSERT_EQ(3000, stub_sws_cfg.threshold_max);
	ASSERT_EQ(200,  stub_sws_cfg.initial_air_baseline);
	ASSERT_EQ(2500, stub_sws_cfg.initial_water_baseline);
	ASSERT_EQ(5000, stub_sws_cfg.test_interval_ms);
	ASSERT_EQ(3600, stub_sws_cfg.max_dive_time_s);
	ASSERT_EQ(10,   stub_sws_cfg.min_surface_time_s);
	ASSERT_EQ(1,    stub_sws_cfg.enabled);
	TEST_PASS();
}

/*******************************************************************************
 * NVM SAVE TESTS
 ******************************************************************************/

/** Save writes correct magic, version, and CRC */
void test_save_writes_valid_header(void)
{
	reset_test_state();
	stub_tx_cfg.tx_initial_interval_s = 10;

	ASSERT_TRUE(MGR_NVM_save());

	NVM_Config_t *cfg = (NVM_Config_t *)flash_page;
	ASSERT_EQ_HEX(NVM_MAGIC, cfg->magic);
	ASSERT_EQ(NVM_VERSION, cfg->version);

	/* Verify CRC */
	uint32_t computed = nvm_crc32(cfg, offsetof(NVM_Config_t, crc32));
	ASSERT_EQ_HEX(computed, cfg->crc32);
	TEST_PASS();
}

/** Save then load round-trip preserves all fields */
void test_save_load_roundtrip(void)
{
	reset_test_state();

	/* Set config */
	stub_deploy_mode = 1;
	stub_led_mode = 2;
	stub_tx_cfg.tx_initial_interval_s = 30;
	stub_tx_cfg.tx_growth_percent = 15;
	stub_tx_cfg.tx_max_interval_s = 600;
	stub_tx_cfg.tx_max_count = 100;
	stub_sws_cfg.threshold_min = 150;
	stub_sws_cfg.threshold_max = 2800;
	stub_sws_cfg.initial_air_baseline = 300;
	stub_sws_cfg.initial_water_baseline = 2400;
	stub_sws_cfg.test_interval_ms = 10000;
	stub_sws_cfg.max_dive_time_s = 7200;
	stub_sws_cfg.min_surface_time_s = 30;
	stub_sws_cfg.enabled = 1;

	ASSERT_TRUE(MGR_NVM_save());

	/* Clear stubs, then load */
	memset(&stub_tx_cfg, 0, sizeof(stub_tx_cfg));
	memset(&stub_sws_cfg, 0, sizeof(stub_sws_cfg));
	stub_deploy_mode = 0;
	stub_led_mode = 0;

	ASSERT_TRUE(MGR_NVM_load());

	ASSERT_EQ(1,     stub_deploy_mode);
	ASSERT_EQ(2,     stub_led_mode);
	ASSERT_EQ(30,    stub_tx_cfg.tx_initial_interval_s);
	ASSERT_EQ(15,    stub_tx_cfg.tx_growth_percent);
	ASSERT_EQ(600,   stub_tx_cfg.tx_max_interval_s);
	ASSERT_EQ(100,   stub_tx_cfg.tx_max_count);
	ASSERT_EQ(150,   stub_sws_cfg.threshold_min);
	ASSERT_EQ(2800,  stub_sws_cfg.threshold_max);
	ASSERT_EQ(300,   stub_sws_cfg.initial_air_baseline);
	ASSERT_EQ(2400,  stub_sws_cfg.initial_water_baseline);
	ASSERT_EQ(10000, stub_sws_cfg.test_interval_ms);
	ASSERT_EQ(7200,  stub_sws_cfg.max_dive_time_s);
	ASSERT_EQ(30,    stub_sws_cfg.min_surface_time_s);
	ASSERT_EQ(1,     stub_sws_cfg.enabled);
	TEST_PASS();
}

/** Save with flash write error → returns false */
void test_save_flash_write_error(void)
{
	reset_test_state();
	flash_write_fail = true;
	ASSERT_FALSE(MGR_NVM_save());
	TEST_PASS();
}

/*******************************************************************************
 * NVM RESET TESTS
 ******************************************************************************/

/** Reset fills flash with 0xFF (erased state) */
void test_reset_fills_ff(void)
{
	reset_test_state();

	/* First save valid data */
	ASSERT_TRUE(MGR_NVM_save());

	/* Then reset */
	ASSERT_TRUE(MGR_NVM_reset());

	/* Flash should be all 0xFF */
	NVM_Config_t *cfg = (NVM_Config_t *)flash_page;
	ASSERT_EQ_HEX(0xFFFFFFFF, cfg->magic);

	/* Load after reset should fail */
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Single bit flip in data → CRC mismatch */
void test_single_bit_corruption(void)
{
	reset_test_state();
	stub_tx_cfg.tx_initial_interval_s = 10;
	ASSERT_TRUE(MGR_NVM_save());

	/* Flip one bit in the flash data */
	flash_page[8] ^= 0x01;

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("MGR_NVM Unit Tests");

	/* CRC32 tests */
	RUN_TEST(test_crc32_known_vector);
	RUN_TEST(test_crc32_single_byte);
	RUN_TEST(test_crc32_consistency);
	RUN_TEST(test_crc32_different_data);
	RUN_TEST(test_crc32_empty);

	/* Struct layout tests */
	RUN_TEST(test_nvm_config_alignment);
	RUN_TEST(test_nvm_crc32_field_at_end);
	RUN_TEST(test_nvm_magic_at_offset_zero);

	/* Load tests */
	RUN_TEST(test_load_erased_flash);
	RUN_TEST(test_load_wrong_magic);
	RUN_TEST(test_load_wrong_version);
	RUN_TEST(test_load_bad_crc);
	RUN_TEST(test_load_flash_read_error);
	RUN_TEST(test_load_valid_config);

	/* Save tests */
	RUN_TEST(test_save_writes_valid_header);
	RUN_TEST(test_save_load_roundtrip);
	RUN_TEST(test_save_flash_write_error);

	/* Reset tests */
	RUN_TEST(test_reset_fills_ff);
	RUN_TEST(test_single_bit_corruption);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
