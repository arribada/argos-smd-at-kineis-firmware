/**
 * @file    test_nvm.c
 * @brief   Unit tests for MGR_NVM CRC32 computation, v2 config, and v1→v2 migration
 *
 * Tests the NVM CRC32/MPEG-2 implementation (same as STM32 HW CRC),
 * the NVM_Config_t (v2) struct layout, and v1→v2 migration logic.
 *
 * Flash and application dependencies are stubbed.
 */

#include "test_framework.h"
#include <stddef.h>

/*******************************************************************************
 * NVM DEFINITIONS (from mgr_nvm.h / mgr_nvm.c)
 ******************************************************************************/

#define NVM_MAGIC   0x434F4E46UL  /* "CONF" */
#define NVM_VERSION 2

/* V1 layout (for migration tests) */
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
} NVM_Config_v1_t;

/* V2 layout (current) */
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
	uint16_t bat_min_tx_mV;
	uint8_t  _pad3[2];
	uint32_t crc32;
} NVM_Config_t;

#define MGR_BAT_DEFAULT_MIN_TX_MV  3200  /* Default for migration */

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
 * APP STUBS
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
static uint16_t stub_bat_min_tx_mV;

/*******************************************************************************
 * SIMPLIFIED MGR_NVM_load / MGR_NVM_save (v2 logic under test)
 ******************************************************************************/

#define FLASH_NVM_CONFIG_ADDR 0x0803F800UL

static bool MGR_NVM_load(void)
{
	NVM_Config_t cfg;

	if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;

	if (cfg.magic != NVM_MAGIC)
		return false;

	/* Handle v1 → v2 migration */
	if (cfg.version == 1) {
		NVM_Config_v1_t v1;
		if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &v1, sizeof(v1)) != KNS_STATUS_OK)
			return false;

		uint32_t computed_crc = nvm_crc32(&v1, offsetof(NVM_Config_v1_t, crc32));
		if (computed_crc != v1.crc32)
			return false;

		/* Migrate: copy shared fields, add defaults for new fields */
		memset(&cfg, 0, sizeof(cfg));
		cfg.magic                     = NVM_MAGIC;
		cfg.version                   = NVM_VERSION;
		cfg.deploy_mode               = v1.deploy_mode;
		cfg.led_mode                  = v1.led_mode;
		cfg.tx_initial_interval_s     = v1.tx_initial_interval_s;
		cfg.tx_growth_percent         = v1.tx_growth_percent;
		cfg.tx_max_count              = v1.tx_max_count;
		cfg.tx_max_interval_s         = v1.tx_max_interval_s;
		cfg.sws_threshold_min         = v1.sws_threshold_min;
		cfg.sws_threshold_max         = v1.sws_threshold_max;
		cfg.sws_initial_air_baseline  = v1.sws_initial_air_baseline;
		cfg.sws_initial_water_baseline = v1.sws_initial_water_baseline;
		cfg.sws_test_interval_ms      = v1.sws_test_interval_ms;
		cfg.sws_max_dive_time_s       = v1.sws_max_dive_time_s;
		cfg.sws_min_surface_time_s    = v1.sws_min_surface_time_s;
		cfg.sws_enabled               = v1.sws_enabled;
		cfg.bat_min_tx_mV             = MGR_BAT_DEFAULT_MIN_TX_MV;
		cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));

		/* Write migrated v2 back to flash */
		MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg));
	} else if (cfg.version == NVM_VERSION) {
		uint32_t computed_crc = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
		if (computed_crc != cfg.crc32)
			return false;
	} else {
		return false;  /* Unknown version */
	}

	/* Apply config */
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

	stub_deploy_mode   = cfg.deploy_mode;
	stub_led_mode      = cfg.led_mode;
	stub_bat_min_tx_mV = cfg.bat_min_tx_mV;

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

	cfg.deploy_mode   = stub_deploy_mode;
	cfg.led_mode      = stub_led_mode;
	cfg.bat_min_tx_mV = stub_bat_min_tx_mV;

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
	stub_bat_min_tx_mV = 0;
	flash_read_fail = false;
	flash_write_fail = false;
}

/* Helper to write a valid v1 config into flash_page */
static void write_v1_to_flash(uint8_t deploy, uint8_t led,
	uint16_t tx_init, uint8_t tx_growth, uint16_t tx_max, uint8_t tx_count,
	uint16_t sws_en)
{
	NVM_Config_v1_t v1;
	memset(&v1, 0, sizeof(v1));
	v1.magic = NVM_MAGIC;
	v1.version = 1;
	v1.deploy_mode = deploy;
	v1.led_mode = led;
	v1.tx_initial_interval_s = tx_init;
	v1.tx_growth_percent = tx_growth;
	v1.tx_max_interval_s = tx_max;
	v1.tx_max_count = tx_count;
	v1.sws_threshold_min = 0;
	v1.sws_threshold_max = 2000;
	v1.sws_initial_air_baseline = 50;
	v1.sws_initial_water_baseline = 750;
	v1.sws_test_interval_ms = 1000;
	v1.sws_max_dive_time_s = 7200;
	v1.sws_min_surface_time_s = 10;
	v1.sws_enabled = (uint8_t)sws_en;
	v1.crc32 = nvm_crc32(&v1, offsetof(NVM_Config_v1_t, crc32));
	memcpy(flash_page, &v1, sizeof(v1));
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

/** CRC32 consistency: same data -> same CRC */
void test_crc32_consistency(void)
{
	uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
	uint32_t crc1 = nvm_crc32(data, sizeof(data));
	uint32_t crc2 = nvm_crc32(data, sizeof(data));
	ASSERT_EQ_HEX(crc1, crc2);
	TEST_PASS();
}

/** CRC32 different data -> different CRC */
void test_crc32_different_data(void)
{
	uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
	uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};
	uint32_t crc1 = nvm_crc32(data1, 4);
	uint32_t crc2 = nvm_crc32(data2, 4);
	ASSERT_TRUE(crc1 != crc2);
	TEST_PASS();
}

/** CRC32 empty data -> init value */
void test_crc32_empty(void)
{
	uint32_t crc = nvm_crc32(NULL, 0);
	ASSERT_EQ_HEX(0xFFFFFFFF, crc);
	TEST_PASS();
}

/*******************************************************************************
 * NVM_Config_t (V2) STRUCT LAYOUT TESTS
 ******************************************************************************/

/** Struct is 4-byte aligned */
void test_nvm_config_alignment(void)
{
	ASSERT_EQ(0, sizeof(NVM_Config_t) % 4);
	TEST_PASS();
}

/** Struct is 8-byte aligned (64-bit flash writes) */
void test_nvm_config_doubleword_alignment(void)
{
	ASSERT_EQ(0, sizeof(NVM_Config_t) % 8);
	ASSERT_EQ(48, sizeof(NVM_Config_t));
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

/** bat_min_tx_mV field exists before CRC */
void test_nvm_bat_field_before_crc(void)
{
	size_t bat_off = offsetof(NVM_Config_t, bat_min_tx_mV);
	size_t crc_off = offsetof(NVM_Config_t, crc32);
	ASSERT_TRUE(bat_off < crc_off);
	TEST_PASS();
}

/** V1 struct is 44 bytes (for migration reference) */
void test_nvm_v1_size(void)
{
	ASSERT_EQ(44, sizeof(NVM_Config_v1_t));
	TEST_PASS();
}

/*******************************************************************************
 * NVM LOAD TESTS (V2)
 ******************************************************************************/

/** Load from erased flash (0xFF) -> returns false */
void test_load_erased_flash(void)
{
	reset_test_state();
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load with wrong magic -> returns false */
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

/** Load with unknown version -> returns false */
void test_load_unknown_version(void)
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

/** Load with corrupted CRC -> returns false */
void test_load_bad_crc(void)
{
	reset_test_state();
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.magic = NVM_MAGIC;
	cfg.version = NVM_VERSION;
	cfg.tx_initial_interval_s = 10;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	cfg.tx_initial_interval_s = 999;  /* Corrupt after CRC */
	memcpy(flash_page, &cfg, sizeof(cfg));

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load with flash read error -> returns false */
void test_load_flash_read_error(void)
{
	reset_test_state();
	flash_read_fail = true;
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Load valid v2 config -> restores all fields including bat_min_tx_mV */
void test_load_valid_v2_config(void)
{
	reset_test_state();

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
	cfg.bat_min_tx_mV = 3300;
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	memcpy(flash_page, &cfg, sizeof(cfg));

	ASSERT_TRUE(MGR_NVM_load());

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
	ASSERT_EQ(3300, stub_bat_min_tx_mV);
	TEST_PASS();
}

/*******************************************************************************
 * V1 -> V2 MIGRATION TESTS
 ******************************************************************************/

/** Load v1 config -> migrates to v2 with default bat_min_tx_mV */
void test_migration_v1_to_v2(void)
{
	reset_test_state();
	write_v1_to_flash(1, 2, 30, 15, 600, 100, 1);

	ASSERT_TRUE(MGR_NVM_load());

	/* Shared fields preserved */
	ASSERT_EQ(1,     stub_deploy_mode);
	ASSERT_EQ(2,     stub_led_mode);
	ASSERT_EQ(30,    stub_tx_cfg.tx_initial_interval_s);
	ASSERT_EQ(15,    stub_tx_cfg.tx_growth_percent);
	ASSERT_EQ(600,   stub_tx_cfg.tx_max_interval_s);
	ASSERT_EQ(100,   stub_tx_cfg.tx_max_count);
	ASSERT_EQ(1,     stub_sws_cfg.enabled);

	/* New field gets default */
	ASSERT_EQ(MGR_BAT_DEFAULT_MIN_TX_MV, stub_bat_min_tx_mV);
	TEST_PASS();
}

/** After v1 migration, flash contains valid v2 config */
void test_migration_writes_v2_to_flash(void)
{
	reset_test_state();
	write_v1_to_flash(0, 1, 10, 50, 300, 5, 0);

	ASSERT_TRUE(MGR_NVM_load());

	/* Flash should now contain a valid v2 */
	NVM_Config_t cfg;
	memcpy(&cfg, flash_page, sizeof(cfg));
	ASSERT_EQ(NVM_VERSION, cfg.version);
	ASSERT_EQ_HEX(NVM_MAGIC, cfg.magic);

	uint32_t computed = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	ASSERT_EQ_HEX(computed, cfg.crc32);
	TEST_PASS();
}

/** V1 with corrupted CRC -> migration fails */
void test_migration_v1_bad_crc(void)
{
	reset_test_state();
	write_v1_to_flash(0, 0, 10, 0, 300, 5, 1);

	/* Corrupt one byte */
	flash_page[8] ^= 0x01;

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** After migration, a second load succeeds (reads v2 from flash) */
void test_migration_then_reload(void)
{
	reset_test_state();
	write_v1_to_flash(1, 0, 20, 25, 500, 10, 1);

	ASSERT_TRUE(MGR_NVM_load());

	/* Clear stubs and reload */
	memset(&stub_tx_cfg, 0, sizeof(stub_tx_cfg));
	stub_deploy_mode = 0;
	stub_bat_min_tx_mV = 0;

	ASSERT_TRUE(MGR_NVM_load());
	ASSERT_EQ(1,  stub_deploy_mode);
	ASSERT_EQ(20, stub_tx_cfg.tx_initial_interval_s);
	ASSERT_EQ(MGR_BAT_DEFAULT_MIN_TX_MV, stub_bat_min_tx_mV);
	TEST_PASS();
}

/*******************************************************************************
 * NVM SAVE TESTS (V2)
 ******************************************************************************/

/** Save writes correct magic, version, and CRC */
void test_save_writes_valid_header(void)
{
	reset_test_state();
	stub_tx_cfg.tx_initial_interval_s = 10;
	stub_bat_min_tx_mV = 3100;

	ASSERT_TRUE(MGR_NVM_save());

	NVM_Config_t *cfg = (NVM_Config_t *)flash_page;
	ASSERT_EQ_HEX(NVM_MAGIC, cfg->magic);
	ASSERT_EQ(NVM_VERSION, cfg->version);
	ASSERT_EQ(3100, cfg->bat_min_tx_mV);

	uint32_t computed = nvm_crc32(cfg, offsetof(NVM_Config_t, crc32));
	ASSERT_EQ_HEX(computed, cfg->crc32);
	TEST_PASS();
}

/** Save then load round-trip preserves all fields */
void test_save_load_roundtrip(void)
{
	reset_test_state();

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
	stub_bat_min_tx_mV = 3400;

	ASSERT_TRUE(MGR_NVM_save());

	/* Clear stubs, then load */
	memset(&stub_tx_cfg, 0, sizeof(stub_tx_cfg));
	memset(&stub_sws_cfg, 0, sizeof(stub_sws_cfg));
	stub_deploy_mode = 0;
	stub_led_mode = 0;
	stub_bat_min_tx_mV = 0;

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
	ASSERT_EQ(3400,  stub_bat_min_tx_mV);
	TEST_PASS();
}

/** Save with flash write error -> returns false */
void test_save_flash_write_error(void)
{
	reset_test_state();
	flash_write_fail = true;
	ASSERT_FALSE(MGR_NVM_save());
	TEST_PASS();
}

/** Save with bat_min_tx_mV=0 (disabled) round-trips correctly */
void test_save_load_bat_disabled(void)
{
	reset_test_state();
	stub_bat_min_tx_mV = 0;
	ASSERT_TRUE(MGR_NVM_save());

	stub_bat_min_tx_mV = 9999;
	ASSERT_TRUE(MGR_NVM_load());
	ASSERT_EQ(0, stub_bat_min_tx_mV);
	TEST_PASS();
}

/*******************************************************************************
 * NVM RESET TESTS
 ******************************************************************************/

/** Reset fills flash with 0xFF (erased state) */
void test_reset_fills_ff(void)
{
	reset_test_state();
	ASSERT_TRUE(MGR_NVM_save());
	ASSERT_TRUE(MGR_NVM_reset());

	NVM_Config_t *cfg = (NVM_Config_t *)flash_page;
	ASSERT_EQ_HEX(0xFFFFFFFF, cfg->magic);
	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/** Single bit flip in data -> CRC mismatch */
void test_single_bit_corruption(void)
{
	reset_test_state();
	stub_tx_cfg.tx_initial_interval_s = 10;
	ASSERT_TRUE(MGR_NVM_save());

	flash_page[8] ^= 0x01;

	ASSERT_FALSE(MGR_NVM_load());
	TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("MGR_NVM Unit Tests (v2)");

	/* CRC32 tests */
	RUN_TEST(test_crc32_known_vector);
	RUN_TEST(test_crc32_single_byte);
	RUN_TEST(test_crc32_consistency);
	RUN_TEST(test_crc32_different_data);
	RUN_TEST(test_crc32_empty);

	/* Struct layout tests */
	RUN_TEST(test_nvm_config_alignment);
	RUN_TEST(test_nvm_config_doubleword_alignment);
	RUN_TEST(test_nvm_crc32_field_at_end);
	RUN_TEST(test_nvm_magic_at_offset_zero);
	RUN_TEST(test_nvm_bat_field_before_crc);
	RUN_TEST(test_nvm_v1_size);

	/* Load tests (v2) */
	RUN_TEST(test_load_erased_flash);
	RUN_TEST(test_load_wrong_magic);
	RUN_TEST(test_load_unknown_version);
	RUN_TEST(test_load_bad_crc);
	RUN_TEST(test_load_flash_read_error);
	RUN_TEST(test_load_valid_v2_config);

	/* V1 -> V2 migration tests */
	RUN_TEST(test_migration_v1_to_v2);
	RUN_TEST(test_migration_writes_v2_to_flash);
	RUN_TEST(test_migration_v1_bad_crc);
	RUN_TEST(test_migration_then_reload);

	/* Save tests (v2) */
	RUN_TEST(test_save_writes_valid_header);
	RUN_TEST(test_save_load_roundtrip);
	RUN_TEST(test_save_flash_write_error);
	RUN_TEST(test_save_load_bat_disabled);

	/* Reset tests */
	RUN_TEST(test_reset_fills_ff);
	RUN_TEST(test_single_bit_corruption);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
