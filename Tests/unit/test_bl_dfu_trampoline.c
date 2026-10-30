/**
 * @file    test_bl_dfu_trampoline.c
 * @brief   Bootloader DFU: page-0 trampoline, deferred page-0 commit, idempotent resend.
 *
 * Runs the real Bootloader/Src/bl_dfu.c + bl_crc.c against a simulated flash.
 * Invariant under test: at every point of an update, a reset or a power cut
 * must boot the bootloader (never the ST ROM via an erased 0x08000000, never a
 * half-written app); the app vector appears only after a passing VERIFY.
 */

#include "test_framework.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "bl_main.h"
#include "bl_spi_protocol.h"

/* ---- Simulated flash (whole 256 KB bank) -------------------------------- */
#define SIM_BASE    0x08000000UL
#define SIM_SIZE    (256UL * 1024UL)
#define PAGE        BL_FLASH_PAGE_SIZE

static uint8_t  sim[SIM_SIZE];
static bool     sim_unlocked;
static int      fail_page0_write_once;   /* fail the next 2 KB write at page 0 */

#define AT(addr) (&sim[(addr) - SIM_BASE])

bl_flash_status_t bl_flash_unlock(void) { sim_unlocked = true; return BL_FLASH_OK; }
bl_flash_status_t bl_flash_lock(void)   { sim_unlocked = false; return BL_FLASH_OK; }

bl_flash_status_t bl_flash_erase_page(uint32_t page_addr)
{
    if (!sim_unlocked || (page_addr % PAGE) != 0 || page_addr < SIM_BASE ||
        page_addr >= SIM_BASE + SIM_SIZE) {
        return BL_FLASH_ERROR;
    }
    memset(AT(page_addr), 0xFF, PAGE);
    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_erase_pages(uint32_t start_addr, uint32_t num_pages)
{
    for (uint32_t i = 0; i < num_pages; i++) {
        bl_flash_status_t s = bl_flash_erase_page(start_addr + i * PAGE);
        if (s != BL_FLASH_OK) {
            return s;
        }
    }
    return BL_FLASH_OK;
}

/* Programs only erased double-words, like the STM32WL (PROGERR otherwise). */
bl_flash_status_t bl_flash_write(uint32_t address, const void* data, uint32_t length)
{
    static const uint8_t erased[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if ((address % 8) != 0 || (length % 8) != 0 || address < SIM_BASE ||
        address + length > SIM_BASE + SIM_SIZE) {
        return BL_FLASH_ADDR_ERROR;
    }
    if (fail_page0_write_once && address == APP_FLASH_BASE && length == PAGE) {
        fail_page0_write_once = 0;
        return BL_FLASH_ERROR;
    }
    for (uint32_t i = 0; i < length; i += 8) {
        if (memcmp(AT(address + i), erased, 8) != 0) {
            return BL_FLASH_ERROR;
        }
    }
    memcpy(AT(address), data, length);
    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_read(uint32_t address, void* buffer, uint32_t length)
{
    memcpy(buffer, AT(address), length);
    return BL_FLASH_OK;
}

bool bl_flash_verify(uint32_t address, const void* data, uint32_t length)
{
    return memcmp(AT(address), data, length) == 0;
}

bool bl_flash_addr_in_app(uint32_t address)
{
    return address >= APP_FLASH_BASE && address <= APP_FLASH_END;
}

/* ---- Stubs for the rest of the bootloader -------------------------------- */
bl_state_t bl_get_state(void) { return (bl_state_t)0; }
const char* bl_get_version_string(void) { return "BL_V1.0.0"; }
void bl_spi_protocol_build_extended_status(bl_extended_status_t *status) { (void)status; }

#include "../../Bootloader/Src/bl_crc.c"
#include "../../Bootloader/Src/bl_dfu.c"

/* ---- Fixtures ------------------------------------------------------------ */
#define BL_SP       0x20008000UL
#define BL_RESET    0x080387A1UL
#define APP_SP      0x20008000UL
#define APP_RESET   0x0800B55DUL
#define IMG_SIZE    5003U           /* 20 x 248 + 43: last chunk not 8-aligned */
#define CHUNK       BL_CHUNK_SIZE

static uint8_t image[IMG_SIZE];

/* Independent CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no reflect, no xorout). */
static uint32_t ref_crc(const uint8_t* d, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= (uint32_t)d[i] << 24;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80000000UL) ? (crc << 1) ^ 0x04C11DB7UL : (crc << 1);
        }
    }
    return crc;
}

static void put32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }

static void setup(void)
{
    memset(sim, 0xFF, sizeof(sim));
    /* Running app: valid vector + content in every app page. */
    memset(AT(APP_FLASH_BASE), 0xA5, APP_FLASH_SIZE);
    put32(AT(APP_FLASH_BASE), APP_SP);
    put32(AT(APP_FLASH_BASE + 4), APP_RESET);
    /* Bootloader vector table. */
    put32(AT(BL_FLASH_BASE), BL_SP);
    put32(AT(BL_FLASH_BASE + 4), BL_RESET);
    put32(AT(BL_FLASH_BASE + 8), 0x080387EDUL);
    put32(AT(BL_FLASH_BASE + 12), 0x08034085UL);
    /* Data regions that DFU must never touch. */
    memset(AT(0x08032000UL), 0x5C, 0x1000);          /* FLASH_PMLOG */
    memset(AT(0x0803B000UL), 0xC3, 0x5000);          /* FLASH_USER  */

    for (uint32_t i = 0; i < IMG_SIZE; i++) {
        image[i] = (uint8_t)(i * 37U + 11U);
    }
    put32(&image[0], APP_SP);
    put32(&image[4], APP_RESET);

    sim_unlocked = false;
    fail_page0_write_once = 0;
    bl_crc_init();
    bl_dfu_init();
}

static dfu_response_t cmd(dfu_cmd_t c, const uint8_t* data, uint16_t len)
{
    uint8_t resp[64];
    uint16_t rlen = sizeof(resp);
    return bl_dfu_process_cmd(c, data, len, resp, &rlen);
}

static dfu_response_t write_chunk(uint32_t addr, const uint8_t* data, uint16_t len)
{
    uint8_t buf[4 + BL_CHUNK_SIZE];
    put32(buf, addr);
    memcpy(buf + 4, data, len);
    return cmd(DFU_CMD_WRITE, buf, (uint16_t)(4 + len));
}

static uint32_t n_chunks(void) { return (IMG_SIZE + CHUNK - 1) / CHUNK; }

static dfu_response_t send_chunk(uint32_t k)
{
    uint32_t off = k * CHUNK;
    uint32_t len = (IMG_SIZE - off < CHUNK) ? IMG_SIZE - off : CHUNK;
    return write_chunk(APP_FLASH_BASE + off, &image[off], (uint16_t)len);
}

static dfu_response_t verify(uint32_t crc)
{
    uint8_t b[4];
    put32(b, crc);
    return cmd(DFU_CMD_VERIFY, b, 4);
}

typedef enum { BOOT_ST_ROM, BOOT_BL, BOOT_APP, BOOT_OTHER } boot_t;

/* What the core runs after a reset: POR with erased 0x08000000 = ST ROM. */
static boot_t boot_target(void)
{
    uint32_t sp, pc;
    memcpy(&sp, AT(APP_FLASH_BASE), 4);
    memcpy(&pc, AT(APP_FLASH_BASE + 4), 4);
    if (sp == 0xFFFFFFFFUL) return BOOT_ST_ROM;
    if (pc >= BL_FLASH_BASE && pc <= BL_FLASH_END) return BOOT_BL;
    if (pc >= APP_FLASH_BASE && pc < APP_FLASH_BASE + APP_FLASH_SIZE) return BOOT_APP;
    return BOOT_OTHER;
}

/* Mirror of the bl_check_app_valid SP/PC fallback (bl_main.c). */
static bool app_vector_plausible(void)
{
    uint32_t sp, pc;
    memcpy(&sp, AT(APP_FLASH_BASE), 4);
    memcpy(&pc, AT(APP_FLASH_BASE + 4), 4);
    return sp >= SRAM_BASE_ADDR && sp <= SRAM_END_ADDR &&
           pc >= APP_FLASH_BASE && pc <= APP_FLASH_BASE + APP_FLASH_SIZE;
}

static bool all_ff(uint32_t addr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        if (*AT(addr + i) != 0xFF) return false;
    }
    return true;
}

static bool untouched_data_regions(void)
{
    for (uint32_t i = 0; i < 0x1000; i++) if (*AT(0x08032000UL + i) != 0x5C) return false;
    for (uint32_t i = 0; i < 0x5000; i++) if (*AT(0x0803B000UL + i) != 0xC3) return false;
    return true;
}

/* ---- Tests ---------------------------------------------------------------- */

static void test_crc_algorithm_pinned(void)
{
    TEST_START("bootloader CRC is CRC-32/MPEG-2 (check value 0x0376E6E7)");
    bl_crc_init();
    ASSERT_EQ_HEX(0x0376E6E7UL, bl_crc32_calculate("123456789", 9));
    TEST_PASS();
}

static void test_erase_installs_trampoline(void)
{
    TEST_START("ERASE leaves a bootloader trampoline in page 0");
    setup();
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    ASSERT_MEM_EQ(AT(BL_FLASH_BASE), AT(APP_FLASH_BASE), 16);
    ASSERT_TRUE(all_ff(APP_FLASH_BASE + 16, PAGE - 16));        /* no KINE magic at 0x200 */
    ASSERT_TRUE(all_ff(APP_FLASH_BASE + PAGE, APP_FLASH_SIZE - PAGE));
    ASSERT_TRUE(untouched_data_regions());
    ASSERT_EQ(BOOT_BL, boot_target());
    ASSERT_FALSE(app_vector_plausible());
    TEST_PASS();
}

static void test_full_update_commits_page0_last(void)
{
    TEST_START("full update: page 0 programmed only by a passing VERIFY");
    setup();
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 0; k < n_chunks(); k++) {
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
        ASSERT_EQ(BOOT_BL, boot_target());
    }
    ASSERT_MEM_EQ(&image[PAGE], AT(APP_FLASH_BASE + PAGE), IMG_SIZE - PAGE);
    ASSERT_EQ(DFU_RSP_OK, verify(ref_crc(image, IMG_SIZE)));
    ASSERT_MEM_EQ(image, AT(APP_FLASH_BASE), IMG_SIZE);
    ASSERT_TRUE(all_ff(APP_FLASH_BASE + IMG_SIZE, 5));          /* last chunk 0xFF pad */
    ASSERT_EQ(BOOT_APP, boot_target());
    ASSERT_TRUE(bl_dfu_can_jump());
    ASSERT_TRUE(untouched_data_regions());
    TEST_PASS();
}

static void test_reset_at_every_step_boots_bootloader(void)
{
    TEST_START("cut after ERASE / any chunk / failed VERIFY: boots the bootloader");
    for (uint32_t stop = 0; stop <= n_chunks(); stop++) {
        setup();
        ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
        for (uint32_t k = 0; k < stop; k++) {
            ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
        }
        ASSERT_EQ(BOOT_BL, boot_target());
        ASSERT_FALSE(app_vector_plausible());
    }
    ASSERT_EQ(DFU_RSP_CRC_ERROR, verify(0xDEADBEEFUL));
    ASSERT_EQ(BOOT_BL, boot_target());
    ASSERT_FALSE(bl_dfu_can_jump());
    TEST_PASS();
}

static void test_resend_identical_is_idempotent(void)
{
    TEST_START("resend of the last chunk (page 0, straddling, flash) is acked once");
    setup();
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 0; k < n_chunks(); k++) {
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));           /* chunk 8 straddles page 0/1 */
    }
    ASSERT_EQ(DFU_RSP_OK, verify(ref_crc(image, IMG_SIZE)));  /* CRC counted once */
    ASSERT_MEM_EQ(image, AT(APP_FLASH_BASE), IMG_SIZE);
    TEST_PASS();
}

static void test_resend_different_rejected(void)
{
    TEST_START("same address, different data: ADDR_ERROR, image unaffected");
    setup();
    uint8_t bad[CHUNK];
    memset(bad, 0x42, sizeof(bad));
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 0; k < n_chunks(); k++) {
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
        if (k == 0 || k == 8 || k == 12) {
            ASSERT_EQ(DFU_RSP_ADDR_ERROR, write_chunk(APP_FLASH_BASE + k * CHUNK, bad, CHUNK));
        }
    }
    ASSERT_EQ(DFU_RSP_OK, verify(ref_crc(image, IMG_SIZE)));
    ASSERT_MEM_EQ(image, AT(APP_FLASH_BASE), IMG_SIZE);
    TEST_PASS();
}

static void test_second_session_starts_clean(void)
{
    TEST_START("ERASE again in the same session resets page-0 buffer and CRC");
    setup();
    uint8_t junk[64];
    memset(junk, 0x11, sizeof(junk));
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    ASSERT_EQ(DFU_RSP_OK, write_chunk(APP_FLASH_BASE, junk, sizeof(junk)));
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 0; k < n_chunks(); k++) {
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
    }
    ASSERT_EQ(DFU_RSP_OK, verify(ref_crc(image, IMG_SIZE)));
    ASSERT_MEM_EQ(image, AT(APP_FLASH_BASE), IMG_SIZE);
    TEST_PASS();
}

static void test_abort_keeps_trampoline(void)
{
    TEST_START("ABORT mid-update keeps the trampoline, WRITE then needs ERASE");
    setup();
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    ASSERT_EQ(DFU_RSP_OK, send_chunk(0));
    ASSERT_EQ(DFU_RSP_OK, send_chunk(1));
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ABORT, NULL, 0));
    ASSERT_EQ(BOOT_BL, boot_target());
    ASSERT_EQ(DFU_RSP_NOT_READY, send_chunk(2));
    TEST_PASS();
}

static void test_image_without_vector_keeps_trampoline(void)
{
    TEST_START("image without a bootable vector: VERIFY refuses to commit page 0");
    setup();
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 9; k < n_chunks(); k++) {                 /* page 0 never sent */
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
    }
    ASSERT_EQ(DFU_RSP_INVALID_HEADER, verify(ref_crc(&image[9 * CHUNK], IMG_SIZE - 9 * CHUNK)));
    ASSERT_EQ(BOOT_BL, boot_target());
    ASSERT_FALSE(bl_dfu_can_jump());
    TEST_PASS();
}

static void test_commit_failure_reinstalls_trampoline(void)
{
    TEST_START("page-0 commit failure: FLASH_ERROR, trampoline back, retry works");
    setup();
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 0; k < n_chunks(); k++) {
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
    }
    fail_page0_write_once = 1;
    ASSERT_EQ(DFU_RSP_FLASH_ERROR, verify(ref_crc(image, IMG_SIZE)));
    ASSERT_EQ(BOOT_BL, boot_target());
    ASSERT_FALSE(bl_dfu_can_jump());
    ASSERT_EQ(DFU_RSP_OK, verify(ref_crc(image, IMG_SIZE)));      /* host retry */
    ASSERT_EQ(BOOT_APP, boot_target());
    TEST_PASS();
}

static void test_set_header_buffered(void)
{
    TEST_START("SET_HEADER: needs ERASE, lands in page 0 only at VERIFY");
    setup();
    uint8_t hdr[sizeof(app_header_t)];
    memset(hdr, 0, sizeof(hdr));
    put32(hdr, APP_HEADER_MAGIC);

    ASSERT_EQ(DFU_RSP_NOT_READY, cmd(DFU_CMD_SET_HEADER, hdr, sizeof(hdr)));
    ASSERT_EQ(BOOT_APP, boot_target());                         /* running app intact */

    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_ERASE, NULL, 0));
    for (uint32_t k = 0; k < n_chunks(); k++) {
        ASSERT_EQ(DFU_RSP_OK, send_chunk(k));
    }
    ASSERT_EQ(DFU_RSP_OK, cmd(DFU_CMD_SET_HEADER, hdr, sizeof(hdr)));
    ASSERT_TRUE(all_ff(APP_HEADER_ADDR, sizeof(hdr)));
    ASSERT_EQ(DFU_RSP_OK, verify(ref_crc(image, IMG_SIZE)));
    ASSERT_MEM_EQ(hdr, AT(APP_HEADER_ADDR), sizeof(hdr));
    ASSERT_EQ(BOOT_APP, boot_target());
    TEST_PASS();
}

int main(void)
{
    printf("\n=== Bootloader DFU: page-0 trampoline ===\n");
    test_crc_algorithm_pinned();
    test_erase_installs_trampoline();
    test_full_update_commits_page0_last();
    test_reset_at_every_step_boots_bootloader();
    test_resend_identical_is_idempotent();
    test_resend_different_rejected();
    test_second_session_starts_clean();
    test_abort_keeps_trampoline();
    test_image_without_vector_keeps_trampoline();
    test_commit_failure_reinstalls_trampoline();
    test_set_header_buffered();

    printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
    return tests_failed ? 1 : 0;
}
