/**
 * @file    bl_flash.c
 * @brief   Flash memory operations for bootloader
 * @date    2025
 */

#include "bl_flash.h"
#include "stm32wlxx_hal.h"
#include <string.h>

bool bl_flash_init(void)
{
    /* Nothing specific to initialize for flash */
    return true;
}

bl_flash_status_t bl_flash_unlock(void)
{
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return BL_FLASH_ERROR;
    }

    /* Clear any pending flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_lock(void)
{
    if (HAL_FLASH_Lock() != HAL_OK) {
        return BL_FLASH_ERROR;
    }
    return BL_FLASH_OK;
}

uint32_t bl_flash_get_page(uint32_t address)
{
    return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

bl_flash_status_t bl_flash_erase_page(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    HAL_StatusTypeDef status;

    /* Validate address alignment */
    if ((page_addr % FLASH_PAGE_SIZE) != 0) {
        return BL_FLASH_ADDR_ERROR;
    }

    /* Don't allow erasing bootloader region */
    if (bl_flash_addr_in_bootloader(page_addr)) {
        return BL_FLASH_PROTECTED;
    }

    /* Configure erase */
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = bl_flash_get_page(page_addr);
    erase_init.NbPages = 1;

    /* Perform erase */
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    if (status != HAL_OK || page_error != 0xFFFFFFFF) {
        return BL_FLASH_ERROR;
    }

    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_erase_pages(uint32_t start_addr, uint32_t num_pages)
{
    bl_flash_status_t status;

    for (uint32_t i = 0; i < num_pages; i++) {
        uint32_t page_addr = start_addr + (i * FLASH_PAGE_SIZE);
        status = bl_flash_erase_page(page_addr);
        if (status != BL_FLASH_OK) {
            return status;
        }
    }

    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_erase_app(void)
{
    bl_flash_status_t status;

    /* Unlock flash */
    status = bl_flash_unlock();
    if (status != BL_FLASH_OK) {
        return status;
    }

    /* Calculate number of pages in application region */
    /* App region: 0x08000000 to 0x08032FFF (ISR + header + code) */
    uint32_t app_start = APP_FLASH_BASE;
    uint32_t app_end = APP_FLASH_END;
    uint32_t num_pages = ((app_end - app_start) / FLASH_PAGE_SIZE) + 1;

    /* Erase all application pages */
    status = bl_flash_erase_pages(app_start, num_pages);

    /* Lock flash */
    bl_flash_lock();

    return status;
}

bl_flash_status_t bl_flash_write_doubleword(uint32_t address, uint64_t data)
{
    HAL_StatusTypeDef status;

    /* Validate 64-bit alignment */
    if ((address % 8) != 0) {
        return BL_FLASH_ADDR_ERROR;
    }

    /* Don't allow writing to bootloader region */
    if (bl_flash_addr_in_bootloader(address)) {
        return BL_FLASH_PROTECTED;
    }

    /* Program double word */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);

    if (status != HAL_OK) {
        return BL_FLASH_ERROR;
    }

    /* Verify write */
    if (*(__IO uint64_t*)address != data) {
        return BL_FLASH_ERROR;
    }

    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_write(uint32_t address, const void* data, uint32_t length)
{
    bl_flash_status_t status;
    const uint8_t* src = (const uint8_t*)data;
    uint32_t remaining = length;
    uint32_t current_addr = address;

    /* Validate parameters */
    if (data == NULL || length == 0) {
        return BL_FLASH_SIZE_ERROR;
    }

    /* Validate 64-bit alignment */
    if ((address % 8) != 0) {
        return BL_FLASH_ADDR_ERROR;
    }

    /* Unlock flash */
    status = bl_flash_unlock();
    if (status != BL_FLASH_OK) {
        return status;
    }

    /* Write data in 64-bit chunks */
    while (remaining >= 8) {
        uint64_t dword;
        memcpy(&dword, src, 8);

        status = bl_flash_write_doubleword(current_addr, dword);
        if (status != BL_FLASH_OK) {
            bl_flash_lock();
            return status;
        }

        src += 8;
        current_addr += 8;
        remaining -= 8;
    }

    /* Handle remaining bytes (pad with 0xFF) */
    if (remaining > 0) {
        uint64_t dword = 0xFFFFFFFFFFFFFFFF;
        memcpy(&dword, src, remaining);

        status = bl_flash_write_doubleword(current_addr, dword);
        if (status != BL_FLASH_OK) {
            bl_flash_lock();
            return status;
        }
    }

    /* Lock flash */
    bl_flash_lock();

    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_read(uint32_t address, void* buffer, uint32_t length)
{
    if (buffer == NULL || length == 0) {
        return BL_FLASH_SIZE_ERROR;
    }

    /* Validate address is in flash range with overflow protection */
    if (address < FLASH_BASE) {
        return BL_FLASH_ADDR_ERROR;
    }

    /* Check for integer overflow before addition */
    if (length > (UINT32_MAX - address)) {
        return BL_FLASH_ADDR_ERROR;
    }

    uint32_t end_addr = address + length;
    if (end_addr > (FLASH_BASE + BL_FLASH_TOTAL_SIZE)) {
        return BL_FLASH_ADDR_ERROR;
    }

    /* Direct memory read */
    memcpy(buffer, (const void*)address, length);

    return BL_FLASH_OK;
}

bool bl_flash_verify(uint32_t address, const void* data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return false;
    }

    return (memcmp((const void*)address, data, length) == 0);
}

bool bl_flash_addr_in_app(uint32_t address)
{
    return (address >= APP_FLASH_BASE && address <= APP_FLASH_END);
}

bool bl_flash_addr_in_bootloader(uint32_t address)
{
    return (address >= BL_FLASH_BASE && address <= BL_FLASH_END);
}

#if BL_STATE_PERSIST_ENABLED
/* DEAD CODE — disabled because BL_STATE_FLASH_ADDR aliases the credentials page.
 * See bl_config.h (BL_STATE_PERSIST_ENABLED). Production uses TAMP_BKP0R + SRAM. */
bl_flash_status_t bl_flash_read_bl_state(bl_state_flash_t* state)
{
    if (state == NULL) {
        return BL_FLASH_SIZE_ERROR;
    }

    return bl_flash_read(BL_STATE_FLASH_ADDR, state, sizeof(bl_state_flash_t));
}

bl_flash_status_t bl_flash_write_bl_state(const bl_state_flash_t* state)
{
    bl_flash_status_t status;

    if (state == NULL) {
        return BL_FLASH_SIZE_ERROR;
    }

    /* Unlock flash */
    status = bl_flash_unlock();
    if (status != BL_FLASH_OK) {
        return status;
    }

    /* Erase the state page */
    status = bl_flash_erase_page(BL_STATE_FLASH_ADDR);
    if (status != BL_FLASH_OK) {
        bl_flash_lock();
        return status;
    }

    /* Write state structure */
    const uint8_t* src = (const uint8_t*)state;
    uint32_t addr = BL_STATE_FLASH_ADDR;
    uint32_t remaining = sizeof(bl_state_flash_t);

    while (remaining >= 8) {
        uint64_t dword;
        memcpy(&dword, src, 8);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dword) != HAL_OK) {
            bl_flash_lock();
            return BL_FLASH_ERROR;
        }

        src += 8;
        addr += 8;
        remaining -= 8;
    }

    /* Lock flash */
    bl_flash_lock();

    return BL_FLASH_OK;
}

bl_flash_status_t bl_flash_set_dfu_request(bool request)
{
    bl_state_flash_t state;
    bl_flash_status_t status;

    /* Read current state */
    status = bl_flash_read_bl_state(&state);
    if (status != BL_FLASH_OK) {
        /* Initialize new state if read failed */
        memset(&state, 0, sizeof(state));
        state.magic = BL_FLAG_MAGIC;
    }

    /* Check if magic is valid, initialize if not */
    if (state.magic != BL_FLAG_MAGIC) {
        memset(&state, 0, sizeof(state));
        state.magic = BL_FLAG_MAGIC;
    }

    /* Update flag */
    if (request) {
        state.flags |= BL_FLAG_DFU_REQUEST;
    } else {
        state.flags &= ~BL_FLAG_DFU_REQUEST;
    }

    /* Write back */
    return bl_flash_write_bl_state(&state);
}
#endif /* BL_STATE_PERSIST_ENABLED */

/* DFU request flag locations - shared with app
 * Primary: RTC backup registers (survives all resets except VBAT removal)
 * Fallback: SRAM (requires SRAM_RST option byte)
 * Note: DFU_REQUEST_MAGIC, SRAM_DFU_FLAG_ADDR, TAMP_BKP0R_ADDR defined in bl_config.h
 */
#define SRAM_DFU_FLAG_PTR       (*((volatile uint32_t *)SRAM_DFU_FLAG_ADDR))
#define RTC_DFU_FLAG_PTR        (*((volatile uint32_t *)TAMP_BKP0R_ADDR))

/* Enable backup domain access for RTC backup registers */
static void enable_backup_access_bl(void)
{
    /* Enable RTC APB clock - required to access TAMP registers */
    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
    __DSB();
    /* Small delay for clock to settle */
    BL_SETTLE_DELAY(100);

    /* Enable backup domain access (PWR is always accessible on STM32WL) */
    PWR->CR1 |= PWR_CR1_DBP;
    while ((PWR->CR1 & PWR_CR1_DBP) == 0);
    __DSB();
    /* Additional delay for backup domain to be fully accessible */
    BL_SETTLE_DELAY(100);
}

bool bl_flash_is_dfu_requested(void)
{
    /* Enable backup domain access first */
    enable_backup_access_bl();

    /* Check DFU request flag - first in RTC backup register, then SRAM */
    if (RTC_DFU_FLAG_PTR == DFU_REQUEST_MAGIC) {
        return true;
    }
    /* Fallback: check SRAM location */
    if (SRAM_DFU_FLAG_PTR == DFU_REQUEST_MAGIC) {
        return true;
    }
    return false;
}

bl_flash_status_t bl_flash_clear_dfu_request(void)
{
    /* Enable backup domain access first */
    enable_backup_access_bl();

    /* Clear the DFU request flag in both locations */
    RTC_DFU_FLAG_PTR = 0;
    SRAM_DFU_FLAG_PTR = 0;
    __DSB();

    return BL_FLASH_OK;
}
