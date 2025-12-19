/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_flash.c
 * @brief   MCU flash memory system
 * @author  Arribada
 * @date    Creation 2022/01/31
 */

/**
 * @page mcu_flash_page MCU flash
 *
 * Flash memory is used to store ID, ADDR, Secret Key
 *  TODO :RIGH now memory size is not optimized.
 *
 * @note
 *
 */

/**
 * @addtogroup MCU_FLASH
 * @brief MCU FLASH
 * @{
 */


#include "mcu_flash.h"
#include "kns_types.h"
#include <string.h>
#include "stm32wlxx_hal.h"
#include "mgr_log.h" /* @note This log is for debug, can be deleted */
#pragma GCC push_options
#pragma GCC optimize("O0")
#define FLASH_WAIT_TIMEOUT_MS   500U   // à ajuster si besoin

static enum KNS_status_t MCU_FLASH_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    // Attente de la fin de l'opération en cours (BSY à 0)
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            MGR_LOG_DEBUG("FLASH TIMEOUT: BSY still set after %lu ms\r\n", timeout_ms);
            return KNS_STATUS_FLASH_ERR;   // ou KNS_STATUS_TIMEOUT si tu as un code dédié
        }
    }

    // On nettoie les flags d'erreur éventuels avant de continuer
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    |
                           FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR |
                           FLASH_FLAG_SIZERR |
                           FLASH_FLAG_PGSERR);

    return KNS_STATUS_OK;
}

/**
 * @brief Reads data from flash memory byte-by-byte.
 *
 * @param dest Pointer to the buffer where the data will be stored.
 * @param ofset Flash memory address to read from.
 * @param length Number of bytes to read.
 * @return KNS_status_t Status of the operation.
 */

enum KNS_status_t MCU_FLASH_read(uint32_t address, void *buffer, size_t size)
{
    if (!buffer || size == 0) return KNS_STATUS_FLASH_ERR; // Safety check
    memcpy(buffer, (void*)address, size);
    return KNS_STATUS_OK;
}

/**
 * @brief Writes data to flash memory byte-by-byte.
 *
 * @param src Pointer to the buffer containing the data.
 * @param ofset Flash memory address to write to.
 * @param length Number of bytes to write.
 * @return KNS_status_t Status of the operation.
 */
enum KNS_status_t MCU_FLASH_write(uint32_t address, const void *data, size_t size)
{
    if (!data || size == 0) {
		MGR_LOG_DEBUG("Data and/or size are 0\r\n");
    	return KNS_STATUS_FLASH_ERR; // Safety check
    }

    // 1) Vérifier que l’adresse est bien dans la zone FLASH_USER
    if (address < FLASH_USER_START_ADDR ||
        (address + size) > (FLASH_USER_END_ADDR + 1U)) {
        MGR_LOG_DEBUG("FLASH WRITE ERROR: address out of FLASH_USER range "
                      "(addr=0x%08lx, size=%lu)\r\n",
                      address, size);
        return KNS_STATUS_FLASH_ERR;
    }
	MGR_LOG_DEBUG("Ok ready to write flash \r\n");

    // 2) Cette fonction ne gère qu’une seule page (comme ton code)
    uint32_t page_start_addr = address - (address % FLASH_PAGE_SIZE);
    uint32_t page_end_addr   = page_start_addr + FLASH_PAGE_SIZE;

    if ((address + size) > page_end_addr) {
		 MGR_LOG_DEBUG("FLASH WRITE ERROR: write crosses page boundary "
		                      "(addr=0x%08lx, size=%lu)\r\n",
		                      address, size);
        return KNS_STATUS_FLASH_ERR;
    }

    // 3) Alignement adresse sur 64 bits
    if ((address & 0x7U) != 0U) {
		 MGR_LOG_DEBUG("FLASH WRITE ERROR: address not 64-bit aligned "
		                      "(addr=0x%08lx)\r\n",
		                      address);
        return KNS_STATUS_FLASH_ERR;
    }

    MGR_LOG_DEBUG("Ok ready to write flash (addr=0x%08lx, size=%lu)\r\n",
	                  address, size);
    uint64_t page_backup[FLASH_PAGE_SIZE / sizeof(uint64_t)];
    uint8_t *backup_bytes = (uint8_t *)page_backup;

    // Backup current page content from flash
	memcpy(backup_bytes, (const void *)page_start_addr, FLASH_PAGE_SIZE);

	// Modify only the region in the buffer that needs updating
	memcpy(backup_bytes + (address - page_start_addr), data, size);


    HAL_FLASH_Unlock();
    __disable_irq();
    /* Attendre que la flash soit prête avant d'effacer la page */
    if (MCU_FLASH_WaitReady(FLASH_WAIT_TIMEOUT_MS) != KNS_STATUS_OK) {
        __enable_irq();
        HAL_FLASH_Lock();
        return KNS_STATUS_FLASH_ERR;
    }

    // Erase the Flash page
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = (page_start_addr - FLASH_BASE) / FLASH_PAGE_SIZE,
    EraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        __enable_irq();
        return KNS_STATUS_FLASH_ERR;
    }

    // Re-write full page to Flash
//    for (uint32_t i = 0; i < FLASH_PAGE_SIZE / sizeof(uint64_t); i++) {
//    	/* Attendre que la flash soit prête avant d'effacer la page */
//    	if (MCU_FLASH_WaitReady(FLASH_WAIT_TIMEOUT_MS) != KNS_STATUS_OK) {
//    	    __enable_irq();
//    	    HAL_FLASH_Lock();
//    	    return KNS_STATUS_FLASH_ERR;
//    	}
//
//        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, page_start_addr  + (i * sizeof(uint64_t)), page_backup[i]) != HAL_OK) {
//            __enable_irq();
//            HAL_FLASH_Lock();
//            return KNS_STATUS_FLASH_ERR;
//        }
//    }

    for (uint32_t i = 0; i < FLASH_PAGE_SIZE / sizeof(uint64_t); i++) {

        uint32_t target_addr = page_start_addr + (i * sizeof(uint64_t));
        HAL_StatusTypeDef hal_status = HAL_ERROR;
        enum KNS_status_t wait_status;
        uint8_t retry = 0;

        // On tente jusqu'à 3 fois pour ce double-word
        while (retry < 3) {

            // Attendre que la flash soit prête (avec timeout)
            wait_status = MCU_FLASH_WaitReady(FLASH_WAIT_TIMEOUT_MS);
            if (wait_status != KNS_STATUS_OK) {
                MGR_LOG_DEBUG("FLASH PROGRAM TIMEOUT: addr=0x%08lx (try %u)\r\n",
                              target_addr, (unsigned)(retry + 1));
                retry++;
                continue;   // on retente
            }

            // Tentative de programmation
            hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                           target_addr,
                                           page_backup[i]);

            if (hal_status == HAL_OK) {
                // OK, on sort de la boucle de retry pour ce double-word
                break;
            }

			#ifdef DEBUG
            uint32_t err = HAL_FLASH_GetError();
            MGR_LOG_DEBUG("FLASH PROGRAM ERROR: addr=0x%08lx, err=0x%08lx (try %u)\r\n",
                          target_addr, err, (unsigned)(retry + 1));
			#endif


            retry++;
        }

        // Après 3 tentatives, si ce n'est toujours pas OK → on abandonne
        if (hal_status != HAL_OK) {
            __enable_irq();
            HAL_FLASH_Lock();
            return KNS_STATUS_FLASH_ERR;
        }
    }
    __DSB();
    __ISB();
    __enable_irq();
    HAL_FLASH_Lock();
    return KNS_STATUS_OK;
}


/**
 * @brief Read a 64-bit word from flash memory.
 * @param address Address to read from.
 * @return 64-bit value at the given address.
 */
static inline uint64_t read_flash_word(uint32_t address) {
    return *(volatile uint64_t*)address;
}

/**
 * @brief Write a 64-bit word to flash memory.
 * @param address Address to write to.
 * @param value 64-bit value to write.
 * @return HAL status of the operation.
 */
static HAL_StatusTypeDef write_flash_word(uint32_t address, uint64_t value) {
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value);
    HAL_FLASH_Lock();
    return status;
}

/**
 * @brief Read the current wear-leveled counter value.
 * @param start_addr Start address of the wear-leveling area.
 * @param wl_size_words Number of 64-bit words in the wear-leveling area.
 * @param overflow_addr Address where the overflow counter is stored.
 * @return Current counter value.
 */
uint64_t read_wear_counter(uint32_t start_addr, uint32_t wl_size_words, uint32_t overflow_addr) {
    uint32_t valid_index = 0;
    for (uint32_t i = 0; i < wl_size_words; ++i) {
        uint64_t val = read_flash_word(start_addr + i * 8);
        if (val == 0xFFFFFFFFFFFFFFFFULL) break;
        valid_index = i + 1;
    }
    uint64_t overflow = *(uint64_t*)overflow_addr;
    return overflow * wl_size_words + valid_index;
}

/**
 * @brief Increment the wear-leveled counter.
 * @param wl_start Start address of the wear-leveling area.
 * @param wl_size Size in 64-bit words.
 * @param of_addr Address of the overflow counter.
 * @return KNS status of the operation.
 */
enum KNS_status_t increment_wear_counter(uint32_t wl_start, uint32_t wl_size, uint32_t of_addr)
{
    uint32_t current_index = 0;
    for (uint32_t i = 0; i < wl_size; ++i) {
        if (read_flash_word(wl_start + i * 8) == 0xFFFFFFFFFFFFFFFFULL) {
            current_index = i;
            break;
        }
    }

    if (current_index < wl_size) {
        return write_flash_word(wl_start + current_index * 8, 0);
    } else {
        // Full, reset area and increment overflow
        uint64_t of = *(uint64_t*)of_addr;
        ++of;
        if(MCU_FLASH_write(of_addr, &of, sizeof(of)) != KNS_STATUS_OK) {
        //if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, of_addr, of) != HAL_OK) {
            HAL_FLASH_Lock();
            return KNS_STATUS_FLASH_ERR;
        }

        HAL_FLASH_Unlock();
        FLASH_EraseInitTypeDef erase = {
            .TypeErase = FLASH_TYPEERASE_PAGES,
            .Page = (wl_start - FLASH_BASE) / FLASH_PAGE_SIZE,
            .NbPages = (wl_size * 8) / FLASH_PAGE_SIZE
        };
        uint32_t error;
        if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
            HAL_FLASH_Lock();
            return KNS_STATUS_FLASH_ERR;
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, wl_start, 0) != HAL_OK) {
            HAL_FLASH_Lock();
            return KNS_STATUS_FLASH_ERR;
        }
        HAL_FLASH_Lock();
        return KNS_STATUS_OK;
    }
}

/**
 * @brief Reset the wear-leveled counter and overflow.
 * @param wl_start Start address of the wear-leveling area.
 * @param wl_size Size in 64-bit words.
 * @param of_addr Address of the overflow counter.
 * @return KNS status of the operation.
 */
enum KNS_status_t reset_wear_counter(uint32_t wl_start, uint32_t wl_size, uint32_t of_addr) {

    uint64_t of = 0;
    if(MCU_FLASH_write(of_addr, &of, sizeof(of)) != KNS_STATUS_OK) {
    //if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, of_addr, of) != HAL_OK) {
        HAL_FLASH_Lock();
        return KNS_STATUS_FLASH_ERR;
    }
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page = (wl_start - FLASH_BASE) / FLASH_PAGE_SIZE,
        .NbPages = (wl_size * 8) / FLASH_PAGE_SIZE
    };
    uint32_t error;
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
        HAL_FLASH_Lock();
        return KNS_STATUS_FLASH_ERR;
    }
    HAL_FLASH_Lock();
    return KNS_STATUS_OK;
}

/**
 * @brief Set the wear-leveled counter to a specific value.
 * @param wl_start Start address of the wear-leveling area.
 * @param wl_size Size in 64-bit words.
 * @param of_addr Address of the overflow counter.
 * @param value New value to set the counter to.
 * @return KNS status of the operation.
 */
enum KNS_status_t set_wear_counter(uint32_t wl_start, uint32_t wl_size, uint32_t of_addr, uint64_t value)
{
    uint64_t of = value / wl_size;
    uint32_t index = value % wl_size;

    uint32_t error;
    if(MCU_FLASH_write(of_addr, &of, sizeof(of)) != KNS_STATUS_OK) {
    //if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, of_addr, of) != HAL_OK) {
        HAL_FLASH_Lock();
        return KNS_STATUS_FLASH_ERR;
    }

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page = (wl_start - FLASH_BASE) / FLASH_PAGE_SIZE,
        .NbPages = (wl_size * 8) / FLASH_PAGE_SIZE
    };
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
        HAL_FLASH_Lock();
        return KNS_STATUS_FLASH_ERR;
    }

    for (uint32_t i = 0; i < index; ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, wl_start + i * 8, 0) != HAL_OK) {
            HAL_FLASH_Lock();
            return KNS_STATUS_FLASH_ERR;
        }
    }
    HAL_FLASH_Lock();
    return KNS_STATUS_OK;
}

#pragma GCC pop_options
/**
 * @brief Read the MSG counter value.
 * @return Current MSG counter value.
 */
uint64_t MCU_FLASH_read_msg_counter(void) {
    return read_wear_counter(FLASH_MSG_COUNTER_WL_START_ADDR, FLASH_MSG_COUNTER_WL_SIZE, FLASH_MSG_COUNTER_OF_ADDR);
}

/**
 * @brief Increment the MSG counter.
 * @return KNS status of the operation.
 */
enum KNS_status_t MCU_FLASH_increment_msg_counter(void) {
    return increment_wear_counter(FLASH_MSG_COUNTER_WL_START_ADDR, FLASH_MSG_COUNTER_WL_SIZE, FLASH_MSG_COUNTER_OF_ADDR);
}

/**
 * @brief Reset the MSG counter.
 * @return KNS status of the operation.
 */
enum KNS_status_t MCU_FLASH_reset_msg_counter(void) {
    return reset_wear_counter(FLASH_MSG_COUNTER_WL_START_ADDR, FLASH_MSG_COUNTER_WL_SIZE, FLASH_MSG_COUNTER_OF_ADDR);
}

/**
 * @brief Set the MSG counter to a specific value.
 * @param value New value to set.
 * @return KNS status of the operation.
 */
enum KNS_status_t MCU_FLASH_set_msg_counter(uint64_t value) {
    return set_wear_counter(FLASH_MSG_COUNTER_WL_START_ADDR, FLASH_MSG_COUNTER_WL_SIZE, FLASH_MSG_COUNTER_OF_ADDR, value);
}

/**
 * @brief Read the WKU counter value.
 * @return Current WKU counter value.
 */
uint64_t MCU_FLASH_read_wku_counter(void) {
    return read_wear_counter(FLASH_WKU_COUNTER_WL_START_ADDR, FLASH_WKU_COUNTER_WL_SIZE, FLASH_WKU_COUNTER_OF_ADDR);
}

/**
 * @brief Increment the WKU counter.
 * @return KNS status of the operation.
 */
enum KNS_status_t MCU_FLASH_increment_wku_counter(void) {
    return increment_wear_counter(FLASH_WKU_COUNTER_WL_START_ADDR, FLASH_WKU_COUNTER_WL_SIZE, FLASH_WKU_COUNTER_OF_ADDR);
}

/**
 * @brief Reset the WKU counter.
 * @return KNS status of the operation.
 */
enum KNS_status_t MCU_FLASH_reset_wku_counter(void) {
    return reset_wear_counter(FLASH_WKU_COUNTER_WL_START_ADDR, FLASH_WKU_COUNTER_WL_SIZE, FLASH_WKU_COUNTER_OF_ADDR);
}

/**
 * @brief Set the WKU counter to a specific value.
 * @param value New value to set.
 * @return KNS status of the operation.
 */
enum KNS_status_t MCU_FLASH_set_wku_counter(uint64_t value) {
    return set_wear_counter(FLASH_WKU_COUNTER_WL_START_ADDR, FLASH_WKU_COUNTER_WL_SIZE, FLASH_WKU_COUNTER_OF_ADDR, value);
}








// enum KNS_status_t MCU_FLASH_increment_counter(void)
// {
//     uint64_t counter = 0;
//     uint32_t address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER;

//     MGR_LOG_DEBUG("MCU_FLASH_increment_counter\r\n");

//     // Read the existing counter value
//     if (MCU_FLASH_read(address, &counter, sizeof(counter)) != KNS_STATUS_OK) {
//         return KNS_STATUS_FLASH_ERR;
//     }

//     // Handle uninitialized flash (all 1s)
//     if (counter == 0xFFFFFFFFFFFFFFFF) {
//         counter = 1;
//     } else {
//         counter++;
//     }

//     // Write the updated counter value using the existing full-page-safe write
//     return MCU_FLASH_write(address, &counter, sizeof(counter));
// }

// enum KNS_status_t MCU_FLASH_set_counter(uint64_t *counter)
// {
//     uint32_t address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER;

//     return MCU_FLASH_write(address, counter, sizeof(*counter));
// }

// enum KNS_status_t MCU_FLASH_get_latest_counter(uint64_t *counter)
// {
//     if (!counter) return KNS_STATUS_FLASH_ERR;

//     uint32_t address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER;

//     if (MCU_FLASH_read(address, counter, sizeof(*counter)) != KNS_STATUS_OK) {
//         return KNS_STATUS_FLASH_ERR;
//     }

//     // If flash is uninitialized (all bits 1), treat it as zero
//     if (*counter == 0xFFFFFFFFFFFFFFFF) {
//         *counter = 0;
//     }

//     return KNS_STATUS_OK;
// }

// enum KNS_status_t MCU_FLASH_reset_counter(void)
// {
//     uint64_t reset_value = 0;
//     uint32_t address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER;

//     MGR_LOG_DEBUG("MCU_FLASH_reset_counter\r\n");

//     return MCU_FLASH_write(address, &reset_value, sizeof(reset_value));
// }


//enum KNS_status_t MCU_FLASH_increment_counter(void)
//{
//    uint64_t counter = 0;
//    uint32_t write_address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER;
//
//    MGR_LOG_DEBUG("MCU_FLASH_increment_counter\r\n");
//    if (MCU_FLASH_is_counter_full()) {
//        return KNS_STATUS_FLASH_ERR; // flash is full
//    }
//
//    for (uint32_t i = 0; i < FLASH_COUNTER_MAX_WRITES; i++) {
//        uint64_t temp;
//        MCU_FLASH_read(write_address + (i * FLASH_COUNTER_SIZE), &temp, sizeof(temp));
//        if (temp == 0xFFFFFFFFFFFFFFFF) { // unused flash
//            if (i == 0)
//                counter = 1;
//            else {
//                MCU_FLASH_read(write_address + ((i - 1) * FLASH_COUNTER_SIZE), &counter, sizeof(counter));
//                counter++;
//            }
//
//            return MCU_FLASH_write(write_address + (i * FLASH_COUNTER_SIZE), &counter, sizeof(counter));
//        }
//    }
//    return KNS_STATUS_FLASH_ERR;
//}
//
//enum KNS_status_t MCU_FLASH_get_latest_counter(uint64_t *counter)
//{
//    if (!counter) return KNS_STATUS_FLASH_ERR;
//
//    uint32_t write_address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER;
//    uint64_t temp = 0;
//
//    for (uint32_t i = 0; i < FLASH_COUNTER_MAX_WRITES; i++) {
//        MCU_FLASH_read(write_address + (i * FLASH_COUNTER_SIZE), &temp, sizeof(temp));
//        if (temp == 0xFFFFFFFFFFFFFFFF) {
//            if (i == 0) *counter = 0;
//            else MCU_FLASH_read(write_address + ((i - 1) * FLASH_COUNTER_SIZE), counter, sizeof(*counter));
//            return KNS_STATUS_OK;
//        }
//    }
//
//    MCU_FLASH_read(write_address + ((FLASH_COUNTER_MAX_WRITES - 1) * FLASH_COUNTER_SIZE), counter, sizeof(*counter));
//    return KNS_STATUS_OK;
//}
//
//bool MCU_FLASH_is_counter_full(void)
//{
//    uint64_t temp;
//    uint32_t last_address = FLASH_USER_START_ADDR + FLASH_WKU_OCUNTER +
//                            ((FLASH_COUNTER_MAX_WRITES - 1) * FLASH_COUNTER_SIZE);
//    MCU_FLASH_read(last_address, &temp, sizeof(temp));
//    return (temp != 0xFFFFFFFFFFFFFFFF);
//}
//
//enum KNS_status_t MCU_FLASH_reset_counter(void)
//{
//    uint64_t page_backup[FLASH_PAGE_SIZE / sizeof(uint64_t)];
//    memcpy(page_backup, (uint64_t*)FLASH_USER_START_ADDR, FLASH_PAGE_SIZE);
//
//    memset(((uint8_t*)page_backup) + FLASH_WKU_OCUNTER, 0xFF,
//           (FLASH_COUNTER_MAX_WRITES * FLASH_COUNTER_SIZE));
//
//    HAL_FLASH_Unlock();
//
//    FLASH_EraseInitTypeDef EraseInitStruct;
//    uint32_t PageError;
//    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//    EraseInitStruct.Page = FLASH_USER_NMB_PAGE;
//    EraseInitStruct.NbPages = 1;
//
//    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
//        HAL_FLASH_Lock();
//        return KNS_STATUS_FLASH_ERR;
//    }
//
//    for (uint32_t i = 0; i < FLASH_PAGE_SIZE / sizeof(uint64_t); i++) {
//        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
//                              FLASH_USER_START_ADDR + (i * sizeof(uint64_t)),
//                              page_backup[i]) != HAL_OK) {
//            HAL_FLASH_Lock();
//            return KNS_STATUS_FLASH_ERR;
//        }
//    }
//
//    HAL_FLASH_Lock();
//    return KNS_STATUS_OK;
//}
