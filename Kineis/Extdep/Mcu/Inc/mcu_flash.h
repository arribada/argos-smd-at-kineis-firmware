/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_flash.h
 * @brief   MCU flash memory system
 * @author  Arribada
 * @date    Creation 2022/01/31
 */

/**
 * @page mcu_flash_page MCU flash
 *
 * Flash memory is used to store ID, ADDR, Secret Key
 *
 * @note 
 *
 */

/**
 * @addtogroup MCU_FLASH
 * @brief MCU FLASH
 * @{
 */

#ifndef MCU_FLASH_H
#define MCU_FLASH_H
#include "stm32wlxx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "kns_types.h"

#define FLASH_TOTAL_SIZE          (256 * 1024UL)    // 256 KB Flash

// Definition of constant for flash user memory
//#define FLASH_USER_START_ADDR     0x0803F800
#define FLASH_USER_START_ADDR     0x0803B000UL
#define FLASH_USER_END_ADDR       FLASH_END_ADDR // End address of user flash area is same at end flash address
#define FLASH_USER_SIZE           (FLASH_USER_END_ADDR + 1UL - FLASH_USER_START_ADDR)
#define FLASH_USER_NMB_PAGE       (FLASH_USER_SIZE / FLASH_PAGE_SIZE)

//First page reserved for KNS configuration and APP configuration
// Variables to manage ID in flash
#define FLASH_ID_OFFSET            0
#define FLASH_ID_SIZE              1 //64 bits size
#define FLASH_ID_BYTE_SIZE         8
// Variables to manage ADDR in flash
#define FLASH_ADDR_OFFSET          (FLASH_ID_OFFSET + FLASH_ID_BYTE_SIZE)
#define FLASH_ADDR_SIZE            1
#define FLASH_ADDR_BYTE_SIZE       8
// Variables to manage SECRET KEY in flash
#define FLASH_SECKEY_OFFSET        (FLASH_ADDR_OFFSET + FLASH_ADDR_BYTE_SIZE)
#define FLASH_SECKEY_SIZE          2
#define FLASH_SECKEY_BYTE_SIZE     16
// Variables to manage RADIOCONF in flash
#define FLASH_RADIOCONF_OFFSET     (FLASH_SECKEY_OFFSET + FLASH_SECKEY_BYTE_SIZE)
#define FLASH_RADIOCONF_SIZE       2
#define FLASH_RADIOCONF_BYTE_SIZE  16

// Application variables offsets and sizes (between RADIOCONF and COUNTER)
#define FLASH_APP_VARS_OFFSET                     (FLASH_RADIOCONF_OFFSET + FLASH_RADIOCONF_BYTE_SIZE)
#define FLASH_U8_IS_RUNNING_OFFSET                (FLASH_APP_VARS_OFFSET)
#define FLASH_U8_NMB_MSG_OFFSET                   (FLASH_U8_IS_RUNNING_OFFSET + 1)
#define FLASH_U8_WAIT_MSG_TIMER_S_OFFSET          (FLASH_U8_NMB_MSG_OFFSET + 1)
#define FLASH_U8_WAIT_STARTUP_RESTIMER_MIN_OFFSET (FLASH_U8_WAIT_MSG_TIMER_S_OFFSET + 1)
#define FLASH_U8_WAIT_SEQUENCE_NMB_STARTUP_OFFSET (FLASH_U8_WAIT_STARTUP_RESTIMER_MIN_OFFSET + 1)
#define FLASH_U8_BAT_LEVEL_THRESHOLD_OFFSET       (FLASH_U8_WAIT_SEQUENCE_NMB_STARTUP_OFFSET + 1)

// Add padding (reserved bytes) to maintain 64-bit alignment
#define FLASH_APP_VARS_RESERVED_OFFSET        (FLASH_U8_BAT_LEVEL_THRESHOLD_OFFSET + 1)
#define FLASH_APP_VARS_RESERVED_SIZE_BYTE     2

#define FLASH_APP_VARS_BYTE_SIZE              (6 + FLASH_APP_VARS_RESERVED_SIZE_BYTE) // Total size = 8 bytes (64-bit aligned)
#define FLASH_APP_VARS_SIZE                   1
// Variables to manage MESSAGE COUNTER in flash
// MSG_COUNTER_OF = Message counter overflow of wear leveling counter
// MSG_COUNTER_WL = Message counter wear leveling counter
#define FLASH_MSG_COUNTER_OF_OFFSET     (FLASH_APP_VARS_OFFSET + FLASH_APP_VARS_BYTE_SIZE)
#define FLASH_MSG_COUNTER_OF_ADDR       (FLASH_USER_START_ADDR + FLASH_MSG_COUNTER_OF_OFFSET)
#define FLASH_MSG_COUNTER_OF_SIZE       1
#define FLASH_MSG_COUNTER_OF_BYTE_SIZE  8 // (64 bits aligned for WL counter)


// Wake up^counter with WEAR LEVELING
#define FLASH_WKU_COUNTER_OF_OFFSET           (FLASH_MSG_COUNTER_OF_OFFSET + FLASH_MSG_COUNTER_OF_BYTE_SIZE)
#define FLASH_WKU_COUNTER_OF_ADDR             (FLASH_USER_START_ADDR + FLASH_WKU_COUNTER_OF_OFFSET)
#define FLASH_WKU_COUNTER_OF_SIZE             1
#define FLASH_WKU_COUNTER_OF_BYTE_SIZE        8

// Define on specific page section
#define FLASH_MSG_COUNTER_WL_OFFSET     (FLASH_PAGE_SIZE)
#define FLASH_MSG_COUNTER_WL_SIZE       1024 // Wealeveling can count to 2048 before to increment Overflow
#define FLASH_MSG_COUNTER_WL_BYTE_SIZE  (FLASH_MSG_COUNTER_WL_SIZE * 8)
#define FLASH_MSG_COUNTER_WL_START_ADDR       (FLASH_USER_START_ADDR + FLASH_MSG_COUNTER_WL_OFFSET)


#define FLASH_WKU_COUNTER_WL_OFFSET           (FLASH_MSG_COUNTER_WL_OFFSET + FLASH_MSG_COUNTER_WL_BYTE_SIZE)
#define FLASH_WKU_COUNTER_WL_SIZE             1024 // Wear leveling can count to 2048 before to increment Overflow
#define FLASH_WKU_COUNTER_WL_BYTE_SIZE        (FLASH_WKU_COUNTER_WL_SIZE * 8) // 64-bit aligned
#define FLASH_WKU_COUNTER_WL_START_ADDR       (FLASH_USER_START_ADDR + FLASH_WKU_COUNTER_WL_OFFSET)
extern uint32_t _sflash_user_data;  // optional use

// TODO: Move it to another pertinent place?

enum KNS_status_t MCU_FLASH_read(uint32_t address, void *buffer, size_t size);
enum KNS_status_t MCU_FLASH_write(uint32_t address, const void *data, size_t size);

// enum KNS_status_t MCU_FLASH_increment_counter(void);
// enum KNS_status_t MCU_FLASH_set_counter(uint64_t *counter);
// enum KNS_status_t MCU_FLASH_get_latest_counter(uint64_t *counter);
// //bool MCU_FLASH_is_counter_full(void);
// enum KNS_status_t MCU_FLASH_reset_counter(void);

uint64_t MCU_FLASH_read_msg_counter(void);
enum KNS_status_t MCU_FLASH_increment_msg_counter(void);
enum KNS_status_t MCU_FLASH_reset_msg_counter(void);
enum KNS_status_t MCU_FLASH_set_msg_counter(uint64_t value);

uint64_t MCU_FLASH_read_wku_counter(void);
enum KNS_status_t MCU_FLASH_increment_wku_counter(void);
enum KNS_status_t MCU_FLASH_reset_wku_counter(void);
enum KNS_status_t MCU_FLASH_set_wku_counter(uint64_t value);
#endif
