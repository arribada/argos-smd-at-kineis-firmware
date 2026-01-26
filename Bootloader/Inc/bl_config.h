/**
 * @file    bl_config.h
 * @brief   Bootloader configuration and memory layout definitions
 * @date    2025
 */

#ifndef BL_CONFIG_H
#define BL_CONFIG_H

#include <stdint.h>

/*******************************************************************************
 * BOOTLOADER VERSION
 ******************************************************************************/
#define BL_VERSION_MAJOR        1
#define BL_VERSION_MINOR        0
#define BL_VERSION_PATCH        0
#define BL_VERSION              ((BL_VERSION_MAJOR << 16) | (BL_VERSION_MINOR << 8) | BL_VERSION_PATCH)
#define BL_VERSION_STRING       "BL_V1.0.0"

/*******************************************************************************
 * FLASH MEMORY LAYOUT (256 KB Total)
 *
 * 0x08000000 +------------------+
 *            | BOOTLOADER       | 32 KB (16 pages)
 * 0x08008000 +------------------+
 *            | APP_HEADER       | 256 bytes
 * 0x08008100 +------------------+
 *            | APPLICATION      | ~183 KB
 * 0x0803B000 +------------------+
 *            | FLASH_USER       | 18 KB (Kineis config)
 * 0x0803B800 +------------------+
 *            | BL_STATE         | 2 KB (Bootloader state/flags)
 * 0x0803C000 +------------------+
 *            | WEAR_LEVELING    | ~16 KB (counters)
 * 0x08040000 +------------------+ End of Flash
 ******************************************************************************/

/* Bootloader region */
#define BL_FLASH_BASE           0x08000000UL
#define BL_FLASH_SIZE           0x8000UL        /* 32 KB */
#define BL_FLASH_END            (BL_FLASH_BASE + BL_FLASH_SIZE - 1)
#define BL_FLASH_PAGES          16              /* 32KB / 2KB per page */

/* Application header region */
#define APP_HEADER_ADDR         0x08008000UL
#define APP_HEADER_SIZE         0x100UL         /* 256 bytes */

/* Application code region */
#define APP_FLASH_BASE          0x08008100UL
#define APP_FLASH_SIZE          0x32F00UL       /* ~183 KB */
#define APP_FLASH_END           (APP_FLASH_BASE + APP_FLASH_SIZE - 1)
#define APP_MAX_SIZE            APP_FLASH_SIZE

/* Flash user data (Kineis config - shared with app) */
#define FLASH_USER_START        0x0803B000UL
#define FLASH_USER_SIZE_TOTAL   0x5000UL        /* 20 KB total */

/* Bootloader state storage */
#define BL_STATE_FLASH_ADDR     0x0803B800UL
#define BL_STATE_FLASH_SIZE     0x800UL         /* 2 KB (1 page) */

/* Flash parameters - use BL_ prefix to avoid conflict with HAL */
#define BL_FLASH_PAGE_SIZE      0x800UL         /* 2 KB per page */
#define BL_FLASH_TOTAL_SIZE     (256 * 1024UL)  /* 256 KB */

/*******************************************************************************
 * APPLICATION HEADER MAGIC AND VERSION
 ******************************************************************************/
#define APP_HEADER_MAGIC        0x4B494E45UL    /* "KINE" in ASCII */
#define APP_HEADER_VERSION      0x0001

/*******************************************************************************
 * DFU PROTOCOL CONFIGURATION
 ******************************************************************************/

/* Protocol detection */
#define BL_DETECTION_TIMEOUT_MS     3000        /* 3 seconds */
#define BL_DEFAULT_PROTOCOL         BL_PROTO_UART

/* UART configuration */
#define BL_UART_BAUDRATE            9600
#define BL_UART_WORDLENGTH          UART_WORDLENGTH_8B
#define BL_UART_STOPBITS            UART_STOPBITS_1
#define BL_UART_PARITY              UART_PARITY_NONE

/* Communication buffers */
#define BL_RX_BUFFER_SIZE           512
#define BL_TX_BUFFER_SIZE           512
#define BL_CHUNK_SIZE               256         /* Data chunk size for writes */

/* Timeouts */
#define BL_CMD_TIMEOUT_MS           5000        /* Command timeout */
#define BL_WRITE_TIMEOUT_MS         500         /* Flash write timeout */
#define BL_ERASE_TIMEOUT_MS         500         /* Flash erase timeout per page */

/* Retry counts */
#define BL_FLASH_WRITE_RETRIES      3
#define BL_COMM_RETRIES             3

/*******************************************************************************
 * DFU COMMAND IDs (Unified for UART and SPI)
 ******************************************************************************/
typedef enum {
    DFU_CMD_PING        = 0x01,
    DFU_CMD_GET_INFO    = 0x02,
    DFU_CMD_ERASE       = 0x03,
    DFU_CMD_WRITE       = 0x04,
    DFU_CMD_READ        = 0x05,
    DFU_CMD_VERIFY      = 0x06,
    DFU_CMD_RESET       = 0x07,
    DFU_CMD_JUMP        = 0x08,
    DFU_CMD_GET_STATUS  = 0x09,
    DFU_CMD_ABORT       = 0x0A,
    DFU_CMD_SET_HEADER  = 0x0B,
    DFU_CMD_ENTER       = 0x0F,     /* Enter DFU mode (from app) */
    DFU_CMD_MAX
} dfu_cmd_t;

/* SPI-specific command IDs (offset for MGR_SPI_CMD compatibility) */
#define SPI_CMD_DFU_BASE        0x30
#define SPI_CMD_DFU_PING        (SPI_CMD_DFU_BASE + 0x00)   /* 0x30 */
#define SPI_CMD_DFU_GET_INFO    (SPI_CMD_DFU_BASE + 0x01)   /* 0x31 */
#define SPI_CMD_DFU_ERASE       (SPI_CMD_DFU_BASE + 0x02)   /* 0x32 */
#define SPI_CMD_DFU_WRITE_REQ   (SPI_CMD_DFU_BASE + 0x03)   /* 0x33 */
#define SPI_CMD_DFU_WRITE_DATA  (SPI_CMD_DFU_BASE + 0x04)   /* 0x34 */
#define SPI_CMD_DFU_READ_REQ    (SPI_CMD_DFU_BASE + 0x05)   /* 0x35 */
#define SPI_CMD_DFU_READ_DATA   (SPI_CMD_DFU_BASE + 0x06)   /* 0x36 */
#define SPI_CMD_DFU_VERIFY      (SPI_CMD_DFU_BASE + 0x07)   /* 0x37 */
#define SPI_CMD_DFU_RESET       (SPI_CMD_DFU_BASE + 0x08)   /* 0x38 */
#define SPI_CMD_DFU_JUMP        (SPI_CMD_DFU_BASE + 0x09)   /* 0x39 */
#define SPI_CMD_DFU_GET_STATUS  (SPI_CMD_DFU_BASE + 0x0A)   /* 0x3A */
#define SPI_CMD_DFU_ABORT       (SPI_CMD_DFU_BASE + 0x0B)   /* 0x3B */
#define SPI_CMD_DFU_SET_HEADER  (SPI_CMD_DFU_BASE + 0x0C)   /* 0x3C */
#define SPI_CMD_DFU_ENTER       (SPI_CMD_DFU_BASE + 0x0F)   /* 0x3F */

/*******************************************************************************
 * DFU RESPONSE CODES
 ******************************************************************************/
typedef enum {
    DFU_RSP_OK              = 0x00,
    DFU_RSP_ERROR           = 0x01,
    DFU_RSP_CRC_ERROR       = 0x02,
    DFU_RSP_ADDR_ERROR      = 0x03,
    DFU_RSP_SIZE_ERROR      = 0x04,
    DFU_RSP_FLASH_ERROR     = 0x05,
    DFU_RSP_BUSY            = 0x06,
    DFU_RSP_INVALID_CMD     = 0x07,
    DFU_RSP_TIMEOUT         = 0x08,
    DFU_RSP_NOT_READY       = 0x09,
    DFU_RSP_INVALID_HEADER  = 0x0A,
    DFU_RSP_VERIFY_ERROR    = 0x0B
} dfu_response_t;

/*******************************************************************************
 * BOOTLOADER STATE MACHINE
 ******************************************************************************/
typedef enum {
    BL_STATE_INIT = 0,
    BL_STATE_CHECK_APP,
    BL_STATE_DETECT_PROTOCOL,
    BL_STATE_DFU_IDLE,
    BL_STATE_DFU_UART,
    BL_STATE_DFU_SPI,
    BL_STATE_ERASING,
    BL_STATE_RECEIVING,
    BL_STATE_WRITING,
    BL_STATE_VALIDATE,
    BL_STATE_JUMP_APP,
    BL_STATE_ERROR
} bl_state_t;

/*******************************************************************************
 * PROTOCOL TYPE
 ******************************************************************************/
typedef enum {
    BL_PROTO_NONE = 0,
    BL_PROTO_UART,
    BL_PROTO_SPI
} bl_protocol_t;

/*******************************************************************************
 * BOOTLOADER FLAGS (stored in flash at BL_STATE_FLASH_ADDR)
 ******************************************************************************/
#define BL_FLAG_MAGIC           0x424C464CUL    /* "BLFL" */
#define BL_FLAG_DFU_REQUEST     0x00000001UL    /* App requested DFU mode */
#define BL_FLAG_APP_VALID       0x00000002UL    /* App CRC validated */
#define BL_FLAG_UPDATE_PENDING  0x00000004UL    /* Update in progress */

typedef struct __attribute__((packed)) {
    uint32_t magic;             /* BL_FLAG_MAGIC */
    uint32_t flags;             /* BL_FLAG_xxx */
    uint32_t last_error;        /* Last error code */
    uint32_t update_count;      /* Number of successful updates */
    uint32_t reserved[12];      /* Reserved for future use */
} bl_state_flash_t;

#endif /* BL_CONFIG_H */
