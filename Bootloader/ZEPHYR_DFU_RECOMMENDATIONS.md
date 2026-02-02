# Recommendations for Zephyr SPI DFU Master Implementation

This document provides guidelines for implementing the SPI DFU master in Zephyr to communicate with the STM32WL bootloader.

> **See also:** `Docs/ZEPHYR_MASTER_UNIFIED_PROTOCOL.md` for a unified reference covering both bootloader and application protocols with aligned status codes.

## 1. Protocol Overview

The bootloader supports two protocols:
- **Legacy Protocol**: Direct command byte (backward compatible)
- **A+ Protocol**: Framed with magic byte, sequence, CRC-8 (recommended)

### A+ Protocol Frame Format

```
Request (Master → Slave):
┌──────┬─────┬─────┬─────┬────────────┬───────┐
│ 0xAA │ SEQ │ CMD │ LEN │ DATA[0..n] │ CRC8  │
└──────┴─────┴─────┴─────┴────────────┴───────┘
   1B     1B    1B    1B     0-250B       1B

Response (Slave → Master):
┌──────┬─────┬────────┬─────┬────────────┬───────┐
│ 0x55 │ SEQ │ STATUS │ LEN │ DATA[0..n] │ CRC8  │
└──────┴─────┴────────┴─────┴────────────┴───────┘
   1B     1B     1B      1B     0-250B       1B
```

### CRC-8 Calculation

```c
uint8_t crc8_ccitt(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;  /* Initial value */

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  /* Polynomial 0x07 */
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

CRC is computed over: `MAGIC + SEQ + CMD/STATUS + LEN + DATA` (excludes CRC byte itself).

---

## 2. Command Reference with Timing

### DFU Commands (0x30-0x3F)

| Code | Command | Payload TX | Payload RX | Flash Op | Min Delay |
|------|---------|------------|------------|----------|-----------|
| 0x30 | PING | 0 | ~10B | No | 15ms |
| 0x31 | GET_INFO | 0 | 17B | No | 15ms |
| 0x32 | ERASE | 0 | 1B | **~2s** | **3000ms** |
| 0x33 | WRITE_REQ | 6B | 1B | No | 15ms |
| 0x34 | WRITE_DATA | ≤256B | 1B | ~6ms | 20ms |
| 0x35 | READ_REQ | 6B | 1B | No | 15ms |
| 0x36 | READ_DATA | 0 | ≤256B | No | 15ms |
| 0x37 | VERIFY | 4B | 1B | No | 15ms |
| 0x38 | RESET | 0 | 1B | No | 100ms |
| 0x39 | JUMP | 0 | 1B | No | 100ms |
| 0x3A | GET_STATUS | 0 | 32B | No | 15ms |
| 0x3B | ABORT | 0 | 1B | No | 15ms |
| 0x3C | SET_HEADER | 256B | 1B | ~6ms | 20ms |

---

## 3. Zephyr Implementation Recommendations

### 3.1 SPI Configuration

```c
/* Recommended SPI settings */
#define DFU_SPI_FREQUENCY   1000000  /* 1 MHz - safe for DMA slave */
#define DFU_SPI_MODE        SPI_MODE_CPHA  /* Mode 0: CPOL=0, CPHA=0 */
#define DFU_SPI_WORD_SIZE   8

static const struct spi_config spi_cfg = {
    .frequency = DFU_SPI_FREQUENCY,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
                 SPI_WORD_SET(DFU_SPI_WORD_SIZE),
    .slave = 0,
    .cs = {
        .gpio = DFU_CS_GPIO_SPEC,
        .delay = 10,  /* 10us CS delay */
    },
};
```

### 3.2 Transaction Model

The bootloader is a SPI slave using DMA. The response to Transaction N is received during Transaction N+1.

```
Transaction 1: Master sends CMD_A
               Master receives: [idle pattern 0xAA...]

Transaction 2: Master sends CMD_B (or NOP)
               Master receives: Response to CMD_A

Transaction 3: Master sends CMD_C
               Master receives: Response to CMD_B
```

### 3.3 Timing with k_sleep()

```c
/* Timing constants (milliseconds) */
#define DFU_TIMING_DEFAULT_MS       15
#define DFU_TIMING_FLASH_WRITE_MS   20
#define DFU_TIMING_ERASE_MS         3000  /* CRITICAL! */
#define DFU_TIMING_RESET_MS         100

int dfu_send_command(uint8_t cmd, const uint8_t *data, uint16_t len,
                     uint8_t *response, uint16_t *resp_len, uint32_t delay_ms)
{
    /* Build A+ frame */
    uint8_t tx_buf[260];
    uint16_t tx_len = build_aplus_frame(tx_buf, cmd, data, len);

    /* Transaction 1: Send command */
    struct spi_buf tx = { .buf = tx_buf, .len = tx_len };
    struct spi_buf_set tx_bufs = { .buffers = &tx, .count = 1 };

    uint8_t rx_buf[260];
    struct spi_buf rx = { .buf = rx_buf, .len = tx_len };
    struct spi_buf_set rx_bufs = { .buffers = &rx, .count = 1 };

    spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);

    /* Wait for slave to process */
    k_sleep(K_MSEC(delay_ms));

    /* Transaction 2: Get response */
    memset(tx_buf, 0xAA, sizeof(tx_buf));  /* Idle pattern */
    tx.len = 64;  /* Fixed size for response */
    rx.len = 64;

    spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);

    /* Parse response */
    return parse_aplus_response(rx_buf, response, resp_len);
}
```

### 3.4 Handling Long Operations (ERASE)

**Option A: Fixed Delay (Simple)**
```c
int dfu_erase(void)
{
    uint8_t resp[16];
    uint16_t resp_len;

    /* ERASE command - slave blocks for ~2 seconds */
    return dfu_send_command(0x32, NULL, 0, resp, &resp_len,
                            DFU_TIMING_ERASE_MS);
}
```

**Option B: Polling with GET_STATUS (Recommended)**
```c
int dfu_erase_with_polling(void)
{
    uint8_t resp[64];
    uint16_t resp_len;
    int ret;

    /* Send ERASE command */
    ret = dfu_send_command(0x32, NULL, 0, resp, &resp_len, 100);
    if (ret != 0) return ret;

    /* Poll GET_STATUS until READY or timeout */
    uint32_t start = k_uptime_get_32();
    while ((k_uptime_get_32() - start) < DFU_TIMING_ERASE_MS) {
        k_sleep(K_MSEC(100));  /* Poll every 100ms */

        ret = dfu_send_command(0x3A, NULL, 0, resp, &resp_len, 15);
        if (ret != 0) continue;

        bl_extended_status_t *status = (bl_extended_status_t*)resp;

        if (status->dfu_op_state == BL_DFU_STATE_READY) {
            return 0;  /* Erase complete */
        }
        if (status->dfu_op_state == BL_DFU_STATE_ERROR) {
            return -EIO;  /* Erase failed */
        }
    }

    return -ETIMEDOUT;
}
```

### 3.5 Extended Status Structure

```c
typedef struct __attribute__((packed)) {
    uint8_t protocol_version;   /* 0x01 for A+ */
    uint8_t bootloader_state;   /* bl_state_t */
    uint8_t dfu_op_state;       /* Current operation state */
    uint8_t last_error;         /* Last error code */
    uint8_t session_active;     /* DFU session flag */
    uint8_t erase_done;         /* Erase completed */
    uint8_t verify_passed;      /* CRC verified */
    uint8_t reserved;
    uint32_t received_bytes;    /* Total bytes received */
    uint32_t write_address;     /* Current write address */
    uint32_t expected_crc;      /* Expected CRC */
    uint32_t calculated_crc;    /* Calculated CRC */
    uint32_t frame_count;       /* Frames processed */
    uint32_t crc_error_count;   /* CRC errors */
} bl_extended_status_t;

/* DFU Operation States */
typedef enum {
    BL_DFU_STATE_IDLE = 0,      /* No operation */
    BL_DFU_STATE_ERASING,       /* Erase in progress */
    BL_DFU_STATE_WRITING,       /* Write in progress */
    BL_DFU_STATE_VERIFYING,     /* CRC verification */
    BL_DFU_STATE_READY,         /* Ready for next op */
    BL_DFU_STATE_COMPLETE,      /* DFU complete */
    BL_DFU_STATE_ERROR,         /* Error occurred */
} bl_dfu_op_state_t;
```

---

## 4. Complete DFU Sequence Example

```c
int dfu_update_firmware(const uint8_t *fw_data, size_t fw_size)
{
    int ret;
    uint8_t resp[64];
    uint16_t resp_len;

    /* 1. PING - Verify bootloader is running */
    ret = dfu_send_command(0x30, NULL, 0, resp, &resp_len, 15);
    if (ret != 0 || resp[0] != 0x00) {
        LOG_ERR("PING failed");
        return -EIO;
    }
    LOG_INF("Bootloader version: %s", &resp[1]);

    /* 2. ERASE - Clear application area */
    LOG_INF("Erasing flash...");
    ret = dfu_erase_with_polling();
    if (ret != 0) {
        LOG_ERR("ERASE failed");
        return ret;
    }
    LOG_INF("Erase complete");

    /* 3. WRITE - Send firmware in chunks */
    uint32_t addr = 0x08008100;  /* APP_FLASH_BASE */
    size_t offset = 0;
    uint32_t crc = 0xFFFFFFFF;

    while (offset < fw_size) {
        size_t chunk_size = MIN(256, fw_size - offset);

        /* WRITE_REQ */
        uint8_t write_req[6];
        memcpy(write_req, &addr, 4);
        uint16_t len16 = chunk_size;
        memcpy(write_req + 4, &len16, 2);

        ret = dfu_send_command(0x33, write_req, 6, resp, &resp_len, 15);
        if (ret != 0) return ret;

        /* WRITE_DATA */
        ret = dfu_send_command(0x34, &fw_data[offset], chunk_size,
                               resp, &resp_len, 20);
        if (ret != 0 || resp[0] != 0x00) {
            LOG_ERR("WRITE failed at 0x%08X", addr);
            return -EIO;
        }

        /* Update CRC */
        crc = crc32_update(crc, &fw_data[offset], chunk_size);

        addr += chunk_size;
        offset += chunk_size;

        LOG_INF("Progress: %d%%", (offset * 100) / fw_size);
    }

    /* 4. VERIFY - Check CRC */
    uint8_t verify_data[4];
    memcpy(verify_data, &crc, 4);

    ret = dfu_send_command(0x37, verify_data, 4, resp, &resp_len, 15);
    if (ret != 0 || resp[0] != 0x00) {
        LOG_ERR("VERIFY failed: CRC mismatch");
        return -EIO;
    }
    LOG_INF("CRC verified successfully");

    /* 5. JUMP - Boot new firmware */
    ret = dfu_send_command(0x39, NULL, 0, resp, &resp_len, 100);
    if (ret != 0) {
        LOG_ERR("JUMP failed");
        return ret;
    }

    LOG_INF("DFU complete - application starting");
    return 0;
}
```

---

## 5. Error Handling

### Status Codes

| Code | Name | Description |
|------|------|-------------|
| 0x00 | OK | Success |
| 0x01 | ERROR | Generic error |
| 0x02 | CRC_ERROR | CRC mismatch (DFU data) |
| 0x03 | ADDR_ERROR | Invalid address |
| 0x04 | SIZE_ERROR | Invalid size |
| 0x05 | FLASH_ERROR | Flash operation failed |
| 0x06 | BUSY | Device busy |
| 0x07 | INVALID_CMD | Unknown command |
| 0x08 | TIMEOUT | Operation timeout |
| 0x09 | NOT_READY | Prerequisite not met |
| 0x0A | INVALID_HEADER | Bad app header |
| 0x0B | VERIFY_ERROR | Verification failed |
| 0x10 | FRAME_CRC_ERROR | Protocol CRC error |
| 0x11 | SEQ_ERROR | Sequence number error |
| 0x12 | FRAME_ERROR | Malformed frame |

### Retry Strategy

```c
#define DFU_MAX_RETRIES 3

int dfu_send_with_retry(uint8_t cmd, const uint8_t *data, uint16_t len,
                        uint8_t *resp, uint16_t *resp_len, uint32_t delay_ms)
{
    for (int i = 0; i < DFU_MAX_RETRIES; i++) {
        int ret = dfu_send_command(cmd, data, len, resp, resp_len, delay_ms);

        if (ret == 0 && resp[0] == 0x00) {
            return 0;  /* Success */
        }

        if (resp[0] == 0x10) {  /* FRAME_CRC_ERROR */
            LOG_WRN("CRC error, retrying (%d/%d)", i + 1, DFU_MAX_RETRIES);
            k_sleep(K_MSEC(50));
            continue;
        }

        /* Non-recoverable error */
        return -EIO;
    }

    return -ETIMEDOUT;
}
```

---

## 6. Protocol Detection

The bootloader auto-detects the protocol mode:

- First byte `0xAA` → A+ protocol
- First byte `0x30-0x3F` → Legacy protocol (DFU command)

**Recommendation**: Always use A+ protocol for:
- CRC protection against data corruption
- Sequence tracking for debugging
- Extended error information

---

## 7. Zephyr Workqueue for Async Operations

For non-blocking DFU operations:

```c
struct dfu_work {
    struct k_work work;
    const uint8_t *fw_data;
    size_t fw_size;
    dfu_callback_t callback;
};

static void dfu_work_handler(struct k_work *work)
{
    struct dfu_work *dfu = CONTAINER_OF(work, struct dfu_work, work);

    int ret = dfu_update_firmware(dfu->fw_data, dfu->fw_size);

    if (dfu->callback) {
        dfu->callback(ret);
    }
}

void dfu_start_async(const uint8_t *fw_data, size_t fw_size,
                     dfu_callback_t callback)
{
    static struct dfu_work dfu_work;

    dfu_work.fw_data = fw_data;
    dfu_work.fw_size = fw_size;
    dfu_work.callback = callback;

    k_work_init(&dfu_work.work, dfu_work_handler);
    k_work_submit(&dfu_work.work);
}
```

---

## 8. Summary Checklist

- [ ] Use A+ protocol with CRC-8 for reliability
- [ ] Implement proper timing delays between transactions
- [ ] Handle ERASE command with 3+ second timeout or polling
- [ ] Use GET_STATUS for async operation monitoring
- [ ] Implement retry logic for CRC errors
- [ ] Verify CRC32 before JUMP command
- [ ] Use Zephyr workqueue for non-blocking DFU
