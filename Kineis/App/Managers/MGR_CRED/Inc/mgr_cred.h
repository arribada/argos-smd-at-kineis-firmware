/**
 * @file    mgr_cred.h
 * @brief   Credential durability: CRC-protected mirror + boot-time restore.
 *
 * WHY A MIRROR INSTEAD OF MOVING CREDENTIALS TO THEIR OWN IMMUTABLE PAGE
 * --------------------------------------------------------------------------
 * Page 0 of FLASH_USER (0x0803B000) holds BOTH the immutable credentials
 * (ID @0 / ADDR @8 / SECKEY @16 / RADIOCONF @32, see mcu_flash.h) AND the
 * mutable wear-leveling overflow counters (MSG @56, WKU @64). MCU_FLASH_write
 * does a whole-page read-modify-write (erase + reprogram), so every counter
 * overflow transits page 0 through a blank state — a brownout there wipes the
 * credentials, after which the getters silently fall back to a TEST identity
 * (mcu_nvm.c MCU_NVM_getID/getAddr, mcu_aes.c get_device_sec_key).
 *
 * The credential OFFSETS are the load-bearing contract with the GUI/STANDALONE
 * provisioning tool (it writes by fixed offset over AT/SPI). We MUST NOT move
 * them, so page 0 keeps its mixed layout and stays 100% backward compatible
 * with already-provisioned and already-deployed units. Instead we keep a
 * CRC-protected MIRROR of the 48-byte credential block on a dedicated,
 * separately-erased page (0x08032800, outside FLASH_USER) and auto-restore
 * page 0 from it at boot. The mirror is written only when the credentials
 * change (first boot / re-provision), never on the TX hot path. Restore is a
 * SINGLE atomic page-0 RMW so it can never leave a half-populated page.
 *
 * FUTURE: when a coordinated provisioning-tool update is possible, move the
 * credentials to their OWN immutable page (never co-located with RMW counters),
 * which makes this mirror redundant. Until then the mirror is the safety net.
 */

#ifndef MGR_CRED_H
#define MGR_CRED_H

#include <stdint.h>
#include <stddef.h>
#include "mcu_flash.h"   /* FLASH_*_OFFSET / FLASH_*_BYTE_SIZE / FLASH_USER_START_ADDR */

/* Dedicated mirror page — outside FLASH_USER, reserved by the linker (the
 * STM32WL55XX_FLASH_APP.ld "Flash extension log" sibling page, formerly
 * earmarked BAT_LOG). The app ROM region ends at 0x08032000 so the linker
 * never places code/data here. */
#define MGR_CRED_MIRROR_ADDR   0x08032800UL
#define MGR_CRED_MIRROR_PAGE   101u            /* (0x08032800 - 0x08000000) / 0x800 */
#define MGR_CRED_MAGIC         0x43524544UL    /* "CRED" */
#define MGR_CRED_VERSION       1u

/* Contiguous credential payload — identical layout/offsets to page 0. */
#define MGR_CRED_PAYLOAD_BYTES (FLASH_ID_BYTE_SIZE + FLASH_ADDR_BYTE_SIZE + \
                                FLASH_SECKEY_BYTE_SIZE + FLASH_RADIOCONF_BYTE_SIZE) /* 48 */

/* 64-byte (8 double-word) mirror record. magic+version live in the 7th dword
 * and the crc in the 8th; they are programmed LAST so a torn write reads as
 * invalid (magic stays erased). crc32 covers bytes [0 .. 55]. */
typedef struct __attribute__((packed)) {
	uint8_t  id[FLASH_ID_BYTE_SIZE];                /* 8  @0  */
	uint8_t  addr[FLASH_ADDR_BYTE_SIZE];            /* 8  @8  */
	uint8_t  seckey[FLASH_SECKEY_BYTE_SIZE];        /* 16 @16 */
	uint8_t  radioconf[FLASH_RADIOCONF_BYTE_SIZE];  /* 16 @32 */
	uint32_t magic;                                 /* @48 — MGR_CRED_MAGIC      */
	uint16_t version;                               /* @52 — MGR_CRED_VERSION    */
	uint16_t reserved;                              /* @54 — 0                   */
	uint32_t crc32;                                 /* @56 — over bytes [0..55]  */
	uint32_t crc_pad;                               /* @60 — 0, keeps total /8   */
} MGR_CRED_Mirror_t;

_Static_assert(sizeof(MGR_CRED_Mirror_t) == 64,
	"mirror must be 64 bytes (8x double-word) for flash program alignment");
_Static_assert(offsetof(MGR_CRED_Mirror_t, magic) == MGR_CRED_PAYLOAD_BYTES,
	"credential payload must be the first 48 bytes, contiguous");
_Static_assert(offsetof(MGR_CRED_Mirror_t, crc32) == 56,
	"crc32 must cover exactly the first 56 bytes");

typedef enum {
	MGR_CRED_OK_NOOP = 0,    /**< page-0 valid, mirror already current — no flash write */
	MGR_CRED_MIRROR_WRITTEN, /**< page-0 valid, mirror seeded/updated from page 0       */
	MGR_CRED_RESTORED,       /**< page-0 was blank, page 0 restored from a valid mirror */
	MGR_CRED_FAIL_LOUD,      /**< page-0 creds blank AND no valid mirror — lost/unprovisioned */
} MGR_CRED_Result_t;

/**
 * @brief Reconcile page-0 credentials with the mirror. Call BEFORE any
 *        credential read / MAC init (the boot path), and again from the
 *        CRED_FAIL wait state to detect AT re-provisioning.
 *
 * - page 0 fully provisioned + mirror current   -> OK_NOOP        (no write)
 * - page 0 fully provisioned + mirror stale/none -> MIRROR_WRITTEN (seed/update mirror)
 * - page 0 blank/partial     + mirror valid      -> RESTORED       (atomic page-0 rewrite)
 * - page 0 blank/partial     + no valid mirror   -> FAIL_LOUD      (caller must fail loud)
 *
 * @return the action taken (see MGR_CRED_Result_t).
 */
MGR_CRED_Result_t MGR_CRED_syncAndRestore(void);

/** @brief Software CRC32 (poly 0xEDB88320, reflected) — exposed for unit tests. */
uint32_t MGR_CRED_crc32(const void *buf, size_t len);

#endif /* MGR_CRED_H */
