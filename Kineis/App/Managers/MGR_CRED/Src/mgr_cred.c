/**
 * @file    mgr_cred.c
 * @brief   Credential durability — see mgr_cred.h for the WHY (mirror vs
 *          immutable page) and the backward-compatibility rationale.
 *
 * Uses HAL_FLASH_* directly for the mirror page (0x08032800, outside
 * FLASH_USER → rejected by MCU_FLASH_write's range guard) — same pattern as
 * MGR_PMLOG. Page-0 restore uses MCU_FLASH_write (page 0 IS in FLASH_USER):
 * one call writes all 48 credential bytes in a SINGLE erase+reprogram, so a
 * restore can never leave page 0 half-populated.
 *
 * HARD RULES (do not regress):
 *  - Read page-0 creds and the mirror via RAW MCU_FLASH_read only. NEVER use
 *    MCU_NVM_getID/getAddr/getRadioConfZonePtr/MCU_AES_get_device_sec_key here:
 *    those substitute TEST/default values and would break the equality
 *    invariant → spurious every-boot mirror rewrites (flash wear).
 *  - Compare the 48-byte PAYLOAD only, never the whole struct (page 0 has no
 *    magic/crc, so a full-struct compare always differs → every-boot rewrite).
 *  - Never write the mirror from a page 0 that is not fully provisioned: a
 *    partially-blank page would otherwise poison a good mirror.
 */

#include "mgr_cred.h"
#include "mgr_log.h"
#include "stm32wlxx_hal.h"
#include <string.h>

/* Software CRC32 (poly 0xEDB88320, reflected, init/xorout 0xFFFFFFFF). Table-
 * free; chosen over the HW CRC peripheral so it runs this early, needs no
 * shared peripheral, and is host-unit-testable. */
uint32_t MGR_CRED_crc32(const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	uint32_t crc = 0xFFFFFFFFu;
	for (size_t i = 0; i < len; i++) {
		crc ^= p[i];
		for (int b = 0; b < 8; b++)
			crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
	}
	return crc ^ 0xFFFFFFFFu;
}

static bool all_ff(const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++)
		if (p[i] != 0xFFu)
			return false;
	return true;
}

/* Load + validate the mirror. Strictly stronger than PMLOG's magic-only gate:
 * a torn write (magic programmed last) or any bit-rot fails the CRC. */
static bool mirror_load_valid(MGR_CRED_Mirror_t *m)
{
	MCU_FLASH_read(MGR_CRED_MIRROR_ADDR, m, sizeof(*m));
	return m->magic == MGR_CRED_MAGIC &&
	       m->version == MGR_CRED_VERSION &&
	       MGR_CRED_crc32(m, 56) == m->crc32;
}

static bool mirror_erase(void)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef e = {0};
	uint32_t perr = 0;
	e.TypeErase = FLASH_TYPEERASE_PAGES;
	e.Page      = MGR_CRED_MIRROR_PAGE;
	e.NbPages   = 1;
	HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &perr);
	HAL_FLASH_Lock();
	if (st != HAL_OK)
		MGR_LOG_ERR("[CRED] mirror erase failed (st=%d)\r\n", (int)st);
	return st == HAL_OK;
}

/* Program the 8 double-words. Creds (dw0..5) first, crc (dw7) next, magic+
 * version (dw6) LAST: a torn write leaves magic erased → mirror reads invalid. */
static bool mirror_program(const MGR_CRED_Mirror_t *m)
{
	if (!mirror_erase())
		return false;

	uint64_t dw[8];
	memcpy(dw, m, sizeof(dw));   /* copy out — struct is packed, avoid unaligned 64-bit loads */

	HAL_FLASH_Unlock();
	__disable_irq();
	HAL_StatusTypeDef st = HAL_OK;
	for (int i = 0; i < 6 && st == HAL_OK; i++)   /* dw0..5: creds */
		st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
		                       MGR_CRED_MIRROR_ADDR + (uint32_t)i * 8u, dw[i]);
	if (st == HAL_OK)                             /* dw7: crc */
		st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
		                       MGR_CRED_MIRROR_ADDR + 7u * 8u, dw[7]);
	if (st == HAL_OK)                             /* dw6: magic+version LAST */
		st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
		                       MGR_CRED_MIRROR_ADDR + 6u * 8u, dw[6]);
	__DSB();
	__ISB();
	__enable_irq();
	HAL_FLASH_Lock();
	if (st != HAL_OK)
		MGR_LOG_ERR("[CRED] mirror program failed (st=%d)\r\n", (int)st);
	return st == HAL_OK;
}

static void mirror_build(MGR_CRED_Mirror_t *m, const uint8_t payload[MGR_CRED_PAYLOAD_BYTES])
{
	memset(m, 0, sizeof(*m));
	memcpy(m, payload, MGR_CRED_PAYLOAD_BYTES);   /* id+addr+seckey+radioconf */
	m->magic   = MGR_CRED_MAGIC;
	m->version = MGR_CRED_VERSION;
	m->crc32   = MGR_CRED_crc32(m, 56);
}

/* True when ID, ADDR and SECKEY are all present. RADIOCONF is excluded — it
 * has a compiled-in static fallback (mcu_nvm.c) and a blank RADIOCONF is a
 * legitimate provisioned state, so it must not gate restore/fail-loud. */
static bool page0_fully_provisioned(const uint8_t p0[MGR_CRED_PAYLOAD_BYTES])
{
	return !all_ff(p0 + FLASH_ID_OFFSET,     FLASH_ID_BYTE_SIZE) &&
	       !all_ff(p0 + FLASH_ADDR_OFFSET,   FLASH_ADDR_BYTE_SIZE) &&
	       !all_ff(p0 + FLASH_SECKEY_OFFSET, FLASH_SECKEY_BYTE_SIZE);
}

MGR_CRED_Result_t MGR_CRED_syncAndRestore(void)
{
	uint8_t p0[MGR_CRED_PAYLOAD_BYTES];
	MCU_FLASH_read(FLASH_USER_START_ADDR + FLASH_ID_OFFSET, p0, sizeof(p0));

	MGR_CRED_Mirror_t m;
	const bool mirror_valid = mirror_load_valid(&m);

	if (page0_fully_provisioned(p0)) {
		/* Steady state: only write the mirror when the credentials actually
		 * changed (payload-only compare) → no per-boot flash wear. */
		if (mirror_valid && memcmp(p0, &m, MGR_CRED_PAYLOAD_BYTES) == 0)
			return MGR_CRED_OK_NOOP;

		MGR_CRED_Mirror_t nm;
		mirror_build(&nm, p0);
		if (mirror_program(&nm)) {
			MGR_LOG_INFO("[CRED] mirror updated from page 0\r\n");
			return MGR_CRED_MIRROR_WRITTEN;
		}
		/* Mirror write failed but page 0 is intact — retry next boot. */
		return MGR_CRED_OK_NOOP;
	}

	/* page 0 blank or partially blank (some cred field 0xFF). NEVER write the
	 * mirror from here — that would clobber a good mirror with a torn page. */
	if (mirror_valid) {
		/* Atomic restore: one MCU_FLASH_write rewrites all 48 contiguous
		 * credential bytes in a single page-0 erase+reprogram. A brownout
		 * here re-blanks page 0 (recoverable next boot); it can never leave a
		 * half-populated page that the gate above would mis-handle.
		 *
		 * We restore ONLY the credentials, not the page-0 wear-counter
		 * overflow words (MSG @56 / WKU @64) that the same brownout blanked.
		 * That is safe by design: the on-air MC is 9-bit (getMC & 0x1FF) and
		 * its value is (overflow*1024 + WL_index) & 0x1FF. Since 1024 = 2*512,
		 * overflow*1024 contributes nothing to the low 9 bits, so the on-air
		 * residue equals WL_index & 0x1FF — and the WL slot pages survive a
		 * page-0-only brownout. Losing the overflow count therefore CANNOT
		 * replay or skip an on-air MC (the next-boot sequence is byte-identical
		 * to the un-brownout one); only the internal 64-bit high-water restarts
		 * its overflow tally, which nothing on the air path consumes. */
		if (MCU_FLASH_write(FLASH_USER_START_ADDR + FLASH_ID_OFFSET, &m,
		                    MGR_CRED_PAYLOAD_BYTES) != KNS_STATUS_OK) {
			MGR_LOG_ERR("[CRED] page-0 restore write failed\r\n");
			return MGR_CRED_FAIL_LOUD;
		}
		/* Re-read + re-validate: a torn/failed restore must fail loud, not
		 * pass as healed. */
		MCU_FLASH_read(FLASH_USER_START_ADDR + FLASH_ID_OFFSET, p0, sizeof(p0));
		if (page0_fully_provisioned(p0)) {
			MGR_LOG_WARN("[CRED] page-0 credentials restored from mirror\r\n");
			return MGR_CRED_RESTORED;
		}
		return MGR_CRED_FAIL_LOUD;
	}

	/* No log here: syncAndRestore is polled every second from the CRED_FAIL
	 * wait state, so logging FAIL_LOUD here would spam. The caller logs the
	 * blank-creds condition ONCE on the INIT_MAC -> CRED_FAIL transition. */
	return MGR_CRED_FAIL_LOUD;
}
