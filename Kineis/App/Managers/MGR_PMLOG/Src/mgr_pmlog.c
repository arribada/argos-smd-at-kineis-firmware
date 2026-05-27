/**
 * @file    mgr_pmlog.c
 * @brief   Post-mortem persistent flash log (see mgr_pmlog.h)
 *
 * Uses HAL_FLASH_Program directly rather than MCU_FLASH_write because
 * (a) the PMLOG region is outside FLASH_USER (would be rejected by the
 * existing range check) and (b) the MCU_FLASH_write helper is read-modify-
 * write (erase + rewrite the whole page) which is the opposite of what
 * we want for an append-only log.
 */

#include "mgr_pmlog.h"
#include "mgr_log.h"
#include "stm32wlxx_hal.h"
#include <string.h>

/* In-RAM cursor: next slot index to write. Rebuilt from flash at init. */
static uint16_t s_next_slot   = 0;
static uint16_t s_valid_count = 0;
static uint16_t s_next_seq    = 0;

static const MGR_PMLOG_Entry_t *slot_at(uint16_t idx)
{
	return (const MGR_PMLOG_Entry_t *)(MGR_PMLOG_FLASH_ADDR +
		(uint32_t)idx * MGR_PMLOG_ENTRY_BYTES);
}

static bool slot_is_empty(uint16_t idx)
{
	return slot_at(idx)->magic == MGR_PMLOG_MAGIC_EMPTY;
}

/* Erase the single PMLOG page. Blocks ~25 ms — only called on init recovery
 * (corruption) or when the page is full. */
static bool pmlog_erase_page(void)
{
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef erase = {0};
	uint32_t page_err = 0;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Page      = (MGR_PMLOG_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
	erase.NbPages   = 1;

	HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_err);
	HAL_FLASH_Lock();

	if (st != HAL_OK) {
		MGR_LOG_DEBUG("[PMLOG] Erase failed (st=%d page_err=%lu)\r\n",
			(int)st, (unsigned long)page_err);
		return false;
	}
	s_next_slot = 0;
	s_valid_count = 0;
	s_next_seq = 0;
	return true;
}

void MGR_PMLOG_init(void)
{
	/* Walk slots from 0 to find the first empty one — that's our cursor.
	 * Count valid entries on the way. If we hit a slot whose magic is
	 * neither VALID nor EMPTY → corruption → erase and restart. */
	s_next_slot   = MGR_PMLOG_MAX_ENTRIES;  /* "full" until proven otherwise */
	s_valid_count = 0;
	s_next_seq    = 0;
	uint16_t max_seq = 0;
	bool corrupt = false;

	for (uint16_t i = 0; i < MGR_PMLOG_MAX_ENTRIES; i++) {
		const MGR_PMLOG_Entry_t *e = slot_at(i);
		if (e->magic == MGR_PMLOG_MAGIC_VALID) {
			s_valid_count++;
			if (e->seq >= max_seq) max_seq = e->seq;
		} else if (e->magic == MGR_PMLOG_MAGIC_EMPTY) {
			if (s_next_slot == MGR_PMLOG_MAX_ENTRIES)
				s_next_slot = i;
		} else {
			corrupt = true;
			break;
		}
	}

	if (corrupt) {
		MGR_LOG_DEBUG("[PMLOG] Corrupt page detected — erasing\r\n");
		(void)pmlog_erase_page();
		return;
	}

	if (s_valid_count > 0)
		s_next_seq = (uint16_t)(max_seq + 1u);
	if (s_next_slot == MGR_PMLOG_MAX_ENTRIES) {
		/* Page is fully populated, no empty slot — next log will wrap. */
		s_next_slot = 0;  /* will trigger an erase on next write */
	}

	MGR_LOG_DEBUG("[PMLOG] Init: %u valid entries, next_slot=%u next_seq=%u\r\n",
		s_valid_count, s_next_slot, s_next_seq);
}

/* Program two 64-bit words at `addr`. Returns true on success. */
static bool program_two_dwords(uint32_t addr, const uint64_t *dw)
{
	if ((addr & 0x7u) != 0u) return false;

	HAL_FLASH_Unlock();
	HAL_StatusTypeDef st1 = HAL_FLASH_Program(
		FLASH_TYPEPROGRAM_DOUBLEWORD, addr,     dw[0]);
	HAL_StatusTypeDef st2 = HAL_FLASH_Program(
		FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8, dw[1]);
	HAL_FLASH_Lock();

	if (st1 != HAL_OK || st2 != HAL_OK) {
		MGR_LOG_DEBUG("[PMLOG] Program failed st1=%d st2=%d\r\n",
			(int)st1, (int)st2);
		return false;
	}
	return true;
}

void MGR_PMLOG_log(MGR_EVTLOG_Severity_t severity, uint8_t type,
                   uint8_t state, uint16_t data)
{
	/* If we wrapped, erase the page first. s_next_slot==0 with the page
	 * fully populated is the wrap condition latched in init / on previous
	 * call. Detect it by checking whether slot 0 is currently valid. */
	if (s_next_slot >= MGR_PMLOG_MAX_ENTRIES || !slot_is_empty(s_next_slot)) {
		if (!pmlog_erase_page())
			return;  /* erase failed → silently skip; can't risk a corrupt write */
	}

	MGR_PMLOG_Entry_t entry;
	entry.magic     = MGR_PMLOG_MAGIC_VALID;
	entry.tick_s    = HAL_GetTick() / 1000u;
	entry.seq       = s_next_seq;
	entry.severity  = (uint8_t)severity;
	entry.type      = type;
	entry.state     = state;
	entry.reserved  = 0;
	entry.data      = data;

	/* Copy into a 64-bit-aligned local buffer for HAL_FLASH_Program. */
	uint64_t dw[2];
	memcpy(dw, &entry, sizeof(entry));

	uint32_t addr = MGR_PMLOG_FLASH_ADDR +
		(uint32_t)s_next_slot * MGR_PMLOG_ENTRY_BYTES;

	if (program_two_dwords(addr, dw)) {
		s_next_slot++;
		s_valid_count++;
		if (s_next_seq < 0xFFFFu) s_next_seq++;
	}
}

uint16_t MGR_PMLOG_count(void)
{
	return s_valid_count;
}

const MGR_PMLOG_Entry_t *MGR_PMLOG_get(uint16_t index)
{
	/* Oldest-first iteration: entries are written linearly from slot 0 up
	 * to s_next_slot - 1. After a wrap (erase + restart), the same property
	 * holds since the page is fresh. So index maps directly to slot index
	 * AMONG VALID ENTRIES. The first index valid_count entries are guaranteed
	 * contiguous from slot 0 (no holes possible in append-only flash). */
	if (index >= s_valid_count)
		return NULL;
	return slot_at(index);
}

void MGR_PMLOG_clear(void)
{
	if (pmlog_erase_page())
		MGR_LOG_DEBUG("[PMLOG] Cleared\r\n");
}
