/**
 * @file    mgr_pmlog.h
 * @brief   Post-mortem persistent flash log (ERROR-severity events)
 *
 * Survives every reset class including full power-off (SHUTDOWN / VBAT loss).
 * Designed for forensic analysis of tags recovered from the field with a dead
 * battery: the last N critical events stay in flash so the operator can
 * understand what happened before the tag died.
 *
 * Storage : 1 dedicated flash page at 0x08032000 (2 KB), reserved by the
 *           linker (FLASH_PMLOG region). The app region was shrunk from 204K
 *           to 200K to carve out this + a reserved sibling page for future
 *           BAT_LOG use at 0x08032800.
 *
 * Format  : append-only circular buffer of 16-byte entries (128 entries/page).
 *           Each entry stamped with magic, sequence, severity, type, state,
 *           data, and the boot tick in seconds. When the page fills up we
 *           erase it and start over from slot 0 (one page erase per 128 events
 *           → ~10 k page lifetime ≈ 1.28 M events before wear failure).
 *
 * Trigger : hooked in MGR_EVTLOG_log() — every event whose severity resolves
 *           to EVT_SEV_ERROR is mirrored here. Lower severities are NOT
 *           copied to flash (would burn the page far too fast).
 *
 * Read    : AT+PMLOG=? dumps all valid entries oldest-first. AT+PMLOG erases.
 */

#ifndef MGR_PMLOG_H
#define MGR_PMLOG_H

#include <stdint.h>
#include "mgr_evtlog.h"

/* Flash region — kept in sync with STM32WL55XX_FLASH_APP.ld FLASH_PMLOG.
 * The address must NOT collide with the app region (max 200K = 0x08032000). */
#define MGR_PMLOG_FLASH_ADDR    0x08032000UL
#define MGR_PMLOG_FLASH_SIZE    2048UL          /* 1 erase page on STM32WL55 */

#define MGR_PMLOG_ENTRY_BYTES   16u
#define MGR_PMLOG_MAX_ENTRIES   (MGR_PMLOG_FLASH_SIZE / MGR_PMLOG_ENTRY_BYTES)

/* Sentinel value for an empty (post-erase) slot. Flash erased state = all 1s. */
#define MGR_PMLOG_MAGIC_EMPTY   0xFFFFFFFFUL
/* Sentinel for a valid entry — chosen distinct from EMPTY and unlikely to
 * collide with random data after corruption. */
#define MGR_PMLOG_MAGIC_VALID   0x504D4C45UL    /* "PMLE" */

typedef struct __attribute__((packed)) {
	uint32_t magic;     /**< MAGIC_VALID, or MAGIC_EMPTY for unused slot */
	uint32_t tick_s;    /**< HAL_GetTick()/1000 at the time of logging */
	uint16_t seq;       /**< Sequence number within the page (monotonic) */
	uint8_t  severity;  /**< MGR_EVTLOG_Severity_t */
	uint8_t  type;      /**< MGR_EVTLOG_Type_t */
	uint8_t  state;     /**< UW_DOPPLER state at the time */
	uint8_t  reserved;  /**< 0 — keeps total at 16 bytes (2× double-word) */
	uint16_t data;      /**< Event-specific payload (mirror of EVT log data) */
} MGR_PMLOG_Entry_t;

_Static_assert(sizeof(MGR_PMLOG_Entry_t) == MGR_PMLOG_ENTRY_BYTES,
	"PMLOG entry must be exactly 16 bytes for flash double-word alignment");

/**
 * @brief Initialise the PM log: scan the flash page to find the next empty slot.
 *
 * Safe to call at boot. Does NOT erase the page. If the page is fully empty
 * (post-erase) or corrupted, the next call to MGR_PMLOG_log() starts at slot 0.
 */
void MGR_PMLOG_init(void);

/**
 * @brief Append one entry to the flash log.
 *
 * Intended to be called from MGR_EVTLOG_log() for ERROR-severity events only.
 * If the page is full, erases it before writing slot 0. Each erase touches the
 * page in ~25 ms which is acceptable since this only fires on critical events.
 */
void MGR_PMLOG_log(MGR_EVTLOG_Severity_t severity, uint8_t type,
                   uint8_t state, uint16_t data);

/**
 * @brief Return the number of valid entries currently in the page.
 * @return 0..MGR_PMLOG_MAX_ENTRIES
 */
uint16_t MGR_PMLOG_count(void);

/**
 * @brief Read entry by oldest-first index.
 * @param index  0..count()-1
 * @return Pointer into flash (read-only) or NULL if out of range.
 */
const MGR_PMLOG_Entry_t *MGR_PMLOG_get(uint16_t index);

/** @brief Erase the page (operator action via AT+PMLOG). */
void MGR_PMLOG_clear(void);

#endif /* MGR_PMLOG_H */
