/**
 * @file    mgr_evtlog.c
 * @brief   Event logger - circular buffer in SRAM2 retention RAM
 *
 * The buffer is placed in .retentionRamBss (SRAM2) so it survives
 * all resets except power-off. On first boot (or after power cycle),
 * the magic word will be wrong and the buffer is initialized fresh.
 *
 * Buffer layout (2056 bytes total):
 *   - Header: magic(4) + head(2) + count(2) = 8 bytes
 *   - Entries: 256 x 8 bytes = 2048 bytes
 *
 * Indexing: entry 0 = oldest, entry (count-1) = newest.
 * Oldest is at (head - count) mod 256.
 */

/**
 * @addtogroup MGR_EVTLOG
 * @{
 */

#include "mgr_evtlog.h"
#include "stm32wlxx_hal.h"

/* State exported by kns_app_uw_doppler.c for fault/error context */
extern volatile uint32_t g_uw_doppler_state_for_err;

/* Circular buffer in SRAM2 retention RAM */
typedef struct {
	uint32_t magic;
	uint16_t head;   /* Next write index (0..255) */
	uint16_t count;  /* Total entries stored (saturates at 256) */
	MGR_EVTLOG_Entry_t entries[MGR_EVTLOG_MAX_ENTRIES];
} MGR_EVTLOG_Buffer_t;

static __attribute__((__section__(".retentionRamBss")))
MGR_EVTLOG_Buffer_t evtlog_buf;

void MGR_EVTLOG_init(void)
{
	if (evtlog_buf.magic != MGR_EVTLOG_MAGIC) {
		/* First boot or power cycle - initialize */
		evtlog_buf.magic = MGR_EVTLOG_MAGIC;
		evtlog_buf.head  = 0;
		evtlog_buf.count = 0;
	}
	/* If magic is valid, buffer is preserved from previous reset */
}

void MGR_EVTLOG_log(MGR_EVTLOG_Type_t type, uint16_t data)
{
	/* Defensive: clamp head and count if SRAM corruption moved them out of bounds */
	if (evtlog_buf.head >= MGR_EVTLOG_MAX_ENTRIES)
		evtlog_buf.head = 0;
	if (evtlog_buf.count > MGR_EVTLOG_MAX_ENTRIES)
		evtlog_buf.count = MGR_EVTLOG_MAX_ENTRIES;

	MGR_EVTLOG_Entry_t *e = &evtlog_buf.entries[evtlog_buf.head];

	e->tick  = HAL_GetTick();
	e->type  = (uint8_t)type;
	e->state = (uint8_t)g_uw_doppler_state_for_err;
	e->data  = data;

	evtlog_buf.head = (evtlog_buf.head + 1) % MGR_EVTLOG_MAX_ENTRIES;
	if (evtlog_buf.count < MGR_EVTLOG_MAX_ENTRIES)
		evtlog_buf.count++;
}

uint16_t MGR_EVTLOG_count(void)
{
	return evtlog_buf.count;
}

const MGR_EVTLOG_Entry_t *MGR_EVTLOG_get(uint16_t index)
{
	if (index >= evtlog_buf.count)
		return (const MGR_EVTLOG_Entry_t *)0;

	/* Oldest entry is at (head - count) mod 256 */
	uint16_t pos = (evtlog_buf.head + MGR_EVTLOG_MAX_ENTRIES - evtlog_buf.count + index)
			% MGR_EVTLOG_MAX_ENTRIES;
	return &evtlog_buf.entries[pos];
}

void MGR_EVTLOG_clear(void)
{
	evtlog_buf.head  = 0;
	evtlog_buf.count = 0;
	/* Keep magic valid - no need to zero entries */
}

/**
 * @}
 */
