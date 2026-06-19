/**
 * @file    mgr_rate.c
 * @brief   Sliding-window TX rate limiter (see mgr_rate.h)
 */

#include "mgr_rate.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"
#include <string.h>

#define MGR_RATE_RETAIN_MAGIC 0x52415445UL /* "RATE" */

/* All persistent state lives here. Placed in SRAM2 retention so it survives
 * resets. CRC32 detects garbage / first power-on. */
static __attribute__((__section__(".retentionRamBss")))
struct {
	uint32_t magic;
	uint32_t timestamps[MGR_RATE_MAX_CAP];
	uint16_t head;     /**< Next write index (mod capacity) */
	uint16_t count;    /**< Number of valid entries (0..MAX_CAP) */
	uint32_t mono_base_s; /**< Limiter clock snapshot at last commit */
	uint32_t crc32;
} ring;

/* Config is RAM-only — NVM gather/apply does the persistence. */
static uint32_t cfg_window_s = MGR_RATE_DEFAULT_WINDOW_S;
static uint16_t cfg_max_tx   = MGR_RATE_DEFAULT_MAX_TX;

/* Limiter clock: TRUE monotonic seconds built by accumulating HAL_GetTick
 * deltas (wrap-safe uint32 subtraction). The naive (tick/1000) form broke
 * modular arithmetic at the 49.7-day tick wrap: ring entries then looked
 * "future-dated" and were never trimmed, freezing the quota for ~49 days.
 * mono_s wraps at 2^32 s ≈ 136 years instead.
 * The clock resumes from ring.mono_base_s after a soft reset (elapsed time
 * across the reset is lost → entries look slightly younger → conservative). */
static uint32_t mono_s       = 0;
static uint32_t mono_last_ms = 0;
static uint32_t mono_rem_ms  = 0;

/* ---- CRC32 (MPEG-2, polynomial 0x04C11DB7) — same routine as MGR_NVM ---- */
static uint32_t rate_crc32(const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t crc = 0xFFFFFFFFUL;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)p[i] << 24;
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80000000UL)
				crc = (crc << 1) ^ 0x04C11DB7UL;
			else
				crc <<= 1;
		}
	}
	return crc;
}

static uint32_t ring_crc(void)
{
	/* CRC over magic + timestamps + head + count (everything but crc32). */
	return rate_crc32(&ring, offsetof(__typeof__(ring), crc32));
}

static void ring_commit(void)
{
	ring.magic = MGR_RATE_RETAIN_MAGIC;
	ring.mono_base_s = mono_s;
	ring.crc32 = ring_crc();
}

static bool ring_valid(void)
{
	return ring.magic == MGR_RATE_RETAIN_MAGIC &&
	       ring.head < MGR_RATE_MAX_CAP &&
	       ring.count <= MGR_RATE_MAX_CAP &&
	       ring.crc32 == ring_crc();
}

uint32_t MGR_RATE_nowS(void)
{
	/* Advance the monotonic clock by the wrap-safe tick delta. Requires one
	 * call at least every 49.7 days; every TX decision path calls it, and a
	 * gap that long can only under-count elapsed time (conservative for a
	 * quota limiter). */
	const uint32_t now_ms = HAL_GetTick();
	const uint32_t delta  = now_ms - mono_last_ms;

	mono_last_ms = now_ms;
	mono_rem_ms += delta % 1000u;
	mono_s      += delta / 1000u + mono_rem_ms / 1000u;
	mono_rem_ms %= 1000u;
	return mono_s;
}

void MGR_RATE_init(void)
{
	mono_last_ms = HAL_GetTick();
	mono_rem_ms  = 0;

	if (!ring_valid()) {
		MGR_LOG_WARN("[RATE] Retention buffer invalid (magic=0x%08lx) — reinit\r\n",
			(unsigned long)ring.magic);
		mono_s = 0;
		memset(&ring, 0, sizeof(ring));
		ring_commit();  /* sets magic + mono_base_s + crc32 */
	} else {
		/* Resume the limiter clock from the last committed snapshot so
		 * retained timestamps stay comparable across soft resets. */
		mono_s = ring.mono_base_s;
		MGR_LOG_DEBUG("[RATE] Retention OK, count=%u head=%u base=%lus\r\n",
			ring.count, ring.head, (unsigned long)mono_s);
	}
}

/* Oldest valid timestamp (entry at (head - count) mod capacity). Caller must
 * ensure ring.count > 0. */
static uint32_t ring_oldest(void)
{
	uint16_t idx = (uint16_t)((ring.head + MGR_RATE_MAX_CAP -
		ring.count) % MGR_RATE_MAX_CAP);
	return ring.timestamps[idx];
}

/* Remove timestamps older than (now - window). After the call, the ring
 * contains only entries that still count against the budget. */
static void trim_window(uint32_t now_s, uint32_t window_s)
{
	while (ring.count > 0) {
		uint32_t oldest = ring_oldest();

		/* Use unsigned diff so future-dated entries (clock skew on restart)
		 * are NOT dropped — they look "newer than now". */
		if (now_s >= oldest && (now_s - oldest) >= window_s) {
			ring.count--;
		} else {
			break;
		}
	}
}

void MGR_RATE_recordTx(void)
{
	uint32_t now_s = MGR_RATE_nowS();
	trim_window(now_s, cfg_window_s);

	ring.timestamps[ring.head] = now_s;
	ring.head = (uint16_t)((ring.head + 1u) % MGR_RATE_MAX_CAP);
	if (ring.count < MGR_RATE_MAX_CAP)
		ring.count++;
	ring_commit();

	MGR_LOG_DEBUG("[RATE] +1 → %u/%u in %lus\r\n",
		ring.count, cfg_max_tx, (unsigned long)cfg_window_s);
}

bool MGR_RATE_isBlocked(uint32_t *retry_in_s)
{
	uint32_t now_s = MGR_RATE_nowS();
	trim_window(now_s, cfg_window_s);
	if (retry_in_s)
		*retry_in_s = 0;

	if (ring.count < cfg_max_tx)
		return false;

	if (retry_in_s) {
		uint32_t oldest = ring_oldest();
		uint32_t elapsed = (now_s >= oldest) ? (now_s - oldest) : 0;
		*retry_in_s = (elapsed >= cfg_window_s) ? 0
			: (cfg_window_s - elapsed);
	}
	return true;
}

void MGR_RATE_setConfig(uint32_t window_s, uint16_t max_tx)
{
	if (window_s < MGR_RATE_MIN_WINDOW_S) window_s = MGR_RATE_MIN_WINDOW_S;
	if (window_s > MGR_RATE_MAX_WINDOW_S) window_s = MGR_RATE_MAX_WINDOW_S;
	if (max_tx   < MGR_RATE_MIN_MAX_TX)   max_tx   = MGR_RATE_MIN_MAX_TX;
	if (max_tx   > MGR_RATE_MAX_MAX_TX)   max_tx   = MGR_RATE_MAX_MAX_TX;

	cfg_window_s = window_s;
	cfg_max_tx   = max_tx;
	MGR_LOG_INFO("[RATE] Config: %u TX / %lus\r\n",
		cfg_max_tx, (unsigned long)cfg_window_s);
}

void MGR_RATE_getConfig(uint32_t *window_s, uint16_t *max_tx,
                        uint16_t *current_count)
{
	if (window_s)      *window_s      = cfg_window_s;
	if (max_tx)        *max_tx        = cfg_max_tx;
	if (current_count) {
		trim_window(MGR_RATE_nowS(), cfg_window_s);
		*current_count = ring.count;
	}
}

void MGR_RATE_clear(void)
{
	ring.head = 0;
	ring.count = 0;
	ring_commit();
	MGR_LOG_INFO("[RATE] Ring cleared\r\n");
}
