/**
 * @file    mgr_txstats.c
 * @brief   Persistent TX statistics (see mgr_txstats.h)
 */

#include "mgr_txstats.h"
#include "mgr_log.h"
#include <string.h>
#include <stddef.h>

#define MGR_TXSTATS_MAGIC  0x54585354UL  /* "TXST" */

/* All persistent state in .retentionRamBss: survives IWDG and software
 * resets. BOR/NRST(PIN)/OBL re-run Sram2_Init which zeroes this section;
 * magic + CRC32 then detect the wipe and re-init (first-power-on / corruption
 * take the same path). NOT brownout-persistent. */
static __attribute__((__section__(".retentionRamBss")))
struct {
	uint32_t      magic;
	MGR_TXSTATS_t s;
	uint32_t      crc32;
} stats;

/* CRC-32/MPEG-2, same routine as MGR_NVM / MGR_RATE for consistency. */
static uint32_t txs_crc32(const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t crc = 0xFFFFFFFFUL;
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)p[i] << 24;
		for (int b = 0; b < 8; b++)
			crc = (crc & 0x80000000UL) ? (crc << 1) ^ 0x04C11DB7UL : (crc << 1);
	}
	return crc;
}

static uint32_t stats_crc(void)
{
	return txs_crc32(&stats, offsetof(__typeof__(stats), crc32));
}

static void stats_commit(void)
{
	stats.magic = MGR_TXSTATS_MAGIC;
	stats.crc32 = stats_crc();
}

static bool stats_valid(void)
{
	return stats.magic == MGR_TXSTATS_MAGIC && stats.crc32 == stats_crc();
}

void MGR_TXSTATS_init(void)
{
	if (!stats_valid()) {
		MGR_LOG_WARN("[TXSTATS] Retention invalid — zeroing\r\n");
		memset(&stats, 0, sizeof(stats));
		stats_commit();
	} else {
		MGR_LOG_INFO("[TXSTATS] att=%lu done=%lu to=%lu err=%lu streak=%lu/%lu\r\n",
			(unsigned long)stats.s.attempts,
			(unsigned long)stats.s.done,
			(unsigned long)stats.s.timeouts,
			(unsigned long)stats.s.errors,
			(unsigned long)stats.s.consecutive_fail,
			(unsigned long)stats.s.worst_consecutive);
	}
}

static void inc_saturating(uint32_t *p)
{
	if (*p < 0xFFFFFFFFUL) (*p)++;
}

void MGR_TXSTATS_recordAttempt(void)
{
	inc_saturating(&stats.s.attempts);
	stats_commit();
}

void MGR_TXSTATS_recordDone(void)
{
	inc_saturating(&stats.s.done);
	stats.s.consecutive_fail = 0;
	stats_commit();
}

void MGR_TXSTATS_recordTimeout(void)
{
	inc_saturating(&stats.s.timeouts);
	inc_saturating(&stats.s.consecutive_fail);
	if (stats.s.consecutive_fail > stats.s.worst_consecutive)
		stats.s.worst_consecutive = stats.s.consecutive_fail;
	stats_commit();
}

void MGR_TXSTATS_recordError(void)
{
	inc_saturating(&stats.s.errors);
	inc_saturating(&stats.s.consecutive_fail);
	if (stats.s.consecutive_fail > stats.s.worst_consecutive)
		stats.s.worst_consecutive = stats.s.consecutive_fail;
	stats_commit();
}

void MGR_TXSTATS_get(MGR_TXSTATS_t *out)
{
	if (!out) return;
	*out = stats.s;
}

void MGR_TXSTATS_clear(void)
{
	memset(&stats.s, 0, sizeof(stats.s));
	stats_commit();
	MGR_LOG_INFO("[TXSTATS] Cleared\r\n");
}
