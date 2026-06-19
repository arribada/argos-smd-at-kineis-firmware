/**
 * @file    test_cred_mirror.c
 * @brief   Regression spec for MGR_CRED credential durability (Layer 2A/2B).
 *
 * Mirrors the decision matrix + flash model of mgr_cred.c:
 *   - page-0 creds + mirror reconciled at boot via raw reads (payload-only).
 *   - first boot seeds the mirror; steady state is a NO-OP (no flash wear).
 *   - a blank/partial page-0 is restored ATOMICALLY from a CRC-valid mirror;
 *     a partial page is NEVER used to overwrite a good mirror.
 *   - blank page-0 + no valid mirror = FAIL_LOUD (never silently test creds).
 *   - mirror validity = magic && version && CRC32(first 56 bytes); a torn
 *     write leaves magic erased (programmed last) -> invalid.
 */

#include "test_framework.h"
#include <stddef.h>

#define ID_OFF       0
#define ADDR_OFF     8
#define SECKEY_OFF   16
#define RADIO_OFF    32
#define PAYLOAD      48
#define CRED_MAGIC   0x43524544u   /* "CRED" */
#define CRED_VERSION 1u

typedef struct {
	uint8_t  creds[PAYLOAD];   /* id8 addr8 seckey16 radioconf16 */
	uint32_t magic;
	uint16_t version;
	uint16_t reserved;
	uint32_t crc32;
	uint32_t crc_pad;
} mirror_t;                    /* 64 bytes */

typedef enum { R_OK_NOOP=0, R_MIRROR_WRITTEN, R_RESTORED, R_FAIL_LOUD } result_t;

/* ---- fake flash ---- */
static uint8_t  fake_p0[PAYLOAD];   /* page-0 credential region */
static mirror_t fake_mirror;        /* mirror page record */
static int erase_count, mirror_prog_count, page0_write_count;

/* ---- the exact algorithm under test (mirror of mgr_cred.c) ---- */
static uint32_t cred_crc32(const void *buf, size_t len)
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
	for (size_t i = 0; i < n; i++) if (p[i] != 0xFFu) return false;
	return true;
}

static bool page0_fully_provisioned(const uint8_t *p0)
{
	return !all_ff(p0 + ID_OFF, 8) && !all_ff(p0 + ADDR_OFF, 8) &&
	       !all_ff(p0 + SECKEY_OFF, 16);
}

static bool mirror_valid(const mirror_t *m)
{
	return m->magic == CRED_MAGIC && m->version == CRED_VERSION &&
	       cred_crc32(m, 56) == m->crc32;
}

static void mirror_build(mirror_t *m, const uint8_t *payload)
{
	memset(m, 0, sizeof(*m));
	memcpy(m->creds, payload, PAYLOAD);
	m->magic = CRED_MAGIC;
	m->version = CRED_VERSION;
	m->crc32 = cred_crc32(m, 56);
}

/* magic-last program with optional torn injection (after creds+crc, before
 * magic) -> models the firmware's torn-write residue (magic erased). */
static int tear_before_magic = 0;
static void mirror_program(const mirror_t *nm)
{
	memset(&fake_mirror, 0xFF, sizeof(fake_mirror));   /* erase */
	erase_count++;
	memcpy(fake_mirror.creds, nm->creds, PAYLOAD);     /* dw0..5 */
	fake_mirror.version = nm->version;
	fake_mirror.reserved = nm->reserved;
	fake_mirror.crc32 = nm->crc32;                     /* dw7 */
	fake_mirror.crc_pad = nm->crc_pad;
	mirror_prog_count++;
	if (tear_before_magic) return;                     /* brownout: magic stays 0xFF */
	fake_mirror.magic = nm->magic;                     /* dw6 LAST */
}

/* atomic page-0 restore: one RMW writes all 48 contiguous cred bytes */
static void page0_restore(const uint8_t *payload)
{
	memcpy(fake_p0, payload, PAYLOAD);
	page0_write_count++;
}

static result_t sync_and_restore(void)
{
	uint8_t p0[PAYLOAD];
	memcpy(p0, fake_p0, PAYLOAD);                 /* raw read */
	mirror_t m = fake_mirror;                     /* raw read */
	bool mv = mirror_valid(&m);

	if (page0_fully_provisioned(p0)) {
		if (mv && memcmp(p0, m.creds, PAYLOAD) == 0)
			return R_OK_NOOP;
		mirror_t nm;
		mirror_build(&nm, p0);
		mirror_program(&nm);
		return R_MIRROR_WRITTEN;
	}
	if (mv) {
		page0_restore(m.creds);
		memcpy(p0, fake_p0, PAYLOAD);             /* re-read + re-validate */
		return page0_fully_provisioned(p0) ? R_RESTORED : R_FAIL_LOUD;
	}
	return R_FAIL_LOUD;
}

/* ---- helpers ---- */
static void reset_counts(void) { erase_count = mirror_prog_count = page0_write_count = 0; tear_before_magic = 0; }
static void fill(uint8_t *p, int off, int n, uint8_t v) { for (int i=0;i<n;i++) p[off+i]=v; }
static void provision_p0(void) { memset(fake_p0,0xFF,PAYLOAD);
	fill(fake_p0,ID_OFF,8,0xA1); fill(fake_p0,ADDR_OFF,8,0xB2);
	fill(fake_p0,SECKEY_OFF,16,0xC3); fill(fake_p0,RADIO_OFF,16,0xD4); }
static void blank_p0(void)  { memset(fake_p0, 0xFF, PAYLOAD); }
static void blank_mirror(void) { memset(&fake_mirror, 0xFF, sizeof(fake_mirror)); }
static void seed_mirror_from_p0(void) { mirror_t nm; mirror_build(&nm, fake_p0); fake_mirror = nm; }

/* ============================ tests ============================ */

static void test_crc32_known_vector(void)
{
	ASSERT_EQ_HEX(0xCBF43926u, cred_crc32("123456789", 9));   /* CRC-32 check value */
	TEST_PASS();
}

static void test_first_boot_seeds_mirror(void)
{
	provision_p0(); blank_mirror(); reset_counts();
	ASSERT_EQ(R_MIRROR_WRITTEN, sync_and_restore());
	ASSERT_TRUE(mirror_valid(&fake_mirror));
	ASSERT_MEM_EQ(fake_p0, fake_mirror.creds, PAYLOAD);
	ASSERT_EQ(1, mirror_prog_count);
	ASSERT_EQ(0, page0_write_count);
	TEST_PASS();
}

static void test_steady_state_noop_no_write(void)
{
	provision_p0(); seed_mirror_from_p0(); reset_counts();
	ASSERT_EQ(R_OK_NOOP, sync_and_restore());
	ASSERT_EQ(0, mirror_prog_count);     /* no flash wear in steady state */
	ASSERT_EQ(0, page0_write_count);
	ASSERT_EQ(0, erase_count);
	/* repeat: still zero writes */
	ASSERT_EQ(R_OK_NOOP, sync_and_restore());
	ASSERT_EQ(0, mirror_prog_count);
	TEST_PASS();
}

static void test_brownout_restore(void)
{
	provision_p0(); seed_mirror_from_p0();
	uint8_t saved[PAYLOAD]; memcpy(saved, fake_p0, PAYLOAD);
	mirror_t saved_mirror = fake_mirror;
	blank_p0(); reset_counts();                     /* brownout blanked page 0 */
	ASSERT_EQ(R_RESTORED, sync_and_restore());
	ASSERT_MEM_EQ(saved, fake_p0, PAYLOAD);         /* identity healed */
	ASSERT_EQ(0, mirror_prog_count);                /* mirror untouched */
	ASSERT_EQ(0, memcmp(&saved_mirror, &fake_mirror, sizeof(mirror_t)));
	TEST_PASS();
}

static void test_partial_page_restores_not_clobber(void)
{
	/* page 0 with ID set but ADDR/SECKEY blank (torn restore residue) +
	 * valid mirror -> must RESTORE from mirror, never rewrite the mirror. */
	provision_p0(); seed_mirror_from_p0();
	mirror_t good_mirror = fake_mirror;
	blank_p0(); fill(fake_p0, ID_OFF, 8, 0xA1);     /* ID present, rest blank */
	reset_counts();
	ASSERT_EQ(R_RESTORED, sync_and_restore());
	ASSERT_EQ(0, mirror_prog_count);                            /* mirror NOT clobbered */
	ASSERT_EQ(0, memcmp(&good_mirror, &fake_mirror, sizeof(mirror_t)));
	ASSERT_TRUE(page0_fully_provisioned(fake_p0));
	TEST_PASS();
}

static void test_never_provisioned_fail_loud(void)
{
	blank_p0(); blank_mirror(); reset_counts();
	ASSERT_EQ(R_FAIL_LOUD, sync_and_restore());
	ASSERT_TRUE(all_ff(fake_p0, PAYLOAD));          /* NO test creds written to page 0 */
	ASSERT_EQ(0, page0_write_count);
	ASSERT_EQ(0, mirror_prog_count);
	TEST_PASS();
}

static void test_crc_reject_with_valid_page0(void)
{
	provision_p0(); seed_mirror_from_p0();
	fake_mirror.crc32 ^= 0xDEADBEEFu;               /* corrupt CRC */
	ASSERT_FALSE(mirror_valid(&fake_mirror));
	reset_counts();
	ASSERT_EQ(R_MIRROR_WRITTEN, sync_and_restore()); /* page0 good -> reseed mirror */
	ASSERT_TRUE(mirror_valid(&fake_mirror));
	TEST_PASS();
}

static void test_crc_reject_with_blank_page0(void)
{
	provision_p0(); seed_mirror_from_p0();
	fake_mirror.crc32 ^= 0x1u;                       /* corrupt CRC */
	blank_p0(); reset_counts();
	ASSERT_EQ(R_FAIL_LOUD, sync_and_restore());      /* blank + invalid mirror */
	ASSERT_EQ(0, page0_write_count);
	TEST_PASS();
}

static void test_torn_mirror_magic_erased(void)
{
	provision_p0(); reset_counts();
	tear_before_magic = 1;                            /* brownout before magic dword */
	mirror_t nm; mirror_build(&nm, fake_p0); mirror_program(&nm);
	ASSERT_EQ_HEX(0xFFFFFFFFu, fake_mirror.magic);    /* magic stays erased */
	ASSERT_FALSE(mirror_valid(&fake_mirror));         /* -> invalid, not falsely valid */
	TEST_PASS();
}

static void test_version_mismatch_invalid(void)
{
	provision_p0(); seed_mirror_from_p0();
	fake_mirror.version = 2;                           /* future version */
	fake_mirror.crc32 = cred_crc32(&fake_mirror, 56); /* even with a correct CRC... */
	ASSERT_FALSE(mirror_valid(&fake_mirror));         /* ...version gate rejects it */
	TEST_PASS();
}

static void test_reprovision_updates_mirror(void)
{
	provision_p0(); seed_mirror_from_p0(); reset_counts();
	fill(fake_p0, ADDR_OFF, 8, 0x77);                 /* operator changed ADDR */
	ASSERT_EQ(R_MIRROR_WRITTEN, sync_and_restore());
	ASSERT_MEM_EQ(fake_p0, fake_mirror.creds, PAYLOAD);
	TEST_PASS();
}

static void test_radioconf_blank_no_rewrite(void)
{
	/* id/addr/seckey set, RADIOCONF never written (0xFF) = valid provisioned
	 * state. Raw-read both sides -> equal -> OK_NOOP, zero flash wear. */
	memset(fake_p0, 0xFF, PAYLOAD);
	fill(fake_p0, ID_OFF, 8, 0xA1); fill(fake_p0, ADDR_OFF, 8, 0xB2);
	fill(fake_p0, SECKEY_OFF, 16, 0xC3);              /* RADIOCONF stays 0xFF */
	seed_mirror_from_p0(); reset_counts();
	ASSERT_EQ(R_OK_NOOP, sync_and_restore());
	ASSERT_EQ(0, mirror_prog_count);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("MGR_CRED credential durability");
	RUN_TEST(test_crc32_known_vector);
	RUN_TEST(test_first_boot_seeds_mirror);
	RUN_TEST(test_steady_state_noop_no_write);
	RUN_TEST(test_brownout_restore);
	RUN_TEST(test_partial_page_restores_not_clobber);
	RUN_TEST(test_never_provisioned_fail_loud);
	RUN_TEST(test_crc_reject_with_valid_page0);
	RUN_TEST(test_crc_reject_with_blank_page0);
	RUN_TEST(test_torn_mirror_magic_erased);
	RUN_TEST(test_version_mismatch_invalid);
	RUN_TEST(test_reprovision_updates_mirror);
	RUN_TEST(test_radioconf_blank_no_rewrite);
	TEST_SUITE_END();
	TEST_SUMMARY();
}
