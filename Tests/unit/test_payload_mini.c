/**
 * @file    test_payload_mini.c
 * @brief   Unit tests for the payload_format=1 minimal TX frame
 *          (linkit-v4 F.6 Doppler packet).
 *
 * Mirrors the packing in kns_app_uw_doppler.c build_tx_payload():
 *   hdr(3)=0b011 | avg_surf(5, minutes, sat 31) | batt(7) | low(1) | CRC8(8)
 * batt/low must land at the stock F.6 bit offsets (8..14 / 15) so a stock
 * decoder still reads them; the CRC8 (Argos poly 0x8380, init 0, MSB-first)
 * sits in the previously-reserved byte 2.
 */

#include "test_framework.h"

/*******************************************************************************
 * LOGIC UNDER TEST (mirrored from kns_app_uw_doppler.c)
 ******************************************************************************/

#define UW_DOPPLER_PACKET_HEADER  0x3u  /* 0b011 */
#define LDA2_FRAME_BYTES          24u
#define LDA2_DATA_BITS            184u

static void argos_pack_bits(uint8_t *buf, uint32_t *bit_pos,
                            uint32_t value, uint8_t nbits)
{
	for (int i = (int)nbits - 1; i >= 0; i--) {
		uint8_t bit = (uint8_t)((value >> i) & 1U);
		uint32_t byte_idx = *bit_pos >> 3;
		uint8_t  bit_in_byte = 7U - (uint8_t)(*bit_pos & 7U);
		buf[byte_idx] = (uint8_t)((buf[byte_idx] & ~(1U << bit_in_byte)) |
		                          (bit << bit_in_byte));
		(*bit_pos)++;
	}
}

/* Same algorithm as firmware argos_crc8 (linkit-v4 crc8.hpp). */
static uint8_t argos_crc8(const uint8_t *data, uint32_t nbits)
{
	uint16_t crc = 0;
	uint32_t remainder = (8U - (nbits % 8U)) % 8U;
	uint32_t total_bits = nbits + remainder;
	uint32_t total_bytes = total_bits / 8U;

	uint8_t buffer[LDA2_FRAME_BYTES] = {0};
	uint32_t op_pos = remainder;
	uint32_t ip_pos = 0;
	uint32_t left = nbits;
	while (left) {
		uint32_t bits = (left > 8U) ? 8U : left;
		uint8_t byte = 0;
		for (uint32_t k = 0; k < bits; k++) {
			uint32_t b_idx = (ip_pos + k) >> 3;
			uint8_t  b_off = 7U - (uint8_t)((ip_pos + k) & 7U);
			byte = (uint8_t)((byte << 1) | ((data[b_idx] >> b_off) & 1U));
		}
		for (int k = (int)bits - 1; k >= 0; k--) {
			uint32_t b_idx = op_pos >> 3;
			uint8_t  b_off = 7U - (uint8_t)(op_pos & 7U);
			uint8_t bit = (uint8_t)((byte >> k) & 1U);
			buffer[b_idx] = (uint8_t)((buffer[b_idx] & ~(1U << b_off)) |
			                          (bit << b_off));
			op_pos++;
		}
		ip_pos += bits;
		left -= bits;
	}

	for (uint32_t idx = 0; idx < total_bytes; idx++) {
		crc ^= (uint16_t)((uint16_t)buffer[idx] << 8);
		for (int i = 0; i < 8; i++) {
			if (crc & 0x8000U)
				crc ^= 0x8380U;
			crc = (uint16_t)(crc << 1);
		}
	}
	return (uint8_t)(crc >> 8);
}

/* Mirror of the format-1 packing block. Returns bitlen. */
static uint32_t build_mini(uint8_t *usrdata, uint8_t batt_encoded,
                           int is_low, uint32_t surf_min, int modulation)
{
	/* modulation: 0=VLDA4 1=LDA2 2=LDA2L 3=LDK */
	uint32_t sf = surf_min;
	uint32_t bitlen;

	if (sf > 31u)
		sf = 31u;

	memset(usrdata, 0, 25);
	uint32_t pos = 0;
	argos_pack_bits(usrdata, &pos, UW_DOPPLER_PACKET_HEADER, 3);
	argos_pack_bits(usrdata, &pos, sf, 5);
	argos_pack_bits(usrdata, &pos, batt_encoded, 7);
	argos_pack_bits(usrdata, &pos, is_low ? 1u : 0u, 1);
	usrdata[2] = argos_crc8(usrdata, 16);

	switch (modulation) {
	case 0:
		bitlen = 24;
		break;
	case 1:
		bitlen = 32;
		usrdata[3] = argos_crc8(usrdata, 24);
		break;
	case 2:
		bitlen = 196;
		usrdata[LDA2_FRAME_BYTES - 1] = argos_crc8(usrdata, LDA2_DATA_BITS);
		break;
	case 3:
	default:
		bitlen = 128;
		break;
	}
	return bitlen;
}

/*******************************************************************************
 * INDEPENDENT CRC REFERENCE (naive long-division over a bit array)
 ******************************************************************************/

/* Direct byte-at-a-time recurrence (linkit-v4 crc8.hpp spec: 16-bit
 * accumulator, XOR 0x8380 on MSB before shift, result = high byte).
 * Independent of the firmware path under test, which goes through a
 * bit-repacking stage for non-byte-aligned inputs.
 */
static uint8_t crc8_reference(const uint8_t *bytes, uint32_t nbytes)
{
	uint16_t crc = 0;
	for (uint32_t i = 0; i < nbytes; i++) {
		crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
		for (int b = 0; b < 8; b++) {
			if (crc & 0x8000U)
				crc ^= 0x8380U;
			crc = (uint16_t)(crc << 1);
		}
	}
	return (uint8_t)(crc >> 8);
}

/*******************************************************************************
 * TESTS
 ******************************************************************************/

/* 3700 mV → (3700-2700)/20 = 50 = 0b0110010 */
#define BATT_3700  50u

static void test_header_is_doppler_type3(void)
{
	uint8_t f[25];
	build_mini(f, BATT_3700, 0, 5, 0);
	ASSERT_EQ(0x3, f[0] >> 5);  /* bits 0..2 = 0b011 */
	TEST_PASS();
}

static void test_byte_layout_known_vector(void)
{
	uint8_t f[25];
	uint32_t bitlen = build_mini(f, BATT_3700, 0, 5, 0);
	/* byte0 = 011 | 00101 = 0x65 ; byte1 = 0110010 | 0 = 0x64 */
	ASSERT_EQ(24, (long)bitlen);
	ASSERT_EQ_HEX(0x65, f[0]);
	ASSERT_EQ_HEX(0x64, f[1]);
	TEST_PASS();
}

static void test_batt_low_at_stock_f6_offsets(void)
{
	uint8_t f[25];
	build_mini(f, 0x7Fu, 1, 0, 0);
	/* batt bits 8..14 all set, low bit 15 set → byte1 = 0xFF */
	ASSERT_EQ_HEX(0xFF, f[1]);
	/* surf=0 → byte0 = header only = 0b011_00000 */
	ASSERT_EQ_HEX(0x60, f[0]);
	TEST_PASS();
}

static void test_surf_saturates_at_31(void)
{
	uint8_t f31[25], f200[25];
	build_mini(f31, BATT_3700, 0, 31, 0);
	build_mini(f200, BATT_3700, 0, 200, 0);
	ASSERT_EQ_HEX(f31[0], f200[0]);
	ASSERT_EQ_HEX(0x7F, f31[0]); /* 011 11111 */
	TEST_PASS();
}

static void test_crc_matches_independent_reference(void)
{
	uint8_t f[25];
	build_mini(f, BATT_3700, 0, 5, 0);
	ASSERT_EQ_HEX(crc8_reference(f, 2), f[2]);
	build_mini(f, 0x7Fu, 1, 31, 0);
	ASSERT_EQ_HEX(crc8_reference(f, 2), f[2]);
	build_mini(f, 0, 0, 0, 0);
	ASSERT_EQ_HEX(crc8_reference(f, 2), f[2]);
	TEST_PASS();
}

static void test_crc_detects_single_bit_flip(void)
{
	uint8_t f[25];
	build_mini(f, BATT_3700, 0, 5, 0);
	for (int bit = 0; bit < 16; bit++) {
		uint8_t g[2] = { f[0], f[1] };
		g[bit / 8] ^= (uint8_t)(1u << (7 - (bit % 8)));
		if (crc8_reference(g, 2) == f[2]) {
			TEST_FAIL("bit flip not detected by CRC8");
			return;
		}
	}
	TEST_PASS();
}

static void test_lda2_envelope_crc_over_full_packet(void)
{
	uint8_t f[25];
	uint32_t bitlen = build_mini(f, BATT_3700, 0, 5, 1);
	ASSERT_EQ(32, (long)bitlen);
	/* Inner 24-bit packet identical to VLDA4 frame */
	uint8_t v[25];
	build_mini(v, BATT_3700, 0, 5, 0);
	ASSERT_MEM_EQ(v, f, 3);
	/* Envelope CRC covers bytes 0..2 (incl. inner CRC) */
	ASSERT_EQ_HEX(crc8_reference(f, 3), f[3]);
	TEST_PASS();
}

static void test_ldk_zero_pad_after_packet(void)
{
	uint8_t f[25];
	uint32_t bitlen = build_mini(f, BATT_3700, 1, 12, 3);
	ASSERT_EQ(128, (long)bitlen);
	for (int i = 3; i < 16; i++)
		ASSERT_EQ(0, f[i]);
	TEST_PASS();
}

static void test_lda2l_legacy_crc_byte23(void)
{
	uint8_t f[25];
	uint32_t bitlen = build_mini(f, BATT_3700, 0, 5, 2);
	ASSERT_EQ(196, (long)bitlen);
	ASSERT_EQ_HEX(crc8_reference(f, 23), f[23]);
	TEST_PASS();
}

static void test_batt_encoding_examples(void)
{
	/* (mV-2700)/20: 2700→0, 4240→77, ≥5240 saturates at 127 (firmware
	 * argos_convert_battery_voltage). Here just check the packed field
	 * round-trips through the frame. */
	uint8_t f[25];
	for (uint32_t enc = 0; enc <= 127; enc += 25) {
		build_mini(f, (uint8_t)enc, 0, 0, 0);
		uint32_t got = ((uint32_t)(f[1] >> 1)) & 0x7Fu;
		ASSERT_EQ(enc, got);
	}
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER minimal payload (F.6 Doppler frame)");

	RUN_TEST(test_header_is_doppler_type3);
	RUN_TEST(test_byte_layout_known_vector);
	RUN_TEST(test_batt_low_at_stock_f6_offsets);
	RUN_TEST(test_surf_saturates_at_31);
	RUN_TEST(test_crc_matches_independent_reference);
	RUN_TEST(test_crc_detects_single_bit_flip);
	RUN_TEST(test_lda2_envelope_crc_over_full_packet);
	RUN_TEST(test_ldk_zero_pad_after_packet);
	RUN_TEST(test_lda2l_legacy_crc_byte23);
	RUN_TEST(test_batt_encoding_examples);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
