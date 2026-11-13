/*
 * Copyright (C) 2026 PortaPack Mayhem contributors
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/* Narrowband FSK frame rules of the signature detect app (class file "FSK"
 * lines, dd_sig.cpp), shared by the M0 (parser, dwell plan) and the M4
 * receiver (baseband/dd_fsk_rx.hpp): the chip pattern a rule searches for,
 * the frame length rule and the validity check. Plain data and pure
 * functions, no ChibiOS (host tested).
 *
 * Vocabulary as sigdb's [[framing.validity]] (DESIGN.md 7.5): line coding
 * nrz / manchester / manchester_inverted / differential_manchester, bit
 * order, preamble + sync as sent, length field {offset, size, counts =
 * whole_frame | after_field | payload, adjust, endian}, check {xor8 | sum8 |
 * crc8 | crc16, seed, poly, xor_out, reflect_in, reflect_out, coverage_from,
 * coverage_to, position, endian}. Byte offsets count from the first byte
 * after the sync word. */

#ifndef __DD_FSK_HPP__
#define __DD_FSK_HPP__

#include <cstddef>
#include <cstdint>

namespace dd {

constexpr size_t kFskMaxBytes = 64;     /* a frame after the sync, check included */
constexpr size_t kFskRulesPerDwell = 4; /* rules sharing one front end (same f, bw, chip rate) */
constexpr size_t kFskHdr = 16;          /* first bytes of a frame sent to the M0 */
constexpr uint32_t kFskInputRate = 4000000;

enum FskCoding : uint8_t {
    FSK_NRZ,
    FSK_MAN,     /* bit b -> chips b, !b (G.E. Thomas; Z-Wave R1) */
    FSK_MAN_INV, /* bit b -> chips !b, b (IEEE 802.3) */
    FSK_DMAN,    /* differential Manchester: a transition mid-bit always; 0 = a transition at the bit start too, 1 = none */
};
enum FskLenMode : uint8_t {
    FSK_LEN_FIXED,   /* len=N */
    FSK_LEN_FIELD,   /* len=@K: a length field */
    FSK_LEN_CARRIER, /* len=carrier: until the level drops */
};
enum FskCounts : uint8_t {
    FSK_CNT_WHOLE,   /* whole_frame: the field counts every byte after the sync (itself and the check too) */
    FSK_CNT_AFTER,   /* after_field: the bytes after the field, the check included */
    FSK_CNT_PAYLOAD, /* payload: the bytes after the field, the check not included */
};
enum FskCheck : uint8_t {
    FSK_CHK_NONE,
    FSK_CHK_XOR8,
    FSK_CHK_SUM8,
    FSK_CHK_CRC8,
    FSK_CHK_CRC16,
};
/* FskRule::flags */
enum : uint8_t {
    FSK_F_LSB = 0x01,    /* bit order lsb_first */
    FSK_F_LEN_LE = 0x02, /* 2 byte length field little endian */
    FSK_F_REFIN = 0x04,
    FSK_F_REFOUT = 0x08,
    FSK_F_CHK_LE = 0x10, /* crc16 sent little endian */
};
constexpr uint8_t kFskBeforeCheck = 0xFF; /* cov_to: every byte before the check */
constexpr uint8_t kFskLast = 0xFF;        /* chk_pos: the check ends the frame */

/* One rule as the M4 runs it (built by the M0 from a class file line). */
struct FskRule {
    uint64_t pattern; /* chips of the preamble tail + sync word, coded, the latest chip in bit 0 */
    uint8_t chips;    /* pattern length, 8..64 */
    uint8_t max_err;  /* chips allowed wrong */
    uint8_t coding;   /* FskCoding */
    uint8_t flags;    /* FSK_F_* */
    uint8_t len_mode; /* FskLenMode */
    uint8_t len_off;  /* field offset */
    uint8_t len_size; /* field bytes, 1..2 */
    uint8_t len_counts;
    int8_t len_adj;
    uint8_t len_min, len_max; /* total bytes after the sync (fixed: both N) */
    uint8_t check;            /* FskCheck */
    uint8_t cov_from, cov_to; /* covered bytes, inclusive; cov_to kFskBeforeCheck */
    uint8_t chk_pos;          /* first check byte, kFskLast */
    uint8_t tag;              /* the M0's rule index */
    uint16_t seed, poly, xorout;
};

/* One dwell: a front end and the rules it searches for. */
struct FskDwell {
    uint32_t chip_hz; /* chips a second: bit rate, x2 for the Manchester codings */
    uint8_t decim;    /* 4 Msps / decim, 4..250 */
    uint8_t ma;       /* discriminator moving average, 1..8 samples */
    uint8_t nrules;
    uint8_t pad;
    FskRule rules[kFskRulesPerDwell];
};

constexpr uint8_t fsk_chips_per_bit(uint8_t coding) {
    return coding == FSK_NRZ ? 1 : 2;
}

constexpr uint8_t fsk_check_size(uint8_t check) {
    return check == FSK_CHK_NONE ? 0 : (check == FSK_CHK_CRC16 ? 2 : 1);
}

/* The chips of `bits` (MSB first in b[], nbits of them) appended to pat. */
inline void fsk_code_bits(uint64_t& pat, uint8_t& level, const uint8_t* b, size_t from, size_t nbits, uint8_t coding) {
    for (size_t k = from; k < from + nbits; k++) {
        const uint8_t v = (b[k >> 3] >> (7 - (k & 7))) & 1;
        if (coding == FSK_NRZ) {
            pat = (pat << 1) | v;
        } else if (coding == FSK_MAN) {
            pat = (pat << 2) | (uint64_t{v} << 1) | (v ^ 1u);
        } else if (coding == FSK_MAN_INV) {
            pat = (pat << 2) | (uint64_t{v ^ 1u} << 1) | v;
        } else {
            const uint8_t c1 = v ? level : static_cast<uint8_t>(level ^ 1);
            pat = (pat << 2) | (uint64_t{c1} << 1) | (c1 ^ 1u);
            level = static_cast<uint8_t>(c1 ^ 1);
        }
    }
}

/* The search pattern: the last pre_bits of the repeated preamble unit
 * (pre[0..pre_len) as sent, MSB first) then the sync word (sync_bits as
 * sent, MSB first in sync[]). The preamble part is at most 16 bits (2
 * bytes) and what still fits in 64 chips; 0 if pre_len or pre_min is 0.
 * false if the sync alone does not fit. */
inline bool fsk_pattern(const uint8_t* pre, uint8_t pre_len, uint8_t pre_min, const uint8_t* sync, uint8_t sync_bits, uint8_t coding, uint64_t& pat, uint8_t& chips) {
    const uint8_t cpb = fsk_chips_per_bit(coding);
    if (sync_bits == 0 || sync_bits * cpb > 64) return false;
    size_t pbits = pre_len && pre_min ? (pre_min * 8u < 16u ? pre_min * 8u : 16u) : 0;
    const size_t room = 64u / cpb - sync_bits;
    if (pbits > room) pbits = room;
    pat = 0;
    uint8_t level = 0;
    /* the preamble's last pbits: unit bits ending with the unit's last bit */
    const size_t ubits = pre_len * 8u;
    for (size_t k = 0; k < pbits; k++) {
        const size_t pos = (ubits - (pbits - k) % ubits) % ubits;
        fsk_code_bits(pat, level, pre, pos, 1, coding);
    }
    fsk_code_bits(pat, level, sync, 0, sync_bits, coding);
    chips = static_cast<uint8_t>((pbits + sync_bits) * cpb);
    return true;
}

inline uint16_t fsk_reflect(uint16_t v, uint8_t width) {
    uint16_t r = 0;
    for (uint8_t i = 0; i < width; i++)
        if (v & (1u << i)) r = static_cast<uint16_t>(r | (1u << (width - 1 - i)));
    return r;
}

/* CRC of width 8 or 16 over b[0..n), bitwise (Rocksoft model). */
inline uint16_t fsk_crc(const uint8_t* b, size_t n, uint8_t width, uint16_t poly, uint16_t seed, bool refin, bool refout, uint16_t xorout) {
    const uint16_t top = static_cast<uint16_t>(1u << (width - 1));
    const uint16_t mask = static_cast<uint16_t>(width == 16 ? 0xFFFF : 0xFF);
    uint16_t crc = static_cast<uint16_t>(seed & mask);
    for (size_t i = 0; i < n; i++) {
        uint8_t v = b[i];
        if (refin) v = static_cast<uint8_t>(fsk_reflect(v, 8));
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(v) << (width - 8)));
        for (int k = 0; k < 8; k++) crc = static_cast<uint16_t>(((crc & top) ? ((crc << 1) ^ poly) : (crc << 1)) & mask);
    }
    if (refout) crc = fsk_reflect(crc, width);
    return static_cast<uint16_t>((crc ^ xorout) & mask);
}

/* Total frame bytes once the length field is in (have >= off + size): the
 * total, or 0 if out of [len_min, len_max] / kFskMaxBytes. */
inline uint8_t fsk_field_total(const FskRule& r, const uint8_t* b) {
    uint32_t L = b[r.len_off];
    if (r.len_size == 2) L = (r.flags & FSK_F_LEN_LE) ? (b[r.len_off] | (b[r.len_off + 1] << 8)) : ((b[r.len_off] << 8) | b[r.len_off + 1]);
    int32_t total = static_cast<int32_t>(L) + r.len_adj;
    if (r.len_counts != FSK_CNT_WHOLE) total += r.len_off + r.len_size;
    if (r.len_counts == FSK_CNT_PAYLOAD) total += fsk_check_size(r.check);
    if (total < r.len_min || total > r.len_max || total > static_cast<int32_t>(kFskMaxBytes) || total < 1) return 0;
    return static_cast<uint8_t>(total);
}

/* The check over a complete frame of n bytes; true for FSK_CHK_NONE. */
inline bool fsk_check_ok(const FskRule& r, const uint8_t* b, size_t n) {
    const uint8_t cs = fsk_check_size(r.check);
    if (!cs) return true;
    const size_t pos = r.chk_pos == kFskLast ? (n >= cs ? n - cs : n) : r.chk_pos;
    if (pos + cs > n) return false;
    const size_t to = r.cov_to == kFskBeforeCheck ? pos : static_cast<size_t>(r.cov_to) + 1; /* exclusive */
    if (to > n || r.cov_from >= to) return false;
    const uint8_t* p = b + r.cov_from;
    const size_t m = to - r.cov_from;
    uint16_t want = b[pos];
    if (cs == 2) want = (r.flags & FSK_F_CHK_LE) ? static_cast<uint16_t>(b[pos] | (b[pos + 1] << 8)) : static_cast<uint16_t>((b[pos] << 8) | b[pos + 1]);
    uint16_t got = 0;
    if (r.check == FSK_CHK_XOR8 || r.check == FSK_CHK_SUM8) {
        uint8_t x = static_cast<uint8_t>(r.seed);
        for (size_t i = 0; i < m; i++) x = r.check == FSK_CHK_XOR8 ? static_cast<uint8_t>(x ^ p[i]) : static_cast<uint8_t>(x + p[i]);
        got = static_cast<uint8_t>(x ^ r.xorout);
    } else {
        got = fsk_crc(p, m, r.check == FSK_CHK_CRC16 ? 16 : 8, r.poly, r.seed, r.flags & FSK_F_REFIN, r.flags & FSK_F_REFOUT, r.xorout);
    }
    return got == want;
}

} /* namespace dd */

#endif /*__DD_FSK_HPP__*/
