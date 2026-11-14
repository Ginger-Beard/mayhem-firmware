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

#ifndef __GPS_NAV_H__
#define __GPS_NAV_H__

/* GPS L1 C/A navigation message (IS-GPS-200, section 20.3): word parity,
 * subframe sync on the TLM preamble, and the ephemeris of subframes 1-3
 * decoded into an Orbit. For the "Autonomous" method (not in the app yet):
 * the tracking channel hands over 50 bps bits, this assembles them.
 * Header only, host tested (rf_test/gps/run_gps_track_test.sh). */

#include <cmath>
#include <cstdint>

#include "gps_orbit.hpp"

namespace ui::external_app::gps {
namespace nav {

/* Parity masks over a 32-bit word laid out as [D29* D30* d1..d24 D25..D30]
 * (bit 31 = D29* of the previous word, bit 30 = D30*, bits 29..6 = d1..d24):
 * IS-GPS-200 table 20-XIV. */
constexpr uint32_t kParityMask[6] = {0x3B1F3480u, 0x1D8F9A40u, 0x2EC7CD00u, 0x1763E680u, 0x2BB1F340u, 0x0B7A89C0u};

inline uint32_t parity_bits(uint32_t w) { /* w: D29*, D30* and the source data bits d1..d24 */
    /* D25..D30 include D29*, D30*, D29*, D30*, D30*, D29* (bit 31 = D29*, bit 30 = D30*) */
    static constexpr int kStar[6] = {31, 30, 31, 30, 30, 31};
    uint32_t p = 0;
    for (int k = 0; k < 6; k++) {
        uint32_t x = w & kParityMask[k];
        x ^= x >> 16;
        x ^= x >> 8;
        x ^= x >> 4;
        x ^= x >> 2;
        x ^= x >> 1;
        uint32_t bit = x & 1u;
        bit ^= (w >> kStar[k]) & 1u;
        p = (p << 1) | bit;
    }
    return p;
}

/* Check one received 30-bit word (bits 29..0 = D1..D30, transmitted polarity)
 * given D29*, D30* of the previous word; on success the 24 source data bits
 * (d1 in bit 23) are in *data. */
inline bool check_word(uint32_t word30, uint32_t prev30, uint32_t* data) {
    const uint32_t d29s = (prev30 >> 1) & 1u, d30s = prev30 & 1u;
    uint32_t d = (word30 >> 6) & 0xFFFFFFu;
    if (d30s) d ^= 0xFFFFFFu; /* data bits are sent inverted when D30* = 1 */
    const uint32_t w = (d29s << 31) | (d30s << 30) | (d << 6);
    if (parity_bits(w) != (word30 & 0x3Fu)) return false;
    *data = d;
    return true;
}

/* Encode (test helper and documentation): source data bits -> 30-bit word. */
inline uint32_t encode_word(uint32_t data24, uint32_t prev30) {
    const uint32_t d29s = (prev30 >> 1) & 1u, d30s = prev30 & 1u;
    const uint32_t w = (d29s << 31) | (d30s << 30) | ((data24 & 0xFFFFFFu) << 6);
    const uint32_t p = parity_bits(w);
    const uint32_t d = d30s ? (data24 ^ 0xFFFFFFu) : data24;
    return ((d & 0xFFFFFFu) << 6) | p;
}

/* Bits of a subframe's ten data words (24 bits each, d1 first): field at
 * 1-based subframe bit position `pos` (as IS-GPS-200 counts, 30 bits per
 * word incl. parity) of `len` bits. */
inline uint32_t field(const uint32_t words[10], int pos, int len) {
    uint32_t v = 0;
    for (int b = 0; b < len; b++) {
        const int p = pos - 1 + b; /* 0-based subframe bit */
        const int w = p / 30, i = p % 30;
        const uint32_t bit = i < 24 ? (words[w] >> (23 - i)) & 1u : 0u;
        v = (v << 1) | bit;
    }
    return v;
}

inline int32_t sfield(const uint32_t words[10], int pos, int len) {
    const uint32_t v = field(words, pos, len);
    return (v & (1u << (len - 1))) ? static_cast<int32_t>(v | ~((1u << len) - 1u)) : static_cast<int32_t>(v);
}

/* Two 8 + 24 bit pieces of a 32-bit parameter. */
inline uint32_t field32(const uint32_t words[10], int pos_msb8, int pos_lsb24) {
    return (field(words, pos_msb8, 8) << 24) | field(words, pos_lsb24, 24);
}

constexpr double kGpsPi = 3.1415926535898; /* the value IS-GPS-200 specifies */

/* ---- subframe-level decoding (the PGPR tracking loops deliver parity-checked
 * subframes: 10 words of 24 data bits) */

/* HOW: subframe ID (1..5) and the TOW (s of week) of this subframe's START
 * (the HOW holds the next subframe's TOW count). */
inline int sf_id(const uint32_t w[10]) { return static_cast<int>(field(w, 50, 3)); }
inline uint32_t sf_tow_start(const uint32_t w[10]) {
    const uint32_t t = field(w, 31, 17);
    return (t == 0 ? 100799u : t - 1u) * 6u;
}
/* full GPS week from the 10-bit week of subframe 1 and a hint (+-512 weeks) */
/* The week hint from the RTC, never before this firmware's era: an RTC reset to
 * 2000 (or 1980) would otherwise resolve the 10-bit week 1024 weeks too early.
 * Right while the RTC is within ~9.8 years after kMinWeek. */
constexpr int32_t kMinWeek = 2440; /* 2026-10 */
inline int32_t week_floor(int32_t w) { return w < kMinWeek ? kMinWeek : w; }

inline int32_t full_week(uint32_t wn10, int32_t week_hint) {
    int32_t wn = static_cast<int32_t>(wn10 & 0x3FFu);
    if (week_hint > 0) {
        wn += week_hint - (week_hint % 1024);
        if (wn - week_hint > 512) wn -= 1024;
        if (week_hint - wn > 512) wn += 1024;
    }
    return wn;
}
inline uint32_t sf1_week10(const uint32_t w[10]) { return field(w, 61, 10); }
inline int sf1_iodc(const uint32_t w[10]) { return static_cast<int>((field(w, 83, 2) << 8) | field(w, 211, 8)); }
inline int sf2_iode(const uint32_t w[10]) { return static_cast<int>(field(w, 61, 8)); }
inline int sf3_iode(const uint32_t w[10]) { return static_cast<int>(field(w, 271, 8)); }

/* Ephemeris from subframes 1, 2, 3 of one issue (IODC low 8 bits = IODE in
 * both). week_hint resolves the 10-bit week. False if the issues differ. */
inline bool decode_ephemeris(const uint32_t s1[10], const uint32_t s2[10], const uint32_t s3[10], uint8_t prn, int32_t week_hint, Orbit& o) {
    const int iodc = sf1_iodc(s1), iode2 = sf2_iode(s2), iode3 = sf3_iode(s3);
    if (iode2 != iode3 || (iodc & 0xFF) != iode2) return false;
    const double sc = kGpsPi;
    const int32_t wn = full_week(sf1_week10(s1), week_hint);
    o = Orbit{};
    o.health = static_cast<uint8_t>(field(s1, 77, 6) != 0);
    o.tgd = static_cast<float>(sfield(s1, 197, 8) * std::ldexp(1.0, -31));
    double toc = field(s1, 219, 16) * 16.0;
    o.af2 = static_cast<float>(sfield(s1, 241, 8) * std::ldexp(1.0, -55));
    o.af1 = static_cast<float>(sfield(s1, 249, 16) * std::ldexp(1.0, -43));
    o.af0 = static_cast<float>(sfield(s1, 271, 22) * std::ldexp(1.0, -31));
    o.crs = static_cast<float>(sfield(s2, 69, 16) * std::ldexp(1.0, -5));
    o.dn = static_cast<float>(sfield(s2, 91, 16) * std::ldexp(1.0, -43) * sc);
    o.m0 = static_cast<int32_t>(field32(s2, 107, 121)) * std::ldexp(1.0, -31) * sc;
    o.cuc = static_cast<float>(sfield(s2, 151, 16) * std::ldexp(1.0, -29));
    o.e = field32(s2, 167, 181) * std::ldexp(1.0, -33);
    o.cus = static_cast<float>(sfield(s2, 211, 16) * std::ldexp(1.0, -29));
    o.sqrt_a = field32(s2, 227, 241) * std::ldexp(1.0, -19);
    double toe = field(s2, 271, 16) * 16.0;
    o.cic = static_cast<float>(sfield(s3, 61, 16) * std::ldexp(1.0, -29));
    o.omega0 = static_cast<int32_t>(field32(s3, 77, 91)) * std::ldexp(1.0, -31) * sc;
    o.cis = static_cast<float>(sfield(s3, 121, 16) * std::ldexp(1.0, -29));
    o.i0 = static_cast<int32_t>(field32(s3, 137, 151)) * std::ldexp(1.0, -31) * sc;
    o.crc = static_cast<float>(sfield(s3, 181, 16) * std::ldexp(1.0, -5));
    o.omega = static_cast<int32_t>(field32(s3, 197, 211)) * std::ldexp(1.0, -31) * sc;
    o.omega_dot = static_cast<float>(sfield(s3, 241, 24) * std::ldexp(1.0, -43) * sc);
    o.idot = static_cast<float>(sfield(s3, 279, 14) * std::ldexp(1.0, -43) * sc);
    /* toe / toc near the week transition: the transmission time decides the week */
    const double tx = sf_tow_start(s1);
    double we = wn, wc = wn;
    if (toe - tx > 302400.0) we -= 1;
    if (tx - toe > 302400.0) we += 1;
    if (toc - tx > 302400.0) wc -= 1;
    if (tx - toc > 302400.0) wc += 1;
    o.toe = we * kWeekS + toe;
    o.toc = wc * kWeekS + toc;
    o.prn = prn;
    o.ephemeris = 1;
    o.valid = 1;
    return true;
}

/* Almanac page (SF5 pages 1-24, SF4 pages 2-5, 7-10: data ID 01, SV ID
 * 1..32). wna: the 8-bit almanac week from SF5 page 25 resolved to a full
 * week by the caller (or -1: toe left as the time of week only). */
inline bool decode_almanac_page(const uint32_t w[10], AlmRec& a, int* svid_out) {
    const int svid = static_cast<int>(field(w, 63, 6));
    if (svid_out) *svid_out = svid;
    if (field(w, 61, 2) != 1 || svid < 1 || svid > 32) return false;
    const double sc = kGpsPi;
    a = AlmRec{};
    a.prn = static_cast<uint8_t>(svid);
    a.e = static_cast<float>(field(w, 69, 16) * std::ldexp(1.0, -21));
    a.toe = field(w, 91, 8) * 4096.0; /* toa, s of the almanac week */
    a.i0 = static_cast<float>((0.30 + sfield(w, 99, 16) * std::ldexp(1.0, -19)) * sc);
    a.omega_dot = static_cast<float>(sfield(w, 121, 16) * std::ldexp(1.0, -38) * sc);
    a.health = static_cast<uint8_t>(field(w, 137, 8) != 0);
    a.sqrt_a = field(w, 151, 24) * std::ldexp(1.0, -11);
    a.omega0 = static_cast<float>(sfield(w, 181, 24) * std::ldexp(1.0, -23) * sc);
    a.omega = static_cast<float>(sfield(w, 211, 24) * std::ldexp(1.0, -23) * sc);
    a.m0 = static_cast<float>(sfield(w, 241, 24) * std::ldexp(1.0, -23) * sc);
    {
        uint32_t af0 = (field(w, 271, 8) << 3) | field(w, 290, 3); /* 11 bits, two's complement */
        int32_t v = (af0 & 0x400u) ? static_cast<int32_t>(af0 | ~0x7FFu) : static_cast<int32_t>(af0);
        a.af0 = static_cast<float>(v * std::ldexp(1.0, -20));
    }
    a.af1 = static_cast<float>(sfield(w, 279, 11) * std::ldexp(1.0, -38));
    a.valid = 1;
    return true;
}
/* SF5 page 25 (SV ID 51): toa (s of week) and the 8-bit almanac week. */
inline bool decode_page25_sf5(const uint32_t w[10], uint32_t* toa, uint32_t* wna8) {
    if (field(w, 63, 6) != 51) return false;
    *toa = field(w, 69, 8) * 4096u;
    *wna8 = field(w, 77, 8);
    return true;
}

/* Assembles bits into subframes; returns an ephemeris once subframes 1-3
 * with the same IODE / IODC have parity-checked. */
class Decoder {
   public:
    explicit Decoder(int32_t week_hint)
        : week_hint_{week_hint} {}

    /* One data bit (0/1, either polarity: the preamble fixes it). Returns
     * true when `out` received a complete ephemeris. */
    bool bit(uint8_t b, uint8_t prn, Orbit& out) {
        shift_ = ((shift_ << 1) | (b & 1u)) & 0x3FFFFFFFFFFFFFFFull; /* last 62 bits */
        nbits_++;
        if (in_frame_) {
            if (++frame_bits_ % 30 == 0) {
                const int w = frame_bits_ / 30 - 1;
                uint32_t raw = static_cast<uint32_t>(shift_ & 0x3FFFFFFFu);
                if (inverted_) raw ^= 0x3FFFFFFFu;
                uint32_t data;
                if (!check_word(raw, prev_word_, &data)) {
                    parity_errors_++;
                    in_frame_ = false;
                    return false;
                }
                words_[w] = data;
                prev_word_ = raw;
                if (w == 9) {
                    in_frame_ = false;
                    return subframe(prn, out);
                }
            }
            return false;
        }
        /* look for TLM + HOW: preamble 10001011 at the start of the last 60 bits,
         * both words with good parity (either polarity) */
        if (nbits_ < 62) return false;
        const uint32_t w1 = static_cast<uint32_t>((shift_ >> 30) & 0x3FFFFFFFu);
        const uint32_t w2 = static_cast<uint32_t>(shift_ & 0x3FFFFFFFu);
        const uint32_t prev = static_cast<uint32_t>((shift_ >> 60) & 3u); /* D29*, D30* before the TLM */
        for (int inv = 0; inv < 2; inv++) {
            const uint32_t m = inv ? 0x3FFFFFFFu : 0;
            const uint32_t a = w1 ^ m, h = w2 ^ m, p = prev ^ (inv ? 3u : 0u);
            uint32_t d1, d2;
            if (!check_word(a, p, &d1) || ((d1 >> 16) & 0xFFu) != 0x8Bu || !check_word(h, a, &d2)) continue;
            inverted_ = inv;
            words_[0] = d1;
            words_[1] = d2;
            prev_word_ = h;
            in_frame_ = true;
            frame_bits_ = 60;
            subframes_found_++;
            return false;
        }
        return false;
    }

    uint32_t parity_errors() const { return parity_errors_; }
    uint32_t subframes() const { return subframes_ok_; }
    uint32_t sync_count() const { return subframes_found_; }
    /* time of week (s) of the start of the next subframe, from the last HOW */
    uint32_t tow() const { return tow_; }

   private:
    uint64_t shift_{0};
    uint32_t nbits_{0};
    bool in_frame_{false};
    bool inverted_{false};
    int frame_bits_{0};
    uint32_t words_[10]{};
    uint32_t prev_word_{0};
    uint32_t parity_errors_{0}, subframes_ok_{0}, subframes_found_{0};
    uint32_t tow_{0};
    int32_t week_hint_;
    /* collected ephemeris */
    Orbit eph_{};
    int iodc_{-1}, iode2_{-1}, iode3_{-1};
    uint8_t have_{0};

    bool subframe(uint8_t prn, Orbit& out) {
        subframes_ok_++;
        tow_ = field(words_, 31, 17) * 6u;
        const int id = static_cast<int>(field(words_, 50, 3));
        const double sc = kGpsPi;
        if (id == 1) {
            int32_t wn = static_cast<int32_t>(field(words_, 61, 10)); /* 10 bits: nearest to the hint */
            if (week_hint_ > 0) {
                wn += week_hint_ - (week_hint_ % 1024);
                if (wn - week_hint_ > 512) wn -= 1024;
                if (week_hint_ - wn > 512) wn += 1024;
            }
            eph_.health = static_cast<uint8_t>(field(words_, 77, 6) != 0);
            iodc_ = static_cast<int>((field(words_, 83, 2) << 8) | field(words_, 211, 8));
            eph_.tgd = static_cast<float>(sfield(words_, 197, 8) * std::ldexp(1.0, -31));
            eph_.toc = wn * kWeekS + field(words_, 219, 16) * 16.0;
            eph_.af2 = static_cast<float>(sfield(words_, 241, 8) * std::ldexp(1.0, -55));
            eph_.af1 = static_cast<float>(sfield(words_, 249, 16) * std::ldexp(1.0, -43));
            eph_.af0 = static_cast<float>(sfield(words_, 271, 22) * std::ldexp(1.0, -31));
            week_ = wn;
            have_ |= 1;
        } else if (id == 2) {
            iode2_ = static_cast<int>(field(words_, 61, 8));
            eph_.crs = static_cast<float>(sfield(words_, 69, 16) * std::ldexp(1.0, -5));
            eph_.dn = static_cast<float>(sfield(words_, 91, 16) * std::ldexp(1.0, -43) * sc);
            eph_.m0 = static_cast<int32_t>(field32(words_, 107, 121)) * std::ldexp(1.0, -31) * sc;
            eph_.cuc = static_cast<float>(sfield(words_, 151, 16) * std::ldexp(1.0, -29));
            eph_.e = field32(words_, 167, 181) * std::ldexp(1.0, -33);
            eph_.cus = static_cast<float>(sfield(words_, 211, 16) * std::ldexp(1.0, -29));
            eph_.sqrt_a = field32(words_, 227, 241) * std::ldexp(1.0, -19);
            toe_sow_ = field(words_, 271, 16) * 16.0;
            have_ |= 2;
        } else if (id == 3) {
            eph_.cic = static_cast<float>(sfield(words_, 61, 16) * std::ldexp(1.0, -29));
            eph_.omega0 = static_cast<int32_t>(field32(words_, 77, 91)) * std::ldexp(1.0, -31) * sc;
            eph_.cis = static_cast<float>(sfield(words_, 121, 16) * std::ldexp(1.0, -29));
            eph_.i0 = static_cast<int32_t>(field32(words_, 137, 151)) * std::ldexp(1.0, -31) * sc;
            eph_.crc = static_cast<float>(sfield(words_, 181, 16) * std::ldexp(1.0, -5));
            eph_.omega = static_cast<int32_t>(field32(words_, 197, 211)) * std::ldexp(1.0, -31) * sc;
            eph_.omega_dot = static_cast<float>(sfield(words_, 241, 24) * std::ldexp(1.0, -43) * sc);
            iode3_ = static_cast<int>(field(words_, 271, 8));
            eph_.idot = static_cast<float>(sfield(words_, 279, 14) * std::ldexp(1.0, -43) * sc);
            have_ |= 4;
        }
        if (have_ == 7 && iode2_ == iode3_ && (iodc_ & 0xFF) == iode2_) {
            eph_.prn = prn;
            eph_.ephemeris = 1;
            eph_.toe = week_ * kWeekS + toe_sow_;
            eph_.valid = 1;
            out = eph_;
            have_ = 0;
            return true;
        }
        return false;
    }
    int32_t week_{0};
    double toe_sow_{0};
};

}  // namespace nav
}  // namespace ui::external_app::gps

#endif /*__GPS_NAV_H__*/
