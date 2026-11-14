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

#ifndef __GPS_TRK_H__
#define __GPS_TRK_H__

/* Real-time GPS L1 C/A tracking for the M4 (PGPR, "Autonomous" method).
 * Header only and free of ChibiOS: the host replay tests run this code.
 *
 * Correlator ("LUT-bitstream", 8-bit samples, no per-sample carrier):
 *  - The 1 ms DMA buffer (2048 complex int8) is cut into blocks of 32
 *    samples. For each block a shared table holds, for each group of 4
 *    samples (a nibble), the 16 possible sums of those samples (I and Q as
 *    two int16 lanes, SADD16). Built once per block for all channels.
 *  - Per channel the replica code of a block is a 32-bit word (bit k = code
 *    is -1 at sample k): the C/A code at half-chip resolution (2046 bits,
 *    each chip twice) read from the half-chip phase m. The code rate is
 *    1 - delta half-chips per sample (delta ~ 1/1024), so m advances 32 per
 *    block except where the half-chip phase repeats (twice per ms): an exact
 *    replica, code phase resolution ~1/2048 chip. Early / prompt / late are
 *    the same word one half chip apart (+-0.5 chip, 1 chip E-L spacing).
 *  - Block correlation: sum(code * x) = S - 2 * sum(x where the bit is 1) =
 *    S - 2 * (8 table look-ups), S the block sum. 3 x 8 look-ups per block.
 *  - The carrier is wiped per block (phase at the block centre, 256-entry
 *    cos/sin table, SMUAD/SMUSDX), not per sample: at 5 kHz Doppler a block
 *    spans 0.49 rad, a loss of 0.09 dB (0.17 dB at 7 kHz).
 *  - Integrate and dump from code epoch to code epoch (1 ms); the block in
 *    which an epoch ends is split with bit masks.
 * Loops per epoch (T = 1 ms): FLL (cross/dot, 1st order) for the pull-in,
 * then a 2nd-order Costas PLL with FLL assist, carrier-aided 1st-order DLL
 * (dot-product normalised E-L envelope). Lock: PLL indicator cos(2 phi)
 * over 20 ms, C/N0 by the M2M4 moment estimator over 100 ms windows.
 * Loss of lock -> local search (+-3.5 chips x +-500 Hz, 20 ms per cell) ->
 * pull-in again; given up after kSearchGiveUpMs. Bit sync by the histogram
 * of prompt sign changes, 20 ms bits, subframe sync on the TLM preamble
 * with IS-GPS-200 parity (confirmed by the next TLM 300 bits later with the
 * TOW one higher): then the transmit time of every sample is known. The
 * loop structure follows the textbook receiver (Kaplan; Borre et al., "A
 * Software-Defined GPS and Galileo Receiver"); written for this firmware. */

#include <cmath>
#include <cstdint>
#include <cstring>

#include "gps_acq.hpp"
#include "gps_trk_shared.hpp"

namespace gps_trk {

constexpr int kFs = 2048000;
constexpr int kBufN = 2048;      /* samples per DMA buffer (1 ms) */
constexpr int kHalfChips = 2046; /* half chips per code period */
constexpr int kBlk = 32;         /* samples per correlator block */
constexpr int kGroupBlk = 8;     /* blocks per table group */
constexpr int kGroupN = kBlk * kGroupBlk;
constexpr int kDdWords = 68;
constexpr double kL1 = 1575.42e6;
constexpr double kChip = 1.023e6;

/* ---------------------------------------------------------------- SIMD */
#if defined(__ARM_ARCH_7EM__)
static inline uint32_t sadd16(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("sadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
static inline uint32_t ssub16(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("ssub16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
static inline int32_t smuad(uint32_t a, uint32_t b) { /* a.lo*b.lo + a.hi*b.hi */
    int32_t r;
    __asm__("smuad %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
static inline int32_t smusdx(uint32_t a, uint32_t b) { /* a.lo*b.hi - a.hi*b.lo */
    int32_t r;
    __asm__("smusdx %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
static inline int32_t smlad(uint32_t a, uint32_t b, int32_t acc) { /* acc + a.lo*b.lo + a.hi*b.hi */
    int32_t r;
    __asm__("smlad %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
static inline int32_t smlsdx(uint32_t a, uint32_t b, int32_t acc) { /* acc + a.lo*b.hi - a.hi*b.lo */
    int32_t r;
    __asm__("smlsdx %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
static inline uint32_t sxtb16(uint32_t a) {
    uint32_t r;
    __asm__("sxtb16 %0, %1" : "=r"(r) : "r"(a));
    return r;
}
static inline uint32_t sxtb16_ror8(uint32_t a) {
    uint32_t r;
    __asm__("sxtb16 %0, %1, ror #8" : "=r"(r) : "r"(a));
    return r;
}
static inline uint32_t pk_lo_lo(uint32_t lo, uint32_t hi) { /* lo[15:0] | hi[15:0] << 16 */
    uint32_t r;
    __asm__("pkhbt %0, %1, %2, lsl #16" : "=r"(r) : "r"(lo), "r"(hi));
    return r;
}
static inline uint32_t pk_hi_hi(uint32_t hi, uint32_t lo) { /* hi[31:16] << 16 | lo[31:16] */
    uint32_t r;
    __asm__("pkhtb %0, %1, %2, asr #16" : "=r"(r) : "r"(hi), "r"(lo));
    return r;
}
static inline void mem_barrier() {
    __asm__ volatile("dmb" ::: "memory");
}
#else
static inline int16_t lo16(uint32_t a) {
    return static_cast<int16_t>(a & 0xFFFFu);
}
static inline int16_t hi16(uint32_t a) {
    return static_cast<int16_t>(a >> 16);
}
static inline uint32_t pack16(int32_t lo, int32_t hi) {
    return (static_cast<uint32_t>(lo) & 0xFFFFu) | (static_cast<uint32_t>(hi) << 16);
}
static inline uint32_t sadd16(uint32_t a, uint32_t b) {
    return pack16(lo16(a) + lo16(b), hi16(a) + hi16(b));
}
static inline uint32_t ssub16(uint32_t a, uint32_t b) {
    return pack16(lo16(a) - lo16(b), hi16(a) - hi16(b));
}
static inline int32_t smuad(uint32_t a, uint32_t b) {
    return lo16(a) * lo16(b) + hi16(a) * hi16(b);
}
static inline int32_t smusdx(uint32_t a, uint32_t b) {
    return lo16(a) * hi16(b) - hi16(a) * lo16(b);
}
static inline int32_t smlad(uint32_t a, uint32_t b, int32_t acc) {
    return acc + smuad(a, b);
}
static inline int32_t smlsdx(uint32_t a, uint32_t b, int32_t acc) {
    return acc + smusdx(a, b);
}
static inline uint32_t sxtb16(uint32_t a) {
    return pack16(static_cast<int8_t>(a & 0xFF), static_cast<int8_t>((a >> 16) & 0xFF));
}
static inline uint32_t sxtb16_ror8(uint32_t a) {
    return pack16(static_cast<int8_t>((a >> 8) & 0xFF), static_cast<int8_t>((a >> 24) & 0xFF));
}
static inline uint32_t pk_lo_lo(uint32_t lo, uint32_t hi) {
    return (lo & 0xFFFFu) | (hi << 16);
}
static inline uint32_t pk_hi_hi(uint32_t hi, uint32_t lo) {
    return (hi & 0xFFFF0000u) | (lo >> 16);
}
static inline void mem_barrier() {
    __asm__ volatile("" ::: "memory");
}
#endif

/* Nibble tables of one block (32 samples): lut[16*k + p] = sum of the
 * samples 4k+b for the bits b set in p, I | Q << 16. Entry 0 of each nibble
 * stays 0 (set once). Returns the block sum. */
static inline uint32_t build_block_lut(const int8_t* iq, uint32_t* lut) {
    uint32_t s = 0;
    for (int k = 0; k < 8; k++, iq += 8, lut += 16) {
        uint32_t w0, w1;
        std::memcpy(&w0, iq, 4);
        std::memcpy(&w1, iq + 4, 4);
        const uint32_t a0 = sxtb16(w0), b0 = sxtb16_ror8(w0), a1 = sxtb16(w1), b1 = sxtb16_ror8(w1);
        const uint32_t x0 = pk_lo_lo(a0, b0), x1 = pk_hi_hi(b0, a0);
        const uint32_t x2 = pk_lo_lo(a1, b1), x3 = pk_hi_hi(b1, a1);
        const uint32_t e3 = sadd16(x0, x1), e5 = sadd16(x2, x0), e6 = sadd16(x2, x1), e7 = sadd16(x2, e3);
        const uint32_t e15 = sadd16(x3, e7);
        lut[1] = x0;
        lut[2] = x1;
        lut[3] = e3;
        lut[4] = x2;
        lut[5] = e5;
        lut[6] = e6;
        lut[7] = e7;
        lut[8] = x3;
        lut[9] = sadd16(x3, x0);
        lut[10] = sadd16(x3, x1);
        lut[11] = sadd16(x3, e3);
        lut[12] = sadd16(x3, x2);
        lut[13] = sadd16(x3, e5);
        lut[14] = sadd16(x3, e6);
        lut[15] = e15;
        s = sadd16(s, e15);
    }
    return s;
}

/* Sum of the block's samples whose bit is set in w (I | Q << 16). */
static inline uint32_t lut_sum(const uint32_t* lut, uint32_t w) {
    uint32_t t = lut[w & 15u];
    t = sadd16(t, lut[16 + ((w >> 4) & 15u)]);
    t = sadd16(t, lut[32 + ((w >> 8) & 15u)]);
    t = sadd16(t, lut[48 + ((w >> 12) & 15u)]);
    t = sadd16(t, lut[64 + ((w >> 16) & 15u)]);
    t = sadd16(t, lut[80 + ((w >> 20) & 15u)]);
    t = sadd16(t, lut[96 + ((w >> 24) & 15u)]);
    t = sadd16(t, lut[112 + (w >> 28)]);
    return t;
}

/* Fast two-quadrant arctangent of y / x (rad, +-pi/2), max error ~0.0015 rad. */
static inline float atan_2q(float y, float x) {
    if (x == 0.0f) return y == 0.0f ? 0.0f : (y > 0 ? 1.5707963f : -1.5707963f);
    float z = y / x;
    const bool inv = z > 1.0f || z < -1.0f;
    if (inv) z = 1.0f / z;
    const float z2 = z * z;
    float a = z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f + z2 * (-0.11643287f + z2 * (0.05265332f + z2 * -0.01172120f)))));
    if (inv) a = (z > 0 ? 1.5707963f : -1.5707963f) - a;
    return a;
}

/* ---------------------------------------------------------------- nav framer */

/* IS-GPS-200 word parity (table 20-XIV), written for the framer: w holds
 * D29* (bit 31), D30* (bit 30) and the 24 source bits (bits 29..6). */
static inline uint32_t parity6(uint32_t w) {
    static constexpr uint32_t kMask[6] = {0x3B1F3480u, 0x1D8F9A40u, 0x2EC7CD00u, 0x1763E680u, 0x2BB1F340u, 0x0B7A89C0u};
    /* D25..D30 include D29*, D30*, D29*, D30*, D30*, D29* (bit 31 = D29*, bit 30 = D30*) */
    static constexpr int kStar[6] = {31, 30, 31, 30, 30, 31};
    uint32_t p = 0;
    for (int k = 0; k < 6; k++) {
        uint32_t x = w & kMask[k];
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

/* word30: D1..D30 as received (bits 29..0), prev: the previous word (its
 * D29, D30 in bits 1, 0). On good parity *data = d1..d24 (d1 in bit 23). */
static inline bool word_ok(uint32_t word30, uint32_t prev, uint32_t* data) {
    const uint32_t d29s = (prev >> 1) & 1u, d30s = prev & 1u;
    uint32_t d = (word30 >> 6) & 0xFFFFFFu;
    if (d30s) d ^= 0xFFFFFFu;
    if (parity6((d29s << 31) | (d30s << 30) | (d << 6)) != (word30 & 0x3Fu)) return false;
    *data = d;
    return true;
}

/* Bits -> parity-checked subframes. Sync is first found anywhere (TLM
 * preamble + good TLM and HOW parity, either polarity), then confirmed by
 * the next TLM exactly 300 bits later whose HOW TOW is one higher; from then
 * on TLMs are only expected at 300-bit steps (two misses in a row drop the
 * sync). */
class Framer {
   public:
    enum Event : uint8_t { NONE = 0,
                           SUBFRAME = 1 };
    void reset() { *this = Framer{}; }
    /* One bit; returns SUBFRAME when words()/sfid()/tow_start() hold a
     * complete parity-good subframe. pos_in_sf() gives the bit's position in
     * its subframe while synced. */
    Event bit(uint8_t b) {
        shift_ = ((shift_ << 1) | (b & 1u)) & 0x3FFFFFFFFFFFFFFFull;
        nbits_++;
        if (cand_ && !confirmed_) cand_bits_++;
        if (in_frame_) {
            frame_bits_++;
            if (frame_bits_ % 30 == 0) {
                const int w = frame_bits_ / 30 - 1;
                uint32_t raw = static_cast<uint32_t>(shift_ & 0x3FFFFFFFu);
                if (inverted_) raw ^= 0x3FFFFFFFu;
                uint32_t d;
                if (!word_ok(raw, prev_, &d)) {
                    parity_errs_++;
                    in_frame_ = false;
                    since_tlm_ = static_cast<uint16_t>(frame_bits_);
                    return NONE;
                }
                words_[w] = d;
                prev_ = raw;
                words_ok_++;
                if (w == 9) {
                    in_frame_ = false;
                    since_tlm_ = 300;
                    subframes_++;
                    return SUBFRAME;
                }
            }
            return NONE;
        }
        uint32_t tow;
        if (confirmed_) {
            /* expect the next TLM + HOW at 300-bit steps */
            if (++since_tlm_ >= 360) {
                if (try_tlm(&tow) && tow == next_tow_) {
                    misses_ = 0;
                    begin_frame(tow);
                } else if (++misses_ >= 2) {
                    confirmed_ = false;
                    cand_ = false;
                } else {
                    since_tlm_ = 60;
                    next_tow_ = (next_tow_ + 1) % 100800u;
                }
            }
            return NONE;
        }
        if (nbits_ < 62) return NONE;
        if (cand_) {
            if (cand_bits_ < 300) return NONE;
            /* where the next TLM + HOW must end */
            cand_ = false;
            if (try_tlm(&tow) && tow == (cand_tow_ + 1) % 100800u) {
                confirmed_ = true;
                misses_ = 0;
                begin_frame(tow);
                return NONE;
            }
        }
        if (try_tlm(&tow)) {
            cand_ = true;
            cand_tow_ = tow;
            cand_bits_ = 0;
            begin_frame(tow);
        }
        return NONE;
    }
    bool synced() const { return confirmed_; }
    /* the TOW (s of week) of the start of the subframe being / last read */
    uint32_t sf_tow() const { return cur_tow_s_; }
    const uint32_t* words() const { return words_; }
    uint8_t sfid() const { return static_cast<uint8_t>((words_[1] >> 2) & 7u); }
    uint32_t parity_errs() const { return parity_errs_; }
    uint32_t subframes() const { return subframes_; }
    uint32_t words_ok() const { return words_ok_; }
    /* bits since the start of the current subframe (valid while synced): the
     * channel keeps its own count; this is for tests */
    uint32_t nbits() const { return nbits_; }

   private:
    uint64_t shift_{0};
    uint32_t nbits_{0};
    bool in_frame_{false}, inverted_{false};
    bool cand_{false}, confirmed_{false};
    uint8_t misses_{0};
    int frame_bits_{0};
    uint16_t since_tlm_{0};
    uint16_t cand_bits_{0};
    uint32_t cand_tow_{0}, next_tow_{0}, cur_tow_s_{0};
    uint32_t words_[10]{};
    uint32_t prev_{0};
    uint32_t parity_errs_{0}, subframes_{0}, words_ok_{0};

    /* TLM + HOW in the last 60 bits, either polarity; *tow = HOW TOW count */
    bool try_tlm(uint32_t* tow) {
        const uint32_t w1 = static_cast<uint32_t>((shift_ >> 30) & 0x3FFFFFFFu);
        const uint32_t w2 = static_cast<uint32_t>(shift_ & 0x3FFFFFFFu);
        const uint32_t pv = static_cast<uint32_t>((shift_ >> 60) & 3u);
        for (int inv = 0; inv < 2; inv++) {
            const uint32_t m = inv ? 0x3FFFFFFFu : 0;
            const uint32_t a = w1 ^ m, h = w2 ^ m, p = pv ^ (inv ? 3u : 0u);
            uint32_t d1, d2;
            if (!word_ok(a, p, &d1) || ((d1 >> 16) & 0xFFu) != 0x8Bu || !word_ok(h, a, &d2)) continue;
            const uint32_t t = d2 >> 7;         /* 17-bit TOW count of the next subframe */
            const uint32_t id = (d2 >> 2) & 7u; /* subframe ID */
            if (t >= 100800u || id < 1 || id > 5) continue;
            inverted_ = inv;
            words_[0] = d1;
            words_[1] = d2;
            prev_ = h;
            *tow = t;
            return true;
        }
        return false;
    }
    void begin_frame(uint32_t tow) {
        in_frame_ = true;
        frame_bits_ = 60;
        since_tlm_ = 60;
        next_tow_ = (tow + 1) % 100800u;
        cur_tow_s_ = (tow == 0 ? 100800u - 1 : tow - 1) * 6u; /* the HOW holds the NEXT subframe's TOW */
    }
};

/* ---------------------------------------------------------------- channel */

/* cos/sin table for the per-block carrier wipe: cos | sin << 16, amplitude 2047 */
struct CarrierTable {
    uint32_t cs[256];
    CarrierTable() {
        for (int k = 0; k < 256; k++) {
            const double a = 2.0 * 3.14159265358979323846 * k / 256.0;
            const int32_t c = static_cast<int32_t>(std::lround(2047.0 * std::cos(a)));
            const int32_t s = static_cast<int32_t>(std::lround(2047.0 * std::sin(a)));
            cs[k] = (static_cast<uint32_t>(c) & 0xFFFFu) | (static_cast<uint32_t>(s) << 16);
        }
    }
};

struct LoopParams {
    float pll_bn{15.0f};      /* Hz, steady */
    float pll_bn_wide{20.0f}; /* Hz, after the pull-in */
    float fll_bn{10.0f};      /* Hz, pull-in (FLL only) */
    float fll_assist{3.0f};   /* Hz, FLL assist while wide */
    float dll_bn{0.5f};       /* Hz, steady */
    float dll_bn_wide{2.0f};
    /* > 0: a channel that has never verified (1 s of PLL lock at >= 28 dB-Hz) nor
     * found bit sync this long after its start is given up (TRK_FAILED +
     * F_RELEASED): the hand-over of an unconfirmed detection (Autonomous) */
    uint32_t verify_timeout_ms{0};
};

class Channel {
   public:
    static constexpr int kPullinEpochs = 200; /* FLL only */
    static constexpr int kWideEpochs = 800;   /* PLL wide + FLL assist, at least */
    static constexpr int kCn0Win = 100;       /* M2M4 window, epochs */
    static constexpr float kLossCn0 = 25.0f;  /* dB-Hz: two windows below -> lost */
    static constexpr float kVerifyCn0 = 28.0f;
    static constexpr int kSearchK = 20;            /* epochs per search cell */
    static constexpr int kSearchCode = 5;          /* code centres, 3 half chips apart (taps E/P/L) */
    static constexpr int kSearchDop = 5;           /* Doppler bins, 250 Hz apart */
    static constexpr uint32_t kPullinMaxMs = 1500; /* not locked by then -> search */
    static constexpr uint32_t kSearchGiveUpMs = 30000;
    static constexpr float kSearchThr = 2.75f; /* max / mean cell power, K = 20 (as acquisition) */

    void stop() {
        state_ = TRK_IDLE;
        prn_ = 0;
    }
    ChState state() const { return static_cast<ChState>(state_); }
    uint8_t prn() const { return prn_; }

    /* Start on PRN with the code phase (chips, received code at the sample
     * `at`) and the Doppler of an acquisition. */
    void start(int prn, double code_phase_chips, float doppler_hz, const LoopParams& lp) {
        int8_t chips[1023];
        gps_acq::ca_code(prn, chips);
        std::memset(dd_, 0, sizeof(dd_));
        /* DD[b] = D[b - 1], wrapping (no division: a start runs inside one
         * buffer's time, next to the other channels) */
        for (int b = 0, i = kHalfChips - 1; b < kDdWords * 32; b++) {
            if (chips[i >> 1] < 0) dd_[b >> 5] |= 1u << (b & 31);
            if (++i == kHalfChips) i = 0;
        }
        prn_ = static_cast<uint8_t>(prn);
        lp_ = lp;
        reacq_ = 0;
        ever_ok_ = released_ = rel_reported_ = false;
        aided_ = false;
        double h = code_phase_chips * 2.0;
        h = std::fmod(h, static_cast<double>(kHalfChips));
        if (h < 0) h += kHalfChips;
        m_ = static_cast<uint32_t>(h);
        if (m_ >= kHalfChips) m_ = 0;
        f_ = static_cast<uint32_t>((h - std::floor(h)) * 4294967296.0);
        phc_ = 0;
        epochs_ = 0;
        restart(doppler_hz);
    }

    /* ---- hot path: one group of blocks (shared tables) */
    void run_group(const uint32_t* lut, const uint32_t* sblk, const uint32_t* cs, Shared* sh, int ch);
    /* samples missed (DMA buffers lost): propagate the replica and count epochs */
    void skip(uint64_t samples, Shared* sh, int ch);

    /* Transmit time at the current sample (the next to be processed), for
     * the measurements: false without subframe sync. */
    bool tx_time(uint32_t* tx_ms, uint32_t* tx_frac) const {
        if (!tsync_ || state_ != TRK_TRACK) return false;
        /* whole ms since the reference subframe start + this epoch's fraction */
        uint64_t ms = static_cast<uint64_t>(t_tow_ms_) + (epochs_ - t_epoch_);
        ms %= 604800000ull;
        /* half chips (m + f) / 2046 of a ms */
        const double fr = (m_ + f_ / 4294967296.0) / kHalfChips;
        uint64_t q = static_cast<uint64_t>(fr * 4294967296.0);
        if (q > 0xFFFFFFFFull) q = 0xFFFFFFFFull;
        *tx_ms = static_cast<uint32_t>(ms);
        *tx_frac = static_cast<uint32_t>(q);
        return true;
    }
    void info(ChInfo& o) const {
        o.prn = prn_;
        o.state = state_;
        o.flags = flags();
        o.reacq = reacq_;
        o.cn0_x10 = static_cast<uint16_t>(cn0_ > 0 ? cn0_ * 10.0f + 0.5f : 0);
        o.doppler_hz = freq_int_;
        o.lock_ms = state_ == TRK_TRACK ? track_ms_ : 0;
        o.subframes = framer_.subframes();
        o.parity_errs = framer_.parity_errs();
        o.tow_s = last_tow_s_;
        o.mode = state_ == TRK_SEARCH ? 3 : (state_ == TRK_PULLIN ? 0 : (narrow_ ? 2 : 1));
        const float pl = pld_s_ * 100.0f;
        o.pld_x100 = static_cast<int8_t>(pl > 100.0f ? 100 : (pl < -100.0f ? -100 : pl));
        o.bits = bits_out_;
        o.words_ok = framer_.words_ok();
        o.code_x64 = static_cast<uint16_t>((m_ + f_ / 4294967296.0) * 32.0);
        o.sweeps = sweeps_;
        o.code_err = code_err_;
        o.half_fixes = half_fixes_;
    }
    uint8_t flags() const {
        uint8_t f = 0;
        if (pll_lock_) f |= F_PLL;
        if (bit_edge_ >= 0) f |= F_BIT;
        if (tsync_) f |= F_FRAME;
        if (verified_) f |= F_VERIFIED;
        if (ever_ok_) f |= F_EVER;
        if (released_) f |= F_RELEASED;
        if (aided_) f |= F_AIDED;
        return f;
    }
    float cn0() const { return cn0_; }
    double doppler() const { return freq_int_; }
    /* mean NCO frequency since the last call (the measurement interval) */
    double take_mean_doppler() {
        const double d = dop_n_ ? dop_sum_ / dop_n_ : freq_int_;
        dop_sum_ = 0;
        dop_n_ = 0;
        return d;
    }
    uint32_t epochs() const { return epochs_; }
    uint32_t track_ms() const { return track_ms_; }
    bool released() const { return released_; }
    bool aided() const { return aided_; }
    /* A newly risen satellite: search windows of 15 half chips around the
     * predicted code phase (+-wmax windows) and +-500 Hz around the predicted
     * Doppler; found -> pull-in and tracking as usual, else given up after
     * kSearchGiveUpMs. Never released by the verification timeout while it
     * searches (that is for hand-overs of unconfirmed detections). */
    void start_aided(int prn, double code_phase_chips, float doppler_hz, const LoopParams& lp, int wmax) {
        start(prn, code_phase_chips, doppler_hz, lp);
        lp_.verify_timeout_ms = 0;
        aided_ = true;
        good_freq_ = doppler_hz;
        lost();
        reacq_ = 0;
        s_wmax_ = static_cast<uint8_t>(wmax < 0 ? 0 : (wmax > 40 ? 40 : wmax));
    }
    /* true once per release (the tracker reports it, then may reuse the slot) */
    bool take_release() {
        if (!released_ || rel_reported_) return false;
        rel_reported_ = true;
        return true;
    }
    /* test access */
    double code_phase_halfchips() const { return m_ + f_ / 4294967296.0; }
    int bit_edge() const { return bit_edge_; }
    const Framer& framer() const { return framer_; }
    uint8_t reacq() const { return reacq_; }
    uint32_t bits_out() const { return bits_out_; }
    float freq_watch() const { return hf_last_; } /* the last half-epoch frequency error estimate, Hz */
    /* per-bit callback for tests (nullptr in the firmware) */
    void (*on_bit)(void* ctx, int prn, int bit) = nullptr;
    void* on_bit_ctx = nullptr;

   private:
    uint32_t dd_[kDdWords]{};
    /* replica state at the next sample */
    uint32_t m_{0};      /* half-chip index of the prompt, 0..2045 */
    uint32_t f_{0};      /* its fraction, 2^-32 */
    uint32_t delta_{0};  /* 1 - half chips per sample, 2^-32 */
    float delta_res_{0}; /* rounding carried to the next epoch */
    uint32_t phc_{0};    /* carrier phase at the centre of the next block, 2^-32 cycles */
    uint32_t wblk_{0};   /* carrier advance per block (32 samples), 2^-32 cycles: 1.5e-5 Hz steps */
    float wblk_res_{0};  /* its rounding, carried to the next epoch */
    /* epoch in progress (Q negated: SMLSDX): IP -QP, ID -QD with D = (E - L) / 2 (fast
     * blocks: every other one), IE -QE IL -QL (search only, every block) */
    int32_t acc_[8]{};
    bool el_all_{false}; /* the search: E, P, L of every block (exact path) */
    float p2_avg_{0};    /* smoothed |P|^2 for the DLL normalisation */
    double dop_sum_{0};  /* NCO frequency summed per epoch since the last measurement */
    uint32_t dop_n_{0};
    uint32_t epoch_samples_{0};
    bool epoch_gap_{false};
    uint32_t epochs_{0}; /* prompt code epochs since start */

    LoopParams lp_{};
    uint8_t prn_{0};
    uint8_t state_{TRK_IDLE};
    uint8_t reacq_{0};
    float freq_{0};       /* carrier NCO, Hz (float: 0.5 mHz steps at 7 kHz) */
    float freq_int_{0};   /* loop integrator, Hz */
    float code_corr_{0};  /* DLL correction, chips/s */
    uint32_t mode_ms_{0}; /* epochs in the current state */
    uint32_t track_ms_{0};
    float prev_i_{0}, prev_q_{0};
    bool have_prev_{false};
    /* lock detectors */
    float pld_i_{0}, pld_q_{0};
    int pld_n_{0};
    float pld_s_{0};
    bool pll_lock_{false};
    bool narrow_{false};
    uint32_t lock_good_{0}, lock_bad_{0};
    float m2_{0}, m4_{0};
    int cn0_n_{0};
    float cn0_{0};
    int cn0_low_{0};
    int cn0_wins_{0};
    uint32_t verify_ms_{0};
    bool verified_{false};
    bool ever_ok_{false};  /* verified or bit-synced since start() */
    bool released_{false}; /* given up unverified (LoopParams::verify_timeout_ms) */
    bool rel_reported_{false};
    float good_freq_{0};
    /* bit sync and bits */
    uint16_t hist_[20]{};
    int hist_n_{0};
    int8_t prev_sign_{0};
    int bit_edge_{-1}; /* epoch index mod 20 of a bit start */
    float bit_acc_{0};
    int bit_cnt_{0};
    uint32_t bits_out_{0};
    /* frame / time */
    Framer framer_{};
    bool tsync_{false};
    uint32_t t_epoch_{0};  /* epoch index of a subframe start */
    uint32_t t_tow_ms_{0}; /* its TOW, ms */
    uint32_t last_tow_s_{0};
    uint32_t sf_bit_{0}; /* bits since that subframe start */
    /* search */
    float cell_[kSearchCode * kSearchDop * 3]{};
    int s_cell_{0}, s_k_{0};
    int s_off_{0};      /* half chips currently applied */
    int s_base_{0};     /* centre of the current window, half chips from the start position */
    uint8_t s_win_{0};  /* aided search: window index (0, +1, -1, +2, -2 ...) */
    uint8_t s_wmax_{0}; /* aided search: windows either side (0: the local search of a lost channel) */
    bool aided_{false}; /* started by start_aided(): a newly risen satellite, searched around a prediction */
    float s_f0_{0};
    uint32_t s_ms_{0};
    uint16_t sweeps_{0}; /* diagnostics */
    float code_err_{0};
    /* +-500 Hz false-lock detector: with 1 ms correlations both the FLL
     * (cross/dot) and the Costas PLL are blind to half a cycle per ms, so a
     * carrier 500 Hz off "locks" ~4 dB down with the prompt sign flipping every
     * epoch (seen on the device after re-acquisitions). In true lock the sign
     * changes only at bit edges (<= 5 %) and by noise (~15 % at 30 dB-Hz). */
    static constexpr int kHfWin = 100;       /* epochs per frequency-error estimate (noise ~25-55 Hz rms at 33 dB-Hz) */
    static constexpr float kHfThr = 250.0f;  /* Hz: two windows in a row */
    static constexpr float kHfSure = 350.0f; /* Hz: one window */
    int32_t half_i_{0}, half_q_{0};          /* the prompt sums when the epoch's first half ended */
    bool half_taken_{false}, half_ok_{false};
    float h1i_{0}, h1q_{0};
    static constexpr float kHfCoh = 0.3f;
    float hf_dot_{0}, hf_cross_{0}, hf_p1_{0}, hf_p2_{0}, hf_last_{0};
    int hf_n_{0};
    uint8_t hf_hits_{0};
    uint16_t half_fixes_{0};

    void restart(float doppler_hz) {
        freq_ = freq_int_ = good_freq_ = doppler_hz;
        p2_avg_ = 0;
        dop_sum_ = 0;
        dop_n_ = 0;
        half_taken_ = half_ok_ = false;
        hf_dot_ = hf_cross_ = hf_p1_ = hf_p2_ = 0;
        hf_n_ = 0;
        hf_hits_ = 0;
        code_corr_ = 0;
        delta_res_ = 0;
        wblk_res_ = 0;
        el_all_ = false;
        set_nco();
        state_ = TRK_PULLIN;
        mode_ms_ = 0;
        track_ms_ = 0;
        have_prev_ = false;
        pld_i_ = pld_q_ = pld_s_ = 0;
        pld_n_ = 0;
        pll_lock_ = narrow_ = false;
        lock_good_ = lock_bad_ = 0;
        m2_ = m4_ = 0;
        cn0_n_ = 0;
        cn0_ = 0;
        cn0_low_ = 0;
        cn0_wins_ = 0;
        verify_ms_ = 0;
        verified_ = false;
        reset_bits();
        for (auto& a : acc_) a = 0;
        epoch_samples_ = 0;
        epoch_gap_ = false;
    }
    void reset_bits() {
        for (auto& h : hist_) h = 0;
        hist_n_ = 0;
        prev_sign_ = 0;
        bit_edge_ = -1;
        bit_acc_ = 0;
        bit_cnt_ = 0;
        framer_.reset();
        tsync_ = false;
    }
    void set_nco() {
        /* carrier: advance per 32-sample block, 2^-32 cycles */
        const float w = freq_ * (32.0f * 4294967296.0f / kFs) + wblk_res_;
        const float wr = std::floor(w + 0.5f);
        wblk_res_ = w - wr;
        wblk_ = static_cast<uint32_t>(static_cast<int32_t>(wr));
        /* code: delta = 1 - 2 f_code / fs, f_code = 1.023 MHz (1 + f / L1) + DLL, in 2^-32:
         * 2^22 - (2046 / 2048 / L1 * 2^32) f - (2 / fs * 2^32) corr */
        const float d = 4194304.0f - static_cast<float>(2046.0 / 2048.0 / 1575.42e6 * 4294967296.0) * freq_ -
                        static_cast<float>(2.0 / 2048000.0 * 4294967296.0) * code_corr_ + delta_res_;
        const float dr = std::floor(d + 0.5f);
        delta_res_ = d - dr;
        delta_ = static_cast<uint32_t>(static_cast<int32_t>(dr));
    }
    void shift_code(int halfchips) {
        int m = static_cast<int>(m_) + halfchips;
        while (m < 0) m += kHalfChips;
        while (m >= kHalfChips) m -= kHalfChips;
        m_ = static_cast<uint32_t>(m);
    }

    void slow_block(const uint32_t* lut, uint32_t s, uint32_t c, Shared* sh, int ch);
    void epoch_end(Shared* sh, int ch);
    void epoch_skipped(Shared* sh, int ch);
    void track_epoch(float ip, float qp, float id, float qd, bool valid, Shared* sh, int ch);
    void search_epoch(float ie, float qe, float ip, float qp, float il, float ql);
    void bit_epoch(float ip, bool valid, Shared* sh, int ch);
    void got_bit(int b, Shared* sh, int ch);
    void lost();
};

/* 60 of the 64 blocks of a buffer: no half-chip repeat and no epoch end in
 * them. Prompt every block; D = (E - L) / 2 = sum over the late bits - sum
 * over the early bits, every other block (and in every slow block). */
inline void Channel::run_group(const uint32_t* lut, const uint32_t* sblk, const uint32_t* cs, Shared* sh, int ch) {
    if (el_all_) { /* the search: every block on the exact path */
        uint32_t phc = phc_;
        for (int b = 0; b < kGroupBlk; b++, lut += 128) {
            const uint32_t c = cs[(phc + (1u << 23)) >> 24];
            phc += wblk_;
            phc_ = phc;
            slow_block(lut, sblk[b], c, sh, ch);
            phc = phc_;
        }
        return;
    }
    uint32_t m = m_, f = f_, d32 = delta_ * 32u, phc = phc_, wblk = wblk_, ns = 0;
    int32_t ip = acc_[0], qp = acc_[1], id = acc_[2], qd = acc_[3];
    for (int b = 0; b < kGroupBlk; b++, lut += 128) {
        const uint32_t c = cs[(phc + (1u << 23)) >> 24];
        phc += wblk;
        if (f < d32 || m >= static_cast<uint32_t>(kHalfChips - 32)) {
            acc_[0] = ip, acc_[1] = qp, acc_[2] = id, acc_[3] = qd;
            m_ = m, f_ = f, phc_ = phc;
            epoch_samples_ += ns;
            ns = 0;
            slow_block(lut, sblk[b], c, sh, ch);
            m = m_, f = f_, d32 = delta_ * 32u, phc = phc_, wblk = wblk_;
            ip = acc_[0], qp = acc_[1], id = acc_[2], qd = acc_[3];
            continue;
        }
        const uint32_t* d = dd_ + (m >> 5);
        const uint32_t sh5 = m & 31u;
        const uint64_t win = (((static_cast<uint64_t>(d[1]) << 32) | d[0]) >> sh5) | ((static_cast<uint64_t>(d[2]) << 32) << (32 - sh5));
        const uint32_t s = sblk[b];
        const uint32_t tp = lut_sum(lut, static_cast<uint32_t>(win >> 1));
        const uint32_t cp = ssub16(ssub16(s, tp), tp);
        ip = smlad(cp, c, ip);
        qp = smlsdx(cp, c, qp);
        if (!(b & 1)) {
            const uint32_t cd = ssub16(lut_sum(lut, static_cast<uint32_t>(win)), lut_sum(lut, static_cast<uint32_t>(win >> 2)));
            id = smlad(cd, c, id);
            qd = smlsdx(cd, c, qd);
        }
        f -= d32;
        m += 32;
        ns += 32;
    }
    acc_[0] = ip, acc_[1] = qp, acc_[2] = id, acc_[3] = qd;
    m_ = m, f_ = f, phc_ = phc;
    epoch_samples_ += ns;
    if (!half_taken_ && epoch_samples_ >= kBufN / 2) { /* the prompt of the epoch's first half (the false-lock watch) */
        half_i_ = ip;
        half_q_ = qp;
        half_taken_ = true;
    }
}

/* A block with a half-chip repeat and / or the end of an epoch (within it
 * or at its end), and every block of the search: exact replica words, the
 * block split with bit masks. P and D always; E and L in the search. */
__attribute__((noinline)) inline void Channel::slow_block(const uint32_t* lut, uint32_t s, uint32_t c, Shared* sh, int ch) {
    uint32_t m = m_, f = f_;
    const uint32_t delta = delta_, d32 = delta * 32u;
    const uint32_t* d = dd_ + (m >> 5);
    const uint32_t sh5 = m & 31u;
    const uint64_t win = (((static_cast<uint64_t>(d[1]) << 32) | d[0]) >> sh5) | ((static_cast<uint64_t>(d[2]) << 32) << (32 - sh5));
    uint32_t wl = static_cast<uint32_t>(win), wp = static_cast<uint32_t>(win >> 1), we = static_cast<uint32_t>(win >> 2);
    uint32_t adv = 32;
    uint32_t t = 32; /* samples of this block in the current epoch */
    if (f < d32) {   /* the half-chip phase repeats at step j */
        const uint32_t j = f / delta;
        adv = 31;
        if (j <= 30) {
            const uint32_t lo = (2u << j) - 1u;
            wl = (wl & lo) | ((wl << 1) & ~lo);
            wp = (wp & lo) | ((wp << 1) & ~lo);
            we = (we & lo) | ((we << 1) & ~lo);
            if (m + 30 >= static_cast<uint32_t>(kHalfChips)) t = (kHalfChips - m <= j) ? kHalfChips - m : kHalfChips + 1 - m;
        } else if (m + 31 >= static_cast<uint32_t>(kHalfChips)) {
            t = kHalfChips - m;
        }
    } else if (m + 31 >= static_cast<uint32_t>(kHalfChips)) {
        t = kHalfChips - m;
    }
    const bool all = el_all_;
    const uint32_t te = lut_sum(lut, we), tp = lut_sum(lut, wp), tl = lut_sum(lut, wl);
    uint32_t mlo = 0xFFFFFFFFu, slo = s, telo = te, tplo = tp, tllo = tl;
    if (t < 32) {
        mlo = (1u << t) - 1u;
        slo = lut_sum(lut, mlo);
        telo = lut_sum(lut, we & mlo);
        tplo = lut_sum(lut, wp & mlo);
        tllo = lut_sum(lut, wl & mlo);
    }
    /* the samples of the current epoch */
    {
        const uint32_t cp = ssub16(ssub16(slo, tplo), tplo), cd = ssub16(tllo, telo);
        acc_[0] = smlad(cp, c, acc_[0]);
        acc_[1] = smlsdx(cp, c, acc_[1]);
        acc_[2] = smlad(cd, c, acc_[2]);
        acc_[3] = smlsdx(cd, c, acc_[3]);
        if (all) {
            const uint32_t ce = ssub16(ssub16(slo, telo), telo), cl = ssub16(ssub16(slo, tllo), tllo);
            acc_[4] = smlad(ce, c, acc_[4]);
            acc_[5] = smlsdx(ce, c, acc_[5]);
            acc_[6] = smlad(cl, c, acc_[6]);
            acc_[7] = smlsdx(cl, c, acc_[7]);
        }
    }
    if (t >= 32) {
        epoch_samples_ += 32;
        f -= d32;
        m += adv;
        if (m >= static_cast<uint32_t>(kHalfChips)) { /* the epoch ended exactly at the block edge */
            m_ = m - kHalfChips, f_ = f;
            epoch_end(sh, ch);
            return;
        }
        m_ = m, f_ = f;
        return;
    }
    epoch_samples_ += t;
    m_ = m, f_ = f;
    epoch_end(sh, ch); /* may change the NCO rates and, in the search, move m_ */
    const int sft = static_cast<int>(m_) - static_cast<int>(m);
    f -= d32;
    int mm = static_cast<int>(m) + static_cast<int>(adv) - kHalfChips;
    if (sft != 0) { /* the search moved the replica: drop the rest of this block */
        mm += sft;
        while (mm < 0) mm += kHalfChips;
        while (mm >= kHalfChips) mm -= kHalfChips;
        m_ = static_cast<uint32_t>(mm), f_ = f;
        return;
    }
    /* the rest of the block starts the next epoch */
    const uint32_t shi = ssub16(s, slo), tehi = ssub16(te, telo), tphi = ssub16(tp, tplo), tlhi = ssub16(tl, tllo);
    const uint32_t cp = ssub16(ssub16(shi, tphi), tphi), cd = ssub16(tlhi, tehi);
    acc_[0] = smuad(cp, c);
    acc_[1] = smusdx(cp, c);
    acc_[2] = smuad(cd, c);
    acc_[3] = smusdx(cd, c);
    if (all) {
        const uint32_t ce = ssub16(ssub16(shi, tehi), tehi), cl = ssub16(ssub16(shi, tlhi), tlhi);
        acc_[4] = smuad(ce, c);
        acc_[5] = smusdx(ce, c);
        acc_[6] = smuad(cl, c);
        acc_[7] = smusdx(cl, c);
    }
    epoch_samples_ = 32 - t;
    m_ = static_cast<uint32_t>(mm), f_ = f;
}

inline void Channel::skip(uint64_t samples, Shared* sh, int ch) {
    if (state_ == TRK_IDLE || state_ == TRK_FAILED || samples == 0) return;
    /* carrier */
    phc_ += static_cast<uint32_t>(samples / kBlk) * wblk_;
    /* code: samples steps of (1 - delta): m advances samples - repeats */
    const uint64_t total = samples * static_cast<uint64_t>(delta_);
    uint64_t rep = 0;
    if (total > f_) rep = (total - f_ + 0xFFFFFFFFull) >> 32;
    f_ = static_cast<uint32_t>(static_cast<uint64_t>(f_) - total);
    uint64_t mm = static_cast<uint64_t>(m_) + samples - rep;
    uint64_t crossed = mm / kHalfChips;
    m_ = static_cast<uint32_t>(mm % kHalfChips);
    if (crossed == 0) {
        epoch_gap_ = true;
        return;
    }
    epoch_gap_ = true;
    epoch_end(sh, ch); /* the partial epoch, marked as a gap */
    for (uint64_t k = 1; k < crossed; k++) epoch_skipped(sh, ch);
    for (auto& a : acc_) a = 0;
    epoch_samples_ = 0;
    epoch_gap_ = true; /* the epoch now in progress started inside the gap */
}

inline void Channel::epoch_end(Shared* sh, int ch) {
    const float k = 1.0f / 65536.0f;
    const bool valid = !epoch_gap_ && epoch_samples_ >= 2040;
    epoch_gap_ = false;
    epoch_samples_ = 0;
    const float ip = acc_[0] * k, qp = -acc_[1] * k, id = acc_[2] * k, qd = -acc_[3] * k;
    /* the two halves of the epoch's prompt (the first ends at a group boundary near
     * the middle; their centres are always half an epoch apart) */
    half_ok_ = half_taken_ && valid;
    h1i_ = half_i_ * k, h1q_ = -half_q_ * k;
    half_taken_ = false;
    if (state_ == TRK_SEARCH) {
        const float ie = acc_[4] * k, qe = -acc_[5] * k, il = acc_[6] * k, ql = -acc_[7] * k;
        for (auto& a : acc_) a = 0;
        search_epoch(ie, qe, ip, qp, il, ql);
    } else {
        for (auto& a : acc_) a = 0;
        track_epoch(ip, qp, id, qd, valid, sh, ch);
    }
    epochs_++;
    if (lp_.verify_timeout_ms && !ever_ok_) {
        if (verified_ || bit_edge_ >= 0)
            ever_ok_ = true;
        else if (epochs_ > lp_.verify_timeout_ms && state_ != TRK_IDLE && state_ != TRK_FAILED) {
            state_ = TRK_FAILED; /* a false detection must not hold a channel */
            released_ = true;
        }
    } else if (!ever_ok_ && (verified_ || bit_edge_ >= 0))
        ever_ok_ = true;
}

inline void Channel::epoch_skipped(Shared* sh, int ch) {
    if (state_ != TRK_SEARCH) bit_epoch(0, false, sh, ch);
    epochs_++;
    mode_ms_++;
    if (state_ == TRK_TRACK) track_ms_++;
}

inline void Channel::lost() {
    if (++reacq_ == 0) reacq_ = 255;
    state_ = TRK_SEARCH;
    el_all_ = true;
    sweeps_ = 0;
    reset_bits();
    pll_lock_ = narrow_ = verified_ = false;
    cn0_ = 0;
    s_f0_ = good_freq_;
    s_cell_ = 0;
    s_k_ = 0;
    s_off_ = 0;
    s_base_ = 0;
    s_win_ = s_wmax_ = 0;
    s_ms_ = 0;
    for (auto& c : cell_) c = 0;
    /* first cell: code offset -6 half chips, Doppler f0 - 500 */
    code_corr_ = 0;
    shift_code(-6);
    s_off_ = -6;
    freq_ = s_f0_ - 500.0f;
    set_nco();
}

inline void Channel::search_epoch(float ie, float qe, float ip, float qp, float il, float ql) {
    s_ms_++;
    if (s_k_ > 0) { /* the first epoch after a move is partial: skip it */
        float* c = &cell_[3 * s_cell_];
        c[0] += ie * ie + qe * qe;
        c[1] += ip * ip + qp * qp;
        c[2] += il * il + ql * ql;
    }
    if (++s_k_ <= kSearchK) return;
    s_k_ = 0;
    const int ncell = kSearchCode * kSearchDop;
    if (++s_cell_ < ncell) {
        const int ci = s_cell_ % kSearchCode, di = s_cell_ / kSearchCode;
        const int off = s_base_ + (ci - kSearchCode / 2) * 3;
        shift_code(off - s_off_);
        s_off_ = off;
        freq_ = s_f0_ + (di - kSearchDop / 2) * 250.0f;
        set_nco();
        return;
    }
    /* sweep done: best cell against the mean */
    double sum = 0;
    int best = 0;
    for (int i = 0; i < ncell * 3; i++) {
        sum += cell_[i];
        if (cell_[i] > cell_[best]) best = i;
    }
    const double mean = sum / (ncell * 3);
    const int bc = best / 3, tap = best % 3;
    if (mean > 0 && cell_[best] / mean > kSearchThr) {
        const int ci = bc % kSearchCode, di = bc / kSearchCode;
        const int off = s_base_ + (ci - kSearchCode / 2) * 3 + (tap == 0 ? 1 : tap == 2 ? -1
                                                                                        : 0); /* early tap = replica + 1 half chip */
        shift_code(off - s_off_);
        restart(s_f0_ + (di - kSearchDop / 2) * 250.0f);
        return;
    }
    if (s_ms_ > kSearchGiveUpMs) {
        state_ = TRK_FAILED;
        return;
    }
    /* again (the code phase keeps running at the last Doppler); an aided search
     * moves to the next window of 15 half chips: 0, +1, -1, +2, -2 ... +-s_wmax_ */
    if (sweeps_ < 65535) sweeps_++;
    for (auto& c : cell_) c = 0;
    s_cell_ = 0;
    if (s_wmax_) {
        if (++s_win_ > 2 * s_wmax_) s_win_ = 0;
        const int k = (s_win_ + 1) / 2;
        s_base_ = (s_win_ & 1u) ? k * kSearchCode * 3 : -k * kSearchCode * 3;
    }
    const int off = s_base_ + (0 - kSearchCode / 2) * 3;
    shift_code(off - s_off_);
    s_off_ = off;
    freq_ = s_f0_ - 500.0f;
    set_nco();
}

inline void Channel::track_epoch(float ip, float qp, float id, float qd, bool valid, Shared* sh, int ch) {
    mode_ms_++;
    if (state_ == TRK_TRACK) track_ms_++;
    bit_epoch(ip, valid, sh, ch);
    if (!valid) { /* a gap: no loop update; the next epoch has no FLL reference */
        have_prev_ = false;
        return;
    }
    constexpr float T = 1e-3f;
    constexpr float kTwoPi = 6.2831853f;
    /* discriminators */
    const float pll_err = atan_2q(qp, ip) / kTwoPi; /* cycles, +-0.25 */
    float fll_err = 0;
    if (have_prev_) {
        const float dot = prev_i_ * ip + prev_q_ * qp;
        const float cross = prev_i_ * qp - prev_q_ * ip;
        fll_err = atan_2q(cross, dot) / (kTwoPi * T); /* Hz, +-250 */
    }
    prev_i_ = ip;
    prev_q_ = qp;
    have_prev_ = true;
    /* code: coherent dot-product discriminator Re{(E - L) P*} / |P|^2 (= 2 tau for the
     * 1-chip E-L spacing), D from about half the blocks (x 2), |P|^2 smoothed */
    const float p2 = ip * ip + qp * qp;
    p2_avg_ = p2_avg_ > 0 ? p2_avg_ + 0.05f * (p2 - p2_avg_) : p2;
    float code_err = p2_avg_ > 0 ? 2.0f * (id * ip + qd * qp) / p2_avg_ : 0.0f; /* chips (D = (E - L) / 2) */
    if (code_err > 0.5f) code_err = 0.5f;
    if (code_err < -0.5f) code_err = -0.5f;
    code_err_ = code_err;
    /* carrier loop */
    if (state_ == TRK_PULLIN && mode_ms_ <= static_cast<uint32_t>(kPullinEpochs)) {
        const float k = 4.0f * lp_.fll_bn;
        freq_int_ += k * T * fll_err;
        freq_ = freq_int_;
    } else {
        if (state_ == TRK_PULLIN) { /* FLL done: PLL from here on */
            state_ = TRK_TRACK;
            mode_ms_ = 0;
            track_ms_ = 0;
        }
        const float bn = narrow_ ? lp_.pll_bn : lp_.pll_bn_wide;
        const float wn = bn / 0.53f;
        const float wf = narrow_ ? 0.0f : lp_.fll_assist / 0.25f;
        freq_int_ += T * (wn * wn * pll_err + wf * fll_err);
        freq_ = freq_int_ + 1.414f * wn * pll_err;
    }
    /* code loop, carrier aided */
    const float kd = 4.0f * (narrow_ ? lp_.dll_bn : lp_.dll_bn_wide);
    code_corr_ = kd * code_err; /* chips/s */
    set_nco();
    dop_sum_ += freq_;
    dop_n_++;
    /* PLL lock indicator over 20 ms (5 before bit sync: fewer bit edges inside) */
    pld_i_ += ip;
    pld_q_ += qp;
    if (++pld_n_ >= (bit_edge_ >= 0 ? 20 : 5)) {
        const float a = pld_i_ * pld_i_, bq = pld_q_ * pld_q_;
        const float pld = (a + bq) > 0 ? (a - bq) / (a + bq) : 0;
        pld_s_ += 0.25f * (pld - pld_s_);
        pld_i_ = pld_q_ = 0;
        pld_n_ = 0;
        const bool good = pld_s_ > 0.7f && state_ == TRK_TRACK;
        if (good) {
            lock_good_++;
            lock_bad_ = 0;
        } else {
            lock_bad_++;
            lock_good_ = 0;
        }
        pll_lock_ = pld_s_ > 0.6f && state_ == TRK_TRACK;
        if (!narrow_ && state_ == TRK_TRACK && mode_ms_ >= static_cast<uint32_t>(kWideEpochs) && lock_good_ >= 10) {
            narrow_ = true;
        } else if (narrow_ && lock_bad_ >= (bit_edge_ >= 0 ? 40 : 160)) { /* ~0.8 s of poor phase lock: widen again */
            narrow_ = false;
            mode_ms_ = 0;
        }
    }
    /* +-500 Hz false lock (and any large residual after a pull-in): the two
     * half-epoch prompts (0.5 ms apart, four-quadrant: +-1000 Hz) measure the
     * frequency error directly; summed over kHfWin epochs it reads ~0 in true
     * lock, ~+-500 Hz in the false lock and the real offset while a weak
     * channel's PLL wanders. Above kHfSure in one window, or above kHfThr in
     * two windows in a row: the carrier moves by the measured error (not a
     * blind 500 Hz step). Watched from the second half of the pull-in on (the
     * 1 ms FLL is as blind as the PLL), so a false lock is fixed before the
     * low C/N0 it causes ends the channel. */
    if ((state_ == TRK_TRACK || (state_ == TRK_PULLIN && mode_ms_ > kPullinEpochs / 2)) && half_ok_) {
        const float h2i = ip - h1i_, h2q = qp - h1q_;
        hf_dot_ += h1i_ * h2i + h1q_ * h2q;
        hf_cross_ += h1i_ * h2q - h1q_ * h2i;
        hf_p1_ += h1i_ * h1i_ + h1q_ * h1q_;
        hf_p2_ += h2i * h2i + h2q * h2q;
        if (++hf_n_ >= kHfWin) {
            /* coherence of the products: ~0.5 at 33 dB-Hz (true or false lock), ~0.1
             * on noise (a lost satellite's last windows must not move the carrier) */
            const float pp = hf_p1_ * hf_p2_;
            const bool coherent = pp > 0 && (hf_dot_ * hf_dot_ + hf_cross_ * hf_cross_) > kHfCoh * kHfCoh * pp;
            hf_p1_ = hf_p2_ = 0;
            float y = hf_cross_, x = hf_dot_;
            float a = atan_2q(y, x);
            if (x < 0) a += y >= 0 ? 3.14159265f : -3.14159265f;
            const float fe = a / (6.2831853f * (kBufN / 2.0f / kFs)); /* the halves are ~half an epoch apart */
            hf_dot_ = hf_cross_ = 0;
            hf_n_ = 0;
            hf_last_ = fe;
            if (coherent && (fe > kHfThr || fe < -kHfThr)) {
                if (++hf_hits_ >= 2 || fe > kHfSure || fe < -kHfSure) {
                    freq_int_ += fe;
                    freq_ = freq_int_;
                    hf_hits_ = 0;
                    if (half_fixes_ < 65535) half_fixes_++;
                    narrow_ = false;
                    mode_ms_ = 0;
                    lock_good_ = lock_bad_ = 0;
                    pld_s_ = 0;
                    pld_i_ = pld_q_ = 0;
                    pld_n_ = 0;
                    pll_lock_ = false;
                    have_prev_ = false;
                    reset_bits(); /* the bits so far were garbage */
                    set_nco();
                }
            } else {
                hf_hits_ = 0;
            }
        }
    }
    /* C/N0: M2M4 over windows of kCn0Win epochs */
    m2_ += p2;
    m4_ += p2 * p2;
    if (++cn0_n_ >= kCn0Win) {
        const float M2 = m2_ / kCn0Win, M4 = m4_ / kCn0Win;
        float pd2 = 2.0f * M2 * M2 - M4;
        float est = 0;
        if (pd2 > 0) {
            const float pd = std::sqrt(pd2), pn = M2 - pd;
            if (pn > 0) est = 10.0f * std::log10(pd / pn / T);
        }
        if (est < 0) est = 0;
        cn0_ = cn0_wins_ == 0 ? est : cn0_ + 0.3f * (est - cn0_);
        cn0_wins_++;
        m2_ = m4_ = 0;
        cn0_n_ = 0;
        if (est < kLossCn0) {
            cn0_low_++;
        } else {
            cn0_low_ = 0;
            if (pll_lock_) good_freq_ = freq_;
        }
        /* lost: three windows in a row below, or the smoothed estimate below after
         * half a second (noise alone gives scattered single windows above 25) */
        if ((cn0_low_ >= 3 && cn0_wins_ >= 3) || (cn0_wins_ >= 5 && cn0_ < kLossCn0 - 2.0f)) {
            lost();
            return;
        }
    }
    if (state_ == TRK_PULLIN && mode_ms_ > kPullinMaxMs) {
        lost();
        return;
    }
    if (state_ == TRK_TRACK && !verified_) {
        if (pll_lock_ && cn0_ >= kVerifyCn0)
            verify_ms_++;
        else
            verify_ms_ = 0;
        if (verify_ms_ >= 1000) verified_ = true;
        if (track_ms_ > 6000 && !pll_lock_ && cn0_ < kVerifyCn0) { /* never locked properly */
            lost();
            return;
        }
    }
}

inline void Channel::bit_epoch(float ip, bool valid, Shared* sh, int ch) {
    const int e20 = static_cast<int>(epochs_ % 20u);
    if (bit_edge_ < 0) {
        if (!pll_lock_ || !valid) {
            prev_sign_ = 0;
            return;
        }
        const int8_t sgn = ip >= 0 ? 1 : -1;
        if (prev_sign_ && sgn != prev_sign_) {
            hist_[e20]++;
            hist_n_++;
        }
        prev_sign_ = sgn;
        if (hist_n_ >= 30 && hist_n_ % 10 == 0) {
            int best = 0;
            for (int k = 1; k < 20; k++)
                if (hist_[k] > hist_[best]) best = k;
            float others = 0;
            for (int k = 0; k < 20; k++)
                if (k != best) others += hist_[k];
            others /= 19.0f;
            /* the edge bin must stand out from the noise sign flips */
            if (hist_[best] >= 15 && hist_[best] - others > 5.0f * std::sqrt(others + 1.0f)) {
                bit_edge_ = best;
                bit_acc_ = 0;
                bit_cnt_ = -1; /* wait for the first full bit */
            } else if (hist_n_ >= 1500) {
                for (auto& h : hist_) h = 0;
                hist_n_ = 0;
            }
        }
        return;
    }
    if (e20 == bit_edge_) {
        if (bit_cnt_ >= 10)
            got_bit(bit_acc_ < 0 ? 1 : 0, sh, ch);
        else if (bit_cnt_ >= 0)
            got_bit(0, sh, ch); /* mostly lost to gaps: a guess (parity will tell) */
        bit_acc_ = 0;
        bit_cnt_ = 0;
    }
    if (bit_cnt_ >= 0 && valid) {
        bit_acc_ += ip;
        bit_cnt_++;
    } else if (bit_cnt_ >= 0 && !valid) {
        /* nothing: the bit is shorter */
    }
}

inline void Channel::got_bit(int b, Shared* sh, int ch) {
    bits_out_++;
    if (on_bit) on_bit(on_bit_ctx, prn_, b);
    /* the bit just finished started 20 epochs before this epoch */
    const uint32_t bit_start = epochs_ - 20u;
    const bool was_synced = framer_.synced();
    const auto ev = framer_.bit(static_cast<uint8_t>(b));
    if (framer_.synced()) {
        if (!was_synced || ev == Framer::SUBFRAME || !tsync_) {
            /* re-anchor the transmit time on the subframe the framer is in */
        }
        /* bit position in the subframe: the framer reads a frame from its
         * TLM; while in a frame the bit index is known */
    }
    if (ev == Framer::SUBFRAME) {
        /* this bit was bit 299 of the subframe: the subframe started 299 bits before it */
        const uint32_t sf_start = bit_start - 299u * 20u;
        const uint32_t tow_s = framer_.sf_tow();
        if (framer_.synced()) {
            const uint32_t tow_ms = tow_s * 1000u;
            if (tsync_) {
                /* consistency: the predicted TOW of this start */
                const uint64_t pred = (static_cast<uint64_t>(t_tow_ms_) + (sf_start - t_epoch_)) % 604800000ull;
                if (pred != tow_ms) tsync_ = false;
            }
            t_epoch_ = sf_start;
            t_tow_ms_ = tow_ms;
            if (!tsync_) tsync_ = true;
        }
        last_tow_s_ = tow_s;
        if (sh) {
            const uint32_t w = sh->sf_write;
            SubframeSlot& s = sh->sf[w % kSfSlots];
            s.prn = prn_;
            s.sfid = framer_.sfid();
            s.ch = static_cast<uint8_t>(ch);
            s.tow_s = tow_s;
            std::memcpy(s.words, framer_.words(), sizeof(s.words));
            mem_barrier();
            sh->sf_write = w + 1;
        }
    }
    if (!framer_.synced()) tsync_ = false;
}

}  // namespace gps_trk

/* the Galileo E1 channel (uses the helpers above; the Tracker runs it next to the GPS channels) */
#include "gal_trk.hpp"

namespace gps_trk {

/* ---------------------------------------------------------------- tracker */

class Tracker {
   public:
    Tracker(Shared* sh, const CarrierTable* table, gal_acq::TrkShared* gsh = nullptr)
        : sh_{sh}, gsh_{gsh}, cs_{table->cs} {
        std::memset(static_cast<void*>(sh_), 0, sizeof(Shared));
        if (gsh_) std::memset(static_cast<void*>(gsh_), 0, sizeof(gal_acq::TrkShared));
        std::memset(lut_, 0, sizeof(lut_));
    }
    /* ---- the channel pool shared by GPS and Galileo (M4 time). Costs in GPS
     * channels x10: a GPS channel 10, a Galileo channel kGalCost (measured:
     * rf_test/gps m4bench_trk). GPS never waits: a GPS start that overfills the
     * pool frees the weakest Galileo channel. Galileo takes a channel only
     * while the pool has room and the app allows (set_max_gal; 0 = none). */
    static constexpr int kGpsCost = 10;
    static constexpr int kGalCost = 16;
    static constexpr int kPoolBudget = 108;
    /* Galileo channels allowed (the app's setting); channels beyond it are freed */
    GAL_COLD void set_max_gal(int n) {
        max_gal_ = n < 0 ? 0 : (n > gal_trk::kMaxTrk ? gal_trk::kMaxTrk : n);
        for (int i = max_gal_; i < gal_trk::kMaxTrk; i++)
            if (gal_[i].in_use()) gal_[i].release(gal_acq::TR_OFF);
    }
    int max_gal() const { return max_gal_; }
    void set_pool(int budget_x10, int gal_cost_x10) {
        pool_budget_ = budget_x10;
        gal_cost_ = gal_cost_x10;
    }
    int pool_used() const {
        int u = 0;
        for (const auto& c : ch_) u += (c.state() != TRK_IDLE && c.state() != TRK_FAILED) ? kGpsCost : 0;
        for (const auto& g : gal_) u += g.in_use() ? gal_cost_ : 0;
        return u;
    }
    int gal_in_use() const {
        int n = 0;
        for (const auto& g : gal_) n += g.in_use();
        return n;
    }
    /* A Galileo detection (the aided search): half-chip streams, code phase
     * (chips) at sample `at`, Doppler; `now` = the next sample process() gets.
     * Returns gal_acq::TrkHandover. */
    GAL_COLD uint8_t gal_start(int prn, const uint32_t* hc_b, const uint32_t* hc_c, double code_phase_chips, double doppler_hz, uint64_t at, uint64_t now) {
        int slot = -1;
        for (int i = 0; i < gal_trk::kMaxTrk; i++) {
            if (gal_[i].in_use() && gal_[i].prn() == prn) return gal_acq::HO_ALREADY;
            if (slot < 0 && !gal_[i].in_use() && i < max_gal_) slot = i;
        }
        if (max_gal_ <= 0) return gal_acq::HO_OFF;
        if (slot < 0 || pool_used() + gal_cost_ > pool_budget_) {
            if (gsh_) gsh_->refused = static_cast<uint16_t>(gsh_->refused + 1);
            return gal_acq::HO_NO_ROOM;
        }
        const double dt = (static_cast<double>(now) - static_cast<double>(at)) / kFs;
        const double cp = code_phase_chips + dt * kChip * (1.0 + doppler_hz / kL1);
        if (gsh_) gal_[slot].attach_symbols(gsh_->sym[slot], &gsh_->sym_write[slot], &gsh_->sym_epoch[slot]);
        gal_[slot].start(prn, hc_b, hc_c, cp, static_cast<float>(doppler_hz), gp_);
        if (active_ == 0) next_ = now;
        if (gsh_) gsh_->handovers = static_cast<uint16_t>(gsh_->handovers + 1);
        return gal_acq::HO_TRACKED;
    }
    /* Galileo off (the app): every Galileo channel released */
    void gal_off() { set_max_gal(0); }
    /* a lost Galileo channel waiting for a re-acquisition search, round robin
     * (-1: none) */
    int gal_reacq_next() {
        for (int k = 1; k <= gal_trk::kMaxTrk; k++) {
            const int i = (gal_rr_ + k) % gal_trk::kMaxTrk;
            if (gal_[i].wants_search()) {
                gal_rr_ = i;
                return i;
            }
        }
        return -1;
    }
    /* its result: code phase (chips) at sample `at`; `now` as in gal_start */
    GAL_COLD void gal_reacq_done(int i, const gal_acq::Result& r, uint64_t at, uint64_t now) {
        gal_trk::Channel& g = gal_[i];
        if (!g.wants_search()) return;
        if (gsh_) gsh_->reacq_searches = static_cast<uint16_t>(gsh_->reacq_searches + 1);
        if (r.detected) {
            const double dt = (static_cast<double>(now) - static_cast<double>(at)) / kFs;
            g.reacquired(r.code_phase + dt * kChip * (1.0 + r.doppler / kL1), r.doppler);
        } else if (g.search_ms() > kGalGiveUpMs) {
            g.release(gal_acq::TR_LOST);
        }
    }
    static constexpr uint32_t kGalGiveUpMs = 60000;
    gal_trk::Channel& gal(int i) { return gal_[i]; }
    const gal_trk::Channel& gal(int i) const { return gal_[i]; }
    gal_trk::Params& gal_params() { return gp_; }
    gal_acq::TrkShared* gal_shared() { return gsh_; }
    /* channels in use at most (M4 time: ~11.5k cycles each per 1 ms + ~23k shared) */
    void set_max_active(int n) { max_active_ = n < 1 ? 1 : (n > kMaxCh ? kMaxCh : n); }
    int max_active() const { return max_active_; }
    void reset(uint64_t now) {
        for (auto& c : ch_) c.stop();
        for (auto& g : gal_) g.stop();
        active_ = 0;
        next_ = now;
        publish();
    }
    /* Start a channel: code phase (chips) of the received code at sample
     * `at`, Doppler at that time; the replica is propagated to `now`, the
     * first sample of the next buffer given to process(). Returns the
     * channel or -1. */
    int start(int prn, double code_phase_chips, double doppler_hz, uint64_t at, uint64_t now) {
        int slot = -1;
        for (int i = 0; i < max_active_; i++) {
            if (ch_[i].prn() == prn && ch_[i].state() != TRK_IDLE && ch_[i].state() != TRK_FAILED) return i;
            if (slot < 0 && (ch_[i].state() == TRK_IDLE || ch_[i].state() == TRK_FAILED)) slot = i;
        }
        if (slot < 0) return -1;
        /* the code moves at the observed code rate (the sample clock shares the TCXO) */
        const double dt = (static_cast<double>(now) - static_cast<double>(at)) / kFs;
        const double cp = code_phase_chips + dt * kChip * (1.0 + doppler_hz / kL1);
        ch_[slot].start(prn, cp, static_cast<float>(doppler_hz), lp_);
        if (active_ == 0) next_ = now;
        make_room();
        return slot;
    }
    void stop(int prn) {
        for (auto& c : ch_)
            if (c.prn() == prn) c.stop();
    }
    /* PGPR's hand-over of a search result: the detected PRNs strongest first
     * into the channels; the rest wait as spares and take channels that fail
     * later (released unverified detections, or satellites given up). Code
     * phases refer to sample `at`; `now` is the next buffer's first sample.
     * Returns the number of detections. */
    int handover(const gps_acq::Result* res, uint32_t prn_mask, uint64_t at, uint64_t now) {
        int order[32];
        int n = 0;
        for (int i = 0; i < 32; i++)
            if (((prn_mask >> i) & 1u) && res[i].detected && !res[i].once) order[n++] = i;
        for (int a = 1; a < n; a++)
            for (int b = a; b > 0 && res[order[b]].cn0 > res[order[b - 1]].cn0; b--) {
                const int t = order[b];
                order[b] = order[b - 1];
                order[b - 1] = t;
            }
        reset(now);
        n_spare_ = next_spare_ = 0;
        spare_at_ = at;
        for (int k = 0; k < n; k++) {
            const gps_acq::Result& r = res[order[k]];
            if (k < max_active_)
                start(order[k] + 1, r.code_phase, r.doppler, at, now);
            else {
                Spare& s = spare_[n_spare_++];
                s.prn = static_cast<uint8_t>(order[k] + 1);
                s.code_phase = r.code_phase;
                s.doppler = r.doppler;
            }
        }
        return n;
    }
    int spares_left() const { return n_spare_ - next_spare_; }
    /* A search for a newly risen satellite (the app predicts code phase at
     * sample `at` and Doppler from the fix and the orbits): a free channel among
     * the first max_active, else the one extra search slot (index max_active:
     * at most one aided search beyond the channel count, ~15k cycles per ms).
     * Returns the channel or -1 (busy, or the PRN already has a channel). */
    __attribute__((noinline)) int start_aided(int prn, double code_phase_chips, double doppler_hz, uint64_t at, uint64_t now, int wmax) {
        int slot = -1;
        const int lim = max_active_ < kMaxCh ? max_active_ + 1 : kMaxCh;
        for (int i = 0; i < lim; i++) {
            const auto st = ch_[i].state();
            if (ch_[i].prn() == prn && st != TRK_IDLE && st != TRK_FAILED) return -1;
            if (slot < 0 && (st == TRK_IDLE || st == TRK_FAILED) && (i < max_active_ || !extra_busy())) slot = i;
        }
        if (slot < 0) return -1;
        const double dt = (static_cast<double>(now) - static_cast<double>(at)) / kFs;
        const double cp = code_phase_chips + dt * kChip * (1.0 + doppler_hz / kL1);
        ch_[slot].start_aided(prn, cp, static_cast<float>(doppler_hz), lp_, wmax);
        if (active_ == 0) next_ = now;
        make_room();
        sh_->aided_starts = static_cast<uint16_t>(sh_->aided_starts + 1);
        return slot;
    }
    /* the extra slot is taken (by an aided search or the satellite it found) */
    bool extra_busy() const {
        if (max_active_ >= kMaxCh) return true;
        const auto st = ch_[max_active_].state();
        return st != TRK_IDLE && st != TRK_FAILED;
    }
    /* One DMA buffer: n == kBufN samples starting at absolute sample `at`.
     * A jump of `at` (missed buffers) is propagated first. */
    void process(const int8_t* iq, uint64_t at) {
        if (at != next_) {
            if (at > next_) {
                for (int i = 0; i < kMaxCh; i++) ch_[i].skip(at - next_, sh_, i);
                for (auto& g : gal_) g.skip(at - next_);
            }
            sh_->missed++;
        }
        next_ = at;
        /* measurements once per second, at the first buffer of each second */
        if (at / kFs != last_meas_s_) {
            last_meas_s_ = at / kFs;
            measure(at);
            releases_and_refill(at);
        }
        int n_on = 0, n_used = 0;
        nbuf_++;
        for (int i = 0; i < kMaxCh; i++) {
            const auto st = ch_[i].state();
            on_[i] = st != TRK_IDLE && st != TRK_FAILED;
            n_used += on_[i];
            /* a lost channel's search (exact path, ~2x the cost of tracking) runs on
             * every other buffer: 8 lost at once stay within the M4 time */
            if (on_[i] && st == TRK_SEARCH && ((nbuf_ + i) & 1u)) {
                ch_[i].skip(kBufN, sh_, i);
                on_[i] = false;
            }
            n_on += on_[i];
        }
        /* Galileo: tracking channels correlate; one waiting for a search keeps its
         * code phase running (the search window is centred on it) */
        int n_gal = 0;
        for (int i = 0; i < gal_trk::kMaxTrk; i++) {
            gon_[i] = gal_[i].on();
            n_gal += gon_[i];
            n_used += gal_[i].in_use();
            if (gal_[i].wants_search()) gal_[i].skip(kBufN);
        }
        active_ = static_cast<uint8_t>(n_used);
        if (n_on || n_gal) {
            for (int g = 0; g < kBufN / kGroupN; g++) {
                const int8_t* p = iq + 2 * g * kGroupN;
                for (int b = 0; b < kGroupBlk; b++) sblk_[b] = build_block_lut(p + 2 * kBlk * b, lut_ + 128 * b);
                for (int i = 0; i < kMaxCh; i++)
                    if (on_[i]) ch_[i].run_group(lut_, sblk_, cs_, sh_, i);
                for (int i = 0; i < gal_trk::kMaxTrk; i++)
                    if (gon_[i]) gal_[i].run_group(lut_, sblk_, cs_);
            }
        }
        next_ = at + kBufN;
        sh_->buffers++;
    }
    /* Channel table for the app (seqlock). */
    __attribute__((noinline)) void publish() {
        sh_->ch_seq = sh_->ch_seq + 1;
        mem_barrier();
        for (int i = 0; i < kMaxCh; i++) ch_[i].info(sh_->ch[i]);
        sh_->active = active_;
        mem_barrier();
        sh_->ch_seq = sh_->ch_seq + 1;
        if (!gsh_) return;
        gsh_->seq = gsh_->seq + 1;
        mem_barrier();
        for (int i = 0; i < gal_trk::kMaxTrk; i++) gal_[i].info(gsh_->ch[i]);
        gsh_->max_ch = static_cast<uint8_t>(max_gal_);
        gsh_->active = static_cast<uint8_t>(gal_in_use());
        gsh_->pool_x10 = static_cast<uint16_t>(pool_used());
        gsh_->pool_budget_x10 = static_cast<uint16_t>(pool_budget_);
        mem_barrier();
        gsh_->seq = gsh_->seq + 1;
    }
    Channel& channel(int i) { return ch_[i]; }
    const Channel& channel(int i) const { return ch_[i]; }
    Shared* shared() { return sh_; }
    uint8_t active() const { return active_; }
    uint64_t next_sample() const { return next_; }
    LoopParams& params() { return lp_; }

   private:
    Shared* sh_;
    gal_acq::TrkShared* gsh_;
    Channel ch_[kMaxCh]{};
    bool on_[kMaxCh]{};
    gal_trk::Channel gal_[gal_trk::kMaxTrk]{};
    bool gon_[gal_trk::kMaxTrk]{};
    gal_trk::Params gp_{};
    int max_gal_{0};
    int gal_rr_{0};
    int pool_budget_{kPoolBudget};
    int gal_cost_{kGalCost};
    /* a GPS start overfilled the pool: free the weakest Galileo channels
     * (one waiting for a search first) */
    GAL_COLD void make_room() {
        while (pool_used() > pool_budget_) {
            int w = -1;
            for (int i = 0; i < gal_trk::kMaxTrk; i++) {
                if (!gal_[i].in_use()) continue;
                if (w < 0) {
                    w = i;
                    continue;
                }
                const bool si = gal_[i].wants_search(), sw = gal_[w].wants_search();
                if ((si && !sw) || (si == sw && gal_[i].cn0() < gal_[w].cn0())) w = i;
            }
            if (w < 0) return;
            gal_[w].release(gal_acq::TR_POOL);
            if (gsh_) gsh_->preempted = static_cast<uint16_t>(gsh_->preempted + 1);
        }
    }
    uint32_t lut_[128 * kGroupBlk]; /* 8 blocks x 8 nibbles x 16 */
    uint32_t sblk_[kGroupBlk]{};
    const uint32_t* cs_;
    LoopParams lp_{};
    int max_active_{8};
    uint8_t active_{0};
    uint64_t next_{0};
    uint64_t last_meas_s_{~0ull};
    uint32_t nbuf_{0};
    struct Spare {
        uint8_t prn;
        float code_phase, doppler;
    };
    static constexpr uint64_t kSpareMaxAge = 60ull * kFs;
    Spare spare_[32]{};
    uint8_t n_spare_{0}, next_spare_{0};
    uint64_t spare_at_{0};

    /* once a second: report released channels, then give failed channels to
     * the spare detections (their code phase propagated from the search) */
    void releases_and_refill(uint64_t now) {
        for (int i = 0; i < kMaxCh; i++)
            if (ch_[i].take_release()) {
                const uint16_t k = sh_->released;
                sh_->rel_prn[k & 7u] = ch_[i].prn();
                const uint32_t ms = ch_[i].epochs();
                sh_->rel_ms[k & 7u] = static_cast<uint16_t>(ms > 65535u ? 65535u : ms);
                sh_->released = static_cast<uint16_t>(k + 1);
            }
        if (now - spare_at_ > kSpareMaxAge) next_spare_ = n_spare_; /* too old to propagate: a new search finds them */
        while (next_spare_ < n_spare_) {
            bool busy = false; /* never two channels on one PRN */
            const Spare& s = spare_[next_spare_];
            for (int i = 0; i < kMaxCh; i++)
                if (ch_[i].prn() == s.prn && ch_[i].state() != TRK_IDLE && ch_[i].state() != TRK_FAILED) busy = true;
            if (busy) {
                next_spare_++;
                continue;
            }
            int free = -1;
            for (int i = 0; i < max_active_ && free < 0; i++)
                if (ch_[i].state() == TRK_IDLE || ch_[i].state() == TRK_FAILED) free = i;
            if (free < 0) break;
            start(s.prn, s.code_phase, s.doppler, spare_at_, now); /* frees Galileo channels if the pool is full */
            next_spare_++;
            sh_->refills = static_cast<uint16_t>(sh_->refills + 1);
        }
    }

    void measure(uint64_t at) {
        const uint32_t seq = sh_->meas_seq;
        MeasSet& m = sh_->meas[seq & 1u];
        m.seq = seq + 1;
        m.sample = at;
        m.missed = sh_->missed;
        int n = 0;
        for (int i = 0; i < kMaxCh; i++) {
            const Channel& c = ch_[i];
            if (c.state() != TRK_TRACK) continue;
            MeasCh& o = m.ch[n];
            o.prn = c.prn();
            o.flags = c.flags();
            o.cn0_x10 = static_cast<uint16_t>(c.cn0() * 10.0f + 0.5f);
            o.doppler_hz = static_cast<float>(ch_[i].take_mean_doppler());
            o.lock_ms = c.track_ms();
            o.tx_ms = o.tx_frac = 0;
            if (!c.tx_time(&o.tx_ms, &o.tx_frac)) o.flags &= static_cast<uint8_t>(~F_FRAME);
            n++;
        }
        m.n = static_cast<uint8_t>(n);
        if (gsh_) measure_gal(seq + 1, at); /* complete before the GPS counter moves */
        mem_barrier();
        sh_->meas_seq = seq + 1;
    }
    GAL_COLD void measure_gal(uint32_t seq, uint64_t at) {
        const uint32_t gs = gsh_->meas_seq;
        gal_acq::MeasSet& g = gsh_->meas[gs & 1u];
        g.seq = seq;
        g.sample = at;
        for (int i = 0; i < gal_trk::kMaxTrk; i++) gal_[i].meas(g.ch[i]);
        mem_barrier();
        gsh_->meas_seq = gs + 1;
    }
};

/* The tracking image's Galileo searcher between the app and the Galileo
 * channels: the app's aided searches (CMD_GAL; a detection goes to a channel
 * when the app asks for tracking and the pool has room) and the
 * re-acquisition searches of lost channels (around their propagated code
 * phase; one at most every kReacqGap, so the app's searches get turns). The
 * firmware and the host tests run this same code. */
class GalService {
   public:
    explicit GalService(gal_acq::Searcher& s)
        : s_{s} {}
    static constexpr uint64_t kReacqGap = kFs; /* samples between re-acquisition searches */
    static constexpr int kReacqHalfWin = 16;   /* samples either side of the propagated code phase */
    /* the app's search: preempts a re-acquisition search (that channel retries) */
    GAL_COLD void app_search(int prn, const gal_acq::Codes* codes, double code_phase, uint64_t at, double doppler, int half_win, bool track) {
        s_.start(prn, codes, code_phase, at, doppler, half_win, 1, 100.0f, 8);
        owner_ = kApp;
        track_ = track;
    }
    void stop() {
        s_.stop();
        owner_ = kNone;
    }
    /* After Tracker::process(iq, first) for one buffer of n samples: capture,
     * or work units while more() says one fits (the M4 time the loops leave). */
    template <class More>
    __attribute__((noinline)) void run(Tracker* t, const int8_t* iq, int n, uint64_t first, More more) {
        const uint64_t now = first + static_cast<uint64_t>(n);
        if (!s_.busy() && t && now >= reacq_after_) {
            const int k = t->gal_reacq_next();
            if (k >= 0) {
                const gal_trk::Channel& g = t->gal(k);
                s_.load_streams(g.stream(0), g.stream(1));
                s_.start(g.prn(), nullptr, g.code_phase_chips(), now, g.search_doppler(), kReacqHalfWin, 1, 100.0f, 8);
                owner_ = k;
            }
        }
        if (!s_.busy()) return;
        if (s_.wants_samples()) {
            s_.put(iq, n, first);
            return;
        }
        while (!s_.wants_samples() && more())
            if (!s_.step()) {
                done(t, now);
                break;
            }
    }
    bool busy() const { return s_.busy(); }
    bool app_busy() const { return s_.busy() && owner_ == kApp; }
    uint16_t seq() const { return seq_; } /* the app's searches finished */
    const gal_acq::Result& result() const { return app_res_; }
    uint8_t last_handover() const { return last_ho_; }

   private:
    static constexpr int kNone = -2, kApp = -1;
    gal_acq::Searcher& s_;
    int owner_{kNone};
    bool track_{false};
    uint16_t seq_{0};
    uint8_t last_ho_{0};
    uint64_t reacq_after_{0};
    gal_acq::Result app_res_{};

    GAL_COLD void done(Tracker* t, uint64_t now) {
        const gal_acq::Result& r = s_.result();
        const uint64_t at = s_.block_sample();
        if (owner_ == kApp) {
            app_res_ = r;
            uint8_t ho = gal_acq::HO_OFF;
            if (t && track_) {
                if (!r.detected)
                    ho = gal_acq::HO_NOT_DETECTED;
                else
                    ho = t->gal_start(r.prn, s_.stream(0), s_.stream(1), r.code_phase, r.doppler, at, now);
            }
            last_ho_ = ho;
            if (t && t->gal_shared()) {
                t->gal_shared()->last_ho_prn = r.prn;
                t->gal_shared()->last_ho_result = ho;
            }
            seq_++;
        } else if (owner_ >= 0 && t) {
            t->gal_reacq_done(owner_, r, at, now);
            reacq_after_ = now + kReacqGap;
        }
        owner_ = kNone;
    }
};

}  // namespace gps_trk

#endif /*__GPS_TRK_H__*/
