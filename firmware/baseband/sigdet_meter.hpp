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

/* Track meter of the Signature Detect baseband (PSDT MODE_METER): gap-free
 * channel power at one frequency for fox hunting. Header only, no ChibiOS
 * (host tested).
 *
 * Input: 4 Msps int8 I/Q with the target at +fs/4 (the M0 tunes target as for
 * the BLE and pattern modes, sigdet_dsp.hpp "Where the channel sits"). Each sample
 * is shifted by -fs/4 ((-j)^n, sign swaps only) and summed in a boxcar of D =
 * 2^log2d samples (D = 4, 16, 64, 256): one output per D inputs, the power
 * |y|^2 / D^2 of the channel D wide. The boxcar's -3 dB width is ~0.89 fs / D
 * (890, 220, 55, 14 kHz), its first nulls at +-fs / D, sidelobes -13 dB; the DC
 * residue (-fs/4 after the shift) is always in a null.
 *
 * Every kBlock (1024) input samples (256 us) the outputs' mean power is one
 * "block". Per report interval the meter keeps the strongest block (the pulse
 * peak: a 2 ms burst fills ~8 blocks, a single block of noise is a mean of
 * 4..256 outputs, so noise peaks stay within ~1-4 dB of the mean), the mean
 * over all blocks, and how many raw samples were at |x| >= 120 of 127 (the
 * front end near full scale: CLIP).
 *
 * Pulses: a floor follows the blocks below twice the floor (fast) and, very
 * slowly, everything (so a continuous carrier becomes the floor in ~1-2 s);
 * a block over 4 x the floor (+6 dB) starts a pulse, one under 2 x ends it.
 * Start-to-start intervals go into a ring of 8; the report gives their median,
 * the last one and the last pulse width, to a block (256 us).
 *
 * Hum check (set_hum): every block's power also goes into a mains ladder
 * (sigdet_hum.hpp), which lives in memory lent by the Wi-Fi hand-off
 * (sigdet_wifi_slots.hpp). configure() disconnects it. */

#ifndef __SIGDET_METER_HPP__
#define __SIGDET_METER_HPP__

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sigdet_hum.hpp"

namespace sigdet {

struct MeterReport {
    float peak;              /* strongest block of the interval, power re full scale */
    float mean;              /* mean of the interval's blocks */
    float floor;             /* pulse detector floor at the end */
    uint32_t clip;           /* raw I or Q samples at |x| >= 120 in the interval */
    uint16_t blocks;         /* blocks in the interval */
    uint16_t pulses;         /* pulse starts in the interval */
    uint32_t period_us;      /* median start-to-start of the last <= 8 intervals, 0 = fewer than 2 */
    uint32_t last_period_us; /* the latest one, 0 = none */
    uint32_t width_us;       /* the latest finished pulse, 0 = none */
};

class PowerMeter {
   public:
    static constexpr uint32_t kBlock = 1024;       /* input samples per block */
    static constexpr uint32_t kBlockUs = 256;      /* at 4 Msps */
    static constexpr uint32_t kReportBlocks = 196; /* ~50 ms: 20 reports a second */

    void configure(uint8_t log2d) {
        log2d_ = (log2d < 2) ? 2 : (log2d > 8 ? 8 : (log2d & ~1u));
        d_ = 1u << log2d_;
        si_ = sq_ = 0;
        cnt_ = 0;
        bsum_ = 0;
        bout_ = 0;
        braw_ = 0;
        floor_ = 0.0f;
        on_ = false;
        have_start_ = false;
        nring_ = ring_pos_ = 0;
        blk_ = 0;
        last_period_us_ = width_us_ = 0;
        hum_ = nullptr;
        start_interval();
    }
    uint8_t log2d() const { return log2d_; }
    /* Block powers also into h (nullptr: none). */
    void set_hum(HumLadder* h) { hum_ = h; }

    /* count complex samples (interleaved I, Q), a multiple of 4: the shift's
     * phase restarts with every buffer, which stays continuous as long as
     * every buffer (and every lost one) is a multiple of 4 samples long. */
    void feed(const int8_t* iq, size_t count) {
        /* State in locals and the samples as words: through member variables
         * and int8 pointers (which may alias them) the compiler stored and
         * reloaded the state after every byte, ~29 cycles a sample (the 58 %
         * M4 load measured on the device). */
        int32_t si = si_, sq = sq_;
        uint32_t cnt = cnt_, braw = braw_, clip = clip_;
        const uint32_t d = d_;
        for (size_t n = 0; n + 4 <= count; n += 4, iq += 8) {
            uint32_t w0, w1;
            std::memcpy(&w0, iq, 4);
            std::memcpy(&w1, iq + 4, 4);
            const int32_t i0 = static_cast<int8_t>(w0), q0 = static_cast<int8_t>(w0 >> 8);
            const int32_t i1 = static_cast<int8_t>(w0 >> 16), q1 = static_cast<int8_t>(w0 >> 24);
            const int32_t i2 = static_cast<int8_t>(w1), q2 = static_cast<int8_t>(w1 >> 8);
            const int32_t i3 = static_cast<int8_t>(w1 >> 16), q3 = static_cast<int8_t>(w1 >> 24);
            /* x (-j)^n: (i, q), (q, -i), (-i, -q), (-q, i) */
            si += i0 + q1 - i2 - q3;
            sq += q0 - i1 - q2 + i3;
            /* |x| >= 120 in any byte, four at a time: the ones' complement
             * magnitude (|x| or |x| - 1) + 8 reaches bit 7 from 120 on. */
            const uint32_t h = near_full(w0) | (near_full(w1) << 1);
            if (h) clip += popcount32(h);
            cnt += 4;
            if (cnt >= d) {
                bsum_ += static_cast<uint64_t>(static_cast<int64_t>(si) * si + static_cast<int64_t>(sq) * sq);
                bout_++;
                si = sq = 0;
                cnt = 0;
            }
            braw += 4;
            if (braw >= kBlock) {
                clip_ = clip;
                end_block();
                braw = 0;
                clip = clip_;
            }
        }
        si_ = si;
        sq_ = sq;
        cnt_ = cnt;
        braw_ = braw;
        clip_ = clip;
    }

    /* At the end of each report interval: true and r filled. */
    bool take(MeterReport& r) {
        if (iblocks_ < kReportBlocks) return false;
        r.peak = ipeak_;
        r.mean = isum_ / static_cast<float>(iblocks_);
        r.floor = floor_;
        r.clip = clip_;
        r.blocks = static_cast<uint16_t>(iblocks_);
        r.pulses = ipulses_;
        r.period_us = median_period();
        r.last_period_us = last_period_us_;
        r.width_us = width_us_;
        start_interval();
        return true;
    }

    /* Power re full scale -> centi-dB (a tone of |x| = 128 reads 0 dB;
     * floor -12000). */
    static int16_t to_cdb(float p) {
        if (!(p > 1e-12f)) return -12000;
        const float db = 1000.0f * 0.30103f * fast_log2(p);
        return static_cast<int16_t>(db < -12000.0f ? -12000.0f : (db > 3000.0f ? 3000.0f : db));
    }

    /* log2 to ~3e-5 (0.0001 dB): exponent bits plus a series in the mantissa
     * [1, 2); no libm (its log10f costs ~3 KB of the M4 image). */
    static float fast_log2(float x) {
        uint32_t bits;
        std::memcpy(&bits, &x, sizeof(bits));
        const int32_t e = static_cast<int32_t>((bits >> 23) & 0xFF) - 127;
        bits = (bits & 0x007FFFFFu) | 0x3F800000u;
        float m;
        std::memcpy(&m, &bits, sizeof(m));
        /* log2(m) = 2 / ln 2 atanh(t), t = (m - 1) / (m + 1) <= 1/3: four terms. */
        const float t = (m - 1.0f) / (m + 1.0f);
        const float t2 = t * t;
        const float p = 2.8853901f * t * (1.0f + t2 * (1.0f / 3.0f + t2 * (0.2f + t2 * (1.0f / 7.0f))));
        return static_cast<float>(e) + p;
    }

   private:
    /* Bit 7 of each byte: |x| >= 120 (ones' complement |x| + 8 >= 128). */
    static uint32_t near_full(uint32_t w) {
        const uint32_t neg = ((w & 0x80808080u) >> 7) * 0xFFu;
        return ((w ^ neg) + 0x08080808u) & 0x80808080u;
    }
    static uint32_t popcount32(uint32_t v) {
        uint32_t n = 0;
        while (v) {
            v &= v - 1;
            n++;
        }
        return n;
    }

    uint8_t log2d_{4};
    uint32_t d_{16};
    int32_t si_{0}, sq_{0};
    uint32_t cnt_{0};
    uint64_t bsum_{0};
    uint32_t bout_{0};
    uint32_t braw_{0};
    uint32_t clip_{0};
    float floor_{0.0f};
    bool on_{false};
    bool have_start_{false};
    uint32_t blk_{0};
    uint32_t start_blk_{0};
    uint32_t on_blk_{0};
    uint32_t ring_[8]{};
    uint8_t nring_{0}, ring_pos_{0};
    uint32_t last_period_us_{0}, width_us_{0};
    /* interval */
    float ipeak_{0.0f}, isum_{0.0f};
    uint32_t iblocks_{0};
    uint16_t ipulses_{0};
    HumLadder* hum_{nullptr};

    void start_interval() {
        ipeak_ = 0.0f;
        isum_ = 0.0f;
        iblocks_ = 0;
        ipulses_ = 0;
        clip_ = 0;
    }

    void end_block() {
        /* Mean |y|^2 of the block's outputs, re full scale: |y| <= 128 D per
         * component for a full scale input: a tone of |x| = 128 reads 1 (0 dB). */
        const float scale = 1.0f / (static_cast<float>(d_) * static_cast<float>(d_) * 16384.0f);
        const float b = bout_ ? static_cast<float>(bsum_) * scale / static_cast<float>(bout_) : 0.0f;
        bsum_ = 0;
        bout_ = 0;
        braw_ = 0;
        blk_++;
        if (b > ipeak_) ipeak_ = b;
        isum_ += b;
        iblocks_++;

        if (floor_ <= 0.0f) floor_ = b;
        if (b < 2.0f * floor_)
            floor_ += (b - floor_) * (1.0f / 32.0f);
        else
            floor_ += (b - floor_) * (1.0f / 2048.0f);
        if (!on_ && b > 4.0f * floor_) {
            on_ = true;
            on_blk_ = blk_;
            if (have_start_) {
                last_period_us_ = (blk_ - start_blk_) * kBlockUs;
                ring_[ring_pos_] = last_period_us_;
                ring_pos_ = static_cast<uint8_t>((ring_pos_ + 1) & 7);
                if (nring_ < 8) nring_++;
            }
            have_start_ = true;
            start_blk_ = blk_;
            if (ipulses_ < 0xFFFF) ipulses_++;
        } else if (on_ && b < 2.0f * floor_) {
            on_ = false;
            width_us_ = (blk_ - on_blk_) * kBlockUs;
        }
        if (hum_) hum_->add(b);
    }

    uint32_t median_period() const {
        if (nring_ < 2) return 0;
        uint32_t v[8];
        for (size_t i = 0; i < nring_; i++) v[i] = ring_[i];
        for (size_t i = 1; i < nring_; i++)
            for (size_t j = i; j > 0 && v[j - 1] > v[j]; j--) {
                const uint32_t t = v[j];
                v[j] = v[j - 1];
                v[j - 1] = t;
            }
        return v[nring_ / 2];
    }
};

} /* namespace sigdet */

#endif /*__SIGDET_METER_HPP__*/
