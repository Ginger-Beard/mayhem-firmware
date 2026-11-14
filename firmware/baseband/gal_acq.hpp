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

/* the GPS tracker's per-block tables (build_block_lut, lut_sum) and SIMD
 * helpers; outside the guard: gps_trk.hpp includes this file again (through
 * gal_trk.hpp) after its helpers, whichever of the two comes first */
#include "gps_trk.hpp"

#ifndef __GAL_ACQ_H__
#define __GAL_ACQ_H__

/* Galileo E1 aided search (header only, host-tested; for the GPS app's
 * tracking image). Receive only.
 *
 * Once the GPS side has a fix, a Galileo satellite's code phase is known to
 * a few chips and its Doppler to tens of Hz (the app predicts both from the
 * orbit, the position and GPS time at a sample of this image's clock). The
 * search then needs only a narrow window: one 4 ms block of samples (one E1
 * code period) is kept and the E1B (data) and E1C (pilot) BOC(1,1) replicas
 * are correlated at each half-sample offset of the window for each Doppler
 * bin: 4 ms coherent per component, |B|^2 + |C|^2, blocks summed
 * non-coherently. The correlation is the GPS tracker's engine: per 32
 * samples one table of nibble sums (shared by every offset, component and
 * bin), a 32-bit replica word per offset and component (sum = S - 2 x 8
 * look-ups), the carrier of each bin wiped on the block sums at the block
 * centre (< 0.1 dB at 5 kHz). The replicas follow the centre bin's code rate
 * (the other bins' differ by 1e-7 chip over 4 ms). Detection: peak / mean of all cells above a threshold
 * calibrated on noise (host test), then the peak's neighbours are compared
 * (the BOC side peaks at +-0.75 chip are only ~3.8 dB down: the higher of
 * the peak and its +-1.5-sample neighbours wins, and how much higher is
 * reported).
 *
 * Replica: the nearest half chip per sample (8184 half chips per 8192
 * samples), as the GPS tracker's code; host measurement on a real recording:
 * ~1 dB below the exact band-limited replica.
 *
 * Codes: 4092 chips, MSB first, one bit per chip, from /GPS/GALE1.BIN (the
 * app reads 1 KB per satellite: E1B then E1C). */

#include <cmath>
#include <cstdint>
#include <cstring>

#include "gal_e1.hpp"

/* cold code (once per search step or per 4 ms epoch, not per sample): small
 * rather than fast, the tracking image's code region is the constraint */
#if !defined(GAL_COLD)
#define GAL_COLD __attribute__((noinline, optimize("Os")))
#endif

namespace gal_acq {

constexpr int kHalf = 2 * kChips;
constexpr int kN = 8192; /* samples per 4 ms block at 2.048 Msps */
constexpr double kFs = 2048000.0;
constexpr double kL1 = 1575.42e6;
constexpr int kMaxHalfWin = 24; /* samples either side (~12 chips) */
constexpr int kMaxBins = 5;
/* A code as the BOC(1,1) half-chip stream the correlators read (search and
 * tracking): bit b (LSB first) = half chip (b - 2) mod kHalf, 1 = -1; half chip
 * h = chip h / 2 XOR (h & 1) (+ then - within a chip). The two leading half
 * chips and the wrapped tail let a 64-bit window at any prompt index reach
 * two half chips either side. */
constexpr int kStreamWords = 258; /* 8256 bits: 8184 + 2 before + 70 wrapped */
constexpr int kStreamLead = 2;

inline int code_bit(const uint8_t* code, int chip) {
    return (code[chip >> 3] >> (7 - (chip & 7))) & 1;
}

/* Words [w_begin, w_end) of a code's half-chip stream (~160 cycles per word). */
inline void expand_stream(const uint8_t* code, uint32_t* hc, int w_begin, int w_end) {
    for (int w = w_begin; w < w_end; w++) {
        int h = 32 * w - kStreamLead; /* even: the stream starts on a chip boundary */
        if (h < 0) h += kHalf;
        uint32_t word = 0;
        for (int k = 0; k < 32; k += 2) {
            const uint32_t c = static_cast<uint32_t>(code_bit(code, h >> 1));
            word |= (c | ((c ^ 1u) << 1)) << k;
            h += 2;
            if (h >= kHalf) h -= kHalf;
        }
        hc[w] = word;
    }
}

class Searcher {
   public:
    /* peak / mean threshold: 1.2 x the largest of 220 noise trials (8 blocks, +-16
     * samples, 3 bins) on the outdoor test recording (rf_test/gps, run_gps_real_test) */
    static constexpr float kThreshold = 3.0f;
    float threshold{0}; /* 0: kThreshold (every member starts at 0: the object stays in .bss) */

    /* a new search: PRN, codes, predicted code phase (chips) at sample
     * `at`, predicted Doppler (Hz, incl. the receiver clock), window half
     * width (samples), Doppler bins either side and their step, blocks.
     * codes == nullptr: the half-chip streams are already loaded
     * (load_streams: a tracking channel's re-acquisition). */
    GAL_COLD void start(int prn, const Codes* codes, double code_phase_at, uint64_t at, double doppler, int half_win, int bins_each_side, float bin_hz, int blocks) {
        prn_ = prn;
        codes_ = codes;
        cp_at_ = code_phase_at;
        at_ = at;
        fd0_ = doppler;
        hw_ = half_win > kMaxHalfWin ? kMaxHalfWin : (half_win < 2 ? 2 : half_win);
        nb_ = 2 * (bins_each_side > (kMaxBins - 1) / 2 ? (kMaxBins - 1) / 2 : bins_each_side) + 1;
        bin_hz_ = bin_hz;
        blocks_ = blocks < 1 ? 1 : blocks;
        block_ = 0;
        filled_ = 0;
        unit_ = 0;
        phase_ = codes ? EXPAND : CAPTURE; /* the streams first (a few units), then the samples */
        std::memset(power_, 0, sizeof(power_));
        res_ = Result{};
        res_.prn = static_cast<uint8_t>(prn);
    }
    bool busy() const { return phase_ != IDLE; }
    int phase() const { return phase_; } /* diagnostics (the M4 bench): 1 codes, 2 capture, 3 replica, 4 correlation */
    bool wants_samples() const { return phase_ == CAPTURE; }
    const Result& result() const { return res_; }
    void stop() { phase_ = IDLE; }
    /* the sample Result::code_phase refers to (first sample of the last block) */
    uint64_t block_sample() const { return s0_; }
    /* the codes as half-chip streams (comp 0 E1B, 1 E1C): what a tracking
     * channel takes over after a detection */
    const uint32_t* stream(int comp) const { return hc_[comp & 1]; }
    void load_streams(const uint32_t* b, const uint32_t* c) {
        std::memcpy(hc_[0], b, sizeof(hc_[0]));
        std::memcpy(hc_[1], c, sizeof(hc_[1]));
    }

    /* samples of this image's clock: the block starts with the first buffer
     * given while capturing (first_sample: its index) */
    void put(const int8_t* iq, int n, uint64_t first_sample) {
        if (phase_ != CAPTURE) return;
        if (!filled_) s0_ = first_sample;
        const int take = n < kN - filled_ ? n : kN - filled_;
        std::memcpy(raw_ + 2 * filled_, iq, static_cast<size_t>(2 * take));
        filled_ += take;
        if (filled_ == kN) {
            phase_ = REP;
            unit_ = 0;
        }
    }

    /* one unit of work, each ~15k M4 cycles or less (rf_test/gps m4bench_trk
     * GALSEARCH): an eighth of the codes' streams, one replica, or the
     * correlation of kBlkUnit 32-sample blocks (every offset, component and
     * bin). False when done. */
    static constexpr int kBlkUnit = 1;
    __attribute__((noinline)) bool step() {
        switch (phase_) {
            case EXPAND: { /* 8 units: E1B, E1C, a quarter of the stream each */
                const int comp = unit_ >> 2, q = unit_ & 3, per = (kStreamWords + 3) / 4;
                const int w1 = (q + 1) * per < kStreamWords ? (q + 1) * per : kStreamWords;
                expand_stream(comp ? codes_->c : codes_->b, hc_[comp], q * per, w1);
                if (++unit_ == 8) {
                    unit_ = 0;
                    phase_ = CAPTURE;
                }
                return true;
            }
            case REP: /* the block's carrier and code phases, then 4 replicas (component x half-sample shift) */
                if (unit_ == 0)
                    block_setup();
                else
                    replica((unit_ - 1) >> 1, (unit_ - 1) & 1);
                if (++unit_ == 5) {
                    phase_ = CORR;
                    unit_ = 0;
                }
                return true;
            case CORR: {
                if (unit_ < kN / 32) { /* the blocks, then the powers in a unit of their own */
                    for (int k = 0; k < kBlkUnit && unit_ < kN / 32; k++) corr_block(unit_++);
                    return true;
                }
                corr_end();
                unit_ = 0;
                if (++block_ < blocks_) {
                    filled_ = 0;
                    phase_ = CAPTURE;
                    return true;
                }
                finish();
                phase_ = IDLE;
                return false;
            }
            default:
                return false;
        }
    }

   private:
    enum Phase : uint8_t { IDLE,
                           EXPAND,
                           CAPTURE,
                           REP,
                           CORR };
    Phase phase_{IDLE};
    int prn_{0};
    const Codes* codes_{nullptr};
    double cp_at_{0};
    uint64_t at_{0}, s0_{0};
    double fd0_{0};
    int hw_{0}, nb_{0}, blocks_{0}, block_{0}, filled_{0}, unit_{0};
    float bin_hz_{0};
    Result res_{};
    uint32_t hc_[2][kStreamWords]{}; /* E1B, E1C half-chip streams */
    int8_t raw_[2 * kN]{};
    /* replica bits (1 = -1) for samples -hw .. kN + hw: [component][half-sample shift] */
    uint32_t rep_[2][2][(kN + 2 * kMaxHalfWin + 31) / 32 + 2]{};
    /* this block's correlations: [bin][offset (-2 hw .. 2 hw half samples)][component][I, Q] */
    int32_t acc_[kMaxBins][4 * kMaxHalfWin + 1][2][2]{};
    uint32_t lut_[128]{};                                                                   /* one 32-sample block's nibble sums */
    float rot_c_[kMaxBins]{}, rot_s_[kMaxBins]{}, step_c_[kMaxBins]{}, step_s_[kMaxBins]{}; /* carrier at the block centre per bin */
    uint32_t rep_m_[2]{}, rep_f_[2]{}, rep_delta_{0};                                       /* replica phase at the window's first sample, rate (1 - delta) */
    /* power per bin and offset in half samples (-2 hw .. 2 hw) */
    float power_[kMaxBins][4 * kMaxHalfWin + 1]{};

    double bin_hz_at(int b) const { return fd0_ + (b - nb_ / 2) * bin_hz_; }

    /* the soft-double part of a block, in one unit: the code phase of the
     * window's first sample for both half-sample shifts, the replica rate, the
     * carrier of each bin at the first block centre and per block */
    GAL_COLD void block_setup() {
        const double fd = fd0_;
        const double rate = 2.0 * kChips / kN * (1.0 + fd / kL1); /* half chips per sample, < 1 */
        rep_delta_ = static_cast<uint32_t>((1.0 - rate) * 4294967296.0);
        const double dt = (static_cast<double>(s0_) - static_cast<double>(at_)) / kFs;
        for (int sh = 0; sh < 2; sh++) {
            double h = 2.0 * (cp_at_ + dt * 1.023e6 * (1.0 + fd / kL1)) - hw_ * rate + (sh ? 0.5 * rate : 0.0);
            h = std::fmod(h, static_cast<double>(kHalf));
            if (h < 0) h += kHalf;
            uint32_t m = static_cast<uint32_t>(h);
            rep_f_[sh] = static_cast<uint32_t>((h - m) * 4294967296.0);
            rep_m_[sh] = m >= static_cast<uint32_t>(kHalf) ? 0 : m;
        }
        /* half a block of carrier: < 0.3 rad at 6 kHz, so a short series is exact in float */
        auto cs = [](float x, float* c, float* sn) {
            const float x2 = x * x;
            *c = 1.0f - x2 * (0.5f - x2 * (1.0f / 24.0f - x2 * (1.0f / 720.0f)));
            *sn = x * (1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f - x2 * (1.0f / 5040.0f))));
        };
        float c, sn, bc, bs;
        cs(static_cast<float>(2.0 * 3.14159265358979 * 16 / kFs * bin_hz_at(0)), &c, &sn);
        cs(static_cast<float>(2.0 * 3.14159265358979 * 16 / kFs) * bin_hz_, &bc, &bs);
        for (int b = 0; b < nb_; b++) {
            rot_c_[b] = c;
            rot_s_[b] = sn;
            step_c_[b] = c * c - sn * sn; /* 32 samples = twice the half block */
            step_s_[b] = 2.0f * c * sn;
            const float nc = c * bc - sn * bs, ns = c * bs + sn * bc;
            c = nc;
            sn = ns;
        }
        for (int b = 0; b < nb_; b++) /* only what is used (newlib's memset goes byte by byte) */
            for (int i = 0; i < 4 * hw_ + 1; i++) acc_[b][i][0][0] = acc_[b][i][0][1] = acc_[b][i][1][0] = acc_[b][i][1][1] = 0;
    }

    /* one replica: the nearest half chip per sample, 32 samples per word read
     * from the half-chip stream (the half-chip phase repeats once per ~1024
     * samples: that word is split with a mask, as in the tracker) */
    __attribute__((noinline)) void replica(int comp, int sh) {
        const uint32_t delta = rep_delta_, d32 = delta * 32u;
        uint32_t m = rep_m_[sh], f = rep_f_[sh];
        const uint32_t* hc = hc_[comp];
        uint32_t* r = rep_[comp][sh];
        const int n_words = (kN + 2 * hw_ + 31) / 32;
        for (int w = 0; w < n_words; w++) {
            const uint32_t b = m + kStreamLead;
            const uint32_t* d = hc + (b >> 5);
            const uint32_t s5 = b & 31u;
            uint32_t word = s5 ? (d[0] >> s5) | (d[1] << (32 - s5)) : d[0];
            uint32_t adv = 32;
            if (f < d32) { /* the phase repeats after sample j */
                const uint32_t j = f / delta;
                adv = 31;
                if (j <= 30) {
                    const uint32_t lo = (2u << j) - 1u;
                    word = (word & lo) | ((word << 1) & ~lo);
                }
            }
            r[w] = word;
            f -= d32;
            m += adv;
            if (m >= static_cast<uint32_t>(kHalf)) m -= kHalf;
        }
        r[n_words] = r[n_words + 1] = 0;
    }

    /* one 32-sample block: its table, then every offset, component and bin */
    void corr_block(int k) {
        const uint32_t s = gps_trk::build_block_lut(raw_ + 64 * k, lut_);
        uint32_t cs[kMaxBins];
        for (int b = 0; b < nb_; b++) { /* amplitude 511: 256 blocks of full-scale sums stay inside int32 */
            const int32_t c = static_cast<int32_t>(rot_c_[b] * 511.0f), sn = static_cast<int32_t>(rot_s_[b] * 511.0f);
            cs[b] = (static_cast<uint32_t>(c) & 0xFFFFu) | (static_cast<uint32_t>(sn) << 16);
            const float nc = rot_c_[b] * step_c_[b] - rot_s_[b] * step_s_[b], ns = rot_c_[b] * step_s_[b] + rot_s_[b] * step_c_[b];
            rot_c_[b] = nc;
            rot_s_[b] = ns;
        }
        if ((k & 63) == 63)
            for (int b = 0; b < nb_; b++) { /* back onto the unit circle */
                const float g = 1.5f - 0.5f * (rot_c_[b] * rot_c_[b] + rot_s_[b] * rot_s_[b]);
                rot_c_[b] *= g;
                rot_s_[b] *= g;
            }
        const int nd = 4 * hw_ + 1;
        for (int comp = 0; comp < 2; comp++)
            for (int i = 0; i < nd; i++) {
                const int d2 = i - 2 * hw_;
                const int d = d2 >= 0 ? d2 / 2 : -((-d2 + 1) / 2);
                const uint32_t* r = rep_[comp][d2 & 1];
                const int bit = d + hw_ + 32 * k; /* the replica bit of the block's first sample */
                const int wi = bit >> 5, ws = bit & 31;
                const uint32_t w = ws ? (r[wi] >> ws) | (r[wi + 1] << (32 - ws)) : r[wi];
                const uint32_t t = gps_trk::lut_sum(lut_, w);
                const uint32_t cp = gps_trk::ssub16(gps_trk::ssub16(s, t), t);
                for (int b = 0; b < nb_; b++) {
                    int32_t* a = acc_[b][i][comp];
                    a[0] = gps_trk::smlad(cp, cs[b], a[0]);
                    a[1] = gps_trk::smlsdx(cp, cs[b], a[1]);
                }
            }
    }

    /* the block's powers: |E1B|^2 + |E1C|^2 per bin and offset */
    GAL_COLD void corr_end() {
        const int nd = 4 * hw_ + 1;
        for (int b = 0; b < nb_; b++)
            for (int i = 0; i < nd; i++) {
                float p = 0;
                for (int comp = 0; comp < 2; comp++) {
                    const float x = static_cast<float>(acc_[b][i][comp][0]), y = static_cast<float>(acc_[b][i][comp][1]);
                    p += x * x + y * y;
                }
                power_[b][i] += p;
            }
    }

    GAL_COLD void finish() {
        const int nd = 4 * hw_ + 1;
        float best = 0, sum = 0;
        int bb = 0, bd = 0;
        for (int b = 0; b < nb_; b++)
            for (int d = 0; d < nd; d++) {
                sum += power_[b][d];
                if (power_[b][d] > best) {
                    best = power_[b][d];
                    bb = b;
                    bd = d;
                }
            }
        /* mean without the peak's neighbourhood (+-3 samples in its bin) */
        float s2 = sum;
        int n2 = nb_ * nd;
        for (int d = bd - 6; d <= bd + 6; d++)
            if (d >= 0 && d < nd) {
                s2 -= power_[bb][d];
                n2--;
            }
        const float mean = n2 > 0 ? s2 / n2 : 1.0f;
        res_.metric = mean > 0 ? best / mean : 0;
        /* BOC side peaks: the main peak must beat its +-1.5-sample neighbours */
        float side = 0;
        static constexpr int kSide[4] = {-4, -3, 3, 4};
        for (int dd : kSide)
            if (bd + dd >= 0 && bd + dd < nd && power_[bb][bd + dd] > side) side = power_[bb][bd + dd];
        res_.side_db = side > 0 ? 10.0f * std::log10(best / side) : 99.0f;
        /* C/N0 of E1 (B + C): per component (peak - noise) / noise = T C/N0 with
         * T = 4 ms (the blocks sum signal and noise alike); the components carry half
         * each. Not corrected for the replica / filter loss (~1 dB + the BOC
         * power outside the 1.75 MHz filter): reads ~3-4 dB under a full estimator. */
        const float snr = mean > 0 ? (best - mean) / mean : 0;
        res_.cn0 = snr > 0 ? 10.0f * std::log10(2.0f * snr / 0.004f) : 0;
        res_.offset = static_cast<int16_t>(bd - 2 * hw_); /* half samples */
        res_.doppler = static_cast<float>(bin_hz_at(bb));
        const double dt = (static_cast<double>(s0_) - static_cast<double>(at_)) / kFs;
        double cp = cp_at_ + dt * 1.023e6 * (1.0 + res_.doppler / kL1) + 0.5 * (bd - 2 * hw_) * kChips / static_cast<double>(kN);
        cp = std::fmod(cp, static_cast<double>(kChips));
        if (cp < 0) cp += kChips;
        res_.code_phase = static_cast<float>(cp);
        res_.blocks = static_cast<uint8_t>(blocks_);
        res_.detected = res_.metric > (threshold > 0 ? threshold : kThreshold) ? 1 : 0;
        res_.done = 1;
    }
};

}  // namespace gal_acq

#endif /*__GAL_ACQ_H__*/
