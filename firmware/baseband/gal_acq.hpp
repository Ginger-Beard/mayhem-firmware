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

#ifndef __GAL_ACQ_H__
#define __GAL_ACQ_H__

/* Galileo E1 aided search (header only, host-tested; for the GPS app's
 * tracking image). Receive only.
 *
 * Once the GPS side has a fix, a Galileo satellite's code phase is known to
 * a few chips and its Doppler to tens of Hz (the app predicts both from the
 * orbit, the position and GPS time at a sample of this image's clock). The
 * search then needs only a narrow window: one 4 ms block of samples (one E1
 * code period) is kept, the carrier wiped for each Doppler bin, and the E1B
 * (data) and E1C (pilot) BOC(1,1) replicas correlated at each sample offset
 * of the window: 4 ms coherent per component, |B|^2 + |C|^2, blocks summed
 * non-coherently. Detection: peak / mean of all cells above a threshold
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

namespace gal_acq {

constexpr int kHalf = 2 * kChips;
constexpr int kN = 8192; /* samples per 4 ms block at 2.048 Msps */
constexpr double kFs = 2048000.0;
constexpr double kL1 = 1575.42e6;
constexpr int kMaxHalfWin = 24; /* samples either side (~12 chips) */
constexpr int kMaxBins = 5;

inline int code_bit(const uint8_t* code, int chip) {
    return (code[chip >> 3] >> (7 - (chip & 7))) & 1;
}

class Searcher {
   public:
    /* peak / mean threshold: 1.2 x the largest of 220 noise trials (8 blocks, +-16
     * samples, 3 bins) on the outdoor test recording (rf_test/gps, run_gps_real_test) */
    static constexpr float kThreshold = 3.0f;
    float threshold{0}; /* 0: kThreshold (every member starts at 0: the object stays in .bss) */

    /* a new search: PRN, codes, predicted code phase (chips) at sample
     * `at`, predicted Doppler (Hz, incl. the receiver clock), window half
     * width (samples), Doppler bins either side and their step, blocks */
    void start(int prn, const Codes* codes, double code_phase_at, uint64_t at, double doppler, int half_win, int bins_each_side, float bin_hz, int blocks) {
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
        bin_ = 0;
        phase_ = CAPTURE;
        std::memset(power_, 0, sizeof(power_));
        res_ = Result{};
        res_.prn = static_cast<uint8_t>(prn);
    }
    bool busy() const { return phase_ != IDLE; }
    bool wants_samples() const { return phase_ == CAPTURE; }
    const Result& result() const { return res_; }
    void stop() { phase_ = IDLE; }

    /* samples of this image's clock: the block starts with the first buffer
     * given while capturing (first_sample: its index) */
    void put(const int8_t* iq, int n, uint64_t first_sample) {
        if (phase_ != CAPTURE) return;
        if (!filled_) s0_ = first_sample;
        const int take = n < kN - filled_ ? n : kN - filled_;
        std::memcpy(raw_ + 2 * filled_, iq, static_cast<size_t>(2 * take));
        filled_ += take;
        if (filled_ == kN) {
            phase_ = WIPE;
            bin_ = 0;
            unit_ = 0;
        }
    }

    /* one unit of work, each ~40k M4 cycles or less (gcc -O3: ~14 cycles per
     * sample in the correlation): a quarter of a bin's carrier wipe, a quarter
     * of one replica, or a quarter of the samples of one component at one
     * offset. False when done. */
    bool step() {
        switch (phase_) {
            case WIPE:
                wipe_part(unit_);
                if (++unit_ == 4) {
                    phase_ = REP;
                    unit_ = 0;
                }
                return true;
            case REP: /* 4 replicas in 4 parts each */
                replica((unit_ >> 3) & 1, (unit_ >> 2) & 1, unit_ & 3);
                if (++unit_ == 16) {
                    phase_ = CORR;
                    unit_ = 0;
                }
                return true;
            case CORR: {
                /* unit: offset, component, quarter of the samples */
                const int d2 = unit_ / 8 - 2 * hw_, comp = (unit_ >> 2) & 1, quarter = unit_ & 3;
                correlate(d2, comp, quarter);
                if (++unit_ < 8 * (4 * hw_ + 1)) return true;
                unit_ = 0;
                if (++bin_ < nb_) {
                    phase_ = WIPE;
                    return true;
                }
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
                           CAPTURE,
                           WIPE,
                           REP,
                           CORR };
    Phase phase_{IDLE};
    int prn_{0};
    const Codes* codes_{nullptr};
    double cp_at_{0};
    uint64_t at_{0}, s0_{0};
    double fd0_{0};
    int hw_{0}, nb_{0}, blocks_{0}, block_{0}, bin_{0}, filled_{0}, unit_{0};
    float bin_hz_{0};
    Result res_{};
    int8_t raw_[2 * kN]{};
    int8_t y_[2 * kN]{}; /* this bin's carrier-wiped samples */
    int32_t sum_i_{0}, sum_q_{0};
    int32_t ni_{0}, nq_{0};            /* the sums of the quarters so far */
    uint32_t rep_hi_{0}, rep_frac_{0}; /* replica phase between its parts */
    float rot_c_{0}, rot_s_{0};
    /* replica bits (1 = -1) for samples -hw .. kN + hw: [component][half-sample shift] */
    uint32_t rep_[2][2][(kN + 2 * kMaxHalfWin + 31) / 32 + 2]{};
    float part_[2]{}; /* this offset's |E1B|^2 (the E1C one completes it) */
    /* power per bin and offset in half samples (-2 hw .. 2 hw) */
    float power_[kMaxBins][4 * kMaxHalfWin + 1]{};

    double bin_hz_at(int b) const { return fd0_ + (b - nb_ / 2) * bin_hz_; }

    /* the carrier at each 16-sample group's centre (|fd| < 6 kHz: < 0.1 dB),
     * by a float rotation; samples scaled to stay within int8 */
    void wipe_part(int q) {
        const float w = static_cast<float>(-2.0 * 3.14159265358979 * bin_hz_at(bin_) / kFs);
        const int k_begin = q * (kN / 4), k_end = k_begin + kN / 4;
        if (q == 0) {
            rot_c_ = std::cos(8.0f * w);
            rot_s_ = std::sin(8.0f * w);
            sum_i_ = sum_q_ = 0;
        }
        const float dc = std::cos(16.0f * w), ds = std::sin(16.0f * w);
        for (int k0 = k_begin; k0 < k_end; k0 += 16) {
            const int c = static_cast<int>(rot_c_ * 180.0f), s = static_cast<int>(rot_s_ * 180.0f);
            for (int k = k0; k < k0 + 16; k++) {
                const int i = raw_[2 * k], qq = raw_[2 * k + 1];
                const int yi = (i * c - qq * s) >> 8, yq = (i * s + qq * c) >> 8;
                y_[2 * k] = static_cast<int8_t>(yi);
                y_[2 * k + 1] = static_cast<int8_t>(yq);
                sum_i_ += yi;
                sum_q_ += yq;
            }
            const float nc = rot_c_ * dc - rot_s_ * ds, ns = rot_c_ * ds + rot_s_ * dc;
            rot_c_ = nc;
            rot_s_ = ns;
            if ((k0 & 1023) == 0) { /* keep the rotation on the unit circle */
                const float m = 1.5f - 0.5f * (rot_c_ * rot_c_ + rot_s_ * rot_s_);
                rot_c_ *= m;
                rot_s_ *= m;
            }
        }
    }

    /* one replica: the nearest half chip per sample by an integer phase
     * accumulator (2^-32 half chips), BOC(1,1): + then - within a chip */
    void replica(int comp, int sh, int part) {
        const double fd = bin_hz_at(bin_);
        const double rate = 2.0 * kChips / kN * (1.0 + fd / kL1); /* half chips per sample */
        const uint32_t step_frac = static_cast<uint32_t>((rate - std::floor(rate)) * 4294967296.0);
        const uint32_t step_int = static_cast<uint32_t>(rate);
        if (part == 0) { /* the phase at the window's first sample */
            const double dt = (static_cast<double>(s0_) - static_cast<double>(at_)) / kFs;
            double h = 2.0 * (cp_at_ + dt * 1.023e6 * (1.0 + fd / kL1)) - hw_ * rate + (sh ? 0.5 * rate : 0.0);
            h = std::fmod(h, static_cast<double>(kHalf));
            if (h < 0) h += kHalf;
            rep_hi_ = static_cast<uint32_t>(h);
            rep_frac_ = static_cast<uint32_t>((h - rep_hi_) * 4294967296.0);
        }
        uint32_t hi = rep_hi_, frac = rep_frac_;
        const uint8_t* code = comp ? codes_->c : codes_->b;
        uint32_t* r = rep_[comp][sh];
        const int n_bits = kN + 2 * hw_;
        const int n_words = (n_bits + 31) / 32;
        const int w_begin = part * ((n_words + 3) / 4), w_end = part == 3 ? n_words : (part + 1) * ((n_words + 3) / 4);
        for (int w = w_begin; w < w_end; w++) {
            uint32_t word = 0;
            for (int b = 0; b < 32; b++) {
                const uint32_t bit = static_cast<uint32_t>(code_bit(code, static_cast<int>(hi >> 1)) ^ (hi & 1u));
                word |= bit << b;
                const uint32_t f2 = frac + step_frac;
                hi += step_int + (f2 < frac ? 1u : 0u);
                frac = f2;
                if (hi >= static_cast<uint32_t>(kHalf)) hi -= kHalf;
            }
            r[w] = word;
        }
        rep_hi_ = hi;
        rep_frac_ = frac;
        if (part == 3) r[n_words] = 0;
    }

    /* one component at one offset (half samples), a quarter of the samples:
     * the sums over the samples whose replica chip is -1, then sum - 2 x that */
    void correlate(int d2, int comp, int quarter) {
        const int d = d2 >= 0 ? d2 / 2 : -((-d2 + 1) / 2);
        const int sh = d2 & 1;
        const uint32_t* r = rep_[comp][sh];
        const int base = d + hw_; /* bit index of sample 0 */
        const int wsh = base & 31;
        if (!quarter) ni_ = nq_ = 0;
        int32_t ni = ni_, nq = nq_;
        const int k_begin = quarter * (kN / 4), k_end = k_begin + kN / 4;
        const int8_t* y = y_ + 2 * k_begin;
        for (int k = k_begin; k < k_end; k += 32) {
            const int wi = (base + k) >> 5;
            uint32_t w = wsh ? (r[wi] >> wsh) | (r[wi + 1] << (32 - wsh)) : r[wi];
            for (int b = 0; b < 32; b++) {
                const int32_t m = -static_cast<int32_t>(w & 1u);
                ni += y[0] & m;
                nq += y[1] & m;
                y += 2;
                w >>= 1;
            }
        }
        ni_ = ni;
        nq_ = nq;
        if (quarter != 3) return;
        const float ci = static_cast<float>(sum_i_ - 2 * ni), cq = static_cast<float>(sum_q_ - 2 * nq);
        const float p = ci * ci + cq * cq;
        if (comp == 0)
            part_[0] = p;
        else
            power_[bin_][d2 + 2 * hw_] += part_[0] + p;
    }

    void finish() {
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
