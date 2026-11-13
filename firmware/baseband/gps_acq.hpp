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

#ifndef __GPS_ACQ_H__
#define __GPS_ACQ_H__

/* GPS L1 C/A acquisition for the M4 (PGPR baseband), header only and free of
 * ChibiOS so the host replay test runs this exact code on IQ files.
 *
 * Input: complex int8 at 2.048 Msps (2048 samples = one 1 ms block, which is
 * also one DMA transfer). A snapshot of K ms (K <= 20) is stored 2-bit per
 * component (sign + magnitude, threshold ~1 sigma; ~0.5 dB loss), 20 KiB.
 * Search, one PRN and one Doppler bin per step(): for each 1 ms block, NCO
 * carrier wipe-off -> 2048-pt FFT -> x conj(code FFT) -> IFFT -> |.|^2,
 * summed over the K blocks (1 ms coherent, K non-coherent); peak / mean of
 * every cell searched is the metric (CFAR), compared with kThreshold[K] for
 * 1e-3 false alarms per PRN over 25 bins x 2048 code phases.
 * Detected PRNs are then refined in the time domain (fine Doppler +-250 Hz,
 * code phase where early and late powers balance) for the snapshot fix.
 * The algorithm and its numbers come from the PC prototype
 * (mayhem-build/proto/gps, notes/gps-app-feasibility-2026-11-12.md). */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "gps_rx_result.hpp"

namespace gps_acq {

constexpr uint32_t kFs = 2048000;
constexpr int kN = 2048; /* samples per 1 ms block */
constexpr int kMaxK = 20;
constexpr int kStep = 500; /* Doppler bin, Hz */
constexpr double kChipRate = 1.023e6;
constexpr double kL1 = 1575.42e6;

/* Metric thresholds by K (index K-1): Gamma(K, 1/K) quantile for 1e-3 / 51200. */
constexpr float kThreshold[kMaxK] = {17.7513f, 10.4174f, 7.8178f, 6.4582f, 5.6116f, 5.0287f, 4.6003f,
                                     4.2706f, 4.0080f, 3.7933f, 3.6139f, 3.4616f, 3.3303f, 3.2158f,
                                     3.1149f, 3.0253f, 2.9450f, 2.8726f, 2.8069f, 2.7471f};

struct cf {
    float re, im;
};

/* C/A code (IS-GPS-200 table 3-Ia), chips as +1 / -1. */
inline void ca_code(int prn, int8_t* out) {
    static constexpr uint8_t taps[32][2] = {{2, 6}, {3, 7}, {4, 8}, {5, 9}, {1, 9}, {2, 10}, {1, 8}, {2, 9}, {3, 10}, {2, 3}, {3, 4}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {9, 10}, {1, 4}, {2, 5}, {3, 6}, {4, 7}, {5, 8}, {6, 9}, {1, 3}, {4, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 6}, {2, 7}, {3, 8}, {4, 9}};
    uint16_t g1 = 0x3FF, g2 = 0x3FF; /* bit i = stage i+1 */
    const int a = taps[prn - 1][0] - 1, b = taps[prn - 1][1] - 1;
    for (int i = 0; i < 1023; i++) {
        const int bit = ((g1 >> 9) ^ (g2 >> a) ^ (g2 >> b)) & 1;
        out[i] = bit ? -1 : 1;
        const uint16_t f1 = ((g1 >> 2) ^ (g1 >> 9)) & 1;
        const uint16_t f2 = ((g2 >> 1) ^ (g2 >> 2) ^ (g2 >> 5) ^ (g2 >> 7) ^ (g2 >> 8) ^ (g2 >> 9)) & 1;
        g1 = static_cast<uint16_t>(((g1 << 1) | f1) & 0x3FF);
        g2 = static_cast<uint16_t>(((g2 << 1) | f2) & 0x3FF);
    }
}

inline uint32_t bit_reverse11(uint32_t i) {
#if defined(__ARM_ARCH_7EM__)
    uint32_t r;
    __asm__("rbit %0, %1" : "=r"(r) : "r"(i));
    return r >> 21;
#else
    uint32_t r = 0;
    for (int k = 0; k < 11; k++) r |= ((i >> k) & 1u) << (10 - k);
    return r;
#endif
}

/* 2048-point radix-2 complex FFT, input in bit-reversed order, forward sign
 * exp(-j...). Twiddles from a quarter-wave cosine table (513 floats). */
class Fft {
   public:
    Fft() {
        for (int k = 0; k <= kN / 4; k++) cosq_[k] = static_cast<float>(std::cos(2.0 * 3.14159265358979323846 * k / kN));
    }
    /* exp(-j 2 pi k / N), 0 <= k < N/2 */
    void twiddle(int k, float& c, float& s) const {
        if (k <= kN / 4) {
            c = cosq_[k];
            s = -cosq_[kN / 4 - k];
        } else {
            c = -cosq_[kN / 2 - k];
            s = -cosq_[k - kN / 4];
        }
    }
    void run(cf* x) const {
        for (int len = 2, step = kN / 2; len <= kN; len <<= 1, step >>= 1) {
            const int half = len >> 1;
            for (int k = 0; k < half; k++) {
                float wr, wi;
                twiddle(k * step, wr, wi);
                for (int i = k; i < kN; i += len) {
                    cf& a = x[i];
                    cf& b = x[i + half];
                    const float tr = wr * b.re - wi * b.im;
                    const float ti = wr * b.im + wi * b.re;
                    b.re = a.re - tr;
                    b.im = a.im - ti;
                    a.re += tr;
                    a.im += ti;
                }
            }
        }
    }

   private:
    float cosq_[kN / 4 + 1];
};

/* K ms of complex samples, 4 bits each (2-bit I, 2-bit Q). */
class Snapshot {
   public:
    void start(int k_ms) {
        k_ = k_ms < 1 ? 1 : (k_ms > kMaxK ? kMaxK : k_ms);
        filled_ = 0;
        th_i_ = th_q_ = 0;
        dc_i_ = dc_q_ = 0;
        ac_sq_ = 0;
        clip_ = 0;
        peak_ = 0;
    }
    bool full() const { return filled_ >= static_cast<uint32_t>(k_ * kN); }
    int k() const { return k_; }
    /* Append samples (interleaved int8 I, Q); returns how many were taken. */
    size_t put(const int8_t* iq, size_t n) {
        /* DC offset per channel, from every buffer (1 ms): the 2-bit quantiser is
         * symmetric about it. A HackRF Pro recording with the amp on had -4.7 LSB
         * of DC, which made random PRNs cross the threshold (at 0 Hz residual
         * Doppler even after a one-off estimate), so it is tracked per buffer. */
        {
            int32_t si = 0, sq = 0;
            for (size_t k = 0; k < n; k++) {
                si += iq[2 * k];
                sq += iq[2 * k + 1];
            }
            const int32_t nn = static_cast<int32_t>(n ? n : 1);
            dc_i_ = (si * 16 + (si >= 0 ? nn / 2 : -nn / 2)) / nn;
            dc_q_ = (sq * 16 + (sq >= 0 ? nn / 2 : -nn / 2)) / nn;
        }
        if (th_i_ == 0) { /* magnitude threshold ~1 sigma (1.2533 x mean |x - dc|), first buffer, 1/16 LSB */
            uint32_t ai = 0, aq = 0;
            for (size_t k = 0; k < n; k++) {
                const int32_t a = iq[2 * k] * 16 - dc_i_, b = iq[2 * k + 1] * 16 - dc_q_;
                ai += static_cast<uint32_t>(a < 0 ? -a : a);
                aq += static_cast<uint32_t>(b < 0 ? -b : b);
            }
            th_i_ = threshold16(ai, n);
            th_q_ = threshold16(aq, n);
        }
        size_t taken = 0;
        while (taken < n && !full()) {
            const int i = iq[2 * taken], q = iq[2 * taken + 1];
            const int32_t a = i * 16 - dc_i_, b = q * 16 - dc_q_; /* DC removed, 1/16 LSB */
            ac_sq_ += static_cast<uint32_t>((a * a + b * b + 128) >> 8);
            if (i >= 127 || i <= -127 || q >= 127 || q <= -127) clip_++;
            const int ai = i < 0 ? -i : i, aq = q < 0 ? -q : q;
            if (ai > peak_) peak_ = ai;
            if (aq > peak_) peak_ = aq;
            const int32_t ma = a < 0 ? -a : a, mb = b < 0 ? -b : b;
            /* a zero sample takes the sign of the sample index parity: no DC from rounding */
            const bool odd = filled_ & 1;
            const uint8_t ci = static_cast<uint8_t>((a < 0 || (a == 0 && odd) ? 2 : 0) | (ma >= th_i_ ? 1 : 0));
            const uint8_t cq = static_cast<uint8_t>((b < 0 || (b == 0 && odd) ? 2 : 0) | (mb >= th_q_ ? 1 : 0));
            const uint8_t nib = static_cast<uint8_t>((ci << 2) | cq);
            uint8_t& byte = data_[filled_ >> 1];
            byte = (filled_ & 1) ? static_cast<uint8_t>((byte & 0x0F) | (nib << 4)) : nib;
            filled_++;
            taken++;
        }
        return taken;
    }
    /* Level of a 2-bit code: +-1 or +-3. */
    static float level(uint8_t c) { return (c & 2 ? -1.0f : 1.0f) * (c & 1 ? 3.0f : 1.0f); }
    void sample(uint32_t idx, float& i, float& q) const {
        const uint8_t nib = static_cast<uint8_t>((data_[idx >> 1] >> ((idx & 1) * 4)) & 0x0F);
        i = level(nib >> 2);
        q = level(nib & 3);
    }
    /* Noise statistics for the gain advice. */
    /* AC rms per component (DC removed), LSB */
    float rms() const { return filled_ ? std::sqrt(static_cast<float>(ac_sq_) / (2.0f * filled_)) : 0.0f; }
    /* DC offset of the first buffer, LSB */
    float dc_i() const { return dc_i_ / 16.0f; }
    float dc_q() const { return dc_q_ / 16.0f; }
    uint32_t clipped() const { return clip_; }
    uint32_t filled() const { return filled_; }
    int peak() const { return peak_; }

   private:
    uint8_t data_[kMaxK * kN / 2]{};
    uint32_t filled_{0};
    int k_{0};                  /* zero-initialised object: .bss, not .data */
    int32_t th_i_{0}, th_q_{0}; /* 1/16 LSB */
    int32_t dc_i_{0}, dc_q_{0}; /* 1/16 LSB */
    uint32_t ac_sq_{0};
    static int32_t threshold16(uint32_t sum_abs16, size_t n) {
        const uint32_t mean16 = static_cast<uint32_t>(sum_abs16 / (n ? n : 1));
        const int32_t th = static_cast<int32_t>((mean16 * 1253u + 500u) / 1000u);
        return th < 8 ? 8 : th; /* at least half an LSB */
    }
    uint32_t clip_{0};
    int peak_{0};
};

using Result = gps_rx::Result;

/* The search over a PRN list. step() does one work unit: one 1 ms block of
 * one Doppler bin of one PRN (search), or one replica correlation over the
 * snapshot (refinement), a few ms of M4 time each, so the baseband thread
 * stays responsive. Working memory: 2 x 16 KiB complex buffers, an 8 KiB
 * accumulator, tables (4 KiB) and the snapshot. */
class Acquirer {
   public:
    struct Request {
        uint32_t prn_mask{0};  /* bit p-1: search PRN p */
        int16_t center[32]{};  /* Doppler window centre per PRN, Hz */
        uint8_t half_bins{12}; /* bins either side of the centre (12: +-6 kHz) */
        uint8_t refine{1};     /* refine detected PRNs */
    };

    explicit Acquirer(const Snapshot& snap)
        : snap_{snap} {
        for (int k = 0; k < 256; k++) sin256_[k] = static_cast<float>(std::sin(2.0 * 3.14159265358979323846 * k / 256));
    }
    void start(const Request& r) {
        req_ = r;
        for (int p = 0; p < 32; p++) {
            res_[p] = Result{};
            res_[p].prn = static_cast<uint8_t>(p + 1);
        }
        prn_ = 0;
        phase_ = SEARCH;
        refine_prn_ = 0;
        next_prn();
    }
    void stop() { phase_ = DONE; }
    bool busy() const { return phase_ != DONE; }
    /* One unit of work; false when finished. */
    bool step() {
        if (phase_ == SEARCH) {
            if (prn_ > 32) {
                phase_ = req_.refine ? REFINE : DONE;
                refine_prn_ = 0;
                if (phase_ == REFINE) next_refine();
                return phase_ != DONE;
            }
            search_block();
            return true;
        }
        if (phase_ == REFINE) {
            if (refine_prn_ > 32) {
                phase_ = DONE;
                return false;
            }
            refine_unit();
            return phase_ != DONE;
        }
        return false;
    }
    const Result& result(int prn) const { return res_[prn - 1]; }
    const Result* results() const { return res_; }
    /* Progress: PRNs finished of the requested ones. */
    int done_count() const {
        int n = 0;
        for (const auto& r : res_) n += r.searched;
        return n;
    }
    int requested() const {
        int n = 0;
        for (int p = 0; p < 32; p++) n += (req_.prn_mask >> p) & 1;
        return n;
    }
    bool refining() const { return phase_ == REFINE; }

   private:
    enum Phase : uint8_t { SEARCH,
                           REFINE,
                           DONE };
    static constexpr int kFineSteps = 21; /* fine Doppler: -250 .. +250 Hz, 25 Hz */
    static constexpr int kCodePts = 49;   /* code offsets: -1.5 .. +1.5 chips, 1/16 */
    const Snapshot& snap_;
    Fft fft_{};
    Request req_{};
    Result res_[32]{};
    cf work_[kN]{};
    cf code_[kN]{};
    float acc_[kN]{};
    float sin256_[256]{};
    int8_t chips_[1023]{};
    int prn_{0}, bin_{0}, nbins_{0}, blk_{0};
    uint32_t ph_{0}, ph_step_{0};
    int refine_prn_{0}, rj_{0};
    float rf_best_{0};
    double rp_best_{0};
    double pw_[kCodePts]{};
    Phase phase_{DONE};
    /* running search state of the current PRN */
    double total_{0};
    float best_{-1}, nb_lo_{0}, nb_hi_{0};
    int best_idx_{0}, best_bin_{0};

    float sinf_(uint32_t ph) const { return sin256_[ph >> 24]; }
    float cosf_(uint32_t ph) const { return sin256_[((ph >> 24) + 64) & 0xFF]; }

    static uint32_t nco_step(float fd) {
        return static_cast<uint32_t>(static_cast<int32_t>(std::lround(static_cast<double>(fd) / kFs * 4294967296.0)));
    }

    void next_prn() {
        do {
            prn_++;
        } while (prn_ <= 32 && !((req_.prn_mask >> (prn_ - 1)) & 1));
        if (prn_ > 32) return;
        bin_ = 0;
        blk_ = 0;
        nbins_ = 2 * req_.half_bins + 1;
        total_ = 0;
        best_ = -1;
        /* conj(FFT(code sampled at 2.048 Msps, nearest chip)) */
        ca_code(prn_, chips_);
        for (int n = 0; n < kN; n++) {
            const int chip = static_cast<int>((static_cast<uint32_t>(n) * 1023u) >> 11);
            cf& c = code_[bit_reverse11(static_cast<uint32_t>(n))];
            c.re = chips_[chip];
            c.im = 0;
        }
        fft_.run(code_);
        for (int n = 0; n < kN; n++) code_[n].im = -code_[n].im;
    }

    void next_refine() {
        do {
            refine_prn_++;
        } while (refine_prn_ <= 32 && !res_[refine_prn_ - 1].detected);
        rj_ = 0;
        rp_best_ = -1;
        if (refine_prn_ <= 32) {
            ca_code(refine_prn_, chips_);
            rf_best_ = res_[refine_prn_ - 1].doppler;
        }
    }

    void search_block() {
        if (blk_ == 0) {
            const float fd = req_.center[prn_ - 1] + static_cast<float>((bin_ - req_.half_bins) * kStep);
            ph_step_ = nco_step(fd);
            ph_ = 0;
            for (int n = 0; n < kN; n++) acc_[n] = 0;
        }
        const uint32_t base = static_cast<uint32_t>(blk_ * kN);
        uint32_t ph = ph_;
        for (int n = 0; n < kN; n++) {
            float i, q;
            snap_.sample(base + static_cast<uint32_t>(n), i, q);
            const float c = cosf_(ph), s = sinf_(ph);
            ph += ph_step_;
            cf& w = work_[bit_reverse11(static_cast<uint32_t>(n))];
            w.re = i * c + q * s; /* x * exp(-j ph) */
            w.im = q * c - i * s;
        }
        ph_ = ph;
        fft_.run(work_);
        /* multiply by conj(code) and conjugate (IFFT via the forward FFT), in
         * place, then the bit-reversal permutation for the second FFT */
        for (int n = 0; n < kN; n++) {
            const cf a = work_[n], b = code_[n];
            work_[n].re = a.re * b.re - a.im * b.im;
            work_[n].im = -(a.re * b.im + a.im * b.re);
        }
        for (uint32_t n = 0; n < static_cast<uint32_t>(kN); n++) {
            const uint32_t r = bit_reverse11(n);
            if (r > n) {
                const cf t = work_[n];
                work_[n] = work_[r];
                work_[r] = t;
            }
        }
        fft_.run(work_);
        for (int n = 0; n < kN; n++) acc_[n] += work_[n].re * work_[n].re + work_[n].im * work_[n].im;
        if (++blk_ < snap_.k()) return;
        /* bin complete */
        blk_ = 0;
        float m = -1, sum = 0;
        int mi = 0;
        for (int n = 0; n < kN; n++) {
            sum += acc_[n];
            if (acc_[n] > m) {
                m = acc_[n];
                mi = n;
            }
        }
        total_ += sum;
        if (m > best_) {
            best_ = m;
            best_idx_ = mi;
            best_bin_ = bin_;
            nb_lo_ = acc_[(mi + kN - 1) % kN];
            nb_hi_ = acc_[(mi + 1) % kN];
        }
        if (++bin_ >= nbins_) finish_prn();
    }

    void finish_prn() {
        Result& r = res_[prn_ - 1];
        const double mean = total_ / (static_cast<double>(kN) * nbins_);
        r.metric = mean > 0 ? static_cast<float>(best_ / mean) : 0.0f;
        const float den = nb_lo_ - 2 * best_ + nb_hi_;
        float frac = den != 0 ? 0.5f * (nb_lo_ - nb_hi_) / den : 0.0f;
        if (frac > 0.5f) frac = 0.5f;
        if (frac < -0.5f) frac = -0.5f;
        const double m = best_idx_ + frac;
        double cp = std::fmod(-m * kChipRate / kFs, 1023.0);
        if (cp < 0) cp += 1023.0;
        r.code_phase = static_cast<float>(cp);
        r.doppler = req_.center[prn_ - 1] + static_cast<float>((best_bin_ - req_.half_bins) * kStep);
        r.searched = 1;
        r.detected = r.metric > kThreshold[snap_.k() - 1];
        /* C/N0: per-block SNR of the peak cell (power over the noise mean),
         * +2 dB for the 2-bit, sampling and bin-edge losses (calibrated on
         * synthetic signals: within 1 dB at 36-45 dB-Hz). */
        const double snr = static_cast<double>(r.metric) - 1.0;
        r.cn0 = snr > 0 ? static_cast<float>(10.0 * std::log10(snr / 1e-3) + 2.0) : 0.0f;
        next_prn();
    }

    /* Non-coherent sum over the K blocks of the coherent 1 ms power of a
     * replica at code phase cp (chips at sample 0) and Doppler fd. */
    double power(double cp, float fd) const {
        const int k_ms = snap_.k();
        const double rate = kChipRate * (1.0 + fd / kL1) / kFs;
        const uint32_t step = nco_step(fd);
        uint32_t ph = 0;
        double p = 0;
        int64_t cfix = static_cast<int64_t>(cp * 4294967296.0); /* 32.32 chips */
        const int64_t cstep = static_cast<int64_t>(rate * 4294967296.0);
        const int64_t wrap = static_cast<int64_t>(1023) << 32;
        for (int blk = 0; blk < k_ms; blk++) {
            float si = 0, sq = 0;
            const uint32_t base = static_cast<uint32_t>(blk * kN);
            for (int n = 0; n < kN; n++) {
                float i, q;
                snap_.sample(base + static_cast<uint32_t>(n), i, q);
                const float c = cosf_(ph), s = sinf_(ph);
                ph += step;
                const float ch = chips_[static_cast<int>(cfix >> 32)];
                cfix += cstep;
                if (cfix >= wrap) cfix -= wrap;
                si += ch * (i * c + q * s);
                sq += ch * (q * c - i * s);
            }
            p += static_cast<double>(si) * si + static_cast<double>(sq) * sq;
        }
        return p;
    }

    /* Refinement, one correlation per call: fine Doppler (+-250 Hz at the
     * coarse code phase), then P(d) at 49 code offsets and the centre where
     * the powers 0.5 chip early and late balance (as a DLL settles). */
    void refine_unit() {
        Result& r = res_[refine_prn_ - 1];
        if (rj_ < kFineSteps) {
            const float f = r.doppler + 25.0f * (rj_ - kFineSteps / 2);
            const double p = power(r.code_phase, f);
            if (p > rp_best_) {
                rp_best_ = p;
                rf_best_ = f;
            }
            rj_++;
            return;
        }
        const int j = rj_ - kFineSteps;
        if (j < kCodePts) {
            pw_[j] = power(r.code_phase + (j - 24) / 16.0, rf_best_);
            rj_++;
            if (j + 1 < kCodePts) return;
        }
        constexpr int h = 8; /* 0.5 chip */
        int jmax = h;
        for (int k = h; k < kCodePts - h; k++)
            if (pw_[k] > pw_[jmax]) jmax = k;
        double best = (jmax - 24) / 16.0;
        for (int k = jmax - h; k < jmax + h && k + 1 < kCodePts - h; k++) {
            if (k < h) continue;
            const double a = pw_[k + h] - pw_[k - h], b = pw_[k + 1 + h] - pw_[k + 1 - h];
            if (a == 0 || (a > 0) != (b > 0)) {
                const double frac = (a != b) ? a / (a - b) : 0.0;
                best = (k - 24 + frac) / 16.0;
                break;
            }
        }
        double cp = std::fmod(r.code_phase + best, 1023.0);
        if (cp < 0) cp += 1023.0;
        r.code_phase = static_cast<float>(cp);
        r.doppler = rf_best_;
        r.refined = 1;
        next_refine();
    }
};

/* A whole search as the PGPR baseband runs it, host testable:
 *  1. capture snapshot A and check its level: AC rms below kMinRms LSB (a
 *     starved ADC, mostly 0 / +-1) stops here with LEVEL_LOW and no results;
 *     clipping above kMaxClipPermille is reported as LEVEL_HIGH (search goes on);
 *  2. search A;
 *  3. confirmation (if asked): capture snapshot B, search only the PRNs found
 *     in A around their Doppler (+-1 kHz), refine there; a PRN counts as
 *     detected only if B finds it at a consistent Doppler (+-750 Hz) and its
 *     code phase moved by what that Doppler predicts (code / carrier = 1/1540)
 *     over the time between A and B. Others stay as "once" (not reported as
 *     heard). Noise and interference peaks do not repeat that way.
 * The caller feeds samples while want_samples() and steps otherwise; times
 * are in seconds on any monotonic clock (the M4 uses its cycle counter). */
class Session {
   public:
    enum Phase : uint8_t { IDLE,
                           CAPTURE_A,
                           SEARCH_A,
                           CAPTURE_B,
                           SEARCH_B,
                           DONE };
    enum Level : uint8_t { LEVEL_OK = 0,
                           LEVEL_LOW = 1,
                           LEVEL_HIGH = 2 };
    static constexpr float kMinRms = 1.5f;             /* LSB, AC rms per component */
    static constexpr uint32_t kMaxClipPermille = 10;   /* 1 % of the samples at full scale */
    static constexpr float kConfirmDopplerHz = 750.0f; /* A vs B */
    static constexpr int kConfirmHalfBins = 2;         /* B search: +-1 kHz around A's Doppler */
    struct Params {
        uint32_t prn_mask{0};
        int16_t center[32]{};
        uint8_t half_bins{12};
        uint8_t k_ms{20};
        uint8_t refine{1};
        uint8_t confirm{1};
    };

    Session(Snapshot& snap, Acquirer& acq)
        : snap_{snap}, acq_{acq} {}

    void start(const Params& p) {
        p_ = p;
        for (int i = 0; i < 32; i++) {
            final_[i] = Result{};
            final_[i].prn = static_cast<uint8_t>(i + 1);
        }
        level_ = LEVEL_OK;
        snap_.start(p.k_ms);
        phase_ = CAPTURE_A;
        t_a_ = t_b_ = -1;
    }
    void stop() {
        acq_.stop();
        phase_ = IDLE;
    }
    bool want_samples() const { return phase_ == CAPTURE_A || phase_ == CAPTURE_B; }
    void put(const int8_t* iq, size_t n, double t_s) {
        if (!want_samples()) return;
        if (snap_.filled() == 0) (phase_ == CAPTURE_A ? t_a_ : t_b_) = t_s;
        snap_.put(iq, n);
        if (!snap_.full()) return;
        if (phase_ == CAPTURE_A) {
            const uint32_t f = snap_.filled();
            if (snap_.rms() < kMinRms) {
                level_ = LEVEL_LOW;
                phase_ = DONE;
                return;
            }
            if (f && snap_.clipped() * 1000u / f > kMaxClipPermille) level_ = LEVEL_HIGH;
            Acquirer::Request r;
            r.prn_mask = p_.prn_mask;
            std::memcpy(r.center, p_.center, sizeof(r.center));
            r.half_bins = p_.half_bins > 20 ? 20 : p_.half_bins;
            r.refine = p_.confirm ? 0 : p_.refine;
            acq_.start(r);
            phase_ = SEARCH_A;
        } else {
            Acquirer::Request r;
            for (int i = 0; i < 32; i++)
                if (a_[i].detected) {
                    r.prn_mask |= 1u << i;
                    r.center[i] = static_cast<int16_t>(std::lround(a_[i].doppler));
                }
            r.half_bins = kConfirmHalfBins;
            r.refine = p_.refine;
            acq_.start(r);
            phase_ = SEARCH_B;
        }
    }
    /* One work unit; false when there is nothing to step (capturing, done). */
    bool step() {
        if (phase_ != SEARCH_A && phase_ != SEARCH_B) return false;
        if (acq_.step()) return true;
        if (phase_ == SEARCH_A) {
            bool any = false;
            for (int i = 0; i < 32; i++) {
                a_[i] = acq_.result(i + 1);
                any |= a_[i].detected != 0;
            }
            if (p_.confirm && any) {
                snap_.start(p_.k_ms);
                phase_ = CAPTURE_B;
            } else {
                for (int i = 0; i < 32; i++) final_[i] = a_[i];
                phase_ = DONE;
            }
            return false;
        }
        /* SEARCH_B done: merge */
        const double dt = t_b_ - t_a_;
        for (int i = 0; i < 32; i++) {
            const Result& b = acq_.result(i + 1);
            Result r = a_[i].detected ? b : a_[i];
            r.searched = a_[i].searched;
            if (a_[i].detected) {
                const bool ok = consistent(a_[i], b, dt);
                r.detected = ok;
                r.once = !ok;
                if (!ok) { /* keep A's numbers for display */
                    r.metric = a_[i].metric;
                    r.doppler = a_[i].doppler;
                    r.code_phase = a_[i].code_phase;
                    r.cn0 = a_[i].cn0;
                    r.refined = 0;
                }
            }
            final_[i] = r;
        }
        phase_ = DONE;
        return false;
    }
    static bool consistent(const Result& a, const Result& b, double dt) {
        if (!b.detected || dt < 0) return false;
        if (std::fabs(b.doppler - a.doppler) > kConfirmDopplerHz) return false;
        const double fd = 0.5 * (static_cast<double>(a.doppler) + b.doppler);
        const double pred = a.code_phase + dt * fd * (kChipRate / kL1);
        double err = std::fmod(static_cast<double>(b.code_phase) - pred + 1023.0 * 1000.0 + 511.5, 1023.0) - 511.5;
        if (err < 0) err = -err;
        const double tol = 1.5 + 300.0 * dt * (kChipRate / kL1); /* code phase + 300 Hz of Doppler error over dt */
        return err <= tol;
    }
    Phase phase() const { return phase_; }
    Level level() const { return level_; }
    const Result* results() const { return final_; }
    double dt() const { return t_b_ - t_a_; }
    bool confirming() const { return phase_ == CAPTURE_B || phase_ == SEARCH_B; }

   private:
    Snapshot& snap_;
    Acquirer& acq_;
    Params p_{};
    Result a_[32]{};
    Result final_[32]{};
    Phase phase_{IDLE};
    Level level_{LEVEL_OK};
    double t_a_{-1}, t_b_{-1};
};

}  // namespace gps_acq

#endif /*__GPS_ACQ_H__*/
