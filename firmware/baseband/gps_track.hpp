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

#ifndef __GPS_TRACK_H__
#define __GPS_TRACK_H__

/* GPS L1 C/A tracking channel (structure for the "Autonomous" method; NOT
 * used by the PGPR baseband yet: one channel of this plain-C correlator
 * costs most of the M4 at 2.048 Msps, see notes/gps-app-2026-11-13.md).
 * Host tested on synthetic signals (rf_test/gps/run_gps_track_test.sh).
 *
 * Per channel, integrate-and-dump from code epoch to code epoch (1 ms):
 *  carrier NCO 32-bit + 256-entry int8 table, code NCO in 32.32 chips,
 *  early / prompt / late at -0.5 / 0 / +0.5 chip, int8 x int8 products;
 *  FLL (first 300 ms) then a 2nd-order Costas PLL (15 Hz); a 1st-order DLL
 *  (1 Hz) aided by the carrier. After lock: bit sync (histogram of prompt
 *  sign changes over the 20 epochs of a bit), 50 bps bits. The loop design
 *  follows the PC prototype (proto/gps/gpsproto/track.py), which follows the
 *  textbook structure (Kaplan; Borre et al., "A Software-Defined GPS and
 *  Galileo Receiver"). */

#include <cmath>
#include <cstdint>

#include "gps_acq.hpp"

namespace gps_track {

class Channel {
   public:
    static constexpr int kFllEpochs = 300;
    static constexpr int kBitSyncStart = 400; /* epochs before the histogram starts */
    static constexpr int kBitSyncLen = 1000;

    void start(int prn, double code_phase_chips, double doppler_hz) {
        gps_acq::ca_code(prn, code_);
        code_[1023] = code_[0];
        for (int k = 0; k < 256; k++) {
            sin8_[k] = static_cast<int8_t>(std::lround(127.0 * std::sin(2.0 * 3.14159265358979323846 * k / 256)));
            cos8_[k] = static_cast<int8_t>(std::lround(127.0 * std::cos(2.0 * 3.14159265358979323846 * k / 256)));
        }
        prn_ = prn;
        code_phase_ = code_phase_chips;
        carr_freq_ = doppler_hz;
        loop_int_ = doppler_hz;
        code_freq_ = gps_acq::kChipRate * (1.0 + doppler_hz / gps_acq::kL1);
        carr_phase_ = 0;
        epochs_ = 0;
        have_prev_ = false;
        for (auto& h : hist_) h = 0;
        prev_sign_ = 0;
        edge_ = -1;
        bit_acc_ = 0;
        bit_count_ = 0;
        nbp_ = wbp_ = 0;
        cn0_blocks_ = 0;
        cn0_sum_ = 0;
        seg_ = 0;
        reset_acc();
    }

    /* Feed samples (interleaved int8 I/Q at 2.048 Msps). Calls on_bit(bit)
     * for every decided data bit after bit sync. */
    template <typename F>
    void feed(const int8_t* iq, int n, F&& on_bit) {
        const double chips_per_sample = code_freq_ / gps_acq::kFs;
        const int64_t cstep = static_cast<int64_t>(chips_per_sample * 4294967296.0);
        int64_t cfix = static_cast<int64_t>(code_phase_ * 4294967296.0);
        const int64_t epoch = static_cast<int64_t>(1023) << 32;
        const int64_t half = static_cast<int64_t>(1) << 31;
        const uint32_t pstep = static_cast<uint32_t>(static_cast<int32_t>(std::lround(carr_freq_ / gps_acq::kFs * 4294967296.0)));
        for (int k = 0; k < n; k++) {
            const int32_t i = iq[2 * k], q = iq[2 * k + 1];
            const uint32_t t = carr_phase_ >> 24;
            carr_phase_ += pstep;
            const int32_t c = cos8_[t], s = sin8_[t];
            const int32_t bi = i * c + q * s; /* x * exp(-j phase) */
            const int32_t bq = q * c - i * s;
            const int ip = static_cast<int>(cfix >> 32);
            const int ie = static_cast<int>(((cfix + half) % epoch) >> 32);
            const int il = static_cast<int>(((cfix - half + epoch) % epoch) >> 32);
            const int32_t ce = code_[ie], cp = code_[ip], cl = code_[il];
            ie_ += ce * bi;
            qe_ += ce * bq;
            ip_ += cp * bi;
            qp_ += cp * bq;
            il_ += cl * bi;
            ql_ += cl * bq;
            cfix += cstep;
            if (cfix >= epoch) { /* code epoch: dump and update the loops */
                cfix -= epoch;
                code_phase_ = static_cast<double>(cfix) / 4294967296.0;
                epoch_update(on_bit);
                return feed_rest(iq + 2 * (k + 1), n - k - 1, on_bit);
            }
        }
        code_phase_ = static_cast<double>(cfix) / 4294967296.0;
    }

    int prn() const { return prn_; }
    double doppler() const { return carr_freq_; }
    uint32_t epochs() const { return epochs_; }
    int bit_edge() const { return edge_; }
    /* narrowband / wideband power ratio C/N0 over the bits so far, dB-Hz */
    double cn0() const {
        if (cn0_blocks_ < 5) return 0;
        const double mu = cn0_sum_ / cn0_blocks_;
        if (mu <= 1.0 || mu >= 20.0) return 0;
        return 10.0 * std::log10((mu - 1.0) / (20.0 - mu) / 1e-3);
    }
    /* prompt correlations of the last epoch (for lock indicators / tests) */
    int32_t last_ip() const { return last_ip_; }
    int32_t last_qp() const { return last_qp_; }

   private:
    int8_t code_[1024]{};
    int8_t sin8_[256]{}, cos8_[256]{};
    int prn_{0};
    double code_phase_{0}, code_freq_{0}, carr_freq_{0}, loop_int_{0};
    uint32_t carr_phase_{0};
    uint32_t epochs_{0};
    int64_t ie_{0}, qe_{0}, ip_{0}, qp_{0}, il_{0}, ql_{0};
    int32_t last_ip_{0}, last_qp_{0};
    double prev_i_{0}, prev_q_{0};
    bool have_prev_{false};
    uint16_t hist_[20]{};
    int prev_sign_{0};
    int edge_{-1};
    int64_t bit_acc_{0};
    int bit_count_{0};
    /* C/N0 over 20 ms blocks aligned to the bits */
    double nbp_i_{0}, nbp_q_{0}, wbp_{0}, nbp_{0};
    int seg_{0};
    uint32_t cn0_blocks_{0};
    double cn0_sum_{0};

    void reset_acc() { ie_ = qe_ = ip_ = qp_ = il_ = ql_ = 0; }

    template <typename F>
    void feed_rest(const int8_t* iq, int n, F&& on_bit) {
        if (n > 0) feed(iq, n, on_bit);
    }

    template <typename F>
    void epoch_update(F&& on_bit) {
        const double i_p = static_cast<double>(ip_), q_p = static_cast<double>(qp_);
        last_ip_ = static_cast<int32_t>(ip_ / 256);
        last_qp_ = static_cast<int32_t>(qp_ / 256);
        constexpr double T = 1e-3;
        constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
        /* carrier */
        const double pll_err = i_p != 0 ? std::atan(q_p / i_p) / kTwoPi : 0.0; /* cycles */
        double fll_err = 0;
        if (have_prev_) {
            const double dot = prev_i_ * i_p + prev_q_ * q_p;
            const double cross = prev_i_ * q_p - prev_q_ * i_p;
            fll_err = dot != 0 ? std::atan(cross / dot) / (kTwoPi * T) : 0.0; /* Hz, +-250 */
        }
        prev_i_ = i_p;
        prev_q_ = q_p;
        have_prev_ = true;
        constexpr double wn_p = 15.0 / 0.53;
        constexpr double k_f = 4.0 * 8.0; /* 1st-order FLL, Bn 8 Hz */
        if (epochs_ < kFllEpochs) {
            loop_int_ += k_f * T * fll_err;
            carr_freq_ = loop_int_;
        } else {
            loop_int_ += wn_p * wn_p * T * pll_err;
            carr_freq_ = loop_int_ + 1.414 * wn_p * pll_err;
        }
        /* code: normalised early-minus-late envelope, carrier aided */
        const double e = std::sqrt(static_cast<double>(ie_) * ie_ + static_cast<double>(qe_) * qe_);
        const double l = std::sqrt(static_cast<double>(il_) * il_ + static_cast<double>(ql_) * ql_);
        const double code_err = (e + l) > 0 ? 0.5 * (e - l) / (e + l) : 0.0;
        code_freq_ = gps_acq::kChipRate * (1.0 + carr_freq_ / gps_acq::kL1) + 4.0 * 1.0 * code_err;
        /* bit sync, then bits */
        const int sign = ip_ >= 0 ? 1 : -1;
        if (edge_ < 0) {
            if (epochs_ >= static_cast<uint32_t>(kBitSyncStart)) {
                if (prev_sign_ && sign != prev_sign_) hist_[epochs_ % 20]++;
                if (epochs_ >= static_cast<uint32_t>(kBitSyncStart + kBitSyncLen)) {
                    int best = 0;
                    for (int k = 1; k < 20; k++)
                        if (hist_[k] > hist_[best]) best = k;
                    edge_ = best;
                }
            }
        } else {
            if (static_cast<int>(epochs_ % 20) == edge_ && bit_count_ > 0) {
                if (bit_count_ == 20) {
                    on_bit(bit_acc_ < 0 ? 1 : 0);
                    /* C/N0 (NWPR) per bit */
                    if (wbp_ > 0) {
                        cn0_sum_ += (nbp_i_ * nbp_i_ + nbp_q_ * nbp_q_) / wbp_;
                        cn0_blocks_++;
                    }
                }
                bit_acc_ = 0;
                bit_count_ = 0;
                nbp_i_ = nbp_q_ = wbp_ = 0;
            }
            bit_acc_ += ip_;
            bit_count_++;
            const double a = i_p / 65536.0, b = q_p / 65536.0;
            nbp_i_ += a;
            nbp_q_ += b;
            wbp_ += a * a + b * b;
        }
        prev_sign_ = sign;
        epochs_++;
        reset_acc();
    }
};

}  // namespace gps_track

#endif /*__GPS_TRACK_H__*/
