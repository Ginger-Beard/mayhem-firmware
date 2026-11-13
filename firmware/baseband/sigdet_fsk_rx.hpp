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

/* Narrowband FSK / GFSK frame receiver of the Signature Detect baseband (PSDT
 * MODE_FSK), configured by the class file's FSK rules (common/sigdet_fsk.hpp).
 * The Z-Wave R1 receiver of build-zwave generalised: Z-Wave is the built-in
 * rule the "ZW" line stands for. Header only, no ChibiOS (host tested on the
 * Z-Wave recording and on synthetic frames). Receive only.
 *
 * Input: 4 Msps int8 I/Q, the channel at +fs/4 (the M0 tunes target = f).
 *   1. Shift by -fs/4 ((-j)^n, sign swaps), boxcar of D (4..250): 4 MS/s / D,
 *      first nulls at +-4 MHz / D; then a 2 tap average.
 *   2. Discriminator 2 Im(y conj y') / (|y|^2 + |y'|^2) (the sine of the phase
 *      step for equal magnitudes: no atan, one divide), moving average of
 *      `ma` samples.
 *   3. Carrier offset = the discriminator's DC: a slow average while
 *      searching, held during a frame.
 *   4. Chips at chip_hz: integrate and dump, the chip clock pulled to the zero
 *      crossings.
 *   5. Sync: the last chips against every rule's pattern (preamble tail +
 *      sync word, coded) or its complement (the polarity), <= max_err chips
 *      wrong. After the first match the receiver looks kLookahead chips
 *      further for a better one (a preamble shifted by a chip or two can
 *      pass with errors), takes the best (rule, polarity, position) and
 *      replays the chips after it.
 *   6. Bits by the rule's coding and bit order; bytes; the length rule
 *      (fixed, length field, or until the level drops); the check. A frame
 *      that passes (or any frame of a rule with no check) goes to the
 *      callback with its first <= kFskHdr bytes, level, measured deviation,
 *      carrier offset and bit rate. */

#ifndef __SIGDET_FSK_RX_HPP__
#define __SIGDET_FSK_RX_HPP__

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sigdet_fsk.hpp"

/* The per sample loop stays -O3 and inline; the per chip / per frame /
 * per dwell code is out of line and small (the PSDT image's 64 KB). */
#if defined(__GNUC__) && !defined(__clang__)
#define SIGDET_FSK_COLD __attribute__((noinline, optimize("Os")))
#else
#define SIGDET_FSK_COLD __attribute__((noinline))
#endif
#define SIGDET_FSK_COLD_HOT __attribute__((noinline))

namespace sigdet {

/* asin for |x| <= 1, series to x^5 (within 1 % to |x| = 0.7; the
 * deviation and offset are reported, not decided on). */
inline float fsk_asin(float x) {
    const float x2 = x * x;
    return x * (1.0f + x2 * (1.0f / 6.0f + x2 * (3.0f / 40.0f)));
}

struct FskFrame {
    uint8_t tag;        /* the rule's FskRule::tag */
    uint8_t len;        /* frame bytes after the sync, the check included */
    uint8_t nhdr;       /* bytes in hdr */
    uint8_t violations; /* Manchester chip pairs that were not complementary */
    bool checked;       /* the rule's check passed (false: a rule without a check) */
    uint8_t hdr[kFskHdr];
    float power;    /* mean |y|^2 over the frame, re full scale (a tone of |x| = 128 reads 1) */
    float dev_hz;   /* mean |frequency deviation| over the frame's chips */
    float off_hz;   /* carrier offset (held DC) */
    float rate_bps; /* bit rate the chip clock settled to */
};

struct FskStats {
    uint32_t syncs;   /* a pattern found */
    uint32_t ok;      /* frames with a passing check */
    uint32_t nocheck; /* frames of rules without a check */
    uint32_t bad_len; /* length field out of range, or no carrier drop by len_max */
    uint32_t bad_sum; /* check wrong */
};

class FskRx {
   public:
    static constexpr uint8_t kLookahead = 12;
    using Callback = void (*)(void* ctx, const FskFrame& f);

    FskRx() { reset(); }

    /* A new dwell plan (a retune): fresh state, the counters kept. */
    SIGDET_FSK_COLD void configure(const FskDwell& d) {
        cfg_ = d;
        if (cfg_.nrules > kFskRulesPerDwell) cfg_.nrules = kFskRulesPerDwell;
        cfg_.decim = static_cast<uint8_t>(cfg_.decim < 4 ? 4 : cfg_.decim & ~3u); /* sums of 4 samples */
        if (cfg_.ma < 1) cfg_.ma = 1;
        if (cfg_.ma > kMaxMa) cfg_.ma = kMaxMa;
        fs2_ = static_cast<float>(kFskInputRate) / static_cast<float>(cfg_.decim);
        spc_ = cfg_.chip_hz ? fs2_ / static_cast<float>(cfg_.chip_hz) : 10.0f;
        inv_ma_ = 1.0f / static_cast<float>(cfg_.ma);
        centre_ = spc_ < 8.0f;
        for (size_t i = 0; i < cfg_.nrules; i++) {
            const uint8_t n = cfg_.rules[i].chips > 64 ? 64 : cfg_.rules[i].chips;
            mask_[i] = n >= 64 ? ~uint64_t{0} : ((uint64_t{1} << n) - 1);
        }
        reset();
    }
    const FskDwell& config() const { return cfg_; }

    void reset() {
        si_ = sq_ = 0;
        cnt_ = 0;
        yi_ = yq_ = pi_ = pq_ = 0.0f;
        for (auto& d : dh_) d = 0.0f;
        dpos_ = 0;
        dsum_ = 0.0f;
        dc_ = 0.0f;
        ph_ = 0.0f;
        acc_ = 0.0f;
        last_c_ = 0.0f;
        chips_ = 0;
        searching_ = true;
        cand_left_ = 0;
        power_sum_ = 0.0f;
        power_n_ = 0;
    }
    void clear_stats() { stats_ = FskStats{}; }
    const FskStats& stats() const { return stats_; }

    /* count complex samples, a multiple of 4 (see sigdet_meter.hpp). */
    SIGDET_FSK_COLD_HOT void feed(const int8_t* iq, size_t count, Callback cb, void* ctx) {
        if (!cfg_.nrules) return;
        int32_t si = si_, sq = sq_;
        uint32_t cnt = cnt_;
        const uint32_t decim = cfg_.decim;
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
            cnt += 4;
            if (cnt >= decim) {
                sample(static_cast<float>(si), static_cast<float>(sq), cb, ctx);
                si = sq = 0;
                cnt = 0;
            }
        }
        si_ = si;
        sq_ = sq;
        cnt_ = cnt;
    }

   private:
    static constexpr uint8_t kMaxMa = 8;
    FskDwell cfg_{};
    uint64_t mask_[kFskRulesPerDwell]{};
    float fs2_{200000.0f};
    float spc_{10.0f};
    float inv_ma_{0.25f};
    bool centre_{false};
    int32_t si_{0}, sq_{0};
    uint32_t cnt_{0};
    float yi_{0}, yq_{0}; /* last boxcar output */
    float pi_{0}, pq_{0}; /* last filtered sample */
    float dh_[kMaxMa]{};
    uint8_t dpos_{0};
    float dsum_{0};
    float dc_{0};
    float ph_{0};  /* samples since the last chip boundary */
    float acc_{0}; /* this chip's integral */
    float last_c_{0};
    uint64_t chips_{0};
    bool searching_{true};
    uint8_t cand_left_{0}, cand_err_{0}, cand_since_{0}, cand_rule_{0};
    bool cand_inv_{false};
    /* the frame in progress */
    const FskRule* r_{nullptr};
    bool inv_{false};
    bool pair_{false};
    uint8_t first_{0}, last_chip_{0};
    uint8_t byte_{0}, nbits_{0}, nbytes_{0}, total_{0}, viol_{0}, viol_byte_{0};
    uint8_t buf_[kFskMaxBytes]{};
    float power_sum_{0};
    uint32_t power_n_{0};
    float byte_pow_{0};
    uint32_t byte_n_{0};
    float dev_sum_{0};
    float corr_sum_{0};
    uint32_t nchips_{0};
    FskStats stats_{};

    /* SWAR popcount: the M4 has no instruction for it. */
    static int popcount32(uint32_t v) {
        v = v - ((v >> 1) & 0x55555555u);
        v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
        return static_cast<int>((((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24);
    }
    static int popcount64(uint64_t v) { return popcount32(static_cast<uint32_t>(v)) + popcount32(static_cast<uint32_t>(v >> 32)); }

    void sample(float bi, float bq, Callback cb, void* ctx) {
        /* 2 tap average of the boxcar outputs. */
        const float yi = 0.5f * (bi + yi_), yq = 0.5f * (bq + yq_);
        yi_ = bi;
        yq_ = bq;
        const float cross = pi_ * yq - pq_ * yi;
        const float m2 = yi * yi + yq * yq + pi_ * pi_ + pq_ * pq_;
        const float d = m2 > 1.0f ? 2.0f * cross / m2 : 0.0f;
        pi_ = yi;
        pq_ = yq;
        dsum_ += d - dh_[dpos_];
        dh_[dpos_] = d;
        if (++dpos_ >= cfg_.ma) dpos_ = 0;
        const float ds = inv_ma_ * dsum_;
        if (searching_) dc_ += (ds - dc_) * (1.0f / 64.0f);
        const float c = ds - dc_;
        if (!searching_) {
            const float p = yi * yi + yq * yq;
            power_sum_ += p;
            power_n_++;
            byte_pow_ += p;
            byte_n_++;
        }

        /* Chip clock: a zero crossing should fall on a boundary. */
        if ((c > 0.0f) != (last_c_ > 0.0f)) {
            const float e = ph_ < 0.5f * spc_ ? ph_ : ph_ - spc_;
            const float k = 0.15f * e;
            ph_ -= k;
            if (!searching_) corr_sum_ += k;
        }
        last_c_ = c;
        /* Short chips (< 8 samples, high rates): only the middle half of the
         * chip counts, the edges carry the neighbours' (GFSK) smear. */
        if (!centre_ || (ph_ >= 0.25f * spc_ && ph_ < 0.75f * spc_)) acc_ += c;
        ph_ += 1.0f;
        if (ph_ >= spc_) {
            ph_ -= spc_;
            if (!searching_) {
                dev_sum_ += std::fabs(acc_) / spc_;
                nchips_++;
            }
            chip(acc_ > 0.0f ? 1 : 0, cb, ctx);
            acc_ = 0.0f;
        }
    }

    SIGDET_FSK_COLD void chip(uint8_t v, Callback cb, void* ctx) {
        chips_ = (chips_ << 1) | v;
        if (!searching_) {
            data_chip(v, cb, ctx);
            return;
        }
        int best = 99, best_rule = 0;
        bool best_inv = false;
        for (uint8_t i = 0; i < cfg_.nrules; i++) {
            const FskRule& r = cfg_.rules[i];
            const int e = popcount64((chips_ ^ r.pattern) & mask_[i]);
            const int ei = r.chips - e; /* against the complement */
            const int b = e < ei ? e : ei;
            if (b <= r.max_err && b < best) {
                best = b;
                best_rule = i;
                best_inv = ei < e;
            }
        }
        if (cand_left_ == 0 && best == 99) return;
        if (cand_left_ == 0) {
            cand_left_ = kLookahead;
            cand_err_ = 99;
            cand_since_ = 0;
        } else {
            cand_since_++;
        }
        if (best < cand_err_) {
            cand_err_ = static_cast<uint8_t>(best);
            cand_rule_ = static_cast<uint8_t>(best_rule);
            cand_inv_ = best_inv;
            cand_since_ = 0;
        }
        if (--cand_left_) return;
        start_frame(cb, ctx);
    }

    SIGDET_FSK_COLD void start_frame(Callback cb, void* ctx) {
        stats_.syncs++;
        searching_ = false;
        r_ = &cfg_.rules[cand_rule_];
        inv_ = cand_inv_;
        pair_ = false;
        nbits_ = nbytes_ = viol_ = viol_byte_ = 0;
        byte_ = 0;
        total_ = r_->len_mode == FSK_LEN_FIXED ? r_->len_min : 0;
        power_sum_ = byte_pow_ = 0.0f;
        power_n_ = byte_n_ = 0;
        dev_sum_ = corr_sum_ = 0.0f;
        nchips_ = 0;
        const uint8_t k = cand_since_;
        last_chip_ = static_cast<uint8_t>((chips_ >> k) & 1); /* the pattern's last chip */
        for (int i = k - 1; i >= 0 && !searching_; i--) data_chip(static_cast<uint8_t>((chips_ >> i) & 1), cb, ctx);
    }

    SIGDET_FSK_COLD void data_chip(uint8_t v, Callback cb, void* ctx) {
        uint8_t bit;
        if (r_->coding == FSK_NRZ) {
            bit = static_cast<uint8_t>(v ^ (inv_ ? 1 : 0));
        } else {
            if (!pair_) {
                first_ = v;
                pair_ = true;
                return;
            }
            pair_ = false;
            if (v == first_ && viol_ < 255) viol_++;
            if (r_->coding == FSK_MAN)
                bit = static_cast<uint8_t>(first_ ^ (inv_ ? 1 : 0));
            else if (r_->coding == FSK_MAN_INV)
                bit = static_cast<uint8_t>(first_ ^ (inv_ ? 0 : 1));
            else
                bit = first_ == last_chip_ ? 1 : 0;
            last_chip_ = v;
        }
        if (r_->flags & FSK_F_LSB)
            byte_ = static_cast<uint8_t>(byte_ | (bit << nbits_));
        else
            byte_ = static_cast<uint8_t>((byte_ << 1) | bit);
        if (++nbits_ < 8) return;
        nbits_ = 0;
        buf_[nbytes_++] = byte_;
        byte_ = 0;
        const float bp = byte_n_ ? byte_pow_ / static_cast<float>(byte_n_) : 0.0f;
        byte_pow_ = 0.0f;
        byte_n_ = 0;
        if (r_->len_mode == FSK_LEN_FIELD && !total_ && nbytes_ >= r_->len_off + r_->len_size) {
            total_ = fsk_field_total(*r_, buf_);
            if (!total_) {
                stats_.bad_len++;
                searching_ = true;
                return;
            }
        }
        if (r_->len_mode == FSK_LEN_CARRIER) {
            /* the level of this byte against the frame's mean so far: -9 dB ends it (this byte not counted) */
            const float mean = power_n_ ? power_sum_ / static_cast<float>(power_n_) : 0.0f;
            if (nbytes_ > r_->len_min && bp < mean * 0.125f) {
                viol_ = viol_byte_; /* not the dropped byte's */
                finish(static_cast<uint8_t>(nbytes_ - 1), cb, ctx);
                return;
            }
            viol_byte_ = viol_;
            if (nbytes_ >= r_->len_max) {
                stats_.bad_len++;
                searching_ = true;
                return;
            }
            return;
        }
        if (total_ && nbytes_ >= total_) {
            finish(total_, cb, ctx);
            return;
        }
        if (nbytes_ >= kFskMaxBytes) {
            stats_.bad_len++;
            searching_ = true;
        }
    }

    SIGDET_FSK_COLD void finish(uint8_t n, Callback cb, void* ctx) {
        searching_ = true;
        if (!fsk_check_ok(*r_, buf_, n)) {
            stats_.bad_sum++;
            return;
        }
        const bool checked = r_->check != FSK_CHK_NONE;
        if (checked)
            stats_.ok++;
        else
            stats_.nocheck++;
        FskFrame f{};
        f.tag = r_->tag;
        f.len = n;
        const uint8_t cs = fsk_check_size(r_->check);
        const uint8_t body = n > cs ? static_cast<uint8_t>(n - cs) : n;
        f.nhdr = static_cast<uint8_t>(body < kFskHdr ? body : kFskHdr);
        std::memcpy(f.hdr, buf_, f.nhdr);
        f.violations = viol_;
        f.checked = checked;
        /* |y| of a tone |x| = 128: 128 x D (boxcar), halves of two equal samples */
        const float full = 128.0f * 128.0f * static_cast<float>(cfg_.decim) * static_cast<float>(cfg_.decim);
        f.power = power_n_ ? power_sum_ / static_cast<float>(power_n_) / full : 0.0f;
        constexpr float k2pi = 1.0f / 6.2831853f;
        const float dm = nchips_ ? dev_sum_ / static_cast<float>(nchips_) : 0.0f;
        f.dev_hz = fsk_asin(dm < 1.0f ? dm : 1.0f) * fs2_ * k2pi;
        const float dc = dc_ < -1.0f ? -1.0f : (dc_ > 1.0f ? 1.0f : dc_);
        f.off_hz = fsk_asin(dc) * fs2_ * k2pi;
        const float spc = nchips_ ? spc_ + corr_sum_ / static_cast<float>(nchips_) : spc_;
        f.rate_bps = spc > 0.0f ? fs2_ / spc / static_cast<float>(fsk_chips_per_bit(r_->coding)) : 0.0f;
        if (cb) cb(ctx, f);
    }
};

} /* namespace sigdet */

#endif /*__SIGDET_FSK_RX_HPP__*/
