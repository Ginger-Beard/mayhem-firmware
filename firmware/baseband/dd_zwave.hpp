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

/* Z-Wave (ITU-T G.9959) R1 frame receiver of the drone detect baseband (PDRN
 * MODE_ZWAVE): 9.6 kb/s FSK, Manchester, preamble 0x55.., start of frame
 * 0xF0, length in byte 7, XOR checksum seeded 0xFF over all bytes but the
 * last. Header only, no ChibiOS (host tested on a recording of a real
 * network and on synthetic frames). Receive only; no payload is decoded.
 *
 * Input: 4 Msps int8 I/Q, the channel (908.42 MHz) at +fs/4 (the M0 tunes it
 * as for the BLE / pattern / meter modes).
 *   1. Shift by -fs/4 ((-j)^n, sign swaps) and a boxcar of 20: 200 kS/s, first
 *      nulls at +-200 kHz (the frames are ~+-21 kHz deviation, the carrier
 *      within ~10 kHz); then a 2 tap average.
 *   2. Discriminator 2 Im(y conj y') / (|y|^2 + |y'|^2) (= sin of the phase
 *      step for equal magnitudes: no atan, one divide), moving average of 4.
 *   3. The carrier offset (DC of the discriminator) follows a slow average
 *      while searching and is held during a frame (Manchester is DC free).
 *   4. Chips at 19.2 kchip/s (10.42 samples each): integrate and dump, the
 *      chip clock pulled to the zero crossings.
 *   5. Sync: the last 48 chips against preamble 0x55 0x55 + SOF 0xF0
 *      (Manchester: bit b -> chips b, !b) or its complement, <= kSyncErrors
 *      chips wrong. The complement sets the polarity, the match the bit
 *      alignment.
 *   6. Bits = the first chip of each pair; byte 7 the length (10..64); at the
 *      end the checksum. A good frame goes to the callback with its first
 *      <= 16 bytes (home ID, source, frame control, length, destination,
 *      routing header and command class: the header) and its level. */

#ifndef __DD_ZWAVE_HPP__
#define __DD_ZWAVE_HPP__

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dd {

/* Preamble 0x55 0x55 + SOF 0xF0 as 48 Manchester chips (bit b -> b, !b), the
 * latest chip in bit 0. */
constexpr uint64_t zwave_sync_chips() {
    uint64_t p = 0;
    const uint32_t bits = 0x5555F0;
    for (int k = 23; k >= 0; k--) {
        const uint64_t b = (bits >> k) & 1;
        p = (p << 2) | (b << 1) | (b ^ 1);
    }
    return p;
}

struct ZwaveFrame {
    static constexpr size_t kHdr = 16;
    uint8_t len;        /* the frame's length byte (incl. the checksum) */
    uint8_t nhdr;       /* bytes in hdr */
    uint8_t hdr[kHdr];  /* the first bytes (header) */
    uint8_t violations; /* Manchester chip pairs that were not complementary */
    float power;        /* mean |y|^2 over the frame, re full scale (a tone of |x| = 128 reads 1) */
};

struct ZwaveStats {
    uint32_t syncs;   /* preamble + SOF found */
    uint32_t ok;      /* checksum valid */
    uint32_t bad_len; /* length byte out of range */
    uint32_t bad_sum; /* checksum wrong */
};

class ZwaveRx {
   public:
    static constexpr uint32_t kDecim = 20; /* 4 Msps -> 200 kS/s */
    static constexpr float kFs = 200000.0f;
    static constexpr float kChip = kFs / 19200.0f; /* 10.4167 samples per chip */
    static constexpr int kSyncErrors = 4;
    static constexpr uint8_t kLookahead = 12; /* chips searched for a better sync after the first */
    static constexpr uint8_t kMinLen = 10;
    static constexpr uint8_t kMaxLen = 64;
    using Callback = void (*)(void* ctx, const ZwaveFrame& f);

    ZwaveRx() { reset(); }

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
    void clear_stats() { stats_ = ZwaveStats{}; }
    const ZwaveStats& stats() const { return stats_; }

    /* count complex samples, a multiple of 4 (see dd_meter.hpp). */
    void feed(const int8_t* iq, size_t count, Callback cb, void* ctx) {
        int32_t si = si_, sq = sq_;
        uint32_t cnt = cnt_;
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
            if (cnt >= kDecim) {
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
    int32_t si_{0}, sq_{0};
    uint32_t cnt_{0};
    float yi_{0}, yq_{0}; /* last boxcar output */
    float pi_{0}, pq_{0}; /* last filtered sample */
    float dh_[4]{};
    uint8_t dpos_{0};
    float dsum_{0};
    float dc_{0};
    float ph_{0};  /* samples since the last chip boundary */
    float acc_{0}; /* this chip's integral */
    float last_c_{0};
    uint64_t chips_{0};
    bool searching_{true};
    uint8_t cand_left_{0}, cand_err_{0}, cand_since_{0};
    bool cand_inv_{false};
    bool inv_{false};
    bool pair_{false};
    uint8_t first_{0};
    uint8_t byte_{0}, nbits_{0}, nbytes_{0}, len_{0}, viol_{0};
    uint8_t buf_[kMaxLen]{};
    float power_sum_{0};
    uint32_t power_n_{0};
    ZwaveStats stats_{};

    static constexpr uint64_t kSync = zwave_sync_chips();
    static constexpr uint64_t kMask48 = (uint64_t{1} << 48) - 1;

    static int popcount64(uint64_t v) {
        int n = 0;
        while (v) {
            v &= v - 1;
            n++;
        }
        return n;
    }

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
        dpos_ = static_cast<uint8_t>((dpos_ + 1) & 3);
        const float ds = 0.25f * dsum_;
        if (searching_) dc_ += (ds - dc_) * (1.0f / 64.0f);
        const float c = ds - dc_;
        if (!searching_) {
            power_sum_ += yi * yi + yq * yq;
            power_n_++;
        }

        /* Chip clock: a zero crossing should fall on a boundary. */
        if ((c > 0.0f) != (last_c_ > 0.0f)) {
            const float e = ph_ < 0.5f * kChip ? ph_ : ph_ - kChip;
            ph_ -= 0.15f * e;
        }
        last_c_ = c;
        acc_ += c;
        ph_ += 1.0f;
        if (ph_ >= kChip) {
            ph_ -= kChip;
            chip(acc_ > 0.0f ? 1 : 0, cb, ctx);
            acc_ = 0.0f;
        }
    }

    void chip(uint8_t v, Callback cb, void* ctx) {
        chips_ = (chips_ << 1) | v;
        if (searching_) {
            const uint64_t c48 = chips_ & kMask48;
            const int e = popcount64(c48 ^ kSync);
            const int ei = popcount64(c48 ^ (~kSync & kMask48));
            const int best = e < ei ? e : ei;
            /* The preamble's and the SOF's chips are alike enough that a
             * match a few chips early can pass with errors: after the first
             * one, look kLookahead chips further for a better one, then take
             * the best and replay the chips that followed it. */
            if (cand_left_ == 0 && best <= kSyncErrors) {
                cand_left_ = kLookahead;
                cand_err_ = 99;
            }
            if (cand_left_ == 0) return;
            cand_since_++;
            if (best < cand_err_) {
                cand_err_ = best;
                cand_inv_ = ei < e;
                cand_since_ = 0;
            }
            if (--cand_left_) return;
            stats_.syncs++;
            searching_ = false;
            inv_ = cand_inv_;
            pair_ = false;
            nbits_ = nbytes_ = len_ = viol_ = 0;
            byte_ = 0;
            power_sum_ = 0.0f;
            power_n_ = 0;
            const uint8_t k = cand_since_;
            for (int i = k - 1; i >= 0 && !searching_; i--) data_chip(static_cast<uint8_t>((chips_ >> i) & 1), cb, ctx);
            return;
        }
        data_chip(v, cb, ctx);
    }

    void data_chip(uint8_t v, Callback cb, void* ctx) {
        if (!pair_) {
            first_ = v;
            pair_ = true;
            return;
        }
        pair_ = false;
        if (v == first_ && viol_ < 255) viol_++;
        byte_ = static_cast<uint8_t>((byte_ << 1) | (first_ ^ (inv_ ? 1 : 0)));
        if (++nbits_ < 8) return;
        nbits_ = 0;
        buf_[nbytes_++] = byte_;
        byte_ = 0;
        if (nbytes_ == 8) {
            len_ = buf_[7];
            if (len_ < kMinLen || len_ > kMaxLen) {
                stats_.bad_len++;
                searching_ = true;
                return;
            }
        }
        if (nbytes_ < 8 || nbytes_ < len_) return;
        searching_ = true;
        uint8_t x = 0xFF;
        for (size_t i = 0; i + 1 < len_; i++) x ^= buf_[i];
        if (x != buf_[len_ - 1]) {
            stats_.bad_sum++;
            return;
        }
        stats_.ok++;
        ZwaveFrame f{};
        f.len = len_;
        f.nhdr = static_cast<uint8_t>(static_cast<size_t>(len_ - 1) < ZwaveFrame::kHdr ? len_ - 1 : ZwaveFrame::kHdr);
        std::memcpy(f.hdr, buf_, f.nhdr);
        f.violations = viol_;
        /* |y| of a tone |x| = 128: 128 x 20 (boxcar), halves of two equal samples */
        f.power = power_n_ ? power_sum_ / static_cast<float>(power_n_) / (128.0f * 128.0f * kDecim * kDecim) : 0.0f;
        if (cb) cb(ctx, f);
    }
};

} /* namespace dd */

#endif /*__DD_ZWAVE_HPP__*/
