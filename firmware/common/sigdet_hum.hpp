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

/* Mains hum check of the Signature Detect interference scan: is the power in
 * a narrow channel modulated at harmonics of 50 or 60 Hz? Shared by the M4
 * (HumLadder, fed with the track meter's 256 us block powers) and the M0
 * (hum_verdict()). Header only, no ChibiOS (host tested).
 *
 * Why the channel power: amplitude hum (a 120 Hz gated noise, a carrier with
 * mains ripple) modulates it directly, and a carrier frequency-modulated by
 * the mains sweeps through a channel narrower than its swing, which turns the
 * FM into channel power pulses at the same harmonics (slope detection). On
 * the 2026-11-13 recordings of a mains-FM oscillator the harmonics 2..8 stood
 * 29..63 dB over the envelope spectrum's median in a 14 kHz channel. Noise
 * alone puts a Goertzel bin 15 dB over the mean of 12 references with a
 * probability of about e^-30 per bin.
 *
 * HumLadder: Goertzel filters over one window of block powers (DC removed
 * with the previous window's mean) at k x 60 Hz and k x 50 Hz (k = 1..12) and
 * at reference frequencies between harmonics: k x 60 + 30 Hz (a source
 * periodic at 30 Hz lights these as much as the 60 Hz ones, so it is not
 * called mains) and k x 50 + 25 Hz (never on a 60 Hz harmonic). A
 * window of ~1 s (3906 blocks of 256 us) resolves 1 Hz: the mains wander
 * (+-0.05 Hz, x12 = +-0.6 Hz) stays inside the main lobe. Output: each harmonic in
 * dB over its family's mean reference level.
 *
 * hum_verdict(): of the harmonics 2..12 of a family, leaving out 300 and 600 Hz
 * (shared by both), at least kHumMinHarm at >= kHumBarDb, and
 * at most one of the other family's: a source periodic at 20 or 40 Hz (seen
 * on a weather broadcast) lights up both families and is not mains. Three,
 * not more: noise gated at twice the mains frequency (full-wave rectified
 * supplies, dimmers) has the even harmonics only, and they fall off. The
 * fundamental is left out: receivers pick up a little 50/60 Hz themselves, and
 * a strong hum source leaks a fundamental into channels around it. */

#ifndef __SIGDET_HUM_HPP__
#define __SIGDET_HUM_HPP__

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace sigdet {

constexpr size_t kHumHarm = 12;
constexpr int8_t kHumBarDb = 15;
constexpr uint8_t kHumMinHarm = 3;

struct HumLevels {
    int8_t h60[kHumHarm]; /* k x 60 Hz (k = 1..12) over the 60 Hz family's reference, dB */
    int8_t h50[kHumHarm]; /* k x 50 Hz over the 50 Hz family's reference, dB */
    uint16_t blocks;      /* blocks in the window (0: none yet) */
};

struct HumVerdict {
    uint8_t family; /* 0 none, 50, 60 */
    uint8_t count;  /* harmonics of that family (2..12, shared ones left out) over the bar */
    uint8_t other;  /* ... of the other family */
    int8_t best_db; /* strongest of those harmonics */
};

/* Harmonic k (1-based) shared by both families (300 and 600 Hz). */
constexpr bool hum_shared(uint8_t family, size_t k) {
    return (family == 60 && (k == 5 || k == 10)) || (family == 50 && (k == 6 || k == 12));
}

inline HumVerdict hum_verdict(const HumLevels& l) {
    uint8_t c60 = 0, c50 = 0;
    int8_t b60 = -128, b50 = -128;
    for (size_t k = 2; k <= kHumHarm; k++) {
        if (!hum_shared(60, k)) {
            const int8_t v = l.h60[k - 1];
            if (v >= kHumBarDb) c60++;
            if (v > b60) b60 = v;
        }
        if (!hum_shared(50, k)) {
            const int8_t v = l.h50[k - 1];
            if (v >= kHumBarDb) c50++;
            if (v > b50) b50 = v;
        }
    }
    HumVerdict v{0, 0, 0, 0};
    if (!l.blocks) return v;
    if (c60 >= kHumMinHarm && c50 <= 1) {
        v = {60, c60, c50, b60};
    } else if (c50 >= kHumMinHarm && c60 <= 1) {
        v = {50, c50, c60, b50};
    } else {
        /* Nothing, or both families: report the stronger count for the log. */
        v.count = c60 >= c50 ? c60 : c50;
        v.other = c60 >= c50 ? c50 : c60;
        v.best_db = b60 >= b50 ? b60 : b50;
    }
    return v;
}

/* Goertzel bank on block powers. Float (M4); the app never runs it. */
class HumLadder {
   public:
    static constexpr size_t kFreqs = 4 * kHumHarm; /* h60, h50, r60, r50 */

    /* rate: blocks per second; window: blocks per result. */
    void configure(float rate, uint32_t window) {
        window_ = window ? window : 1;
        for (size_t k = 0; k < kHumHarm; k++) {
            const float kk = static_cast<float>(k + 1);
            coef_[k] = coef(60.0f * kk, rate);
            coef_[kHumHarm + k] = coef(50.0f * kk, rate);
            coef_[2 * kHumHarm + k] = coef(60.0f * kk + 30.0f, rate);
            coef_[3 * kHumHarm + k] = coef(50.0f * kk + 25.0f, rate);
        }
        mean_ = 0.0f;
        have_mean_ = false;
        restart();
        ready_ = false;
    }

    void add(float p) {
        if (!have_mean_) {
            /* The first window: the mean of its first 16 blocks stands in. */
            pre_ += p;
            if (++npre_ < 16) return;
            mean_ = pre_ / 16.0f;
            have_mean_ = true;
            return;
        }
        const float x = p - mean_;
        sum_ += p;
        for (size_t i = 0; i < kFreqs; i++) {
            const float s = x + coef_[i] * s1_[i] - s2_[i];
            s2_[i] = s1_[i];
            s1_[i] = s;
        }
        if (++n_ >= window_) finish();
    }

    /* The last finished window, once. */
    bool take(HumLevels& out) {
        if (!ready_) return false;
        out = last_;
        ready_ = false;
        return true;
    }

   private:
    float coef_[kFreqs]{};
    float s1_[kFreqs]{}, s2_[kFreqs]{};
    float mean_{0.0f}, sum_{0.0f}, pre_{0.0f};
    uint32_t window_{1}, n_{0}, npre_{0};
    bool have_mean_{false}, ready_{false};
    HumLevels last_{};

    /* 2 cos(2 pi f / rate) by a short series (no libm on the M4 image). */
    static float coef(float f, float rate) {
        float x = 6.28318530718f * f / rate;
        while (x > 3.14159265359f) x -= 6.28318530718f;
        const float x2 = x * x;
        const float c = 1.0f - x2 / 2 * (1.0f - x2 / 12 * (1.0f - x2 / 30 * (1.0f - x2 / 56 * (1.0f - x2 / 90 * (1.0f - x2 / 132)))));
        return 2.0f * c;
    }

    static float log2f_fast(float x) {
        uint32_t i;
        std::memcpy(&i, &x, sizeof(i));
        const float e = static_cast<float>(static_cast<int32_t>((i >> 23) & 0xff) - 127);
        i = (i & 0x007fffffu) | 0x3f800000u;
        float m;
        std::memcpy(&m, &i, sizeof(m));
        return e + (-1.7417939f + (2.8212026f + (-1.4699568f + (0.44717955f - 0.056570851f * m) * m) * m) * m) * 1.44269504f;
    }

    static int8_t db8(float ratio) {
        if (!(ratio > 1e-12f)) return -127;
        const float d = 3.01029996f * log2f_fast(ratio); /* 10 log10 */
        return static_cast<int8_t>(d > 127.0f ? 127 : (d < -127.0f ? -127 : (d >= 0 ? d + 0.5f : d - 0.5f)));
    }

    void restart() {
        for (size_t i = 0; i < kFreqs; i++) s1_[i] = s2_[i] = 0.0f;
        n_ = 0;
        sum_ = 0.0f;
    }

    void finish() {
        float pw[kFreqs];
        for (size_t i = 0; i < kFreqs; i++) {
            const float c = coef_[i];
            pw[i] = s1_[i] * s1_[i] + s2_[i] * s2_[i] - c * s1_[i] * s2_[i];
        }
        float r60 = 0.0f, r50 = 0.0f;
        for (size_t k = 0; k < kHumHarm; k++) {
            r60 += pw[2 * kHumHarm + k];
            r50 += pw[3 * kHumHarm + k];
        }
        r60 /= kHumHarm;
        r50 /= kHumHarm;
        for (size_t k = 0; k < kHumHarm; k++) {
            last_.h60[k] = db8(r60 > 0 ? pw[k] / r60 : 0.0f);
            last_.h50[k] = db8(r50 > 0 ? pw[kHumHarm + k] / r50 : 0.0f);
        }
        last_.blocks = static_cast<uint16_t>(n_ > 65535 ? 65535 : n_);
        ready_ = true;
        mean_ = sum_ / static_cast<float>(n_);
        restart();
    }
};

} /* namespace sigdet */

#endif /*__SIGDET_HUM_HPP__*/
