/*
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

/* DJI DroneID burst DETECTOR (presence only, no decoding) of the Signature Detect
 * baseband (PSDT, MODE_DRONEID). Header only, no heap, free of ChibiOS, so
 * firmware/test/baseband runs exactly this code on the host.
 *
 * Port of the PC prototype's kernel (mayhem-build/proto/droneid,
 * cpp/sigdet_droneid.hpp + detector.py, validated there on recorded real DJI
 * bursts) for 15.36 Msps only, with its recommended parameters fixed:
 * box2 power, k = 16 blocks, on +1.5 dB / off +0.8 dB over the floor, on-time
 * 535..720 us, 40 us pre-trigger, CP fold at the LTE lag, ZC root 600 test
 * at the two layouts' predicted reference symbol, +-12 lags, detect at a
 * normalised correlation >= 0.06. Written from the public PHY description
 * (dji-droneid-spec RF_PHY, NDSS'23); no third-party code.
 *
 * The OFDM burst (O1 / O2 / O4 shell: 8 or 9 symbols of 1096 samples at
 * 15.36 Msps, FFT 1024, CP 80 then 72; ~9 MHz occupied; 572 us or 643 us)
 * must be centred at 0 Hz: the M0 tunes the LO onto the DroneID centre.
 *
 * Stage A, every sample (Gate::feed, baseband thread):
 *   b_j   = sum over a 32 sample block of |x[2m] + x[2m+1]|^2 (a 2 tap boxcar,
 *           decimating: keeps +-4.5 MHz, drops noise towards +-fs/2)
 *   S_j   = b_j + ... + b_{j-15}
 *   floor = min over the last 16 DMA buffers of the buffer's mean block power
 *           scaled to 16 blocks, updated at each buffer's last block
 *   off -> on when 256 S > floor * 362, on -> off when 256 S < floor * 308;
 *   on-time (shifted back 8 blocks for the smoothing) in 8218..11059 samples
 *   -> candidate. From the rising edge on, the samples from 614 before the
 *   start are copied into the snapshot (the part before the current DMA
 *   buffer from the previous one, which the DMA does not overwrite until two
 *   transfers later); copying stops at the falling edge or when full.
 * Stage B, once per candidate on the frozen snapshot (StageB::evaluate, event
 * thread, ~0.7 M cycles by instruction count):
 *   B1 CP fold: q[n] = x[n] conj(x[n+1024]); P, E summed over 64 samples and
 *      folded with the symbol period 1096; argmax |F| / G gives the symbol
 *      grid, |F| / G the CP coherence, arg F the fractional CFO.
 *   B2 layout from the on-time (> 620 us: 9 symbols, ZC600 in symbol 3; else
 *      8 symbols, ZC600 in symbol 2), both tried, the larger kept.
 *   B3 ZC root 600 correlation at the predicted useful start +- 12 lags
 *      against the int8 reference: |<x, r>|^2 / (|x|^2 |r|^2) >= 0.06.
 *
 * Differences from the prototype kernel, none of which changes a result (the
 * host test checks candidates, scores, lags and decisions against it, bit
 * for bit, on every recording):
 *   - snapshot 11680 samples (the longest accepted candidate needs 614 +
 *     11059) instead of 16384; no 1152 sample history copy (the previous DMA
 *     buffer is read in place); fixed 15.36 Msps parameters;
 *   - B1 computes the fold in one pass with a ring of the last 64 fold
 *     values (0.75 KB instead of 18 KB of arrays): the same integer sums;
 *   - B3 and the fold use dual 16 bit MACs (SMLAD) on the M4 with exact
 *     plain C equivalents on the host: the same integer sums;
 *   - B2's double arithmetic is exact integer arithmetic; the float metric
 *     is built with fp-contract off, so the M4 (which has VFMA) and the host
 *     round alike.
 *
 * Integer overflow bounds: stage A blocks <= 16 * 2 * 256^2 = 2^21, on-time
 * sums <= 346 blocks * 2^21 < 2^31; fold values <= 12 symbols * 64 * 2^15
 * < 2^25; ZC sums <= 1024 * 2 * 128 * 127 < 2^25. */

#ifndef __SIGDET_DRONEID_H__
#define __SIGDET_DRONEID_H__

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sigdet_droneid_tables.hpp"

namespace sigdet {
namespace droneid {

constexpr uint32_t kSampleRate = 15360000;
constexpr size_t kBuffer = 2048; /* samples per DMA buffer = one floor segment */
constexpr size_t kBlock = 32;    /* samples per power block */
constexpr size_t kSegBlocks = kBuffer / kBlock;
constexpr size_t kFloorSegs = 16;
constexpr uint32_t kSmooth = 16; /* blocks in the moving sum (power of two) */
constexpr uint32_t kSmoothShift = 4;
constexpr uint32_t kOnQ8 = 362;    /* 256 * 10^(1.5/10), rounded */
constexpr uint32_t kOffQ8 = 308;   /* 256 * 10^(0.8/10) */
constexpr uint32_t kDurLo = 8218;  /* ceil(535 us * fs) */
constexpr uint32_t kDurHi = 11059; /* floor(720 us * fs) */
constexpr uint32_t kPre = 614;     /* 40 us */
constexpr size_t kSnapSamples = 11680;
static_assert(kSnapSamples >= kPre + kDurHi, "snapshot shorter than the longest candidate");
/* The pre-trigger part reaches at most kPre + 8 blocks before the current
 * buffer: within the previous one. */
static_assert(kPre + (kSmooth / 2) * kBlock <= kBuffer, "pre-trigger beyond the previous buffer");

constexpr uint32_t kLag = 1024;    /* useful symbol: the CP repeats it 1024 samples later */
constexpr uint32_t kWin = 64;      /* CP window */
constexpr uint32_t kSym = 1096;    /* symbol period (CP 72 + 1024) */
constexpr uint32_t kCp0 = 80;      /* first CP */
constexpr uint32_t kLayout = 9523; /* floor(620 us * fs): above, 9 symbols */
constexpr int32_t kSpan = 12;      /* +- lags around the predicted ZC start */
constexpr size_t kRefLen = 1024;
constexpr float kZcMin = 0.06f;

#if defined(__ARM_FEATURE_SIMD32)
inline uint32_t sxtb16(uint32_t x) {
    uint32_t r;
    __asm__("sxtb16 %0, %1" : "=r"(r) : "r"(x));
    return r;
}
inline uint32_t sxtb16_ror8(uint32_t x) {
    uint32_t r;
    __asm__("sxtb16 %0, %1, ror #8" : "=r"(r) : "r"(x));
    return r;
}
inline uint32_t smlad(uint32_t a, uint32_t b, uint32_t acc) {
    uint32_t r;
    __asm__("smlad %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
inline uint32_t smlsd(uint32_t a, uint32_t b, uint32_t acc) {
    uint32_t r;
    __asm__("smlsd %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
inline uint32_t smlsdx(uint32_t a, uint32_t b, uint32_t acc) {
    uint32_t r;
    __asm__("smlsdx %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
inline uint32_t pkhbt(uint32_t a, uint32_t b) { /* (lo a, lo b) */
    uint32_t r;
    __asm__("pkhbt %0, %1, %2, lsl #16" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
inline uint32_t pkhtb(uint32_t a, uint32_t b) { /* (hi a, hi b) */
    uint32_t r;
    __asm__("pkhtb %0, %2, %1, asr #16" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
inline uint32_t sadd16(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("sadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
#else
/* Exact plain C equivalents (host). */
inline int32_t lo16(uint32_t x) {
    return static_cast<int16_t>(x & 0xFFFF);
}
inline int32_t hi16(uint32_t x) {
    return static_cast<int16_t>(x >> 16);
}
inline uint32_t pack16(int32_t lo, int32_t hi) {
    return (static_cast<uint32_t>(lo) & 0xFFFF) | (static_cast<uint32_t>(hi) << 16);
}
inline uint32_t sxtb16(uint32_t x) {
    return pack16(static_cast<int8_t>(x & 0xFF), static_cast<int8_t>((x >> 16) & 0xFF));
}
inline uint32_t sxtb16_ror8(uint32_t x) {
    return sxtb16((x >> 8) | (x << 24));
}
inline uint32_t smlad(uint32_t a, uint32_t b, uint32_t acc) {
    return acc + static_cast<uint32_t>(lo16(a) * lo16(b) + hi16(a) * hi16(b));
}
inline uint32_t smlsd(uint32_t a, uint32_t b, uint32_t acc) {
    return acc + static_cast<uint32_t>(lo16(a) * lo16(b) - hi16(a) * hi16(b));
}
inline uint32_t smlsdx(uint32_t a, uint32_t b, uint32_t acc) {
    return acc + static_cast<uint32_t>(lo16(a) * hi16(b) - hi16(a) * lo16(b));
}
inline uint32_t pkhbt(uint32_t a, uint32_t b) {
    return pack16(lo16(a), lo16(b));
}
inline uint32_t pkhtb(uint32_t a, uint32_t b) {
    return pack16(hi16(a), hi16(b));
}
inline uint32_t sadd16(uint32_t a, uint32_t b) {
    return pack16(lo16(a) + lo16(b), hi16(a) + hi16(b));
}
#endif

/* Two I/Q samples (int8 I0 Q0 I1 Q1) at any 2 byte alignment. */
inline uint32_t load2(const int8_t* p) {
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

/* One I/Q sample as (I, Q) int16 halves: bytes I Q -> v | v << 8 has I in
 * byte 0 and Q in byte 2. */
inline uint32_t iq16(const int8_t* p) {
    uint16_t h;
    memcpy(&h, p, 2);
    const uint32_t v = h;
    return sxtb16(v | (v << 8));
}

/* Power of one 32 sample block through the 2 tap boxcar, decimated by 2:
 * sum of |x0 + x1|^2 over the 16 sample pairs (int8 interleaved, 4 byte
 * aligned). */
inline uint32_t block_power_box2(const uint32_t* w) {
    uint32_t acc = 0;
    for (size_t i = 0; i < kBlock / 2; i += 2) {
        const uint32_t a = w[i], b = w[i + 1];
        const uint32_t ia = sxtb16(a), ib = sxtb16(b);
        const uint32_t qa = sxtb16_ror8(a), qb = sxtb16_ror8(b);
        const uint32_t si = sadd16(pkhbt(ia, ib), pkhtb(ia, ib)); /* (I0 + I1, I2 + I3) */
        const uint32_t sq = sadd16(pkhbt(qa, qb), pkhtb(qa, qb));
        acc = smlad(si, si, acc);
        acc = smlad(sq, sq, acc);
    }
    return acc;
}

/* sum |x|^2 over n samples (any 2 byte alignment, n even). */
inline uint32_t power_sum(const int8_t* x, size_t n) {
    uint32_t e = 0;
    for (size_t t = 0; t < n; t += 2) {
        const uint32_t v = load2(x + 2 * t);
        const uint32_t ii = sxtb16(v), qq = sxtb16_ror8(v);
        e = smlad(ii, ii, e);
        e = smlad(qq, qq, e);
    }
    return e;
}

struct Candidate {
    uint32_t start;      /* stream sample of the on-time start (smoothing corrected) */
    uint32_t end;        /* ... of its end */
    uint32_t mean_pw_q4; /* mean box2 block power per sample over the on-time, x16 */
    uint32_t floor_q4;   /* the floor per sample, x16 (same scale) */
};

/* Stage A plus the capture. Baseband thread, one call per DMA buffer. */
class Gate {
   public:
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) void reset() {
        for (size_t i = 0; i < kSmooth; i++) hist_[i] = 0;
        for (size_t i = 0; i < kFloorSegs; i++) segstat_[i] = 0;
        hi_ = 0;
        ssum_ = 0;
        seg_blk_ = 0;
        seg_acc_ = 0;
        nseg_ = 0;
        floor_ = 0;
        on_thr_ = 0;
        off_thr_ = 0;
        on_ = false;
        on_block_ = 0;
        acc_ = 0;
        blk_ = 0;
        n_ = 0;
        capturing_ = false;
        snap_valid_ = false;
        snap_len_ = 0;
        snap_start_ = 0;
        snap_frozen_ = false;
        candidates_ = 0;
        busy_skips_ = 0;
    }

    /* One DMA buffer of kBuffer samples (int8 I,Q interleaved, 4 byte
     * aligned). prev: the previous buffer of the stream, still intact, or
     * nullptr (start, or a buffer was lost in between): a capture whose
     * pre-trigger part would need it is not started. Returns true when a
     * candidate was completed in this buffer: candidate() and the snapshot
     * stay untouched until release(). */
    bool feed(const int8_t* buf, const int8_t* prev) {
        const uint32_t* w = reinterpret_cast<const uint32_t*>(buf);
        bool got = false;
        /* A capture in progress takes the whole buffer first (all of it up to
         * this buffer's end is then in the snapshot when a falling edge is
         * found below). */
        if (capturing_) append(buf, 0, kBuffer);
        /* The per block state in registers (the snapshot writes would make
         * the compiler reload members); written back below. */
        uint32_t hi = hi_, ssum = ssum_, seg_blk = seg_blk_, seg_acc = seg_acc_, blk_abs = blk_;
        for (size_t blk = 0; blk < kSegBlocks; blk++, blk_abs++) {
            const uint32_t b = block_power_box2(w + blk * (kBlock / 2));
            ssum += b - hist_[hi];
            hist_[hi] = b;
            hi = (hi + 1) & (kSmooth - 1);
            seg_acc += b;
            if (++seg_blk == kSegBlocks) {
                segstat_[nseg_ % kFloorSegs] = (seg_acc << kSmoothShift) >> 6; /* mean over 64 blocks, x 16 */
                nseg_++;
                const size_t m = nseg_ < kFloorSegs ? nseg_ : kFloorSegs;
                uint32_t f = 0xFFFFFFFFu;
                for (size_t i = 0; i < m; i++)
                    if (segstat_[i] < f) f = segstat_[i];
                floor_ = f ? f : 1;
                on_thr_ = static_cast<uint64_t>(floor_) * kOnQ8;
                off_thr_ = static_cast<uint64_t>(floor_) * kOffQ8;
                seg_blk = 0;
                seg_acc = 0;
            }
            if (!floor_) continue;
            const uint64_t s256 = static_cast<uint64_t>(ssum) << 8;
            if (!on_) {
                if (s256 > on_thr_) {
                    on_ = true;
                    on_block_ = blk_abs;
                    acc_ = 0;
                    start_capture(buf, prev, blk_abs);
                }
            } else {
                acc_ += b;
                if (s256 < off_thr_) {
                    on_ = false;
                    const uint32_t st = (on_block_ - kSmooth / 2) * kBlock;
                    const uint32_t en = (blk_abs - kSmooth / 2) * kBlock;
                    const uint32_t d = en - st;
                    /* The snapshot must reach the on-time end. */
                    const bool snap_ok = snap_valid_ && !snap_frozen_ && en >= snap_start_ &&
                                         en - snap_start_ <= snap_len_;
                    capturing_ = false;
                    snap_valid_ = false;
                    if (d >= kDurLo && d <= kDurHi) {
                        candidates_++;
                        if (!snap_ok) {
                            busy_skips_++;
                        } else {
                            const uint32_t nb = blk_abs - on_block_;
                            cand_.start = st;
                            cand_.end = en;
                            cand_.mean_pw_q4 = static_cast<uint32_t>((static_cast<uint64_t>(acc_) * 16) / (nb * kBlock));
                            cand_.floor_q4 = (floor_ * 16) >> (kSmoothShift + 5);
                            snap_frozen_ = true;
                            got = true;
                        }
                    }
                }
            }
        }
        hi_ = hi;
        ssum_ = ssum;
        seg_blk_ = seg_blk;
        seg_acc_ = seg_acc;
        blk_ = blk_abs;
        n_ += kBuffer;
        return got;
    }

    /* The stream is not contiguous any more (a DMA buffer was lost): a
     * capture in progress is dropped. */
    void discontinuity() {
        capturing_ = false;
        snap_valid_ = false;
    }

    const Candidate& candidate() const { return cand_; }
    const int8_t* snapshot() const { return snap_; }
    uint32_t snapshot_start() const { return snap_start_; }
    bool snapshot_busy() const { return snap_frozen_; }
    void release() { snap_frozen_ = false; }
    uint32_t floor() const { return floor_; }
    /* Candidates by duration, and of them those not captured (the snapshot
     * was frozen for stage B, or the capture was dropped). */
    uint32_t candidates() const { return candidates_; }
    uint32_t busy_skips() const { return busy_skips_; }

   private:
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) void append(const int8_t* src, size_t from, size_t to) {
        size_t n = to - from;
        if (snap_len_ + n > kSnapSamples) n = kSnapSamples - snap_len_;
        /* one I/Q sample = 2 bytes, both sides 2 byte aligned: two samples per
         * word access (the M4 allows unaligned LDR / STR) */
        const int8_t* s = src + 2 * from;
        int8_t* d = snap_ + 2 * snap_len_;
        size_t i = 0;
        for (; i + 2 <= n; i += 2) {
            uint32_t v;
            memcpy(&v, s + 2 * i, 4);
            memcpy(d + 2 * i, &v, 4);
        }
        if (i < n) {
            d[2 * i] = s[2 * i];
            d[2 * i + 1] = s[2 * i + 1];
        }
        snap_len_ += n;
        if (snap_len_ >= kSnapSamples) capturing_ = false; /* full: keep what we have */
    }

    /* Rising edge in the current buffer (stream block blk_abs): the snapshot
     * starts kPre samples before the smoothing corrected start. */
    void start_capture(const int8_t* buf, const int8_t* prev, uint32_t blk_abs) {
        if (snap_frozen_) return; /* stage B still busy with the last one */
        const uint32_t st = (blk_abs - kSmooth / 2) * kBlock;
        const uint32_t a = st > kPre ? st - kPre : 0;
        if (a < n_ && !prev) return; /* the part before this buffer is gone */
        snap_start_ = a;
        snap_len_ = 0;
        capturing_ = true;
        snap_valid_ = true;
        if (a < n_) append(prev, kBuffer - (n_ - a), kBuffer);
        append(buf, a > n_ ? a - n_ : 0, kBuffer);
    }

    uint32_t hist_[kSmooth]{};
    uint32_t segstat_[kFloorSegs]{};
    uint32_t hi_{0}, ssum_{0}, seg_blk_{0}, seg_acc_{0}, nseg_{0}, floor_{0};
    uint64_t on_thr_{0}, off_thr_{0}; /* floor * on / off thresholds (q8) */
    bool on_{false};
    bool capturing_{false};
    bool snap_valid_{false};
    volatile bool snap_frozen_{false}; /* set here, cleared by the stage B thread */
    uint32_t on_block_{0};
    uint32_t acc_{0}; /* <= 11059 / 32 blocks * 2^21: fits */
    uint32_t blk_{0}; /* stream block counter (wraps after ~2.6 h at 15.36 Msps; a dwell resets it) */
    uint32_t n_{0};   /* stream samples before the current buffer */
    uint32_t snap_start_{0}, snap_len_{0};
    uint32_t candidates_{0}, busy_skips_{0};
    Candidate cand_{};
    alignas(4) int8_t snap_[2 * kSnapSamples]{};
};

struct Result {
    float cp;       /* CP coherence, 0..1 */
    float zc;       /* normalised ZC600 correlation, 0..1 */
    int32_t zc_lag; /* best lag relative to the prediction */
    int32_t grid_m; /* CP grid phase */
    int32_t uz;     /* predicted ZC600 useful start, snapshot samples */
    uint8_t nsym;   /* 8 (O1 / O4 layout) or 9 (O2) */
    bool det;
    float cfo_frac_hz; /* fractional carrier offset (within +-7.5 kHz) */
};

/* Stage B: CP fold, layout, ZC test on a frozen snapshot. */
class StageB {
   public:
    StageB() {
        uint32_t e = 0;
        for (size_t i = 0; i < 2 * kRefLen; i++) e += static_cast<uint32_t>(kZc600[i] * kZc600[i]);
        er_ = e;
    }

    /* x: snapshot (int8 interleaved), n samples of it valid; st/en: the
     * on-time relative to x. */
    __attribute__((optimize("fp-contract=off"))) Result evaluate(const int8_t* x, uint32_t n, uint32_t st, uint32_t en) {
        Result r{};
        constexpr uint32_t L = kLag, W = kWin, Ti = kSym;
        constexpr uint32_t J = Ti + W - 1;
        /* B1: Q[j] = sum over symbols k of q[j + k T] (j < J; only products
         * inside the snapshot), F[m] = Q[m] + ... + Q[m + W - 1] for m < T,
         * one pass in j with the last W values of Q in a ring. */
        uint32_t nsym_fold = 0;
        uint32_t off[kMaxFoldSyms];
        uint32_t lim[kMaxFoldSyms]; /* Q[j] takes symbol k for j < lim[k] */
        if (n > L) {
            const uint32_t nq = n - L;
            const uint32_t ns = (en - st) / kSym + 2;
            for (uint32_t k = 0; k < ns && k < kMaxFoldSyms; k++) {
                const uint32_t o = k * kSym;
                if (o >= nq) break;
                off[k] = o;
                lim[k] = (nq - o) < J ? (nq - o) : J;
                nsym_fold = k + 1;
            }
        }
        int32_t fr = 0, fi = 0;
        uint32_t fe = 0;
        float best = -1.f;
        uint32_t m = 0;
        int32_t best_fr = 0, best_fi = 0;
        for (uint32_t j = 0; j < J; j++) {
            int32_t qr = 0, qi = 0;
            uint32_t qe = 0;
            fold_at(x, j, off, lim, nsym_fold, qr, qi, qe);
            const uint32_t slot = j % W;
            if (j < W) {
                fr += qr;
                fi += qi;
                fe += qe;
            } else {
                fr += qr - ring_r_[slot];
                fi += qi - ring_i_[slot];
                fe += qe - ring_e_[slot];
            }
            ring_r_[slot] = qr;
            ring_i_[slot] = qi;
            ring_e_[slot] = qe;
            if (j + 1 < W) continue;
            /* F[mm] complete: mm = j - W + 1 */
            const uint32_t mm = j + 1 - W;
            if (mm == 0) {
                best_fr = fr;
                best_fi = fi;
            }
            if (fe == 0) continue;
            const float a = static_cast<float>(fr), b = static_cast<float>(fi);
            const float c = std::sqrt(a * a + b * b) / (0.5f * static_cast<float>(fe));
            if (c > best) {
                best = c;
                m = mm;
                best_fr = fr;
                best_fi = fi;
            }
        }
        r.cp = best < 0 ? 0.f : best;
        const float ph = std::atan2(static_cast<float>(best_fi), static_cast<float>(best_fr));
        r.cfo_frac_hz = -ph * static_cast<float>(kSampleRate) / (2.f * 3.14159265f * static_cast<float>(L));
        const uint32_t mg = (m + W) % Ti;
        r.grid_m = static_cast<int32_t>(mg);
        /* B2: symbol 0 is the grid point nearest the energy start + first CP:
         * k = floor((st + 80 - mg) / 1096 + 0.5), in integers. */
        r.nsym = (en - st) > kLayout ? 9 : 8;
        const int32_t v2 = 2 * (static_cast<int32_t>(st) + static_cast<int32_t>(kCp0) - static_cast<int32_t>(mg)) + static_cast<int32_t>(kSym);
        const int32_t k = floor_div(v2, 2 * static_cast<int32_t>(kSym));
        /* ZC600 in symbol 2 (8 symbols) or 3 (9 symbols); the on-time is
         * inflated by up to 14 blocks at high SNR, so test the duration's
         * layout first, then the other one, keep the larger. */
        const int32_t zi0 = r.nsym == 9 ? 3 : 2;
        r.zc = 0.f;
        r.zc_lag = 0;
        r.uz = u_of(mg, k + zi0);
        for (int32_t pass = 0; pass < 2; pass++) {
            const int32_t zi = pass == 0 ? zi0 : 5 - zi0;
            const int32_t uz = u_of(mg, k + zi);
            for (int32_t d = -kSpan; d <= kSpan; d++) {
                const int32_t a = uz + d;
                if (a < 0 || static_cast<uint32_t>(a) + kRefLen > n) continue;
                int32_t cr = 0, ci = 0;
                uint32_t ex = 0;
                correlate(x + 2 * a, cr, ci, ex);
                const float num = static_cast<float>(cr) * cr + static_cast<float>(ci) * ci;
                const float den = static_cast<float>(ex) * static_cast<float>(er_);
                const float v = den > 0 ? num / den : 0.f;
                if (v > r.zc) {
                    r.zc = v;
                    r.zc_lag = d;
                    r.uz = uz;
                }
            }
        }
        r.det = r.zc >= kZcMin;
        return r;
    }

    /* sum conj(ref) * x and sum |x|^2 over the reference length; x at any
     * 2 byte alignment. Two samples per step: SXTB16 splits I0 Q0 I1 Q1 into
     * (I0, I1) and (Q0, Q1), SMLAD multiplies and adds both halves. */
    void correlate(const int8_t* x, int32_t& cr, int32_t& ci, uint32_t& ex) const {
        const uint32_t* ref = reinterpret_cast<const uint32_t*>(kZc600);
        uint32_t a = 0, b1 = 0, b2 = 0, e = 0;
        for (size_t t = 0; t < kRefLen / 2; t++) {
            const uint32_t v = load2(x + 4 * t);
            const uint32_t ii = sxtb16(v), qq = sxtb16_ror8(v);
            const uint32_t rw = ref[t];
            const uint32_t rr = sxtb16(rw), rj = sxtb16_ror8(rw);
            a = smlad(ii, rr, a); /* I R + Q J */
            a = smlad(qq, rj, a);
            b1 = smlad(qq, rr, b1); /* Q R - I J */
            b2 = smlad(ii, rj, b2);
            e = smlad(ii, ii, e);
            e = smlad(qq, qq, e);
        }
        cr = static_cast<int32_t>(a);
        ci = static_cast<int32_t>(b1 - b2);
        ex = e;
    }

    uint32_t ref_energy() const { return er_; }

   private:
    /* The fold spans at most kDurHi / 1096 + 2 = 12 symbols. */
    static constexpr uint32_t kMaxFoldSyms = kDurHi / kSym + 2;

    static int32_t floor_div(int32_t a, int32_t b) {
        const int32_t q = a / b;
        return (a % b != 0 && a < 0) ? q - 1 : q;
    }

    static int32_t u_of(uint32_t mg, int32_t kz) {
        return kz >= 0 ? static_cast<int32_t>(mg + static_cast<uint32_t>(kz) * kSym)
                       : static_cast<int32_t>(mg) - static_cast<int32_t>(static_cast<uint32_t>(-kz) * kSym);
    }

    /* Q[j] = sum over the symbols k with j < lim[k] of
     * x[o + j] conj(x[o + j + L]), |x[o + j]|^2 + |x[o + j + L]|^2, o = off[k].
     * lim[] does not grow with k. With a = (ar, ai), b = (br, bi) as int16
     * pairs: SMLAD a.b = ar br + ai bi; SMLSDX adds ar bi - ai br, the
     * negated imaginary part. */
    static void fold_at(const int8_t* x, uint32_t j, const uint32_t* off, const uint32_t* lim, uint32_t ns, int32_t& qr, int32_t& qi, uint32_t& qe) {
        uint32_t r = 0, ni = 0, e = 0;
        for (uint32_t k = 0; k < ns; k++) {
            if (j >= lim[k]) break;
            const int8_t* p = x + 2 * (off[k] + j);
            const uint32_t a = iq16(p), b = iq16(p + 2 * kLag);
            r = smlad(a, b, r);
            ni = smlsdx(a, b, ni);
            e = smlad(a, a, e);
            e = smlad(b, b, e);
        }
        qr = static_cast<int32_t>(r);
        qi = -static_cast<int32_t>(ni);
        qe = e;
    }

    uint32_t er_{0};
    int32_t ring_r_[kWin]{};
    int32_t ring_i_[kWin]{};
    uint32_t ring_e_[kWin]{};
};

/* Gate + stage B, the whole detector's memory (lives in PSDT's mode arena). */
struct Detector {
    Gate gate{};
    StageB stage_b{};
};

}  // namespace droneid
}  // namespace sigdet

#endif /*__SIGDET_DRONEID_H__*/
