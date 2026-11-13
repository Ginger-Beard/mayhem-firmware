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

/* DSP kernels of the Signature Detect baseband (PSDT). Header only and free of
 * ChibiOS, so firmware/test/baseband can run them on the host.
 *
 * GfskDemod: BLE 1M PHY advertising receiver (legacy adverts, 37 byte max
 * payload), used for Remote ID (ASTM F3411 / Open Drone ID over BLE 4).
 * PatternDemod: the same front end (GfskFrontEnd) with a configurable sync
 * word instead of the access address (pattern scan: RC toys and other
 * 1 Mb/s GFSK links); its description is at the class.
 *
 * Where the channel sits: the M0 tunes through ReceiverModel, whose
 * tuning_offset() puts the LO a quarter of the sample rate below the target
 * for every image that doesn't take RxFs4Config. At 4 Msps the BLE channel is
 * therefore at +1 MHz in the baseband, as for proc_btlerx (which shifts it down
 * with FIRC8xR16x24FS4Decim4). On PRALINE the analogue LO is a further
 * afe_rate / 4 (8 MHz at 4 Msps) away, the FPGA rotates that back and blocks
 * DC, so what is left at 0 Hz is a residue, not the raw LO leak; on a
 * HackRF One it would be the LO leak itself. Either way it is 1 MHz below the
 * channel, and the neighbours 2 MHz either side are at -1 MHz and at +3 MHz,
 * which aliases onto -1 MHz with whatever the decimator leaves of it.
 *
 * Channel filter, in one go with the shift to 0 Hz:
 *     y1[n] = sum_{k=0..3} x[n-k] (-j)^(n-k)     (shift by -fs/4, 4 tap boxcar)
 *     y[n]  = y1[n] + y1[n-1]                    (2 tap boxcar)
 * Written as y = (-j)^n z the rotation drops out of everything downstream:
 *     z1[n] = j z1[n-1] + x[n] - x[n-4]          z[n] = z1[n] + j z1[n-1]
 * The 4 tap boxcar has zeros at +-1 and +-2 MHz from the channel, the 2 tap one
 * a second zero at +-2 MHz, so the DC residue (-1 MHz) and both neighbours sit
 * in nulls and their skirts are 18 dB down at 1.4 MHz. Integer adds only.
 *
 * Discriminator: d[n] = Im(y[n] conj(y[n-1])) = -Re(z[n] conj(z[n-1])) =
 * |y|^2 sin(2 pi f / fs), no atan, no normalisation, then m[n] = d[n] + d[n-1].
 * With x in int8: |z| components <= 1024, |d| < 2^21, |m| < 2^22, all int32.
 * (Of the filter / post-sum combinations tried in the host test this one had
 * the best sensitivity, CFO tolerance and adjacent channel rejection together;
 * a 4 tap boxcar alone loses 0 dB adjacent interference, a 4+4 cascade loses
 * 3 dB at +-100 kHz CFO.)
 *
 * Sync: the sign of m[n] - m[n-4] (one symbol earlier) is shifted into a 32
 * bit register for sample phases 0 and 2. Where the access address has a
 * transition (18 of its 32 bits, counting the step from the preamble) that
 * sign is known, and a frequency offset, which only adds a constant to m,
 * cancels out. All 18 as expected is a trigger (allowing one error gained
 * nothing in the host test). Then every sample from 2 before to 6 after the
 * trigger's symbol start is tried as the end of the access address: its 32
 * soft values sliced at the mean of their own "1" and "0" levels may have at
 * most one wrong bit, and of those that pass the one with the widest eye
 * (mean "1" minus mean "0") is taken, so all 4 phases compete and the best
 * one wins. Noise triggers some 60 to 70 times a second (more than 2^-18 of
 * the 2 million tests: successive signs of m[n] - m[n-4] are anticorrelated,
 * and the access address' transitions mostly alternate) and passes the soft
 * check about 1 in 10^8.
 *
 * Slicer threshold for the packet: the mean of the 8 preamble symbols. They
 * alternate, so it is the midpoint of the eye whatever the inter-symbol
 * interference (the access address' own midpoint is off by a few percent of
 * the eye: its zeros are more often isolated than its ones). That threshold is
 * the carrier offset, so no CFO correction is needed before slicing.
 *
 * Then 8 * (2 + length + 3) bits are sliced on the chosen phase, dewhitened
 * (LFSR from the channel index) and checked: length 6..37, then CRC24 with
 * init 0x555555. All state lives in the object, so a packet may straddle any
 * number of execute() calls. The transition registers keep running while a
 * packet is demodulated, so a false sync that dies at the length check costs
 * at most the 16 header symbols of search.
 *
 * Per packet: mean |z|^2 at the bit decisions (RSSI) and the slicer threshold
 * over that power (CFO).
 *
 * Cost (Cortex-M4 at 200 MHz, 4 Msps, 512 us per 2048 sample DMA buffer): the
 * work runs in blocks of up to 512 samples, in two passes. filter() is the
 * channel filter and discriminator alone, about 11 cycles a sample with all
 * its state in registers; detect() runs the transition registers over the
 * block, 2 loads and a few ALU operations a symbol, and calls into the
 * packet state machine only around triggers and while a packet is being
 * demodulated. (One pass that did both needed 17 live registers and spilled
 * to the stack at every sample: 31 cycles a sample, 2/3 of the M4.) A soft
 * check is 32 loads and adds with the access address unrolled into the code,
 * and gives up at the second wrong bit, so a trigger costs 1000 to 3000
 * cycles rather than 12000. The ring keeps a copy of its last 160 entries
 * in front of its start, so every look back (at most 40 symbols) is one
 * contiguous run of memory. */

#ifndef __SIGDET_DSP_H__
#define __SIGDET_DSP_H__

#include <cstddef>
#include <cstdint>

namespace sigdet {

constexpr uint32_t kBleAdvAccessAddress = 0x8E89BED6;
constexpr uint32_t kBleAdvCrcInit = 0x555555;
constexpr size_t kBleMinAdvPayload = 6;   // AdvA only
constexpr size_t kBleMaxAdvPayload = 37;  // AdvA + 31 bytes AdvData
constexpr size_t kBlePduMax = 2 + kBleMaxAdvPayload + 3;
constexpr uint32_t kGfskSampleRate = 4000000;
constexpr size_t kGfskSamplesPerSymbol = 4;

/* PDU types of the advertising channel. */
enum BleAdvPduType : uint8_t {
    ADV_IND = 0,
    ADV_DIRECT_IND = 1,
    ADV_NONCONN_IND = 2,
    SCAN_REQ = 3,
    SCAN_RSP = 4,
    CONNECT_IND = 5,
    ADV_SCAN_IND = 6,
};

/* Whitening LFSR x^7 + x^4 + 1 as the Core spec draws it, position i in bit i:
 * position 0 is 1, positions 1..6 hold the channel index, MSB in position 1. */
constexpr uint8_t ble_whiten_init(uint8_t channel) {
    uint8_t r = 1;
    for (int i = 0; i < 6; i++)
        if (channel & (0x20 >> i)) r |= 2 << i;
    return r;
}

/* Next whitening bit; the register is advanced. */
inline uint32_t ble_whiten_bit(uint8_t& r) {
    const uint32_t w = (r >> 6) & 1;
    r = ((r << 1) & 0x7F) | w;
    if (w) r ^= 0x10;
    return w;
}

/* Whitens or dewhitens (the same thing) len bytes in place, LSB first. */
inline void ble_whiten(uint8_t channel, uint8_t* data, size_t len) {
    uint8_t r = ble_whiten_init(channel);
    for (size_t i = 0; i < len; i++) {
        uint8_t w = 0;
        for (int b = 0; b < 8; b++) w |= ble_whiten_bit(r) << b;
        data[i] ^= w;
    }
}

constexpr uint32_t reverse24(uint32_t v) {
    uint32_t r = 0;
    for (int i = 0; i < 24; i++)
        if (v & (1u << i)) r |= 1u << (23 - i);
    return r;
}

/* CRC24 x^24+x^10+x^9+x^6+x^4+x^3+x+1 over the PDU bits in air order (LSB of
 * each byte first). The result compares with the three CRC bytes as received:
 * crc0 | crc1 << 8 | crc2 << 16. */
inline uint32_t ble_crc24(const uint8_t* data, size_t len, uint32_t init = kBleAdvCrcInit) {
    uint32_t state = reverse24(init);
    for (size_t i = 0; i < len; i++) {
        uint32_t cur = data[i];
        for (int b = 0; b < 8; b++) {
            const uint32_t next = (state ^ cur) & 1;
            cur >>= 1;
            state >>= 1;
            if (next) state ^= 0xDA6000;
        }
    }
    return state;
}

/* 1000 * log10(p), p > 0, to within about 0.15 dB (centi-dB). */
inline int32_t power_to_cdb(uint32_t p) {
    static constexpr uint8_t frac[32] = {0, 11, 22, 33, 44, 54, 63, 73, 82, 92, 100, 109, 118, 126, 134, 142,
                                         150, 157, 165, 172, 179, 186, 193, 200, 207, 213, 220, 226, 232, 238, 244, 250};
    if (p == 0) return -10000;
    const int32_t l = 31 - __builtin_clz(p);
    const uint32_t m = (l >= 5) ? (p >> (l - 5)) & 31 : (p << (5 - l)) & 31;
    const int32_t log2_q8 = l * 256 + frac[m];
    return (log2_q8 * 30103) / 25600;  // 1000 * log10(2) / 256
}

/* |z|^2 of a full scale (|x| = 128) tone at the channel centre is (8 * 128)^2,
 * 60.21 dB; the filter takes 0.7 dB off a BLE signal, so a full scale BLE
 * signal reads 59.51 dB at the bit decisions. */
constexpr int32_t kFullScaleCdb = 5951;

/* Bit t: the access address has a transition into its bit t (bit -1 is the
 * last preamble bit, the complement of the AA's LSB). */
constexpr uint32_t ble_aa_transition_mask() {
    uint32_t mask = 0;
    uint32_t prev = (~kBleAdvAccessAddress) & 1;
    for (int t = 0; t < 32; t++) {
        const uint32_t b = (kBleAdvAccessAddress >> t) & 1;
        if (b != prev) mask |= 1u << t;
        prev = b;
    }
    return mask;
}

/* Packed pairs of int16 (I in the low half, Q in the high half) and the few
 * Cortex-M4 SIMD instructions the demodulator needs, with plain C versions for
 * the host test. */
#if defined(__ARM_FEATURE_SIMD32)
/* GCC 9 has no ACLE SIMD32 intrinsics; same inline asm as CMSIS. */
inline uint32_t simd_sxtb16(uint32_t x) {
    uint32_t r;
    __asm__("sxtb16 %0, %1" : "=r"(r) : "r"(x));
    return r;
}
inline uint32_t simd_sxtb16_ror8(uint32_t x) {
    uint32_t r;
    __asm__("sxtb16 %0, %1, ror #8" : "=r"(r) : "r"(x));
    return r;
}
inline uint32_t simd_ssub16(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("ssub16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
/* lo = a.lo - b.hi, hi = a.hi + b.lo */
inline uint32_t simd_sasx(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("sasx %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
inline int32_t simd_smuad(uint32_t a, uint32_t b) {
    int32_t r;
    __asm__("smuad %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
/* (a.lo, b.lo) */
inline uint32_t simd_pack_lo(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("pkhbt %0, %1, %2, lsl #16" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
/* (a.hi, b.hi) */
inline uint32_t simd_pack_hi(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("pkhtb %0, %2, %1, asr #16" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
#else
inline int32_t lo16(uint32_t x) {
    return static_cast<int16_t>(x & 0xFFFF);
}
inline int32_t hi16(uint32_t x) {
    return static_cast<int16_t>(x >> 16);
}
inline uint32_t pack16(int32_t lo, int32_t hi) {
    return (static_cast<uint32_t>(lo) & 0xFFFF) | (static_cast<uint32_t>(hi) << 16);
}
inline uint32_t simd_sxtb16(uint32_t x) {
    return pack16(static_cast<int8_t>(x & 0xFF), static_cast<int8_t>((x >> 16) & 0xFF));
}
inline uint32_t simd_sxtb16_ror8(uint32_t x) {
    return simd_sxtb16((x >> 8) | (x << 24));
}
inline uint32_t simd_ssub16(uint32_t a, uint32_t b) {
    return pack16(lo16(a) - lo16(b), hi16(a) - hi16(b));
}
/* lo = a.lo - b.hi, hi = a.hi + b.lo */
inline uint32_t simd_sasx(uint32_t a, uint32_t b) {
    return pack16(lo16(a) - hi16(b), hi16(a) + lo16(b));
}
inline int32_t simd_smuad(uint32_t a, uint32_t b) {
    return lo16(a) * lo16(b) + hi16(a) * hi16(b);
}
inline uint32_t simd_pack_lo(uint32_t a, uint32_t b) {
    return (a & 0xFFFF) | (b << 16);
}
inline uint32_t simd_pack_hi(uint32_t a, uint32_t b) {
    return (a >> 16) | (b & 0xFFFF0000);
}
#endif

struct BlePacket {
    uint8_t pdu[kBlePduMax]; /* dewhitened header, payload, CRC */
    uint8_t length;          /* payload length, 6..37 */
    uint8_t channel;
    uint8_t phase;       /* sample phase used, 0..3 */
    uint8_t aa_errors;   /* access address bits wrong after the soft check, 0..1 */
    int16_t rssi_cdb;    /* mean in-channel power over the packet, centi-dB re full scale */
    int16_t cfo_khz;     /* carrier offset from the channel centre */
    uint32_t end_sample; /* demodulator's sample count (wrapping) at the last CRC bit */

    uint8_t pdu_type() const { return pdu[0] & 0x0F; }
};

/* The front end both receivers share (GfskDemod for BLE, PatternDemod for a
 * plain sync word): 4 Msps int8 I/Q with the channel at +fs/4, taken in
 * aligned groups of 4 samples (one symbol at 1 Mb/s), channel filter and
 * discriminator (filter(), see the top of this file) into a ring of u = -m,
 * and |z|^2 of each group's last sample in pw_. The receiver D gets every
 * block of up to 512 samples (never across the end of the ring) through
 * D::detect(groups, args...): its samples are at(n_) .. at(n_ + 4 groups - 1),
 * and D::detect() advances n_ past them. CRTP, so the hot loops stay what
 * they were before the split (no virtual call, everything inlined). */
template <class D>
class GfskFrontEnd {
   public:
    /* Bring-up aid: demodulate the conjugate (mirror the spectrum), in case
     * the front end's I/Q come out swapped. Off normally. */
    void set_conjugate(bool on) { conjugate_ = on; }

   protected:
    /* Ring of discriminator values, a power of two > the 512 sample block +
     * the deepest look back, with that look back mirrored in front of it. */
    static constexpr size_t kRingSize = 1024;
    static constexpr uint32_t kRingMask = kRingSize - 1;
    static constexpr size_t kGuard = 160;       /* preamble start: 40 symbols back */
    static constexpr size_t kBlockGroups = 128; /* 512 samples */
    static_assert(kBlockGroups * 4 + kGuard + 8 <= kRingSize, "a block must not overwrite what it looks back at");

    /* Discriminator value of sample n, or of up to kGuard samples before it
     * through the returned pointer (negative offsets). */
    int32_t* at(uint32_t n) { return ring_ + kGuard + (n & kRingMask); }
    const int32_t* at(uint32_t n) const { return ring_ + kGuard + (n & kRingMask); }
    /* |z|^2 of the group holding sample n (the ring's last 256 groups). */
    uint32_t group_power(uint32_t n) const { return pw_[(n >> 2) & (kRingMask >> 2)]; }

    /* iq: count interleaved int8 I/Q pairs. Works in aligned groups of 4
     * samples; up to 3 samples are kept for the next call when count isn't a
     * multiple of 4. */
    template <class... A>
    void run(const int8_t* iq, size_t count, A... a) {
        size_t i = 0;
        if (staged_) {
            while (staged_ < 4 && i < count) stage(iq, i++);
            if (staged_ < 4) return;
            run_groups(stage_, 1, a...);
            staged_ = 0;
        }
        const size_t groups = (count - i) / 4;
        if (groups) run_groups(iq + 2 * i, groups, a...);
        i += 4 * groups;
        while (i < count) stage(iq, i++);
    }

    /* Drops the filter history and the ring (on a retune). (Loops kept as
     * loops: newlib-nano's memset, which GCC would call here, stores byte by
     * byte, 30000 cycles for the ring.) */
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) void reset_front() {
        staged_ = 0;
        z1_ = z_ = 0;
        u_ = 0;
        for (size_t k = 0; k < 4; k++) old_[k] = 0;
        for (size_t i = 0; i < kGuard + kRingSize; i++) ring_[i] = 0;
        for (size_t i = 0; i < kRingSize / 4; i++) pw_[i] = 0;
    }

    uint32_t n_{0}; /* sample count (wrapping) of the next block's first sample */

   private:
    template <class... A>
    void run_groups(const int8_t* p, size_t groups, A... a) {
        while (groups) {
            const uint32_t pos = n_ & kRingMask;
            if (pos == 0) mirror_guard();
            size_t block = (kRingSize - pos) / 4;
            if (block > kBlockGroups) block = kBlockGroups;
            if (block > groups) block = groups;
            if (conjugate_)
                filter<true>(p, block);
            else
                filter<false>(p, block);
            static_cast<D*>(this)->detect(block, a...);
            p += 8 * block;
            groups -= block;
        }
    }

    /* New lap of the ring: the tail of the last one goes in front. (A loop:
     * newlib-nano's memcpy, which GCC would put here, goes byte by byte.) */
    __attribute__((noinline, optimize("no-tree-loop-distribute-patterns"))) void mirror_guard() {
        struct Words8 {
            int32_t v[8];
        };
        static_assert(kGuard % 8 == 0, "guard in whole Words8");
        Words8* const dst = reinterpret_cast<Words8*>(ring_);
        const Words8* const src = reinterpret_cast<const Words8*>(ring_ + kRingSize);
        for (size_t i = 0; i < kGuard / 8; i++) dst[i] = src[i];
    }

    void stage(const int8_t* iq, size_t i) {
        stage_[2 * staged_] = iq[2 * i];
        stage_[2 * staged_ + 1] = iq[2 * i + 1];
        staged_++;
    }

    /* Pass 1 over `groups` times 4 samples (groups > 0, not across the end of
     * the ring): x - x[n-4] and the two boxcars are three packed adds per
     * sample, the discriminator one dual multiply. The ring gets u = -m (m:
     * discriminator, positive for a "1"), pw_ |z|^2 of each group's last
     * sample for the packet power. (Out of line, like the receivers' detect():
     * each loop gets all the registers.) */
    template <bool kConjugate>
    __attribute__((noinline)) void filter(const int8_t* p, size_t groups) {
        uint32_t z1p = z1_, zp = z_;
        int32_t up = u_;
        uint32_t o0 = old_[0], o1 = old_[1], o2 = old_[2], o3 = old_[3];
        uint32_t n0, n1, n2, n3;
        int32_t* r = at(n_);
        uint32_t* q = pw_ + ((n_ >> 2) & (kRingMask >> 2));

/* One group of 4 samples at p + OFF: the new x[n] into N0..N3, x[n-4] from
 * O0..O3. Two of them a turn, with the roles swapped, so the comb's history
 * changes registers without moves. */
#define SIGDET_GFSK_STEP(OFF, J, S, O)                         \
    const uint32_t z1_##J = simd_sasx(simd_ssub16(S, O), z1p); \
    const uint32_t z_##J = simd_sasx(z1_##J, z1p);             \
    z1p = z1_##J;                                              \
    const int32_t u_##J = simd_smuad(z_##J, zp);               \
    zp = z_##J;                                                \
    r[OFF / 2 + J] = u_##J + up;                               \
    up = u_##J;
#define SIGDET_GFSK_GROUP(OFF, N0, N1, N2, N3, O0, O1, O2, O3)          \
    {                                                                   \
        uint32_t w0, w1;                                                \
        __builtin_memcpy(&w0, p + OFF, 4);                              \
        __builtin_memcpy(&w1, p + OFF + 4, 4);                          \
        /* [I0 Q0 I1 Q1] -> (I0, I1), (Q0, Q1) -> (I0, Q0), (I1, Q1) */ \
        const uint32_t a0 = simd_sxtb16(w0), a1 = simd_sxtb16(w1);      \
        uint32_t b0 = simd_sxtb16_ror8(w0), b1 = simd_sxtb16_ror8(w1);  \
        if (kConjugate) {                                               \
            b0 = simd_ssub16(0, b0);                                    \
            b1 = simd_ssub16(0, b1);                                    \
        }                                                               \
        N0 = simd_pack_lo(a0, b0);                                      \
        N1 = simd_pack_hi(a0, b0);                                      \
        N2 = simd_pack_lo(a1, b1);                                      \
        N3 = simd_pack_hi(a1, b1);                                      \
        SIGDET_GFSK_STEP(OFF, 0, N0, O0)                                \
        SIGDET_GFSK_STEP(OFF, 1, N1, O1)                                \
        SIGDET_GFSK_STEP(OFF, 2, N2, O2)                                \
        SIGDET_GFSK_STEP(OFF, 3, N3, O3)                                \
        q[OFF / 8] = static_cast<uint32_t>(simd_smuad(zp, zp));         \
    }

        if (groups & 1) {
            SIGDET_GFSK_GROUP(0, n0, n1, n2, n3, o0, o1, o2, o3)
            o0 = n0;
            o1 = n1;
            o2 = n2;
            o3 = n3;
            p += 8;
            r += 4;
            q += 1;
        }
        for (const int8_t* const end = p + 8 * (groups & ~static_cast<size_t>(1)); p != end; p += 16, r += 8, q += 2) {
            SIGDET_GFSK_GROUP(0, n0, n1, n2, n3, o0, o1, o2, o3)
            SIGDET_GFSK_GROUP(8, o0, o1, o2, o3, n0, n1, n2, n3)
        }
#undef SIGDET_GFSK_GROUP
#undef SIGDET_GFSK_STEP

        z1_ = z1p;
        z_ = zp;
        u_ = up;
        old_[0] = o0;
        old_[1] = o1;
        old_[2] = o2;
        old_[3] = o3;
    }

   protected:
    uint32_t pw_[kRingSize / 4]{}; /* |z|^2 per group */

   private:
    uint32_t z1_{0}, z_{0}; /* packed */
    int32_t u_{0};
    uint32_t old_[4]{}; /* packed x[n-4] per phase */
    int32_t ring_[kGuard + kRingSize]{};
    alignas(4) int8_t stage_[8]{};
    uint32_t staged_{0};
    bool conjugate_{false};
};

class GfskDemod : public GfskFrontEnd<GfskDemod> {
   public:
    using Callback = void (*)(void* context, const BlePacket& packet);

    struct Stats {
        uint32_t aa_hits;  /* syncs that started a packet */
        uint32_t len_bad;  /* ... and died at the length check */
        uint32_t crc_bad;  /* ... and failed the CRC */
        uint32_t ok;       /* ... and passed it */
        uint32_t triggers; /* transition matches that ran the soft check (aa_hits of them passed) */
        uint32_t capped;   /* ... that were ignored: kMaxSyncsPerCall reached */
        uint32_t bits;     /* bits demodulated (all packets, good or bad) */
    };

    /* Soft checks per execute() call at most (about 3000 cycles each when a
     * strong signal passes all 9 candidates). Real adverts are 80 symbols or
     * more, so a 2048 sample buffer holds at most 7 of them; noise triggers
     * come at some tens per second. The cap only bounds the work under
     * pathological interference. */
    static constexpr uint32_t kMaxSyncsPerCall = 8;

    void configure(uint8_t channel) {
        channel_ = channel;
        for (size_t i = 0; i < kBlePduMax; i++) whiten_[i] = 0;
        ble_whiten(channel, whiten_, kBlePduMax);
        reset();
    }

    /* Drops any packet in progress and the filter history (on a retune). */
    void reset() {
        state_ = State::Search;
        pending_ = false;
        tr_[0] = tr_[1] = 0;
        reset_front();
    }

    const Stats& stats() const { return stats_; }
    void clear_stats() { stats_ = Stats{}; }
    uint8_t channel() const { return channel_; }

    /* iq: count interleaved int8 I/Q pairs at 4 Msps, channel at +fs/4. Works
     * in aligned groups of 4 samples (one symbol); up to 3 samples are kept
     * for the next call when count isn't a multiple of 4. */
    void execute(const int8_t* iq, size_t count, Callback callback, void* context) {
        sync_budget_ = kMaxSyncsPerCall;
        run(iq, count, callback, context);
    }

   private:
    friend class GfskFrontEnd<GfskDemod>;

    enum class State : uint8_t {
        Search,
        Demod
    };

    /* Bit t set where that transition goes 0 -> 1 (frequency rises). */
    static constexpr uint32_t kTrMask = ble_aa_transition_mask();
    static constexpr uint32_t kTrExpect = kBleAdvAccessAddress & kTrMask;

    /* Pass 2 over the same groups: the sign of m[n] - m[n-4] into the
     * transition registers of phases 0 and 2, a trigger where all 18
     * transitions of the access address are as expected. The packet state
     * machine (step()) runs only for groups with a trigger, a pending sync or
     * a packet in progress. */
    __attribute__((noinline)) void detect(size_t groups, Callback callback, void* context) {
        const int32_t* r = at(n_);
        const int32_t* const first = r;
        const int32_t* const end = r + 4 * groups;
        Transitions tr{r[-4], r[-2], tr_[0], tr_[1]};
        /* In registers rather than rebuilt from immediates at every use. */
        uint32_t tr_mask = kTrMask, tr_expect = kTrExpect;
        __asm__("" : "+r"(tr_mask), "+r"(tr_expect));

        while (r != end) {
            if (state_ == State::Demod) {
                r = demod_groups(r, end, n_ + static_cast<uint32_t>(r - first), tr, callback, context);
                continue;
            }
            /* Search: nothing but the transition registers until a trigger
             * (or, with a sync pending, the next group). */
            const bool waiting = pending_;
            int32_t m0p = tr.m0p, m2p = tr.m2p;
            uint32_t t0 = tr.t0, t2 = tr.t2;
            bool trigger = false;
            /* Two groups a turn, so m0p and m2p trade registers without moves. */
#define SIGDET_GFSK_SEARCH(M0, M2, M0P, M2P)                                 \
    M0 = r[0];                                                               \
    M2 = r[2];                                                               \
    t0 = (t0 >> 1) | sign_bit(M0, M0P);                                      \
    t2 = (t2 >> 1) | sign_bit(M2, M2P);                                      \
    trigger = ((t0 & tr_mask) == tr_expect) | ((t2 & tr_mask) == tr_expect); \
    r += 4;
            int32_t m0q, m2q;
            while (r != end) {
                SIGDET_GFSK_SEARCH(m0q, m2q, m0p, m2p)
                if (__builtin_expect(trigger || waiting || r == end, 0)) {
                    m0p = m0q;
                    m2p = m2q;
                    break;
                }
                SIGDET_GFSK_SEARCH(m0p, m2p, m0q, m2q)
                if (__builtin_expect(trigger || waiting, 0)) break;
            }
#undef SIGDET_GFSK_SEARCH
            tr = Transitions{m0p, m2p, t0, t2};
            if (trigger || waiting) step(n_ + static_cast<uint32_t>(r - first) - 4, trigger, callback, context);
        }

        n_ += 4 * groups;
        tr_[0] = tr.t0;
        tr_[1] = tr.t2;
    }

    /* Transition registers of phases 0 and 2 and the m[n-4] they compare with. */
    struct Transitions {
        int32_t m0p, m2p;
        uint32_t t0, t2;
    };

    /* Bit 31 set where a < b (|m| < 2^22: the difference can't wrap). */
    static uint32_t sign_bit(int32_t a, int32_t b) {
        return (static_cast<uint32_t>(a) - static_cast<uint32_t>(b)) & 0x80000000u;
    }

    /* A packet in progress, from the group at r (sample n): one bit per group,
     * at the same offset in each, with the packet state in registers. The
     * transition registers keep running (their triggers are ignored, as
     * they are while a packet is demodulated). Returns where it stopped: the
     * block's end, or the group after the packet's last (or length) bit. */
    __attribute__((noinline)) const int32_t* demod_groups(const int32_t* r, const int32_t* const end, uint32_t n, Transitions& tr, Callback callback, void* context) {
        const int32_t* const start = r;
        const uint32_t d = next_bit_ - n; /* 0..3: step() has caught up */
        const int32_t neg_bias = -bias_;  /* "1": -u > bias, i.e. u < -bias */
        uint32_t bits = bits_, raw = raw_;
        int32_t m0p = tr.m0p, m2p = tr.m2p;
        uint32_t t0 = tr.t0, t2 = tr.t2;
        bool done = false;

        while (r != end) {
            const int32_t m0 = r[0], m2 = r[2];
            t0 = (t0 >> 1) | sign_bit(m0, m0p);
            t2 = (t2 >> 1) | sign_bit(m2, m2p);
            m0p = m0;
            m2p = m2;
            raw |= static_cast<uint32_t>(r[d] < neg_bias) << (bits & 7);
            bits++;
            r += 4;
            if ((bits & 7) == 0) {
                if (byte_done(bits, raw)) {
                    if (state_ == State::Search) break; /* length */
                    if (bits == total_bits_) {
                        done = true;
                        break;
                    }
                }
            }
        }

        /* Packet power: |z|^2 of each of these groups (no wrap inside a block). */
        const uint32_t* q = pw_ + ((n >> 2) & (kRingMask >> 2));
        uint32_t power = power_;
        for (const int32_t* g = start; g != r; g += 4) power += *q++;
        power_ = power;

        bits_ = bits;
        raw_ = raw;
        next_bit_ = n + static_cast<uint32_t>(r - start) + d;
        stats_.bits += static_cast<uint32_t>(r - start) / 4;
        tr = Transitions{m0p, m2p, t0, t2};
        if (done) finish(n + static_cast<uint32_t>(r - start) - 1, callback, context);
        return r;
    }

    /* The packet state machine for the group starting at sample n. */
    __attribute__((noinline)) void step(uint32_t n, bool trigger, Callback callback, void* context) {
        if (state_ == State::Search) {
            if (trigger && !pending_) {
                if (sync_budget_) {
                    sync_budget_--;
                    pending_ = true;
                    pending_n_ = n;
                } else {
                    stats_.capped++;
                }
            }
            /* Candidates up to 6 samples after the trigger group's start must be in. */
            if (pending_ && static_cast<int32_t>(n + 3 - (pending_n_ + 6)) >= 0) sync();
        }
        /* One decision per group, two when catching up after a sync; |z|^2 of
         * the group's last sample for the packet power. */
        while (state_ == State::Demod && static_cast<int32_t>(n + 3 - next_bit_) >= 0) {
            power_ += pw_[(n >> 2) & (kRingMask >> 2)];
            if (demod_bit(-*at(next_bit_))) finish(n + 3, callback, context);
        }
    }

    /* Soft check of the access address ending at sample n. AA has 18 ones and 14
     * zeros: mean1 = S1 / 18, mean0 = S0 / 14, so 252 * midpoint = 7 S1 + 9 S0
     * and 252 * eye = 7 S1 - 9 S0 (S: sums of v = -u). Returns the eye, 0 if it
     * fails. |u| <= 2 * 2 * 1024^2 = 2^22, so 252 |u| and both sums times 7 or
     * 9 stay below 2^30: int32 is exact. Unrolled, so the AA bits are code. */
    static_assert(__builtin_popcount(kBleAdvAccessAddress) == 18, "AA ones");

    int32_t soft_check(uint32_t n, uint32_t& errors) const {
        const int32_t* const e = at(n) - 4 * 31; /* e[4 t]: bit t */
        int32_t s1 = 0, s0 = 0;                  /* sums of u */
#pragma GCC unroll 32
        for (uint32_t t = 0; t < 32; t++) {
            if ((kBleAdvAccessAddress >> t) & 1)
                s1 += e[4 * t];
            else
                s0 += e[4 * t];
        }
        const int32_t eye = 9 * s0 - 7 * s1;
        if (eye <= 0) return 0;
        /* v > midpoint <=> 252 u < 7 s1 + 9 s0 */
        const int32_t thr = 7 * s1 + 9 * s0;
        uint32_t err = 0;
#pragma GCC unroll 32
        for (uint32_t t = 0; t < 32; t++) {
            const uint32_t b = (252 * e[4 * t] < thr) ? 1 : 0;
            err += b ^ ((kBleAdvAccessAddress >> t) & 1);
            if (err > 1) return 0;
        }
        errors = err;
        return eye;
    }

    /* A trigger was seen in the group starting at pending_n_ (phase 0 or 2).
     * Every sample from 2 before it to 6 after is a candidate end of the access
     * address; the widest eye wins. */
    __attribute__((noinline)) void sync() {
        pending_ = false;
        stats_.triggers++;
        int32_t best_eye = 0;
        uint32_t best_n = 0, best_errors = 0;
        for (uint32_t c = pending_n_ - 2; c != pending_n_ + 7; c++) {
            uint32_t errors = 0;
            const int32_t eye = soft_check(c, errors);
            if (eye > best_eye) {
                best_eye = eye;
                best_n = c;
                best_errors = errors;
            }
        }
        if (best_eye <= 0) return;

        /* Slicer threshold: the 8 preamble symbols before the AA, 0 1 0 1 0 1 0 1. */
        const int32_t* const e = at(best_n);
        int32_t pre = 0;
        for (uint32_t t = 32; t < 40; t++) pre -= e[-4 * static_cast<int32_t>(t)];
        cand_n_ = best_n;
        cand_bias_ = pre / 8;
        cand_errors_ = best_errors;
        commit();
    }

    void commit() {
        state_ = State::Demod;
        bias_ = cand_bias_;
        next_bit_ = cand_n_ + 4;
        bits_ = 0;
        total_bits_ = 16;
        raw_ = 0;
        power_ = 0;
        for (size_t i = 0; i < kBlePduMax; i++) pdu_[i] = 0;
        stats_.aa_hits++;
    }

    /* A byte of the PDU is in (bits a multiple of 8): dewhiten it into pdu_
     * and clear raw. True after the header (the packet may have ended at the
     * length check: state_ Search) and when the packet is complete. */
    bool byte_done(uint32_t bits, uint32_t& raw) {
        const uint32_t i = (bits >> 3) - 1;
        pdu_[i] = static_cast<uint8_t>(raw ^ whiten_[i]);
        raw = 0;
        if (bits == 16) {
            const uint32_t len = pdu_[1];
            if (len < kBleMinAdvPayload || len > kBleMaxAdvPayload) {
                stats_.len_bad++;
                state_ = State::Search;
                return true;
            }
            total_bits_ = (2 + len + 3) * 8;
            return true;
        }
        return bits == total_bits_;
    }

    /* One bit at the chosen phase (step()'s path; demod_groups() has its own
     * copy). True when the last CRC bit is in. */
    bool demod_bit(int32_t m) {
        raw_ |= static_cast<uint32_t>(m > bias_) << (bits_ & 7);
        bits_++;
        stats_.bits++;
        next_bit_ += 4;
        if ((bits_ & 7) == 0 && byte_done(bits_, raw_)) return state_ == State::Demod && bits_ == total_bits_;
        return false;
    }

    __attribute__((noinline)) void finish(uint32_t n, Callback callback, void* context) {
        state_ = State::Search;
        const size_t len = pdu_[1];
        const size_t crc_at = 2 + len;
        const uint32_t rx_crc = pdu_[crc_at] | (pdu_[crc_at + 1] << 8) | (pdu_[crc_at + 2] << 16);
        if (ble_crc24(pdu_, crc_at) != rx_crc) {
            stats_.crc_bad++;
            return;
        }
        stats_.ok++;

        BlePacket& p = packet_;
        for (size_t i = 0; i < kBlePduMax; i++) p.pdu[i] = pdu_[i];
        p.length = len;
        p.channel = channel_;
        p.phase = cand_n_ & 3;
        p.aa_errors = cand_errors_;
        const uint32_t power = power_ / total_bits_;
        p.rssi_cdb = static_cast<int16_t>(power_to_cdb(power) - kFullScaleCdb);
        /* m = 2 |z|^2 sin(2 pi f / fs): f = bias / (2 power) * fs / (2 pi),
         * times a factor for the modulation, which spreads the phase steps
         * around the CFO's (calibrated in the host test). */
        int32_t cfo = 0;
        if (power) cfo = static_cast<int32_t>((static_cast<int64_t>(bias_) * kCfoScale) / (static_cast<int64_t>(power) * 1000));
        if (cfo > 32767) cfo = 32767;
        if (cfo < -32768) cfo = -32768;
        p.cfo_khz = static_cast<int16_t>(cfo);
        p.end_sample = n;
        if (callback) callback(context, p);
    }

    /* 1000 * fs / (2 pi) / 2 * modulation factor, Hz per unit of bias / power. */
    static constexpr int64_t kCfoScale = 350000;

    /* Sync state. */
    uint32_t tr_[2]{}; /* phases 0 and 2 */

    State state_{State::Search};
    bool pending_{false};
    uint32_t sync_budget_{0};
    uint32_t pending_n_{0};
    uint32_t cand_n_{0};
    int32_t cand_bias_{0};
    uint32_t cand_errors_{0};

    /* Packet in progress. */
    int32_t bias_{0};
    uint32_t next_bit_{0};
    uint32_t bits_{0};
    uint32_t total_bits_{0};
    uint32_t power_{0};
    uint32_t raw_{0}; /* bits of the byte in progress, still whitened */
    uint8_t pdu_[kBlePduMax]{};

    /* All zero, so a static GfskDemod lands in .bss; configure() before use. */
    uint8_t channel_{0};
    uint8_t whiten_[kBlePduMax]{}; /* whitening of the channel, byte by byte */
    Stats stats_{};
    BlePacket packet_{};
};

/* PatternDemod: the same front end with a plain sync word in place of the
 * BLE access address, for 1 Mb/s GFSK emitters that are not BLE (RC toys,
 * nRF24 / XN297 style links): no dewhitening, no length, no CRC. A hit is
 * the last sync_bits hard bits on one sample phase matching the configured
 * sync word with at most max_errors wrong bits; optionally the payload_bits
 * after it are returned.
 *
 * Bits: the sign of m (no slicer bias: a carrier offset eats into the eye,
 * fine up to roughly half the deviation, e.g. +-100 kHz for a +-250 kHz
 * toy), one register pair per sample phase, so every phase of the 4 sample
 * symbol competes. Polarity is taken into the stored sync word (inverted:
 * bit 1 = negative deviation), so it costs nothing per sample.
 *
 * Cost: the pattern is checked at every sample. A full Hamming distance
 * there would be two software popcounts (the M4 has no popcount): too slow.
 * Instead the sync word is cut into the 16 bit pieces of the 64 bit window
 * (bits 0-15, 16-31, 32-47, 48-63, the last bit received in bit 0) that hold
 * any of its bits; with at most max_errors wrong bits and max_errors < the
 * number of pieces, at least one piece is clean (pigeonhole), and a clean
 * piece is one shift-and-branch on a register. Only when one is the full
 * distance is counted. So max_errors <= pieces - 1 = ceil(sync_bits / 16) - 1
 * (40 bits: 2, 64 bits: 3); configure() clamps it. The shortest piece decides
 * how often noise gets through to the count: 8 bits (a 40 bit sync), 1 in
 * 256 samples, ~16000 counts a second, well under 1 % of the M4.
 *
 * Detection runs per sample phase over the block (two registers for the
 * window, the sync word and the masks stay in registers), candidates go
 * into a short list, and after the four passes the list is sorted and
 * merged: hits of one packet on neighbouring phases (within kMergeSamples)
 * are one, with the fewest errors, at the middle of the phases that had that
 * many. The payload is read from the ring once its last bit is in (the ring
 * holds the last 1024 samples; payload_bits <= 64 is 256 samples), the
 * packet power from pw_ over the sync and payload. All state is in the
 * object, so a packet may straddle any number of execute() calls.
 *
 * Bit rates: only 1 Mb/s (4 samples a bit) is built. The rate is a
 * parameter so 2 Mb/s (2 samples a bit: a wider channel filter, stride 2)
 * and 250 kb/s (16 samples a bit: integrate the discriminator over 4 groups,
 * stride 16) can be added in scan() without changing the interface;
 * configure() rejects them for now. */

enum class PatternRate : uint8_t {
    R1M = 0,   /* 1 Mb/s, 4 samples a bit (built) */
    R2M = 1,   /* reserved */
    R250K = 2, /* reserved */
};

constexpr size_t kPatternMaxBits = 64;
constexpr size_t kPatternMinBits = 8;

/* Largest max_errors the 16 bit piece pre-check finds every match for. */
constexpr uint8_t pattern_max_errors(uint8_t sync_bits) {
    return sync_bits == 0 ? 0 : static_cast<uint8_t>((sync_bits + 15) / 16 - 1);
}

struct PatternConfig {
    uint64_t sync{0};        /* right aligned: the last sync bit in bit 0, the first in bit sync_bits - 1 */
    uint8_t sync_bits{0};    /* kPatternMinBits..kPatternMaxBits */
    uint8_t max_errors{0};   /* clamped to pattern_max_errors(sync_bits) */
    uint8_t payload_bits{0}; /* 0..64, returned after the sync */
    bool invert{false};      /* bit 1 = negative deviation */
    PatternRate rate{PatternRate::R1M};
    uint8_t tag{0}; /* copied into every hit (the app's channel / entry) */
};

struct PatternHit {
    uint64_t payload;    /* payload_bits right aligned, the first one in the MSB */
    uint32_t end_sample; /* demodulator's sample count (wrapping) at the last sync bit */
    int16_t rssi_cdb;    /* mean in-channel power over sync and payload, centi-dB re full scale */
    uint8_t errors;      /* sync bits wrong */
    uint8_t phase;       /* sample phase used, 0..3 */
    uint8_t payload_bits;
    uint8_t tag;
};

class PatternDemod : public GfskFrontEnd<PatternDemod> {
   public:
    using Callback = void (*)(void* context, const PatternHit& hit);

    struct Stats {
        uint32_t cands;   /* clean pieces: full distance counted */
        uint32_t matches; /* ... within max_errors (all phases, before merging) */
        uint32_t hits;    /* merged hits delivered */
        uint32_t capped;  /* matches dropped: kMaxCands per block reached */
    };

    /* Matches kept per 512 sample block: one packet gives up to 4 (one per
     * phase); a block can hold the syncs of few packets. Bounds the merge. */
    static constexpr size_t kMaxCands = 16;
    /* Matches this close (samples) are the same packet (neighbouring phases). */
    static constexpr uint32_t kMergeSamples = 6;

    /* False if the configuration can't be served (length, rate); the
     * receiver then finds nothing. Resets everything but the counters. */
    bool configure(const PatternConfig& c) {
        cfg_ = c;
        ok_ = c.rate == PatternRate::R1M && c.sync_bits >= kPatternMinBits && c.sync_bits <= kPatternMaxBits &&
              c.payload_bits <= kPatternMaxBits;
        const uint8_t emax = pattern_max_errors(c.sync_bits);
        if (cfg_.max_errors > emax) cfg_.max_errors = emax;
        const uint64_t mask = c.sync_bits >= 64 ? ~0ull : ((1ull << c.sync_bits) - 1);
        const uint64_t sync = (c.invert ? ~c.sync : c.sync) & mask;
        plo_ = static_cast<uint32_t>(sync);
        phi_ = static_cast<uint32_t>(sync >> 32);
        mlo_ = static_cast<uint32_t>(mask);
        mhi_ = static_cast<uint32_t>(mask >> 32);
        pieces_ = ok_ ? static_cast<uint8_t>((c.sync_bits + 15) / 16) : 0;
        reset();
        return ok_;
    }

    void reset() {
        for (size_t k = 0; k < 4; k++) lo_[k] = hi_[k] = 0;
        cluster_ = false;
        waiting_ = 0;
        reset_front();
    }

    const PatternConfig& config() const { return cfg_; }
    const Stats& stats() const { return stats_; }
    void clear_stats() { stats_ = Stats{}; }
    /* PSDT rebuilds the receiver after a DroneID dwell (shared memory) and puts its counters back. */
    void restore_stats(const Stats& s) { stats_ = s; }

    void execute(const int8_t* iq, size_t count, Callback callback, void* context) {
        run(iq, count, callback, context);
    }

   private:
    friend class GfskFrontEnd<PatternDemod>;

    struct Cand {
        uint32_t n; /* sample of the last sync bit */
        uint8_t errors;
    };

    /* One block: the four phase passes, then merge and deliver. */
    __attribute__((noinline)) void detect(size_t groups, Callback callback, void* context) {
        ncand_ = 0;
        const int32_t* const r = at(n_);
        switch (pieces_) {
            case 1:
                scan<1>(r, groups);
                break;
            case 2:
                scan<2>(r, groups);
                break;
            case 3:
                scan<3>(r, groups);
                break;
            case 4:
                scan<4>(r, groups);
                break;
            default:
                break;
        }
        n_ += 4 * groups;
        merge(callback, context);
    }

    /* The window of one phase from r (stride one group) to end: per sample,
     * shift the bit in (lo = lo << 1 | (u < 0), hi = hi << 1 | old bit 31 of
     * lo), x = window ^ sync, then the clean piece test: a full 16 bit piece
     * is clean when its half of x is zero (a shift and a branch), the top
     * piece, partly beyond sync_bits, when x & mtop is (a tst and a branch).
     * Stops after a clean piece (found) or at end. On the M4 in assembly: GCC
     * spilled and hoisted the slow path's call arguments into the loop (25
     * cycles a sample); this is about 15 at K = 3. K: pieces holding sync bits. */
    template <int K>
    static const int32_t* scan_run(const int32_t* r, const int32_t* const end, uint32_t& lo, uint32_t& hi, uint32_t plo, uint32_t phi, uint32_t mtop, bool& found) {
#if defined(__ARM_FEATURE_SIMD32)
#define SIGDET_PAT_HEAD            \
    "1: cmp %[r], %[end]\n\t"      \
    "beq 3f\n\t"                   \
    "ldr %[t], [%[r]], #16\n\t"    \
    "lsls %[t], %[t], #1\n\t"      \
    "adcs %[lo], %[lo], %[lo]\n\t" \
    "adc %[hi], %[hi], %[hi]\n\t"  \
    "eor %[t], %[lo], %[plo]\n\t"
#define SIGDET_PAT_TAIL    \
    "2: movs %[f], #1\n\t" \
    "b 4f\n\t"             \
    "3: movs %[f], #0\n\t" \
    "4:\n\t"
        uint32_t t, u, f;
        if (K == 1) {
            __asm__ volatile(SIGDET_PAT_HEAD "tst %[t], %[mtop]\n\tbne 1b\n\t" SIGDET_PAT_TAIL
                             : [r] "+r"(r), [lo] "+r"(lo), [hi] "+r"(hi), [t] "=&r"(t), [u] "=&r"(u), [f] "=&r"(f)
                             : [end] "r"(end), [plo] "r"(plo), [phi] "r"(phi), [mtop] "r"(mtop)
                             : "cc", "memory");
        } else if (K == 2) {
            __asm__ volatile(SIGDET_PAT_HEAD "lsls %[u], %[t], #16\n\tbeq 2f\n\ttst %[t], %[mtop]\n\tbne 1b\n\t" SIGDET_PAT_TAIL
                             : [r] "+r"(r), [lo] "+r"(lo), [hi] "+r"(hi), [t] "=&r"(t), [u] "=&r"(u), [f] "=&r"(f)
                             : [end] "r"(end), [plo] "r"(plo), [phi] "r"(phi), [mtop] "r"(mtop)
                             : "cc", "memory");
        } else if (K == 3) {
            __asm__ volatile(SIGDET_PAT_HEAD
                             "lsls %[u], %[t], #16\n\tbeq 2f\n\tlsrs %[t], %[t], #16\n\tbeq 2f\n\t"
                             "eor %[t], %[hi], %[phi]\n\ttst %[t], %[mtop]\n\tbne 1b\n\t" SIGDET_PAT_TAIL
                             : [r] "+r"(r), [lo] "+r"(lo), [hi] "+r"(hi), [t] "=&r"(t), [u] "=&r"(u), [f] "=&r"(f)
                             : [end] "r"(end), [plo] "r"(plo), [phi] "r"(phi), [mtop] "r"(mtop)
                             : "cc", "memory");
        } else {
            __asm__ volatile(SIGDET_PAT_HEAD
                             "lsls %[u], %[t], #16\n\tbeq 2f\n\tlsrs %[t], %[t], #16\n\tbeq 2f\n\t"
                             "eor %[t], %[hi], %[phi]\n\tlsls %[u], %[t], #16\n\tbeq 2f\n\ttst %[t], %[mtop]\n\tbne 1b\n\t" SIGDET_PAT_TAIL
                             : [r] "+r"(r), [lo] "+r"(lo), [hi] "+r"(hi), [t] "=&r"(t), [u] "=&r"(u), [f] "=&r"(f)
                             : [end] "r"(end), [plo] "r"(plo), [phi] "r"(phi), [mtop] "r"(mtop)
                             : "cc", "memory");
        }
#undef SIGDET_PAT_HEAD
#undef SIGDET_PAT_TAIL
        found = f != 0;
        return r;
#else
        while (r != end) {
            hi = (hi << 1) | (lo >> 31);
            lo = (lo << 1) | (static_cast<uint32_t>(*r) >> 31);
            r += 4;
            const uint32_t xl = lo ^ plo, xh = hi ^ phi;
            bool clean;
            if (K == 1)
                clean = !(xl & mtop);
            else if (K == 2)
                clean = (xl << 16) == 0 || !(xl & mtop);
            else if (K == 3)
                clean = (xl << 16) == 0 || (xl >> 16) == 0 || !(xh & mtop);
            else
                clean = (xl << 16) == 0 || (xl >> 16) == 0 || (xh << 16) == 0 || !(xh & mtop);
            if (clean) {
                found = true;
                return r;
            }
        }
        found = false;
        return r;
#endif
    }

    /* Each phase over the block; count() for every clean piece. */
    template <int K>
    __attribute__((noinline)) void scan(const int32_t* const block, size_t groups) {
        const uint32_t plo = plo_, phi = phi_;
        const uint32_t mtop = K == 1 ? mlo_ : K == 2 ? (mlo_ & 0xFFFF0000u)
                                          : K == 3   ? mhi_
                                                     : (mhi_ & 0xFFFF0000u);
        for (uint32_t ph = 0; ph < 4; ph++) {
            uint32_t lo = lo_[ph], hi = hi_[ph];
            const int32_t* r = block + ph;
            const int32_t* const end = r + 4 * groups;
            for (;;) {
                bool found;
                r = scan_run<K>(r, end, lo, hi, plo, phi, mtop, found);
                if (!found) break;
                count(lo, hi, n_ + static_cast<uint32_t>(r - block) - 4);
            }
            lo_[ph] = lo;
            hi_[ph] = hi;
        }
    }

    __attribute__((noinline)) void count(uint32_t lo, uint32_t hi, uint32_t n) {
        stats_.cands++;
        const uint32_t xl = (lo ^ plo_) & mlo_, xh = (hi ^ phi_) & mhi_;
        const uint32_t errors = static_cast<uint32_t>(__builtin_popcount(xl) + __builtin_popcount(xh));
        if (errors > cfg_.max_errors) return;
        stats_.matches++;
        if (ncand_ == kMaxCands) {
            stats_.capped++;
            return;
        }
        cand_[ncand_++] = Cand{n, static_cast<uint8_t>(errors)};
    }

    static bool before(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) < 0; }

    /* Candidates of this block in sample order into the open cluster; a
     * cluster closes kMergeSamples after its last match and waits for its
     * payload; finished ones go to the callback. */
    __attribute__((noinline)) void merge(Callback callback, void* context) {
        for (size_t i = 1; i < ncand_; i++) {
            const Cand c = cand_[i];
            size_t j = i;
            for (; j > 0 && before(c.n, cand_[j - 1].n); j--) cand_[j] = cand_[j - 1];
            cand_[j] = c;
        }
        for (size_t i = 0; i < ncand_; i++) {
            const Cand& c = cand_[i];
            if (cluster_ && static_cast<uint32_t>(c.n - last_n_) <= kMergeSamples) {
                last_n_ = c.n;
                if (c.errors < best_errors_) {
                    best_errors_ = c.errors;
                    best_first_ = best_last_ = c.n;
                } else if (c.errors == best_errors_) {
                    best_last_ = c.n;
                }
                continue;
            }
            close_cluster();
            cluster_ = true;
            last_n_ = best_first_ = best_last_ = c.n;
            best_errors_ = c.errors;
        }
        /* Nothing after n_ - 1 can join a cluster ending before n_ - 1 - kMergeSamples. */
        if (cluster_ && static_cast<uint32_t>(n_ - 1 - last_n_) > kMergeSamples) close_cluster();

        /* Deliver the waiting hits whose payload is complete (n_ - 1 is the
         * newest sample in the ring), oldest first. */
        const uint32_t tail = 4u * cfg_.payload_bits;
        while (waiting_ && static_cast<int32_t>((n_ - 1) - (wait_[0].n + tail)) >= 0) {
            deliver(wait_[0], callback, context);
            for (size_t k = 1; k < waiting_; k++) wait_[k - 1] = wait_[k];
            waiting_--;
        }
    }

    __attribute__((noinline)) void close_cluster() {
        if (!cluster_) return;
        cluster_ = false;
        if (waiting_ == kMaxWaiting) {
            stats_.capped++;
            return;
        }
        wait_[waiting_++] = Cand{best_first_ + (best_last_ - best_first_) / 2, best_errors_};
    }

    __attribute__((noinline)) void deliver(const Cand& c, Callback callback, void* context) {
        PatternHit h{};
        uint64_t payload = 0;
        const uint32_t pbits = cfg_.payload_bits;
        for (uint32_t k = 1; k <= pbits; k++) payload = (payload << 1) | (static_cast<uint32_t>(*at(c.n + 4 * k)) >> 31);
        if (cfg_.invert) payload = ~payload & (pbits >= 64 ? ~0ull : ((1ull << pbits) - 1));
        /* Power: the groups of the sync bits and the payload bits. */
        uint32_t groups = cfg_.sync_bits + pbits;
        uint64_t power = 0;
        uint32_t g = c.n - 4u * (cfg_.sync_bits - 1);
        for (uint32_t k = 0; k < groups; k++, g += 4) power += group_power(g);
        h.payload = payload;
        h.end_sample = c.n;
        h.rssi_cdb = static_cast<int16_t>(power_to_cdb(static_cast<uint32_t>(power / groups)) - kFullScaleCdb);
        h.errors = c.errors;
        h.phase = c.n & 3;
        h.payload_bits = static_cast<uint8_t>(pbits);
        h.tag = cfg_.tag;
        stats_.hits++;
        if (callback) callback(context, h);
    }

    /* Hits waiting for their payload: a few packets in 256 samples at most. */
    static constexpr size_t kMaxWaiting = 4;

    /* All zero, so a static PatternDemod lands in .bss; configure() before use. */
    PatternConfig cfg_{};
    bool ok_{false};
    uint8_t pieces_{0};
    uint32_t plo_{0}, phi_{0}, mlo_{0}, mhi_{0};
    uint32_t lo_[4]{}, hi_[4]{};
    Cand cand_[kMaxCands]{};
    size_t ncand_{0};
    bool cluster_{false};
    uint32_t last_n_{0}, best_first_{0}, best_last_{0};
    uint8_t best_errors_{0};
    Cand wait_[kMaxWaiting]{};
    size_t waiting_{0};
    Stats stats_{};
};

/* Remote ID forwarding limits (M4 -> M0). A transmitter repeats each payload on
 * several advertising events, and the M0's application queue is shallow, so
 * what is forwarded is thinned out here. */

/* Entries of the duplicate table: a few drones in range, each with a handful of
 * ODID message types in rotation; the oldest entry is replaced beyond that. */
constexpr size_t kDedupSlots = 16;
/* Same advert inside this window is a repeat of one transmission burst; longer
 * than the repeats of one burst, shorter than a message's refresh interval. */
constexpr uint32_t kDedupWindowMs = 300;
/* Rate cap window: short enough to bound the burst the M0 queue must absorb. */
constexpr uint32_t kRateWindowMs = 100;
/* Forwards per rate window: what the M0 can drain in that time without the
 * application queue overflowing. */
constexpr uint32_t kRateMaxFrames = 8;

/* Duplicate table and rate cap of what the M4 forwards: kWindowMs the
 * duplicate window, kMaxFrames forwards per kRateWindowMs at most. */
template <uint32_t kWindowMs, uint32_t kMaxFrames>
class ForwardFilter {
   public:
    /* True when the same advert (AdvA, ODID message type, counter, payload sum)
     * was forwarded less than kDedupWindowMs ago. Otherwise records it as
     * forwarded now and returns false. A suppressed duplicate does not refresh
     * the stored time, so a payload repeated forever is forwarded every
     * kDedupWindowMs. */
    bool duplicate(const uint8_t* addr, uint8_t type, uint8_t counter, uint16_t sum, uint32_t t_ms) {
        Entry* spare = nullptr;
        for (auto& e : entries_) {
            if (!e.used) {
                if (!spare) spare = &e;
                continue;
            }
            if (e.type == type && e.counter == counter && e.sum == sum && same_addr(e.addr, addr)) {
                if (static_cast<uint32_t>(t_ms - e.t_ms) < kWindowMs) return true;
                store(e, addr, type, counter, sum, t_ms);
                return false;
            }
        }
        if (!spare) {
            /* Table full: replace the oldest (any expired entry is older than
             * every live one). */
            spare = &entries_[0];
            uint32_t oldest_age = 0;
            for (auto& e : entries_) {
                const uint32_t age = t_ms - e.t_ms;
                if (age > oldest_age) {
                    spare = &e;
                    oldest_age = age;
                }
            }
        }
        store(*spare, addr, type, counter, sum, t_ms);
        return false;
    }

    /* True when one more forward fits in the current kRateWindowMs window,
     * and counts it; false (not counted) beyond kRateMaxFrames. */
    bool rate_ok(uint32_t t_ms) {
        if (static_cast<uint32_t>(t_ms - window_start_) >= kRateWindowMs) {
            window_start_ = t_ms;
            window_count_ = 0;
        }
        if (window_count_ >= kMaxFrames) return false;
        window_count_++;
        return true;
    }

    void reset() {
        for (auto& e : entries_) e.used = false;
        window_start_ = 0;
        window_count_ = 0;
    }

   private:
    struct Entry {
        uint32_t t_ms;
        uint8_t addr[6];
        uint8_t type;
        uint8_t counter;
        uint16_t sum;
        bool used;
    };

    static bool same_addr(const uint8_t* a, const uint8_t* b) {
        for (size_t i = 0; i < 6; i++)
            if (a[i] != b[i]) return false;
        return true;
    }

    static void store(Entry& e, const uint8_t* addr, uint8_t type, uint8_t counter, uint16_t sum, uint32_t t_ms) {
        e.t_ms = t_ms;
        for (size_t i = 0; i < 6; i++) e.addr[i] = addr[i];
        e.type = type;
        e.counter = counter;
        e.sum = sum;
        e.used = true;
    }

    /* All zero, so a static RidDedup lands in .bss. */
    Entry entries_[kDedupSlots]{};
    uint32_t window_start_{0};
    uint32_t window_count_{0};
};

/* Remote ID frames. */
using RidDedup = ForwardFilter<kDedupWindowMs, kRateMaxFrames>;

/* Other adverts forwarded for presence (sigdet_ble_vendor.hpp: DJI): the app
 * wants about one a second per device to keep an RSSI trend, not every
 * advertising event (~10 a second each). */
constexpr uint32_t kVendorDedupWindowMs = 1000;
/* ... at most this many per kRateWindowMs, beside the Remote ID frames. */
constexpr uint32_t kVendorRateMaxFrames = 4;
using VendorDedup = ForwardFilter<kVendorDedupWindowMs, kVendorRateMaxFrames>;

/* Any other advert with AdvData (FLAG_FWD_ADV: signature rules and the
 * survey log of the app): one per address and payload a second, at most this
 * many per kRateWindowMs beside Remote ID and DJI (30 a second: a household
 * has 20-40 advertisers; the app aggregates them). */
constexpr uint32_t kOtherRateMaxFrames = 3;
using OtherDedup = ForwardFilter<kVendorDedupWindowMs, kOtherRateMaxFrames>;

/* Wi-Fi Remote ID frames (sigdet_dsss.hpp) per kRateWindowMs at most: a
 * transmitter beacons at 1-10 Hz with a new message counter each time, so
 * this serves three drones at 10 Hz; three of the largest messages (~280
 * bytes) are 40 % of the M0's 2 KB queue. */
constexpr uint32_t kWifiRateMaxFrames = 3;
using WifiDedup = ForwardFilter<kDedupWindowMs, kWifiRateMaxFrames>;

/* Pattern hits forwarded per kRateWindowMs at most: a toy sends ~67 packets
 * a second on its channel, a stronger emitter of the same pattern no more
 * than a few hundred; the app needs counts, not every hit. */
constexpr uint32_t kPatternRateMaxHits = 8;

} /* namespace sigdet */

#endif /*__SIGDET_DSP_H__*/
