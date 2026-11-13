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

/* 802.11b 1 Mb/s DSSS receiver (DBPSK, long preamble) for a 4 Msps slice of
 * a 2.4 GHz Wi-Fi channel: the drone detect baseband's Wi-Fi Remote ID mode
 * (Open Drone ID beacons and NAN frames sent at 1 Mb/s). Header only, no
 * heap, integer maths, ChibiOS free (firmware/test/baseband runs it on the
 * host). Shares the SIMD helpers of dd_dsp.hpp.
 *
 * Where the slice sits. The M0 tunes through ReceiverModel, which puts the
 * LO a quarter of the sample rate below the target (dd_dsp.hpp: the BLE
 * channel lands at +1 MHz). The 4 MHz wide baseband is centred on the LO, so
 * for a Wi-Fi channel centred at Fc and a wanted slice centre Fc + s:
 *     target = Fc + s + fs/4 = Fc + s + 1 MHz
 *     s = -2 MHz: target = Fc - 1 MHz (slice covers Fc - 4 .. Fc)
 *     s = +2 MHz: target = Fc + 3 MHz (slice covers Fc .. Fc + 4)
 * so both halves of the channel are reached by retuning alone (same sample
 * rate, same filters as BLE). Whole MHz offsets keep the carrier from turning
 * between 1 us symbols; the matched filter taps depend on s (dsss_taps()).
 * The DC residue (PRALINE: FPGA DC block) sits at the slice centre, inside
 * the Wi-Fi channel, 2 MHz from its centre.
 *
 * Signal model (prototype: mayhem-build/proto/dsss): in the slice the 11 chip
 * Barker symbol becomes a ~4 sample complex pulse whose shape depends on s
 * and on the sub-sample timing. Two 4 tap matched filters, A for a symbol
 * centred on a sample, B for one half a sample later.
 *
 * Per sample, integer only:
 *   x      int8 I/Q -> packed int16 pair (I low, Q high)
 *   y[n]   = sum_k h_k x[n-k]     2 dual MACs a tap (smusd -> Re, smuadx ->
 *                                 Im), |y| < 2^14 with |h| <= 16: packed as is
 *   d[n]   = Re y[n] conj y[n-4]  one smuad: DBPSK differential detection
 * Search (filter A on the sample phases of kSearchPhases: 4, 2, or the
 * firmware's 3 = phases 0 and 2 filtered, 1 and 3 interpolated):
 *   raw bits go into a register per phase; every 8 symbols the last 24
 *   descrambled bits D = R ^ R>>4 ^ R>>7 (descrambler z^-7 + z^-4 + 1, bit 0
 *   newest) are compared with all ones (the long SYNC is 128 scrambled
 *   ones); at most trig_zeros wrong is a trigger.
 * Sync (8 hypotheses: filters A and B x 4 phases): per symbol and hypothesis
 *   the register, D, a leaky |y|^2 and the SFD test: D's low 16 bits == SFD
 *   0xF3A0 (sent LSB first) and at most one of the 9 bits before it not one.
 *   The first match opens a 4 sample window; of the hypotheses matching in it
 *   the one with the largest |y|^2 wins. No SFD in kSyncSymbols: Search.
 * Lock (the winner only): one MF output and one decision a symbol, the same
 *   descrambler; 48 header bits, CRC-16 CCITT (preset ones, complemented),
 *   SIGNAL 0x0A (1 Mb/s), LENGTH = 8 us a byte; then the MPDU (bytes LSB
 *   first) with a nibble-table CRC-32 as it comes, a frame control filter on
 *   the first byte (beacon, probe response, action), the FCS check, the
 *   callback. 2 Mb/s and CCK headers are counted, not decoded (hook: the
 *   DQPSK path of the prototype's float receiver); short preamble frames
 *   (56 scrambled zeros, SFD 0x05CF) never trigger the long SYNC search.
 * CFO: in Sync each hypothesis sums its data-wiped differential products
 *   (d + j e) sign(d), leaky; Lock decides on the sign of Re{(d + j e)
 *   conj(w)} = d wr + e wi (64 bit). Recovers about +-120 kHz.
 *
 * Real time. The work per sample depends on the state; measured on the
 * HackRF Pro's M4 (DWT, DDSTAT cyc_*): Search 30 cycles (4 phases; the
 * instruction count said 25), 27 (mode 3), 21 (2 phases); Sync ~108 (about
 * 440 samples a frame); Lock ~29; the CFO pass ~5.3 k a frame. A 2048 sample
 * DMA buffer that held a whole Sync and the start of Lock would need more
 * than the ~102 k cycles of its 512 us. So the samples go through an input ring
 * (push()) and are demodulated in blocks of kBlockGroups groups (step()):
 * the caller runs blocks while its cycle budget allows (process()), leaving
 * the rest for the next buffer. A Sync burst is spread over the following
 * buffers (every other state runs well below the budget, so the backlog
 * drains); a backlog beyond the ring (a pathological stream) is dropped as
 * a whole (stats().overflows) and the receiver restarts in Search. The
 * input ring also keeps the CFO and state logic independent of how the
 * samples arrive: any chunking gives the same frames. */

#ifndef __DD_DSSS_H__
#define __DD_DSSS_H__

#include <cstddef>
#include <cstdint>

#include "dd_dsp.hpp"

namespace dd {

#if defined(__ARM_FEATURE_SIMD32)
/* a.lo b.lo - a.hi b.hi */
inline int32_t simd_smusd(uint32_t a, uint32_t b) {
    int32_t r;
    __asm__("smusd %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
inline int32_t simd_smlsd(uint32_t a, uint32_t b, int32_t acc) {
    int32_t r;
    __asm__("smlsd %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
/* a.lo b.hi + a.hi b.lo */
inline int32_t simd_smuadx(uint32_t a, uint32_t b) {
    int32_t r;
    __asm__("smuadx %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
inline int32_t simd_smladx(uint32_t a, uint32_t b, int32_t acc) {
    int32_t r;
    __asm__("smladx %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
/* a.lo b.hi - a.hi b.lo */
inline int32_t simd_smusdx(uint32_t a, uint32_t b) {
    int32_t r;
    __asm__("smusdx %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
/* acc + a.lo b.lo + a.hi b.hi */
inline int32_t simd_smlad(uint32_t a, uint32_t b, int32_t acc) {
    int32_t r;
    __asm__("smlad %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
/* ((a.lo + b.lo) >> 1, (a.hi + b.hi) >> 1) */
inline uint32_t simd_shadd16(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm__("shadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
#else
inline int32_t simd_smusd(uint32_t a, uint32_t b) {
    return lo16(a) * lo16(b) - hi16(a) * hi16(b);
}
inline int32_t simd_smlsd(uint32_t a, uint32_t b, int32_t acc) {
    return acc + simd_smusd(a, b);
}
inline int32_t simd_smuadx(uint32_t a, uint32_t b) {
    return lo16(a) * hi16(b) + hi16(a) * lo16(b);
}
inline int32_t simd_smladx(uint32_t a, uint32_t b, int32_t acc) {
    return acc + simd_smuadx(a, b);
}
inline int32_t simd_smusdx(uint32_t a, uint32_t b) {
    return lo16(a) * hi16(b) - hi16(a) * lo16(b);
}
inline int32_t simd_smlad(uint32_t a, uint32_t b, int32_t acc) {
    return acc + lo16(a) * lo16(b) + hi16(a) * hi16(b);
}
inline uint32_t simd_shadd16(uint32_t a, uint32_t b) {
    return pack16((lo16(a) + lo16(b)) >> 1, (hi16(a) + hi16(b)) >> 1);
}
#endif

constexpr uint32_t kDsssSampleRate = 4000000;
/* MPDU bytes kept: a Remote ID beacon is ~150-400 bytes (ours: 193). */
constexpr size_t kDsssMaxMpdu = 512;
/* 1000 log10(128^2): a complex tone of amplitude 128 (int8 full scale) is
 * 0 dBFS, as for the BLE receiver. */
constexpr int32_t kDsssFullScaleCdb = 4214;
/* The 4 MHz slice holds this much less than the whole DSSS channel (the
 * Barker spectrum has a null at the channel centre): measured over the C5's
 * decoded beacons, -30.8 dBFS over 20 MHz and -34.6..-35.0 in the -2 and
 * +2 MHz slices. Added to the slice power, so the RSSI estimates the
 * channel power. */
constexpr int32_t kDsssSliceToChannelCdb = 400;

struct DsssFrame {
    uint8_t mpdu[kDsssMaxMpdu]; /* MAC header .. FCS */
    uint16_t length;            /* bytes incl. FCS */
    uint8_t phase;              /* sample phase 0..3 */
    uint8_t filter;             /* matched filter 0 (A) or 1 (B) */
    int16_t rssi_cdb;           /* channel power estimate, centi-dB re int8 full scale */
    uint32_t end_sample;        /* sample count (wrapping) of the last bit */
};

/* Nibble-table CRC-32 (reflected 0xEDB88320), one byte at a time. */
inline uint32_t crc32_byte(uint32_t crc, uint8_t b) {
    static constexpr uint32_t t[16] = {0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4,
                                       0x4DB26158, 0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
                                       0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};
    crc ^= b;
    crc = (crc >> 4) ^ t[crc & 15];
    crc = (crc >> 4) ^ t[crc & 15];
    return crc;
}
constexpr uint32_t kCrc32Residue = 0xDEBB20E3; /* crc over data + FCS, init ~0, no final xor */

/* PLCP header CRC-16 over SIGNAL, SERVICE, LENGTH (bit 0 sent first);
 * returns the 16 check bits in air order (bit 0 first). */
inline uint32_t plcp_crc16(uint32_t bits32) {
    uint32_t reg = 0xFFFF;
    for (int i = 0; i < 32; i++) {
        const uint32_t fb = ((reg >> 15) ^ (bits32 >> i)) & 1;
        reg = (reg << 1) & 0xFFFF;
        if (fb) reg ^= 0x1021;
    }
    reg ^= 0xFFFF;
    uint32_t out = 0; /* b15 of the remainder is sent first */
    for (int i = 0; i < 16; i++) out |= ((reg >> (15 - i)) & 1) << i;
    return out;
}

/* SFD 0xF3A0 sent LSB first; in D (bit 0 = newest) its last bit is bit 0. */
constexpr uint32_t dsss_sfd_pattern() {
    uint32_t p = 0;
    for (int j = 0; j < 16; j++) p |= ((0xF3A0u >> (15 - j)) & 1) << j;
    return p;
}

/* PLCP SIGNAL values (rate in 100 kb/s). Only 1 Mb/s is demodulated. */
constexpr uint32_t kPlcpSignal1M = 0x0A;
constexpr uint32_t kPlcpSignal2M = 0x14;

/* Matched filters for a slice offset s (slice centre - channel centre):
 * h[f][k] = (re, im) of filter f (0: A, timing 0; 1: B, half a sample
 * later), unit energy each at one common scale (largest component 16),
 * y = sum h_k x[n-k]. From the prototype's theory pulse (rect chips through
 * the PRALINE decimators hbfir2 + hbfir1 at 4 Msps, proto/dsss
 * kernel_compare.py quant_taps). taps(+s) = conj(taps(-s)). */
struct DsssTaps {
    int16_t h[2][4][2];
};
constexpr DsssTaps kDsssTaps[5] = {
    /* -2 MHz */ {{{{-5, -7}, {5, -16}, {5, -9}, {-11, 5}}, {{-3, -14}, {11, -15}, {-8, -1}, {-2, 4}}}},
    /* -1 MHz */ {{{{-11, -10}, {-16, -5}, {-11, 10}, {0, -14}}, {{-14, -16}, {-14, 11}, {-5, -6}, {3, -5}}}},
    /*  0 MHz */ {{{{-14, 0}, {-4, 0}, {15, 0}, {4, 0}}, {{-16, 0}, {12, 0}, {6, 0}, {5, 0}}}},
    /* +1 MHz */ {{{{-11, 10}, {-16, 5}, {-11, -10}, {0, 14}}, {{-14, 16}, {-14, -11}, {-5, 6}, {3, 5}}}},
    /* +2 MHz */ {{{{-5, 7}, {5, 16}, {5, 9}, {-11, -5}}, {{-3, 14}, {11, 15}, {-8, 1}, {-2, -4}}}},
};
/* Taps for a whole-MHz slice offset -2..+2, nullptr otherwise. */
inline const DsssTaps* dsss_taps(int offset_mhz) {
    return (offset_mhz < -2 || offset_mhz > 2) ? nullptr : &kDsssTaps[offset_mhz + 2];
}

/* When Search starts a Sync. checks: consecutive checkpoints (8 symbols
 * apart) with a SYNC-like register; sync_symbols: symbols a Sync waits for
 * the SFD; rearm: after a Sync without SFD, no trigger until a checkpoint
 * without a SYNC-like register. The defaults are the prototype's. */
struct DsssTrigger {
    uint32_t checks{1};
    uint32_t sync_symbols{200};
    bool rearm{false};
};
/* The firmware's rule. Two checkpoints: noise, which passes one 24 bit test
 * dozens of times a second at the firmware's gain, then hardly ever starts
 * a Sync (each costs up to sync_symbols x 4 samples at ~135 cycles); 128
 * symbols: the SFD ends at most 113 symbols after the earliest trigger; no
 * re-trigger on a SYNC-like signal that goes on (a tone 0.5 MHz off the
 * slice centre flips the DBPSK phase every symbol: all ones). Host tests:
 * the same frames as the prototype's rule on every recording and at every
 * Eb/N0 from 12 to 18 dB on both slices. */
constexpr DsssTrigger kDsssTriggerFirmware{2, 128, true};

/* kSearchPhases: the sample phases Search tests for the SYNC.
 *   4: filter A on all 4 phases (the reference; 25 cycles a sample by
 *      instruction count, 30 measured on the HackRF Pro's M4 (121 a group):
 *      more than the DSSS budget, the input ring overflowed ~30 times a
 *      second of Wi-Fi dwell on the bench);
 *   3: filter A on phases 0 and 2, phases 1 and 3 interpolated (the mean of
 *      the outputs either side, one shadd16 each), 4 trigger registers:
 *      107 cycles a group measured; host: ~1 dB behind 4 phases at the
 *      threshold, ~0.5 dB ahead of 2; sheds to 2 phases while the backlog
 *      is long (kShedOn). The firmware's;
 *   2: phases 0 and 2 only (83 cycles a group measured). */
template <int kSearchPhases = 4>
class DsssDemod;
/* The search the firmware runs (proc_drone_detect.cpp) and the host tests
 * test as "the firmware's kernel". */
constexpr int kDsssFirmwareSearch = 3;

template <int kSearchPhases>
class DsssDemod {
    static_assert(kSearchPhases >= 2 && kSearchPhases <= 4, "search on 2, 3 (2 + 2 interpolated) or 4 phases");

   public:
    using Callback = void (*)(void* context, const DsssFrame& frame);
    using Trigger = DsssTrigger;

    /* Input ring: a whole number of 2048 sample DMA buffers, > one buffer +
     * the longest backlog a frame leaves (a Sync burst: ~1100 samples). */
    static constexpr size_t kRingSamples = 8192;
    /* Groups (4 samples) per step() by state: each step works in one state
     * only (a change of state ends it), so its work is bounded by that
     * state's cost: a few k cycles each, so the caller can run steps up to
     * close to its limit (a Search block of 64 groups measured 44-46 us on
     * the HackRF Pro with the 4 phase search, longer than the step bound). */
    static constexpr size_t kSearchGroups = 32;
    static constexpr size_t kSyncGroups = 8;
    static constexpr size_t kLockGroups = 32;
    static constexpr size_t kBlockGroups = 32; /* the largest of them */

    struct Stats {
        uint32_t triggers;  /* SYNC detections */
        uint32_t sfd;       /* SFDs found (Lock: a frame seen) */
        uint32_t sync_lost; /* triggers without SFD */
        uint32_t hdr_bad;   /* PLCP CRC failed */
        uint32_t hdr_2m;    /* PLCP fine, 2 Mb/s DQPSK: not demodulated */
        uint32_t not_1m;    /* PLCP fine, other rate (CCK) or a length out of range */
        uint32_t filtered;  /* dropped at the frame control byte (not beacon / probe response / action) */
        uint32_t fcs_bad;
        uint32_t ok;        /* FCS valid frames given to the callback */
        uint32_t symbols;   /* symbols demodulated in Lock */
        uint32_t overflows; /* backlog dropped: the ring was full */
        uint32_t dropped;   /* ... samples dropped with it */
        /* Groups (4 samples) demodulated in each state, for the cost model. */
        uint32_t g_search, g_sync, g_lock;
        uint32_t g_shed;     /* ... of g_search searched on 2 phases to shed load (mode 3, kShedOn) */
        uint32_t cfo_passes; /* carrier estimates (one a frame, a step of their own) */
    };

    /* taps: dsss_taps(s). filters: 1 (A only) or 2. invert: half-MHz slice
     * offset (decisions inverted). cfo: carrier correction from the SYNC.
     * mgmt_only: keep beacons, probe responses and action frames only.
     * trig_zeros: wrong bits allowed in the 24 bit SYNC trigger (0..4).
     * The trigger rule (Trigger) defaults to the prototype's; the firmware
     * uses kTriggerFirmware. search_phases: 2, 3 or 4 (see kSearchPhases),
     * anything else the class's kSearchPhases (bring-up switch: one image
     * can compare them on the device).
     * Drops the backlog and any frame in progress. */
    void configure(const DsssTaps& taps, int filters, bool invert, bool cfo, bool mgmt_only, int trig_zeros = 3, const Trigger& trigger = Trigger{}, int search_phases = 0) {
        phases_ = static_cast<uint8_t>((search_phases >= 2 && search_phases <= 4) ? search_phases : 0);
        for (int f = 0; f < 2; f++)
            for (int k = 0; k < 4; k++)
                h_[f][k] = static_cast<uint32_t>(static_cast<uint16_t>(taps.h[f][k][0])) |
                           (static_cast<uint32_t>(static_cast<uint16_t>(taps.h[f][k][1])) << 16);
        nf_ = filters == 2 ? 2 : 1;
        invert_ = invert ? 1u : 0u;
        cfo_ = cfo;
        mgmt_only_ = mgmt_only;
        trig_zeros_ = static_cast<uint32_t>(trig_zeros);
        trig_checks_ = trigger.checks < 1 ? 1 : trigger.checks;
        sync_symbols_ = trigger.sync_symbols;
        rearm_ = trigger.rearm;
        reset();
    }

    /* Drops the backlog, any frame in progress and the filter history (on a
     * retune). (Loops kept as loops: newlib-nano's memset goes byte by byte.) */
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) void reset() {
        state_ = State::Search;
        for (int k = 0; k < 8; k++) r_[k] = yp_[k] = 0, e_[k] = 0;
        for (size_t i = 0; i < kHist; i++) xb_[i] = 0;
        for (size_t i = 0; i < 4; i++) yb_[i] = 0;
        pending_ = false;
        armed_ = blocked_ = false;
        cfo_pending_ = false;
        shedding_ = searched2_ = false;
        drop_backlog();
    }

    const Stats& stats() const { return stats_; }
    /* The search in use: 2, 3 or 4 (kSearchPhases). */
    int search_phases() const { return phases_ ? phases_ : kSearchPhases; }
    void clear_stats() { stats_ = Stats{}; }
    /* PDRN rebuilds the receiver after a DroneID dwell (shared memory) and puts its counters back. */
    void restore_stats(const Stats& s) { stats_ = s; }

    /* Samples waiting in the ring. */
    size_t backlog() const { return w_ - n_; }

    /* Appends count interleaved int8 I/Q pairs to the ring. If they don't
     * fit, the backlog is dropped first (counted) and the receiver restarts
     * in Search. Returns the samples dropped. */
    size_t push(const int8_t* iq, size_t count) {
        size_t dropped = 0;
        if (count > kRingSamples - kRetroSamples) { /* keep the newest */
            dropped += count - (kRingSamples - kRetroSamples);
            iq += 2 * (count - (kRingSamples - kRetroSamples));
            count = kRingSamples - kRetroSamples;
        }
        if (backlog() + count > kRingSamples - kRetroSamples) {
            dropped += backlog() & ~size_t{3};
            stats_.overflows++;
            to_search();
            pending_ = blocked_ = cfo_pending_ = false;
            drop_backlog();
        }
        stats_.dropped += dropped;
        const size_t at = w_ & kRingMask;
        const size_t first = count < kRingSamples - at ? count : kRingSamples - at;
        copy_in(ring_ + 2 * at, iq, first);
        copy_in(ring_, iq + 2 * first, count - first);
        w_ += static_cast<uint32_t>(count);
        return dropped;
    }

    /* Demodulates one block of the backlog: up to the state's group count,
     * never across the end of the ring, ending early where the state
     * changes. False when fewer than 4 samples were waiting. */
    bool step(Callback cb, void* ctx) {
        size_t groups = backlog() / 4;
        if (!groups) return false;
        const size_t at = n_ & kRingMask; /* n_ is a multiple of 4, so is the ring */
        const size_t to_end = (kRingSamples - at) / 4;
        const size_t most = state_ == State::Search ? kSearchGroups : (state_ == State::Sync ? kSyncGroups : kLockGroups);
        if (groups > to_end) groups = to_end;
        if (groups > most) groups = most;
        cb_ = cb;
        ctx_ = ctx;
        run(ring_ + 2 * at, groups);
        return true;
    }

    /* Blocks while the backlog lasts and elapsed() + block_bound <= limit,
     * elapsed() being the caller's clock (DWT cycles since the start of its
     * buffer on the M4, a cost model on the host). block_bound: the most one
     * step can take. */
    template <class Elapsed>
    void process(Elapsed&& elapsed, uint32_t limit, uint32_t block_bound, Callback cb, void* ctx) {
        while (backlog() >= 4 && elapsed() + block_bound <= limit) step(cb, ctx);
    }

    /* Everything at once (host tests; any count). */
    void execute(const int8_t* iq, size_t count, Callback cb, void* ctx) {
        while (count) {
            const size_t c = count > kRingSamples / 2 ? kRingSamples / 2 : count;
            push(iq, c);
            while (step(cb, ctx)) {
            }
            iq += 2 * c;
            count -= c;
        }
    }

   private:
    enum class State : uint8_t { Search,
                                 Sync,
                                 Lock };

    static constexpr uint32_t kRingMask = kRingSamples - 1;
    /* Samples behind the read position the ring must still hold: the CFO
     * pass at the SFD looks back over the whole Sync (<= 200 symbols). */
    static constexpr size_t kRetroSamples = 1024;
    /* The CFO pass sums over at most the last this many symbols before the
     * SFD: its leaky sums (time constant 32 symbols) weigh anything older
     * by < e^-2, and a whole Sync (up to ~120 symbols) made it the longest
     * step (~9 k cycles measured). */
    static constexpr uint32_t kCfoSymbols = 64;
    /* Mode 3 load shedding (run()): 2 phase search above kShedOn samples of
     * backlog, back to mode 3 below kShedOff. A frame's Sync and CFO pass
     * leave ~2000 samples; mode 3's Search drains them at ~9 cycles a group
     * of spare (measured: 107 of ~116), the 2 phase search ~4x faster. */
    static constexpr size_t kShedOn = 4096;
    static constexpr size_t kShedOff = 2048;
    static_assert((kRingSamples & kRingMask) == 0 && kRingSamples % 2048 == 0, "ring: power of two, whole DMA buffers");
    static constexpr size_t kHist = 6; /* MF history: 3 taps + up to 3 samples back (ysfd_) */
    static constexpr uint32_t kSfd = dsss_sfd_pattern();
    /* Phases with a true filter A output in Search (their y and raw-bit
     * history carries into Sync). */
    uint32_t search_mask() const { return search_phases() == 4 ? 0xF : 0x5; }

    /* Only whole groups are demodulated: the read position stays a multiple
     * of 4, so a group never straddles the end of the ring. */
    void drop_backlog() { n_ += static_cast<uint32_t>(backlog() & ~size_t{3}); }

    /* Word copies when aligned (the DMA buffers are), else bytes. (A loop:
     * GCC would call newlib-nano's byte-by-byte memcpy here.) */
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) static void copy_in(int8_t* dst, const int8_t* src, size_t samples) {
        if (((reinterpret_cast<uintptr_t>(dst) | reinterpret_cast<uintptr_t>(src)) & 3) == 0) {
            struct Words8 {
                uint32_t v[8];
            };
            size_t words = samples / 2;
            Words8* d8 = reinterpret_cast<Words8*>(dst);
            const Words8* s8 = reinterpret_cast<const Words8*>(src);
            for (; words >= 8; words -= 8) *d8++ = *s8++;
            uint32_t* d = reinterpret_cast<uint32_t*>(d8);
            const uint32_t* s = reinterpret_cast<const uint32_t*>(s8);
            while (words--) *d++ = *s++;
            if (samples & 1) {
                reinterpret_cast<int8_t*>(d)[0] = reinterpret_cast<const int8_t*>(s)[0];
                reinterpret_cast<int8_t*>(d)[1] = reinterpret_cast<const int8_t*>(s)[1];
            }
            return;
        }
        for (size_t i = 0; i < 2 * samples; i++) dst[i] = src[i];
    }

    /* ------------------------------------------------------------ front end */
    /* Unpack groups into packed int16 (I | Q << 16), at group `to` of the
     * block (after the history). */
    void unpack(const int8_t* p, size_t groups, size_t to = 0) {
        uint32_t* x = xb_ + kHist + 4 * to;
        for (size_t g = 0; g < groups; g++, p += 8, x += 4) {
            uint32_t w0, w1;
            __builtin_memcpy(&w0, p, 4);
            __builtin_memcpy(&w1, p + 4, 4);
            const uint32_t a0 = simd_sxtb16(w0), a1 = simd_sxtb16(w1);
            const uint32_t b0 = simd_sxtb16_ror8(w0), b1 = simd_sxtb16_ror8(w1);
            x[0] = simd_pack_lo(a0, b0);
            x[1] = simd_pack_hi(a0, b0);
            x[2] = simd_pack_lo(a1, b1);
            x[3] = simd_pack_hi(a1, b1);
        }
    }

    /* 8 dual MACs and a pack: |y| <= 4 * 128 * 16 * 2 = 2^14. */
    static uint32_t mf4(uint32_t x0, uint32_t x1, uint32_t x2, uint32_t x3, uint32_t h0, uint32_t h1, uint32_t h2, uint32_t h3) {
        int32_t re = simd_smusd(x0, h0);
        int32_t im = simd_smuadx(x0, h0);
        re = simd_smlsd(x1, h1, re);
        im = simd_smladx(x1, h1, im);
        re = simd_smlsd(x2, h2, re);
        im = simd_smladx(x2, h2, im);
        re = simd_smlsd(x3, h3, re);
        im = simd_smladx(x3, h3, im);
        return simd_pack_lo(static_cast<uint32_t>(re), static_cast<uint32_t>(im));
    }

    /* Search: filter A over the searched phases of the block straight from
     * the int8 samples (unpacked in registers, not stored: only Sync and
     * Lock read x back), taps and the x window in registers. The last three
     * samples of the history start the window. */
    void filter(const int8_t* p, size_t groups, int ph) {
        if (ph == 4)
            filter_t<4>(p, groups);
        else if (ph == 3)
            filter_t<3>(p, groups);
        else
            filter_t<2>(p, groups);
    }
    template <int P>
    __attribute__((noinline)) void filter_t(const int8_t* p, size_t groups) {
        const uint32_t h0 = h_[0][0], h1 = h_[0][1], h2 = h_[0][2], h3 = h_[0][3];
        uint32_t* y = yb_ + 4;
        uint32_t* const end = yb_ + 4 + 4 * groups;
        uint32_t a = xb_[kHist - 3], b = xb_[kHist - 2], c = xb_[kHist - 1];
        /* Mode 3: phase 2 of the group before (block history yb_[2]). Its
         * phase 3 is the mean of that and phase 0 of this group, so it is
         * stored one group late, in this group's slot 3: register 3 runs a
         * symbol behind, which the trigger does not mind; Sync starts it
         * afresh (search_mask()). */
        uint32_t p2 = yb_[2];
        for (; y != end; p += 8, y += 4) {
            uint32_t w0, w1;
            __builtin_memcpy(&w0, p, 4);
            __builtin_memcpy(&w1, p + 4, 4);
            const uint32_t i0 = simd_sxtb16(w0), q0 = simd_sxtb16_ror8(w0);
            const uint32_t d = simd_pack_lo(i0, q0), e = simd_pack_hi(i0, q0);
            const uint32_t y0 = mf4(d, c, b, a, h0, h1, h2, h3);
            y[0] = y0;
            if (P == 4) y[1] = mf4(e, d, c, b, h0, h1, h2, h3);
            const uint32_t i1 = simd_sxtb16(w1), q1 = simd_sxtb16_ror8(w1);
            const uint32_t f = simd_pack_lo(i1, q1), g = simd_pack_hi(i1, q1);
            const uint32_t y2 = mf4(f, e, d, c, h0, h1, h2, h3);
            y[2] = y2;
            if (P == 4) y[3] = mf4(g, f, e, d, h0, h1, h2, h3);
            if (P == 3) {
                y[1] = simd_shadd16(y0, y2);
                y[3] = simd_shadd16(p2, y0);
                p2 = y2;
            }
            a = e;
            b = f;
            c = g;
        }
    }

    /* One block in the current state, up to where the state changes (the
     * rest is unpacked again by the next step, which is rare: once a frame
     * or a false trigger). */
    __attribute__((noinline)) void run(const int8_t* p, size_t groups) {
        if (cfo_pending_) { /* a step of its own: ~7 k cycles */
            finish_cfo();
            return;
        }
        size_t g;
        if (state_ == State::Search) {
            /* Mode 3 sheds load on 2 phases while the backlog is long (a
             * busy channel: Syncs back to back), with hysteresis; back on 4
             * registers, 1 and 3 restart empty (24 zeros: no trigger until
             * they hold 31 fresh symbols). */
            int ph = search_phases();
            if (ph == 3) {
                const size_t b = backlog();
                if (b > kShedOn)
                    shedding_ = true;
                else if (b < kShedOff)
                    shedding_ = false;
                if (shedding_) ph = 2;
            }
            if (ph != 2 && searched2_) r_[1] = r_[3] = 0;
            searched2_ = ph == 2;
            filter(p, groups, ph);
            g = search(0, groups, ph);
            stats_.g_search += static_cast<uint32_t>(g);
            if (ph == 2 && search_phases() == 3) stats_.g_shed += static_cast<uint32_t>(g);
            /* x of the last two groups taken, for the history copy below. */
            const size_t from = g >= 2 ? g - 2 : 0;
            unpack(p + 8 * from, g - from, from);
        } else if (state_ == State::Sync) {
            unpack(p, groups);
            g = sync(0, groups);
            stats_.g_sync += static_cast<uint32_t>(g);
        } else {
            unpack(p, groups);
            g = lock(0, groups);
            stats_.g_lock += static_cast<uint32_t>(g);
        }
        for (size_t k = 0; k < kHist; k++) xb_[k] = xb_[4 * g + k];
        for (size_t k = 0; k < 4; k++) yb_[k] = yb_[4 * g + k];
        n_ += 4 * static_cast<uint32_t>(g);
    }

    static uint32_t sign(int32_t v) { return static_cast<uint32_t>(v) >> 31; }

    /* --------------------------------------------------------------- Search */
    size_t search(size_t g, size_t groups, int ph) { return ph != 2 ? search_t<true>(g, groups) : search_t<false>(g, groups); }
    /* kFourRegs: 4 bit registers (modes 3 and 4), else phases 0 and 2. */
    template <bool kFourRegs>
    __attribute__((noinline)) size_t search_t(size_t g, size_t groups) {
        const uint32_t* q = yb_ + 4 + 4 * g;
        uint32_t r0 = r_[0], r1 = r_[1], r2 = r_[2], r3 = r_[3];
        uint32_t p0 = q[-4], p1 = q[-3], p2 = q[-2], p3 = q[-1]; /* y a symbol back */
        const uint32_t inv = invert_;
        while (g < groups) {
            /* groups up to and including the next one with symbol = 0 mod 8 */
            size_t stop = g + ((8 - (((n_ >> 2) + g) & 7)) & 7) + 1;
            const bool check = stop <= groups;
            if (!check) stop = groups;
/* One symbol of every searched phase: y of group Q against P, the group
 * before (two of them a turn with the roles swapped: no register moves). */
#define DD_DSSS_BITS(Q, Y0, Y1, Y2, Y3, P0, P1, P2, P3)    \
    Y0 = (Q)[0];                                           \
    Y2 = (Q)[2];                                           \
    r0 = (r0 << 1) | (sign(simd_smuad(Y0, P0)) ^ inv);     \
    r2 = (r2 << 1) | (sign(simd_smuad(Y2, P2)) ^ inv);     \
    if (kFourRegs) {                                       \
        Y1 = (Q)[1];                                       \
        Y3 = (Q)[3];                                       \
        r1 = (r1 << 1) | (sign(simd_smuad(Y1, P1)) ^ inv); \
        r3 = (r3 << 1) | (sign(simd_smuad(Y3, P3)) ^ inv); \
    }
            uint32_t a0 = 0, a1 = 0, a2 = 0, a3 = 0;
            size_t n = stop - g;
            for (; n >= 2; n -= 2, q += 8) {
                DD_DSSS_BITS(q, a0, a1, a2, a3, p0, p1, p2, p3)
                DD_DSSS_BITS(q + 4, p0, p1, p2, p3, a0, a1, a2, a3)
            }
            if (n) {
                DD_DSSS_BITS(q, a0, a1, a2, a3, p0, p1, p2, p3)
                p0 = a0;
                p1 = a1;
                p2 = a2;
                p3 = a3;
                q += 4;
            }
#undef DD_DSSS_BITS
            search_syms_ += static_cast<uint32_t>(stop - g);
            g = stop;
            if (!check) break;
            const bool hit = kFourRegs ? sync_like4(r0, r1, r2, r3) : sync_like4(r0, r0, r2, r2);
            const bool fire = hit && !blocked_ && (armed_ || trig_checks_ < 2);
            /* Unblocked by a checkpoint without SYNC once the registers are
             * full again (31 symbols after to_search() emptied them). */
            if (!hit && search_syms_ >= 32) blocked_ = false;
            armed_ = hit;
            if (__builtin_expect(fire, 0)) {
                start_sync(g);
                break;
            }
        }
        r_[0] = r0;
        r_[1] = r1;
        r_[2] = r2;
        r_[3] = r3;
        return g;
    }

    /* Zeros among the last 24 descrambled bits of a raw-bit register. */
    static uint32_t sync_zeros(uint32_t r) { return ~(r ^ (r >> 4) ^ (r >> 7)) & 0xFFFFFF; }

    /* Any of the registers with at most trig_zeros_ zeros: clear that many
     * lowest set bits of the zeros (no popcount on the M4; clearing in an
     * empty word is harmless, so the default 3 is straight-line code). */
    __attribute__((noinline)) bool sync_like4(uint32_t ra, uint32_t rb, uint32_t rc, uint32_t rd) const {
        uint32_t za = sync_zeros(ra), zb = sync_zeros(rb), zc = sync_zeros(rc), zd = sync_zeros(rd);
        if (__builtin_expect(trig_zeros_ != 3, 0)) return sync_like_n(za) || sync_like_n(zb) || sync_like_n(zc) || sync_like_n(zd);
        za &= za - 1;
        zb &= zb - 1;
        zc &= zc - 1;
        zd &= zd - 1;
        za &= za - 1;
        zb &= zb - 1;
        zc &= zc - 1;
        zd &= zd - 1;
        za &= za - 1;
        zb &= zb - 1;
        zc &= zc - 1;
        zd &= zd - 1;
        return (za == 0) | (zb == 0) | (zc == 0) | (zd == 0);
    }
    __attribute__((noinline)) bool sync_like_n(uint32_t z) const {
        for (uint32_t t = trig_zeros_; t && z; t--) z &= z - 1;
        return z == 0;
    }

    /* Into Sync before group g: hypotheses (f, ph) = index 4 f + ph. Filter
     * A's registers of the searched phases keep their history; the rest
     * start empty (they fill long before the SFD). */
    void start_sync(size_t g) {
        stats_.triggers++;
        state_ = State::Sync;
        sync_left_ = sync_symbols_;
        pending_ = false;
        sync_start_ = n_ + 4 * static_cast<uint32_t>(g);
        const uint32_t* q = yb_ + 4 + 4 * g; /* y of the group before g at q[-4..-1] */
        const uint32_t mask = search_mask();
        for (int k = 0; k < 8; k++) {
            e_[k] = 0;
            if (k >= 4 || !((mask >> k) & 1)) r_[k] = 0;
            yp_[k] = (k < 4 && ((mask >> k) & 1)) ? q[k - 4] : 0;
            if (k < 4) yp_start_[k] = yp_[k];
        }
    }

    /* ----------------------------------------------------------------- Sync */
    /* Per sample: both filters' outputs (x window loaded once), then per
     * hypothesis the differential bit, its register, a leaky |y|^2 and the
     * SFD test; the rare SFD match is out of line. The CFO sums are done
     * once, for the winner only, at the SFD (cfo_pass()). */
    __attribute__((noinline)) size_t sync(size_t g, size_t groups) {
        /* Both filters over the block first (taps in registers), each after
         * its outputs of a symbol ago (yp_), then the hypotheses in the order
         * of samples and, per sample, filter A before B. */
        for (size_t k = 0; k < 4; k++) {
            yb_[k] = yp_[k];
            yq_[k] = yp_[4 + k];
        }
        filter_x(h_[0], yb_, groups);
        if (nf_ == 2) filter_x(h_[1], yq_, groups);
        const uint32_t inv = invert_;
        const bool two = nf_ == 2;
        for (; g < groups; g++) {
            const uint32_t* ya = yb_ + 4 + 4 * g;
            const uint32_t* yb = yq_ + 4 + 4 * g;
            for (uint32_t ph = 0; ph < 4; ph++) {
                const uint32_t n = n_ + static_cast<uint32_t>(4 * g + ph);
                hypothesis(ph, ya[ph], ya[static_cast<int>(ph) - 4], inv, n);
                if (two) hypothesis(4 + ph, yb[ph], yb[static_cast<int>(ph) - 4], inv, n);
                if (pending_ && n == pending_end_) {
                    pending_ = false;
                    start_lock();
                    return g; /* lock() skips the samples before next_sym_ */
                }
            }
            if (--sync_left_ == 0) {
                stats_.sync_lost++;
                blocked_ = rearm_; /* a SYNC-like signal that goes on: not again until it stops */
                to_search();
                return g + 1;
            }
        }
        for (size_t k = 0; k < 4; k++) {
            yp_[k] = yb_[4 * groups + k];
            yp_[4 + k] = yq_[4 * groups + k];
        }
        return g;
    }

    /* Hypothesis k at sample n: y now and a symbol ago. */
    void hypothesis(uint32_t k, uint32_t yn, uint32_t yp, uint32_t inv, uint32_t n) {
        const int32_t d = simd_smuad(yn, yp);
        const uint32_t r = (r_[k] << 1) | (sign(d) ^ inv);
        r_[k] = r;
        const int32_t e = e_[k];
        e_[k] = e - (e >> 4) + static_cast<int32_t>(static_cast<uint32_t>(simd_smuad(yn, yn)) >> 8);
        const uint32_t dsc = r ^ (r >> 4) ^ (r >> 7);
        if (__builtin_expect((dsc & 0xFFFF) == kSfd, 0)) sfd_match(k, dsc, n);
    }

    /* One filter, all 4 phases, over the unpacked block: y[4 + i] for
     * sample i (y[0..3] hold the outputs a symbol before the block). */
    __attribute__((noinline)) void filter_x(const uint32_t* taps, uint32_t* y, size_t groups) const {
        const uint32_t h0 = taps[0], h1 = taps[1], h2 = taps[2], h3 = taps[3];
        const uint32_t* x = xb_ + kHist;
        uint32_t* q = y + 4;
        uint32_t* const end = y + 4 + 4 * groups;
        uint32_t a = x[-3], b = x[-2], c = x[-1];
        for (; q != end; x += 4, q += 4) {
            const uint32_t d = x[0], e = x[1], f = x[2], gg = x[3];
            q[0] = mf4(d, c, b, a, h0, h1, h2, h3);
            q[1] = mf4(e, d, c, b, h0, h1, h2, h3);
            q[2] = mf4(f, e, d, c, h0, h1, h2, h3);
            q[3] = mf4(gg, f, e, d, h0, h1, h2, h3);
            a = e;
            b = f;
            c = gg;
        }
    }

    /* Carrier estimate of hypothesis k (filter k >> 2, phase k & 3) over its
     * symbols of this Sync, from its first (sync_start_ + phase) to the SFD's
     * last (best_n_), again from the ring: the leaky sums of its data-wiped
     * differential products (d + j e) sign(d), exactly as if they had been
     * kept per sample for all 8 hypotheses (<= 128 symbols, ~4 k cycles). */
    __attribute__((noinline)) void cfo_pass(uint32_t k, int32_t& wr, int32_t& wi) const {
        const uint32_t f = k >> 2, ph = k & 3;
        const uint32_t h0 = h_[f][0], h1 = h_[f][1], h2 = h_[f][2], h3 = h_[f][3];
        uint32_t yp = (f == 0 && ((search_mask() >> ph) & 1)) ? yp_start_[ph] : 0;
        int32_t sr = 0, si = 0;
        uint32_t t = sync_start_ + ph;
        bool prime = false; /* capped: the first symbol only gives yp */
        if (static_cast<int32_t>(best_n_ - t) > static_cast<int32_t>(4 * kCfoSymbols)) {
            t = best_n_ - 4 * kCfoSymbols;
            prime = true;
        }
        for (; static_cast<int32_t>(best_n_ - t) >= 0; t += 4) {
            uint32_t x0, x1, x2, x3;
            const uint32_t at = (t - 3) & kRingMask;
            if (at <= kRingMask - 3) { /* samples t-3..t in one run: two (unaligned) word loads */
                uint32_t w0, w1;
                __builtin_memcpy(&w0, ring_ + 2 * at, 4);
                __builtin_memcpy(&w1, ring_ + 2 * at + 4, 4);
                const uint32_t i0 = simd_sxtb16(w0), q0 = simd_sxtb16_ror8(w0), i1 = simd_sxtb16(w1), q1 = simd_sxtb16_ror8(w1);
                x3 = simd_pack_lo(i0, q0);
                x2 = simd_pack_hi(i0, q0);
                x1 = simd_pack_lo(i1, q1);
                x0 = simd_pack_hi(i1, q1);
            } else {
                x0 = ring_x(t);
                x1 = ring_x(t - 1);
                x2 = ring_x(t - 2);
                x3 = ring_x(t - 3);
            }
            const uint32_t yn = mf4(x0, x1, x2, x3, h0, h1, h2, h3);
            if (prime) {
                yp = yn;
                prime = false;
                continue;
            }
            const int32_t d = simd_smuad(yn, yp), e = simd_smusdx(yp, yn);
            yp = yn;
            const bool neg = d < 0;
            sr += (neg ? -(d >> 6) : (d >> 6)) - (sr >> 5);
            si += (neg ? -(e >> 6) : (e >> 6)) - (si >> 5);
        }
        wr = sr;
        wi = si;
    }

    /* Packed x of sample t from the ring. */
    uint32_t ring_x(uint32_t t) const {
        const int8_t* p = ring_ + 2 * (t & kRingMask);
        return static_cast<uint32_t>(static_cast<uint16_t>(p[0])) | (static_cast<uint32_t>(static_cast<uint16_t>(p[1])) << 16);
    }

    /* SFD bits matched on hypothesis k at sample n: check the ones before it,
     * open or extend the 4 sample window, keep the strongest. */
    __attribute__((noinline)) void sfd_match(uint32_t k, uint32_t dsc, uint32_t n) {
        const uint32_t z = (~dsc >> 16) & 0x1FF;
        if (z & (z - 1)) return; /* more than one of the 9 bits before the SFD not one */
        if (!pending_) {
            pending_ = true;
            pending_end_ = n + 3;
            best_e_ = -1;
        }
        if (e_[k] > best_e_) {
            best_e_ = e_[k];
            best_k_ = k;
            best_n_ = n;
        }
    }

    void to_search() {
        state_ = State::Search;
        armed_ = false;
        search_syms_ = 0;
        for (int k = 0; k < 4; k++) r_[k] = 0;
    }

    void start_lock() {
        const uint32_t k = best_k_;
        stats_.sfd++;
        state_ = State::Lock;
        lock_f_ = k >> 2;
        lock_phase_ = k & 3;
        rlock_ = r_[k];
        next_sym_ = best_n_ + 4;
        ylast_ = ysfd_(k);
        bits_ = 0;
        hdr_ = 0;
        nbytes_ = 0;
        total_bytes_ = 0;
        byte_ = 0;
        crc_ = 0xFFFFFFFFu;
        power_ = 0;
        cfo_pending_ = cfo_; /* the next step, before any symbol of Lock */
    }

    /* The winner's carrier estimate, scaled to 13 bits for Lock's 64 bit
     * decision products. */
    __attribute__((noinline)) void finish_cfo() {
        cfo_pending_ = false;
        stats_.cfo_passes++;
        int32_t wr, wi;
        cfo_pass(best_k_, wr, wi);
        int32_t a = wr < 0 ? -wr : wr, b = wi < 0 ? -wi : wi;
        int32_t m = a > b ? a : b;
        int sh = 0;
        while (m > (1 << 12)) {
            m >>= 1;
            sh++;
        }
        cr_ = wr >> sh;
        ci_ = wi >> sh;
        if (cr_ == 0 && ci_ == 0) cr_ = 1;
    }

    /* y of the winning hypothesis at the SFD's last symbol (best_n_), which
     * is at most 3 samples back, so still in the unpacked block or its
     * history: recomputed rather than stored for all 8 hypotheses. */
    uint32_t ysfd_(uint32_t k) const {
        const int32_t i = static_cast<int32_t>(best_n_ - n_); /* block index, >= -3 */
        const uint32_t* x = xb_ + kHist + i;
        const uint32_t f = k >> 2;
        return mf4(x[0], x[-1], x[-2], x[-3], h_[f][0], h_[f][1], h_[f][2], h_[f][3]);
    }

    /* ----------------------------------------------------------------- Lock */
    /* One decision a symbol on the winning phase, the frame state in
     * registers; the 48 header bits and each completed byte out of line. */
    __attribute__((noinline)) size_t lock(size_t g, size_t groups) {
        const uint32_t ph = lock_phase_, f = lock_f_;
        /* Catch up after the SFD: the first symbol may lie a group or two on. */
        while (g < groups && static_cast<int32_t>(n_ + static_cast<uint32_t>(4 * g + ph) - next_sym_) < 0) g++;
        if (g == groups) return g;
        const uint32_t h0 = h_[f][0], h1 = h_[f][1], h2 = h_[f][2], h3 = h_[f][3];
        const uint32_t inv = invert_;
        const int32_t cr = cr_, ci = ci_;
        const bool cfo = cfo_;
        uint32_t r = rlock_, yp = ylast_, power = power_, bits = bits_, byte = byte_;
        const size_t g0 = g;
        bool over = false;
        const uint32_t* x = xb_ + kHist + 4 * g + ph;
        for (; g < groups; g++, x += 4) {
            const uint32_t x0 = x[0], x1 = x[-1], x2 = x[-2], x3 = x[-3];
            const uint32_t yn = mf4(x0, x1, x2, x3, h0, h1, h2, h3);
            uint32_t neg;
            if (cfo) {
                const int64_t vv = static_cast<int64_t>(simd_smuad(yn, yp)) * cr + static_cast<int64_t>(simd_smusdx(yp, yn)) * ci;
                neg = vv < 0 ? 1u : 0u;
            } else {
                neg = sign(simd_smuad(yn, yp));
            }
            yp = yn;
            /* |x|^2 of the symbol's 4 samples (not just the decision's, which
             * sits on the pulse peak: ~5 dB high). */
            power = static_cast<uint32_t>(simd_smlad(x3, x3, simd_smlad(x2, x2, simd_smlad(x1, x1, simd_smlad(x0, x0, static_cast<int32_t>(power))))));
            const uint32_t raw = neg ^ inv;
            const uint32_t s = raw ^ ((r >> 3) & 1) ^ ((r >> 6) & 1);
            r = (r << 1) | raw;
            if (__builtin_expect(bits < 48, 0)) {
                if (!header_bit(s, bits)) {
                    over = true;
                    g++;
                    break;
                }
                continue;
            }
            byte |= s << (bits & 7);
            bits++;
            if ((bits & 7) == 0) {
                power_ = power;
                if (!byte_in(static_cast<uint8_t>(byte), n_ + static_cast<uint32_t>(4 * g + ph))) {
                    over = true;
                    g++;
                    break;
                }
                byte = 0;
            }
        }
        rlock_ = r;
        ylast_ = yp;
        power_ = power;
        bits_ = bits;
        byte_ = byte;
        next_sym_ = n_ + static_cast<uint32_t>(4 * g + ph);
        stats_.symbols += static_cast<uint32_t>(g - g0);
        if (over) to_search();
        return g;
    }

    /* One of the 48 PLCP header bits (bits: count so far, advanced); false
     * when the header ends the frame (CRC, rate, length). */
    __attribute__((noinline)) bool header_bit(uint32_t s, uint32_t& bits) {
        hdr_ |= static_cast<uint64_t>(s) << bits;
        bits++;
        if (bits < 48) return true;
        const uint32_t b32 = static_cast<uint32_t>(hdr_);
        const uint32_t crc = static_cast<uint32_t>(hdr_ >> 32) & 0xFFFF;
        if (plcp_crc16(b32) != crc) {
            stats_.hdr_bad++;
            return false;
        }
        const uint32_t signal = b32 & 0xFF, length_us = b32 >> 16;
        total_bytes_ = length_us / 8;
        if (signal == kPlcpSignal2M) {
            stats_.hdr_2m++; /* hook: DQPSK (2 bits a symbol) would go here */
            return false;
        }
        if (signal != kPlcpSignal1M || total_bytes_ < 14 || total_bytes_ > kDsssMaxMpdu) {
            stats_.not_1m++;
            return false;
        }
        return true;
    }

    /* A complete MPDU byte (its last bit at sample n): CRC-32, frame control
     * filter, and at the last byte the FCS check and the callback. False
     * when the frame is over. */
    __attribute__((noinline)) bool byte_in(uint8_t b, uint32_t n) {
        frame_.mpdu[nbytes_++] = b;
        crc_ = crc32_byte(crc_, b);
        if (nbytes_ == 1 && mgmt_only_ && b != 0x80 && b != 0x50 && b != 0xD0) {
            stats_.filtered++;
            return false;
        }
        if (nbytes_ < total_bytes_) return true;
        if (crc_ != kCrc32Residue) {
            stats_.fcs_bad++;
            return false;
        }
        stats_.ok++;
        frame_.length = static_cast<uint16_t>(nbytes_);
        frame_.phase = static_cast<uint8_t>(lock_phase_);
        frame_.filter = static_cast<uint8_t>(lock_f_);
        /* Mean |x|^2 over the frame's samples, x 8 (4 samples <= 2^17 a
         * symbol, <= 4144 symbols: power < 2^30, twice that < 2^31). */
        const uint32_t nsym = 48 + 8 * nbytes_;
        const uint32_t mean8 = (power_ << 1) / nsym;
        frame_.rssi_cdb = static_cast<int16_t>(power_to_cdb(mean8 ? mean8 : 1) - 903 - kDsssFullScaleCdb + kDsssSliceToChannelCdb);
        frame_.end_sample = n;
        if (cb_) cb_(ctx_, frame_);
        return false;
    }

    /* State (all zero-initialised: a static object lands in .bss) */
    uint32_t w_{0}; /* samples pushed (wrapping) */
    uint32_t n_{0}; /* samples demodulated (wrapping): the next block's first */
    uint32_t h_[2][4]{};
    uint32_t xb_[kHist + 4 * kBlockGroups]{};
    uint32_t yb_[4 + 4 * kBlockGroups]{}; /* filter A output (Search, Sync) */
    uint32_t yq_[4 + 4 * kSyncGroups]{};  /* filter B output (Sync) */
    uint32_t r_[8]{};                     /* raw-bit registers: Search uses 0..3 */
    uint32_t yp_[8]{};                    /* y a symbol back, per Sync hypothesis */
    int32_t e_[8]{};
    uint32_t yp_start_[4]{};  /* filter A's y a symbol before the Sync, per phase (cfo_pass) */
    uint32_t sync_start_{0};  /* first sample of the Sync */
    uint32_t search_syms_{0}; /* symbols in Search since to_search() */
    bool cfo_pending_{false}; /* Lock started, its carrier estimate not yet made */
    State state_{State::Search};
    uint32_t nf_{0};
    uint32_t invert_{0};
    bool cfo_{false};
    bool mgmt_only_{false};
    uint32_t trig_zeros_{0};
    uint32_t trig_checks_{0}; /* all members zero: a static object lands in .bss */
    uint8_t phases_{0};       /* search mode, 0: kSearchPhases */
    bool shedding_{false};    /* mode 3 searching on 2 phases (backlog) */
    bool searched2_{false};   /* the last Search block ran on 2 phases (registers 1, 3 stale) */
    uint32_t sync_symbols_{0};
    bool rearm_{false};
    bool armed_{false};   /* the last checkpoint had a SYNC-like register */
    bool blocked_{false}; /* rearm_: a Sync timed out, no trigger until a checkpoint without */
    uint32_t sync_left_{0};
    bool pending_{false};
    uint32_t pending_end_{0};
    int32_t best_e_{0};
    uint32_t best_k_{0};
    uint32_t best_n_{0};
    uint32_t lock_phase_{0};
    uint32_t lock_f_{0};
    uint32_t rlock_{0};
    uint32_t ylast_{0};
    uint32_t next_sym_{0};
    int32_t cr_{0}, ci_{0};
    uint32_t bits_{0};
    uint64_t hdr_{0};
    uint32_t byte_{0};
    uint32_t nbytes_{0};
    uint32_t total_bytes_{0};
    uint32_t crc_{0};
    uint32_t power_{0};
    Callback cb_{nullptr};
    void* ctx_{nullptr};
    Stats stats_{};
    DsssFrame frame_{};
    /* Last: the hot members above stay within a load's 4 KB offset reach. */
    alignas(4) int8_t ring_[2 * kRingSamples]{};
};

} /* namespace dd */

#endif /*__DD_DSSS_H__*/
