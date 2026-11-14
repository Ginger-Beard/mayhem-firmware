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

/* Wi-Fi frame hand-off of the Signature Detect baseband (PSDT, HackRF Pro),
 * and the lease of its memory to the mains hum ladder. Header only, no
 * ChibiOS (host tested).
 *
 * Hand-off: FCS valid frames go from the baseband thread (MODE_DSSS, put())
 * to the event thread (drain()) through two slots, no lock: head counts the
 * slots filled, tail the slots done; both wrap at 256.
 *
 * Lease: outside Wi-Fi dwells the slots are idle. A hum dwell (MODE_METER
 * with FLAG_HUM) keeps its Goertzel bank (sigdet::HumLadder, ~630 B) in their
 * memory instead of in static RAM. One slot (532 B) is too small for the
 * bank, so the lease takes both (1,064 B); it needs both empty anyway.
 *
 * Threads: one core; the baseband thread preempts the event thread, never
 * the reverse. put(), lend_hum() and return_hum() run on the baseband
 * thread, drain() on the event thread.
 *
 * INVARIANT: every byte of the slots has exactly one owner at a time:
 *   - put() (baseband thread): slot head % 2, from its check head - tail < 2
 *     until the store head + 1;
 *   - drain() (event thread): slot tail % 2, from its check tail != head until
 *     the store tail + 1 (its wipe of the frame is before that store);
 *   - the hum ladder (baseband thread): both slots, while hum_ != nullptr;
 *   - nobody otherwise: an empty slot holds zeros or a wiped frame.
 * lend_hum() hands the memory to the ladder only when it reads head == tail.
 * Then no slot is being filled (put() runs on this thread, so it is not in
 * progress), and none is being drained: the event thread touches slot bytes
 * only between reading tail != head and its store tail + 1, and it can move
 * tail up to head only. While lent, head does not move: put() refuses, and it
 * runs only in MODE_DSSS, which the processor never enters before
 * return_hum(). So tail == head through the lease, and the event thread reads
 * no slot byte. return_hum() destroys the ladder and re-creates both slots
 * zeroed (frame length 0) with head == tail: empty by the index protocol, and
 * hum state can never be parsed as a frame, even by mistake.
 * Ordering: one core, so a compiler barrier between the slot accesses and
 * the index store (or after the index load) is enough; byte loads and
 * stores of the volatile indexes are atomic. */

#ifndef __SIGDET_WIFI_SLOTS_HPP__
#define __SIGDET_WIFI_SLOTS_HPP__

#include "sigdet_dsss.hpp"
#include "sigdet_hum.hpp"
#include "sigdet_meter.hpp"

#include <cstddef>
#include <cstdint>
#include <new>

namespace sigdet {

/* One FCS valid frame waiting for the event thread. */
struct WifiSlot {
    DsssFrame frame;
    uint32_t t_ms;
    uint8_t channel;
    int8_t slice;
    bool legacy;
};

class WifiSlots {
   public:
    static constexpr uint8_t kSlots = 2;

    /* Baseband thread: the frame into the free slot, then published. False
     * when both slots are busy (or lent): the frame is dropped. By words: GCC
     * would call newlib-nano's byte memcpy. */
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) bool put(const DsssFrame& frame, uint32_t t_ms, uint8_t channel, int8_t slice, bool legacy) {
        const uint8_t head = head_;
        if (hum_ || static_cast<uint8_t>(head - tail_) >= kSlots) return false;
        barrier(); /* the slot after the index check */
        WifiSlot& s = slot_[head % kSlots];
        const size_t words = (static_cast<size_t>(frame.length) + 3u) / 4u;
        const uint32_t* src = reinterpret_cast<const uint32_t*>(frame.mpdu);
        uint32_t* dst = reinterpret_cast<uint32_t*>(s.frame.mpdu);
        for (size_t i = 0; i < words && i < kDsssMaxMpdu / 4; i++) dst[i] = src[i];
        s.frame.length = frame.length;
        s.frame.rssi_cdb = frame.rssi_cdb;
        s.t_ms = t_ms;
        s.channel = channel;
        s.slice = slice;
        s.legacy = legacy;
        barrier(); /* the slot before the index */
        head_ = static_cast<uint8_t>(head + 1);
        return true;
    }

    /* Event thread: fn(const WifiSlot&) for every waiting slot, oldest
     * first, then the frame is wiped and the slot freed (no copy of an
     * ordinary frame outlives this). */
    template <class F>
    __attribute__((optimize("no-tree-loop-distribute-patterns"))) void drain(F&& fn) {
        while (tail_ != head_) {
            barrier(); /* the slot after the index check */
            const uint8_t tail = tail_;
            WifiSlot& s = slot_[tail % kSlots];
            fn(static_cast<const WifiSlot&>(s));
            uint32_t* w = reinterpret_cast<uint32_t*>(s.frame.mpdu);
            const size_t words = (static_cast<size_t>(s.frame.length) + 3u) / 4u;
            for (size_t i = 0; i < words && i < kDsssMaxMpdu / 4; i++) w[i] = 0;
            barrier(); /* the wipe before the index */
            tail_ = static_cast<uint8_t>(tail + 1);
        }
    }

    /* Baseband thread: the slots' memory as a fresh hum ladder (not yet
     * configured), or nullptr while a frame is waiting or being drained
     * (head != tail): the caller waits and asks again. */
    HumLadder* lend_hum() {
        if (hum_) return hum_;
        if (head_ != tail_) return nullptr;
        barrier(); /* the memory after the index check */
        hum_ = new (static_cast<void*>(slot_)) HumLadder{};
        return hum_;
    }

    /* Baseband thread: the end of the lease. The ladder goes, both slots are
     * re-created zeroed (length 0); head == tail as at the start. */
    void return_hum() {
        if (!hum_) return;
        hum_->~HumLadder();
        for (auto& s : slot_) new (&s) WifiSlot{};
        barrier();
        hum_ = nullptr;
    }

    HumLadder* hum() const { return hum_; }
    bool lent() const { return hum_ != nullptr; }
    /* No frame waiting or being drained. */
    bool idle() const { return head_ == tail_; }
    uint8_t pending() const { return static_cast<uint8_t>(head_ - tail_); }
    /* The raw memory (host tests). */
    const uint8_t* bytes() const { return reinterpret_cast<const uint8_t*>(slot_); }
    static constexpr size_t kBytes = sizeof(WifiSlot) * kSlots;

   private:
    WifiSlot slot_[kSlots]{};
    volatile uint8_t head_{0};
    volatile uint8_t tail_{0};
    HumLadder* hum_{nullptr};

    static void barrier() { __asm__ volatile("" ::: "memory"); }
};

static_assert(sizeof(HumLadder) <= WifiSlots::kBytes, "the hum ladder must fit in the Wi-Fi slots");
static_assert(alignof(HumLadder) <= alignof(WifiSlot), "the hum ladder's alignment must fit the Wi-Fi slots'");

/* The hum dwell on the M4 (MODE_METER with FLAG_HUM), baseband thread only:
 * the meter's 256 us block powers into a ladder in the lent slots. The
 * processor and the host tests run exactly this. */
class HumDwell {
   public:
    /* After meter.configure(): false while a frame is waiting or being
     * drained (the caller feeds nothing and asks again with the next
     * buffer). */
    bool begin(WifiSlots& slots, PowerMeter& meter) {
        HumLadder* h = slots.lend_hum();
        if (!h) return false;
        h->configure(kHumBlockRate, kHumWindowBlocks);
        meter.set_hum(h);
        window_ = 0;
        clip_ = clip_sum_ = 0;
        mean_ = 0.0f;
        return true;
    }
    /* After each meter.feed() of the dwell (which also feeds the ladder):
     * true when a ladder window ended: lv, and window() / mean() / clip()
     * for it. The meter's own reports are taken and dropped (only their clip
     * counts are kept). */
    bool poll(WifiSlots& slots, PowerMeter& meter, HumLevels& lv) {
        HumLadder* h = slots.hum();
        if (!h) return false;
        MeterReport r;
        if (meter.take(r)) clip_sum_ += r.clip;
        if (!h->take(lv)) return false;
        if (window_ < 0xFF) window_++;
        mean_ = h->window_mean();
        clip_ = clip_sum_;
        clip_sum_ = 0;
        return true;
    }
    /* The end of the dwell (any new configuration): the meter lets go of the
     * ladder, the slots are returned empty. */
    void end(WifiSlots& slots, PowerMeter& meter) {
        meter.set_hum(nullptr);
        slots.return_hum();
    }
    uint8_t window() const { return window_; } /* windows ended, 1-based */
    float mean() const { return mean_; }       /* the last window's mean block power */
    uint32_t clip() const { return clip_; }    /* raw samples at |x| >= 120 in about the last window */

   private:
    uint8_t window_{0};
    float mean_{0.0f};
    uint32_t clip_{0}, clip_sum_{0};
};

} /* namespace sigdet */

#endif /*__SIGDET_WIFI_SLOTS_HPP__*/
