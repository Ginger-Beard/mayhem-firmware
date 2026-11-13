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

/* Track view (fox hunt) logic of the Signature Detect app: what to track, the
 * meter's peak hold / min / max / history, the Geiger beep, the BLE packet
 * rate. Integer only, no UI or ChibiOS dependency (host testable). Levels in
 * centi-dB, times in ms. */

#ifndef __SIGDET_TRACK_HPP__
#define __SIGDET_TRACK_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::signature_detect {

/* Measurement bandwidths of the M4 meter (sigdet_meter.hpp): a boxcar of
 * 2^log2d samples at 4 Msps, -3 dB ~0.89 x 4 MHz / 2^log2d. */
constexpr size_t kTrackBws = 4;
constexpr uint8_t kTrackBwLog2[kTrackBws] = {2, 4, 6, 8};
constexpr uint16_t kTrackBwKhz[kTrackBws] = {890, 220, 55, 14};
/* The narrowest bandwidth that still holds an emitter bw_khz wide (0: the
 * widest). */
size_t track_bw_for(uint32_t bw_khz);

struct TrackTarget {
    enum Kind : uint8_t {
        FREQ,     /* MODE_METER at f_khz, bandwidth bw */
        BLE_ADDR, /* one advertiser (FLAG_TARGET) */
        BLE_RULE, /* the class's BLE rule `rule` (rotating addresses): every advert, rate capped */
        ZW_NET,   /* a Z-Wave network: home ID in addr[0..3], valid frames only (PSDT MODE_ZWAVE) */
    };
    Kind kind{FREQ};
    uint32_t f_khz{462160};
    uint8_t bw{1}; /* kTrackBw* index */
    uint8_t addr[6]{};
    uint8_t rule{0};
    char label[24]{};
};

/* Meter statistics of the track view. */
class TrackStats {
   public:
    static constexpr size_t kHist = 200; /* columns: 20 s at 10 per second */
    static constexpr uint32_t kColumnMs = 100;
    static constexpr int16_t kDecayCdbPerS = 1000; /* peak hold falls 10 dB a second */
    static constexpr int16_t kNone = -32768;

    void reset_minmax(uint32_t now_ms);
    void clear(uint32_t now_ms);
    /* One measurement (the M4's 50 ms pulse peak, or one BLE packet). */
    void add(int16_t cdb, uint32_t now_ms);
    /* Time passes: peak hold decay, history columns (call at >= 10 Hz). */
    void tick(uint32_t now_ms);

    int16_t level() const { return level_; } /* latest measurement, kNone none */
    int16_t peak_hold() const { return hold_; }
    int16_t min() const { return min_; }
    int16_t max() const { return max_; }
    uint32_t last_ms() const { return last_ms_; }
    /* Strongest measurement since the last call (for the 1 Hz log row). */
    int16_t take_second_max();
    /* History column i (0 = oldest), dB (cdB / 100), -128 = nothing. */
    int8_t column(size_t i) const { return hist_[(head_ + i) % kHist]; }

   private:
    int16_t level_{kNone}, hold_{kNone}, min_{kNone}, max_{kNone}, sec_max_{kNone};
    uint32_t last_ms_{0}, hold_ms_{0}, col_ms_{0};
    int8_t hist_[kHist]{};
    int8_t col_max_{-128};
    size_t head_{0}; /* oldest column */
    bool started_{false};

    void push_column(int8_t v);
};

/* Geiger beep: none below the squelch; above it the beeps come faster and
 * higher with the level: interval 1000 ms at the squelch down to 60 ms at
 * +40 dB, pitch 500 Hz up to 2500 Hz. */
class Geiger {
   public:
    static constexpr int16_t kSpanCdb = 4000;
    static constexpr uint32_t kSlowMs = 1000;
    static constexpr uint32_t kFastMs = 60;
    /* Whether a beep is due now; hz and ms of it when so. */
    bool due(int16_t level_cdb, int16_t squelch_cdb, uint32_t now_ms, uint16_t& hz, uint16_t& ms);

   private:
    uint32_t last_ms_{0};
    bool started_{false};
};

/* Packets per second of a BLE / Z-Wave target over the last 10 s (an
 * advertiser every 3 s reads 0.33, not 0.0); past kMax packets in the window,
 * from the span of the last kMax. */
class PacketRate {
   public:
    static constexpr size_t kMax = 64;
    static constexpr uint32_t kWindowMs = 10000;
    void add(uint32_t now_ms);
    /* x100 */
    uint32_t rate100(uint32_t now_ms) const;

   private:
    uint32_t t_[kMax]{};
    size_t n_{0}, head_{0};
};

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_TRACK_HPP__*/
