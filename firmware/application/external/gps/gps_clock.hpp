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

#ifndef __GPS_CLOCK_H__
#define __GPS_CLOCK_H__

/* Sub-second time for the GPS app (no UI, host-tested).
 *
 * The RTC counts whole seconds. Comparing a GPS time of some past instant
 * with a whole-second RTC reading taken later gives an offset that jumps by
 * a second whenever an RTC second ticked in between (seen on the device:
 * -952 / +48 ms alternating). Instead:
 *  - RtcPhase: the RTC's 1 Hz interrupt marks where its second starts on the
 *    app's millisecond clock, so the RTC reads to ~10 ms (event latency);
 *  - SampleClock: the baseband's sample count at each status message maps a
 *    measurement's sample to the app's millisecond clock;
 *  - the offset compares the GPS time and the RTC at the same instant (now). */

#include <cstdint>

namespace ui::external_app::gps {

struct RtcPhase {
    uint32_t tick_s{0};  /* RTC reading (whole seconds) when its second started */
    uint32_t tick_ms{0}; /* the app's ms clock at that moment */
    bool ok{false};

    /* the RTC's 1 Hz tick, or a write (which restarts the RTC's second) */
    void on_tick(uint32_t rtc_s, uint32_t now_ms) {
        tick_s = rtc_s;
        tick_ms = now_ms;
        ok = true;
    }
    /* fraction of the RTC second now; 0 while the phase is unknown or the
     * reading is not the second the last tick started */
    double frac(uint32_t rtc_s, uint32_t now_ms) const {
        if (!ok || rtc_s != tick_s) return 0.0;
        const int32_t d = static_cast<int32_t>(now_ms - tick_ms);
        if (d <= 0) return 0.0;
        return d >= 999 ? 0.999 : static_cast<double>(d) / 1000.0;
    }
};

struct SampleClock {
    static constexpr double kFs = 2048000.0;
    uint64_t sample{0}; /* the image's sample count ... */
    uint32_t ms{0};     /* ... when the app's ms clock read this */
    bool ok{false};

    void at_status(uint64_t sample_now, uint32_t now_ms) {
        sample = sample_now;
        ms = now_ms;
        ok = true;
    }
    void reset() { ok = false; }
    /* the app's ms clock at sample s (s may be before or after the anchor),
     * as an offset from `now_ms`: negative for the past */
    double ms_before_now(uint64_t s, uint32_t now_ms) const {
        const double anchor_age = static_cast<double>(static_cast<int32_t>(now_ms - ms));
        const double ds = static_cast<double>(static_cast<int64_t>(s - sample)) * 1000.0 / kFs;
        return ds - anchor_age;
    }
};

/* GPS time now, from the GPS time at a sample and that sample's place on the
 * app's clock (ms relative to now, negative = past) */
inline double gps_now_from_sample(double gps_at_sample, double sample_ms_rel_now) {
    return gps_at_sample - sample_ms_rel_now / 1000.0;
}

}  // namespace ui::external_app::gps

#endif /*__GPS_CLOCK_H__*/
