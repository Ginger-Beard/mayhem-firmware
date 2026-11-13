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

/* Pointing aids for one selected satellite (GPS or TLE): compass words, the
 * pass of interest (the current one, else the next) found in resumable units,
 * the pass arc sampled for the sky plot, and the plain-words text of the
 * pointing guide. No UI; host tested (rf_test/gps/gps_orbit_test.cpp). */

#ifndef __GPS_POINT_H__
#define __GPS_POINT_H__

#include <cstddef>
#include <cstdint>

#include "gps_orbit.hpp"
#include "gps_pass.hpp"

namespace ui::external_app::gps {

/* 16-point compass word for an azimuth in degrees (0 = N, clockwise). */
const char* compass16(double az_deg);

/* The pass to point at: the one in progress, else the next within 24 h,
 * then its arc (kPts samples rise..set) and the azimuth at the peak. Work in
 * units of one elevation each (step()), so a caller can slice it. */
class PassTrack {
   public:
    static constexpr int kPts = 24;
    /* src must stay valid while busy. leo: TLE satellites (30 s coarse step,
     * look back 30 min); else GPS (120 s, look back 7 h). */
    void start(const SatSource& src, const Observer& obs, double now, bool leo);
    bool step(int units); /* true while there is more to do */
    void clear() { state_ = IDLE; }
    bool busy() const { return state_ == FIND || state_ == SAMPLE; }
    bool ready() const { return state_ == READY; }
    bool none() const { return state_ == NONE; } /* no pass within 24 h */
    const Pass& pass() const { return pass_; }
    float top_az() const { return top_az_; }
    int points() const { return n_; }
    int16_t az(int i) const { return az_[i]; }
    int16_t el(int i) const { return el_[i]; }

   private:
    enum State : uint8_t { IDLE,
                           FIND,
                           SAMPLE,
                           READY,
                           NONE };
    State state_{IDLE};
    SatSource src_{};
    Observer obs_{};
    PassFinder finder_{};
    Pass pass_{};
    double now_{0}, step_{30}, end_{0};
    float top_az_{0};
    int k_{0}, n_{0};
    int16_t az_[kPts]{}, el_[kPts]{};
};

/* "14:02" in local time (unix UTC + offset minutes). */
void put_clock(char* buf, size_t size, double unix_s, int32_t off_min);
/* "NW 310, 24 up" */
void put_point(char* buf, size_t size, double az, double el);
/* "rises in 5m12" / "peak in 2m05" / "sets in 7m30" / "set 3m ago" */
void put_countdown(char* buf, size_t size, const Pass& p, double now);
/* Three lines of the plain pass summary (local time, compass words):
 * "Rises 14:02 NW 310", "Peak  14:07 42 up, SW", "Sets  14:12 SE 135"
 * ("Up now since 13:58" / "Sets after 24 h" where they apply). */
void pass_summary(const Pass& p, float top_az, double now, int32_t off_min, char* l1, char* l2, char* l3, size_t size);

}  // namespace ui::external_app::gps

#endif /*__GPS_POINT_H__*/
