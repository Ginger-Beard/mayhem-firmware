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

/* Follow: the model behind the Doppler-corrected receive view (no UI, no
 * radio calls; host tested in rf_test/gps/gps_orbit_test.cpp).
 *   look_sgp4   where the satellite is and how fast it recedes, from ONE
 *               SGP4 evaluation (position and velocity), so a 4 Hz update
 *               costs one propagation
 *   Tuner       hardware centre + the audio baseband's DDC offset (the stock
 *               "sliding" shift of the NFM and AM images): the PLL is set
 *               once, the Doppler is followed in the baseband
 *   Meter       level, peak hold, min/max and the history strip
 *   FadeHint    "try rotating the antenna" when the level fades smoothly
 *               while the satellite is well up
 *   tuning_error_hz  the residual error from clock, TLE age and position */

#ifndef __GPS_FOLLOW_H__
#define __GPS_FOLLOW_H__

#include <cstddef>
#include <cstdint>

#include "gps_orbit.hpp"
#include "gps_sgp4.hpp"

namespace ui::external_app::gps {

struct Look {
    double az{0}, el{-90}; /* degrees */
    double range_m{0};
    double rr{0}; /* range rate, m/s: > 0 receding (frequency low) */
};

/* One SGP4 propagation at unix time t: az/el, range and range rate seen from
 * a fixed observer (the TEME velocity rotated to ECEF minus the earth's
 * rotation). false if the propagator failed. */
bool look_sgp4(Sgp4& s, const Observer& obs, double unix_s, Look& out);

/* Received frequency offset (Hz) of a downlink f0 at range rate rr:
 * f0 * (1 - rr / c) - f0. */
int32_t doppler_hz(uint64_t f0, double rr);

/* Hardware centre and DDC offset. With a DDC (NFM / AM images) the centre is
 * set once (f0 + the mode offset) and the Doppler goes to the DDC, which the
 * stock sliding feature allows up to +-50 kHz; past kDdcLimit the centre
 * moves. Without one (WFM) the centre itself is retuned in kRetuneStep
 * steps. */
class Tuner {
   public:
    static constexpr int32_t kDdcLimit = 40000;  /* Hz; the stock limit is 50 kHz */
    static constexpr int32_t kDdcStep = 5;       /* resend the DDC on this change */
    static constexpr int32_t kRetuneStep = 1000; /* no DDC: retune on this change */
    struct Cmd {
        bool retune{false};
        uint64_t center{0};
        bool ddc{false};
        int32_t ddc_hz{0};
    };
    /* offset: added to every frequency (CW: -700 Hz, so a beacon is a
     * 700 Hz tone in USB) */
    void reset(uint64_t f0, bool has_ddc, int32_t offset);
    /* the baseband was reconfigured: its DDC is back at 0 */
    void ddc_lost() {
        ddc_sent_ = 0;
        ddc_valid_ = false;
    }
    Cmd update(int32_t doppler);
    uint64_t f0() const { return f0_; }
    uint64_t center() const { return center_; }
    int32_t ddc() const { return ddc_sent_; }
    /* the frequency listened to now */
    int64_t listening() const { return static_cast<int64_t>(center_) + ddc_sent_; }

   private:
    uint64_t f0_{0}, center_{0};
    int32_t offset_{0}, ddc_sent_{0};
    bool has_ddc_{true}, tuned_{false}, ddc_valid_{false};
};

/* The signal meter: one level per channel-statistics message (10 a second
 * from the stock audio images). */
class Meter {
   public:
    static constexpr int kHist = 200;      /* columns */
    static constexpr int kPerColumn = 2;   /* levels per column: 0.2 s, 40 s in all */
    static constexpr float kDecayDbS = 10; /* peak hold falls this fast */
    static constexpr int kFloorDb = -128;  /* history value "nothing" */
    void reset();                          /* min / max / peak / history */
    void feed(int32_t db, float dt_s);     /* returns nothing; see the getters */
    int32_t level() const { return level_; }
    float peak() const { return peak_; }
    int32_t min() const { return min_; }
    int32_t max() const { return max_; }
    bool any() const { return n_ > 0; }
    /* history column i (0 = oldest) */
    int8_t hist(int i) const { return hist_[(head_ + i) % kHist]; }
    uint32_t columns_done() const { return cols_; }

   private:
    int32_t level_{kFloorDb}, min_{0}, max_{kFloorDb};
    float peak_{kFloorDb};
    uint32_t n_{0}, cols_{0};
    int8_t hist_[kHist]{};
    int head_{0}, sub_{0};
    int32_t col_max_{kFloorDb};
};

/* Polarisation / fading hint (a heuristic): one level every 0.1 s with the
 * predicted elevation. It fires when, with the satellite at 15 deg or more,
 * the 1 s average falls smoothly (no 1 s step over 5 dB) by 6 dB or more
 * from its peak of the last 20 s, over at least 3 s, while it is still 3 dB
 * or more above the floor (a transmitter that stops drops to the floor in
 * one step: no hint). Shown 15 s, then quiet for 90 s. */
class FadeHint {
   public:
    void reset();
    void feed(int32_t db, float el_deg);
    bool showing() const { return show_ > 0; }
    uint32_t fired() const { return fired_; }

   private:
    static constexpr int kSec = 20;
    int8_t avg_[kSec]{}; /* 1 s averages, newest at (head_ - 1) */
    int head_{0}, n_{0};
    int32_t acc_{0};
    int sub_{0};
    int32_t floor_{0};
    int show_{0}, quiet_{0}; /* tenths of a second */
    uint32_t fired_{0};
    void second(float el_deg);
};

/* Residual tuning error (Hz) of the Doppler correction: the timing error is
 * the clock error plus the TLE's along-track error (about 1 km + 1.5 km per
 * day of age for LEO, as time at ~7 km/s) plus the position error (km, as
 * time at ~7 km/s); the frequency error is that time times the Doppler rate
 * f0 * |d(rr)/dt| / c. Host test: within this bound on an ISS pass. */
double tuning_error_hz(uint64_t f0, double rr_dot, double clock_err_s, double tle_age_days, double pos_err_km);

}  // namespace ui::external_app::gps

#endif /*__GPS_FOLLOW_H__*/
