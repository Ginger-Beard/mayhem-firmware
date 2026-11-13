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

#ifndef __GPS_PASS_H__
#define __GPS_PASS_H__

/* Pass prediction for one satellite: rise, highest point, set. Driven in
 * small steps (step()) so the UI can spread the work over frames. The
 * satellite is a position callback (ECEF metres at a unix time), so GPS
 * almanac / ephemeris orbits and SGP4 element sets share the code. */

#include <cstdint>

#include "gps_orbit.hpp"
#include "gps_sgp4.hpp"

namespace ui::external_app::gps {

struct Pass {
    double rise{0}; /* unix s (UTC); == search start if already up */
    double top{0};
    double set{0}; /* 0 if it does not set within the search span */
    float max_el{-90};
    float rise_az{0};
    float set_az{0};
    bool up_at_start{false};
};

/* A satellite as the pass finder sees it. */
struct SatSource {
    enum Kind : uint8_t { NONE,
                          KEPLER,
                          SGP4 } kind{NONE};
    const Orbit* orbit{nullptr};
    Sgp4* sgp4{nullptr};
    /* ECEF (m) at unix time t; false if the propagator failed. */
    bool position(double unix_s, Vec3& out) const;
};

/* Elevation (deg) of a source seen from an observer at unix time t; -99 on error. */
double elevation(const SatSource& src, const Observer& obs, double unix_s, double* az = nullptr);

class PassFinder {
   public:
    /* Search [t0, t0 + span] with a coarse step (s): 30 for LEO, 120 for GPS. */
    void start(const SatSource& src, const Observer& obs, double t0, double span, double step);
    /* Advance up to n work units (one elevation each: a coarse step, one
     * bisection or golden-section iteration); true when the search is over.
     * A unit is one SGP4/Kepler evaluation, so a caller can bound the time of
     * a slice (an M0 frame) whatever the phase of the search. */
    bool step(int n);
    bool done() const { return done_; }
    bool found() const { return found_; }
    const Pass& pass() const { return pass_; }
    /* share of the span searched, 0..100 */
    int progress_pct() const { return end_ > t0_ ? static_cast<int>(100.0 * (t_ - t0_) / (end_ - t0_)) : 100; }

   private:
    SatSource src_{};
    Observer rx_{};
    double t0_{0}, end_{0}, dt_{0}, t_{0};
    double el_prev_{0};
    double best_t_{0}, best_el_{-99};
    bool up_{false}, done_{true}, found_{false};
    Pass pass_{};
    /* resumable sub-searches (each unit does one elevation) */
    enum Phase : uint8_t { SCAN,
                           BISECT,   /* horizon crossing between ba_ and bb_ */
                           CROSS_AZ, /* azimuth at the crossing */
                           TOP_C,    /* golden section: first two samples */
                           TOP_D,
                           TOP_ITER,
                           TOP_END };
    Phase phase_{SCAN};
    bool rising_{false};
    uint8_t k_{0};
    double ba_{0}, bb_{0};                 /* bisection bracket */
    double ga_{0}, gb_{0}, gc_{0}, gd_{0}; /* golden-section bracket and samples */
    double gfc_{0}, gfd_{0};
    void unit();
    void start_top();
};

/* The pass list of the Passes tab: the next pass of up to kMax TLE
 * satellites over the next 24 h, searched one satellite after another in
 * step()s, and the text of each line (30 columns). Host tested. */
class PassList {
   public:
    static constexpr size_t kMax = 4;
    static constexpr size_t kNameCols = 10;
    static constexpr double kSpan = 86400.0;
    static constexpr double kStep = 30.0;
    /* work: the propagator state used for the search (kept by the caller) */
    void start(const Tle* tles, size_t n, Sgp4* work, const Vec3& rx, double now);
    void stop() { busy_ = false; }
    /* up to `units` work units of the pass search (PassFinder::step; starting
     * the next satellite's propagator is one unit too); false when all done */
    bool step(int units);
    bool busy() const { return busy_; }
    size_t count() const { return n_; }
    bool found(size_t i) const { return i < n_ && ok_[i]; }
    const Pass& pass(size_t i) const { return res_[i]; }
    /* "ISS (ZARYA in 1h05 47d 9m", "ISS (ZARYA UP 23d 4m", "ISS (ZARYA search 35%" */
    void line(size_t i, double now, char* buf, size_t size) const;

   private:
    const Tle* tles_{nullptr};
    size_t n_{0};
    Sgp4* work_{nullptr};
    Observer obs_{};
    double t0_{0};
    PassFinder finder_{};
    Pass res_[kMax]{};
    bool ok_[kMax]{};
    bool done_[kMax]{};
    size_t idx_{0};
    bool busy_{false};
    bool start_current();
};

}  // namespace ui::external_app::gps

#endif /*__GPS_PASS_H__*/
