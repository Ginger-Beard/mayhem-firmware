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

/* Elevation (deg) of a source seen from rx at unix time t; -99 on error. */
double elevation(const SatSource& src, const Vec3& rx, double unix_s, double* az = nullptr);

class PassFinder {
   public:
    /* Search [t0, t0 + span] with a coarse step (s): 30 for LEO, 120 for GPS. */
    void start(const SatSource& src, const Vec3& rx, double t0, double span, double step);
    /* Advance up to n coarse steps; true when the search is over. */
    bool step(int n);
    bool done() const { return done_; }
    bool found() const { return found_; }
    const Pass& pass() const { return pass_; }

   private:
    SatSource src_{};
    Vec3 rx_{};
    double t0_{0}, end_{0}, dt_{0}, t_{0};
    double el_prev_{0};
    double best_t_{0}, best_el_{-99};
    bool up_{false}, done_{true}, found_{false};
    Pass pass_{};
    double crossing(double a, double b, bool rising);
    void refine_top();
};

}  // namespace ui::external_app::gps

#endif /*__GPS_PASS_H__*/
