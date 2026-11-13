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

#ifndef __GPS_SOLVE_H__
#define __GPS_SOLVE_H__

/* Snapshot ("coarse-time") positioning: from the sub-millisecond code phases
 * of a 20 ms snapshot, broadcast ephemerides, a rough time (the RTC, within
 * about 30 s) and a rough position (within ~100 km), solve x, y, z, the
 * receiver clock bias and the coarse-time error (5 unknowns, >= 5
 * satellites; >= 6 to check the result). Integer milliseconds come from the
 * a priori position (van Diggelen, "A-GPS", ch. 4; SnapperGPS, Beuchert et
 * al. 2021, uses the same coarse-time navigation with tolerances of
 * 100-150 km and up to a minute). Satellite clocks (incl. relativity, TGD)
 * and a simple troposphere (2.4 m / sin el) are modelled; no ionosphere.
 * A simple RAIM drops the worst satellite while the residuals are large.
 * Runs on the M0 in soft double: about 10 iterations x 6-10 satellites. */

#include <cstdint>

#include "gps_orbit.hpp"

namespace ui::external_app::gps {

struct Measurement {
    const Orbit* orbit;
    double code_phase; /* chips at the snapshot's first sample (= SV time mod 1 ms) */
    float metric;      /* acquisition strength, for RAIM's choice when the geometry fails */
};

struct Solution {
    Vec3 pos{};
    double clock_m{0};
    double time_corr_s{0}; /* true time - a priori time */
    double resid_rms_m{0};
    float hdop{0};
    uint8_t used{0};
    bool ok{false};
};

constexpr int kMaxMeas = 14;

/* Working arrays, kept off the M0 stack (the app puts this in the M4 RAM the
 * baseband lends; ~1 KB). */
struct SolveWork {
    Measurement in[kMaxMeas]; /* the caller's input list may live here too */
    double z[kMaxMeas];
    double pr[kMaxMeas];
    double resid[kMaxMeas];
    Measurement use[kMaxMeas];
    double a[5][6];
    double w[5][6];
    /* set from another thread to abandon a solve (checked per satellite row) */
    volatile uint8_t cancel;
};

/* One coarse-time least-squares solution with every measurement; residuals
 * in w.resid. */
bool coarse_time_solve(const Measurement* m, int n, const Vec3& prior, double t_prior_gps, Solution& out, SolveWork& w, bool tropo = true);

/* With RAIM: drop the worst residual (or, if the solution is implausible,
 * the weakest satellite) until rms < max_rms (a third of it with only 6
 * satellites) with >= 6 satellites. */
/* max_dt_s: largest accepted correction of the a priori time (the clock must be
 * that close; a larger one is a wrong solution). */
bool snapshot_fix(const Measurement* m, int n, const Vec3& prior, double t_prior_gps, Solution& out, SolveWork& w, double max_rms = 30.0, double max_dt_s = 60.0);

/* Coarse fixes with ALMANAC orbits (no ephemeris file). Numbers from
 * rf_test/gps/run_gps_alm_age.sh: almanacs 0.8 to 365 days old against a day of
 * broadcast ephemerides (2020-12-06 and 2026-11-12), 60 receivers x 12 epochs.
 * Older than kAlmMaxAgeDays the fix is refused (median error > 10 km, many
 * solutions fail). */
constexpr double kAlmMaxAgeDays = 30.0;
constexpr double kAlmMaxDtS = 30.0;
/* Typical line-of-sight range error (orbit + clock) of an almanac this old, m. */
double almanac_range_err_m(double age_days);
/* RAIM residual limit for snapshot_fix with almanac orbits, m (use it with
 * max_dt_s = kAlmMaxDtS: the clock must be within ~30 s anyway). */
double almanac_max_rms(double age_days);
/* On-screen error estimate (about 2 fixes in 3 within it), m. */
double almanac_err_m(double age_days, double hdop);

}  // namespace ui::external_app::gps

#endif /*__GPS_SOLVE_H__*/
