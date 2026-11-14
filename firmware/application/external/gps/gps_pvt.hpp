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

#ifndef __GPS_PVT_H__
#define __GPS_PVT_H__

/* Position / velocity / time from tracked pseudoranges ("Autonomous"): the
 * tracking loops measure each satellite's transmit time at a common sample
 * (whole ms from the decoded TOW and bit / epoch counts, plus the code phase),
 * so the pseudoranges have no millisecond ambiguity. Least squares in x, y,
 * z, clock (4 unknowns, >= 4 satellites), satellite clocks (incl. relativity
 * and TGD), Sagnac, a simple troposphere (2.4 m / sin el, as the snapshot
 * solver); NO ionosphere model (a few m, up to ~10 m at low elevation and
 * high solar activity: part of the error estimate). With >= 5 satellites a
 * RAIM-style check drops the worst residual while the rms is too large.
 * Velocity and clock drift from the Dopplers (optional, 3 more orbit
 * evaluations per satellite). Runs in the app's low-priority solver thread. */

#include <cstdint>

#include "gps_orbit.hpp"

namespace ui::external_app::gps {

constexpr int kMaxPvt = 12;

/* Times: a double holding continuous GPS seconds (~1.5e9) resolves only 2^-22 s
 * (71 m of range), so transmit / receive times are kept relative to an
 * integer-second reference epoch PvtWork::t_ref. */
struct PvtMeas {
    const Orbit* orbit;
    double dt_tx;      /* satellite time of transmission at the measurement sample - t_ref, s */
    double doppler_hz; /* observed carrier Doppler (incl. the receiver clock) */
    float cn0;
    uint8_t prn;
};

/* why pvt_solve returned false (diagnostics log) */
enum PvtFail : uint8_t { PVT_OK = 0,
                         PVT_FEW,      /* fewer than 4 satellites */
                         PVT_CANCEL,   /* stopped from the UI */
                         PVT_LS,       /* least squares singular or not converging */
                         PVT_RAIM,     /* residuals still above the limit with >= 5 */
                         PVT_HEIGHT }; /* converged far from the earth's surface */

struct PvtSolution {
    Vec3 pos{};
    Vec3 vel{};
    double clock_m{0};   /* receiver clock bias, m (t_rx guess - true) * c */
    double drift_mps{0}; /* receiver clock drift, m/s (TCXO: / c = fractional frequency error) */
    double t_rx{0};      /* receive time, continuous GPS s (clock bias removed; 2^-22 s resolution) */
    double dt_rx{0};     /* the same relative to PvtWork::t_ref (full resolution) */
    double resid_rms_m{0};
    float hdop{0}, vdop{0}, pdop{0};
    uint8_t used{0};
    uint8_t dropped{0}; /* satellites removed by the RAIM check */
    uint8_t masked{0};  /* satellites below the elevation mask left out */
    float trop_m{0};    /* mean troposphere / ionosphere correction applied, m (diagnostics) */
    float iono_m{0};
    uint8_t checked{0}; /* 1: >= 5 satellites, residuals consistent */
    bool vel_ok{false};
    bool ok{false};
    uint8_t fail{PVT_OK};
    uint8_t iters{0}; /* least-squares iterations of the last solve */
};

struct PvtWork {
    double t_ref; /* continuous GPS s, an integer */
    PvtMeas in[kMaxPvt];
    Vec3 sat[kMaxPvt];   /* at transmission, ECEF of the transmit time */
    double clk[kMaxPvt]; /* satellite clock, s */
    double resid[kMaxPvt];
    double el[kMaxPvt];
    double az[kMaxPvt];
    /* corrections (set by the caller before pvt_solve; the defaults are what
     * the app uses): troposphere 0 none, 1 the old 2.4 m / sin(el), 2 standard
     * atmosphere zenith delay at the fix's height + mapping function;
     * ionosphere: the broadcast (Klobuchar) model when iono_ok; elevation mask
     * for the solution (applied while >= 5 satellites stay above it) */
    uint8_t tropo{2};
    uint8_t iono_ok{0};
    float mask_deg{5.0f};
    float ion_a[4]{}, ion_b[4]{};         /* alpha s, s/sc, s/sc^2, s/sc^3; beta s ... */
    double rx_lat{0}, rx_lon{0}, rx_h{0}; /* deg, deg, m: set with the elevations */
    double a[4][8];                       /* normal equations / inverse work (kept off the solver thread's stack) */
    double q[4][4];
    uint8_t use[kMaxPvt];
    volatile uint8_t cancel;
};

/* dt_rx_guess: receive time of the measurement - w.t_ref (s); any value
 * within ~0.1 s works (the clock absorbs it); prior: a position or {0,0,0}.
 * max_rms: RAIM residual limit (m). velocity: also solve v and the drift. */
bool pvt_solve(PvtWork& w, int n, const Vec3& prior, double dt_rx_guess, PvtSolution& out, double max_rms = 25.0, bool velocity = true);

/* Troposphere slant delay (m): standard atmosphere (1013.25 hPa, 15 C, 50 %
 * humidity at sea level, lapse rate 6.5 K/km) zenith delays after Saastamoinen
 * at height h, with the Black & Eisner mapping 1.001 / sqrt(0.002001 + sin^2 el)
 * (finite at the horizon). */
double tropo_delay_m(double el_deg, double lat_deg, double h_m);
/* Broadcast ionosphere (IS-GPS-200 20.3.3.5.2.5, Klobuchar) on L1, m. */
double iono_delay_m(const float a[4], const float b[4], double lat_deg, double lon_deg, double az_deg, double el_deg, double tow_s);

/* On-screen error estimate (horizontal, m): HDOP x 6 m (code noise, no
 * ionosphere model, simple troposphere); an estimate, not a measured figure. */
inline double pvt_err_m(float hdop) {
    return hdop * 6.0;
}

}  // namespace ui::external_app::gps

#endif /*__GPS_PVT_H__*/
