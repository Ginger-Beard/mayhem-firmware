/*
 * Near-earth SGP4 satellite propagation (TLE elements).
 *
 * Port of the near-earth part of the SGP4 code by David Vallado et al.
 * ("Revisiting Spacetrack Report #3", AIAA 2006-6753), following the Python
 * port "sgp4" by Brandon Rhodes, which carries this notice:
 *
 *   The MIT License (MIT)
 *   Copyright (c) 2012-2016 Brandon Rhodes
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a
 *   copy of this software and associated documentation files (the
 *   "Software"), to deal in the Software without restriction, including
 *   without limitation the rights to use, copy, modify, merge, publish,
 *   distribute, sublicense, and/or sell copies of the Software, and to permit
 *   persons to whom the Software is furnished to do so, subject to the
 *   following conditions:
 *
 *   The above copyright notice and this permission notice shall be included
 *   in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Changes for the PortaPack (2026, PortaPack Mayhem contributors, same terms):
 * C++ without heap, WGS-72 constants, "improved" operation mode only, the
 * deep-space (SDP4, period >= 225 min) branch left out: such element sets are
 * refused (error 7). Checked against the published test vectors of the
 * near-earth satellites in SGP4-VER.TLE (host test rf_test/run_gps_orbit_test.sh).
 */

#ifndef __GPS_SGP4_H__
#define __GPS_SGP4_H__

#include <cstddef>
#include <cstdint>

#include "gps_orbit.hpp"

namespace ui::external_app::gps {

/* Two-line elements, as parsed (units converted for SGP4). */
struct Tle {
    char name[17]{};
    uint32_t catnr{0};
    double jd_epoch{0}; /* Julian date (UTC) of the epoch */
    double bstar{0};
    double ndot{0};  /* rad/min^2 */
    double nddot{0}; /* rad/min^3 */
    double ecco{0};
    double argpo{0}; /* rad */
    double inclo{0};
    double mo{0};
    double no_kozai{0}; /* rad/min */
    double nodeo{0};
    bool valid{false};
};

/* Parse the two element lines (line 0 / name is separate). Checks the line
 * numbers and both checksums. */
bool parse_tle(const char* line1, const char* line2, Tle& out);

/* Propagator state after init (near-earth only). */
struct Sgp4 {
    double bstar, ecco, argpo, inclo, mo, nodeo, no_unkozai;
    double aycof, con41, cc1, cc4, cc5, d2, d3, d4, delmo, eta, argpdot, omgcof;
    double sinmao, t2cof, t3cof, t4cof, t5cof, x1mth2, x7thm1, mdot, nodedot;
    double xlcof, xmcof, nodecf, jd_epoch;
    int isimp;
    int error; /* 0 ok, 1-6 as Vallado, 7 deep space (not supported) */
};

bool sgp4_init(const Tle& tle, Sgp4& s);
/* Position (km) and velocity (km/s) in the TEME frame, tsince minutes after
 * the epoch. false on an error (s.error set). */
bool sgp4_propagate(Sgp4& s, double tsince_min, double r[3], double v[3]);

/* Greenwich mean sidereal time (rad) for a UT1 Julian date. */
double gmst(double jd_ut1);
/* TEME position (km) at Julian date jd -> ECEF (m) (GMST rotation only;
 * polar motion and the equation of the equinoxes are below a few hundred
 * metres at LEO, irrelevant for pass prediction). */
Vec3 teme_to_ecef(const double r_km[3], double jd);

/* Julian date of a unix time (UTC) and back. */
inline double unix_to_jd(double unix_s) {
    return unix_s / 86400.0 + 2440587.5;
}
inline double jd_to_unix(double jd) {
    return (jd - 2440587.5) * 86400.0;
}

}  // namespace ui::external_app::gps

#endif /*__GPS_SGP4_H__*/
