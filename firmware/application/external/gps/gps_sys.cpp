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

#include "gps_sys.hpp"

#include <cmath>

namespace ui::external_app::gps {

bool gnssalm_line(const char* s, size_t n, Orbit& o, uint8_t& sys) {
    /* fields: sys prn health sqrtA e i0 Omega0 OmegaDot omega M0 week toa af0 af1 [k] */
    const char* tok[15];
    size_t len[15];
    size_t nt = 0;
    for (size_t i = 0; i < n && nt < 15;) {
        while (i < n && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
        if (i >= n) break;
        tok[nt] = s + i;
        const size_t b = i;
        while (i < n && s[i] != ' ' && s[i] != '\t' && s[i] != '\r' && s[i] != '\n') i++;
        len[nt++] = i - b;
    }
    if (nt < 12 || len[0] != 1) return false;
    switch (tok[0][0]) {
        case 'E':
            sys = SYS_GAL;
            break;
        case 'C':
            sys = SYS_BDS;
            break;
        case 'R':
            sys = SYS_GLO;
            break;
        case 'S':
            sys = SYS_SBAS;
            break;
        default:
            return false; /* comments, GPS */
    }
    double v[11];
    for (size_t k = 1; k < 12; k++) v[k - 1] = parse_double(tok[k], len[k]);
    int32_t prn = static_cast<int32_t>(v[0]);
    if (sys == SYS_SBAS && prn < 100) prn += 100; /* RINEX numbers SBAS as PRN - 100 */
    if (prn < 1 || prn > 158 || v[2] < 4000.0 || v[2] > 7000.0 || v[3] < 0 || v[3] > 0.3 || v[9] < 1000 || v[10] < 0 || v[10] >= kWeekS)
        return false;
    o = Orbit{};
    o.prn = static_cast<uint8_t>(prn);
    o.health = v[1] != 0 ? 1 : 0;
    o.sqrt_a = v[2];
    o.e = v[3];
    o.i0 = v[4];
    o.omega0 = v[5];
    o.omega_dot = static_cast<float>(v[6]);
    o.omega = v[7];
    o.m0 = v[8];
    o.toe = static_cast<double>(static_cast<int32_t>(v[9])) * kWeekS + v[10];
    o.toc = o.toe;
    if (nt >= 14) { /* the satellite clock at toa (the system's own time) */
        o.af0 = static_cast<float>(parse_double(tok[12], len[12]));
        o.af1 = static_cast<float>(parse_double(tok[13], len[13]));
    }
    o.ephemeris = 0;
    o.valid = 1;
    return true;
}

GalAlm gal_alm(const Orbit& o) {
    GalAlm g{};
    g.sqrt_a = static_cast<float>(o.sqrt_a);
    g.e = static_cast<float>(o.e);
    g.i0 = static_cast<float>(o.i0);
    g.omega0 = static_cast<float>(o.omega0);
    g.omega = static_cast<float>(o.omega);
    g.m0 = static_cast<float>(o.m0);
    g.omega_dot = o.omega_dot;
    g.af0 = o.af0;
    g.af1 = o.af1;
    g.toa = static_cast<uint32_t>(o.toe);
    g.prn = o.prn;
    return g;
}

Orbit gal_orbit(const GalAlm& g) {
    Orbit o{};
    o.sqrt_a = g.sqrt_a;
    o.e = g.e;
    o.i0 = g.i0;
    o.omega0 = g.omega0;
    o.omega = g.omega;
    o.m0 = g.m0;
    o.omega_dot = g.omega_dot;
    o.af0 = g.af0;
    o.af1 = g.af1;
    o.toe = o.toc = static_cast<double>(g.toa);
    o.prn = g.prn;
    o.valid = 1;
    return o;
}

void gal_predict(const Orbit& o, const Vec3& rx, double t_rx, double* code_chips, double* doppler_geo_hz) {
    double t_tx = 0, clk = 0;
    geometric_range(o, rx, t_rx, &t_tx);
    sat_position(o, t_tx, &clk);
    /* a code period starts at each 4 ms of the satellite's clock (GST: the GPS
     * seconds to a few ns) */
    const double sv = (t_tx + clk) / 0.004;
    *code_chips = (sv - std::floor(sv)) * 4092.0;
    const double r1 = geometric_range(o, rx, t_rx + 0.5), r0 = geometric_range(o, rx, t_rx - 0.5);
    *doppler_geo_hz = -(r1 - r0) * (1575.42e6 / kC);
}

}  // namespace ui::external_app::gps
