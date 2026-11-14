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

#include "gps_pvt.hpp"

#include <cmath>

namespace ui::external_app::gps {

namespace {

constexpr double kL1Hz = 1575.42e6;

/* 4x4 normal equations (a: 4 x 5 augmented in the first 5 columns), Gauss with pivoting */
bool solve4(double (*a)[8], double x[4]) {
    for (int c = 0; c < 4; c++) {
        int p = c;
        for (int r = c + 1; r < 4; r++)
            if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-12) return false;
        if (p != c)
            for (int k = 0; k < 5; k++) {
                const double t = a[c][k];
                a[c][k] = a[p][k];
                a[p][k] = t;
            }
        for (int r = c + 1; r < 4; r++) {
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < 5; k++) a[r][k] -= f * a[c][k];
        }
    }
    for (int c = 3; c >= 0; c--) {
        double s = a[c][4];
        for (int k = c + 1; k < 4; k++) s -= a[c][k] * x[k];
        x[c] = s / a[c][c];
    }
    return true;
}

/* inverse of a symmetric positive 4x4 held in a[.][0..3] (Gauss-Jordan, a: 4 x 8), into r; false if singular */
bool inv4(double (*a)[8], double r[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 4; j < 8; j++) a[i][j] = (j - 4 == i ? 1.0 : 0.0);
    for (int c = 0; c < 4; c++) {
        int p = c;
        for (int i = c + 1; i < 4; i++)
            if (std::fabs(a[i][c]) > std::fabs(a[p][c])) p = i;
        if (std::fabs(a[p][c]) < 1e-15) return false;
        if (p != c)
            for (int j = 0; j < 8; j++) {
                const double t = a[c][j];
                a[c][j] = a[p][j];
                a[p][j] = t;
            }
        const double d = a[c][c];
        for (int j = 0; j < 8; j++) a[c][j] /= d;
        for (int i = 0; i < 4; i++)
            if (i != c) {
                const double f = a[i][c];
                for (int j = 0; j < 8; j++) a[i][j] -= f * a[c][j];
            }
    }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) r[i][j] = a[i][j + 4];
    return true;
}

/* elevations (for the troposphere) once the position is near the earth:
 * one geodetic conversion per call, not per satellite */
__attribute__((noinline)) void elevations(PvtWork& w, int n, const Vec3& x, double t_rx, double b) {
    const Observer ob = make_observer(x);
    ecef_to_lla(x, w.rx_lat, w.rx_lon, w.rx_h);
    for (int i = 0; i < n; i++) {
        const Vec3 s = earth_rotate(w.sat[i], t_rx - w.in[i].dt_tx - b / kC);
        double az, el;
        az_el(ob, s, az, el);
        w.el[i] = el;
        w.az[i] = az;
    }
}

}  // namespace

double tropo_delay_m(double el_deg, double lat_deg, double h_m) {
    const double h = h_m < 0 ? 0 : (h_m > 9000 ? 9000 : h_m);
    const double p = 1013.25 * std::pow(1.0 - 2.2557e-5 * h, 5.2568); /* hPa */
    const double t = 288.15 - 6.5e-3 * h;                             /* K */
    const double e = 6.108 * 0.5 * std::exp((17.15 * t - 4684.0) / (t - 38.45));
    const double zhd = 0.0022768 * p / (1.0 - 0.00266 * std::cos(2.0 * lat_deg * kPi / 180.0) - 0.00028e-3 * h);
    const double zwd = 0.002277 * (1255.0 / t + 0.05) * e;
    const double se = std::sin((el_deg < 0 ? 0 : el_deg) * kPi / 180.0);
    return (zhd + zwd) * 1.001 / std::sqrt(0.002001 + se * se);
}

double iono_delay_m(const float a[4], const float b[4], double lat_deg, double lon_deg, double az_deg, double el_deg, double tow_s) {
    const double e = (el_deg < 0 ? 0 : el_deg) / 180.0; /* semicircles */
    const double az = az_deg * kPi / 180.0;
    const double psi = 0.0137 / (e + 0.11) - 0.022;
    double phi = lat_deg / 180.0 + psi * std::cos(az);
    if (phi > 0.416) phi = 0.416;
    if (phi < -0.416) phi = -0.416;
    const double lam = lon_deg / 180.0 + psi * std::sin(az) / std::cos(phi * kPi);
    const double phm = phi + 0.064 * std::cos((lam - 1.617) * kPi);
    double t = 4.32e4 * lam + tow_s;
    t = std::fmod(t, 86400.0);
    if (t < 0) t += 86400.0;
    const double f = 1.0 + 16.0 * std::pow(0.53 - e, 3);
    double amp = a[0] + phm * (a[1] + phm * (a[2] + phm * a[3]));
    double per = b[0] + phm * (b[1] + phm * (b[2] + phm * b[3]));
    if (amp < 0) amp = 0;
    if (per < 72000.0) per = 72000.0;
    const double x = 2.0 * kPi * (t - 50400.0) / per;
    const double tiono = std::fabs(x) < 1.57 ? f * (5e-9 + amp * (1.0 - x * x / 2.0 + x * x * x * x / 24.0)) : f * 5e-9;
    return tiono * kC;
}

namespace {

/* the atmosphere's delay on satellite i (m), as configured in w */
inline double atmo_m(const PvtWork& w, int i, double t_rx) {
    double d = 0;
    if (w.tropo == 1) d += 2.4 / std::sin((w.el[i] < 3 ? 3 : w.el[i]) * kPi / 180.0);
    if (w.tropo == 2) d += tropo_delay_m(w.el[i], w.rx_lat, w.rx_h);
    if (w.iono_ok) d += iono_delay_m(w.ion_a, w.ion_b, w.rx_lat, w.rx_lon, w.az[i], w.el[i], std::fmod(w.t_ref + t_rx, 604800.0));
    return d;
}

/* one least-squares solution with the satellites marked in w.use */
__attribute__((noinline)) bool ls(PvtWork& w, int n, Vec3& x, double& b, double t_rx, bool tropo, double* rms, int* used) {
    bool have_el = norm(x) > 6.0e6;
    if (have_el && tropo) elevations(w, n, x, t_rx, b);
    for (int it = 0; it < 12; it++) {
        double(*a)[8] = w.a;
        for (int p = 0; p < 4; p++)
            for (int q = 0; q < 5; q++) a[p][q] = 0;
        int m = 0;
        for (int i = 0; i < n; i++) {
            if (!w.use[i]) continue;
            if (w.cancel) return false;
            const double rho = kC * (t_rx - w.in[i].dt_tx);
            /* Sagnac: rotate by the travel time, not by the pseudorange (the guess of t_rx can
             * be ~10 ms off: 20 m along the earth's rotation) */
            const Vec3 s = earth_rotate(w.sat[i], (rho - b) / kC);
            const Vec3 d{s.x - x.x, s.y - x.y, s.z - x.z};
            const double r = norm(d);
            const double trop = (tropo && have_el) ? atmo_m(w, i, t_rx) : 0.0;
            const double res = rho + kC * w.clk[i] - trop - r - b;
            const double h[4] = {-d.x / r, -d.y / r, -d.z / r, 1.0};
            for (int p = 0; p < 4; p++) {
                for (int q = 0; q < 4; q++) a[p][q] += h[p] * h[q];
                a[p][4] += h[p] * res;
            }
            w.resid[i] = res;
            m++;
        }
        if (m < 4) return false;
        double dx[4];
        if (!solve4(a, dx)) return false;
        x.x += dx[0];
        x.y += dx[1];
        x.z += dx[2];
        b += dx[3];
        const double step = std::fabs(dx[0]) + std::fabs(dx[1]) + std::fabs(dx[2]);
        if (!have_el && norm(x) > 6.0e6 && step < 1.0e5) { /* close enough: elevations from here on */
            have_el = true;
            if (tropo) elevations(w, n, x, t_rx, b);
            continue;
        }
        if (have_el && step + std::fabs(dx[3]) < 1e-3) break;
    }
    /* residuals at the solution */
    double ss = 0;
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (!w.use[i]) continue;
        const double rho = kC * (t_rx - w.in[i].dt_tx);
        const Vec3 s = earth_rotate(w.sat[i], (rho - b) / kC);
        const Vec3 d{s.x - x.x, s.y - x.y, s.z - x.z};
        const double trop = tropo ? atmo_m(w, i, t_rx) : 0.0;
        w.resid[i] = rho + kC * w.clk[i] - trop - norm(d) - b;
        ss += w.resid[i] * w.resid[i];
        m++;
    }
    *rms = m > 4 ? std::sqrt(ss / (m - 4)) : 0.0; /* per degree of freedom */
    *used = m;
    return true;
}

}  // namespace

bool pvt_solve(PvtWork& w, int n, const Vec3& prior, double dt_rx_guess, PvtSolution& out, double max_rms, bool velocity) {
    const double t_rx_guess = dt_rx_guess; /* all times below are relative to w.t_ref */
    out = PvtSolution{};
    if (n > kMaxPvt) n = kMaxPvt;
    if (n < 4) {
        out.fail = PVT_FEW;
        return false;
    }
    /* satellite positions at transmission (the receive time does not enter) */
    for (int i = 0; i < n; i++) {
        if (w.cancel) {
            out.fail = PVT_CANCEL;
            return false;
        }
        const Orbit& o = *w.in[i].orbit;
        double clk = 0;
        sat_position(o, w.t_ref + w.in[i].dt_tx, &clk);
        const double tau = w.in[i].dt_tx - clk;
        w.sat[i] = sat_position(o, w.t_ref + tau, &clk);
        w.clk[i] = clk;
        w.use[i] = 1;
        w.el[i] = 90;
    }
    Vec3 x = prior;
    double b = 0;
    double rms = 0;
    int used = 0;
    if (!ls(w, n, x, b, t_rx_guess, true, &rms, &used)) {
        out.fail = w.cancel ? PVT_CANCEL : PVT_LS;
        return false;
    }
    /* elevation mask: leave out satellites below it while >= 5 stay above */
    uint8_t masked = 0;
    if (w.mask_deg > 0 && norm(x) > 6.0e6) {
        int above = 0;
        for (int i = 0; i < n; i++) above += w.use[i] && w.el[i] >= w.mask_deg;
        if (above >= 5 && above < used) {
            for (int i = 0; i < n; i++)
                if (w.use[i] && w.el[i] < w.mask_deg) {
                    w.use[i] = 0;
                    masked++;
                }
            if (!ls(w, n, x, b, t_rx_guess, true, &rms, &used)) {
                out.fail = w.cancel ? PVT_CANCEL : PVT_LS;
                return false;
            }
        }
    }
    out.masked = masked;
    /* RAIM: drop the worst while the residuals disagree (>= 5 to check) */
    uint8_t dropped = 0;
    while (used >= 5 && rms > max_rms) {
        int worst = -1;
        for (int i = 0; i < n; i++)
            if (w.use[i] && (worst < 0 || std::fabs(w.resid[i]) > std::fabs(w.resid[worst]))) worst = i;
        w.use[worst] = 0;
        dropped++;
        Vec3 x2 = x;
        double b2 = b;
        if (!ls(w, n, x2, b2, t_rx_guess, true, &rms, &used)) {
            out.fail = w.cancel ? PVT_CANCEL : PVT_LS;
            return false;
        }
        x = x2;
        b = b2;
    }
    out.resid_rms_m = rms;
    out.used = static_cast<uint8_t>(used);
    out.dropped = dropped;
    if (used >= 5 && rms > max_rms) {
        out.fail = PVT_RAIM;
        return false;
    }
    /* plausibility: near the earth's surface */
    double lat, lon, h;
    ecef_to_lla(x, lat, lon, h);
    if (h < -1000.0 || h > 20000.0) {
        out.fail = PVT_HEIGHT;
        return false;
    }
    out.pos = x;
    out.clock_m = b;
    out.dt_rx = t_rx_guess - b / kC;
    out.t_rx = w.t_ref + out.dt_rx;
    out.resid_rms_m = rms;
    out.used = static_cast<uint8_t>(used);
    out.dropped = dropped;
    out.checked = used >= 5 ? 1 : 0;
    { /* corrections applied (mean over the satellites used), for the log */
        double st = 0, si = 0;
        int k = 0;
        for (int i = 0; i < n; i++) {
            if (!w.use[i]) continue;
            if (w.tropo == 2) st += tropo_delay_m(w.el[i], w.rx_lat, w.rx_h);
            if (w.tropo == 1) st += 2.4 / std::sin((w.el[i] < 3 ? 3 : w.el[i]) * kPi / 180.0);
            if (w.iono_ok) si += iono_delay_m(w.ion_a, w.ion_b, w.rx_lat, w.rx_lon, w.az[i], w.el[i], std::fmod(out.t_rx, 604800.0));
            k++;
        }
        out.trop_m = k ? static_cast<float>(st / k) : 0.0f;
        out.iono_m = k ? static_cast<float>(si / k) : 0.0f;
    }
    /* DOP in east / north / up */
    {
        double(*hth)[8] = w.a;
        double(*q)[4] = w.q;
        for (int p = 0; p < 4; p++)
            for (int k = 0; k < 4; k++) hth[p][k] = 0;
        const Observer ob = make_observer(x);
        for (int i = 0; i < n; i++) {
            if (!w.use[i]) continue;
            const Vec3 d{w.sat[i].x - x.x, w.sat[i].y - x.y, w.sat[i].z - x.z};
            const double r = norm(d);
            /* local unit vector: east, north, up */
            const double ux = d.x / r, uy = d.y / r, uz = d.z / r;
            const double e = -ob.slo * ux + ob.clo * uy;
            const double nn = -ob.sla * ob.clo * ux - ob.sla * ob.slo * uy + ob.cla * uz;
            const double u = ob.cla * ob.clo * ux + ob.cla * ob.slo * uy + ob.sla * uz;
            const double hr[4] = {-e, -nn, -u, 1.0};
            for (int p = 0; p < 4; p++)
                for (int k = 0; k < 4; k++) hth[p][k] += hr[p] * hr[k];
        }
        if (inv4(hth, q)) {
            out.hdop = static_cast<float>(std::sqrt(q[0][0] + q[1][1]));
            out.vdop = static_cast<float>(std::sqrt(q[2][2]));
            out.pdop = static_cast<float>(std::sqrt(q[0][0] + q[1][1] + q[2][2]));
        }
    }
    out.ok = true;
    if (!velocity) return true;
    /* velocity and clock drift from the Dopplers: -lambda f = (vs - v).u + drift - c * clock rate of the satellite */
    {
        const double lambda = kC / kL1Hz;
        double(*a)[8] = w.a;
        for (int p = 0; p < 4; p++)
            for (int k = 0; k < 5; k++) a[p][k] = 0;
        int m = 0;
        for (int i = 0; i < n; i++) {
            if (!w.use[i]) continue;
            if (w.cancel) return true;
            const Orbit& o = *w.in[i].orbit;
            const double tau = w.t_ref + w.in[i].dt_tx - w.clk[i];
            const Vec3 vs = sat_velocity(o, tau);
            const double tt = out.dt_rx - w.in[i].dt_tx;
            const Vec3 s = earth_rotate(w.sat[i], tt);
            const Vec3 vsr = earth_rotate(vs, tt);
            const Vec3 d{s.x - x.x, s.y - x.y, s.z - x.z};
            const double r = norm(d);
            const double u[3] = {d.x / r, d.y / r, d.z / r};
            /* satellite clock drift: af1 + 2 af2 (t - toc) */
            const double sclk = o.af1 + 2.0 * o.af2 * (tau - o.toc);
            /* vs is the ECEF velocity (finite difference of ECEF positions); the Sagnac
             * term of the range rate is ~1e-5 m/s and left out */
            const double z = -lambda * w.in[i].doppler_hz + kC * sclk - (vsr.x * u[0] + vsr.y * u[1] + vsr.z * u[2]);
            const double h[4] = {-u[0], -u[1], -u[2], 1.0};
            for (int p = 0; p < 4; p++) {
                for (int k = 0; k < 4; k++) a[p][k] += h[p] * h[k];
                a[p][4] += h[p] * z;
            }
            m++;
        }
        double v[4];
        if (m >= 4 && solve4(a, v)) {
            out.vel = {v[0], v[1], v[2]};
            out.drift_mps = v[3];
            out.vel_ok = true;
        }
    }
    return true;
}

}  // namespace ui::external_app::gps
