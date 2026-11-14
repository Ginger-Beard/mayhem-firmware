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

/* n x n normal equations (n <= 5; right side in column 5), Gauss with pivoting */
bool solve_n(double (*a)[6], int n, double* x) {
    for (int c = 0; c < n; c++) {
        int p = c;
        for (int r = c + 1; r < n; r++)
            if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-12) return false;
        if (p != c)
            for (int k = 0; k < 6; k++) {
                const double t = a[c][k];
                a[c][k] = a[p][k];
                a[p][k] = t;
            }
        for (int r = c + 1; r < n; r++) {
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < 6; k++) a[r][k] -= f * a[c][k];
        }
    }
    for (int c = n - 1; c >= 0; c--) {
        double s = a[c][5];
        for (int k = c + 1; k < n; k++) s -= a[c][k] * x[k];
        x[c] = s / a[c][c];
    }
    return true;
}

/* in-place inverse of a symmetric positive n x n (n <= 5) held in a[.][0..n-1]
 * (Gauss-Jordan; no pivoting needed for a positive definite matrix) */
bool inv_n(double (*a)[6], int n) {
    for (int k = 0; k < n; k++) {
        const double piv = a[k][k];
        if (std::fabs(piv) < 1e-15) return false;
        a[k][k] = 1.0;
        for (int j = 0; j < n; j++) a[k][j] /= piv;
        for (int i = 0; i < n; i++) {
            if (i == k) continue;
            const double f = a[i][k];
            a[i][k] = 0.0;
            for (int j = 0; j < n; j++) a[i][j] -= f * a[k][j];
        }
    }
    return true;
}

double atmo_m(const PvtWork& w, int i, double t_rx);

/* elevations (for the troposphere) once the position is near the earth:
 * one geodetic conversion per call, not per satellite; the atmosphere's
 * delay at them too (it changes only with them) */
__attribute__((noinline)) void elevations(PvtWork& w, int n, const Vec3& x, double t_rx, double b) {
    const Observer ob = make_observer(x);
    ecef_to_lla(x, w.rx_lat, w.rx_lon, w.rx_h);
    for (int i = 0; i < n; i++) {
        if (!w.use[i]) continue;
        const Vec3 s = earth_rotate(w.sat[i], t_rx - w.in[i].dt_tx - b / kC + w.clk[i]);
        double az, el;
        az_el(ob, s, az, el);
        w.el[i] = el;
        w.az[i] = az;
        w.atm[i] = atmo_m(w, i, t_rx);
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
double atmo_m(const PvtWork& w, int i, double t_rx) {
    double d = 0;
    if (w.tropo == 1) d += 2.4 / std::sin((w.el[i] < 3 ? 3 : w.el[i]) * kPi / 180.0);
    if (w.tropo == 2) d += tropo_delay_m(w.el[i], w.rx_lat, w.rx_h);
    if (w.iono_ok) d += iono_delay_m(w.ion_a, w.ion_b, w.rx_lat, w.rx_lon, w.az[i], w.el[i], std::fmod(w.t_ref + t_rx, 604800.0));
    return d;
}

/* unknowns of a solution with the satellites marked in w.use: 4, or 5 with
 * the Galileo - GPS clock offset (w.isb and a Galileo satellite used) */
int unknowns(const PvtWork& w, int n) {
    if (!w.isb) return 4;
    for (int i = 0; i < n; i++)
        if (w.use[i] && w.in[i].sys) return 5;
    return 4;
}

/* one least-squares solution with the satellites marked in w.use; g: the
 * Galileo - GPS clock offset (m), solved when unknowns() says 5 */
__attribute__((noinline)) bool ls(PvtWork& w, int n, Vec3& x, double& b, double& g, double t_rx, bool tropo, double* rms, int* used) {
    bool have_el = norm(x) > 6.0e6;
    if (have_el && tropo) elevations(w, n, x, t_rx, b);
    const int k = unknowns(w, n);
    if (k == 4) g = 0;
    for (int it = 0; it < 12; it++) {
        double(*a)[6] = w.a;
        for (int p = 0; p < 5; p++)
            for (int q = 0; q < 6; q++) a[p][q] = 0;
        int m = 0;
        for (int i = 0; i < n; i++) {
            if (!w.use[i]) continue;
            if (w.cancel) return false;
            const double rho = kC * (t_rx - w.in[i].dt_tx);
            /* Sagnac: rotate by the travel time, not by the pseudorange (the guess of t_rx can
             * be ~10 ms off: 20 m along the earth's rotation; the satellite clock, up to ~1 ms,
             * is not travel time either: up to ~1 m) */
            const Vec3 s = earth_rotate(w.sat[i], (rho - b) / kC + w.clk[i]);
            const Vec3 d{s.x - x.x, s.y - x.y, s.z - x.z};
            const double r = norm(d);
            const double trop = (tropo && have_el) ? w.atm[i] : 0.0;
            const double gs = w.in[i].sys ? g : 0.0;
            const double res = rho + kC * w.clk[i] - trop - r - b - gs;
            const double h[5] = {-d.x / r, -d.y / r, -d.z / r, 1.0, w.in[i].sys ? 1.0 : 0.0};
            for (int p = 0; p < k; p++) {
                for (int q = 0; q < k; q++) a[p][q] += h[p] * h[q];
                a[p][5] += h[p] * res;
            }
            w.resid[i] = res;
            m++;
        }
        if (m < k) return false;
        double dx[5];
        if (!solve_n(a, k, dx)) return false;
        x.x += dx[0];
        x.y += dx[1];
        x.z += dx[2];
        b += dx[3];
        if (k == 5) g += dx[4];
        const double step = std::fabs(dx[0]) + std::fabs(dx[1]) + std::fabs(dx[2]);
        if (!have_el && norm(x) > 6.0e6 && step < 1.0e5) { /* close enough: elevations from here on */
            have_el = true;
            if (tropo) elevations(w, n, x, t_rx, b);
            continue;
        }
        if (have_el && step + std::fabs(dx[3]) + (k == 5 ? std::fabs(dx[4]) : 0.0) < 1e-3) break;
    }
    /* residuals at the solution */
    double ss = 0;
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (!w.use[i]) continue;
        const double rho = kC * (t_rx - w.in[i].dt_tx);
        const Vec3 s = earth_rotate(w.sat[i], (rho - b) / kC + w.clk[i]);
        const Vec3 d{s.x - x.x, s.y - x.y, s.z - x.z};
        const double trop = tropo ? (have_el ? w.atm[i] : atmo_m(w, i, t_rx)) : 0.0;
        w.resid[i] = rho + kC * w.clk[i] - trop - norm(d) - b - (w.in[i].sys ? g : 0.0);
        ss += w.resid[i] * w.resid[i];
        m++;
    }
    *rms = m > k ? std::sqrt(ss / (m - k)) : 0.0; /* per degree of freedom */
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
        w.el[i] = 90;
        w.use[i] = (w.sys_mask >> w.in[i].sys) & 1u;
        if (!w.use[i] || ((w.sats_done >> w.in[i].sys) & 1u)) continue;
        const Orbit& o = *w.in[i].orbit;
        double clk = 0;
        sat_position(o, w.t_ref + w.in[i].dt_tx, &clk);
        const double tau = w.in[i].dt_tx - clk;
        w.sat[i] = sat_position(o, w.t_ref + tau, &clk);
        w.clk[i] = clk;
    }
    {
        int m = 0;
        for (int i = 0; i < n; i++) m += w.use[i];
        if (m < 4) {
            out.fail = PVT_FEW;
            return false;
        }
    }
    Vec3 x = prior;
    double b = 0, g = 0;
    double rms = 0;
    int used = 0;
    if (!ls(w, n, x, b, g, t_rx_guess, true, &rms, &used)) {
        out.fail = w.cancel ? PVT_CANCEL : PVT_LS;
        return false;
    }
    /* elevation mask: leave out satellites below it while >= 5 stay above */
    uint8_t masked = 0;
    if (w.mask_deg > 0 && norm(x) > 6.0e6) {
        int above = 0;
        for (int i = 0; i < n; i++) above += w.use[i] && w.el[i] >= w.mask_deg;
        if (above >= unknowns(w, n) + 1 && above < used) {
            for (int i = 0; i < n; i++)
                if (w.use[i] && w.el[i] < w.mask_deg) {
                    w.use[i] = 0;
                    masked++;
                }
            if (!ls(w, n, x, b, g, t_rx_guess, true, &rms, &used)) {
                out.fail = w.cancel ? PVT_CANCEL : PVT_LS;
                return false;
            }
        }
    }
    out.masked = masked;
    /* RAIM: drop the worst while the residuals disagree (one more satellite
     * than unknowns to check) */
    uint8_t dropped = 0;
    while (used > unknowns(w, n) && rms > max_rms) {
        int worst = -1;
        for (int i = 0; i < n; i++)
            if (w.use[i] && (worst < 0 || std::fabs(w.resid[i]) > std::fabs(w.resid[worst]))) worst = i;
        w.use[worst] = 0;
        dropped++;
        Vec3 x2 = x;
        double b2 = b, g2 = g;
        if (!ls(w, n, x2, b2, g2, t_rx_guess, true, &rms, &used)) {
            out.fail = w.cancel ? PVT_CANCEL : PVT_LS;
            return false;
        }
        x = x2;
        b = b2;
        g = g2;
    }
    const int k_used = unknowns(w, n);
    out.resid_rms_m = rms;
    out.used = static_cast<uint8_t>(used);
    out.dropped = dropped;
    if (used > k_used && rms > max_rms) {
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
    out.checked = used > k_used ? 1 : 0;
    for (int i = 0; i < n; i++) out.used_gal = static_cast<uint8_t>(out.used_gal + (w.use[i] && w.in[i].sys));
    {
        const double gd = g * 10.0;
        out.isb_dm = static_cast<int16_t>(gd > 32767 ? 32767 : (gd < -32767 ? -32767 : std::lround(gd)));
    }
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
    /* DOP in east / north / up (with the Galileo - GPS offset as an unknown when solved) */
    {
        double(*q)[6] = w.a;
        for (int p = 0; p < 5; p++)
            for (int k = 0; k < 6; k++) q[p][k] = 0;
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
            const double hr[5] = {-e, -nn, -u, 1.0, w.in[i].sys ? 1.0 : 0.0};
            for (int p = 0; p < k_used; p++)
                for (int k = 0; k < k_used; k++) q[p][k] += hr[p] * hr[k];
        }
        if (inv_n(q, k_used)) {
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
        double(*a)[6] = w.a;
        for (int p = 0; p < 4; p++)
            for (int k = 0; k < 6; k++) a[p][k] = 0;
        int m = 0;
        for (int i = 0; i < n; i++) {
            if (!w.use[i]) continue;
            if (w.cancel) return true;
            const Orbit& o = *w.in[i].orbit;
            const double tau = w.t_ref + w.in[i].dt_tx - w.clk[i];
            const Vec3 vs = sat_velocity(o, tau);
            const double tt = out.dt_rx - w.in[i].dt_tx + w.clk[i];
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
                a[p][5] += h[p] * z;
            }
            m++;
        }
        double v[4];
        if (m >= 4 && solve_n(a, 4, v)) {
            out.vel = {v[0], v[1], v[2]};
            out.drift_mps = v[3];
            out.vel_ok = true;
        }
    }
    return true;
}

}  // namespace ui::external_app::gps
