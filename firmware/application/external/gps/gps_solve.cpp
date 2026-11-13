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

#include "gps_solve.hpp"

#include <cmath>

namespace ui::external_app::gps {

namespace {

constexpr double kLms = kC * 1e-3; /* metres per code period */

double wrap_ms(double x) {
    double r = std::fmod(x + kLms / 2, kLms);
    if (r < 0) r += kLms;
    return r - kLms / 2;
}

/* Satellite clock and troposphere: what to add to the geometric range. */
double extra(const Orbit& o, const Vec3& rx, double t_tx, const Vec3& sat, bool tropo) {
    double clk = 0;
    sat_position(o, t_tx, &clk);
    double d = -kC * clk;
    if (tropo) {
        double az, el;
        az_el(rx, sat, az, el);
        if (el < 3) el = 3;
        d += 2.4 / std::sin(el * kPi / 180.0);
    }
    return d;
}

/* Solve the 5x5 normal equations (A^T A) x = A^T r by Gaussian elimination
 * with partial pivoting; false if singular. */
bool solve5(double a[5][6], double x[5]) {
    for (int c = 0; c < 5; c++) {
        int p = c;
        for (int r = c + 1; r < 5; r++)
            if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-12) return false;
        if (p != c)
            for (int k = 0; k < 6; k++) {
                const double t = a[c][k];
                a[c][k] = a[p][k];
                a[p][k] = t;
            }
        for (int r = c + 1; r < 5; r++) {
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < 6; k++) a[r][k] -= f * a[c][k];
        }
    }
    for (int c = 4; c >= 0; c--) {
        double s = a[c][5];
        for (int k = c + 1; k < 5; k++) s -= a[c][k] * x[k];
        x[c] = s / a[c][c];
    }
    return true;
}

}  // namespace

bool coarse_time_solve(const Measurement* m, int n, const Vec3& prior, double t_prior, Solution& out, SolveWork& w, bool tropo) {
    out = Solution{};
    if (n < 5 || n > kMaxMeas) return false;
    double* z = w.z;
    double* pr = w.pr;
    double* resid = w.resid;
    Vec3 x = prior;
    double t = t_prior, b = 0;
    /* pseudorange mod 1 ms from the code phase (SV time mod 1 ms) */
    for (int i = 0; i < n; i++) {
        double v = std::fmod(-kC * m[i].code_phase / 1.023e6, kLms);
        if (v < 0) v += kLms;
        z[i] = v;
    }
    /* integer milliseconds from the a priori position and time */
    for (int i = 0; i < n; i++) {
        double t_tx;
        Vec3 s;
        const double rho = geometric_range(*m[i].orbit, x, t, &t_tx, &s);
        pr[i] = rho + extra(*m[i].orbit, x, t_tx, s, tropo);
    }
    b = wrap_ms(z[0] - pr[0]);
    for (int i = 0; i < n; i++) {
        const double nn = std::floor((pr[i] + b - z[i]) / kLms + 0.5);
        pr[i] = z[i] + nn * kLms;
    }
    double rms = 0;
    for (int it = 0; it < 12; it++) {
        double(&a)[5][6] = w.a;
        for (auto& row : a)
            for (auto& v : row) v = 0;
        double ss = 0;
        for (int i = 0; i < n; i++) {
            if (w.cancel) return false;
            double t_tx;
            Vec3 s;
            const double rho = geometric_range(*m[i].orbit, x, t, &t_tx, &s);
            const Vec3 u{(s.x - x.x) / rho, (s.y - x.y) / rho, (s.z - x.z) / rho};
            const Vec3 v = sat_velocity(*m[i].orbit, t_tx);
            const double rate = v.x * u.x + v.y * u.y + v.z * u.z;
            const double h[5] = {-u.x, -u.y, -u.z, 1.0, rate};
            const double r = pr[i] - (rho + b + extra(*m[i].orbit, x, t_tx, s, tropo));
            resid[i] = r;
            ss += r * r;
            for (int p = 0; p < 5; p++) {
                for (int q = 0; q < 5; q++) a[p][q] += h[p] * h[q];
                a[p][5] += h[p] * r;
            }
        }
        rms = std::sqrt(ss / n);
        double d[5];
        if (!solve5(a, d)) return false;
        x.x += d[0];
        x.y += d[1];
        x.z += d[2];
        b += d[3];
        t += d[4];
        if (std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) < 1e-3 && std::fabs(d[4]) < 1e-6) break;
    }
    /* HDOP at the solution (4-unknown geometry, local east/north) */
    {
        double lat, lon, h;
        ecef_to_lla(x, lat, lon, h);
        const double la = lat * kPi / 180, lo = lon * kPi / 180;
        const Vec3 e{-std::sin(lo), std::cos(lo), 0};
        const Vec3 nn{-std::sin(la) * std::cos(lo), -std::sin(la) * std::sin(lo), std::cos(la)};
        const Vec3 up{std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
        double(&a)[5][6] = w.a;
        for (auto& row : a)
            for (auto& v : row) v = 0;
        for (int i = 0; i < n; i++) {
            Vec3 s;
            const double rho = geometric_range(*m[i].orbit, x, t, nullptr, &s);
            const Vec3 u{(s.x - x.x) / rho, (s.y - x.y) / rho, (s.z - x.z) / rho};
            const double hh[4] = {-(u.x * e.x + u.y * e.y + u.z * e.z), -(u.x * nn.x + u.y * nn.y + u.z * nn.z),
                                  -(u.x * up.x + u.y * up.y + u.z * up.z), 1.0};
            for (int p = 0; p < 4; p++)
                for (int q = 0; q < 4; q++) a[p][q] += hh[p] * hh[q];
        }
        /* invert the 4x4 by solving for the first two unit vectors (5x5 solver with a dummy 5th) */
        double q00 = 0, q11 = 0;
        for (int col = 0; col < 2; col++) {
            double(&ww)[5][6] = w.w;
            for (auto& row : ww)
                for (auto& v : row) v = 0;
            for (int p = 0; p < 4; p++) {
                for (int q = 0; q < 4; q++) ww[p][q] = a[p][q];
                ww[p][5] = p == col ? 1.0 : 0.0;
            }
            ww[4][4] = 1.0;
            double sol[5];
            if (solve5(ww, sol)) (col ? q11 : q00) = sol[col];
        }
        out.hdop = static_cast<float>(std::sqrt(q00 + q11 > 0 ? q00 + q11 : 0));
    }
    out.pos = x;
    out.clock_m = b;
    out.time_corr_s = t - t_prior;
    out.resid_rms_m = rms;
    out.used = static_cast<uint8_t>(n);
    out.ok = true;
    return true;
}

bool snapshot_fix(const Measurement* m, int n, const Vec3& prior, double t_prior, Solution& out, SolveWork& w, double max_rms, double max_dt_s) {
    Measurement* use = w.use;
    if (n > kMaxMeas) n = kMaxMeas;
    for (int i = 0; i < n; i++) use[i] = m[i];
    out = Solution{};
    while (n >= 6 && !w.cancel) {
        Solution s;
        const bool ok = coarse_time_solve(use, n, prior, t_prior, s, w);
        const Vec3 d{s.pos.x - prior.x, s.pos.y - prior.y, s.pos.z - prior.z};
        const bool plausible = ok && norm(d) < 150e3 && std::fabs(s.time_corr_s) < max_dt_s;
        /* with 6 satellites only one residual degree of freedom is left: be stricter */
        const double limit = n >= 7 ? max_rms : max_rms / 3.0;
        if (plausible && s.resid_rms_m < limit) {
            out = s;
            return true;
        }
        int drop = 0;
        for (int i = 1; i < n; i++) {
            if (plausible ? std::fabs(w.resid[i]) > std::fabs(w.resid[drop]) : use[i].metric < use[drop].metric) drop = i;
        }
        for (int i = drop; i + 1 < n; i++) use[i] = use[i + 1];
        n--;
    }
    return false;
}

double almanac_range_err_m(double age_days) {
    const double a = age_days < 0 ? 0 : age_days;
    return 450.0 + 60.0 * a + 2.5 * a * a;
}

double almanac_max_rms(double age_days) {
    return 2.5 * almanac_range_err_m(age_days);
}

double almanac_err_m(double age_days, double hdop) {
    /* horizontal error / HDOP at the 68th percentile, by age (7 heard satellites) */
    static constexpr double kAge[] = {0, 1, 7, 14, 30};
    static constexpr double kK[] = {400, 450, 1800, 5500, 25000};
    const double a = age_days < 0 ? 0 : age_days;
    double k = kK[4];
    for (int i = 0; i < 4; i++)
        if (a <= kAge[i + 1]) {
            k = kK[i] + (kK[i + 1] - kK[i]) * (a - kAge[i]) / (kAge[i + 1] - kAge[i]);
            break;
        }
    return k * (hdop > 0.5 ? hdop : 0.5);
}

}  // namespace ui::external_app::gps
