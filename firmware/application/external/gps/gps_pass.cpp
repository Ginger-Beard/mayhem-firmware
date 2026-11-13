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

#include "gps_pass.hpp"

namespace ui::external_app::gps {

bool SatSource::position(double unix_s, Vec3& out) const {
    if (kind == KEPLER && orbit) {
        out = sat_position(*orbit, unix_to_gps(unix_s));
        return true;
    }
    if (kind == SGP4 && sgp4) {
        double r[3], v[3];
        const double jd = unix_to_jd(unix_s);
        if (!sgp4_propagate(*sgp4, (jd - sgp4->jd_epoch) * 1440.0, r, v)) return false;
        out = teme_to_ecef(r, jd);
        return true;
    }
    return false;
}

double elevation(const SatSource& src, const Vec3& rx, double unix_s, double* az) {
    Vec3 p;
    if (!src.position(unix_s, p)) return -99.0;
    double a, e;
    az_el(rx, p, a, e);
    if (az) *az = a;
    return e;
}

void PassFinder::start(const SatSource& src, const Vec3& rx, double t0, double span, double step) {
    src_ = src;
    rx_ = rx;
    t0_ = t0;
    end_ = t0 + span;
    dt_ = step;
    t_ = t0;
    pass_ = Pass{};
    found_ = false;
    done_ = false;
    best_el_ = -99;
    double az = 0;
    el_prev_ = elevation(src_, rx_, t_, &az);
    if (el_prev_ <= -98.0) {
        done_ = true;
        return;
    }
    up_ = el_prev_ > 0.0;
    if (up_) {
        pass_.rise = t0;
        pass_.rise_az = static_cast<float>(az);
        pass_.up_at_start = true;
        best_t_ = t0;
        best_el_ = el_prev_;
    }
}

/* Bisection of the horizon crossing between a (below/above) and b. */
double PassFinder::crossing(double a, double b, bool rising) {
    for (int k = 0; k < 9; k++) {
        const double m = 0.5 * (a + b);
        const bool above = elevation(src_, rx_, m) > 0.0;
        if (above == rising)
            b = m;
        else
            a = m;
    }
    return 0.5 * (a + b);
}

/* Golden-section search for the highest point around the best coarse sample. */
void PassFinder::refine_top() {
    double a = best_t_ - dt_, b = best_t_ + dt_;
    if (a < t0_) a = t0_;
    constexpr double g = 0.6180339887;
    double c = b - g * (b - a), d = a + g * (b - a);
    double fc = elevation(src_, rx_, c), fd = elevation(src_, rx_, d);
    for (int k = 0; k < 10; k++) {
        if (fc > fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - g * (b - a);
            fc = elevation(src_, rx_, c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + g * (b - a);
            fd = elevation(src_, rx_, d);
        }
    }
    const double t = 0.5 * (a + b);
    const double e = elevation(src_, rx_, t);
    if (e > best_el_) {
        best_el_ = e;
        best_t_ = t;
    }
    pass_.top = best_t_;
    pass_.max_el = static_cast<float>(best_el_);
}

bool PassFinder::step(int n) {
    while (!done_ && n-- > 0) {
        const double t = t_ + dt_;
        if (t > end_) {
            if (up_) { /* still up at the end of the span */
                refine_top();
                found_ = true;
            }
            done_ = true;
            break;
        }
        double az = 0;
        const double el = elevation(src_, rx_, t, &az);
        if (el <= -98.0) {
            done_ = true;
            break;
        }
        if (!up_ && el > 0.0) {
            up_ = true;
            pass_.rise = crossing(t_, t, true);
            elevation(src_, rx_, pass_.rise, &az);
            pass_.rise_az = static_cast<float>(az);
            best_t_ = t;
            best_el_ = el;
        } else if (up_ && el <= 0.0) {
            pass_.set = crossing(t_, t, false);
            elevation(src_, rx_, pass_.set, &az);
            pass_.set_az = static_cast<float>(az);
            refine_top();
            found_ = true;
            done_ = true;
        } else if (up_ && el > best_el_) {
            best_el_ = el;
            best_t_ = t;
        }
        el_prev_ = el;
        t_ = t;
    }
    return done_;
}

}  // namespace ui::external_app::gps
