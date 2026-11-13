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
#include "gps_fmt.hpp"

#include <cstring>

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

double elevation(const SatSource& src, const Observer& rx, double unix_s, double* az) {
    Vec3 p;
    if (!src.position(unix_s, p)) return -99.0;
    double a, e;
    az_el(rx, p, a, e);
    if (az) *az = a;
    return e;
}

void PassFinder::start(const SatSource& src, const Observer& rx, double t0, double span, double step) {
    src_ = src;
    rx_ = rx;
    t0_ = t0;
    end_ = t0 + span;
    dt_ = step;
    t_ = t0;
    pass_ = Pass{};
    found_ = false;
    done_ = false;
    phase_ = SCAN;
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

/* The search as a state machine, one elevation per unit (the arithmetic and
 * the order of evaluations are those of a straight bisection (9 steps) and
 * golden-section search (10 steps), so results do not depend on slicing). */
void PassFinder::start_top() {
    ga_ = best_t_ - dt_;
    gb_ = best_t_ + dt_;
    if (ga_ < t0_) ga_ = t0_;
    constexpr double g = 0.6180339887;
    gc_ = gb_ - g * (gb_ - ga_);
    gd_ = ga_ + g * (gb_ - ga_);
    phase_ = TOP_C;
}

void PassFinder::unit() {
    constexpr double g = 0.6180339887;
    double az = 0;
    switch (phase_) {
        case SCAN: {
            const double t = t_ + dt_;
            if (t > end_) {
                if (up_) /* still up at the end of the span */
                    start_top();
                else
                    done_ = true;
                return;
            }
            const double el = elevation(src_, rx_, t, &az);
            if (el <= -98.0) {
                done_ = true;
                return;
            }
            if (!up_ && el > 0.0) {
                up_ = true;
                ba_ = t_;
                bb_ = t;
                rising_ = true;
                k_ = 0;
                phase_ = BISECT;
                best_t_ = t;
                best_el_ = el;
            } else if (up_ && el <= 0.0) {
                ba_ = t_;
                bb_ = t;
                rising_ = false;
                k_ = 0;
                phase_ = BISECT;
            } else if (up_ && el > best_el_) {
                best_el_ = el;
                best_t_ = t;
            }
            el_prev_ = el;
            t_ = t;
            return;
        }
        case BISECT: {
            const double m = 0.5 * (ba_ + bb_);
            const bool above = elevation(src_, rx_, m) > 0.0;
            if (above == rising_)
                bb_ = m;
            else
                ba_ = m;
            if (++k_ >= 9) phase_ = CROSS_AZ;
            return;
        }
        case CROSS_AZ: {
            const double tc = 0.5 * (ba_ + bb_);
            elevation(src_, rx_, tc, &az);
            if (rising_) {
                pass_.rise = tc;
                pass_.rise_az = static_cast<float>(az);
                phase_ = SCAN;
            } else {
                pass_.set = tc;
                pass_.set_az = static_cast<float>(az);
                start_top();
            }
            return;
        }
        case TOP_C:
            gfc_ = elevation(src_, rx_, gc_);
            phase_ = TOP_D;
            return;
        case TOP_D:
            gfd_ = elevation(src_, rx_, gd_);
            k_ = 0;
            phase_ = TOP_ITER;
            return;
        case TOP_ITER:
            if (gfc_ > gfd_) {
                gb_ = gd_;
                gd_ = gc_;
                gfd_ = gfc_;
                gc_ = gb_ - g * (gb_ - ga_);
                gfc_ = elevation(src_, rx_, gc_);
            } else {
                ga_ = gc_;
                gc_ = gd_;
                gfc_ = gfd_;
                gd_ = ga_ + g * (gb_ - ga_);
                gfd_ = elevation(src_, rx_, gd_);
            }
            if (++k_ >= 10) phase_ = TOP_END;
            return;
        case TOP_END: {
            const double t = 0.5 * (ga_ + gb_);
            const double e = elevation(src_, rx_, t);
            if (e > best_el_) {
                best_el_ = e;
                best_t_ = t;
            }
            pass_.top = best_t_;
            pass_.max_el = static_cast<float>(best_el_);
            found_ = true;
            done_ = true;
            return;
        }
    }
}

bool PassFinder::step(int n) {
    while (!done_ && n-- > 0) unit();
    return done_;
}

void PassList::start(const Tle* tles, size_t n, Sgp4* work, const Vec3& rx, double now) {
    tles_ = tles;
    n_ = n < kMax ? n : kMax;
    work_ = work;
    obs_ = make_observer(rx);
    t0_ = now;
    idx_ = 0;
    for (size_t i = 0; i < kMax; i++) ok_[i] = done_[i] = false;
    busy_ = n_ > 0 && tles_ && work_;
    while (busy_ && !start_current()) {
    }
}

/* Start the search of satellite idx_ (skipping element sets SGP4 refuses);
 * false if that one could not start (the next is tried by the caller). */
bool PassList::start_current() {
    while (idx_ < n_) {
        if (sgp4_init(tles_[idx_], *work_)) {
            SatSource src;
            src.kind = SatSource::SGP4;
            src.sgp4 = work_;
            finder_.start(src, obs_, t0_, kSpan, kStep);
            return true;
        }
        done_[idx_] = true;
        idx_++;
    }
    busy_ = false;
    return true;
}

bool PassList::step(int units) {
    if (!busy_) return false;
    if (!finder_.step(units)) return true;
    ok_[idx_] = finder_.found();
    res_[idx_] = finder_.pass();
    done_[idx_] = true;
    idx_++;
    start_current();
    return busy_;
}

namespace {

/* "1h05" / "45m" / "12s" */
void put_span(Fmt& f, double s) {
    if (s < 0) s = 0;
    const uint32_t v = static_cast<uint32_t>(s);
    if (v >= 3600)
        f.u(v / 3600).c('h').d2((v % 3600) / 60);
    else if (v >= 60)
        f.u(v / 60).c('m');
    else
        f.u(v).c('s');
}

}  // namespace

void PassList::order(uint8_t* out) const {
    auto key = [this](size_t i) { /* class, then time */
                                  if (!done_[i]) return 2.0e12 + static_cast<double>(i);
                                  if (!ok_[i]) return 3.0e12 + static_cast<double>(i);
                                  return res_[i].up_at_start ? 0.0 : res_[i].rise;
    };
    for (size_t i = 0; i < n_; i++) out[i] = static_cast<uint8_t>(i);
    for (size_t i = 1; i < n_; i++) { /* insertion sort: stable, at most 12 */
        const uint8_t v = out[i];
        size_t j = i;
        while (j > 0 && key(out[j - 1]) > key(v)) {
            out[j] = out[j - 1];
            j--;
        }
        out[j] = v;
    }
}

void PassList::line(size_t i, double now, char* buf, size_t size) const {
    Fmt f{buf, size};
    if (i >= n_) return;
    const char* name = tles_[i].name;
    for (size_t k = 0; k < kNameCols && name[k]; k++) f.c(name[k]);
    f.pad(kNameCols + 1);
    if (!done_[i]) {
        if (busy_ && i == idx_)
            f.s("search ").u(static_cast<uint32_t>(finder_.progress_pct())).c('%');
        else
            f.s("waiting");
        return;
    }
    if (!ok_[i]) {
        f.s("no pass 24h");
        return;
    }
    const Pass& p = res_[i];
    if (p.up_at_start) {
        f.s("UP ");
    } else {
        f.s("in ");
        put_span(f, p.rise - now);
        f.c(' ');
    }
    f.u(static_cast<uint32_t>(p.max_el < 0 ? 0 : p.max_el + 0.5f)).s("d ");
    if (p.set > 0) put_span(f, p.set - (p.up_at_start ? now : p.rise));
}

}  // namespace ui::external_app::gps
