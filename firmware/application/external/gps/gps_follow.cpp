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

#include "gps_follow.hpp"

#include <cmath>

namespace ui::external_app::gps {

namespace {
int64_t abs64(int64_t v) {
    return v < 0 ? -v : v;
}
}  // namespace

bool look_sgp4(Sgp4& s, const Observer& obs, double unix_s, Look& out) {
    double r[3], v[3];
    const double jd = unix_to_jd(unix_s);
    if (!sgp4_propagate(s, (jd - s.jd_epoch) * 1440.0, r, v)) return false;
    const double g = gmst(jd);
    const double c = std::cos(g), sn = std::sin(g);
    /* TEME -> ECEF (as teme_to_ecef), metres and m/s */
    const Vec3 p{(c * r[0] + sn * r[1]) * 1000.0, (-sn * r[0] + c * r[1]) * 1000.0, r[2] * 1000.0};
    const Vec3 vel{(c * v[0] + sn * v[1]) * 1000.0 + kOmegaE * p.y, (-sn * v[0] + c * v[1]) * 1000.0 - kOmegaE * p.x,
                   v[2] * 1000.0};
    const Vec3 d{p.x - obs.pos.x, p.y - obs.pos.y, p.z - obs.pos.z};
    const double range = norm(d);
    out.range_m = range;
    out.rr = range > 0 ? (d.x * vel.x + d.y * vel.y + d.z * vel.z) / range : 0.0;
    az_el(obs, p, out.az, out.el);
    return true;
}

int32_t doppler_hz(uint64_t f0, double rr) {
    return static_cast<int32_t>(std::lround(-static_cast<double>(f0) * rr / kC));
}

/* ---------------------------------------------------------------- tuner */

void Tuner::reset(uint64_t f0, bool has_ddc, int32_t offset) {
    f0_ = f0;
    has_ddc_ = has_ddc;
    offset_ = offset;
    center_ = 0;
    ddc_sent_ = 0;
    tuned_ = false;
    ddc_valid_ = false;
}

Tuner::Cmd Tuner::update(int32_t doppler) {
    Cmd cmd;
    const int64_t want = static_cast<int64_t>(f0_) + offset_ + doppler;
    if (!has_ddc_) {
        if (!tuned_ || abs64(want - static_cast<int64_t>(center_)) >= kRetuneStep) {
            center_ = static_cast<uint64_t>(want);
            tuned_ = true;
            cmd.retune = true;
            cmd.center = center_;
        }
        return cmd;
    }
    if (!tuned_) {
        center_ = static_cast<uint64_t>(static_cast<int64_t>(f0_) + offset_);
        tuned_ = true;
        cmd.retune = true;
    }
    int64_t ddc = want - static_cast<int64_t>(center_);
    if (ddc > kDdcLimit || ddc < -kDdcLimit) { /* far off (high bands): move the centre */
        center_ = static_cast<uint64_t>(want);
        cmd.retune = true;
        ddc = 0;
    }
    cmd.center = center_;
    if (!ddc_valid_ || cmd.retune || abs64(ddc - ddc_sent_) >= kDdcStep) {
        ddc_sent_ = static_cast<int32_t>(ddc);
        ddc_valid_ = true;
        cmd.ddc = true;
        cmd.ddc_hz = ddc_sent_;
    }
    return cmd;
}

/* ---------------------------------------------------------------- meter */

void Meter::reset() {
    min_ = 0;
    max_ = kFloorDb;
    peak_ = kFloorDb;
    n_ = 0;
    cols_ = 0;
    head_ = 0;
    sub_ = 0;
    col_max_ = kFloorDb;
    for (auto& h : hist_) h = static_cast<int8_t>(kFloorDb);
}

void Meter::feed(int32_t db, float dt_s) {
    if (db < kFloorDb) db = kFloorDb;
    if (db > 0) db = 0;
    level_ = db;
    if (!n_ || db < min_) min_ = db;
    if (db > max_) max_ = db;
    n_++;
    peak_ -= kDecayDbS * dt_s;
    if (peak_ < db) peak_ = static_cast<float>(db);
    if (db > col_max_) col_max_ = db;
    if (++sub_ >= kPerColumn) {
        hist_[head_] = static_cast<int8_t>(col_max_);
        head_ = (head_ + 1) % kHist;
        cols_++;
        sub_ = 0;
        col_max_ = kFloorDb;
    }
}

/* ---------------------------------------------------------------- fade hint */

void FadeHint::reset() {
    head_ = n_ = sub_ = 0;
    acc_ = 0;
    floor_ = 0;
    show_ = quiet_ = 0;
}

void FadeHint::feed(int32_t db, float el_deg) {
    if (db < -127) db = -127;
    if (db > 0) db = 0;
    if (show_ > 0) show_--;
    if (quiet_ > 0) quiet_--;
    acc_ += db;
    if (++sub_ < 10) return;
    second(el_deg);
    sub_ = 0;
    acc_ = 0;
}

void FadeHint::second(float el_deg) {
    const int32_t a = acc_ / 10;
    avg_[head_] = static_cast<int8_t>(a);
    head_ = (head_ + 1) % kSec;
    if (n_ < kSec) n_++;
    /* floor (tenths of a dB): follows a lower level at once, rises 0.1 dB/s */
    if (n_ == 1 || a * 10 < floor_)
        floor_ = a * 10;
    else
        floor_ += 1;
    if (n_ < 5 || el_deg < 15.0f || show_ > 0 || quiet_ > 0) return;
    auto at = [this](int k) { /* k = 0 oldest .. n_-1 newest */
                              return static_cast<int32_t>(avg_[(head_ - n_ + k + 2 * kSec) % kSec]);
    };
    int p = 0;
    for (int k = 1; k < n_; k++)
        if (at(k) >= at(p)) p = k; /* the latest maximum */
    const int last = n_ - 1;
    const int32_t peak = at(p), now = at(last);
    if (last - p < 3 || peak - now < 6) return;
    if (peak * 10 - floor_ < 100 || now * 10 < floor_ + 30) return;
    for (int k = p + 1; k <= last; k++)
        if (at(k - 1) - at(k) > 5) return; /* a step: a transmitter that stopped */
    show_ = 150;
    quiet_ = 150 + 900;
    fired_++;
}

/* ---------------------------------------------------------------- error */

double tuning_error_hz(uint64_t f0, double rr_dot, double clock_err_s, double tle_age_days, double pos_err_km) {
    if (tle_age_days < 0) tle_age_days = 0;
    const double dt = clock_err_s + (1.0 + 1.5 * tle_age_days + pos_err_km) / 7.0;
    return static_cast<double>(f0) * std::fabs(rr_dot) / kC * dt;
}

}  // namespace ui::external_app::gps
