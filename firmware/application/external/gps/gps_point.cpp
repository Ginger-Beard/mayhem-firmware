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

#include "gps_point.hpp"
#include "gps_fmt.hpp"

#include <cmath>

namespace ui::external_app::gps {

const char* compass16(double az) {
    static const char* const kNames[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                           "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    double a = std::fmod(az, 360.0);
    if (a < 0) a += 360.0;
    return kNames[static_cast<int>(std::floor(a / 22.5 + 0.5)) & 15];
}

void PassTrack::start(const SatSource& src, const Observer& obs, double now, bool leo) {
    src_ = src;
    obs_ = obs;
    now_ = now;
    step_ = leo ? 30.0 : 120.0;
    const double back = leo ? 1800.0 : 7 * 3600.0; /* longest pass: ~20 min LEO, ~7 h GPS */
    finder_.start(src_, obs_, now - back, 86400.0 + back, step_);
    pass_ = Pass{};
    k_ = n_ = 0;
    state_ = FIND;
}

bool PassTrack::step(int units) {
    while (units-- > 0) {
        if (state_ == FIND) {
            if (!finder_.done()) {
                finder_.step(1);
                continue;
            }
            if (!finder_.found()) {
                state_ = NONE;
                return false;
            }
            const Pass& p = finder_.pass();
            if (p.set != 0 && p.set < now_) { /* that one is over: the next */
                const double t0 = p.set + step_;
                if (t0 >= now_ + 86400.0) {
                    state_ = NONE;
                    return false;
                }
                finder_.start(src_, obs_, t0, now_ + 86400.0 - t0, step_);
                continue;
            }
            pass_ = p;
            end_ = p.set != 0 ? p.set : now_ + 86400.0;
            k_ = n_ = 0;
            state_ = SAMPLE;
        } else if (state_ == SAMPLE) {
            double az = 0;
            if (k_ < kPts) {
                const double t = pass_.rise + (end_ - pass_.rise) * k_ / (kPts - 1);
                const double el = elevation(src_, obs_, t, &az);
                az_[n_] = static_cast<int16_t>(std::lround(az));
                el_[n_] = static_cast<int16_t>(std::lround(el < 0 ? 0 : el));
                n_++;
                k_++;
            } else {
                elevation(src_, obs_, pass_.top, &az);
                top_az_ = static_cast<float>(az);
                state_ = READY;
                return false;
            }
        } else {
            return false;
        }
    }
    return busy();
}

namespace {

void put_span2(Fmt& f, double s) {
    if (s < 0) s = -s;
    const uint32_t v = static_cast<uint32_t>(s + 0.5);
    if (v >= 3600)
        f.u(v / 3600).c('h').d2((v % 3600) / 60);
    else if (v >= 60)
        f.u(v / 60).c('m').d2(v % 60);
    else
        f.u(v).c('s');
}

void clock(Fmt& f, double unix_s, int32_t off_min) {
    const double t = unix_s + off_min * 60.0;
    const int32_t days = static_cast<int32_t>(std::floor(t / 86400.0));
    const uint32_t sod = static_cast<uint32_t>(t - days * 86400.0);
    f.d2(sod / 3600).c(':').d2((sod / 60) % 60);
}

void dir(Fmt& f, double az) {
    double a = std::fmod(az, 360.0);
    if (a < 0) a += 360.0;
    f.s(compass16(a)).c(' ').u(static_cast<uint32_t>(std::lround(a)) % 360);
}

}  // namespace

void put_clock(char* buf, size_t size, double unix_s, int32_t off_min) {
    Fmt f{buf, size};
    clock(f, unix_s, off_min);
}

void put_point(char* buf, size_t size, double az, double el) {
    Fmt f{buf, size};
    dir(f, az);
    if (el >= 0)
        f.s(", ").u(static_cast<uint32_t>(std::lround(el))).s(" up");
    else
        f.s(", below horizon");
}

void put_countdown(char* buf, size_t size, const Pass& p, double now) {
    Fmt f{buf, size};
    if (now < p.rise) {
        f.s("rises in ");
        put_span2(f, p.rise - now);
    } else if (now < p.top) {
        f.s("peak in ");
        put_span2(f, p.top - now);
    } else if (p.set == 0) {
        f.s("up for over 24 h");
    } else if (now < p.set) {
        f.s("sets in ");
        put_span2(f, p.set - now);
    } else {
        f.s("set ");
        put_span2(f, now - p.set);
        f.s(" ago");
    }
}

void pass_summary(const Pass& p, float top_az, double now, int32_t off_min, char* l1, char* l2, char* l3, size_t size) {
    {
        Fmt f{l1, size};
        if (p.up_at_start) {
            f.s("Up now, rose before ");
            clock(f, p.rise, off_min);
        } else {
            f.s(p.rise <= now ? "Rose  " : "Rises ");
            clock(f, p.rise, off_min);
            f.c(' ');
            dir(f, p.rise_az);
        }
    }
    {
        Fmt f{l2, size};
        f.s("Peak  ");
        clock(f, p.top, off_min);
        f.c(' ').u(static_cast<uint32_t>(std::lround(p.max_el < 0 ? 0 : p.max_el))).s(" up, ").s(compass16(top_az));
    }
    {
        Fmt f{l3, size};
        if (p.set == 0) {
            f.s("Sets after 24 h");
        } else {
            f.s(p.set <= now ? "Set   " : "Sets  ");
            clock(f, p.set, off_min);
            f.c(' ');
            dir(f, p.set_az);
        }
    }
}

}  // namespace ui::external_app::gps
