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

/* The "System" wheel (Heard and Passes) and the other systems' sky from
 * /GPS/GNSSALM.TXT (gps_sys.hpp). The file is read again every 30 s while
 * a view shows another system, a few lines per display frame (each line one
 * orbit evaluation, ~10 ms on the device), with the File on the heap only
 * for that pass; only the elevation and azimuth per satellite stay, in the
 * lent M4 RAM. Nothing here receives: these are predictions until a system
 * is decoded (Galileo is next). */

#include "ui_gps.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

namespace ui::external_app::gps {

#if defined(PRALINE)

namespace {
constexpr systime_t kSysSliceTicks = MS2ST(6);
}

/* the wheel moved (Heard or Passes): both fields follow, the sky is read
 * again if it is old */
void GpsView::set_sys_view(uint8_t v) {
    if (v >= VIEW_N) v = VIEW_ALL_L1;
    if (v == sys_view_) return;
    sys_view_ = v;
    view_heard_.sync_sys();
    view_passes_.sync_sys();
    log_sys("view");
    sys_maybe_start();
    view_passes_.update_sky();
    view_heard_.show();
}

bool GpsView::sys_wanted() {
    if (gal_on_ && tracking_) return true; /* the aided Galileo search needs the orbits */
    if (sys_view_ == VIEW_GPS) return false;
    const uint32_t tab = tab_view_.selected();
    return tab == 1 || tab == 2; /* Heard, Passes */
}

void GpsView::sys_maybe_start() {
    if (!mem_ || !pos_valid_ || sys_active_ || loading() || !sys_wanted()) return;
    const SysSky& k = mem_->sys;
    const uint32_t now = static_cast<uint32_t>(now_best());
    if (k.file == 2 && k.at && now - k.at < 10u * SysSky::kRefreshS) return; /* missing: look again now and then */
    if (k.at && now - k.at < SysSky::kRefreshS) return;
    sys_active_ = true;
    sys_tmp_n_ = 0;
    std::memset(sys_tmp_up_, 0, sizeof(sys_tmp_up_));
    std::memset(sys_tmp_total_, 0, sizeof(sys_tmp_total_));
    sys_tmp_toa_ = 0;
    sys_tmp_bad_ = 0;
    sys_tmp_ngal_ = 0;
    mem_->sys.line_len = 0;
}

void GpsView::sys_close() {
    if (!sys_file_) return;
    sys_file_->~File();
    chHeapFree(sys_file_);
    sys_file_ = nullptr;
}

/* one GNSSALM.TXT line: elevation and azimuth now */
void GpsView::sys_take_line() {
    SysSky& k = mem_->sys;
    const size_t n = k.line_len;
    k.line_len = 0;
    if (!n || k.line[0] == '#') return;
    Orbit o;
    uint8_t sys = 0;
    if (!gnssalm_line(k.line, n, o, sys)) {
        if (sys_tmp_bad_ < 0xFFFF) sys_tmp_bad_++;
        return;
    }
    if (sys_tmp_n_ >= SysSky::kMax) return;
    const double tg = now_gps();
    const Vec3 sp = sat_position(o, tg);
    double az = 0, el = -90;
    az_el(obs_, sp, az, el);
    /* (the views read k.sat[0, k.n) meanwhile: old and new positions mix for
     * a second or two, both close to the truth) */
    SysSat& s = k.sat[sys_tmp_n_++];
    s.sys = sys;
    s.prn = o.prn;
    s.health = o.health;
    s.el = static_cast<int8_t>(std::lround(el < -90 ? -90 : (el > 90 ? 90 : el)));
    s.az2 = static_cast<uint8_t>(static_cast<uint32_t>(std::lround((az < 0 ? az + 360 : az) / 2.0)) % 180u);
    sys_tmp_total_[sys]++;
    if (sys == SYS_GAL && !o.health && sys_tmp_ngal_ < GpsScratch::kMaxGal) mem_->gal[sys_tmp_ngal_++] = gal_alm(o); /* for the aided E1 search */
    if (!o.health && el > 0) sys_tmp_up_[sys]++;
    if (o.toe > sys_tmp_toa_) sys_tmp_toa_ = o.toe;
}

void GpsView::sys_finish(uint8_t file_state) {
    sys_close();
    sys_active_ = false;
    SysSky& k = mem_->sys;
    k.n = static_cast<uint8_t>(sys_tmp_n_);
    mem_->n_gal = static_cast<uint8_t>(sys_tmp_ngal_);
    std::memcpy(k.up, sys_tmp_up_, sizeof(k.up));
    std::memcpy(k.total, sys_tmp_total_, sizeof(k.total));
    k.toa_newest = sys_tmp_toa_;
    k.bad_lines = sys_tmp_bad_;
    k.file = file_state == 1 && !sys_tmp_n_ ? 3 : file_state;
    k.at = static_cast<uint32_t>(now_best());
    log_sys("sky");
    view_passes_.update_sky();
    view_heard_.show();
}

/* frame handler: a few lines until the slice is used (the unit, one line,
 * runs whole) */
void GpsView::sys_step(systime_t t0) {
    if (!sys_active_ || !mem_) return;
    if (!sys_file_) {
        void* const m = chHeapAlloc(nullptr, sizeof(File)); /* not operator new: it halts on a short heap */
        if (!m) return;                                     /* no heap now: next frame */
        sys_file_ = new (m) File{};
        if (sys_file_->open(u"GPS/GNSSALM.TXT").is_valid()) {
            sys_finish(2);
            return;
        }
    }
    SysSky& k = mem_->sys;
    char chunk[48];
    do {
        const auto r = sys_file_->read(chunk, sizeof(chunk));
        if (!r || r.value() == 0) {
            sys_take_line(); /* a last line without a line end */
            sys_finish(1);
            return;
        }
        for (size_t i = 0; i < r.value(); i++) {
            const char c = chunk[i];
            if (c == '\n')
                sys_take_line();
            else if (k.line_len < sizeof(k.line) - 1)
                k.line[k.line_len++] = c;
        }
    } while (chTimeNow() - t0 < kSysSliceTicks);
}

/* the file's age in days (its newest record), -1 unknown */
double GpsView::sys_age_days() const {
    if (!mem_ || !mem_->sys.toa_newest) return -1;
    return (now_gps() - mem_->sys.toa_newest) / 86400.0;
}

/* up to `max` satellites of a system (SYS_N: every other system on L1),
 * highest first, usable ones only; returns the count */
size_t GpsView::sys_top(uint8_t sys, uint8_t* idx, size_t max) const {
    if (!mem_) return 0;
    const SysSky& k = mem_->sys;
    uint32_t taken[(SysSky::kMax + 31) / 32] = {};
    size_t n = 0;
    while (n < max) {
        int best = -1;
        for (size_t i = 0; i < k.n; i++) {
            const SysSat& s = k.sat[i];
            if ((taken[i >> 5] >> (i & 31)) & 1u) continue;
            if (s.health || s.el <= 0) continue;
            if (sys < SYS_N ? s.sys != sys : !sys_on_l1(s.sys, s.prn)) continue;
            if (best < 0 || s.el > k.sat[best].el) best = static_cast<int>(i);
        }
        if (best < 0) break;
        taken[best >> 5] |= 1u << (best & 31);
        idx[n++] = static_cast<uint8_t>(best);
    }
    return n;
}

/* "E11", "C23", "R7", "S131" (lists) */
void GpsView::sys_label(const SysSat& s, char* b, size_t size) const {
    Fmt f{b, size};
    /* a 3-character sky label for SBAS: "S31" for PRN 131 */
    f.c(sys_letter(s.sys)).u(s.sys == SYS_SBAS && size <= 4 && s.prn >= 100 ? s.prn - 100u : s.prn);
}

Color sys_color(uint8_t sys) {
    switch (sys) {
        case SYS_GAL:
            return Color::cyan();
        case SYS_BDS:
            return Color::magenta();
        case SYS_GLO:
            return Color::red();
        case SYS_SBAS:
            return Color::yellow();
        default:
            return Color::green();
    }
}

#endif /* PRALINE */

}  // namespace ui::external_app::gps
