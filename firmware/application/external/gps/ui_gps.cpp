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

#include "ui_gps.hpp"

#include "portapack.hpp"
#include "rtc_time.hpp"
#include "theme.hpp"

#ifdef PRALINE
#include "baseband_api.hpp"
#include "gps_fmt.hpp"
#include "hal.h"

#include <cmath>
#include <cstring>
#endif

using namespace portapack;

namespace ui::external_app::gps {

#ifdef PRALINE

namespace {

constexpr int kLoadLinesPerFrame = 60; /* a few KB of file per display frame */
constexpr double kL1Lambda = kC / 1575.42e6;
constexpr uint8_t kAssistedHalfBins = 6; /* +-3 kHz around the predicted Doppler: TCXO <= ~1.5 ppm */
constexpr uint8_t kColdHalfBins = 14;    /* +-7 kHz */
constexpr double kMinFixEl = 5.0;

static_assert(sizeof(GpsScratch) <= GpsRxStatusMessage::kScratchSize, "GpsScratch: raise GpsRxStatusMessage::kScratchSize");

const char* tle_short(const Tle& t, char* buf, size_t n) {
    size_t i = 0;
    for (; i + 1 < n && t.name[i] && i < 8; i++) buf[i] = t.name[i];
    buf[i] = 0;
    return buf;
}

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

/* Degrees with 5 decimals from 1e-7 degrees (about 1 m). */
void put_deg(Fmt& f, int32_t e7) {
    f.fixed(e7 / 100, 5);
}

}  // namespace

/* ---------------------------------------------------------------- widgets */

void TextLines::set(size_t row, const char* text, Color color) {
    if (row >= rows_) return;
    Line& l = lines_[row];
    if (std::strncmp(l.text, text, kCols) == 0 && l.color.v == color.v) return;
    std::strncpy(l.text, text, kCols);
    l.text[kCols] = 0;
    l.color = color;
    set_dirty();
}

void TextLines::clear() {
    for (size_t i = 0; i < rows_; i++) set(i, "", Color::white());
}

void TextLines::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto bg = Theme::getInstance()->bg_darkest->background;
    const int lh = font_->line_height();
    const size_t cols = static_cast<size_t>(r.width() / font_->char_width());
    for (size_t i = 0; i < rows_; i++) {
        const Point p{r.left(), static_cast<Coord>(r.top() + static_cast<int>(i) * lh)};
        if (p.y() + lh > r.bottom()) break;
        size_t n = std::strlen(lines_[i].text);
        if (n > cols) n = cols;
        const int w = n ? painter.draw_string(p, *font_, lines_[i].color, bg, std::string_view{lines_[i].text, n}) : 0;
        painter.fill_rectangle({p.x() + w, p.y(), r.width() - w, lh}, bg);
    }
}

void SkyPlot::add(int16_t az, int16_t el, const char* label, Color color, bool filled) {
    if (n_ >= kMax) return;
    Dot& d = dots_[n_++];
    d.az = az;
    d.el = el;
    std::strncpy(d.label, label, sizeof(d.label) - 1);
    d.label[sizeof(d.label) - 1] = 0;
    d.color = color;
    d.filled = filled;
}

void SkyPlot::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto bg = Theme::getInstance()->bg_darkest->background;
    const Color grid = Color::dark_grey();
    painter.fill_rectangle(r, bg);
    const int cx = r.left() + r.width() / 2, cy = r.top() + r.height() / 2;
    const int rad = (r.width() < r.height() ? r.width() : r.height()) / 2 - 2;
    /* rings at 0, 30, 60 deg elevation; cross for N-S / E-W */
    for (int ring = 0; ring < 3; ring++) {
        const int rr = rad * (3 - ring) / 3;
        int x = rr, y = 0, err = 1 - rr; /* midpoint circle */
        while (x >= y) {
            const int pts[8][2] = {{x, y}, {y, x}, {-y, x}, {-x, y}, {-x, -y}, {-y, -x}, {y, -x}, {x, -y}};
            for (auto& pt : pts) painter.fill_rectangle({cx + pt[0], cy + pt[1], 1, 1}, grid);
            y++;
            if (err < 0)
                err += 2 * y + 1;
            else {
                x--;
                err += 2 * (y - x) + 1;
            }
        }
    }
    painter.draw_hline({cx - rad, cy}, 2 * rad + 1, grid);
    painter.draw_vline({cx, cy - rad}, 2 * rad + 1, grid);
    painter.draw_string({cx - 2, r.top()}, ui::font::fixed_5x8, Color::light_grey(), bg, "N");
    if (title_) painter.draw_string({r.left(), r.top()}, ui::font::fixed_5x8, Color::light_grey(), bg, title_);
    for (size_t i = 0; i < n_; i++) {
        const Dot& d = dots_[i];
        const int el = d.el < 0 ? 0 : d.el;
        const double rr = rad * (90.0 - el) / 90.0;
        const double a = d.az * kPi / 180.0;
        const int x = cx + static_cast<int>(rr * std::sin(a));
        const int y = cy - static_cast<int>(rr * std::cos(a));
        if (d.filled)
            painter.fill_rectangle({x - 2, y - 2, 5, 5}, d.color);
        else
            painter.draw_rectangle({x - 2, y - 2, 5, 5}, d.color);
        painter.draw_string({x + 3, y - 3}, ui::font::fixed_5x8, d.color, bg, d.label);
    }
}

/* ---------------------------------------------------------------- loader */

OrbitLoader::~OrbitLoader() {
    close();
}

void OrbitLoader::close() {
    delete file_;
    file_ = nullptr;
}

bool OrbitLoader::open(Stage s) {
    close();
    stage_ = s;
    rpos_ = rlen_ = llen_ = 0;
    eof_ = false;
    tle_state_ = 0;
    const char16_t* name = s == ALMANAC ? u"GPS/ALMANAC.TXT" : (s == EPHEMERIS ? u"GPS/EPHEM.RNX" : u"GPS/TLE.TXT");
    file_ = new File();
    if (file_->open(name).is_valid()) { /* an error */
        close();
        return false;
    }
    return true;
}

void OrbitLoader::start(double now_gps) {
    now_gps_ = now_gps;
    yuma_.reset(static_cast<int32_t>(now_gps / kWeekS));
    rinex_.reset();
    stage_ = IDLE;
    if (!open(ALMANAC) && !open(EPHEMERIS) && !open(TLES)) stage_ = DONE;
}

bool OrbitLoader::next_line() {
    llen_ = 0;
    while (true) {
        if (rpos_ >= rlen_) {
            if (eof_) return llen_ > 0;
            const auto r = file_->read(rbuf_, sizeof(rbuf_));
            if (r.is_error() || r.value() == 0) {
                eof_ = true;
                line_[llen_] = 0;
                return llen_ > 0;
            }
            rlen_ = r.value();
            rpos_ = 0;
        }
        const char ch = rbuf_[rpos_++];
        if (ch == '\n') {
            line_[llen_] = 0;
            return true;
        }
        if (ch == '\r') continue;
        if (llen_ + 1 < sizeof(line_)) line_[llen_++] = ch;
    }
}

void OrbitLoader::take(GpsView& v) {
    if (!v.mem_) return;
    Orbit o;
    if (stage_ == ALMANAC) {
        if (yuma_.line(line_, o)) {
            Orbit& slot = v.mem_->orbits[o.prn - 1];
            if (!slot.valid || (!slot.ephemeris && std::fabs(o.toe - now_gps_) < std::fabs(slot.toe - now_gps_))) {
                if (!slot.valid) v.n_alm_++;
                slot = o;
            }
        }
    } else if (stage_ == EPHEMERIS) {
        if (rinex_.line(line_, o) && std::fabs(o.toe - now_gps_) < 4 * 3600.0) {
            Orbit& slot = v.mem_->orbits[o.prn - 1];
            const bool better = !slot.valid || !slot.ephemeris ||
                                (slot.health != 0 && o.health == 0) ||
                                (o.health == slot.health && std::fabs(o.toe - now_gps_) < std::fabs(slot.toe - now_gps_));
            if (better) {
                if (!slot.valid || !slot.ephemeris) v.n_eph_++;
                slot = o;
                if (o.toe > v.eph_toe_newest_) v.eph_toe_newest_ = o.toe;
            }
        }
    } else if (stage_ == TLES) {
        const size_t n = std::strlen(line_);
        if (line_[0] == '1' && line_[1] == ' ' && n >= 69) {
            std::strncpy(tle_l1_, line_, sizeof(tle_l1_) - 1);
            tle_state_ = 1;
        } else if (line_[0] == '2' && line_[1] == ' ' && n >= 69 && tle_state_ == 1) {
            tle_state_ = 0;
            if (v.n_tle_ < GpsScratch::kMaxTle) {
                Tle& t = v.mem_->tles[v.n_tle_];
                if (parse_tle(tle_l1_, line_, t)) {
                    std::strncpy(t.name, tle_name_, sizeof(t.name) - 1);
                    if (!t.name[0]) Fmt{t.name, sizeof(t.name)}.u(t.catnr);
                    v.n_tle_++;
                }
            }
        } else if (n > 0) { /* a name line ("0 " prefix in some files) */
            const char* s = (line_[0] == '0' && line_[1] == ' ') ? line_ + 2 : line_;
            size_t k = 0;
            while (s[k] && k + 1 < sizeof(tle_name_)) {
                tle_name_[k] = s[k];
                k++;
            }
            while (k && tle_name_[k - 1] == ' ') k--;
            tle_name_[k] = 0;
            tle_state_ = 0;
        }
    }
}

bool OrbitLoader::step(GpsView& v) {
    if (stage_ == IDLE || stage_ == DONE || !file_) return false;
    for (int i = 0; i < kLoadLinesPerFrame; i++) {
        if (!next_line()) {
            const Stage s = stage_;
            close();
            bool opened = false;
            if (s == ALMANAC) opened = open(EPHEMERIS) || open(TLES);
            if (s == EPHEMERIS) opened = open(TLES);
            if (!opened) {
                stage_ = DONE;
                return false;
            }
            return true;
        }
        take(v);
    }
    return true;
}

/* ---------------------------------------------------------------- passes */

PassesView::PassesView(GpsView& main, Rect r)
    : View{r},
      main_{main},
      text_head_{{0, 0, UI_POS_WIDTH(24), UI_POS_HEIGHT(kRows)}, head_, kRows, &ui::font::fixed_8x16},
      button_pos_{{UI_POS_X(24), 0, UI_POS_WIDTH(6), UI_POS_HEIGHT(1)}, "Pos"},
      button_load_{{UI_POS_X(24), UI_POS_Y(3), UI_POS_WIDTH(6), UI_POS_HEIGHT(1)}, "Load"},
      labels_zone_{{{UI_POS_X(15), UI_POS_Y(1)}, "Zone", Theme::getInstance()->fg_light->foreground}},
      field_zone_h_{{UI_POS_X(20), UI_POS_Y(1)}, 3, {-12, 14}, 1, ' '},
      field_zone_m_{{UI_POS_X(23), UI_POS_Y(1)}, 3, {{":00", 0}, {":30", 30}, {":45", 45}}},
      sky_{{0, UI_POS_Y(4) + 2, 112, 112}, "GPS now"},
      text_side_{{116, UI_POS_Y(4) + 2, 124, 112}, side_, kSideRows, &ui::font::fixed_5x8},
      text_passes_{{0, UI_POS_Y(4) + 116, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kPassRows)}, pass_lines_, kPassRows, &ui::font::fixed_8x16} {
    add_children({&text_head_, &button_pos_, &button_load_, &labels_zone_, &field_zone_h_, &field_zone_m_, &sky_, &text_side_, &text_passes_});
    /* zone as hours + minutes (e.g. -5 :00, 5 :30); the setting keeps minutes */
    const int32_t z = main_.utc_off_min_;
    const int32_t zh = z / 60, zm = (z < 0 ? -z : z) % 60;
    field_zone_h_.set_value(zh);
    field_zone_m_.set_selected_index(zm == 30 ? 1 : (zm == 45 ? 2 : 0), false);
    field_zone_h_.on_change = [this](int32_t) {
        zone_changed();
    };
    field_zone_m_.on_change = [this](size_t, OptionsField::value_t) {
        zone_changed();
    };
    button_pos_.on_select = [this](Button&) {
        main_.nav_.push<PositionView>(main_);
    };
    button_load_.on_select = [this](Button&) {
        main_.start_load();
    };
}

void PassesView::zone_changed() {
    const int32_t h = field_zone_h_.value();
    const int32_t m = field_zone_m_.selected_index_value();
    main_.utc_off_min_ = h * 60 + (h < 0 ? -m : m);
    refresh();
    update_sky();
    restart_passes();
}

void PassesView::focus() {
    button_pos_.focus();
}

void PassesView::refresh() {
    char b[TextLines::kCols + 1];
    const Color white = Color::white(), yellow = Color::yellow(), grey = Color::light_grey();
    {
        Fmt f{b, sizeof(b)};
        if (main_.pos_valid_) {
            put_deg(f, main_.pos_.lat_e7);
            f.c(' ');
            put_deg(f, main_.pos_.lon_e7);
            text_head_.set(0, b, white);
        } else {
            text_head_.set(0, "No position: Pos", yellow);
        }
    }
    auto put_hms = [](Fmt& f, double t) {
        const int32_t days = static_cast<int32_t>(std::floor(t / 86400.0));
        const uint32_t sod = static_cast<uint32_t>(t - days * 86400.0);
        f.d2(sod / 3600).c(':').d2((sod / 60) % 60).c(':').d2(sod % 60);
    };
    {
        Fmt f{b, sizeof(b)};
        f.s("Local ");
        put_hms(f, main_.local_unix());
        text_head_.set(1, b, grey);
    }
    {
        Fmt f{b, sizeof(b)};
        const double t = main_.now_unix();
        int32_t y, m, d;
        civil_from_days(static_cast<int32_t>(std::floor(t / 86400.0)), y, m, d);
        f.s("UTC   ");
        put_hms(f, t);
        f.c(' ').d2(static_cast<uint32_t>(m)).c('-').d2(static_cast<uint32_t>(d));
        if (main_.utc_off_min_ == 0)
            text_head_.set(2, "Zone 0: clock must be UTC!", yellow); /* the commonest mistake */
        else
            text_head_.set(2, b, white);
    }
    {
        Fmt f{b, sizeof(b)};
        const auto st = main_.loader_.stage();
        if (!main_.mem_) {
            text_head_.set(3, "Starting...", yellow);
        } else if (st != OrbitLoader::DONE && st != OrbitLoader::IDLE) {
            f.s(st == OrbitLoader::ALMANAC ? "Reading ALMANAC" : (st == OrbitLoader::EPHEMERIS ? "Reading EPHEM" : "Reading TLE"));
            text_head_.set(3, b, yellow);
        } else if (!main_.n_alm_ && !main_.n_eph_ && !main_.n_tle_) {
            text_head_.set(3, "No /GPS orbit files", yellow);
        } else {
            f.s("Alm ").u(main_.n_alm_).s(" Eph ").u(main_.n_eph_).s(" TLE ").u(main_.n_tle_);
            text_head_.set(3, b, white);
        }
    }
}

void PassesView::update_sky() {
    if (hidden()) return; /* redrawn when the tab is selected */
    sky_.clear();
    text_side_.clear();
    if (!main_.pos_valid_ || !main_.mem_) {
        sky_.set_dirty();
        text_side_.set(0, main_.pos_valid_ ? "" : "Set a position", Color::yellow());
        return;
    }
    const double t = main_.now_unix();
    const double tg = unix_to_gps(t);
    char b[TextLines::kCols + 1];
    /* GPS satellites above the horizon; the side list in PRN order */
    size_t nv = 0;
    for (size_t i = 0; i < 32; i++) {
        const Orbit& o = main_.mem_->orbits[i];
        if (!o.valid || o.health) continue;
        double az, el;
        az_el(main_.obs_, sat_position(o, tg), az, el);
        if (el < 0) continue;
        char lbl[4];
        Fmt{lbl, sizeof(lbl)}.u(o.prn);
        sky_.add(static_cast<int16_t>(az), static_cast<int16_t>(el), lbl, Color::green(), true);
        nv++;
        if (nv + 1 < kSideRows) {
            Fmt f{b, sizeof(b)};
            f.c('G').d2(o.prn).s(" el").u(static_cast<uint32_t>(el), 2).s(" az").u(static_cast<uint32_t>(az), 3);
            text_side_.set(nv, b, Color::white());
        }
    }
    {
        Fmt f{b, sizeof(b)};
        f.s("GPS up now: ").u(static_cast<uint32_t>(nv));
        text_side_.set(0, b, Color::light_grey());
    }
    /* TLE satellites (orange) if up */
    for (size_t i = 0; i < main_.n_tle_; i++) {
        if (!sgp4_init(main_.mem_->tles[i], main_.mem_->sky)) continue;
        SatSource src;
        src.kind = SatSource::SGP4;
        src.sgp4 = &main_.mem_->sky;
        double az = 0;
        const double el = elevation(src, main_.obs_, t, &az);
        if (el < 0) continue;
        char nm[9];
        tle_short(main_.mem_->tles[i], nm, 4);
        sky_.add(static_cast<int16_t>(az), static_cast<int16_t>(el), nm, Color::orange(), false);
    }
    sky_.set_dirty();
}

void PassesView::restart_passes() {
    passes_.stop();
    if (main_.pos_valid_ && main_.n_tle_ && main_.mem_)
        passes_.start(main_.mem_->tles, main_.n_tle_, &main_.mem_->search, main_.rx_, main_.now_unix());
    show_passes();
}

/* A slice of the pass search per display frame, by time (~8 ms), so the UI
 * stays responsive however fast the M0 runs it. */
void PassesView::frame() {
    if (!passes_.busy() || !main_.mem_) return;
    const systime_t t0 = chTimeNow();
    do {
        if (!passes_.step(2)) break;
    } while (chTimeNow() - t0 < MS2ST(8));
    show_passes();
}

void PassesView::show_passes() {
    char b[TextLines::kCols + 1];
    const double now = main_.now_unix();
    for (size_t i = 0; i < kPassRows; i++) {
        if (!main_.n_tle_ || !main_.mem_) {
            text_passes_.set(i, i == 0 ? "No TLE (GPS/TLE.TXT)" : "", Color::light_grey());
            continue;
        }
        if (!main_.pos_valid_) {
            text_passes_.set(i, i == 0 ? "Passes need a position" : "", Color::yellow());
            continue;
        }
        if (i >= passes_.count()) {
            text_passes_.set(i, "", Color::white());
            continue;
        }
        passes_.line(i, now, b, sizeof(b));
        text_passes_.set(i, b, passes_.found(i) ? Color::white() : Color::light_grey());
    }
}

/* ---------------------------------------------------------------- heard */

HeardView::HeardView(GpsView& main, Rect r)
    : View{r},
      main_{main},
      field_depth_{{UI_POS_X(0), 0}, 10, {{"Quick 10ms", 0}, {"Deep 20ms", 1}}},
      button_search_{{UI_POS_X(17), 0, UI_POS_WIDTH(7), UI_POS_HEIGHT(1)}, "Search"},
      button_stop_{{UI_POS_X(24), 0, UI_POS_WIDTH(6), UI_POS_HEIGHT(1)}, "Stop"},
      text_status_{{0, UI_POS_Y(1), UI_POS_MAXWIDTH, UI_POS_HEIGHT(kStatusRows)}, status_, kStatusRows, &ui::font::fixed_8x16},
      text_list_{{0, UI_POS_Y(3) + 2, 118, 8 * kListRows}, list_, kListRows, &ui::font::fixed_5x8},
      sky_{{120, UI_POS_Y(3) + 2, 120, 120}, "Heard"} {
    add_children({&field_depth_, &button_search_, &button_stop_, &text_status_, &text_list_, &sky_});
    button_stop_.on_select = [this](Button&) {
        main_.stop_search();
        show();
    };
    field_depth_.set_selected_index(main_.depth_ ? 1 : 0, false);
    field_depth_.on_change = [this](size_t i, OptionsField::value_t) {
        main_.depth_ = static_cast<uint32_t>(i);
    };
    button_search_.on_select = [this](Button&) {
        main_.start_search(GpsView::JOB_HEARD, main_.depth_ ? 20 : 10);
        show();
    };
    show();
}

void HeardView::focus() {
    button_search_.focus();
}

void HeardView::show() {
    if (hidden()) return; /* redrawn when the tab is selected (GpsView::on_frame) */
    char b[TextLines::kCols + 1];
    const auto& st = main_.status_;
    const bool mine = main_.job_ == GpsView::JOB_HEARD || main_.job_ == GpsView::JOB_FIX;
    /* line 0: what is going on */
    {
        Fmt f{b, sizeof(b)};
        Color c = Color::white();
        if (!main_.mem_) {
            f.s("Baseband starting...");
            c = Color::yellow();
        } else if (!mine) {
            f.s(main_.assisted_possible() ? "Search: assisted" : "Search: all 32 PRNs");
            c = Color::light_grey();
        } else if (!main_.done_) {
            if (st.state == GpsRxStatusMessage::CAPTURING || main_.awaiting_seq_)
                f.s(st.confirming ? "Confirming: capture" : "Capturing ").u(st.k_ms).s(" ms...");
            else if (st.confirming)
                f.s("Confirming ").u(st.done_prn).c('/').u(st.total_prn);
            else
                f.s(st.state == GpsRxStatusMessage::REFINING ? "Refining " : "Searching ").u(st.done_prn).c('/').u(st.total_prn).c(' ').u(static_cast<uint32_t>(main_.progress_pct())).c('%');
            c = Color::yellow();
        } else if (st.level == GpsRxStatusMessage::LEVEL_LOW) {
            f.s("Signal too low: no search");
            c = Color::red();
        } else {
            int n = 0;
            for (int p = 0; p < 32; p++) n += main_.results_ && main_.results_[p].detected;
            f.s("Heard ").u(static_cast<uint32_t>(n)).s(" (confirmed)");
            c = n ? Color::green() : Color::yellow();
        }
        text_status_.set(0, b, c);
    }
    /* line 1: the measured input level (AC rms per component of the snapshot,
     * DC removed) against the target, and the M4 time of one search step */
    if (mine && st.rms_x10) {
        Fmt f{b, sizeof(b)};
        f.s("Level ").fixed(st.rms_x10, 1).s(" LSB");
        Color c = Color::green();
        if (st.level == GpsRxStatusMessage::LEVEL_LOW || st.rms_x10 < 15) {
            f.s(" LOW gain+");
            c = Color::red();
        } else if (st.level == GpsRxStatusMessage::LEVEL_HIGH || st.clip_permille > 10) {
            f.s(" CLIP gain-");
            c = Color::red();
        } else if (st.rms_x10 < 40) {
            f.s(" low");
            c = Color::yellow();
        } else {
            f.s(" ok");
        }
        if (st.unit_cycles) f.s(" ").u(st.unit_cycles / 20000u).s("0us"); /* M4 time of one work unit, 10 us steps */
        text_status_.set(1, b, c);
    } else {
        text_status_.set(1, "", Color::white());
    }
    /* list: detected PRNs by C/N0 with a bar; guidance when none */
    text_list_.clear();
    sky_.clear();
    if (!main_.done_ || !main_.results_) {
        sky_.set_dirty();
        return;
    }
    uint8_t order[32]; /* detected PRN indices, strongest first (indices only: small stack) */
    size_t n = 0;
    for (int p = 0; p < 32; p++)
        if (main_.results_[p].detected) order[n++] = static_cast<uint8_t>(p);
    for (size_t a = 1; a < n; a++)
        for (size_t k = a; k > 0 && main_.results_[order[k]].cn0 > main_.results_[order[k - 1]].cn0; k--) {
            const uint8_t t = order[k];
            order[k] = order[k - 1];
            order[k - 1] = t;
        }
    if (st.level == GpsRxStatusMessage::LEVEL_LOW) {
        text_list_.set(0, "Input too low to trust:", Color::red());
        text_list_.set(1, "the ADC barely moves.", Color::white());
        text_list_.set(3, "- raise LNA / VGA (40/46)", Color::white());
        text_list_.set(4, "- active antenna? turn", Color::white());
        text_list_.set(5, "  Ant power on (status bar)", Color::white());
        text_list_.set(6, "- target level 4-30 LSB", Color::white());
        sky_.set_dirty();
        return;
    }
    if (!n) {
        text_list_.set(0, "No satellites heard.", Color::yellow());
        text_list_.set(2, "- active antenna? turn", Color::white());
        text_list_.set(3, "  Ant power on (status bar)", Color::white());
        text_list_.set(4, "- open sky, outdoors", Color::white());
        text_list_.set(5, "- level 4-30 LSB:", Color::white());
        text_list_.set(6, "  adjust LNA / VGA", Color::white());
        text_list_.set(7, "- Deep 20ms", Color::white());
    }
    for (size_t k = 0; k < n && k < kListRows; k++) {
        const auto& r = main_.results_[order[k]];
        Fmt f{b, sizeof(b)};
        f.c('G').d2(r.prn).c(' ');
        int bars = static_cast<int>((r.cn0 - 32.0f) / 1.5f);
        if (bars < 1) bars = 1;
        if (bars > 10) bars = 10;
        for (int i = 0; i < 10; i++) f.c(i < bars ? '|' : ' ');
        f.c(' ').u(static_cast<uint32_t>(r.cn0 + 0.5f)).c(' ');
        const int32_t dop = static_cast<int32_t>(std::lround(r.doppler / 100.0f));
        if (dop >= 0) f.c('+');
        f.fixed(dop, 1).c('k');
        text_list_.set(k, b, r.cn0 >= 38 ? Color::green() : Color::white());
    }
    /* seen in one snapshot only: shown grey with '?', not counted as heard */
    size_t row = n ? n : 9;
    for (int p = 0; p < 32 && row < kListRows; p++) {
        const auto& r = main_.results_[p];
        if (!r.once) continue;
        Fmt f{b, sizeof(b)};
        f.c('G').d2(r.prn).s("? once, not confirmed");
        text_list_.set(row++, b, Color::grey());
    }
    /* sky plot when orbits and a position are known: heard filled */
    if (main_.mem_ && main_.pos_valid_) {
        const double tg = main_.now_gps();
        for (int p = 0; p < 32; p++) {
            const Orbit& o = main_.mem_->orbits[p];
            if (!o.valid || o.health) continue;
            double az, el;
            az_el(main_.obs_, sat_position(o, tg), az, el);
            if (el < 0) continue;
            char lbl[4];
            Fmt{lbl, sizeof(lbl)}.u(o.prn);
            const bool heard = main_.results_[p].detected;
            sky_.add(static_cast<int16_t>(az), static_cast<int16_t>(el), lbl, heard ? Color::green() : Color::grey(), heard);
        }
    }
    sky_.set_dirty();
}

/* ---------------------------------------------------------------- fix */

FixView::FixView(GpsView& main, Rect r)
    : View{r},
      main_{main},
      field_method_{{UI_POS_X(0), 0}, 18, {{"Snapshot+eph file", 0}, {"Autonomous", 1}}},
      button_fix_{{UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(14), UI_POS_HEIGHT(1)}, "Get fix"},
      button_save_{{UI_POS_X(16), UI_POS_Y(1), UI_POS_WIDTH(14), UI_POS_HEIGHT(1)}, "Save pos"},
      text_{{0, UI_POS_Y(2) + 4, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kRows)}, lines_, kRows, &ui::font::fixed_8x16} {
    add_children({&field_method_, &button_fix_, &button_save_, &text_});
    field_method_.set_selected_index(0, false);
    field_method_.on_change = [this](size_t, OptionsField::value_t) {
        show();
    };
    button_fix_.on_select = [this](Button&) {
        if (field_method_.selected_index_value() != 0) return;
        if (main_.searching()) { /* the button reads "Stop" while a search runs */
            main_.stop_search();
            show();
            return;
        }
        main_.fix_tried_ = false;
        main_.fix_ok_ = false;
        main_.start_search(GpsView::JOB_FIX, 20);
        show();
    };
    button_save_.on_select = [this](Button&) {
        if (!main_.fix_ok_) {
            text_.set(kRows - 1, "No fix to save", Color::yellow());
            return;
        }
        saved_position::Position p;
        double lat, lon, h;
        ecef_to_lla(main_.fix_.pos, lat, lon, h);
        p.lat_e7 = static_cast<int32_t>(std::lround(lat * 1e7));
        p.lon_e7 = static_cast<int32_t>(std::lround(lon * 1e7));
        p.alt_m = static_cast<int32_t>(std::lround(h));
        p.source = saved_position::SOURCE_HACKRF_GPS;
        p.time = main_.fix_fat_;
        p.sats = main_.fix_.used;
        p.acc_m = static_cast<uint32_t>(std::lround(main_.fix_.hdop * 15.0f));
        if (saved_position::save(p)) {
            main_.load_position();
            text_.set(kRows - 1, "Saved for all apps", Color::green());
        } else {
            text_.set(kRows - 1, "SD write failed", Color::red());
        }
    };
    show();
}

void FixView::focus() {
    button_fix_.focus();
}

void FixView::show() {
    if (hidden()) return; /* redrawn when the tab is selected (GpsView::on_frame) */
    char b[TextLines::kCols + 1];
    if (main_.searching() != fix_running_shown_) {
        fix_running_shown_ = main_.searching();
        button_fix_.set_text(fix_running_shown_ ? "Stop" : "Get fix");
    }
    text_.clear();
    if (field_method_.selected_index_value() != 0) {
        text_.set(0, "Autonomous: NOT AVAILABLE", Color::yellow());
        text_.set(1, "yet. It needs real-time", Color::white());
        text_.set(2, "tracking of 4+ satellites", Color::white());
        text_.set(3, "for 30+ s to decode the", Color::white());
        text_.set(4, "orbits from the signal;", Color::white());
        text_.set(5, "not built. Use Snapshot", Color::white());
        text_.set(6, "with GPS/EPHEM.RNX.", Color::white());
        return;
    }
    /* requirements */
    int vis = 0;
    if (main_.mem_ && main_.pos_valid_) {
        const double tg = main_.now_gps();
        for (int p = 0; p < 32; p++) {
            const Orbit& o = main_.mem_->orbits[p];
            if (!o.valid || !o.ephemeris || o.health) continue;
            double az, el;
            az_el(main_.obs_, sat_position(o, tg), az, el);
            vis += el > kMinFixEl;
        }
    }
    const auto& st = main_.status_;
    if (main_.job_ == GpsView::JOB_FIX && !main_.done_) {
        Fmt f{b, sizeof(b)};
        if (st.state == GpsRxStatusMessage::CAPTURING || main_.awaiting_seq_)
            f.s("Capturing 20 ms...");
        else
            f.s(st.state == GpsRxStatusMessage::REFINING ? "Refining " : "Searching ").u(st.done_prn).c('/').u(st.total_prn).c(' ').u(static_cast<uint32_t>(main_.progress_pct())).c('%');
        text_.set(0, b, Color::yellow());
    } else if (main_.solve_pending_) {
        text_.set(0, "Solving...", Color::yellow());
    } else if (main_.fix_ok_) {
        const Solution& s = main_.fix_;
        double lat, lon, h;
        ecef_to_lla(s.pos, lat, lon, h);
        {
            Fmt f{b, sizeof(b)};
            f.s("Lat ");
            put_deg(f, static_cast<int32_t>(std::lround(lat * 1e7)));
            text_.set(0, b, Color::green());
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Lon ");
            put_deg(f, static_cast<int32_t>(std::lround(lon * 1e7)));
            text_.set(1, b, Color::green());
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Alt ").d(static_cast<int32_t>(std::lround(h))).s(" m (rough)");
            text_.set(2, b, Color::white());
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Err est. ~").u(static_cast<uint32_t>(std::lround(s.hdop * 15.0f))).s(" m HDOP ").fixed(static_cast<int32_t>(s.hdop * 10), 1);
            text_.set(3, b, Color::white());
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Sats ").u(s.used).s(" used, ").u(main_.fix_heard_).s(" heard");
            text_.set(4, b, Color::white());
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Resid ").fixed(static_cast<int32_t>(s.resid_rms_m * 10), 1).s(" m, clock ");
            f.d(static_cast<int32_t>(std::lround(s.time_corr_s))).c('s');
            text_.set(5, b, Color::white());
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Ephemeris age ");
            put_span(f, main_.fix_eph_age_s_ < 0 ? -main_.fix_eph_age_s_ : main_.fix_eph_age_s_);
            text_.set(6, b, Color::white());
        }
    } else if (main_.done_ && main_.job_ == GpsView::JOB_FIX && st.level == GpsRxStatusMessage::LEVEL_LOW) {
        text_.set(0, "Signal too low: no search", Color::red());
        text_.set(1, "raise LNA / VGA (40/46);", Color::white());
        text_.set(2, "Ant power for an active", Color::white());
        text_.set(3, "antenna (status bar)", Color::white());
    } else if (main_.fix_tried_) {
        Fmt f{b, sizeof(b)};
        f.s("No fix: ").u(main_.fix_heard_).s(" sats heard");
        text_.set(0, b, Color::yellow());
        text_.set(1, "needs 6+ with ephemeris;", Color::white());
        text_.set(2, "see Heard for signal level", Color::white());
    }
    /* what the method needs (always shown below) */
    const size_t r0 = 8;
    if (!main_.pos_valid_) text_.set(r0, "Needs a rough position", Color::yellow());
    if (!main_.n_eph_) text_.set(r0 + 1, "Needs GPS/EPHEM.RNX (today)", Color::yellow());
    if (main_.pos_valid_ && main_.n_eph_) {
        Fmt f{b, sizeof(b)};
        f.s("Eph sats up: ").u(static_cast<uint32_t>(vis));
        text_.set(r0, b, vis >= 6 ? Color::light_grey() : Color::yellow());
    }
}

/* ---------------------------------------------------------------- position */

PositionView::PositionView(NavigationView& nav, GpsView& main)
    : main_{main},
      geopos_{{0, UI_POS_Y(1)}, GeoPos::alt_unit::METERS, GeoPos::spd_unit::HIDDEN},
      text_info_{{0, UI_POS_Y(7), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "Saved for all apps (Manual)"},
      button_save_{{0, UI_POS_Y(9), UI_POS_WIDTH_PERCENT(50), UI_POS_HEIGHT(2)}, "Save"} {
    add_children({&geopos_, &text_info_, &button_save_});
    if (main_.pos_valid_) {
        geopos_.set_lat(main_.pos_.lat_e7 / 1e7f);
        geopos_.set_lon(main_.pos_.lon_e7 / 1e7f);
        geopos_.set_altitude(main_.pos_.alt_m);
    }
    button_save_.on_select = [this, &nav](Button&) {
        saved_position::Position p;
        p.lat_e7 = static_cast<int32_t>(std::lround(static_cast<double>(geopos_.lat()) * 1e7));
        p.lon_e7 = static_cast<int32_t>(std::lround(static_cast<double>(geopos_.lon()) * 1e7));
        p.alt_m = geopos_.altitude();
        p.source = saved_position::SOURCE_MANUAL;
        p.time = saved_position::now_fat();
        if (!p.valid() || !saved_position::save(p)) {
            text_info_.set("Not saved (0/0 or SD error)");
            return;
        }
        main_.load_position();
        nav.pop();
    };
}

void PositionView::focus() {
    geopos_.focus();
}

/* ---------------------------------------------------------------- main */

GpsView::GpsView(NavigationView& nav)
    : nav_{nav},
      settings_{
          "rx_gps",
          app_settings::Mode::RX,
          {{"utc_off_min"sv, &utc_off_min_},
           {"tab"sv, &tab_},
           {"depth"sv, &depth_}}},
      view_fix_{*this, view_rect_},
      view_heard_{*this, view_rect_},
      view_passes_{*this, view_rect_},
      tab_view_{
          {"Fix", Theme::getInstance()->fg_cyan->foreground, &view_fix_},
          {"Heard", Theme::getInstance()->fg_green->foreground, &view_heard_},
          {"Passes", Theme::getInstance()->fg_yellow->foreground, &view_passes_}} {
    baseband::run_image(portapack::spi_flash::image_tag_gps_rx);

    add_children({&labels_, &field_lna_, &field_vga_, &field_amp_, &text_ant_, &text_warn_,
                  &tab_view_, &view_fix_, &view_heard_, &view_passes_});
    tab_view_.set_parent_rect({0, UI_POS_Y(2), screen_width, 24});

    if (!settings_.loaded()) { /* an active antenna gives plenty: moderate gains */
        receiver_model.set_rf_amp(false);
        receiver_model.set_lna(40); /* bench recordings: LNA 32 / VGA 24 left the ADC at ~0.6 LSB rms */
        receiver_model.set_vga(46);
    }
    field_lna_.set_value(receiver_model.lna());
    field_vga_.set_value(receiver_model.vga());
    field_amp_.set_value(receiver_model.rf_amp());

    show_antenna();

    load_position();
    view_passes_.refresh();
    tick_token_ = rtc_time::signal_tick_second += [this]() {
        on_tick_second();
    };
    /* TabView hides only the previous view: hide all three, then show one */
    view_fix_.hidden(true);
    view_heard_.hidden(true);
    view_passes_.hidden(true);
    tab_view_.set_selected(tab_ < 3 ? tab_ : 2);
}

GpsView::~GpsView() {
    rtc_time::signal_tick_second -= tick_token_;
    tab_ = tab_view_.selected();
    mem_ = nullptr; /* the lent RAM goes with the baseband */
    if (rx_on_) receiver_model.disable();
    baseband::shutdown();
}

void GpsView::focus() {
    tab_view_.focus();
}

/* The RTC hardware keeps standard local time when Mayhem's DST setting is on
 * (rtc_time adds the hour only when read), or whatever the clock was set to
 * when it is off: UTC = that - the zone, so the zone is the standard offset
 * with DST on and the offset in force with DST off. */
double GpsView::now_unix() const {
    rtc::RTC raw;
    rtcGetTime(&RTCD1, &raw);
    return static_cast<double>(rtc_time::rtcToUnixUTC(raw)) - utc_off_min_ * 60.0;
}

double GpsView::local_unix() const {
    return static_cast<double>(rtc_time::rtcToUnixUTC(rtc_time::now()));
}

void GpsView::load_position() {
    pos_valid_ = saved_position::load(pos_);
    if (pos_valid_) {
        rx_ = lla_to_ecef(pos_.lat_e7 / 1e7, pos_.lon_e7 / 1e7, pos_.alt_m);
        obs_ = make_observer(rx_);
    }
    view_passes_.refresh();
    view_passes_.update_sky();
    view_passes_.restart_passes();
    view_fix_.show();
}

void GpsView::start_load() {
    if (!mem_) {
        load_wanted_ = true;
        return;
    }
    load_wanted_ = false;
    for (auto& o : mem_->orbits) o = Orbit{};
    n_alm_ = n_eph_ = n_tle_ = 0;
    eph_toe_newest_ = 0;
    loader_.start(now_gps());
    view_passes_.refresh();
}

bool GpsView::assisted_possible() const {
    return mem_ && pos_valid_ && (n_alm_ || n_eph_);
}

bool GpsView::start_search(Job job, uint8_t k_ms) {
    if (!mem_) return false;
    if (!rx_on_) {
        receiver_model.enable();
        rx_on_ = true;
    }
    GpsRxConfigMessage m;
    m.cmd = GpsRxConfigMessage::CMD_SEARCH;
    m.k_ms = k_ms;
    m.refine = job == JOB_FIX;
    m.confirm = 1; /* a PRN is reported only if a second snapshot confirms it */
    int nprn = 0;
    if (assisted_possible()) {
        /* visible PRNs (by the orbit files), Doppler predicted from the range rate */
        const double tg = now_gps();
        const double min_el = job == JOB_FIX ? kMinFixEl : 0.0;
        for (int p = 0; p < 32; p++) {
            const Orbit& o = mem_->orbits[p];
            if (!o.valid || o.health || (job == JOB_FIX && !o.ephemeris)) continue;
            const Vec3 s = sat_position(o, tg);
            double az, el;
            az_el(obs_, s, az, el);
            if (el <= min_el) continue;
            const Vec3 v = sat_velocity(o, tg);
            const Vec3 d{s.x - rx_.x, s.y - rx_.y, s.z - rx_.z};
            const double r = norm(d);
            const double rr = (v.x * d.x + v.y * d.y + v.z * d.z) / r;
            m.prn_mask |= 1u << p;
            m.center[p] = static_cast<int16_t>(std::lround(-rr / kL1Lambda));
            nprn++;
        }
        m.half_bins = kAssistedHalfBins;
    }
    if (!nprn) {
        if (job == JOB_FIX) return false;
        m.prn_mask = 0xFFFFFFFFu;
        m.half_bins = kColdHalfBins;
        nprn = 32;
    }
    job_units_ = static_cast<uint32_t>(nprn) * (2u * m.half_bins + 1u) * k_ms;
    job_ = job;
    done_ = false;
    awaiting_seq_ = true;
    results_ = nullptr;
    baseband::gps_rx_config(m);
    return true;
}

void GpsView::stop_search() {
    if (!searching()) return;
    GpsRxConfigMessage m;
    m.cmd = GpsRxConfigMessage::CMD_STOP;
    baseband::gps_rx_config(m);
    job_ = JOB_NONE;
    awaiting_seq_ = false;
    results_ = nullptr;
    done_ = false;
    solve_pending_ = false;
}

int GpsView::progress_pct() const {
    if (!job_units_) return 0;
    const uint32_t p = status_.units * 100u / job_units_;
    return static_cast<int>(p > 99 ? 99 : p);
}

void GpsView::on_status(const GpsRxStatusMessage& m) {
    status_.state = m.state;
    status_.level = m.level;
    status_.confirming = m.confirming;
    status_.done_prn = m.done_prn;
    status_.total_prn = m.total_prn;
    status_.k_ms = m.k_ms;
    status_.rms_x10 = m.rms_x10;
    status_.clip_permille = m.clip_permille;
    status_.seq = m.seq;
    status_.units = m.units;
    status_.unit_cycles = m.unit_cycles;
    if (!mem_ && m.scratch && m.scratch_size >= sizeof(GpsScratch)) {
        mem_ = reinterpret_cast<GpsScratch*>(m.scratch);
        std::memset(static_cast<void*>(mem_), 0, sizeof(GpsScratch));
        if (load_wanted_) start_load();
    }
    if (awaiting_seq_ && m.state == GpsRxStatusMessage::CAPTURING) {
        awaiting_seq_ = false;
        job_seq_ = m.seq;
    }
    if (!awaiting_seq_ && job_ != JOB_NONE && m.seq == job_seq_ && m.state == GpsRxStatusMessage::DONE && !done_) {
        done_ = true;
        results_ = static_cast<const gps_rx::Result*>(m.results);
        if (job_ == JOB_FIX) solve_pending_ = true;
    }
    view_heard_.show();
    view_fix_.show();
}

/* Snapshot fix on the M0: detected PRNs with an ephemeris, the saved position
 * as the a priori, the RTC as the coarse time (~0.2-0.5 s of soft double). */
void GpsView::solve_fix() {
    solve_pending_ = false;
    fix_tried_ = true;
    fix_ok_ = false;
    fix_heard_ = 0;
    if (!mem_ || !results_ || !pos_valid_) return;
    Measurement* m = mem_->solve.in; /* off the stack */
    int n = 0;
    double newest = 0;
    for (int p = 0; p < 32; p++) {
        const auto& r = results_[p];
        const Orbit& o = mem_->orbits[p];
        if (!r.detected) continue;
        fix_heard_++;
        if (!o.valid || !o.ephemeris || n >= kMaxMeas) continue;
        m[n++] = {&o, r.code_phase, r.metric};
        if (o.toe > newest) newest = o.toe;
    }
    const double tg = now_gps();
    fix_ok_ = snapshot_fix(m, n, rx_, tg, fix_, mem_->solve);
    fix_fat_ = saved_position::now_fat();
    fix_eph_age_s_ = tg - newest;
}

void GpsView::on_frame() {
    /* the selected tab's content is refreshed when it becomes visible */
    const uint32_t sel = tab_view_.selected();
    if (sel != tab_shown_) {
        tab_shown_ = sel;
        if (sel == 0) view_fix_.show();
        if (sel == 1) view_heard_.show();
        if (sel == 2) {
            view_passes_.refresh();
            view_passes_.update_sky();
        }
    }
    if (solve_pending_) {
        solve_fix();
        view_fix_.show();
        return;
    }
    if (loader_.stage() != OrbitLoader::DONE && loader_.stage() != OrbitLoader::IDLE) {
        if (!loader_.step(*this)) { /* all files read */
            view_passes_.refresh();
            view_passes_.update_sky();
            view_passes_.restart_passes();
            view_fix_.show();
            view_heard_.show();
        }
        return; /* one job per frame */
    }
    view_passes_.frame();
}

/* Antenna power, read only (never switched by this app). */
void GpsView::show_antenna() {
    const int8_t on = portapack::get_antenna_bias() ? 1 : 0;
    if (on == bias_shown_) return;
    bias_shown_ = on;
    text_ant_.set(on ? "Ant: ON" : "Ant: off");
    text_warn_.set(on ? "ANT POWER 3.3V ON: active antenna only!" : "");
    text_warn_.set_style(Theme::getInstance()->fg_red);
}

void GpsView::on_tick_second() {
    show_antenna();
    view_passes_.frame(); /* the pass search also advances here (frames may be busy) */
    view_passes_.refresh();
    if (++sky_tick_ >= 10) {
        sky_tick_ = 0;
        view_passes_.update_sky();
    }
}

#else /* not PRALINE */

GpsView::GpsView(NavigationView& nav)
    : button_back_{{0, UI_POS_Y(4), UI_POS_WIDTH_PERCENT(50), UI_POS_HEIGHT(2)}, "Back"} {
    add_children({&text_, &button_back_});
    button_back_.on_select = [&nav](Button&) {
        nav.pop();
    };
}

void GpsView::focus() {
    button_back_.focus();
}

#endif

}  // namespace ui::external_app::gps
