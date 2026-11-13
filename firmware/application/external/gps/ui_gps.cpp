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
#include "gps_fmt.hpp"

#include <cmath>
#include <cstring>
#endif

using namespace portapack;

namespace ui::external_app::gps {

#ifdef PRALINE

namespace {

constexpr int kLoadLinesPerFrame = 60; /* a few KB of file per display frame */
constexpr double kSearchSpan = 86400.0;

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
    const int32_t week = static_cast<int32_t>(now_gps / kWeekS);
    yuma_.reset(week);
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
    Orbit o;
    if (stage_ == ALMANAC) {
        if (yuma_.line(line_, o)) {
            Orbit& slot = v.orbits_[o.prn - 1];
            if (!slot.valid || (!slot.ephemeris && std::fabs(o.toe - now_gps_) < std::fabs(slot.toe - now_gps_))) {
                if (!slot.valid) v.n_alm_++;
                slot = o;
                v.alm_week_ = static_cast<int32_t>(o.toe / kWeekS);
            }
        }
    } else if (stage_ == EPHEMERIS) {
        if (rinex_.line(line_, o) && std::fabs(o.toe - now_gps_) < 4 * 3600.0) {
            Orbit& slot = v.orbits_[o.prn - 1];
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
            if (v.n_tle_ < GpsView::kMaxTle) {
                Tle& t = v.tles_[v.n_tle_];
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
      text_head_{{0, 0, UI_POS_WIDTH(24), UI_POS_HEIGHT(3)}, head_, kRows, &ui::font::fixed_8x16},
      button_pos_{{UI_POS_X(24), 0, UI_POS_WIDTH(6), UI_POS_HEIGHT(1)}, "Pos"},
      button_load_{{UI_POS_X(24), UI_POS_Y(2), UI_POS_WIDTH(6), UI_POS_HEIGHT(1)}, "Load"},
      labels_off_{{{UI_POS_X(19), UI_POS_Y(1)}, "Off", Theme::getInstance()->fg_light->foreground}},
      field_off_{{UI_POS_X(23), UI_POS_Y(1)}, 4, {-720, 840}, 15, ' '},
      sky_{{0, UI_POS_Y(3) + 2, 136, 136}},
      text_side_{{140, UI_POS_Y(3) + 2, 100, 136}, side_, kSideRows, &ui::font::fixed_5x8},
      text_passes_{{0, UI_POS_Y(3) + 142, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kPassRows)}, pass_lines_, kPassRows, &ui::font::fixed_8x16} {
    add_children({&text_head_, &button_pos_, &button_load_, &labels_off_, &field_off_, &sky_, &text_side_, &text_passes_});
    field_off_.set_value(main_.utc_off_min_);
    field_off_.on_change = [this](int32_t v) {
        main_.utc_off_min_ = v;
        refresh();
        update_sky();
        restart_passes();
    };
    button_pos_.on_select = [this](Button&) {
        main_.nav_.push<PositionView>(main_);
    };
    button_load_.on_select = [this](Button&) {
        main_.start_load();
    };
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
            f.fixed(main_.pos_.lat_e7 / 1000, 4).c(' ').fixed(main_.pos_.lon_e7 / 1000, 4);
            text_head_.set(0, b, white);
        } else {
            text_head_.set(0, "No position: Pos", yellow);
        }
    }
    {
        Fmt f{b, sizeof(b)};
        const double t = main_.now_unix();
        const int32_t days = static_cast<int32_t>(std::floor(t / 86400.0));
        int32_t y, m, d;
        civil_from_days(days, y, m, d);
        const uint32_t sod = static_cast<uint32_t>(t - days * 86400.0);
        f.s("UTC ").d2(static_cast<uint32_t>(m)).c('-').d2(static_cast<uint32_t>(d)).c(' ').d2(sod / 3600).c(':').d2((sod / 60) % 60).c(':').d2(sod % 60);
        text_head_.set(1, b, grey);
    }
    {
        Fmt f{b, sizeof(b)};
        const auto st = main_.loader_.stage();
        if (st != OrbitLoader::DONE && st != OrbitLoader::IDLE) {
            f.s(st == OrbitLoader::ALMANAC ? "Reading ALMANAC" : (st == OrbitLoader::EPHEMERIS ? "Reading EPHEM" : "Reading TLE"));
            text_head_.set(2, b, yellow);
        } else if (!main_.n_alm_ && !main_.n_eph_ && !main_.n_tle_) {
            text_head_.set(2, "No /GPS orbit files", yellow);
        } else {
            f.s("Alm ").u(main_.n_alm_).s(" Eph ").u(main_.n_eph_).s(" TLE ").u(main_.n_tle_);
            text_head_.set(2, b, white);
        }
    }
}

void PassesView::update_sky() {
    sky_.clear();
    for (size_t i = 0; i < kSideRows; i++) text_side_.set(i, "", Color::white());
    if (!main_.pos_valid_) {
        sky_.set_dirty();
        text_side_.set(0, "Set a position", Color::yellow());
        return;
    }
    const double t = main_.now_unix();
    const double tg = unix_to_gps(t);
    /* GPS satellites above the horizon, highest first in the side list */
    Vis* vis = vis_;
    size_t nv = 0;
    for (size_t i = 0; i < 32; i++) {
        const Orbit& o = main_.orbits_[i];
        if (!o.valid || o.health) continue;
        double az, el;
        az_el(main_.rx_, sat_position(o, tg), az, el);
        if (el < 0) continue;
        vis[nv++] = {o.prn, static_cast<int16_t>(el), static_cast<int16_t>(az)};
    }
    for (size_t a = 1; a < nv; a++)
        for (size_t b = a; b > 0 && vis[b].el > vis[b - 1].el; b--) {
            const Vis tmp = vis[b];
            vis[b] = vis[b - 1];
            vis[b - 1] = tmp;
        }
    char b[TextLines::kCols + 1];
    {
        Fmt f{b, sizeof(b)};
        f.s("GPS up ").u(static_cast<uint32_t>(nv));
        text_side_.set(0, b, Color::light_grey());
    }
    for (size_t k = 0; k < nv; k++) {
        char lbl[4];
        Fmt{lbl, sizeof(lbl)}.u(vis[k].prn);
        sky_.add(vis[k].az, vis[k].el, lbl, Color::green(), true);
        if (k + 2 < kSideRows) {
            Fmt f{b, sizeof(b)};
            f.c('G').d2(vis[k].prn).s(" el").u(static_cast<uint32_t>(vis[k].el), 2).s(" az").u(static_cast<uint32_t>(vis[k].az), 3);
            text_side_.set(k + 1, b, Color::white());
        }
    }
    /* TLE satellites (orange) if up */
    for (size_t i = 0; i < main_.n_tle_; i++) {
        if (!sgp4_init(main_.tles_[i], sky_sgp4_)) continue;
        SatSource src;
        src.kind = SatSource::SGP4;
        src.sgp4 = &sky_sgp4_;
        double az = 0;
        const double el = elevation(src, main_.rx_, t, &az);
        if (el < 0) continue;
        char nm[9];
        tle_short(main_.tles_[i], nm, 4);
        sky_.add(static_cast<int16_t>(az), static_cast<int16_t>(el), nm, Color::orange(), false);
    }
    sky_.set_dirty();
}

void PassesView::restart_passes() {
    for (auto& ok : result_ok_) ok = false;
    search_idx_ = 0;
    searching_ = false;
    if (!main_.pos_valid_ || !main_.n_tle_) {
        show_passes();
        return;
    }
    if (sgp4_init(main_.tles_[0], sgp4_)) {
        SatSource src;
        src.kind = SatSource::SGP4;
        src.sgp4 = &sgp4_;
        finder_.start(src, main_.rx_, main_.now_unix(), kSearchSpan, 30.0);
        searching_ = true;
    } else {
        search_idx_ = 1;
        searching_ = search_idx_ < main_.n_tle_;
    }
    show_passes();
}

void PassesView::frame() {
    if (!searching_) return;
    if (!finder_.done()) {
        finder_.step(12);
        if (!finder_.done()) return;
    }
    if (search_idx_ < kPassRows) {
        result_ok_[search_idx_] = finder_.found();
        results_[search_idx_] = finder_.pass();
    }
    search_idx_++;
    while (search_idx_ < main_.n_tle_ && search_idx_ < kPassRows) {
        if (sgp4_init(main_.tles_[search_idx_], sgp4_)) {
            SatSource src;
            src.kind = SatSource::SGP4;
            src.sgp4 = &sgp4_;
            finder_.start(src, main_.rx_, main_.now_unix(), kSearchSpan, 30.0);
            show_passes();
            return;
        }
        search_idx_++;
    }
    searching_ = false;
    show_passes();
}

void PassesView::show_passes() {
    char b[TextLines::kCols + 1];
    const double now = main_.now_unix();
    for (size_t i = 0; i < kPassRows; i++) {
        if (i >= main_.n_tle_) {
            text_passes_.set(i, i == 0 && !main_.n_tle_ ? "No TLE (GPS/TLE.TXT)" : "", Color::light_grey());
            continue;
        }
        Fmt f{b, sizeof(b)};
        char nm[9];
        f.s(tle_short(main_.tles_[i], nm, sizeof(nm))).pad(9);
        if (!main_.pos_valid_) {
            f.s("no position");
        } else if (searching_ && i >= search_idx_) {
            f.s("...");
        } else if (!result_ok_[i]) {
            f.s("no pass 24h");
        } else {
            const Pass& p = results_[i];
            if (p.up_at_start)
                f.s("UP ");
            else {
                f.s("in ");
                put_span(f, p.rise - now);
                f.c(' ');
            }
            f.u(static_cast<uint32_t>(p.max_el < 0 ? 0 : p.max_el)).c('d').c(' ');
            if (p.set > 0) put_span(f, p.set - (p.up_at_start ? now : p.rise));
        }
        text_passes_.set(i, b, Color::white());
    }
}

/* ---------------------------------------------------------------- heard */

HeardView::HeardView(GpsView& main, Rect r)
    : View{r},
      main_{main},
      text_{{0, 0, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kRows)}, lines_, kRows, &ui::font::fixed_8x16},
      button_start_{{0, UI_POS_Y(kRows), UI_POS_WIDTH_PERCENT(50), UI_POS_HEIGHT(1)}, "Search"} {
    add_children({&text_, &button_start_});
    text_.set(0, "Satellite search: not in", Color::yellow());
    text_.set(1, "this build yet (stage 2).", Color::yellow());
}

void HeardView::focus() {
    button_start_.focus();
}

/* ---------------------------------------------------------------- fix */

FixView::FixView(GpsView& main, Rect r)
    : View{r},
      main_{main},
      field_method_{{UI_POS_X(0), 0}, 20, {{"Snapshot + eph file", 0}, {"Autonomous", 1}}},
      text_{{0, UI_POS_Y(1), UI_POS_MAXWIDTH, UI_POS_HEIGHT(kRows)}, lines_, kRows, &ui::font::fixed_8x16},
      button_save_{{0, UI_POS_Y(kRows + 1), UI_POS_WIDTH_PERCENT(50), UI_POS_HEIGHT(1)}, "Save pos"} {
    add_children({&field_method_, &text_, &button_save_});
    field_method_.set_selected_index(0);
    field_method_.on_change = [this](size_t, OptionsField::value_t) {
        show_method();
    };
    button_save_.on_select = [this](Button&) {
        text_.set(kRows - 1, "No fix to save yet", Color::yellow());
    };
    show_method();
}

void FixView::focus() {
    field_method_.focus();
}

void FixView::show_method() {
    for (size_t i = 0; i < kRows; i++) text_.set(i, "", Color::white());
    if (field_method_.selected_index_value() == 0) {
        text_.set(0, "Snapshot fix: not in", Color::yellow());
        text_.set(1, "this build yet (stage 3).", Color::yellow());
    } else {
        text_.set(0, "Autonomous: NOT AVAILABLE", Color::yellow());
        text_.set(1, "yet (tracking + nav data", Color::yellow());
        text_.set(2, "decoding not built).", Color::yellow());
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
           {"tab"sv, &tab_}}},
      view_fix_{*this, view_rect_},
      view_heard_{*this, view_rect_},
      view_passes_{*this, view_rect_},
      tab_view_{
          {"Fix", Theme::getInstance()->fg_cyan->foreground, &view_fix_},
          {"Heard", Theme::getInstance()->fg_green->foreground, &view_heard_},
          {"Passes", Theme::getInstance()->fg_yellow->foreground, &view_passes_}} {
    add_children({&labels_, &field_lna_, &field_vga_, &field_amp_, &check_bias_, &text_warn_,
                  &tab_view_, &view_fix_, &view_heard_, &view_passes_});
    tab_view_.set_parent_rect({0, UI_POS_Y(2), screen_width, 24});

    bias_was_ = portapack::get_antenna_bias();
    check_bias_.set_value(bias_was_);
    check_bias_.on_select = [this](Checkbox&, bool v) {
        portapack::set_antenna_bias(v);
        receiver_model.set_antenna_bias();
        text_warn_.set(v ? "ANT POWER 3.3V: active antenna only!" : "");
    };
    if (bias_was_) text_warn_.set("ANT POWER 3.3V: active antenna only!");

    load_position();
    start_load();
    view_passes_.refresh();
    tick_token_ = rtc_time::signal_tick_second += [this]() {
        on_tick_second();
    };
    tab_view_.set_selected(tab_ < 3 ? tab_ : 2);
}

GpsView::~GpsView() {
    rtc_time::signal_tick_second -= tick_token_;
    tab_ = tab_view_.selected();
    /* Antenna power back as it was before the app (the port may get another antenna). */
    if (portapack::get_antenna_bias() != bias_was_) portapack::set_antenna_bias(bias_was_);
}

void GpsView::focus() {
    tab_view_.focus();
}

double GpsView::now_unix() const {
    return static_cast<double>(rtc_time::rtcToUnixUTC(rtc_time::now())) - utc_off_min_ * 60.0;
}

void GpsView::load_position() {
    pos_valid_ = saved_position::load(pos_);
    if (pos_valid_) rx_ = lla_to_ecef(pos_.lat_e7 / 1e7, pos_.lon_e7 / 1e7, pos_.alt_m);
    view_passes_.refresh();
    view_passes_.update_sky();
    view_passes_.restart_passes();
}

void GpsView::start_load() {
    for (auto& o : orbits_) o = Orbit{};
    n_alm_ = n_eph_ = n_tle_ = 0;
    alm_week_ = 0;
    eph_toe_newest_ = 0;
    loader_.start(unix_to_gps(now_unix()));
    view_passes_.refresh();
}

void GpsView::on_frame() {
    if (loader_.stage() != OrbitLoader::DONE && loader_.stage() != OrbitLoader::IDLE) {
        if (!loader_.step(*this)) { /* all files read */
            view_passes_.refresh();
            view_passes_.update_sky();
            view_passes_.restart_passes();
        }
        return; /* one job per frame */
    }
    view_passes_.frame();
}

void GpsView::on_tick_second() {
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
