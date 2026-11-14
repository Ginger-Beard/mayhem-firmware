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
#include "gps_cache.hpp"
#include "gps_nav.hpp"

#include "portapack.hpp"
#include "rtc_time.hpp"
#include "theme.hpp"

#ifdef PRALINE
#include "baseband_api.hpp"
#include "gps_fmt.hpp"
#include "hal.h"

#include <cmath>
#include <cstring>
#include <new>
#endif

using namespace portapack;

namespace ui::external_app::gps {

#ifdef PRALINE

namespace {

/* UI-thread work per display frame: whole units (a file line, one satellite,
 * one elevation) until this much time is used. Each unit is ~0.03-1 ms of M0
 * work by the emulator count (rf_test/gps/m0bench), more on the device, so a
 * frame stays short and a finger tap (3 touch frames processed while pressed)
 * is not lost. */
constexpr systime_t kSliceTicks = MS2ST(6);
constexpr size_t kMaxLinesPerFrame = 200;
constexpr uint32_t kGeoPeriodS = 5; /* a new sky-cache pass every 5 s */
/* solver thread: snapshot_fix 152 + coarse_time_solve 296 + geometric_range 128
 * + sat_position 104 B (-fstack-usage) + libm and the exception frame */
constexpr size_t kSolverStack = 1536;
constexpr int kMinFixSats = 6; /* 5 unknowns + 1 to check them */
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
    used_ |= 1u << row;
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

void TextLines::set_highlight(int row) {
    const int8_t r = static_cast<int8_t>(row < 0 || row >= static_cast<int>(rows_) ? -1 : row);
    if (r == hl_) return;
    hl_ = r;
    set_dirty();
}

bool TextLines::on_touch(const TouchEvent event) {
    if (!on_row || event.type != TouchEvent::Type::End) return on_row != nullptr;
    const int y = event.point.y() - screen_rect().top();
    const int lh = font_->line_height();
    if (y >= 0 && y / lh < static_cast<int>(rows_)) on_row(static_cast<size_t>(y / lh));
    return true;
}

void BigText::set(const char* text, Color color) {
    if (std::strncmp(text_, text, sizeof(text_) - 1) == 0 && color_.v == color.v) return;
    std::strncpy(text_, text, sizeof(text_) - 1);
    color_ = color;
    set_dirty();
}

void BigText::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto bg = Theme::getInstance()->bg_darkest->background;
    const Style st{ui::font::fixed_8x16, bg, color_};
    int x = r.left();
    for (const char* c = text_; *c && x + 16 <= r.right(); c++) x += painter.draw_char({x, r.top()}, st, *c, 2);
    if (x < r.right()) painter.fill_rectangle({x, r.top(), r.right() - x, r.height()}, bg);
}

void TextLines::end() {
    const uint32_t used = used_;
    for (size_t i = 0; i < rows_; i++)
        if (!(used & (1u << i))) set(i, "", Color::white());
    used_ = used;
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
        const bool hl = static_cast<int>(i) == hl_; /* the selection: inverted */
        const Color fg = hl ? Color::black() : lines_[i].color, bk = hl ? lines_[i].color : bg;
        const int w = n ? painter.draw_string(p, *font_, fg, bk, std::string_view{lines_[i].text, n}) : 0;
        painter.fill_rectangle({p.x() + w, p.y(), r.width() - w, lh}, bk);
    }
}

bool SkyPlot::on_touch(const TouchEvent event) {
    if (event.type == TouchEvent::Type::Start) focus();
    return true;
}

void SkyPlot::add(int16_t az, int16_t el, const char* label, Color color, bool filled, bool selected) {
    if (n_ >= kMax) return;
    Dot& d = dots_[n_++];
    d.az = az;
    d.el = el;
    std::strncpy(d.label, label, sizeof(d.label) - 1);
    d.label[sizeof(d.label) - 1] = 0;
    d.color = color;
    d.filled = filled;
    d.sel = selected;
    hash_ = (hash_ ^ (selected ? 0x5a5a5a5au : 0u)) * 16777619u;
    const uint32_t v[4] = {static_cast<uint32_t>(az), static_cast<uint32_t>(el), color.v, static_cast<uint32_t>(filled) | (static_cast<uint32_t>(label[0]) << 8) | (static_cast<uint32_t>(label[0] ? label[1] : 0) << 16)};
    for (uint32_t x : v) hash_ = (hash_ ^ x) * 16777619u;
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
    if (has_focus()) painter.draw_rectangle(r, Theme::getInstance()->fg_yellow->foreground); /* the wheel selects here */
    if (track_ && track_->ready()) {                                                         /* the selected satellite's pass arc */
        for (int i = 0; i < track_->points(); i++) {
            const double rr = rad * (90.0 - track_->el(i)) / 90.0;
            const float a = track_->az(i) * static_cast<float>(kPi / 180.0);
            const int x = cx + static_cast<int>(rr * std::sin(a)), y = cy - static_cast<int>(rr * std::cos(a));
            painter.fill_rectangle({x - 1, y - 1, 2, 2}, Color::cyan());
        }
    }
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
        if (d.sel) painter.draw_rectangle({x - 5, y - 5, 11, 11}, Color::white());
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

bool OrbitLoader::open(Stage s, GpsView& v) {
    close();
    stage_ = s;
    rpos_ = rlen_ = llen_ = 0;
    line_no_ = 0;
    eof_ = false;
    tle_state_ = 0;
    air_prn_ = 0;
    const char16_t* name = s == ALMANAC ? u"GPS/ALMANAC.TXT"
                                        : (s == EPHEMERIS ? u"GPS/EPHEM.RNX" : (s == EPH_AIR ? u"GPS/EPH_AIR.TXT" : (s == SATFREQ ? u"GPS/SATFREQ.TXT" : u"GPS/TLE.TXT")));
    file_ = new File();
    if (file_->open(name).is_valid()) { /* an error */
        close();
        return false;
    }
    if (s != EPH_AIR) (s == ALMANAC ? v.alm_file_ : (s == EPHEMERIS ? v.eph_file_ : (s == SATFREQ ? v.freq_file_ : v.tle_file_))) = true;
    return true;
}

/* The files in order (the downlink table before the TLEs, which it ranks). */
bool OrbitLoader::open_after(Stage s, GpsView& v) {
    for (int k = static_cast<int>(s) + 1; k < static_cast<int>(DONE); k++)
        if (open(static_cast<Stage>(k), v)) return true;
    return false;
}

void OrbitLoader::start(double now_gps, GpsView& v) {
    now_gps_ = now_gps;
    yuma_.reset(static_cast<int32_t>(now_gps / kWeekS));
    rinex_.reset();
    stage_ = IDLE;
    if (!open_after(IDLE, v)) stage_ = DONE;
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
            line_no_++;
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
                v.mem_->alm[o.prn - 1] = to_alm(o);
                if (o.toe > v.alm_toa_) v.alm_toa_ = o.toe;
            }
        }
    } else if (stage_ == EPHEMERIS) {
        const bool rec = rinex_.line(line_, o);
        if (rec && o.toe > v.eph_file_newest_) v.eph_file_newest_ = o.toe;
        if (rec && std::fabs(o.toe - now_gps_) < GpsView::kEphMaxAgeS) {
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
    } else if (stage_ == EPH_AIR) {
        uint8_t prn, sfid;
        int32_t week;
        uint32_t tow, w[10];
        if (!eph_air_parse(line_, &prn, &week, &sfid, &tow, w)) return;
        if (prn != air_prn_ || sfid == 1) {
            air_prn_ = prn;
            v.mem_->cache_have = 0;
        }
        std::memcpy(v.mem_->cache_sf[sfid - 1], w, sizeof(w));
        v.mem_->cache_have |= static_cast<uint8_t>(1u << (sfid - 1));
        if (v.mem_->cache_have != 7) return;
        v.mem_->cache_have = 0;
        if (!nav::decode_ephemeris(v.mem_->cache_sf[0], v.mem_->cache_sf[1], v.mem_->cache_sf[2], prn, week, o)) return;
        if (o.toe > v.eph_file_newest_) v.eph_file_newest_ = o.toe;
        if (std::fabs(o.toe - now_gps_) >= GpsView::kEphMaxAgeS) return;
        Orbit& slot = v.mem_->orbits[o.prn - 1];
        const bool better = !slot.valid || !slot.ephemeris ||
                            (slot.health != 0 && o.health == 0) ||
                            (o.health == slot.health && std::fabs(o.toe - now_gps_) < std::fabs(slot.toe - now_gps_));
        if (better) {
            if (!slot.valid || !slot.ephemeris) v.n_eph_++;
            slot = o;
            if (o.toe > v.eph_toe_newest_) v.eph_toe_newest_ = o.toe;
        }
    } else if (stage_ == SATFREQ) {
        if (v.n_freq_ >= GpsScratch::kMaxFreq) return;
        const auto r = parse_satfreq(line_, v.mem_->freq[v.n_freq_]);
        if (r == FreqLine::OK) v.n_freq_++;
        if (r == FreqLine::BAD) {
            if (!v.freq_bad_) v.freq_bad_line_ = line_no_ ? line_no_ : 1;
            if (v.freq_bad_ < 255) v.freq_bad_++;
        }
    } else if (stage_ == TLES) {
        const size_t n = std::strlen(line_);
        if (line_[0] == '1' && line_[1] == ' ' && n >= 69) {
            std::strncpy(tle_l1_, line_, sizeof(tle_l1_) - 1);
            tle_state_ = 1;
        } else if (line_[0] == '2' && line_[1] == ' ' && n >= 69 && tle_state_ == 1) {
            tle_state_ = 0;
            take_tle(v);
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

/* A TLE (lines in tle_l1_ / line_): the first kMaxTle sets in file order,
 * except that a satellite listed in SATFREQ.TXT replaces one that is not, so
 * a whole CelesTrak group file can be copied as TLE.TXT. */
void OrbitLoader::take_tle(GpsView& v) {
    GpsScratch& m = *v.mem_;
    Tle& t = m.tle_tmp;
    if (!parse_tle(tle_l1_, line_, t)) return;
    std::strncpy(t.name, tle_name_, sizeof(t.name) - 1);
    if (!t.name[0]) Fmt{t.name, sizeof(t.name)}.u(t.catnr);
    uint8_t fi = GpsScratch::kNoFreq;
    for (uint8_t k = 0; k < v.n_freq_ && fi == GpsScratch::kNoFreq; k++)
        if (satfreq_matches(m.freq[k], t.catnr, t.name)) fi = k;
    for (uint8_t k = 0; k < v.n_tle_; k++)
        if (m.tles[k].catnr == t.catnr) return; /* a duplicate */
    size_t slot = v.n_tle_;
    if (slot >= GpsScratch::kMaxTle) {
        if (fi == GpsScratch::kNoFreq) return;
        for (slot = 0; slot < GpsScratch::kMaxTle && m.tle_freq[slot] != GpsScratch::kNoFreq; slot++) {
        }
        if (slot >= GpsScratch::kMaxTle) return; /* all slots hold table satellites */
    } else {
        v.n_tle_++;
    }
    m.tles[slot] = t;
    m.tle_freq[slot] = fi;
}

bool OrbitLoader::step(GpsView& v, systime_t t0) {
    if (stage_ == IDLE || stage_ == DONE || !file_) return false;
    for (size_t i = 0; i < kMaxLinesPerFrame && (i == 0 || chTimeNow() - t0 < kSliceTicks); i++) {
        if (!next_line()) {
            const Stage s = stage_;
            close();
            if (!open_after(s, v)) {
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
    sky_.set_focusable(true); /* the wheel steps through the satellites listed */
    text_passes_.on_row = [this](size_t r) {
        if (off_ + r < npass()) tap(GpsView::SEL_TLE, order_[off_ + r]);
    };
    text_side_.on_row = [this](size_t r) {
        if (r < kSideRows && side_prn_[r]) tap(GpsView::SEL_GPS, side_prn_[r] - 1);
    };
}

PassList* PassesView::pl() const {
    return main_.mem_ ? &main_.mem_->passes : nullptr;
}

size_t PassesView::npass() const {
    const PassList* p = pl();
    return p ? p->count() : 0;
}

int PassesView::pos_of_tle(uint8_t idx) const {
    for (size_t i = 0; i < npass(); i++)
        if (order_[i] == idx) return static_cast<int>(i);
    return -1;
}

int PassesView::items() const {
    int n = static_cast<int>(npass());
    for (size_t r = 0; r < kSideRows; r++) n += side_prn_[r] != 0;
    return n;
}

/* the wheel's items: every TLE satellite in list order (the window scrolls to
 * the selection), then the GPS side list */
void PassesView::item(int i, uint8_t& kind, uint8_t& idx) const {
    const int np = static_cast<int>(npass());
    if (i < np) {
        kind = GpsView::SEL_TLE;
        idx = order_[i];
        return;
    }
    int k = i - np;
    for (size_t r = 0; r < kSideRows; r++)
        if (side_prn_[r] && k-- == 0) {
            kind = GpsView::SEL_GPS;
            idx = side_prn_[r] - 1;
            return;
        }
    kind = GpsView::SEL_NONE;
    idx = 0;
}

bool PassesView::on_encoder(const EncoderEvent delta) {
    const int n = items();
    if (!n || !main_.mem_) return false;
    int cur = -1;
    for (int i = 0; i < n && cur < 0; i++) {
        uint8_t k, x;
        item(i, k, x);
        if (k == main_.sel_kind_ && x == main_.sel_idx_) cur = i;
    }
    const int nx = cur < 0 ? (delta > 0 ? 0 : n - 1) : ((cur + static_cast<int>(delta)) % n + n) % n;
    uint8_t k, x;
    item(nx, k, x);
    main_.select(k, x);
    return true;
}

bool PassesView::on_key(const KeyEvent key) {
    if (key != KeyEvent::Select || main_.sel_kind_ == GpsView::SEL_NONE) return false;
    main_.open_selected();
    return true;
}

/* first tap selects, a second tap on the same one opens it */
void PassesView::tap(uint8_t kind, uint8_t idx) {
    if (kind == main_.sel_kind_ && idx == main_.sel_idx_)
        main_.open_selected();
    else
        main_.select(kind, idx);
}

void PassesView::show_selection() {
    const bool tle = main_.sel_kind_ == GpsView::SEL_TLE, gps = main_.sel_kind_ == GpsView::SEL_GPS;
    const int pos = tle ? pos_of_tle(main_.sel_idx_) : -1;
    if (pos >= 0 && (pos < off_ || pos >= off_ + static_cast<int>(kPassRows))) {
        off_ = static_cast<uint8_t>(pos < off_ ? pos : pos - static_cast<int>(kPassRows) + 1);
        show_passes(); /* calls back here with the window moved */
        return;
    }
    text_passes_.set_highlight(pos >= 0 ? pos - off_ : -1);
    int row = -1;
    for (size_t r = 0; r < kSideRows && gps; r++)
        if (side_prn_[r] == main_.sel_idx_ + 1) row = static_cast<int>(r);
    text_side_.set_highlight(row);
}

void PassesView::zone_changed() {
    const int32_t h = field_zone_h_.value();
    const int32_t m = field_zone_m_.selected_index_value();
    main_.utc_off_min_ = h * 60 + (h < 0 ? -m : m);
    main_.geo_ready_ = false; /* the sky at the corrected time */
    main_.geo_restart();
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
            f.s(st == OrbitLoader::ALMANAC ? "Reading ALMANAC..." : (st == OrbitLoader::EPHEMERIS ? "Reading EPHEM..." : (st == OrbitLoader::SATFREQ ? "Reading SATFREQ..." : "Reading TLE...")));
            text_head_.set(3, b, yellow);
        } else if (!main_.n_alm_ && !main_.n_eph_ && !main_.n_tle_) {
            text_head_.set(3, "No orbit files (below)", yellow);
        } else {
            /* "Alm 31 3d Eph 0 TLE 5": counts, almanac age in days */
            const double age = main_.alm_age_days();
            f.s("Alm ").u(main_.n_alm_);
            if (age >= 0) f.c(' ').u(static_cast<uint32_t>(age + 0.5)).c('d');
            f.s(" Eph ").u(main_.n_eph_).s(" TLE ").u(main_.n_tle_);
            text_head_.set(3, b, age > kAlmMaxAgeDays ? yellow : white);
        }
    }
}

/* From the sky cache (GpsView::geo_step): drawing only, no orbit maths. */
void PassesView::update_sky() {
    if (hidden()) return; /* redrawn when the tab is selected */
    sky_.clear();
    text_side_.begin();
    for (auto& r : side_prn_) r = 0;
    const Color yellow = Color::yellow(), white = Color::white();
    if (!main_.mem_) {
        text_side_.set(0, "Starting...", yellow);
    } else if (!main_.pos_valid_) {
        text_side_.set(0, "No position yet:", yellow);
        text_side_.set(1, "press Pos (top right)", white);
        text_side_.set(2, "and enter your town", white);
        text_side_.set(3, "(100 km is close enough)", white);
    } else if (main_.loading()) {
        text_side_.set(0, "Reading the files...", yellow);
    } else if (!main_.n_alm_ && !main_.n_eph_) {
        text_side_.set(0, "No almanac on the card:", yellow);
        text_side_.set(1, "copy /GPS/ALMANAC.TXT", white);
        text_side_.set(2, "(see /GPS/README.TXT)", white);
    } else if (!main_.geo_ready_) {
        text_side_.set(0, "Computing the sky...", yellow);
    } else {
        char b[TextLines::kCols + 1];
        size_t nv = 0;
        for (size_t i = 0; i < 32; i++) {
            const Orbit& o = main_.mem_->orbits[i];
            const SatGeo& g = main_.mem_->geo[i];
            if (!o.valid || o.health || g.el < 0) continue;
            char lbl[4];
            Fmt{lbl, sizeof(lbl)}.u(o.prn);
            const bool sel = main_.sel_kind_ == GpsView::SEL_GPS && main_.sel_idx_ == i;
            sky_.add(static_cast<int16_t>(g.az), static_cast<int16_t>(g.el), lbl, Color::green(), true, sel);
            nv++;
            if (nv + 1 < kSideRows) {
                side_prn_[nv] = static_cast<uint8_t>(i + 1);
                Fmt f{b, sizeof(b)};
                f.c('G').d2(o.prn).s(" el").u(static_cast<uint32_t>(g.el), 2).s(" az").u(static_cast<uint32_t>(g.az), 3);
                text_side_.set(nv, b, white);
            }
        }
        Fmt f{b, sizeof(b)};
        f.s("GPS up now: ").u(static_cast<uint32_t>(nv));
        text_side_.set(0, b, Color::light_grey());
        for (size_t i = 0; i < main_.n_tle_; i++) { /* TLE satellites (orange) if up */
            const SatGeo& g = main_.mem_->tle_geo[i];
            if (g.el < 0) continue;
            char nm[9];
            tle_short(main_.mem_->tles[i], nm, 4);
            const bool sel = main_.sel_kind_ == GpsView::SEL_TLE && main_.sel_idx_ == i;
            sky_.add(static_cast<int16_t>(g.az), static_cast<int16_t>(g.el), nm, Color::orange(), false, sel);
        }
    }
    sky_.set_track(main_.mem_ && main_.sel_kind_ != GpsView::SEL_NONE ? &main_.mem_->track : nullptr);
    text_side_.end();
    show_selection();
    sky_.commit();
}

void PassesView::restart_passes() {
    PassList* p = pl();
    if (!p) return;
    p->stop();
    if (main_.pos_valid_ && main_.n_tle_)
        p->start(main_.mem_->tles, main_.n_tle_, &main_.mem_->search, main_.rx_, main_.now_unix());
    else
        p->start(nullptr, 0, nullptr, main_.rx_, 0); /* empty */
    show_passes();
}

/* The pass search in whole units (one elevation each) until the frame's time
 * slice is used. */
void PassesView::frame(systime_t t0) {
    PassList* p = pl();
    if (!p || !p->busy()) return;
    do {
        if (!p->step(1)) break;
    } while (chTimeNow() - t0 < kSliceTicks);
    show_passes();
}

void PassesView::tick() {
    if (!hidden()) show_passes();
}

void PassesView::show_passes() {
    char b[TextLines::kCols + 1];
    const double now = main_.now_unix();
    const Color white = Color::white(), yellow = Color::yellow();
    text_passes_.begin();
    const PassList* p = pl();
    if (p) p->order(order_);
    if (off_ + kPassRows > npass()) off_ = static_cast<uint8_t>(npass() > kPassRows ? npass() - kPassRows : 0);
    if (!main_.mem_ || main_.loading()) {
    } else if (!main_.tle_file_) {
        text_passes_.set(0, "No TLE file, so no passes:", yellow);
        text_passes_.set(1, "copy /GPS/TLE.TXT", white);
        text_passes_.set(2, "(see /GPS/README.TXT)", white);
    } else if (!main_.n_tle_) {
        text_passes_.set(0, "TLE.TXT has no usable sets", yellow);
        text_passes_.set(1, "(near-earth satellites,", white);
        text_passes_.set(2, "see /GPS/README.TXT)", white);
    } else if (!main_.pos_valid_) {
        text_passes_.set(0, "Passes need your position:", yellow);
        text_passes_.set(1, "press Pos (top right)", white);
    } else {
        /* oldest TLE: after two weeks pass times can be minutes off */
        double oldest_d = 0;
        const double jd_now = now / 86400.0 + 2440587.5;
        for (size_t i = 0; i < main_.n_tle_; i++) {
            const double d = jd_now - main_.mem_->tles[i].jd_epoch;
            if (d > oldest_d) oldest_d = d;
        }
        size_t rows = kPassRows;
        if (oldest_d > 14.0) {
            rows--;
            Fmt f{b, sizeof(b)};
            f.s("TLE ").u(static_cast<uint32_t>(oldest_d)).s(" d old: refresh (max 14)");
            text_passes_.set(rows, b, yellow);
        }
        for (size_t i = 0; i < rows && off_ + i < npass(); i++) {
            const uint8_t k = order_[off_ + i];
            p->line(k, now, b, sizeof(b));
            if (i == rows - 1 && off_ + rows < npass()) { /* more below: the wheel scrolls */
                size_t L = std::strlen(b);
                while (L < TextLines::kCols - 1) b[L++] = ' ';
                b[TextLines::kCols - 1] = '+';
                b[TextLines::kCols] = 0;
            }
            text_passes_.set(i, b, p->found(k) ? white : Color::light_grey());
        }
    }
    text_passes_.end();
    show_selection();
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
        if (main_.job_ != GpsView::JOB_AUTO) main_.stop_search(); /* Autonomous: stopped on the Fix tab */
        show();
    };
    field_depth_.set_selected_index(main_.depth_ ? 1 : 0, false);
    field_depth_.on_change = [this](size_t i, OptionsField::value_t) {
        main_.depth_ = static_cast<uint32_t>(i);
    };
    sky_.set_focusable(true); /* the wheel steps through the satellites listed */
    text_list_.on_row = [this](size_t r) {
        if (r >= kListRows || !list_prn_[r]) return;
        const uint8_t idx = list_prn_[r] - 1;
        if (main_.sel_kind_ == GpsView::SEL_GPS && main_.sel_idx_ == idx)
            main_.open_selected();
        else
            main_.select(GpsView::SEL_GPS, idx);
    };
    button_search_.on_select = [this](Button&) {
        if (main_.solving()) return; /* the fix's solver still uses the orbits */
        if (main_.job_ == GpsView::JOB_AUTO) return; /* the tracking loops run: Stop on the Fix tab first */
        main_.request_search(GpsView::JOB_HEARD, main_.depth_ ? 20 : 10);
        show();
    };
    show();
}

void HeardView::focus() {
    button_search_.focus();
}

bool HeardView::on_encoder(const EncoderEvent delta) {
    int rows[kListRows], n = 0, cur = -1;
    for (size_t r = 0; r < kListRows; r++)
        if (list_prn_[r]) {
            if (main_.sel_kind_ == GpsView::SEL_GPS && main_.sel_idx_ + 1 == list_prn_[r]) cur = n;
            rows[n++] = static_cast<int>(r);
        }
    if (!n) return false;
    const int nx = cur < 0 ? (delta > 0 ? 0 : n - 1) : ((cur + static_cast<int>(delta)) % n + n) % n;
    main_.select(GpsView::SEL_GPS, list_prn_[rows[nx]] - 1);
    return true;
}

bool HeardView::on_key(const KeyEvent key) {
    if (key != KeyEvent::Select || main_.sel_kind_ == GpsView::SEL_NONE) return false;
    main_.open_selected();
    return true;
}

/* While the tracking loops run (Autonomous): one row per channel. */
void HeardView::show_tracking() {
    char b[TextLines::kCols + 1];
    const Color white = Color::white(), yellow = Color::yellow(), green = Color::green(), grey = Color::grey();
    const AutoState& a = main_.mem_->autos;
    text_status_.begin();
    {
        Fmt f{b, sizeof(b)};
        f.s("Tracking ").u(main_.auto_tracked_).s(" (Fix tab: Stop)");
        text_status_.set(0, b, main_.auto_tracked_ >= 4 ? green : yellow);
    }
    if (main_.trk_sh_) {
        /* M4 load of the loops: cycles per 1 ms buffer / 2000 = percent */
        Fmt f{b, sizeof(b)};
        const uint32_t pct = main_.trk_sh_->cycles_avg / 2000u, mx = main_.trk_sh_->cycles_max / 2000u;
        f.s("M4 ").u(pct).s("% max ").u(mx).s("% missed ").u(main_.trk_sh_->missed);
        text_status_.set(1, b, mx > 95 ? Color::red() : grey);
    }
    text_status_.end();
    text_list_.begin();
    sky_.clear();
    for (auto& r : list_prn_) r = 0;
    text_list_.set_highlight(-1);
    text_list_.set(0, "PRN C/N0 lock bit TOW eph", grey);
    size_t row = 1;
    for (int i = 0; i < gps_trk::kMaxCh && row < kListRows; i++) {
        const gps_trk::ChInfo& c = a.ch[i];
        if (!c.prn || c.state == gps_trk::TRK_IDLE) continue;
        Fmt f{b, sizeof(b)};
        f.c('G').d2(c.prn).c(' ');
        Color col = white;
        if (c.state == gps_trk::TRK_SEARCH) {
            f.s("lost: searching");
            col = yellow;
        } else if (c.state == gps_trk::TRK_FAILED) {
            f.s("lost");
            col = grey;
        } else if (c.state == gps_trk::TRK_PULLIN) {
            f.s("pull-in");
            col = yellow;
        } else {
            f.u((c.cn0_x10 + 5u) / 10u).s("  ");
            f.s(c.flags & gps_trk::F_PLL ? " L" : " -");
            f.s(c.flags & gps_trk::F_BIT ? "   B" : "   -");
            f.s(c.flags & gps_trk::F_FRAME ? "   T" : "   -");
            const Orbit& o = main_.mem_->orbits[c.prn - 1];
            const bool eph = o.valid && o.ephemeris && std::fabs(main_.now_gps() - o.toe) <= GpsView::kEphMaxAgeS;
            f.s(eph ? "   E" : "   -");
            col = (c.flags & gps_trk::F_VERIFIED) ? (eph ? green : white) : yellow;
        }
        list_prn_[row] = c.prn;
        text_list_.set(row++, b, col);
    }
    if (row < kListRows - 2) {
        text_list_.set(row + 1, "L lock B bits T time", grey);
        text_list_.set(row + 2, "E orbit decoded (eph)", grey);
    }
    if (main_.pos_valid_ && main_.geo_ready_) {
        for (int i = 0; i < gps_trk::kMaxCh; i++) {
            const gps_trk::ChInfo& c = a.ch[i];
            if (!c.prn || c.state != gps_trk::TRK_TRACK) continue;
            const SatGeo& g = main_.mem_->geo[c.prn - 1];
            if (g.el < 0) continue;
            char lbl[4];
            Fmt{lbl, sizeof(lbl)}.u(c.prn);
            sky_.add(static_cast<int16_t>(g.az), static_cast<int16_t>(g.el), lbl, green, (c.flags & gps_trk::F_VERIFIED) != 0, false);
        }
    }
    text_list_.end();
    sky_.commit();
}

void HeardView::show() {
    if (hidden()) return; /* redrawn when the tab is selected (GpsView::on_frame) */
    if (main_.job_ == GpsView::JOB_AUTO && main_.tracking_ && main_.mem_) {
        show_tracking();
        return;
    }
    char b[TextLines::kCols + 1];
    const auto& st = main_.status_;
    const bool mine = main_.job_ == GpsView::JOB_HEARD || main_.job_ == GpsView::JOB_FIX || main_.job_ == GpsView::JOB_AUTO;
    const Color white = Color::white(), yellow = Color::yellow(), red = Color::red();
    int heard = 0;
    for (int p = 0; p < 32 && main_.done_ && main_.results_; p++) heard += main_.results_[p].detected;
    text_status_.begin();
    /* line 0: what is going on */
    {
        Fmt f{b, sizeof(b)};
        Color c = white;
        if (!main_.mem_) {
            f.s("Baseband starting...");
            c = yellow;
        } else if (main_.want_job_ != GpsView::JOB_NONE) {
            f.s("Preparing: sky first...");
            c = yellow;
        } else if (!mine) {
            f.s(main_.assisted_possible() ? "Search: assisted, then full" : "Search: all 32 (slow)");
            c = Color::light_grey();
        } else if (!main_.done_) {
            const char* kind = main_.search_kind_ == GpsView::SK_ASSISTED ? "Assisted " : (main_.search_kind_ == GpsView::SK_FULL_AFTER ? "None: full " : "Full ");
            if (st.state == GpsRxStatusMessage::CAPTURING || main_.awaiting_seq_)
                f.s(kind).s(st.confirming ? "confirm..." : "capture...");
            else if (st.confirming)
                f.s(kind).s("confirm ").u(st.done_prn).c('/').u(st.total_prn);
            else
                f.s(kind).u(st.done_prn).c('/').u(st.total_prn).c(' ').u(static_cast<uint32_t>(main_.progress_pct())).c('%');
            c = yellow;
        } else if (st.level == GpsRxStatusMessage::LEVEL_LOW) {
            f.s("Signal too low: no search");
            c = red;
        } else {
            const char* kind = main_.search_kind_ == GpsView::SK_ASSISTED ? " assisted" : " full search";
            if (heard < kMinFixSats) {
                f.s("Heard ").u(static_cast<uint32_t>(heard)).s(" of ").u(kMinFixSats).c(',').s(kind);
                c = yellow;
            } else {
                f.s("Heard ").u(static_cast<uint32_t>(heard)).c(',').s(kind);
                c = Color::green();
            }
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
            c = red;
        } else if (st.level == GpsRxStatusMessage::LEVEL_HIGH || st.clip_permille > 10) {
            f.s(" CLIP gain-");
            c = red;
        } else if (st.rms_x10 < 40) {
            f.s(" low");
            c = yellow;
        } else {
            f.s(" ok");
        }
        if (st.unit_cycles) f.s(" ").fixed(static_cast<int32_t>(st.unit_cycles / 20000u), 1).s("ms"); /* M4 time of one work unit (was shown 10x too small) */
        text_status_.set(1, b, c);
    }
    text_status_.end();

    /* list (23 columns): detected PRNs by C/N0 with a bar; guidance otherwise */
    text_list_.begin();
    sky_.clear();
    for (auto& r : list_prn_) r = 0;
    text_list_.set_highlight(-1);
    if (!mine || !main_.done_ || !main_.results_) {
        if (main_.mem_ && !mine && main_.want_job_ == GpsView::JOB_NONE && !main_.loading() && !main_.assisted_possible()) {
            if (!main_.pos_valid_) {
                text_list_.set(0, "No position: Search", yellow);
                text_list_.set(1, "tries all 32 (slow).", white);
                text_list_.set(2, "Set Pos on Passes tab", white);
            } else {
                text_list_.set(0, "No almanac: Search", yellow);
                text_list_.set(1, "tries all 32 (slow).", white);
                text_list_.set(2, "Copy /GPS/ALMANAC.TXT", white);
                text_list_.set(3, "(see /GPS/README.TXT)", white);
            }
        }
        text_list_.end();
        sky_.commit();
        return;
    }
    if (st.level == GpsRxStatusMessage::LEVEL_LOW) {
        text_list_.set(0, "Input too low to trust:", red);
        text_list_.set(1, "the ADC barely moves.", white);
        text_list_.set(3, "- raise LNA / VGA", white);
        text_list_.set(4, "  (start at 40 / 46)", white);
        text_list_.set(5, "- active antenna? Ant", white);
        text_list_.set(6, "  power on (status bar)", white);
        text_list_.set(7, "- target 4-30 LSB", white);
        text_list_.end();
        sky_.commit();
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
    if (!n) {
        text_list_.set(0, "No satellites heard.", yellow);
        text_list_.set(2, "- active antenna? Ant", white);
        text_list_.set(3, "  power on (status bar)", white);
        text_list_.set(4, "- open sky, outdoors", white);
        text_list_.set(5, "- level 4-30 LSB:", white);
        text_list_.set(6, "  adjust LNA / VGA", white);
        text_list_.set(7, "- Deep 20ms", white);
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
        text_list_.set(k, b, r.cn0 >= 38 ? Color::green() : white);
        list_prn_[k] = static_cast<uint8_t>(order[k] + 1);
        if (main_.sel_kind_ == GpsView::SEL_GPS && main_.sel_idx_ == order[k]) text_list_.set_highlight(static_cast<int>(k));
    }
    /* seen in one snapshot only: shown grey with '?', not counted as heard */
    size_t row = n ? n : 9;
    const size_t last = kListRows - 3; /* the bottom three rows: the level hint */
    for (int p = 0; p < 32 && row < last; p++) {
        const auto& r = main_.results_[p];
        if (!r.once) continue;
        Fmt f{b, sizeof(b)};
        f.c('G').d2(r.prn).s("? once, not confirmed");
        list_prn_[row] = static_cast<uint8_t>(p + 1);
        if (main_.sel_kind_ == GpsView::SEL_GPS && main_.sel_idx_ == p) text_list_.set_highlight(static_cast<int>(row));
        text_list_.set(row++, b, Color::grey());
    }
    /* level below the target (often gains carried over from an older version
     * or another app): say so with the gains in use, even when some heard */
    if (st.rms_x10 && st.rms_x10 < 40) {
        Fmt f{b, sizeof(b)};
        f.s("Level ").fixed(st.rms_x10, 1).s(": target 4-30");
        text_list_.set(last, b, yellow);
        Fmt g{b, sizeof(b)};
        g.s("raise gain: LNA ").u(static_cast<uint32_t>(receiver_model.lna())).s(" VGA ").u(static_cast<uint32_t>(receiver_model.vga()));
        text_list_.set(last + 1, b, white);
        text_list_.set(last + 2, "(start at 40 / 46)", white);
    }
    /* sky plot from the sky cache: heard filled green, others grey */
    if (main_.mem_ && main_.pos_valid_ && main_.geo_ready_) {
        for (int p = 0; p < 32; p++) {
            const Orbit& o = main_.mem_->orbits[p];
            const SatGeo& g = main_.mem_->geo[p];
            if (!o.valid || o.health || g.el < 0) continue;
            char lbl[4];
            Fmt{lbl, sizeof(lbl)}.u(o.prn);
            const bool h = main_.results_[p].detected;
            const bool sel = main_.sel_kind_ == GpsView::SEL_GPS && main_.sel_idx_ == p;
            sky_.add(static_cast<int16_t>(g.az), static_cast<int16_t>(g.el), lbl, h ? Color::green() : Color::grey(), h, sel);
        }
        sky_.set_track(main_.sel_kind_ == GpsView::SEL_GPS ? &main_.mem_->track : nullptr);
    }
    text_list_.end();
    sky_.commit();
}

/* ---------------------------------------------------------------- fix */

FixView::FixView(GpsView& main, Rect r)
    : View{r},
      main_{main},
      field_method_{{UI_POS_X(0), 0}, 10, {{"Snapshot", 0}, {"Autonomous", 1}}},
      button_fix_{{UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(14), UI_POS_HEIGHT(1)}, "Get fix"},
      button_save_{{UI_POS_X(16), UI_POS_Y(1), UI_POS_WIDTH(14), UI_POS_HEIGHT(1)}, "Save pos"},
      text_{{0, UI_POS_Y(2) + 4, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kRows)}, lines_, kRows, &ui::font::fixed_8x16} {
    add_children({&field_method_, &check_log_, &button_fix_, &button_save_, &text_});
    check_log_.set_value(main_.log_on_ != 0);
    check_log_.on_select = [this](Checkbox&, bool v) {
        main_.log_on_ = v ? 1 : 0; /* the file opens / closes in the frame handler */
    };
    field_method_.set_selected_index(0, false);
    field_method_.on_change = [this](size_t, OptionsField::value_t) {
        show();
    };
    button_fix_.on_select = [this](Button&) {
        if (field_method_.selected_index_value() != 0) { /* Autonomous */
            if (main_.job_ == GpsView::JOB_AUTO || main_.want_job_ == GpsView::JOB_AUTO) {
                main_.stop_auto();
                show();
                return;
            }
            save_msg_ = nullptr;
            if (!main_.mem_ || main_.loading()) {
                show();
                return;
            }
            main_.fix_tried_ = false;
            main_.fix_ok_ = false;
            main_.ttff_s_ = main_.ttff_coarse_s_ = 0;
            main_.auto_t0_ = chTimeNow();
            auto_reset(main_.mem_->autos);
            main_.request_search(GpsView::JOB_AUTO, 20);
            show();
            return;
        }
        /* the button reads "Stop" while a search or the solver runs */
        if (main_.searching() || main_.want_job_ == GpsView::JOB_FIX) {
            main_.want_job_ = GpsView::JOB_NONE;
            main_.stop_search();
            show();
            return;
        }
        if (main_.solving()) {
            if (main_.mem_) main_.mem_->solve.cancel = 1;
            show();
            return;
        }
        save_msg_ = nullptr;
        if (main_.fix_blocker() != GpsView::BLOCK_NONE) { /* the reason is on screen */
            show();
            return;
        }
        main_.fix_tried_ = false;
        main_.fix_ok_ = false;
        main_.request_search(GpsView::JOB_FIX, 20);
        show();
    };
    button_save_.on_select = [this](Button&) {
        if (!main_.fix_ok_) {
            save_msg_ = "No fix to save yet";
            save_color_ = Color::yellow();
            show();
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
        p.acc_m = main_.fix_est_m_; /* coarse (almanac) fixes: hundreds of m to km */
        if (saved_position::save(p)) {
            main_.load_position();
            save_msg_ = main_.fix_mode_ == GpsView::FIX_COARSE ? "Saved (coarse) for all apps" : "Saved for all apps";
            save_color_ = Color::green();
        } else {
            save_msg_ = "SD card write failed";
            save_color_ = Color::red();
        }
        show();
    };
    show();
}

void FixView::focus() {
    button_fix_.focus();
}

namespace {
/* "640 m" / "5.4 km" */
void put_dist(Fmt& f, uint32_t m) {
    if (m < 1000)
        f.u(m).s(" m");
    else
        f.fixed(static_cast<int32_t>((m + 50) / 100), 1).s(" km");
}
}  // namespace

/* Rows from `row` on (up to four): why a fix cannot be tried, or what the
 * next one will use. Plain words, 30 columns. */
void FixView::show_blocker(size_t row) {
    char b[TextLines::kCols + 1];
    const Color white = Color::white(), yellow = Color::yellow(), grey = Color::light_grey();
    switch (main_.fix_blocker()) {
        case GpsView::BLOCK_STARTING:
            text_.set(row, "Starting...", yellow);
            return;
        case GpsView::BLOCK_LOADING:
            text_.set(row, "Reading the orbit files...", yellow);
            return;
        case GpsView::BLOCK_NO_POS:
            text_.set(row, "No rough position set yet:", yellow);
            text_.set(row + 1, "Passes tab > Pos; your town", white);
            text_.set(row + 2, "is close enough (100 km).", white);
            return;
        case GpsView::BLOCK_NO_ORBITS:
            text_.set(row, "No orbit file on the card:", yellow);
            text_.set(row + 1, "/GPS/ALMANAC.TXT is missing.", white);
            text_.set(row + 2, "Copy the SD files again", white);
            text_.set(row + 3, "(see /GPS/README.TXT).", white);
            return;
        case GpsView::BLOCK_ALM_OLD: {
            Fmt f{b, sizeof(b)};
            f.s("Almanac is ").u(static_cast<uint32_t>(main_.alm_age_days())).s(" d old, max ").u(static_cast<uint32_t>(kAlmMaxAgeDays)).c(':');
            text_.set(row, b, yellow);
            text_.set(row + 1, "too old for a fix. Copy a", white);
            text_.set(row + 2, "new /GPS/ALMANAC.TXT", white);
            text_.set(row + 3, "(see /GPS/README.TXT).", white);
            return;
        }
        case GpsView::BLOCK_NONE:
            break;
    }
    /* what the next fix uses: fresh ephemerides (precise), else the almanac */
    const double tg = main_.now_gps();
    int up = 0, up_eph = 0;
    if (main_.mem_ && main_.geo_ready_)
        for (int p = 0; p < 32; p++) {
            const Orbit& o = main_.mem_->orbits[p];
            if (!o.valid || o.health || main_.mem_->geo[p].el <= kMinFixEl) continue;
            up++;
            up_eph += o.ephemeris;
        }
    if (main_.n_eph_ && up_eph >= kMinFixSats && up_eph == up) {
        Fmt f{b, sizeof(b)};
        f.s("Precise: ephemeris ");
        put_span(f, std::fabs(tg - main_.eph_toe_newest_));
        f.s(" old");
        text_.set(row, b, grey);
    } else {
        Fmt f{b, sizeof(b)};
        f.s("Coarse fix: almanac ").u(static_cast<uint32_t>(main_.alm_age_days() + 0.5)).s(" d old");
        text_.set(row, b, grey);
        { /* the almanac gives fixes for kAlmMaxAgeDays: replace it about monthly */
            static const char* const kMon[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
            int32_t y, mo, d;
            civil_from_days(static_cast<int32_t>(std::floor(gps_to_unix(main_.alm_toa_ + kAlmMaxAgeDays * 86400.0) / 86400.0)), y, mo, d);
            Fmt g{b, sizeof(b)};
            g.s("until ").u(static_cast<uint32_t>(d)).c(' ').s(kMon[(mo - 1) % 12]).s("; refresh monthly");
            text_.set(row + 1, b, main_.alm_age_days() > 21 ? yellow : grey);
        }
        if (main_.eph_file_ && main_.eph_file_newest_ > 0 && !main_.n_eph_) {
            Fmt g{b, sizeof(b)};
            const double age = tg - main_.eph_file_newest_;
            if (age > 0) {
                g.s("EPHEM.RNX ");
                put_span(g, age);
                g.s(" old (max 4h)");
            } else {
                g.s("EPHEM.RNX is for a later day");
            }
            text_.set(row + 2, b, yellow);
        } else {
            text_.set(row + 2, "Optional EPHEM.RNX: ~20 m", grey);
        }
    }
    if (main_.geo_ready_) {
        Fmt f{b, sizeof(b)};
        f.s("Sats up now: ").u(static_cast<uint32_t>(up)).s(" (").u(kMinFixSats).s(" needed)");
        text_.set(row + 3, b, up >= kMinFixSats ? grey : yellow);
    }
}

/* Autonomous: search -> coarse (almanac) -> upgrading (tracking, decoding
 * ephemerides) -> precise; plain words, 30 columns. */
void FixView::show_auto() {
    char b[TextLines::kCols + 1];
    const Color white = Color::white(), yellow = Color::yellow(), green = Color::green(), grey = Color::light_grey();
    const auto& st = main_.status_;
    const bool running = main_.job_ == GpsView::JOB_AUTO;
    if (!running && main_.want_job_ != GpsView::JOB_AUTO && !main_.fix_ok_) {
        text_.set(0, "Autonomous: decodes the orbits", white);
        text_.set(1, "from the satellites, no files", white);
        text_.set(2, "needed. Get fix: search, a", white);
        text_.set(3, "coarse fix from the almanac,", white);
        text_.set(4, "then 'upgrading' while it", white);
        text_.set(5, "tracks; 'precise' with 4", white);
        text_.set(6, "ephemerides (30 s to 1 min", white);
        text_.set(7, "of clean signal). Open sky,", white);
        text_.set(8, "active antenna, Ant power on.", white);
        if (main_.fix_tried_ && main_.status_.level == GpsRxStatusMessage::LEVEL_LOW) text_.set(9, "Last try: signal too low.", Color::red());
        return;
    }
    /* row 0: the phase */
    {
        Fmt f{b, sizeof(b)};
        Color c = yellow;
        if (main_.want_job_ == GpsView::JOB_AUTO) {
            f.s("Preparing: sky first...");
        } else if (running && !main_.done_) {
            if (st.state == GpsRxStatusMessage::CAPTURING || main_.awaiting_seq_)
                f.s(st.confirming ? "Confirming: capture" : "Capturing 20 ms...");
            else if (st.confirming)
                f.s("Confirming ").u(st.done_prn).c('/').u(st.total_prn);
            else
                f.s(st.state == GpsRxStatusMessage::REFINING ? "Refining " : (main_.search_kind_ == GpsView::SK_FULL_AFTER ? "Full search " : "Searching ")).u(st.done_prn).c('/').u(st.total_prn).c(' ').u(static_cast<uint32_t>(main_.progress_pct())).c('%');
        } else if (running && st.level == GpsRxStatusMessage::LEVEL_LOW) {
            f.s("Signal too low: no search.");
            c = Color::red();
        } else if (running && !main_.tracking_) {
            int heard = 0;
            for (int p = 0; p < 32 && main_.results_; p++) heard += main_.results_[p].detected;
            if (heard)
                f.s("Starting the tracking loops");
            else
                f.s("Heard none: Stop; Heard tips");
        } else if (main_.fix_ok_ && main_.fix_mode_ == GpsView::FIX_PRECISE) {
            f.s("PRECISE");
            if (main_.ttff_s_) f.s(", first fix ").u(main_.ttff_s_).s(" s");
            c = green;
        } else if (main_.auto_eph_ < 4 && main_.tracking_) {
            f.s("Upgrading: ").u(main_.auto_eph_).s(" of 4 ephemerides");
        } else if (main_.tracking_) {
            f.s("Upgrading: solving...");
        } else {
            f.s("Stopped");
            c = grey;
        }
        text_.set(0, b, c);
    }
    if (main_.fix_ok_) {
        const Solution& s = main_.fix_;
        double lat, lon, h;
        ecef_to_lla(s.pos, lat, lon, h);
        const bool coarse = main_.fix_mode_ == GpsView::FIX_COARSE;
        const Color c = coarse ? yellow : green;
        Fmt f{b, sizeof(b)};
        f.s("Lat ");
        put_deg(f, static_cast<int32_t>(std::lround(lat * 1e7)));
        text_.set(1, b, c);
        Fmt g{b, sizeof(b)};
        g.s("Lon ");
        put_deg(g, static_cast<int32_t>(std::lround(lon * 1e7)));
        text_.set(2, b, c);
        Fmt e{b, sizeof(b)};
        if (coarse) {
            e.s("COARSE (almanac), est. +/-");
            put_dist(e, main_.fix_est_m_);
        } else {
            e.s("Err est. ~").u(main_.fix_est_m_).s(" m Alt ").d(static_cast<int32_t>(std::lround(h))).s(" m");
        }
        text_.set(3, b, coarse ? yellow : white);
    }
    /* satellites */
    if (main_.tracking_) {
        Fmt f{b, sizeof(b)};
        f.s("Tracked ").u(main_.auto_tracked_).s(", TOW ").u(main_.auto_synced_).s(", eph ").u(main_.auto_eph_);
        text_.set(4, b, white);
        if (main_.fix_ok_) {
            Fmt g{b, sizeof(b)};
            g.s("Used ").u(main_.fix_.used).s(" HDOP ").fixed(static_cast<int32_t>(main_.fix_.hdop * 10), 1);
            if (main_.mem_ && main_.mem_->pvt_fix.ok && main_.mem_->pvt_fix.checked) g.s(" checked");
            text_.set(5, b, white);
        }
    }
    /* GPS time from the satellites */
    if (main_.gps_time_ok_) {
        const double u = gps_to_unix(main_.now_gps());
        const int64_t t = static_cast<int64_t>(std::floor(u));
        const int32_t sod = static_cast<int32_t>(((t % 86400) + 86400) % 86400);
        Fmt f{b, sizeof(b)};
        f.s("GPS time ").d2(sod / 3600).c(':').d2(sod / 60 % 60).c(':').d2(sod % 60).s(" UTC");
        text_.set(6, b, grey);
    }
    if (main_.mem_ && main_.mem_->pvt_fix.ok && main_.mem_->pvt_fix.vel_ok) {
        const PvtSolution& p = main_.mem_->pvt_fix;
        Fmt f{b, sizeof(b)};
        const int32_t ppb = static_cast<int32_t>(std::lround(p.drift_mps / kC * 1e9));
        f.s("TCXO ");
        if (ppb >= 0) f.c('+');
        f.fixed(ppb / 10, 2).s(" ppm, speed ").fixed(static_cast<int32_t>(norm(p.vel) * 10), 1).s(" m/s");
        text_.set(7, b, grey);
    }
    if (main_.cache_written_) {
        Fmt f{b, sizeof(b)};
        f.s("Saved ").u(main_.cache_written_).s(" eph to /GPS/EPH_AIR");
        text_.set(8, b, grey);
    }
    if (main_.fix_ok_ && main_.fix_mode_ == GpsView::FIX_COARSE && main_.tracking_)
        text_.set(9, "Keep it outdoors: upgrading", grey);
    else if (!main_.fix_ok_ && main_.fix_tried_ && main_.tracking_)
        text_.set(9, "No coarse fix: clock/Pos off?", yellow); /* the snapshot solve was refused; the PVT follows */
}

void FixView::show() {
    if (hidden()) return; /* redrawn when the tab is selected (GpsView::on_frame) */
    char b[TextLines::kCols + 1];
    const bool busy = field_method_.selected_index_value() != 0
                          ? (main_.job_ == GpsView::JOB_AUTO || main_.want_job_ == GpsView::JOB_AUTO)
                          : (main_.searching() || main_.solving() || main_.want_job_ == GpsView::JOB_FIX);
    if (busy != fix_running_shown_) {
        fix_running_shown_ = busy;
        button_fix_.set_text(busy ? "Stop" : "Get fix");
    }
    const Color white = Color::white(), yellow = Color::yellow();
    text_.begin();
    if (field_method_.selected_index_value() != 0) {
        show_auto();
        show_log_error();
        text_.end();
        return;
    }
    const auto& st = main_.status_;
    bool result = false;
    if (main_.want_job_ == GpsView::JOB_FIX) {
        text_.set(0, "Preparing: sky first...", yellow);
    } else if (main_.job_ == GpsView::JOB_FIX && !main_.done_) {
        Fmt f{b, sizeof(b)};
        if (st.state == GpsRxStatusMessage::CAPTURING || main_.awaiting_seq_)
            f.s(st.confirming ? "Confirming: capture" : "Capturing 20 ms...");
        else if (st.confirming)
            f.s("Confirming ").u(st.done_prn).c('/').u(st.total_prn);
        else
            f.s(st.state == GpsRxStatusMessage::REFINING ? "Refining " : (main_.search_kind_ == GpsView::SK_FULL_AFTER ? "Full search " : "Searching ")).u(st.done_prn).c('/').u(st.total_prn).c(' ').u(static_cast<uint32_t>(main_.progress_pct())).c('%');
        text_.set(0, b, yellow);
    } else if (main_.solving() || main_.solve_pending_) {
        text_.set(0, "Solving... (a few seconds;", yellow);
        text_.set(1, "the screen stays usable)", white);
    } else if (main_.fix_ok_) {
        result = true;
        const Solution& s = main_.fix_;
        double lat, lon, h;
        ecef_to_lla(s.pos, lat, lon, h);
        const bool coarse = main_.fix_mode_ == GpsView::FIX_COARSE;
        const Color c = coarse ? yellow : Color::green();
        {
            Fmt f{b, sizeof(b)};
            f.s("Lat ");
            put_deg(f, static_cast<int32_t>(std::lround(lat * 1e7)));
            text_.set(0, b, c);
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Lon ");
            put_deg(f, static_cast<int32_t>(std::lround(lon * 1e7)));
            text_.set(1, b, c);
        }
        if (coarse) {
            text_.set(2, "COARSE fix from the almanac", yellow);
            Fmt f{b, sizeof(b)};
            f.s("almanac ").u(static_cast<uint32_t>(main_.fix_alm_age_d_ + 0.5)).s(" d, est. +/-");
            put_dist(f, main_.fix_est_m_);
            text_.set(3, b, white);
        } else {
            Fmt f{b, sizeof(b)};
            f.s("Alt ").d(static_cast<int32_t>(std::lround(h))).s(" m (rough)");
            text_.set(2, b, white);
            Fmt g{b, sizeof(b)};
            g.s("Err est. ~").u(main_.fix_est_m_).s(" m HDOP ").fixed(static_cast<int32_t>(s.hdop * 10), 1);
            text_.set(3, b, white);
        }
        {
            Fmt f{b, sizeof(b)};
            f.s("Sats ").u(s.used).s(" used, ").u(main_.fix_heard_).s(" heard");
            text_.set(4, b, white);
        }
        {
            Fmt f{b, sizeof(b)};
            if (coarse) {
                f.s("Resid ").u(static_cast<uint32_t>(s.resid_rms_m + 0.5)).s(" m, clock ");
            } else {
                f.s("Eph ");
                put_span(f, std::fabs(main_.fix_eph_age_s_));
                f.s(" old, resid ").fixed(static_cast<int32_t>(s.resid_rms_m * 10), 1).s(" m");
            }
            if (coarse) f.d(static_cast<int32_t>(std::lround(s.time_corr_s))).c('s');
            text_.set(5, b, white);
        }
        if (coarse) {
            text_.set(7, "Same-day /GPS/EPHEM.RNX", Color::light_grey());
            text_.set(8, "tightens it to ~20 m", Color::light_grey());
            text_.set(9, "(optional, /GPS/README.TXT)", Color::light_grey());
        }
    } else if (main_.done_ && main_.job_ == GpsView::JOB_FIX && st.level == GpsRxStatusMessage::LEVEL_LOW) {
        text_.set(0, "Signal too low: no search.", Color::red());
        text_.set(1, "Raise LNA / VGA (40 / 46);", white);
        text_.set(2, "active antenna: Ant power", white);
        text_.set(3, "on (status bar).", white);
    } else if (main_.fix_tried_) {
        Fmt f{b, sizeof(b)};
        if (main_.fix_cand_ < kMinFixSats) {
            f.s("No fix: heard ").u(main_.fix_cand_).s(" of ").u(kMinFixSats).s(" needed");
            text_.set(0, b, yellow);
            text_.set(1, "Outdoors with open sky; see", white);
            text_.set(2, "Heard for the signal level.", white);
        } else {
            f.s("No fix: the ").u(main_.fix_cand_).s(" heard disagree");
            text_.set(0, b, yellow);
            text_.set(1, "(weak or false peaks, or the", white);
            text_.set(2, "clock is off by over 30 s).", white);
            text_.set(3, "Try again; Zone/clock: Passes", white);
        }
    }
    if (!result) show_blocker(6);
    if (save_msg_) text_.set(kRows - 1, save_msg_, save_color_);
    show_log_error();
    text_.end();
}

/* a card problem with the diagnostics log: the bottom row, red (never halts) */
void FixView::show_log_error() {
    if (!main_.log_on_ || !main_.log_err_ || save_msg_) return;
    char b[TextLines::kCols + 1];
    Fmt f{b, sizeof(b)};
    f.s("LOG ERROR: ").s(main_.log_err_).s(" (").u(main_.log_errs_).c(')');
    text_.set(kRows - 1, b, Color::red());
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

/* ---------------------------------------------------------------- satellite */

SatView::SatView(NavigationView&, GpsView& main)
    : main_{main},
      text_top_{{0, 0, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kTopRows)}, top_, kTopRows, &ui::font::fixed_8x16},
      big_dir_{{0, UI_POS_Y(2), UI_POS_MAXWIDTH, 32}},
      big_el_{{0, UI_POS_Y(2) + 32, UI_POS_MAXWIDTH, 32}},
      text_now_{{0, UI_POS_Y(2) + 64, UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, now_, 1, &ui::font::fixed_8x16},
      sky_{{0, UI_POS_Y(3) + 66, 120, 120}, "pass"},
      text_info_{{122, UI_POS_Y(3) + 66, 118, 8 * kInfoRows}, info_, kInfoRows, &ui::font::fixed_5x8},
      text_sum_{{0, UI_POS_Y(3) + 188, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kSumRows)}, sum_, kSumRows, &ui::font::fixed_8x16},
      button_follow_{{122, UI_POS_Y(3) + 66 + 8 * kInfoRows + 2, 118, 28}, "Follow"} {
    add_children({&text_top_, &big_dir_, &big_el_, &text_now_, &sky_, &text_info_, &text_sum_, &button_follow_});
    sky_.set_focusable(true);
    button_follow_.hidden(true);
    /* Follow replaces the GPS app (this view is gone when follow() returns) */
    button_follow_.on_select = [this](Button& b) {
        if (!main_.follow()) b.set_text("No memory");
    };
    tick_ = rtc_time::signal_tick_second += [this]() {
        refresh();
    };
    refresh();
}

SatView::~SatView() {
    rtc_time::signal_tick_second -= tick_;
}

void SatView::focus() {
    sky_.focus();
}

/* Once a second: where to point now and in 30 s, the countdown, the details;
 * two or three orbit evaluations (a few ms of M0 work). */
void SatView::refresh() {
    char b[TextLines::kCols + 1];
    const Color white = Color::white(), yellow = Color::yellow(), grey = Color::light_grey();
    GpsScratch* m = main_.mem_;
    SatSource src;
    text_top_.begin();
    text_info_.begin();
    text_sum_.begin();
    if (!m || !main_.sel_source(src)) {
        text_top_.set(0, "No satellite selected", yellow);
        big_dir_.set("", white);
        big_el_.set("", white);
        text_now_.set(0, "", white);
        text_top_.end();
        text_info_.end();
        text_sum_.end();
        return;
    }
    const bool tle = main_.sel_kind_ == GpsView::SEL_TLE;
    const double t = main_.now_unix();
    main_.sel_name(b, sizeof(b));
    text_top_.set(0, b, Color::cyan());
    double az = 0, az30 = 0;
    const double el = elevation(src, main_.obs_, t, &az);
    const double el30 = elevation(src, main_.obs_, t + 30.0, &az30);
    {
        Fmt f{b, 16};
        double a = std::fmod(az, 360.0);
        if (a < 0) a += 360.0;
        f.s(compass16(a)).c(' ').u(static_cast<uint32_t>(std::lround(a)) % 360).s(" deg");
        big_dir_.set(b, el >= 0 ? Color::green() : grey);
        Fmt g{b, 16};
        if (el >= 0)
            g.u(static_cast<uint32_t>(std::lround(el))).s(" deg up");
        else
            g.s("below horizon");
        big_el_.set(b, el >= 0 ? Color::green() : grey);
    }
    {
        char p[TextLines::kCols + 1];
        put_point(p, sizeof(p), az30, el30);
        Fmt f{b, sizeof(b)};
        f.s("in 30 s: ").s(p);
        text_now_.set(0, b, white);
    }
    const PassTrack& tr = m->track;
    if (tr.ready()) {
        put_countdown(b, sizeof(b), tr.pass(), t);
        text_top_.set(1, b, el >= 0 ? Color::green() : yellow);
        char l1[TextLines::kCols + 1], l2[TextLines::kCols + 1], l3[TextLines::kCols + 1];
        pass_summary(tr.pass(), tr.top_az(), t, main_.utc_off_min_, l1, l2, l3, sizeof(l1));
        text_sum_.set(0, l1, white);
        text_sum_.set(1, l2, white);
        text_sum_.set(2, l3, white);
        Fmt f{b, sizeof(b)};
        f.s("Local times; UTC rise ");
        char c[8];
        put_clock(c, sizeof(c), tr.pass().rise, 0);
        f.s(c);
        text_sum_.set(3, b, grey);
    } else {
        text_top_.set(1, tr.none() ? "No pass in the next 24 h" : "Finding the pass...", yellow);
    }
    /* sky plot: the arc, the satellite now (ringed) */
    sky_.clear();
    if (el >= 0) sky_.add(static_cast<int16_t>(az), static_cast<int16_t>(el), "", Color::green(), true, true);
    sky_.set_track(&tr);
    sky_.commit();
    /* details (23 columns) */
    Vec3 p0, p1;
    const bool havep = src.position(t, p0) && src.position(t + 1.0, p1);
    const Vec3 d0{p0.x - main_.rx_.x, p0.y - main_.rx_.y, p0.z - main_.rx_.z}, d1{p1.x - main_.rx_.x, p1.y - main_.rx_.y, p1.z - main_.rx_.z};
    const double range = norm(d0), rate = norm(d1) - range;
    size_t r = 0;
    if (tle) {
        const Tle& tl = m->tles[main_.sel_idx_];
        const double age = (t / 86400.0 + 2440587.5) - tl.jd_epoch;
        Fmt f{b, sizeof(b)};
        f.s("TLE ").u(static_cast<uint32_t>(age < 0 ? 0 : age)).s(" d old");
        text_info_.set(r++, b, age > 14 ? Color::red() : (age > 7 ? yellow : grey));
        if (age > 14) {
            text_info_.set(r++, "too old to point by:", Color::red());
            text_info_.set(r++, "refresh /GPS/TLE.TXT", Color::red());
        }
        Fmt g{b, sizeof(b)};
        g.s("catalog ").u(tl.catnr);
        text_info_.set(r++, b, grey);
        /* downlinks from SATFREQ.TXT (Follow tunes the first; it can switch) */
        const uint8_t nd = main_.sel_downlinks();
        uint8_t shown = 0;
        for (uint8_t k = 0; k < main_.n_freq_ && shown < 2; k++) {
            const SatFreq& fq = m->freq[k];
            if (!satfreq_matches(fq, tl.catnr, tl.name)) continue;
            char mhz[16];
            put_mhz(mhz, sizeof(mhz), fq.hz);
            Fmt h{b, sizeof(b)};
            h.s(mhz).c(' ').s(freq_mode_name(fq.mode)).c(' ').s(band_name(fq.hz));
            text_info_.set(r++, b, Color::cyan());
            shown++;
        }
        if (nd > shown) {
            Fmt h{b, sizeof(b)};
            h.s("+").u(nd - shown).s(" more downlink").s(nd - shown > 1 ? "s" : "");
            text_info_.set(r++, b, Color::cyan());
        }
        if (!nd) text_info_.set(r++, main_.freq_file_ ? "no downlink in SATFREQ" : "no /GPS/SATFREQ.TXT", grey);
        if (main_.freq_bad_) {
            Fmt h{b, sizeof(b)};
            h.s("SATFREQ line ").u(main_.freq_bad_line_).s(" bad");
            if (main_.freq_bad_ > 1) h.s(" +").u(main_.freq_bad_ - 1u);
            text_info_.set(r++, b, yellow);
        }
    } else {
        const uint8_t i = main_.sel_idx_;
        const Orbit& o = m->orbits[i];
        text_info_.set(r++, o.health ? "health: NOT OK" : "health: ok", o.health ? Color::red() : grey);
        Fmt f{b, sizeof(b)};
        if (o.ephemeris) {
            f.s("orbit: ephemeris ");
            put_span(f, std::fabs(unix_to_gps(t) - o.toe));
        } else {
            f.s("orbit: almanac ").u(static_cast<uint32_t>(main_.alm_age_days() + 0.5)).s(" d");
        }
        text_info_.set(r++, b, grey);
        if (el >= 0) {
            Fmt g{b, sizeof(b)};
            const int32_t dop = static_cast<int32_t>(std::lround(-rate / (kC / 1575.42e6) / 10.0)); /* 10 Hz */
            g.s("Doppler ");
            if (dop >= 0) g.c('+');
            g.fixed(dop, 2).s(" kHz");
            text_info_.set(r++, b, white);
        }
        Fmt h{b, sizeof(b)};
        if (m->heard_at[i]) {
            h.s("heard C/N0 ").u(m->heard_cn0[i]).c(' ');
            put_span(h, t - m->heard_at[i]);
            h.s(" ago");
            text_info_.set(r++, b, Color::green());
        } else {
            text_info_.set(r++, "not heard yet", grey);
        }
    }
    if (havep) {
        Fmt f{b, sizeof(b)};
        f.s("range ").u(static_cast<uint32_t>(range / 1000.0 + 0.5)).s(" km");
        text_info_.set(r++, b, white);
        Fmt g{b, sizeof(b)};
        const int32_t v = static_cast<int32_t>(std::lround(rate / 100.0)); /* 0.1 km/s */
        g.s(v < 0 ? "closing " : "receding ").fixed(v < 0 ? -v : v, 1).s(" km/s");
        text_info_.set(r++, b, grey);
    }
    if (tr.ready()) {
        const Pass& ps = tr.pass();
        Fmt f{b, sizeof(b)};
        f.s("max ").u(static_cast<uint32_t>(ps.max_el < 0 ? 0 : ps.max_el + 0.5f)).s(" up");
        if (ps.set != 0 && !ps.up_at_start) {
            f.s(", ");
            put_span(f, ps.set - ps.rise);
            f.s(" long");
        }
        text_info_.set(r++, b, white);
    }
    const bool can_follow = tle && main_.sel_downlinks() > 0;
    if (button_follow_.hidden() == can_follow) {
        button_follow_.hidden(!can_follow);
        set_dirty();
    }
    if (!can_follow && r + 4 <= kInfoRows) {
        text_info_.set(r++, "", grey);
        text_info_.set(r++, "Point the antenna to the", grey);
        text_info_.set(r++, "compass bearing, tilted", grey);
        text_info_.set(r, "up by the elevation.", grey);
    }
    text_top_.end();
    text_info_.end();
    text_sum_.end();
}

/* ---------------------------------------------------------------- main */

GpsView::GpsView(NavigationView& nav, uint32_t reselect)
    : nav_{nav},
      settings_{
          "rx_gps",
          app_settings::Mode::RX,
          {{"utc_off_min"sv, &utc_off_min_},
           {"tab"sv, &tab_},
           {"depth"sv, &depth_},
           {"log"sv, &log_on_}}},
      reselect_{reselect},
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
    /* rx_gps.ini (RX settings) restores the saved modulation, frequency, rate and
     * filter: older files say WFM, which tunes the radio fs/4 = 512 kHz below L1
     * (ReceiverModel::tuning_offset) and puts every satellite out of the search.
     * PGPR expects L1 at DC and exactly 2.048 Msps: enforce that here. */
    receiver_model.settings().mode = ReceiverModel::Mode::SpectrumAnalysis;
    receiver_model.set_target_frequency(1575420000);
    receiver_model.set_sampling_rate(2048000);
    receiver_model.set_baseband_bandwidth(1750000);

    /* The receiver is on for as long as the app is open, as in the stock
     * receive apps: ReceiverModel::enable() applies the stored antenna-power
     * setting (update_antenna_bias -> radio::set_antenna_bias(portapack::
     * get_antenna_bias())), and the status-bar control acts on an enabled
     * model only. The app never writes that setting. An active antenna's LNA
     * has also settled before the first snapshot. */
    receiver_model.enable();
    rx_on_ = true;
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
    if (log_file_ && mem_) { /* what is still in the ring, then f_close (syncs) */
        for (int k = 0; k < 16 && log_pending(mem_->log); k++) {
            const char* p;
            const size_t n = log_piece(mem_->log, log_pos_, &p);
            if (log_file_->write(p, n).is_error()) break;
            log_taken(mem_->log, n);
            log_pos_ += static_cast<uint32_t>(n);
        }
    }
    log_close();
    tab_ = tab_view_.selected();
    if (solver_) { /* it works in the lent RAM: stop it first (one satellite row at most) */
        if (mem_) mem_->solve.cancel = mem_->autos.pvt.cancel = 1;
        chThdWait(solver_);
        solver_ = nullptr;
    }
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
    geo_ready_ = false; /* the sky from the new position */
    geo_restart();
    view_passes_.refresh();
    view_passes_.update_sky();
    view_passes_.restart_passes();
    view_fix_.show();
}

void GpsView::start_load() {
    if (!mem_ || solving()) { /* the solver reads the orbit table */
        load_wanted_ = true;
        return;
    }
    load_wanted_ = false;
    for (auto& o : mem_->orbits) o = Orbit{};
    for (auto& a : mem_->alm) a = AlmRec{};
    n_alm_ = n_eph_ = n_tle_ = 0;
    n_freq_ = freq_bad_ = 0;
    freq_bad_line_ = 0;
    for (auto& f : mem_->tle_freq) f = GpsScratch::kNoFreq;
    eph_toe_newest_ = 0;
    alm_file_ = eph_file_ = tle_file_ = freq_file_ = false;
    alm_toa_ = eph_file_newest_ = 0;
    geo_ready_ = geo_active_ = false;
    sel_kind_ = SEL_NONE; /* the tables are re-read */
    loader_.start(now_gps(), *this);
    view_passes_.refresh();
}

bool GpsView::loading() const {
    return loader_.stage() != OrbitLoader::DONE && loader_.stage() != OrbitLoader::IDLE;
}

double GpsView::alm_age_days() const {
    if (alm_toa_ <= 0) return -1;
    return std::fabs(now_gps() - alm_toa_) / 86400.0;
}

/* An ephemeris older than kEphMaxAgeS falls back to the satellite's almanac
 * (the app may stay open for hours; the daily file has newer sets, "Load"
 * re-reads it). */
void GpsView::refresh_orbits() {
    if (!mem_ || solving() || loading()) return;
    const double tg = now_gps();
    for (int p = 0; p < 32; p++) {
        Orbit& o = mem_->orbits[p];
        if (!o.valid || !o.ephemeris || std::fabs(tg - o.toe) <= kEphMaxAgeS) continue;
        if (n_eph_) n_eph_--;
        o = mem_->alm[p].valid ? from_alm(mem_->alm[p]) : Orbit{};
    }
}

GpsView::Blocker GpsView::fix_blocker() const {
    if (!mem_) return BLOCK_STARTING;
    if (loading()) return BLOCK_LOADING;
    if (!pos_valid_) return BLOCK_NO_POS;
    if (!n_alm_ && !n_eph_) return BLOCK_NO_ORBITS;
    if (!n_eph_ && alm_age_days() > kAlmMaxAgeDays) return BLOCK_ALM_OLD;
    return BLOCK_NONE;
}

bool GpsView::assisted_possible() const {
    return mem_ && pos_valid_ && (n_alm_ || n_eph_);
}

void GpsView::request_search(Job job, uint8_t k_ms) {
    if (!mem_ || searching()) return;
    if (assisted_possible() && !geo_ready_) { /* started by geo_step() when the sky is known */
        want_job_ = job;
        want_kms_ = k_ms;
        if (!geo_active_) geo_restart();
        return;
    }
    want_job_ = JOB_NONE;
    if (!start_search(job, k_ms) && job == JOB_FIX) {
        fix_tried_ = true; /* nothing above the horizon to search */
        fix_cand_ = 0;
    }
}

/* The sky cache: one unit per call (a GPS satellite: position, az/el and
 * range rate; a TLE: SGP4 init, then its az/el), ~1-4 ms of M0 work each by
 * the emulator count. A pass every kGeoPeriodS; at its end the views redraw. */
void GpsView::geo_restart() {
    geo_idx_ = 0;
    geo_active_ = mem_ && pos_valid_ && !loading();
    geo_started_ = static_cast<uint32_t>(now_unix());
}

void GpsView::geo_step() {
    if (!geo_active_ || !mem_) return;
    const double t = now_unix();
    if (geo_idx_ < 32) {
        const Orbit& o = mem_->orbits[geo_idx_];
        SatGeo& g = mem_->geo[geo_idx_];
        g = SatGeo{};
        if (o.valid && !o.health) {
            const double tg = unix_to_gps(t);
            const Vec3 sp = sat_position(o, tg);
            double az, el;
            az_el(obs_, sp, az, el);
            g.az = static_cast<float>(az);
            g.el = static_cast<float>(el);
            if (el > -5) { /* range rate from a second position 1 s later (the Doppler centre of a search) */
                const Vec3 s2 = sat_position(o, tg + 1.0);
                const Vec3 d1{sp.x - rx_.x, sp.y - rx_.y, sp.z - rx_.z}, d2{s2.x - rx_.x, s2.y - rx_.y, s2.z - rx_.z};
                g.rr = static_cast<float>(norm(d2) - norm(d1));
            }
        }
    } else {
        const size_t k = geo_idx_ - 32u, i = k / 2;
        if (i < n_tle_) {
            SatGeo& g = mem_->tle_geo[i];
            if (!(k & 1)) {
                g = SatGeo{};
                if (!sgp4_init(mem_->tles[i], mem_->sky)) geo_idx_++; /* skip its second unit */
            } else {
                SatSource src;
                src.kind = SatSource::SGP4;
                src.sgp4 = &mem_->sky;
                double az = 0;
                g.el = static_cast<float>(elevation(src, obs_, t, &az));
                g.az = static_cast<float>(az);
            }
        }
    }
    if (++geo_idx_ < geo_units()) return;
    geo_active_ = false;
    geo_ready_ = true;
    view_passes_.update_sky();
    view_heard_.show();
    view_fix_.show();
    if (want_job_ != JOB_NONE) request_search(want_job_, want_kms_);
}

bool GpsView::start_search(Job job, uint8_t k_ms, bool force_full) {
    if (!mem_) return false;
    if (!rx_on_) { /* enabled since the app opened; kept as a guard */
        receiver_model.enable();
        rx_on_ = true;
    }
    GpsRxConfigMessage m;
    m.cmd = GpsRxConfigMessage::CMD_SEARCH;
    m.k_ms = k_ms;
    /* Autonomous: the fast refine (Doppler only): the hand-over is ~10 s younger on the
     * device, and the coarse fix it also feeds is almanac-limited (~1 km) anyway */
    m.refine = job == JOB_AUTO ? 2 : (job == JOB_FIX ? 1 : 0);
    m.confirm = 1; /* a PRN is reported only if a second snapshot confirms it */
    m.track = job == JOB_AUTO; /* the confirmed PRNs go straight to the tracking loops */
    m.max_ch = 8;              /* M4: ~60 % with 8 channels (notes, gps-autonomous) */
    int nprn = 0;
    if (!force_full && assisted_possible() && geo_ready_) {
        /* visible PRNs and their Doppler from the sky cache (at most kGeoPeriodS
         * plus one pass old: the Doppler moves < 1 Hz/s) */
        const double min_el = job == JOB_FIX ? kMinFixEl : 0.0;
        for (int p = 0; p < 32; p++) {
            const Orbit& o = mem_->orbits[p];
            const SatGeo& g = mem_->geo[p];
            if (!o.valid || o.health || g.el <= min_el) continue;
            m.prn_mask |= 1u << p;
            m.center[p] = static_cast<int16_t>(std::lround(-g.rr / kL1Lambda));
            nprn++;
        }
        m.half_bins = kAssistedHalfBins;
    }
    search_kind_ = nprn ? SK_ASSISTED : (force_full ? SK_FULL_AFTER : SK_FULL);
    if (!nprn) {
        if (job == JOB_FIX && !force_full) return false; /* Autonomous may search all 32 (slow, no files) */
        m.prn_mask = 0xFFFFFFFFu;
        m.half_bins = kColdHalfBins;
        nprn = 32;
    }
    job_units_ = static_cast<uint32_t>(nprn) * (2u * m.half_bins + 1u) * k_ms;
    job_ = job;
    done_ = false;
    tracking_ = false;
    awaiting_seq_ = true;
    results_ = nullptr;
    baseband::gps_rx_config(m);
    return true;
}

void GpsView::stop_auto() {
    want_job_ = JOB_NONE;
    if (job_ != JOB_AUTO) return;
    GpsRxConfigMessage m;
    m.cmd = GpsRxConfigMessage::CMD_STOP; /* the search and the tracking loops */
    baseband::gps_rx_config(m);
    job_ = JOB_NONE;
    awaiting_seq_ = false;
    done_ = false;
    solve_pending_ = false;
    tracking_ = false;
    if (solver_ && mem_) mem_->solve.cancel = mem_->autos.pvt.cancel = 1;
}

void GpsView::stop_search() {
    if (job_ == JOB_AUTO) {
        stop_auto();
        return;
    }
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
    status_dt_ms_ = m.dt_ms;
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
    if (!awaiting_seq_ && job_ != JOB_NONE && m.seq == job_seq_ && m.state == GpsRxStatusMessage::DONE && !done_ &&
        search_kind_ == SK_ASSISTED && m.level != GpsRxStatusMessage::LEVEL_LOW && m.results) {
        /* the assisted search found nothing: the prediction may be wrong (position,
         * clock, Zone, an old almanac, a large TCXO offset). Search all 32 PRNs over
         * +-7 kHz instead, 10 ms (enough above ~39 dB-Hz, half the time of 20 ms). */
        const auto* r = static_cast<const gps_rx::Result*>(m.results);
        int det = 0;
        for (int p = 0; p < 32; p++) det += r[p].detected;
        if (!det) {
            start_search(job_, 10, true);
            view_heard_.show();
            view_fix_.show();
            return;
        }
    }
    if (!awaiting_seq_ && job_ != JOB_NONE && m.seq == job_seq_ && m.state == GpsRxStatusMessage::DONE && !done_) {
        done_ = true;
        results_ = static_cast<const gps_rx::Result*>(m.results);
        log_results(results_, m.k_ms);
        if (job_ == JOB_FIX || job_ == JOB_AUTO) solve_pending_ = pos_valid_ && (n_alm_ || n_eph_); /* Autonomous: the coarse fix first */
        if (mem_ && results_) {
            const uint32_t t = static_cast<uint32_t>(now_unix());
            for (int p = 0; p < 32; p++)
                if (results_[p].detected) {
                    mem_->heard_at[p] = t;
                    mem_->heard_cn0[p] = static_cast<uint8_t>(results_[p].cn0 + 0.5f);
                }
        }
    }
    if (m.track) trk_sh_ = static_cast<const gps_trk::Shared*>(m.track);
    if (job_ == JOB_AUTO && done_ && m.state == GpsRxStatusMessage::TRACKING && !tracking_) {
        tracking_ = true;
        log_handover();
    }
    if (job_ == JOB_AUTO && tracking_) {
        auto_poll_m4();
        view_fix_.show();
        view_heard_.show();
        return;
    }
    /* redraw only when something shown changed (idle status comes twice a second) */
    const uint32_t sig = (status_.state | (status_.level << 4) | (status_.confirming << 7)) ^ (status_.done_prn << 8) ^
                         (static_cast<uint32_t>(progress_pct()) << 14) ^ (static_cast<uint32_t>(status_.rms_x10) << 21) ^
                         (done_ ? 0x80000000u : 0) ^ (awaiting_seq_ ? 0x40000000u : 0) ^ (status_.seq << 27);
    if (sig == status_sig_) return;
    status_sig_ = sig;
    view_heard_.show();
    view_fix_.show();
}

/* Snapshot fix: detected PRNs with an orbit, the saved position as the a
 * priori, the RTC as the coarse time. Precise when every one has a fresh
 * ephemeris; otherwise coarse, with the almanac limits (gps_solve.hpp: RAIM
 * residual and error estimate by almanac age, clock within 30 s). The solve
 * runs in a low-priority thread: 0.2 s of soft double per attempt with 7
 * satellites by the emulator count, longer on the device. */
void GpsView::solve_fix() {
    solve_pending_ = false;
    fix_tried_ = true;
    fix_ok_ = false;
    fix_heard_ = fix_cand_ = 0;
    fix_mode_ = FIX_NONE;
    if (!mem_ || !results_ || !pos_valid_ || solver_) {
        log_fix(0, false, !pos_valid_ ? "not tried: no saved position" : (solver_ ? "not tried: solver busy" : "not tried: no results"));
        return;
    }
    Measurement* m = mem_->solve.in; /* off the stack */
    int n = 0;
    bool all_eph = true;
    double newest = 0;
    const double tg = now_gps();
    for (int p = 0; p < 32; p++) {
        const auto& r = results_[p];
        const Orbit& o = mem_->orbits[p];
        if (!r.detected) continue;
        fix_heard_++;
        if (!o.valid || n >= kMaxMeas) continue;
        m[n++] = {&o, r.code_phase, r.metric};
        all_eph = all_eph && o.ephemeris && std::fabs(tg - o.toe) <= kEphMaxAgeS;
        if (o.ephemeris && o.toe > newest) newest = o.toe;
    }
    fix_cand_ = static_cast<uint8_t>(n);
    if (n < kMinFixSats) {
        log_fix(0, false, "not tried: fewer than 6 heard with an orbit");
        return;
    }
    fix_mode_ = all_eph ? FIX_PRECISE : FIX_COARSE;
    fix_alm_age_d_ = alm_age_days();
    fix_eph_age_s_ = tg - newest;
    if (fix_mode_ == FIX_COARSE && (fix_alm_age_d_ < 0 || fix_alm_age_d_ > kAlmMaxAgeDays)) {
        log_fix(0, false, "not tried: no almanac or older than 30 d");
        return;
    }
    solve_n_ = n;
    solve_t_ = tg;
    solve_prior_ = rx_;
    solve_max_rms_ = all_eph ? 30.0 : almanac_max_rms(fix_alm_age_d_);
    solve_max_dt_ = all_eph ? 60.0 : kAlmMaxDtS;
    solve_kind_ = 0;
    mem_->solve.cancel = 0;
    log_solve_start(0);
    solver_done_ = false;
    solver_ = chThdCreateFromHeap(nullptr, kSolverStack, NORMALPRIO - 2, solver_fn, this);
    if (!solver_) { /* no heap for the thread: solve here (the UI waits) */
        log_error("solver thread", "no heap: snapshot solve on the UI thread");
        solve_ok_ = snapshot_fix(mem_->solve.in, solve_n_, solve_prior_, solve_t_, solve_out_, mem_->solve, solve_max_rms_, solve_max_dt_);
        solver_done_ = true;
        finish_solve();
    }
}

msg_t GpsView::solver_fn(void* arg) {
    auto* v = static_cast<GpsView*>(arg);
    if (v->solve_kind_ == 1) { /* PVT from the tracking loops */
        GpsScratch& m = *v->mem_;
        const double max_rms = v->solve_pvt_eph_ ? 25.0 : almanac_max_rms(v->alm_age_days() < 0 ? 30.0 : v->alm_age_days());
        v->solve_ok_ = pvt_solve(m.autos.pvt, v->solve_pvt_n_, v->solve_prior_, v->solve_pvt_trx_, m.pvt, max_rms, true);
        v->solver_done_ = true;
        return 0;
    }
    v->solve_ok_ = snapshot_fix(v->mem_->solve.in, v->solve_n_, v->solve_prior_, v->solve_t_, v->solve_out_, v->mem_->solve,
                                v->solve_max_rms_, v->solve_max_dt_);
    v->solver_done_ = true;
    return 0;
}

void GpsView::finish_solve() {
    if (solver_) {
        chThdWait(solver_);
        solver_ = nullptr;
    }
    if (solve_kind_ == 1) {
        solve_kind_ = 0;
        auto_finish_pvt();
        return;
    }
    const bool cancelled = mem_ && mem_->solve.cancel;
    fix_ok_ = !cancelled && solve_ok_;
    log_fix(0, fix_ok_, cancelled ? "cancelled" : (solve_ok_ ? "ok" : "rejected: residuals / clock over the limits (No fix: the N heard disagree)"));
    if (cancelled) fix_tried_ = false;
    if (fix_ok_) {
        fix_ = solve_out_;
        fix_fat_ = saved_position::now_fat();
        const double est = fix_mode_ == FIX_COARSE ? almanac_err_m(fix_alm_age_d_, fix_.hdop) : fix_.hdop * 15.0;
        fix_est_m_ = static_cast<uint32_t>(std::lround(est));
    }
    if (load_wanted_) start_load(); /* a Load pressed while solving */
    view_fix_.show();
}

/* ---------------------------------------------------------------- autonomous */

/* About 5 times a second while the tracking loops run (from the status
 * message, UI thread): copy the channel table, take new subframes (not while
 * the solver reads the orbit table: they wait in the ring, 16 slots ~ 12 s),
 * and start a PVT on each new measurement set when the solver is free. */
void GpsView::auto_poll_m4() {
    if (!mem_ || !trk_sh_) return;
    AutoState& a = mem_->autos;
    const int32_t week_hint = static_cast<int32_t>(now_gps() / kWeekS);
    bool new_eph = false;
    const gps_trk::Shared* sh = trk_sh_;
    bool newmeas;
    if (solving()) { /* channels and measurements only */
        newmeas = false;
        for (int tries = 0; tries < 3; tries++) {
            const uint32_t s1 = sh->ch_seq;
            if (s1 & 1u) continue;
            for (int i = 0; i < gps_trk::kMaxCh; i++) a.ch[i] = sh->ch[i];
            if (sh->ch_seq == s1) break;
        }
    } else {
        log_subframes();
        newmeas = auto_poll(a, sh, week_hint, mem_->orbits, [&](int prn) {
            new_eph = true;
            log_eph_event("decoded", prn);
        });
        if (newmeas) log_meas();
    }
    if (new_eph) { /* off-air ephemerides now in the orbit table */
        int ne = 0;
        double newest = 0;
        for (int p = 0; p < 32; p++)
            if (mem_->orbits[p].valid && mem_->orbits[p].ephemeris) {
                ne++;
                if (mem_->orbits[p].toe > newest) newest = mem_->orbits[p].toe;
            }
        n_eph_ = static_cast<uint8_t>(ne);
        if (newest > eph_toe_newest_) eph_toe_newest_ = newest;
    }
    /* counts for the screen */
    uint8_t tracked = 0, synced = 0, with_eph = 0;
    for (int i = 0; i < gps_trk::kMaxCh; i++) {
        const auto& c = a.ch[i];
        if (c.state != gps_trk::TRK_TRACK || !(c.flags & gps_trk::F_VERIFIED)) continue;
        tracked++;
        if (c.flags & gps_trk::F_FRAME) synced++;
        if (c.prn >= 1 && c.prn <= 32 && mem_->orbits[c.prn - 1].valid && mem_->orbits[c.prn - 1].ephemeris &&
            std::fabs(now_gps() - mem_->orbits[c.prn - 1].toe) <= kEphMaxAgeS)
            with_eph++;
    }
    auto_tracked_ = tracked;
    auto_synced_ = synced;
    auto_eph_ = with_eph;
    /* GPS time from the first TOW (within ~20 ms; the PVT refines it) */
    if (!gps_time_ok_ && a.have_meas) {
        for (int i = 0; i < a.meas.n; i++)
            if (a.meas.ch[i].flags & gps_trk::F_FRAME) {
                const int32_t wk = a.week > 0 ? a.week : week_hint;
                const double t = wk * kWeekS + a.meas.ch[i].tx_ms * 1e-3 + 0.075;
                gps_off_s_ = t - unix_to_gps(now_unix());
                gps_time_ok_ = true;
                break;
            }
    }
    if (!newmeas || solving() || solve_pending_) return;
    /* a PVT: precise with >= 4 ephemerides, else coarse with almanac orbits */
    const int32_t week = a.week > 0 ? a.week : week_hint;
    int n_eph = 0;
    int n = auto_build_pvt(a, mem_->orbits, false, week, &n_eph);
    bool eph_only = false;
    if (n_eph >= 4) {
        n = auto_build_pvt(a, mem_->orbits, true, week, &n_eph);
        eph_only = true;
    } else if (fix_ok_ && fix_mode_ == FIX_COARSE && mem_->pvt_fix.ok) {
        /* coarse already: refresh it at most every 10 s (the M0 time goes to the solver) */
        static uint32_t last = 0;
        const uint32_t t = chTimeNow();
        if (t - last < S2ST(10)) return;
        last = t;
    }
    if (n < 4) return;
    solve_pvt_n_ = n;
    solve_pvt_eph_ = eph_only;
    solve_pvt_trx_ = auto_t_rx_guess(a, n);
    solve_prior_ = (fix_ok_ ? fix_.pos : (pos_valid_ ? rx_ : Vec3{0, 0, 0}));
    solve_kind_ = 1;
    a.pvt.cancel = 0;
    log_solve_start(eph_only ? 2 : 1);
    solver_done_ = false;
    solver_ = chThdCreateFromHeap(nullptr, kSolverStack, NORMALPRIO - 2, solver_fn, this);
    if (!solver_) { /* no heap now: the next second */
        solve_kind_ = 0;
        log_error("solver thread", "no heap: PVT skipped");
    }
}

void GpsView::auto_finish_pvt() {
    if (!mem_) return;
    const bool cancelled = mem_->autos.pvt.cancel;
    {
        static const char* const kWhy[] = {"ok", "rejected: fewer than 4", "cancelled", "rejected: least squares failed", "rejected: residuals over the limit (RAIM)", "rejected: height implausible"};
        const uint8_t fl = mem_->pvt.fail;
        log_fix(solve_pvt_eph_ ? 2 : 1, !cancelled && solve_ok_, cancelled ? "cancelled" : kWhy[fl < 6 ? fl : 0]);
    }
    if (cancelled || !solve_ok_) return;
    PvtSolution& p = mem_->pvt;
    /* a coarse (almanac) PVT never replaces a precise fix */
    if (!solve_pvt_eph_ && fix_ok_ && fix_mode_ == FIX_PRECISE) return;
    mem_->pvt_fix = p;
    fix_ok_ = true;
    fix_tried_ = true;
    fix_.pos = p.pos;
    fix_.clock_m = p.clock_m;
    fix_.resid_rms_m = p.resid_rms_m;
    fix_.hdop = p.hdop;
    fix_.used = p.used;
    fix_.ok = true;
    fix_fat_ = saved_position::now_fat();
    fix_mode_ = solve_pvt_eph_ ? FIX_PRECISE : FIX_COARSE;
    fix_alm_age_d_ = alm_age_days();
    const double est = solve_pvt_eph_ ? pvt_err_m(p.hdop) : almanac_err_m(fix_alm_age_d_ < 0 ? 30.0 : fix_alm_age_d_, p.hdop);
    fix_est_m_ = static_cast<uint32_t>(std::lround(est));
    const uint32_t since = static_cast<uint32_t>((chTimeNow() - auto_t0_) / CH_FREQUENCY);
    if (solve_pvt_eph_ && !ttff_s_) ttff_s_ = since ? since : 1;
    if (!ttff_coarse_s_) ttff_coarse_s_ = since ? since : 1;
    /* GPS time from the solution (the measurement is < ~1 s old: within the RTC's 1 s anyway) */
    gps_off_s_ = p.t_rx - unix_to_gps(now_unix());
    gps_time_ok_ = true;
    view_fix_.show();
}

/* One EPH_AIR.TXT line per call (three per ephemeris, < 100 bytes each):
 * from the frame handler (UI thread), never while the solver runs or the
 * loader reads. A user's EPHEM.RNX is never written. */
void GpsView::auto_cache_step() {
    AutoState& a = mem_->autos;
    if (cache_line_ == 0) { /* find a fresh set */
        int k = 0;
        for (; k < kAutoSlots; k++) {
            const EphCollect& c = a.col[(cache_cursor_ + k) % kAutoSlots];
            if (c.prn && c.fresh && c.have == 7) break;
        }
        if (k == kAutoSlots) return;
        cache_cursor_ = static_cast<uint8_t>((cache_cursor_ + k) % kAutoSlots);
    }
    EphCollect& c = a.col[cache_cursor_];
    if (!c.fresh || c.have != 7) { /* replaced meanwhile: start again */
        cache_line_ = 0;
        return;
    }
    char line[100];
    const int32_t week = a.week > 0 ? a.week : static_cast<int32_t>(now_gps() / kWeekS);
    const size_t n = eph_air_line(line, sizeof(line), c.prn, week, static_cast<uint8_t>(cache_line_ + 1), nav::sf_tow_start(c.sf[cache_line_]), c.sf[cache_line_]);
    bool ok = false;
    File* f = new File(); /* heap, not the M0 stack (~0.6 KB) */
    if (cache_line_ == 0) {
        /* a new file with its header, or append; start again past ~48 KB */
        bool exists = !f->open(u"GPS/EPH_AIR.TXT").is_valid();
        const bool big = exists && f->size() > 48u * 1024u;
        f->close();
        if (big) {
            delete_file(u"GPS/EPH_AIR.TXT");
            exists = false;
        }
        if (!exists && !f->create(u"GPS/EPH_AIR.TXT").is_valid()) {
            const char* hdr = "# GPS ephemerides decoded off the air (HackRF GPS app, source off-air)\n# S prn week subframe tow_s 10 data words (hex); read with EPHEM.RNX\n";
            f->write(hdr, std::strlen(hdr));
            f->close();
        }
    }
    if (!f->append(u"GPS/EPH_AIR.TXT").is_valid()) {
        ok = !f->write(line, n).is_error();
        f->close();
    }
    delete f;
    if (!ok) { /* no card / no folder: give up on this one */
        cache_errs_++;
        log_error("EPH_AIR.TXT write failed", "");
        c.fresh = 0;
        cache_line_ = 0;
        return;
    }
    if (++cache_line_ == 3) {
        cache_line_ = 0;
        c.fresh = 0;
        cache_written_++;
        log_eph_event("cached", c.prn);
        cache_cursor_ = static_cast<uint8_t>((cache_cursor_ + 1) % kAutoSlots);
    }
}

void GpsView::on_frame() {
    log_frame(); /* the diagnostics log: rows, card writes (sector-bounded), sync */
    /* the selected tab's content is refreshed when it becomes visible */
    const uint32_t sel = tab_view_.selected();
    if (sel != tab_shown_) {
        tab_shown_ = sel;
        if (sel == 0) view_fix_.show();
        if (sel == 1) view_heard_.show();
        if (sel == 2) {
            view_passes_.refresh();
            view_passes_.update_sky();
            view_passes_.tick();
        }
    }
    if (solver_ && solver_done_) finish_solve();
    if (solve_pending_) {
        solve_fix();
        view_fix_.show();
        return;
    }
    /* one job per frame, each in whole units until the time slice is used */
    const systime_t t0 = chTimeNow();
    if (loading()) {
        if (!loader_.step(*this, t0)) { /* all files read */
            view_passes_.refresh();
            geo_restart();
            view_passes_.update_sky();
            view_passes_.restart_passes();
            view_fix_.show();
            view_heard_.show();
            if (reselect_) { /* back from Follow: the satellite followed */
                for (uint8_t k = 0; k < n_tle_; k++)
                    if (mem_->tles[k].catnr == reselect_) select(SEL_TLE, k);
                reselect_ = 0;
            }
        }
        return;
    }
    if (mem_ && !solving() && !loading() && job_ == JOB_AUTO) auto_cache_step(); /* one line of an off-air ephemeris to /GPS/EPH_AIR.TXT */
    if (geo_active_) {
        geo_step();
        return;
    }
    if (mem_ && sel_kind_ != SEL_NONE && mem_->track.busy()) { /* the selected satellite's pass */
        bool more = true;
        do {
            more = mem_->track.step(1);
        } while (more && chTimeNow() - t0 < kSliceTicks);
        if (!more) {
            view_passes_.update_sky();
            view_heard_.show();
        }
        return;
    }
    view_passes_.frame(t0);
}

bool GpsView::sel_source(SatSource& src) {
    if (!mem_) return false;
    if (sel_kind_ == SEL_GPS && sel_idx_ < 32 && mem_->orbits[sel_idx_].valid) {
        src.kind = SatSource::KEPLER;
        src.orbit = &mem_->orbits[sel_idx_];
        return true;
    }
    if (sel_kind_ == SEL_TLE && sel_idx_ < n_tle_) {
        src.kind = SatSource::SGP4;
        src.sgp4 = &mem_->sel;
        return true;
    }
    return false;
}

void GpsView::select(uint8_t kind, uint8_t idx) {
    sel_kind_ = static_cast<SelKind>(kind);
    sel_idx_ = idx;
    if (mem_ && sel_kind_ == SEL_TLE && (idx >= n_tle_ || !sgp4_init(mem_->tles[idx], mem_->sel))) sel_kind_ = SEL_NONE;
    SatSource src;
    if (mem_) mem_->track.clear();
    if (sel_kind_ != SEL_NONE && pos_valid_ && sel_source(src))
        mem_->track.start(src, obs_, now_unix(), sel_kind_ == SEL_TLE);
    view_passes_.update_sky();
    view_heard_.show();
}

void GpsView::open_selected() {
    if (sel_kind_ != SEL_NONE && mem_ && pos_valid_) nav_.push<SatView>(*this);
}

uint8_t GpsView::sel_downlinks() const {
    if (!mem_ || sel_kind_ != SEL_TLE || sel_idx_ >= n_tle_) return 0;
    const Tle& t = mem_->tles[sel_idx_];
    uint8_t n = 0;
    for (uint8_t k = 0; k < n_freq_; k++) n += satfreq_matches(mem_->freq[k], t.catnr, t.name) ? 1 : 0;
    return n;
}

/* The orbit tables live in M4 RAM that PGPR lends and are gone once the
 * stock audio image runs, so Follow gets its own copy of the element set,
 * the observer and the downlinks, on the heap (not the stack: the M0 stack
 * has ~1.3 KB to spare and the replace below runs this view's destructor,
 * which writes the settings file). chHeapAlloc, not operator new: that halts
 * the firmware when the heap is short. */
bool GpsView::follow() {
    if (!mem_ || !pos_valid_ || !sel_downlinks()) return false;
    void* p = chHeapAlloc(nullptr, sizeof(FollowCtx));
    if (!p) return false;
    auto* ctx = new (p) FollowCtx{};
    const Tle& t = mem_->tles[sel_idx_];
    ctx->tle = t;
    ctx->obs = obs_;
    ctx->utc_off_min = utc_off_min_;
    for (uint8_t k = 0; k < n_freq_ && ctx->n_dl < FollowCtx::kMaxDl; k++)
        if (satfreq_matches(mem_->freq[k], t.catnr, t.name)) ctx->dl[ctx->n_dl++] = mem_->freq[k];
    /* Pop the satellite page, then replace this view: this view's destructor
     * stops the solver, disables the receiver and shuts PGPR down before
     * FollowView starts its image. Nothing of this object is used after the
     * replace. */
    NavigationView& nav = nav_;
    nav.pop();
    nav.replace<FollowView>(ctx);
    return true;
}

void GpsView::sel_name(char* buf, size_t size) const {
    Fmt f{buf, size};
    if (sel_kind_ == SEL_GPS)
        f.s("GPS ").c('G').d2(static_cast<uint32_t>(sel_idx_ + 1));
    else if (sel_kind_ == SEL_TLE && mem_)
        f.s(mem_->tles[sel_idx_].name);
}

/* Antenna power, read only (never switched by this app). */
/* Antenna power as applied to the port: the stored setting takes effect
 * through the enabled receiver (ReceiverModel::update_antenna_bias), so it is
 * ON only while the receiver runs with the setting on. Read only. */
void GpsView::show_antenna() {
    const int8_t on = (rx_on_ && portapack::get_antenna_bias()) ? 1 : 0;
    if (on == bias_shown_) return;
    bias_shown_ = on;
    text_ant_.set(on ? "Ant: ON" : "Ant: off");
    text_warn_.set(on ? "PORT 3.3V ON: active ant only" : "");
    text_warn_.set_style(Theme::getInstance()->fg_red);
}

void GpsView::on_tick_second() {
    show_antenna();
    if (mem_ && sel_kind_ != SEL_NONE && mem_->track.ready() && mem_->track.pass().set != 0 &&
        now_unix() > mem_->track.pass().set + 60.0)
        select(sel_kind_, sel_idx_); /* that pass is over: the next one */
    refresh_orbits();
    view_passes_.refresh();
    if (!geo_active_ && !loading() && static_cast<uint32_t>(now_unix()) - geo_started_ >= kGeoPeriodS) geo_restart();
    if (++sky_tick_ % 10 == 0) view_passes_.tick(); /* the "in 1h05" countdowns */
    if (sky_tick_ >= 600) {                         /* passes that ended: search again */
        sky_tick_ = 0;
        view_passes_.restart_passes();
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
