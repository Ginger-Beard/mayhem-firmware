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

/* Follow (see ui_gps.hpp): Doppler-corrected receive of one satellite's
 * downlink on the stock audio images, with a meter, tone and the pointing
 * readout. Receive only. The antenna-power setting is never written: the
 * receiver is switched on with the stock receiver_model.enable(), which
 * applies the user's setting, and the state is only shown. */

#include "ui_gps.hpp"

#ifdef PRALINE

#include "portapack.hpp"
#include "rtc_time.hpp"
#include "theme.hpp"
#include "audio.hpp"
#include "baseband_api.hpp"
#include "gps_fmt.hpp"
#include "hal.h"

#include <cmath>
#include <cstring>

using namespace portapack;

namespace ui::external_app::gps {

namespace {

constexpr systime_t kTrackSlice = MS2ST(4); /* pass search per frame */
constexpr systime_t kDopPeriod = MS2ST(250);
constexpr double kAhead = 0.125;   /* compute for the middle of the next period */
constexpr double kClockErrS = 5.0; /* assumed for the error estimate: RTC set by hand */
constexpr double kPosErrKm = 5.0;  /* ... and a town-level position */
constexpr int32_t kBarLo = -110, kBarHi = -10;
constexpr int32_t kSqOff = -120;        /* Sq at the bottom: squelch off */
constexpr int32_t kPitchSilent = -1000; /* the NFM tone at 0 Hz: DC, removed by its 300 Hz high-pass */

/* the NFM receive filter for a signal width (stock nbfm_configs: 8k5, 11k, 12k5, 16k) */
uint8_t nfm_config(const SatFreq& f) {
    if (f.mode == FMODE_DATA) return 3; /* the widest: a slice of a wide digital signal, for the meter */
    const uint32_t bw = f.bw_hz ? f.bw_hz : 12500;
    if (bw <= 9000) return 0;
    if (bw <= 11500) return 1;
    if (bw <= 14000) return 2;
    return 3;
}

const char* nfm_name(uint8_t c) {
    static const char* const k[] = {"8k5", "11k", "12k5", "16k"};
    return k[c & 3];
}

void put_dbs(Fmt& f, int32_t db) {
    f.d(db);
}

}  // namespace

/* ---------------------------------------------------------------- widgets */

void FollowView::MeterBar::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto bg = Theme::getInstance()->bg_darkest->background;
    const int w = r.width(), h = r.height();
    auto xof = [w](float db) {
        float v = (db - kBarLo) / static_cast<float>(kBarHi - kBarLo);
        v = v < 0 ? 0 : (v > 1 ? 1 : v);
        return static_cast<int>(v * (w - 1));
    };
    painter.fill_rectangle(r, bg);
    if (!m_.any()) return;
    const int x = xof(static_cast<float>(m_.level()));
    const bool over = sq_ <= kSqOff || m_.level() >= sq_;
    painter.fill_rectangle({r.left(), r.top() + 3, x + 1, h - 6}, over ? Color::green() : Color::dark_green());
    painter.fill_rectangle({r.left() + xof(static_cast<float>(m_.min())), r.top(), 1, 3}, Color::yellow());
    painter.fill_rectangle({r.left() + xof(static_cast<float>(m_.max())), r.top(), 1, 3}, Color::yellow());
    painter.fill_rectangle({r.left() + xof(m_.peak()), r.top(), 2, h}, Color::white());
    if (sq_ > kSqOff) {
        const int xs = r.left() + xof(static_cast<float>(sq_));
        painter.fill_rectangle({xs, r.top() + h - 3, 1, 3}, Color::orange());
        painter.fill_rectangle({xs, r.top(), 1, 3}, Color::orange());
    }
}

void FollowView::HistStrip::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto bg = Theme::getInstance()->bg_darkest->background;
    const int h = r.height();
    const int x0 = r.left() + (r.width() - Meter::kHist) / 2;
    if (x0 > r.left()) {
        painter.fill_rectangle({r.left(), r.top(), x0 - r.left(), h}, bg);
        painter.fill_rectangle({x0 + Meter::kHist, r.top(), r.right() - x0 - Meter::kHist, h}, bg);
    }
    const int ys = sq_ > kSqOff ? (sq_ - kBarLo) * h / (kBarHi - kBarLo) : -1;
    for (int i = 0; i < Meter::kHist; i++) {
        const int v = m_.hist(i);
        int y = (v - kBarLo) * h / (kBarHi - kBarLo);
        y = y < 0 ? 0 : (y > h ? h : y);
        const Color c = (sq_ <= kSqOff || v >= sq_) ? Color::green() : Color::dark_green();
        if (h - y > 0) painter.fill_rectangle({x0 + i, r.top(), 1, h - y}, bg);
        if (y > 0) painter.fill_rectangle({x0 + i, r.top() + h - y, 1, y}, c);
        /* the squelch level as a dotted line where the column is below it */
        if (ys > y && ys < h && (i & 3) == 0) painter.fill_rectangle({x0 + i, r.top() + h - ys, 1, 1}, Color::orange());
    }
}

/* ---------------------------------------------------------------- view */

FollowView::FollowView(NavigationView& nav, FollowCtx* ctx)
    : nav_{nav},
      ctx_{ctx},
      settings_{
          "rx_gpsfol",
          app_settings::Mode::RX,
          {{"snd"sv, &snd_},
           {"sq_db"sv, &sq_db_}}},
      text_name_{{0, UI_POS_Y(1), UI_POS_WIDTH(13), UI_POS_HEIGHT(1)}, name_, 1, &ui::font::fixed_8x16},
      field_dl_{{UI_POS_X(14), UI_POS_Y(1)}, 16, {}},
      text_freq_{{0, UI_POS_Y(2), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, freq_, 1, &ui::font::fixed_8x16},
      big_dir_{{0, UI_POS_Y(4), UI_POS_MAXWIDTH, 32}},
      big_el_{{0, UI_POS_Y(4) + 32, UI_POS_MAXWIDTH, 32}},
      text_when_{{0, UI_POS_Y(8), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, when_, 1, &ui::font::fixed_8x16},
      big_level_{{0, UI_POS_Y(9), 112, 32}},
      text_stat_{{114, UI_POS_Y(9), 126, 32}, stat_, kStatRows, &ui::font::fixed_5x8},
      meter_bar_{{0, UI_POS_Y(9) + 34, screen_width, 14}, meter_, sq_db_},
      hist_strip_{{0, UI_POS_Y(9) + 50, screen_width, 40}, meter_, sq_db_},
      text_info_{{0, UI_POS_Y(9) + 92, UI_POS_MAXWIDTH, 8 * kInfoRows}, info_, kInfoRows, &ui::font::fixed_5x8},
      button_reset_{{0, UI_POS_Y(9) + 118, UI_POS_WIDTH_PERCENT(48), 34}, "Reset"},
      button_gps_{{UI_POS_WIDTH_PERCENT(52), UI_POS_Y(9) + 118, UI_POS_WIDTH_PERCENT(48), 34}, "GPS"} {
    setup();
}

void FollowView::setup() {
    add_children({&labels_, &field_lna_, &field_vga_, &field_amp_, &text_ant_, &text_name_, &field_dl_, &text_freq_,
                  &field_volume_, &field_sq_, &field_snd_, &big_dir_, &big_el_, &text_when_, &big_level_, &text_stat_,
                  &meter_bar_, &hist_strip_, &text_info_, &button_reset_, &button_gps_});
    meter_.reset();
    hint_.reset();

    /* the satellite: its own propagator state and pass (the GPS app's are gone) */
    const bool sat_ok = sgp4_init(ctx_->tle, ctx_->sgp4);
    {
        char b[TextLines::kCols + 1];
        Fmt f{b, sizeof(b)};
        f.s(ctx_->tle.name);
        text_name_.set(0, b, Color::cyan());
    }
    OptionsField::options_t opts;
    for (uint8_t k = 0; k < ctx_->n_dl; k++) {
        char mhz[16], b[20];
        put_mhz(mhz, sizeof(mhz), ctx_->dl[k].hz);
        Fmt f{b, sizeof(b)};
        f.s(mhz).c(' ').s(freq_mode_name(ctx_->dl[k].mode));
        opts.emplace_back(b, k);
    }
    field_dl_.set_options(std::move(opts));
    field_dl_.set_selected_index(0, false);
    field_dl_.on_change = [this](size_t i, OptionsField::value_t) {
        if (i >= ctx_->n_dl || i == dl_) return;
        dl_ = static_cast<uint8_t>(i);
        meter_.reset();
        hint_.reset();
        start_rx();
        show_info();
    };

    if (!settings_.loaded()) { /* a passive antenna on UHF: high gains, amp off (switch it on if nothing shows) */
        receiver_model.set_rf_amp(false);
        receiver_model.set_lna(40);
        receiver_model.set_vga(32);
    }
    field_lna_.set_value(receiver_model.lna());
    field_vga_.set_value(receiver_model.vga());
    field_amp_.set_value(receiver_model.rf_amp());
    if (sq_db_ < kSqOff || sq_db_ > 0) sq_db_ = -90;
    if (snd_ > 1) snd_ = 0;
    field_sq_.set_value(sq_db_);
    field_sq_.on_change = [this](int32_t v) {
        sq_db_ = v;
        meter_bar_.set_dirty();
        hist_strip_.set_dirty();
        show_level();
    };
    field_snd_.set_selected_index(snd_, false);
    field_snd_.on_change = [this](size_t, OptionsField::value_t v) {
        if (static_cast<uint32_t>(v) == snd_) return;
        snd_ = static_cast<uint32_t>(v);
        start_rx(); /* Tone runs on the NFM image */
        show_info();
    };
    button_reset_.on_select = [this](Button&) {
        meter_.reset();
        hint_.reset();
        meter_bar_.set_dirty();
        hist_strip_.set_dirty();
        show_level();
    };
    /* back to the GPS app (it re-reads the files and selects this satellite);
     * this view is destroyed inside replace(): nothing of it is used after */
    button_gps_.on_select = [this](Button&) {
        const uint32_t cat = ctx_->tle.catnr;
        NavigationView& nav = nav_;
        nav.replace<GpsView>(cat);
    };

    /* time: whole seconds from the RTC, the fraction from the tick counter
     * since the last RTC second (the tick signal comes at the second) */
    base_unix_ = now_unix();
    base_tick_ = chTimeNow();
    tick_ = rtc_time::signal_tick_second += [this]() {
        rtc::RTC raw;
        rtcGetTime(&RTCD1, &raw);
        base_unix_ = static_cast<double>(rtc_time::rtcToUnixUTC(raw)) - ctx_->utc_off_min * 60.0;
        base_tick_ = chTimeNow();
        show_antenna();
    };

    if (sat_ok) {
        SatSource src;
        src.kind = SatSource::SGP4;
        src.sgp4 = &ctx_->sgp4;
        ctx_->track.start(src, ctx_->obs, base_unix_, true);
        update_doppler(); /* before tuning: the first frequency is already corrected */
    }
    audio::output::start();
    start_rx();
    show_point();
    show_level();
    show_info();
    show_antenna();
}

FollowView::~FollowView() {
    rtc_time::signal_tick_second -= tick_;
    audio::output::stop();
    if (rx_on_) receiver_model.disable();
    baseband::shutdown();
    if (ctx_) {
        ctx_->~FollowCtx();
        chHeapFree(ctx_);
        ctx_ = nullptr;
    }
}

void FollowView::focus() {
    field_sq_.focus();
}

double FollowView::now_unix() const {
    if (base_tick_ == 0) {
        rtc::RTC raw;
        rtcGetTime(&RTCD1, &raw);
        return static_cast<double>(rtc_time::rtcToUnixUTC(raw)) - ctx_->utc_off_min * 60.0;
    }
    double frac = static_cast<double>(chTimeNow() - base_tick_) / CH_FREQUENCY;
    if (frac > 1.5) frac = 1.5; /* a missed tick: do not run away */
    return base_unix_ + frac;
}

/* (Re)start the image for the downlink and Snd, as the stock Audio app does
 * on a modulation change: shut the baseband down, run the image, set the
 * model, enable (which also applies the stored antenna-power setting). */
void FollowView::start_rx() {
    if (!ctx_->n_dl) return;
    const SatFreq& f = ctx_->dl[dl_];
    const bool tone = snd_ == 1;
    ReceiverModel::Mode mode = ReceiverModel::Mode::NarrowbandFMAudio;
    auto tag = portapack::spi_flash::image_tag_nfm_audio;
    uint8_t cfg = nfm_config(f);
    bool ddc = true;
    int32_t offset = 0;
    if (!tone && (f.mode == FMODE_AM || f.mode == FMODE_USB || f.mode == FMODE_LSB || f.mode == FMODE_CW)) {
        mode = ReceiverModel::Mode::AMAudio;
        tag = portapack::spi_flash::image_tag_am_audio;
        /* stock am_configs: 0 DSB 9k, 1 DSB 6k, 2 USB 2k8, 3 LSB 2k8 */
        cfg = f.mode == FMODE_AM ? (f.bw_hz && f.bw_hz <= 7000 ? 1 : 0) : (f.mode == FMODE_LSB ? 3 : 2);
        if (f.mode == FMODE_CW) offset = -700; /* the beacon as a 700 Hz tone in USB */
    } else if (!tone && f.mode == FMODE_WFM) {
        mode = ReceiverModel::Mode::WidebandFMAudio;
        tag = portapack::spi_flash::image_tag_wfm_audio;
        cfg = f.bw_hz && f.bw_hz < 100000 ? 2 : (f.bw_hz && f.bw_hz < 190000 ? 1 : 0); /* 80k / 180k / 200k */
        ddc = false;                                                                   /* the WFM image has no DDC: retune */
    }
    audio::output::mute();
    open_ = false;
    hold_ = 0;
    baseband::shutdown();
    baseband::run_image(tag);
    receiver_model.set_modulation(mode);
    if (mode == ReceiverModel::Mode::NarrowbandFMAudio) {
        receiver_model.set_nbfm_configuration(cfg);
        receiver_model.set_squelch_level(0); /* open: the level squelch below decides */
    } else if (mode == ReceiverModel::Mode::AMAudio) {
        receiver_model.set_am_configuration(cfg);
        receiver_model.set_am_squelch_level(0);
    } else {
        receiver_model.set_wfm_configuration(cfg);
    }
    receiver_model.set_sampling_rate(3072000);
    receiver_model.set_baseband_bandwidth(1750000);
    tuner_.reset(f.hz, ddc, offset);
    const auto c = tuner_.update(look_ok_ ? doppler_hz(f.hz, look_.rr) : 0);
    receiver_model.set_target_frequency(c.center);
    receiver_model.enable();
    rx_on_ = true;
    if (c.ddc) baseband::set_audio_ddc_frequency(c.ddc_hz); /* after enable: its configuration resets the DDC */
    tone_on_ = tone;
    pitch_sent_ = -99999;
    beep_left_ = 0;
    have_level_ = false;
    if (tone) {
        set_pitch(kPitchSilent);
        audio::output::unmute();
    }
    show_antenna();
}

/* One SGP4 evaluation: pointing, range rate, and the DDC / retune. */
void FollowView::update_doppler() {
    const double t = now_unix() + kAhead;
    Look l;
    if (!look_sgp4(ctx_->sgp4, ctx_->obs, t, l)) {
        look_ok_ = false;
        return;
    }
    if (look_ok_ && t > rr_prev_t_ + 0.05) rr_dot_ = (l.rr - rr_prev_) / (t - rr_prev_t_);
    rr_prev_ = l.rr;
    rr_prev_t_ = t;
    look_ = l;
    look_ok_ = true;
    if (!ctx_->n_dl) return;
    const SatFreq& f = ctx_->dl[dl_];
    const int32_t dop = doppler_hz(f.hz, l.rr);
    if (rx_on_) {
        const auto c = tuner_.update(dop);
        if (c.retune) receiver_model.set_target_frequency(c.center);
        if (c.ddc) baseband::set_audio_ddc_frequency(c.ddc_hz);
    }
    /* "437.80823 MHz  Dop +8.23 kHz" (the satellite's signal as received) */
    char b[TextLines::kCols + 1];
    const uint64_t rx = static_cast<uint64_t>(static_cast<int64_t>(f.hz) + dop);
    Fmt g{b, sizeof(b)};
    g.u(static_cast<uint32_t>(rx / 1000000u)).c('.').u(static_cast<uint32_t>((rx % 1000000u) / 10u), 5, '0').s(" Dop ");
    if (dop >= 0) g.c('+');
    g.fixed(dop / 10, 2).s("k");
    text_freq_.set(0, b, Color::white());
}

void FollowView::show_point() {
    char b[TextLines::kCols + 1];
    const Color grey = Color::light_grey();
    if (!look_ok_) {
        big_dir_.set("no orbit", Color::red());
        big_el_.set("", grey);
        text_when_.set(0, "SGP4 refused this TLE", Color::red());
        return;
    }
    const bool up = look_.el >= 0;
    {
        Fmt f{b, 16};
        double a = std::fmod(look_.az, 360.0);
        if (a < 0) a += 360.0;
        f.s(compass16(a)).c(' ').u(static_cast<uint32_t>(std::lround(a)) % 360).s(" deg");
        big_dir_.set(b, up ? Color::green() : grey);
        Fmt g{b, 16};
        if (up)
            g.u(static_cast<uint32_t>(std::lround(look_.el))).s(" deg up");
        else
            g.s("below horizon");
        big_el_.set(b, up ? Color::green() : grey);
    }
    const PassTrack& tr = ctx_->track;
    const double t = now_unix();
    if (tr.ready()) {
        put_countdown(b, sizeof(b), tr.pass(), t);
        Fmt f{b + std::strlen(b), sizeof(b) - std::strlen(b)};
        f.s(", max ").u(static_cast<uint32_t>(tr.pass().max_el < 0 ? 0 : tr.pass().max_el + 0.5f));
        text_when_.set(0, b, up ? Color::green() : Color::yellow());
    } else {
        text_when_.set(0, tr.none() ? "No pass in the next 24 h" : "Finding the pass...", Color::yellow());
    }
}

void FollowView::show_level() {
    char b[TextLines::kCols + 1];
    const Color grey = Color::light_grey();
    text_stat_.begin();
    if (!meter_.any()) {
        big_level_.set("-- dB", grey);
    } else {
        Fmt f{b, 16};
        put_dbs(f, meter_.level());
        f.s(" dB");
        const bool over = sq_db_ <= kSqOff || meter_.level() >= sq_db_;
        big_level_.set(b, over ? Color::green() : Color::white());
        Fmt g{b, sizeof(b)};
        g.s("peak ");
        put_dbs(g, static_cast<int32_t>(std::lround(meter_.peak())));
        text_stat_.set(0, b, Color::white());
        Fmt h{b, sizeof(b)};
        h.s("min ");
        put_dbs(h, meter_.min());
        h.s("  max ");
        put_dbs(h, meter_.max());
        text_stat_.set(1, b, grey);
    }
    {
        Fmt f{b, sizeof(b)};
        if (sq_db_ <= kSqOff)
            f.s("squelch off");
        else if (tone_on_)
            f.s("tone above ").d(sq_db_).s(" dB");
        else
            f.s("sq ").d(sq_db_).s(open_ ? " open" : " shut");
        text_stat_.set(2, b, Color::orange());
    }
    if (ctx_->n_dl) {
        const SatFreq& fq = ctx_->dl[dl_];
        Fmt f{b, sizeof(b)};
        if (tone_on_ || fq.mode == FMODE_NFM || fq.mode == FMODE_DATA)
            f.s("NFM ").s(nfm_name(nfm_config(fq)));
        else
            f.s(freq_mode_name(fq.mode));
        f.s(tone_on_ ? " tone" : "").s(" DDC ").d(tuner_.ddc());
        text_stat_.set(3, b, grey);
    }
    text_stat_.end();
}

/* hint, TLE age and the error estimate, the entry's note */
void FollowView::show_info() {
    char b[TextLines::kCols + 1];
    const Color grey = Color::light_grey(), yellow = Color::yellow();
    text_info_.begin();
    size_t r = 0;
    if (hint_.showing())
        text_info_.set(r++, "Fading high up: rotate antenna", yellow);
    const double age = unix_to_jd(now_unix()) - ctx_->tle.jd_epoch;
    if (ctx_->n_dl) {
        const SatFreq& fq = ctx_->dl[dl_];
        Fmt f{b, sizeof(b)};
        f.s("TLE ").u(static_cast<uint32_t>(age < 0 ? 0 : age)).s(" d");
        if (age > 14) {
            f.s(": too old, refresh");
            text_info_.set(r++, b, Color::red());
        } else {
            const double e = tuning_error_hz(fq.hz, rr_dot_, kClockErrS, age, kPosErrKm);
            f.s(", Doppler err ~");
            if (e >= 1000)
                f.fixed(static_cast<int32_t>(e / 100.0 + 0.5), 1).s(" kHz");
            else
                f.u(static_cast<uint32_t>(e + 0.5)).s(" Hz");
            text_info_.set(r++, b, age > 7 ? yellow : grey);
        }
        Fmt g{b, sizeof(b)};
        g.s(band_name(fq.hz)).c(' ').s(fq.note);
        text_info_.set(r++, b, grey);
        if (r < kInfoRows) {
            if (fq.mode == FMODE_DATA)
                text_info_.set(r++, "data: follow + meter only", grey);
            else if (tone_on_)
                text_info_.set(r++, "Tone: higher = stronger", grey);
        }
    }
    text_info_.end();
}

void FollowView::show_antenna() {
    const int8_t on = (rx_on_ && portapack::get_antenna_bias()) ? 1 : 0;
    if (on == bias_shown_) return;
    bias_shown_ = on;
    text_ant_.set(on ? "Ant: ON" : "Ant: off");
    text_ant_.set_style(on ? Theme::getInstance()->fg_red : Theme::getInstance()->fg_light);
}

void FollowView::set_pitch(int32_t v) {
    if (v == pitch_sent_) return;
    pitch_sent_ = v;
    baseband::set_pitch_rssi(v, true);
}

/* Channel statistics (stock audio images, 10 a second): meter, hint, and
 * the level squelch of the voice audio (mute / unmute only on a change, with
 * 2 dB hysteresis and 0.5 s hold). */
void FollowView::on_stats(int32_t db) {
    const systime_t now = chTimeNow();
    float dt = have_level_ ? static_cast<float>(now - last_stats_) / CH_FREQUENCY : 0.1f;
    if (dt > 1.0f) dt = 1.0f;
    have_level_ = true;
    last_stats_ = now;
    level_ = db;
    meter_.feed(db, dt);
    const bool was = hint_.showing();
    hint_.feed(db, look_ok_ ? static_cast<float>(look_.el) : -90.0f);
    if (hint_.showing() != was) show_info();
    if (!tone_on_) {
        const bool want = sq_db_ <= kSqOff || db >= sq_db_ - (open_ ? 2 : 0);
        if (want)
            hold_ = 5;
        else if (hold_)
            hold_--;
        const bool open = want || hold_ > 0;
        if (open != open_) {
            open_ = open;
            if (open)
                audio::output::unmute();
            else
                audio::output::mute();
        }
    }
    meter_bar_.set_dirty();
    if (meter_.columns_done() != cols_shown_) {
        cols_shown_ = meter_.columns_done();
        hist_strip_.set_dirty();
    }
    show_level();
}

/* Tone: the NFM image's pitch-by-level tone (stock PitchRSSIConfigure),
 * pulsed 50 ms on with gaps that shrink as the level rises: silent below Sq,
 * one beep per ~0.45 s at Sq, continuous 24 dB above; pitch 800 Hz at Sq to
 * 2000 Hz 30 dB above. */
void FollowView::sound_frame() {
    if (!tone_on_) return;
    const int32_t over = level_ - (sq_db_ <= kSqOff ? kBarLo : sq_db_);
    if (!have_level_ || over < 0 || chTimeNow() - last_stats_ > MS2ST(1000)) {
        set_pitch(kPitchSilent);
        beep_left_ = 0;
        beep_phase_ = 0;
        return;
    }
    const int32_t pitch = -200 + (over > 30 ? 30 : over) * 40; /* tone = value + 1000 Hz */
    const int gap = 24 - static_cast<int>(over);
    if (gap <= 0) {
        set_pitch(pitch);
        return;
    }
    if (beep_left_ == 0) {
        beep_phase_ ^= 1;
        beep_left_ = static_cast<uint8_t>(beep_phase_ ? 3 : gap);
    }
    beep_left_--;
    set_pitch(beep_phase_ ? pitch : kPitchSilent);
}

/* Every display frame: the Doppler 4 times a second (one SGP4 call), else a
 * slice of the pass search; the tone pulses. */
void FollowView::on_frame() {
    frames_++;
    const systime_t now = chTimeNow();
    if (now - last_dop_ >= kDopPeriod) {
        last_dop_ = now;
        update_doppler();
        show_point();
        if ((frames_ & 63) == 0) show_info(); /* the error estimate follows the Doppler rate */
    } else if (ctx_->track.busy()) {
        while (ctx_->track.step(1) && chTimeNow() - now < kTrackSlice) {
        }
    } else if (ctx_->track.ready() && ctx_->track.pass().set != 0 && now_unix() > ctx_->track.pass().set + 60.0) {
        SatSource src; /* that pass is over: the next one */
        src.kind = SatSource::SGP4;
        src.sgp4 = &ctx_->sgp4;
        ctx_->track.start(src, ctx_->obs, now_unix(), true);
    }
    sound_frame();
    if (have_level_ && now - last_stats_ > MS2ST(1500) && meter_.any()) {
        big_level_.set("-- dB", Color::light_grey()); /* no statistics: the baseband stopped? */
    }
}

}  // namespace ui::external_app::gps

#endif /* PRALINE */
