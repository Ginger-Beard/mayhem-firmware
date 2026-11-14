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

#ifndef __UI_GPS_H__
#define __UI_GPS_H__

/* GPS: the HackRF Pro as a GPS L1 C/A receiver, receive only.
 *   Fix     position from a 20 ms snapshot + an ephemeris file on the SD card
 *           (coarse-time navigation); "Autonomous" is not available yet.
 *           Save writes /SETTINGS/position.ini (source "HackRF GPS").
 *   Heard   satellites received in a snapshot: PRN, C/N0, Doppler, and a sky
 *           plot when orbits and a position are known
 *   Passes  predicted passes from orbit files (no reception): GPS almanac /
 *           ephemeris and TLE element sets (ISS ...), with a sky plot
 *   Follow  (from the satellite page) a TLE satellite's downlink, Doppler
 *           corrected, with a signal meter, tone and the pointing readout
 * Files: /GPS/ALMANAC.TXT (YUMA), /GPS/EPHEM.RNX (RINEX 2/3 navigation),
 * /GPS/TLE.TXT, /GPS/SATFREQ.TXT (downlinks) (sdcard/GPS/README.TXT). The baseband images on the card (gps_image.hpp) do the signal
 * processing and lends the app M4 RAM for the orbit tables and the solver, so
 * they do not take the M0 heap. */

#include "ch.h"
#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_tabview.hpp"
#include "app_settings.hpp"
#include "saved_position.hpp"
#include "radio_state.hpp"
#include "message.hpp"

#ifdef PRALINE
#include "ui_geomap.hpp"
#include "file.hpp"
#include "gps_rx_result.hpp"
#include "gps_orbit.hpp"
#include "gps_sgp4.hpp"
#include "gps_pass.hpp"
#include "gps_solve.hpp"
#include "gps_point.hpp"
#include "gps_freq.hpp"
#include "gps_follow.hpp"
#include "gps_auto.hpp"
#include "gps_pvt.hpp"
#include "gps_log.hpp"
#include "gps_fmt.hpp"
#include "gps_clock.hpp"
#include "gps_image.hpp"
#include "gps_keep.hpp"
#include "gps_sys.hpp"
#include "gal_e1.hpp"
#include "gal_inav.hpp"
#include "gal_eph.hpp"
#endif

namespace ui::external_app::gps {

/* 16x16 menu icon (a satellite), shared with the external app header. */
inline constexpr uint8_t bitmap_icon_gps_data[32] = {
    0x00, 0x00, 0x0E, 0x00, 0x1F, 0x00, 0x1F, 0x20, 0x0E, 0x70, 0x01, 0xF8, 0x03, 0xF0, 0x07, 0xE0,
    0x0F, 0xC0, 0x1F, 0x80, 0x0E, 0x70, 0x04, 0xF8, 0x00, 0xF8, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00};
inline constexpr Bitmap bitmap_icon_gps{{16, 16}, bitmap_icon_gps_data};

#ifdef PRALINE

/* Lines of text painted from char buffers (no std::string per line). */
class TextLines : public Widget {
   public:
    static constexpr size_t kCols = 30;
    struct Line {
        char text[kCols + 1];
        Color color;
    };
    TextLines(Rect r, Line* lines, size_t rows, const Font* font)
        : Widget{r}, lines_{lines}, rows_{rows}, font_{font} {}
    TextLines(const TextLines&) = delete;
    TextLines& operator=(const TextLines&) = delete;
    void paint(Painter& painter) override;
    void set(size_t row, const char* text, Color color);
    void clear();
    /* begin(), set() the rows wanted, end(): rows not set are blanked. Only
     * rows whose text or colour changed mark the widget dirty, so an
     * unchanged screen is not repainted. */
    void begin() { used_ = 0; }
    void end();
    /* one row drawn inverted (the wheel selection), -1 for none */
    void set_highlight(int row);
    /* row `row` is painted over its first `cols` columns only: a neighbouring
     * widget owns the rest of that row (otherwise the blank fill erases it) */
    void set_short_row(int row, uint8_t cols) {
        short_row_ = static_cast<int8_t>(row);
        short_cols_ = cols;
    }
    /* a tap on a row (rows are line_height apart) */
    std::function<void(size_t)> on_row{};
    bool on_touch(const TouchEvent event) override;

   private:
    Line* lines_;
    size_t rows_;
    const Font* font_;
    uint32_t used_{0};
    int8_t hl_{-1};
    int8_t short_row_{-1};
    uint8_t short_cols_{0};
};

/* 8x16 text drawn at twice the size (the pointing readout). */
class BigText : public Widget {
   public:
    BigText(Rect r)
        : Widget{r} {}
    void set(const char* text, Color color);
    void paint(Painter& painter) override;

   private:
    char text_[16]{};
    Color color_{Color::white()};
};

/* Azimuth / elevation plot: north up, the horizon as the outer ring. */
class SkyPlot : public Widget {
   public:
    struct Dot {
        int16_t az; /* degrees */
        int16_t el; /* degrees */
        char label[4];
        Color color;
        bool filled;
        bool sel; /* the wheel selection: ringed */
    };
    static constexpr size_t kMax = 32;
    SkyPlot(Rect r, const char* title)
        : Widget{r}, title_{title} {}
    SkyPlot(const SkyPlot&) = delete;
    SkyPlot& operator=(const SkyPlot&) = delete;
    void clear() {
        n_ = 0;
        hash_ = 2166136261u;
    }
    void add(int16_t az, int16_t el, const char* label, Color color, bool filled, bool selected = false);
    /* the selected satellite's pass arc (nullptr: none) and a "now" marker */
    void set_track(const PassTrack* t) {
        track_ = t;
        hash_ = (hash_ ^ (t && t->ready() ? static_cast<uint32_t>(t->pass().rise) : 0u)) * 16777619u;
    }
    /* focusable: the wheel then steps through the satellites (the parent view
     * gets the encoder and Select events) */
    bool on_touch(const TouchEvent event) override;
    void on_focus() override { set_dirty(); }
    void on_blur() override { set_dirty(); }
    /* repaint only if the dots differ from what was last painted */
    void commit() {
        if (hash_ != shown_) {
            shown_ = hash_;
            set_dirty();
        }
    }
    void paint(Painter& painter) override;

   private:
    Dot dots_[kMax]{};
    size_t n_{0};
    uint32_t hash_{2166136261u}, shown_{0};
    const PassTrack* track_{nullptr};
    const char* title_; /* drawn top left, e.g. "GPS now" */
};

/* Where a satellite is seen from the saved position (the sky cache, filled
 * one satellite per display frame: all views read it, none computes orbits). */
struct SatGeo {
    float az{0}, el{-90};
    float rr{0}; /* range rate, m/s (Doppler = -rr / lambda) */
};

/* What lives in the M4 RAM the GPS images lend (GpsRxStatusMessage::scratch; kept across them, gps_keep.hpp). */
struct GpsScratch {
    static constexpr size_t kMaxTle = PassList::kMax; /* 12 */
    static constexpr size_t kMaxFreq = 24;
    static constexpr uint8_t kNoFreq = 0xFF;
    Orbit orbits[32]; /* per PRN: a fresh ephemeris, else the almanac */
    AlmRec alm[32];   /* the almanac, kept for when an ephemeris goes stale */
    Tle tles[kMaxTle];
    Sgp4 search; /* pass search */
    Sgp4 sky;    /* sky cache (TLE dots) */
    SolveWork solve;
    SatGeo geo[32];
    SatGeo tle_geo[kMaxTle];
    Sgp4 sel;              /* the selected TLE satellite */
    PassTrack track;       /* the selected satellite's pass (current or next) and arc */
    uint32_t heard_at[32]; /* unix s of the last confirmation, 0 never */
    uint8_t heard_cn0[32];
    SatFreq freq[kMaxFreq];    /* /GPS/SATFREQ.TXT */
    uint8_t tle_freq[kMaxTle]; /* first downlink of each TLE, kNoFreq none */
    Tle tle_tmp;               /* the loader parses here first */
    PassList passes;           /* the Passes tab's list */
    /* "Autonomous": subframes / ephemerides / almanac pages / measurements
     * from the tracking loops, the PVT work space and its last solution */
    AutoState autos;
    PvtSolution pvt;          /* the solver thread's output */
    PvtSolution pvt_fix;      /* the last accepted one (shown, saved) */
    uint32_t cache_sf[3][10]; /* the loader assembles an EPH_AIR.TXT record here */
    nav::Iono iono_cache;     /* ionosphere from EPH_AIR.TXT (an "S prn week 4 ..." line) */
    double iono_cache_t{0};   /* GPS s of the subframe that carried it, 0 none */
    uint8_t cache_have;
    LogRing log; /* diagnostics log rows waiting for the card (ui_gps_log.cpp) */
    SysSky sys;  /* the other systems' satellites now (GNSSALM.TXT, ui_gps_sys.cpp) */
    /* Galileo for the aided E1 search (ui_gps_gal.cpp): orbits from the last
     * GNSSALM.TXT pass, and per PRN the search state (0 not yet, 1 searching,
     * 2 heard, 3 not found), C/N0 dB-Hz and peak / mean x10 */
    static constexpr size_t kMaxGal = 30;
    GalAlm gal[kMaxGal];
    uint8_t n_gal;
    uint8_t gal_state[37];
    uint8_t gal_cn0[37];
    /* the tracking image's Galileo channels as last seen (5 times a second):
     * what the log's events compare with (secondary code, side-peak jumps,
     * losses) and what the Heard rows show */
    struct GalPrev {
        uint8_t prn, state, flags, fl_fixes;
        uint16_t side_jumps, cn0c_x10;
        uint32_t lock_ms;
    };
    GalPrev gal_prev[gal_acq::kMaxTrk];
};

/* What the tracking image lends the app for Galileo (gal_acq::kNavWorkBytes,
 * valid while the tracking image runs; Galileo needs it): the I/NAV
 * decoder, the ephemerides / GGTO decoded from it, the Galileo measurement
 * set of the PVT being solved and the solver's GPS + Galileo outputs. */
struct GalNavMem {
    gal_inav::Work inav;
    gal_eph::Store eph;
    gal_acq::MeasSet meas;                   /* the set of the GPS measurement in AutoState::meas */
    uint8_t meas_ok;                         /* meas belongs to that GPS set */
    uint8_t why[gal_inav::Work::kCh];        /* gal_eph::Why per channel, last PVT input */
    uint8_t gate;                            /* gal_eph::Gate of the last GPS + Galileo solve */
    PvtSolution mix;                         /* GPS + Galileo */
    PvtSolution alone;                       /* Galileo only (System wheel on Galileo) */
    float mix_gal_rms, mix_gps_rms;          /* residual rms of each system's satellites in `mix`, m */
    int16_t mix_res_dm[gal_inav::Work::kCh]; /* Galileo residuals in `mix` (dm; 32767 not used), by PVT order */
    uint8_t mix_prn[gal_inav::Work::kCh];
};

class GpsView;

Color sys_color(uint8_t sys); /* one colour per system on the sky plots (ui_gps_sys.cpp) */

/* Incremental reader of the orbit files: a few KB per display frame, so the
 * UI stays responsive and nothing large sits on the stack. */
class OrbitLoader {
   public:
    enum Stage : uint8_t { IDLE,
                           ALMANAC,
                           EPHEMERIS,
                           EPH_AIR, /* /GPS/EPH_AIR.TXT: off-air ephemerides of earlier runs */
                           SATFREQ,
                           TLES,
                           DONE };
    OrbitLoader() = default;
    OrbitLoader(const OrbitLoader&) = delete;
    OrbitLoader& operator=(const OrbitLoader&) = delete;
    ~OrbitLoader();
    void start(double now_gps, GpsView& v);
    /* Lines until the frame's time slice (started at t0) is used, at least
     * one; true while there is more to do. */
    bool step(GpsView& v, systime_t t0);
    Stage stage() const { return stage_; }

   private:
    File* file_{nullptr};
    Stage stage_{IDLE};
    char rbuf_[256]{};
    size_t rpos_{0}, rlen_{0};
    char line_[96]{};
    size_t llen_{0};
    char tle_name_[17]{};
    char tle_l1_[72]{};
    uint8_t tle_state_{0};
    uint8_t air_prn_{0}; /* EPH_AIR record being assembled (subframes in GpsScratch::cache_sf) */
    uint16_t line_no_{0};
    bool eof_{false};
    double now_gps_{0};
    YumaParser yuma_{0};
    RinexNavParser rinex_{};
    bool open(Stage s, GpsView& v);
    bool open_after(Stage s, GpsView& v); /* the next file that exists */
    void close();
    void take_tle(GpsView& v);
    bool next_line();
    void take(GpsView& v);
};

class PassesView : public View {
   public:
    PassesView(GpsView& main, Rect r);
    void focus() override;
    void refresh();           /* position / time / file lines */
    void update_sky();        /* every few seconds */
    void restart_passes();    /* new position, time or files */
    void frame(systime_t t0); /* the pass search until the frame's slice is used */
    void tick();              /* countdowns ("in 1h05") */
    void sync_sys();          /* the wheel moved on the other tab */

   private:
    GpsView& main_;
    static constexpr size_t kRows = 4;
    TextLines::Line head_[kRows]{};
    TextLines text_head_;
    Button button_pos_;
    Button button_load_;
    Labels labels_zone_;
    NumberField field_zone_h_;
    OptionsField field_zone_m_;
    SkyPlot sky_;
    OptionsField field_sys_;
    static constexpr size_t kSideRows = 12;
    TextLines::Line side_[kSideRows]{};
    TextLines text_side_;
    static constexpr size_t kPassRows = 4; /* a window on up to 12, following the selection */
    TextLines::Line pass_lines_[kPassRows]{};
    TextLines text_passes_;
    uint8_t order_[PassList::kMax]{}; /* list position -> TLE index (by time) */
    uint8_t off_{0};                  /* first list position shown */
    uint8_t side_prn_[kSideRows]{};   /* side-list row -> PRN index + 1 (0: none) */
    PassList* pl() const;             /* in the lent RAM (nullptr before it) */
    size_t npass() const;
    int pos_of_tle(uint8_t idx) const; /* list position of a TLE, -1 */
    void show_passes();
    void zone_changed();
    size_t side_other(size_t row, uint8_t sys) __attribute__((noinline)); /* rows for another system's satellites */
    int items() const;                                                    /* TLE pass rows, then GPS side rows */
    void item(int i, uint8_t& kind, uint8_t& idx) const;
    void show_selection();
    bool on_encoder(const EncoderEvent delta) override;
    bool on_key(const KeyEvent key) override;
    void tap(uint8_t kind, uint8_t idx);
};

class HeardView : public View {
   public:
    HeardView(GpsView& main, Rect r);
    void focus() override;
    void show();     /* status / results changed */
    void sync_sys(); /* the wheel moved on the other tab */
    /* what the rows say (diagnostics log) */
    const TextLines::Line* status_lines() const { return status_; }
    size_t status_rows() const { return kStatusRows; }
    const TextLines::Line* list_lines() const { return list_; }
    size_t list_rows() const { return kListRows; }

   private:
    void show_tracking();
    void show_gps() __attribute__((noinline));   /* GPS / All L1 */
    void show_other() __attribute__((noinline)); /* the wheel shows Galileo, BeiDou, GLONASS or SBAS */
    GpsView& main_;
    OptionsField field_depth_;
    OptionsField field_sys_;
    Checkbox check_gal_{{UI_POS_X(12), 0}, 2, "E1", true}; /* the aided Galileo E1 search */
    Button button_search_;
    Button button_stop_;
    static constexpr size_t kStatusRows = 2;
    TextLines::Line status_[kStatusRows]{};
    TextLines text_status_;
    static constexpr size_t kListRows = 15;
    TextLines::Line list_[kListRows]{};
    TextLines text_list_;
    SkyPlot sky_;
    uint8_t list_prn_[kListRows]{}; /* list row -> PRN index + 1 (0: none) */
    bool on_encoder(const EncoderEvent delta) override;
    bool on_key(const KeyEvent key) override;
};

class FixView : public View {
   public:
    FixView(GpsView& main, Rect r);
    FixView(const FixView&) = delete;
    FixView& operator=(const FixView&) = delete;
    void focus() override;
    void show();
    const TextLines::Line* lines() const { return lines_; }
    size_t rows() const { return kRows; }
    uint32_t method() const { return static_cast<uint32_t>(field_method_.selected_index_value()); }
    void set_log(bool on) { check_log_.set_value(on); }

   private:
    GpsView& main_;
    OptionsField field_method_;
    /* diagnostics log (stock convention: a Log box on the main view; saved) */
    Checkbox check_log_{{UI_POS_X(12), 0}, 3, "Log", true};
    Button button_fix_;
    Button button_save_;
    Button button_clock_;           /* "Set clock": the PortaPack RTC from GPS time, only when pressed */
    bool fix_running_shown_{false}; /* button_fix_ shows "Stop" */
    const char* save_msg_{nullptr}; /* bottom row after "Save pos" */
    Color save_color_{Color::white()};
    void show_blocker(size_t row);
    void show_auto();
    void show_log_error();
    static constexpr size_t kRows = 11;
    TextLines::Line lines_[kRows]{};
    TextLines text_;
};

/* Manual position entry (degrees / minutes / seconds, the stock GeoPos
 * widget); Save writes the shared position file with source "Manual". */
class PositionView : public View {
   public:
    PositionView(NavigationView& nav, GpsView& main);
    void focus() override;
    std::string title() const override { return "GPS position"; };

   private:
    GpsView& main_;
    GeoPos geopos_;
    Text text_info_;
    Button button_save_;
};

/* One satellite: where to point now and in 30 s (compass words), the pass
 * countdown, the pass arc with a "now" marker, a plain-words summary and the
 * details (pushed from Passes / Heard with Select or a second tap). */
class SatView : public View {
   public:
    SatView(NavigationView& nav, GpsView& main);
    ~SatView();
    SatView(const SatView&) = delete;
    SatView& operator=(const SatView&) = delete;
    void focus() override;
    std::string title() const override { return "Satellite"; };

   private:
    GpsView& main_;
    static constexpr size_t kTopRows = 2, kSumRows = 4, kInfoRows = 11;
    TextLines::Line top_[kTopRows]{};
    TextLines text_top_;
    BigText big_dir_;
    BigText big_el_;
    TextLines::Line now_[1]{};
    TextLines text_now_;
    SkyPlot sky_;
    TextLines::Line info_[kInfoRows]{};
    TextLines text_info_;
    TextLines::Line sum_[kSumRows]{};
    TextLines text_sum_;
    Button button_follow_;
    SignalToken tick_{};
    void refresh();
};

/* What Follow takes from the GPS app before the app (and with it the lent M4
 * RAM holding the orbit tables) closes: allocated on the heap by
 * GpsView::follow(), owned by FollowView. */
struct FollowCtx {
    static constexpr size_t kMaxDl = 4;
    Tle tle{};
    Sgp4 sgp4{}; /* initialised by FollowView */
    PassTrack track{};
    Observer obs{};
    int32_t utc_off_min{0};
    double clock_corr_s{0}; /* GPS time - RTC when the GPS app knew it (0 otherwise) */
    uint8_t n_dl{0};
    SatFreq dl[kMaxDl]{}; /* this satellite's downlinks from SATFREQ.TXT */
};

/* Follow: the selected satellite's downlink on a stock audio image (NFM, AM
 * incl. USB/LSB/CW, WFM), the Doppler corrected 4 times a second (the
 * baseband's DDC shift on NFM / AM, a retune on WFM), a signal meter with
 * peak hold, min/max and a 40 s history, Voice or Tone (the stock NFM image's
 * pitch-by-level tone, pulsed faster as the level rises), a level squelch,
 * and the pointing readout. It replaces the GPS app (so its baseband stops); "GPS"
 * goes back to it, the title bar's back arrow to the menu. Message IDs
 * registered: DisplayFrameSync, ChannelStatistics; none of its stock widgets
 * (gain fields, volume, options, buttons) registers any. */
class FollowView : public View {
   public:
    FollowView(NavigationView& nav, FollowCtx* ctx);
    ~FollowView();
    FollowView(const FollowView&) = delete;
    FollowView& operator=(const FollowView&) = delete;
    void focus() override;
    std::string title() const override { return "Follow"; };

   private:
    /* the meter bar (-110..-10 dB: level, peak hold, min/max, squelch) and
     * the history strip (one column per 0.2 s, 40 s), two widgets so the
     * strip repaints only when a column is added */
    class MeterBar : public Widget {
       public:
        MeterBar(Rect r, const Meter& m, const int32_t& sq)
            : Widget{r}, m_{m}, sq_{sq} {}
        void paint(Painter& painter) override;

       private:
        const Meter& m_;
        const int32_t& sq_;
    };
    class HistStrip : public Widget {
       public:
        HistStrip(Rect r, const Meter& m, const int32_t& sq)
            : Widget{r}, m_{m}, sq_{sq} {}
        void paint(Painter& painter) override;

       private:
        const Meter& m_;
        const int32_t& sq_;
    };

    NavigationView& nav_;
    FollowCtx* ctx_;
    RxRadioState radio_state_{ReceiverModel::Mode::NarrowbandFMAudio};
    uint32_t snd_{0};    /* 0 voice, 1 tone (saved) */
    int32_t sq_db_{-90}; /* level squelch, dB (saved) */
    app_settings::SettingsManager settings_;

    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "LNA:   VGA:   AMP:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(3)}, "Vol:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(7), UI_POS_Y(3)}, "Sq:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(15), UI_POS_Y(3)}, "Snd:", Theme::getInstance()->fg_light->foreground}};
    LNAGainField field_lna_{{UI_POS_X(4), UI_POS_Y(0)}};
    VGAGainField field_vga_{{UI_POS_X(11), UI_POS_Y(0)}};
    RFAmpField field_amp_{{UI_POS_X(18), UI_POS_Y(0)}};
    Text text_ant_{{UI_POS_X(22), UI_POS_Y(0), UI_POS_WIDTH(8), UI_POS_HEIGHT(1)}, ""};
    TextLines::Line name_[1]{};
    TextLines text_name_;
    OptionsField field_dl_;
    TextLines::Line freq_[1]{};
    TextLines text_freq_;
    AudioVolumeField field_volume_{{UI_POS_X(4), UI_POS_Y(3)}};
    NumberField field_sq_{{UI_POS_X(10), UI_POS_Y(3)}, 4, {-120, 0}, 1, ' '};
    OptionsField field_snd_{{UI_POS_X(19), UI_POS_Y(3)}, 5, {{"Voice", 0}, {"Tone", 1}}};
    BigText big_dir_;
    BigText big_el_;
    TextLines::Line when_[1]{};
    TextLines text_when_;
    BigText big_level_;
    static constexpr size_t kStatRows = 4;
    TextLines::Line stat_[kStatRows]{};
    TextLines text_stat_;
    Meter meter_{};
    MeterBar meter_bar_;
    HistStrip hist_strip_;
    uint32_t cols_shown_{0};
    static constexpr size_t kInfoRows = 3;
    TextLines::Line info_[kInfoRows]{};
    TextLines text_info_;
    Button button_reset_;
    Button button_gps_;

    /* receiver */
    uint8_t dl_{0};       /* the downlink followed */
    bool tone_on_{false}; /* the image runs the pitch tone */
    bool rx_on_{false};
    Tuner tuner_{};
    /* the constructor's body, out of line: its locals are not on the stack
     * while the settings file is read */
    void setup() __attribute__((noinline));
    void start_rx(); /* (re)start the image for the downlink and Snd */
    /* time: the RTC's seconds plus the ticks since the last second */
    double base_unix_{0};
    systime_t base_tick_{0};
    double now_unix() const;
    SignalToken tick_{};
    /* schedule */
    systime_t last_dop_{0};
    double rr_prev_{0}, rr_prev_t_{0}, rr_dot_{0};
    Look look_{};
    bool look_ok_{false};
    void update_doppler(); /* one SGP4 call + tuning */
    void show_point();
    void show_level();
    void show_info();
    void show_antenna();
    int8_t bias_shown_{-1};
    /* level and sound */
    int32_t level_{-128};
    bool have_level_{false};
    systime_t last_stats_{0};
    FadeHint hint_{};
    bool open_{false}; /* voice: squelch open (audio unmuted) */
    uint8_t beep_phase_{0}, beep_left_{0}, hold_{0};
    int32_t pitch_sent_{-99999};
    void set_pitch(int32_t v);
    void on_stats(int32_t db);
    void sound_frame();
    uint8_t frames_{0};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_frame();
        }};
    MessageHandlerRegistration message_handler_stats{
        Message::ID::ChannelStatistics,
        [this](const Message* const p) {
            this->on_stats(static_cast<const ChannelStatisticsMessage*>(p)->statistics.max_db);
        }};
    void on_frame();
};

class GpsView : public View {
   public:
    /* reselect: a TLE catalogue number to select once the files are read
     * (coming back from Follow), 0 none */
    GpsView(NavigationView& nav, uint32_t reselect = 0);
    ~GpsView();

    GpsView(const GpsView&) = delete;
    GpsView& operator=(const GpsView&) = delete;

    void focus() override;
    std::string title() const override { return "GPS"; };

   private:
    friend class OrbitLoader;
    friend class PassesView;
    friend class HeardView;
    friend class FixView;
    friend class PositionView;
    friend class SatView;
    friend class FollowView;

    enum Job : uint8_t { JOB_NONE,
                         JOB_HEARD,
                         JOB_FIX,
                         JOB_AUTO }; /* snapshot search, then the tracking loops ("Autonomous") */

    NavigationView& nav_;
    RxRadioState radio_state_{
        1575420000 /* GPS L1 */,
        1750000 /* baseband filter */,
        2048000 /* 2048 samples = 1 ms = one DMA buffer */,
        /* SpectrumAnalysis: the only mode ReceiverModel tunes exactly; every other
         * mode tunes fs/4 (512 kHz) off and expects the baseband to shift it back,
         * which the GPS images do not (GpsView's constructor enforces this mode again after
         * the settings file is read) */
        ReceiverModel::Mode::SpectrumAnalysis};
    /* Time zone of the PortaPack clock: local standard time minus UTC, in
     * minutes (saved; e.g. -300 for US Eastern). With Mayhem's DST setting on,
     * the RTC hardware keeps standard time and this is the standard offset;
     * with it off, the clock shows whatever it was set to and this must be
     * the offset in force (e.g. -240 in US summer). */
    int32_t utc_off_min_{0};
    uint32_t tab_{2};                /* last tab (saved) */
    uint32_t depth_{1};              /* Heard snapshot: 0 = 10 ms, 1 = 20 ms (saved) */
    uint32_t log_on_{0};             /* diagnostics log (Log box on the Fix tab, saved) */
    uint32_t sys_view_{VIEW_ALL_L1}; /* the System wheel (Heard, Passes; saved) */
    uint32_t gal_on_{0};             /* the aided Galileo E1 search (Heard "E1" box; saved) */
    uint32_t gal_fix_on_{1};         /* Galileo in the fix (rx_gps.ini gal_fix; 0 = never, GPS-only fixes) */
    app_settings::SettingsManager settings_;
    int8_t bias_shown_{-1};  /* last antenna-power state shown (read only) */
    uint32_t tab_shown_{99}; /* tab whose content was last refreshed */
    bool rx_on_{false};      /* receiver enabled (stock path: applies the stored antenna power) */

    /* Lent M4 RAM: orbit tables, TLEs, propagator states, solver work. */
    GpsScratch* mem_{nullptr};
    uint8_t n_alm_{0}, n_eph_{0}, n_tle_{0};
    uint8_t n_freq_{0}, freq_bad_{0}; /* SATFREQ.TXT: downlinks read, bad lines */
    uint16_t freq_bad_line_{0};       /* the first bad line's number */
    uint32_t reselect_{0};
    double eph_toe_newest_{0};
    /* what the card held at the last load (for the plain-language states) */
    bool alm_file_{false}, eph_file_{false}, tle_file_{false}, freq_file_{false};
    double alm_toa_{0};         /* the almanac's time of applicability, GPS s */
    double eph_file_newest_{0}; /* newest GPS record in EPHEM.RNX, used or not */
    OrbitLoader loader_{};
    bool load_wanted_{true};
    bool loading() const;
    double alm_age_days() const; /* -1 without an almanac */
    /* an ephemeris older than this (either way) is replaced by the almanac */
    static constexpr double kEphMaxAgeS = 4 * 3600.0;
    void refresh_orbits(); /* stale ephemerides -> almanac (not while solving) */

    /* The sky cache (GpsScratch::geo): one satellite per frame, a new pass
     * every few seconds; the views draw from it. */
    uint8_t geo_idx_{0};
    bool geo_ready_{false};  /* a complete pass since the last restart */
    bool geo_active_{false}; /* a pass in progress */
    uint32_t geo_started_{0};
    void geo_restart();
    void geo_step();

    /* The wheel selection (shared by Passes and Heard) and its pass track. */
    enum SelKind : uint8_t { SEL_NONE,
                             SEL_GPS,
                             SEL_TLE };
    SelKind sel_kind_{SEL_NONE};
    uint8_t sel_idx_{0}; /* PRN index or TLE index */
    bool sel_source(SatSource& src);
    void select(uint8_t kind, uint8_t idx); /* also restarts the pass track */
    void open_selected();                   /* push SatView */
    /* Follow the selected TLE satellite (from SatView): copies what Follow
     * needs to the heap, pops SatView and replaces this view; false (nothing
     * done) without a downlink, a position or heap. */
    bool follow();
    uint8_t sel_downlinks() const; /* SATFREQ entries of the selected TLE */
    void sel_name(char* buf, size_t size) const;
    size_t geo_units() const { return 32u + 2u * n_tle_; }

    saved_position::Position pos_{};
    bool pos_valid_{false};
    Vec3 rx_{};
    Observer obs_{};

    /* Baseband status (copied from GpsRxStatusMessage) and the current search. */
    struct Status {
        uint8_t state{GpsRxStatusMessage::IDLE};
        uint8_t level{GpsRxStatusMessage::LEVEL_OK};
        uint8_t confirming{0};
        uint8_t done_prn{0}, total_prn{0}, k_ms{0};
        uint16_t rms_x10{0}, clip_permille{0};
        uint32_t seq{0}, units{0}, unit_cycles{0};
    } status_{};
    const gps_rx::Result* results_{nullptr};
    Job job_{JOB_NONE};
    bool awaiting_seq_{false};
    uint32_t job_seq_{0};
    uint32_t job_units_{0}; /* expected work units, for the progress */
    bool done_{false};      /* results of job_seq_ complete */
    bool solve_pending_{false};
    Job want_job_{JOB_NONE}; /* a search waiting for the sky cache */
    uint8_t want_kms_{0};
    uint32_t status_sig_{0}; /* what the views last showed of the status */

    /* Last fix. */
    enum FixMode : uint8_t { FIX_NONE,
                             FIX_PRECISE,  /* every satellite from a fresh ephemeris */
                             FIX_COARSE }; /* almanac orbits (bundled file) */
    Solution fix_{};
    bool fix_ok_{false};
    bool fix_tried_{false};
    uint32_t fix_fat_{0};
    uint8_t fix_heard_{0};
    uint8_t fix_cand_{0}; /* heard with an orbit */
    FixMode fix_mode_{FIX_NONE};
    double fix_eph_age_s_{0};
    double fix_alm_age_d_{0};
    uint32_t fix_est_m_{0};

    /* The solver runs in a low-priority thread (0.2 s to seconds of soft
     * double on the M0): touch and the display keep running meanwhile. */
    Thread* solver_{nullptr};
    volatile bool solver_done_{false};
    int solve_n_{0};
    double solve_t_{0};
    Vec3 solve_prior_{};
    double solve_max_rms_{30.0};
    double solve_max_dt_{60.0};
    Solution solve_out_{};
    bool solve_ok_{false};
    uint8_t solve_kind_{0}; /* 0 snapshot fix, 1 PVT from the tracking loops */
    int solve_pvt_n_{0};
    double solve_pvt_trx_{0};
    bool solve_pvt_eph_{false};
    static msg_t solver_fn(void* arg);
    void finish_solve();
    bool solving() const { return solver_ != nullptr; }

    double now_unix() const;   /* UTC from the RTC and the zone */
    double local_unix() const; /* the clock as shown (incl. Mayhem DST), as if it were UTC */
    /* GPS time: from the satellites once the tracking loops decoded a TOW
     * (the clock and Zone are then not needed), else the RTC and Zone */
    double now_gps() const { return unix_to_gps(now_unix()) + (gps_time_ok_ ? gps_off_s_ : 0.0); }
    /* UTC for everything the app computes (passes, sky, search, Follow): the RTC
     * corrected by GPS time once the satellites gave it, so a wrong clock cannot
     * block a fix or the predictions. The RTC itself is written only by "Set clock". */
    double now_best() const { return now_unix() + (gps_time_ok_ ? gps_off_s_ : 0.0); }
    void set_clock_from_gps(); /* "Set clock" (Fix tab): the RTC from GPS time, Zone applied */
    void clock_step();         /* frame handler: writes the RTC at the next second boundary */
    bool clock_set_pending_{false};
    int8_t clock_msg_{0};      /* Fix tab: 1 clock set, -1 no GPS time yet */
    bool clock_logged_{false}; /* the GPS - RTC offset logged once (then on changes over 0.1 s) */
    double clock_logged_off_{0};
    double gps_off_s_{0}; /* GPS time from the satellites - RTC-based GPS time (both at the same instant) */
    bool gps_time_ok_{false};
    /* sub-second clocks (gps_clock.hpp): the RTC's second from its 1 Hz tick,
     * the image's sample count from its status messages */
    RtcPhase rtc_phase_{};
    SampleClock sample_clock_{};
    uint64_t results_sample_{0}; /* first sample of the snapshot the search results refer to */
    double results_age_s() const;
    /* GPS time now from the GPS time at a sample of the image's clock */
    double gps_now_at_sample(double gps_at_sample, uint64_t sample) const;

    /* "Autonomous": tracking loops (M4) -> subframes, ephemerides, PVT (M0) */
    const gps_trk::Shared* trk_sh_{nullptr};
    bool tracking_{false}; /* the tracking image reports TRACKING */
    systime_t auto_t0_{0}; /* Get fix pressed */
    uint32_t ttff_s_{0};   /* first precise fix, s after Get fix (0 none yet) */
    uint32_t ttff_coarse_s_{0};
    uint8_t auto_eph_{0};    /* tracked satellites with an ephemeris (off air or file) */
    uint8_t auto_synced_{0}; /* tracked with TOW */
    uint8_t auto_tracked_{0};
    uint8_t cache_cursor_{0}; /* next EphCollect slot to look at for the SD cache */
    uint8_t cache_line_{0};   /* line of the record being written (0..2) */
    uint16_t cache_written_{0};
    void auto_poll_m4(); /* channels, subframes, measurements; starts a PVT */
    void auto_finish_pvt();
    void auto_cache_step(); /* one EPH_AIR.TXT line per call, from the frame handler */
    uint16_t cache_errs_{0};
    bool meas_waiting_{false}; /* a measurement set copied while the solver ran */
    bool req_confirm_{true};   /* the last search asked for a 2nd-snapshot confirmation */
    /* aided search for newly risen satellites while tracking (auto_aided_step) */
    static constexpr float kAidedMinEl = 10.0f; /* deg */
    static constexpr uint8_t kAidedEvery = 5;   /* s between attempts */
    static constexpr uint8_t kAidedRetry = 24;  /* attempts (x 5 s) before the same PRN again: 2 min */
    uint64_t solve_sample_{0}, fix_sample_{0};  /* sample of the solved measurement set */
    double fix_trx_{0};                         /* its GPS time (from the fix) */
    bool fix_have_time_{false};
    uint8_t aided_tick_{0};
    uint8_t aided_wait_[32]{};
    void auto_aided_step();
    /* the ionosphere for the next PVT: decoded this run, else cached within
     * kIonoMaxAgeS, else none (never stale or zero-filled values) */
    static constexpr double kIonoMaxAgeS = 7.0 * 86400.0;
    uint8_t iono_src_{0}; /* 0 none, 1 off the air, 2 cached */
    uint8_t iono_logged_{0xFF};
    void apply_iono();
    void log_aided(int prn, float el, double code, double dop, double clk, int windows, uint32_t age_ms, bool eph);
    uint16_t log_rel_seen_{0}; /* tracker releases already logged */
    int32_t status_dt_ms_{0};  /* the last search's snapshot A -> B interval */

    /* Diagnostics log (ui_gps_log.cpp): rows built in GpsScratch::log, written
     * from the display frame handler only, sector-bounded, synced every 3 s. */
    File* log_file_{nullptr};
    uint32_t log_pos_{0};
    systime_t log_sync_t_{0}, log_sec_t_{0}, log_retry_t_{0};
    bool log_dirty_{false};
    bool log_started_{false}; /* header and START row queued */
    uint16_t log_errs_{0};
    uint8_t log_eph_tick_{0};
    const char* log_err_{nullptr};
    uint32_t log_fix_hash_{0}, log_heard_hash_{0};
    char log_name_[32]{};
    bool log_active() const;
    Fmt log_row(const char* type);
    void log_end(const Fmt& f);
    void log_header();
    void log_start_row();
    void log_frame();
    void log_open();
    void log_close();
    void log_fail(const char* what);
    void log_second();
    void log_screen();
    void log_results(const gps_rx::Result* r, uint8_t k_ms);
    void log_handover();
    void log_iono();                                  /* the ionosphere source in use changed (IONO row) */
    void log_clock(const char* src);                  /* GPS time known / its offset to the RTC moved */
    void log_clock_set(double rtc_before, int64_t t); /* "Set clock" wrote the RTC */
    void log_subframes();
    void log_meas();
    void log_eph_event(const char* what, int prn);
    void log_solve_start(uint8_t kind);
    void log_fix(uint8_t kind, bool ok, const char* why);
    void log_error(const char* what, const char* detail);
    void stop_auto();
    void load_position();
    void start_load();
    /* Why a fix cannot be tried now (FixView explains it), or OK. */
    enum Blocker : uint8_t { BLOCK_NONE,
                             BLOCK_IMAGE, /* a baseband image on the card is missing or does not match */
                             BLOCK_STARTING,
                             BLOCK_LOADING,
                             BLOCK_NO_POS,
                             BLOCK_NO_ORBITS,
                             BLOCK_ALM_OLD };
    Blocker fix_blocker() const;
    /* The baseband images on the SD card (gps_image.hpp, ui_gps_image.cpp):
     * L1 acquisition for searches, L1 tracking for Autonomous. A switch
     * happens in the frame handler; the RAM lent to the app, the search
     * results and the tracking output are in the M4 RAM kept across the
     * images (gps_keep.hpp), so the solver and the views carry on. */
    gps_image::Id image_{gps_image::ID_NONE};      /* running (or starting) */
    gps_image::Id image_want_{gps_image::ID_NONE}; /* a switch is due */
    bool image_ready_{false};                      /* the running image has reported */
    gps_image::Check image_err_{gps_image::OK};    /* the last load's result (shown while not OK) */
    gps_image::Id image_err_id_{gps_image::ID_NONE};
    systime_t image_fail_t_{0};
    /* what to do once the wanted image reports */
    bool pending_search_{false};
    Job pending_job_{JOB_NONE};
    uint8_t pending_kms_{20};
    bool pending_full_{false};
    uint32_t pending_track_mask_{0}; /* Autonomous: PRNs for the tracking image */
    uint32_t results_ms_{0};         /* the app's ms clock at the result snapshot's first sample */
    bool results_ms_ok_{false};
    /* the last loads, for the log (IMG rows) */
    uint16_t img_loads_{0}, img_logged_{0};
    uint32_t img_last_size_{0}, img_last_ms_{0};
    gps_image::Check img_last_check_{gps_image::OK};
    gps_image::Id img_last_id_{gps_image::ID_NONE};
    gps_image::Check load_image(gps_image::Id id); /* shut the running image down, load and start one */
    void want_image(gps_image::Id id);             /* ask for a switch (frame handler) */
    void image_step();                             /* frame handler: a due switch */
    void on_image_ready();                         /* the wanted image reported: what was waiting */
    void send_track();                             /* Autonomous: CMD_TRACK to the tracking image */
    void log_image();
    /* The System wheel and the other systems' sky (ui_gps_sys.cpp) */
    bool sys_active_{false};  /* GNSSALM.TXT being read */
    File* sys_file_{nullptr}; /* on the heap during a pass only */
    size_t sys_tmp_n_{0};     /* this pass: satellites so far */
    uint8_t sys_tmp_up_[SYS_N]{};
    uint8_t sys_tmp_total_[SYS_N]{};
    double sys_tmp_toa_{0};
    uint16_t sys_tmp_bad_{0};
    void set_sys_view(uint8_t v);
    bool sys_wanted(); /* another system shown on a visible tab */
    void sys_maybe_start();
    void sys_close();
    void sys_take_line();
    void sys_finish(uint8_t file_state);
    void sys_step(systime_t t0);
    double sys_age_days() const;
    size_t sys_top(uint8_t sys, uint8_t* idx, size_t max) const;
    void sys_label(const SysSat& s, char* b, size_t size) const;
    void log_sys(const char* event);
    size_t sys_tmp_ngal_{0};
    /* The aided Galileo E1 search (ui_gps_gal.cpp): after a precise GPS fix,
     * one satellite at a time in the tracking image's spare M4 time */
    void* gal_codes_{nullptr};                      /* the tracking image's Codes buffer (status) */
    const void* gal_result_{nullptr};               /* its Result */
    const gal_acq::TrkShared* gal_trk_sh_{nullptr}; /* its Galileo channels (status) */
    uint32_t gal_trk_max_{gal_acq::kMaxTrk};        /* Galileo channels a detection may take (rx_gps.ini gal_trk; 0 = Heard only) */
    bool gal_off_sent_{true};                       /* the tracking image was told Galileo is off */
    uint16_t gal_status_seq_{0};                    /* searches finished (status) */
    uint8_t gal_status_busy_{0};
    uint32_t gal_status_cycles_{0};
    bool gal_pending_{false};
    uint16_t gal_seq_sent_{0};
    uint8_t gal_prn_{0};
    uint8_t gal_file_{0}; /* GALE1.BIN: 0 not read, 1 ok, 2 missing, 3 not a code file */
    systime_t gal_sent_t_{0}, gal_round_t_{0};
    float gal_pred_el_{0}, gal_pred_clk_{0};
    double gal_pred_cp_{0}, gal_pred_dop_{0};
    uint32_t gal_pred_age_ms_{0};
    void gal_step();
    void gal_collect();
    void gal_reset();
    void gal_poll();                                       /* copy the Galileo channels, log their events (UI thread, 5/s) */
    void gal_off();                                        /* the E1 box unticked: the tracking image stops Galileo */
    const GpsScratch::GalPrev* gal_tracked(int prn) const; /* the channel on that PRN, or nullptr */
    uint8_t gal_tracking() const;                          /* Galileo channels in use */
    bool gal_channel(int i, gal_acq::TrkInfo& c) const;    /* copy one Galileo channel (seqlock) */
    void log_gal_second();
    void log_gal_event(const gal_acq::TrkInfo& c, const char* event, int32_t a, int32_t b);
    /* I/NAV pages of the Galileo channels (gal_inav.hpp), decoded in the RAM the
     * tracking image lends (gal_nav_); logged, not used in the fix yet */
    void* gal_nav_{nullptr};
    uint8_t gal_nav_rr_{0};
    void gal_nav_step();                   /* frame handler: at most one page part */
    uint16_t gal_nav_pages(int prn) const; /* pages with a good CRC on that PRN's channel */
    void log_gal_page(const gal_inav::Page& p);
    /* Galileo in the fix (ui_gps_gal.cpp): ephemerides from the pages, the
     * Galileo measurement set, the PVT input, the gate after the solve */
    GalNavMem* gal_mem() const { return static_cast<GalNavMem*>(gal_nav_); }
    void gal_meas_copy();                                           /* with a new GPS measurement set (UI thread) */
    int gal_add_pvt(int n);                                         /* Galileo entries after the n GPS ones; decides the mode */
    void gal_solve_stats();                                         /* solver thread: residuals of the GPS + Galileo solve */
    bool gal_finish(bool gps_ok);                                   /* UI thread: the gate; true = use the GPS + Galileo solution */
    uint8_t gal_eph_ready(int prn) const;                           /* 1: a usable ephemeris for that PRN */
    uint8_t gal_eph_count() const;                                  /* tracked Galileo satellites with a usable ephemeris */
    bool gal_in_fix(int prn) const;                                 /* that PRN was used in the fix shown */
    void gal_fix_text(char* b, size_t n, bool alone = false) const; /* one line: Galileo in the fix, or why not */
    void log_gal_eph(uint8_t ev, int ch, int slot, uint8_t prn);
    void log_gal_meas();
    void log_gal_fix(bool gps_ok, bool use);
    uint8_t solve_gal_n_{0};     /* Galileo entries in the PVT input being solved */
    uint8_t solve_gal_mode_{0};  /* 0 none, 1 GGTO applied (one clock), 2 Galileo - GPS offset solved */
    uint8_t solve_gal_alone_{0}; /* also solve Galileo alone (wheel on Galileo, >= 4) */
    bool solve_mix_ok_{false}, solve_alone_ok_{false};
    float solve_ggto_ns_{0};
    uint8_t fix_gal_{0};       /* Galileo satellites in the fix shown (0: GPS only) */
    uint8_t fix_gal_mode_{0};  /* its solve_gal_mode_ */
    uint64_t fix_gal_prns_{0}; /* bit prn - 1: Galileo satellites in the fix shown */
    uint16_t alone_dm_{0};     /* Galileo alone: horizontal distance from the GPS-only fix, dm (0xFFFF none) */
    uint8_t alone_n_{0};       /* its satellites (0: no Galileo-alone solution last time) */
    uint8_t gal_out_why_{0};   /* why Galileo is not in the fix: 1 off (ini), 2 no input (gal_eph::Why of the best channel + 10), 3 gate (gal_eph::Gate + 20) */
    uint8_t gal_heard() const; /* satellites heard in this round */
    void log_gal(const gal_acq::Result& r);
    void image_fallback_ram(); /* no image: use the kept block as plain RAM */
    /* Start now if the sky cache is ready, else as soon as it is. */
    void request_search(Job job, uint8_t k_ms);
    /* Start a snapshot search: assisted (visible PRNs, predicted Doppler)
     * when orbits, a position and the time allow, else all 32 PRNs. */
    bool start_search(Job job, uint8_t k_ms, bool force_full = false);
    /* What the running / last search covered (Heard says it plainly). */
    enum SearchKind : uint8_t { SK_NONE,
                                SK_ASSISTED,     /* predicted PRNs and Doppler (almanac, position, time) */
                                SK_FULL,         /* all 32 PRNs, +-7 kHz (no orbits or position) */
                                SK_FULL_AFTER }; /* all 32 after an assisted search found none */
    SearchKind search_kind_{SK_NONE};
    /* Stop a running search (the M4 drops it at its next work unit). */
    void stop_search();
    bool searching() const { return job_ != JOB_NONE && !done_; }
    bool assisted_possible() const;
    void on_status(const GpsRxStatusMessage& m);
    void solve_fix(); /* starts the solver thread */
    int progress_pct() const;

    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "LNA:   VGA:   AMP:", Theme::getInstance()->fg_light->foreground}};
    LNAGainField field_lna_{{UI_POS_X(4), UI_POS_Y(0)}};
    VGAGainField field_vga_{{UI_POS_X(11), UI_POS_Y(0)}};
    RFAmpField field_amp_{{UI_POS_X(18), UI_POS_Y(0)}};
    /* Antenna port power (3.3 V, <= 50 mA, for an ACTIVE antenna only) is
     * shown here read only: the app never switches it (by design); the
     * user does, with Mayhem's status-bar control or Settings > Radio. */
    Text text_ant_{{UI_POS_X(22), UI_POS_Y(0), UI_POS_WIDTH(8), UI_POS_HEIGHT(1)}, ""};
    void show_antenna();

    Rect view_rect_{0, UI_POS_Y(1) + 24, screen_width, static_cast<Dim>(screen_height - UI_POS_Y(1) - 24 - UI_POS_Y(1))};
    FixView view_fix_;
    HeardView view_heard_;
    PassesView view_passes_;
    TabView tab_view_;

    SignalToken tick_token_{};
    uint16_t sky_tick_{0};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_frame();
        }};
    MessageHandlerRegistration message_handler_status{
        Message::ID::GpsRxStatus,
        [this](const Message* const p) {
            this->on_status(*static_cast<const GpsRxStatusMessage*>(p));
        }};
    void on_frame();
    void on_tick_second();
};

#else /* not PRALINE */

/* The GPS receiver needs the HackRF Pro (LPC4330 RAM); elsewhere a note. */
class GpsView : public View {
   public:
    GpsView(NavigationView& nav);
    void focus() override;
    std::string title() const override { return "GPS"; };

   private:
    Text text_{{0, UI_POS_Y(2), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "GPS needs a HackRF Pro"};
    Button button_back_;
};

#endif

}  // namespace ui::external_app::gps

#endif /*__UI_GPS_H__*/
