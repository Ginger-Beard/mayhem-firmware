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
 * /GPS/TLE.TXT, /GPS/SATFREQ.TXT (downlinks) (sdcard/GPS/README.TXT). The baseband (PGPR) does the signal
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
    /* a tap on a row (rows are line_height apart) */
    std::function<void(size_t)> on_row{};
    bool on_touch(const TouchEvent event) override;

   private:
    Line* lines_;
    size_t rows_;
    const Font* font_;
    uint32_t used_{0};
    int8_t hl_{-1};
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

/* What lives in the M4 RAM PGPR lends (GpsRxStatusMessage::scratch). */
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
};

class GpsView;

/* Incremental reader of the orbit files: a few KB per display frame, so the
 * UI stays responsive and nothing large sits on the stack. */
class OrbitLoader {
   public:
    enum Stage : uint8_t { IDLE,
                           ALMANAC,
                           EPHEMERIS,
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
    static constexpr size_t kSideRows = 14;
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
    int items() const; /* TLE pass rows, then GPS side rows */
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
    void show(); /* status / results changed */

   private:
    GpsView& main_;
    OptionsField field_depth_;
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

   private:
    GpsView& main_;
    OptionsField field_method_;
    Button button_fix_;
    Button button_save_;
    bool fix_running_shown_{false}; /* button_fix_ shows "Stop" */
    const char* save_msg_{nullptr}; /* bottom row after "Save pos" */
    Color save_color_{Color::white()};
    void show_blocker(size_t row);
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
    uint8_t n_dl{0};
    SatFreq dl[kMaxDl]{}; /* this satellite's downlinks from SATFREQ.TXT */
};

/* Follow: the selected satellite's downlink on a stock audio image (NFM, AM
 * incl. USB/LSB/CW, WFM), the Doppler corrected 4 times a second (the
 * baseband's DDC shift on NFM / AM, a retune on WFM), a signal meter with
 * peak hold, min/max and a 40 s history, Voice or Tone (the stock NFM image's
 * pitch-by-level tone, pulsed faster as the level rises), a level squelch,
 * and the pointing readout. It replaces the GPS app (so PGPR stops); "GPS"
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
                         JOB_FIX };

    NavigationView& nav_;
    RxRadioState radio_state_{
        1575420000 /* GPS L1 */,
        1750000 /* baseband filter */,
        2048000 /* 2048 samples = 1 ms = one DMA buffer */,
        ReceiverModel::Mode::WidebandFMAudio};
    /* Time zone of the PortaPack clock: local standard time minus UTC, in
     * minutes (saved; e.g. -300 for US Eastern). With Mayhem's DST setting on,
     * the RTC hardware keeps standard time and this is the standard offset;
     * with it off, the clock shows whatever it was set to and this must be
     * the offset in force (e.g. -240 in US summer). */
    int32_t utc_off_min_{0};
    uint32_t tab_{2};   /* last tab (saved) */
    uint32_t depth_{1}; /* Heard snapshot: 0 = 10 ms, 1 = 20 ms (saved) */
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
    static msg_t solver_fn(void* arg);
    void finish_solve();
    bool solving() const { return solver_ != nullptr; }

    double now_unix() const;   /* UTC from the RTC and the zone */
    double local_unix() const; /* the clock as shown (incl. Mayhem DST), as if it were UTC */
    double now_gps() const { return unix_to_gps(now_unix()); }
    void load_position();
    void start_load();
    /* Why a fix cannot be tried now (FixView explains it), or OK. */
    enum Blocker : uint8_t { BLOCK_NONE,
                             BLOCK_STARTING,
                             BLOCK_LOADING,
                             BLOCK_NO_POS,
                             BLOCK_NO_ORBITS,
                             BLOCK_ALM_OLD };
    Blocker fix_blocker() const;
    /* Start now if the sky cache is ready, else as soon as it is. */
    void request_search(Job job, uint8_t k_ms);
    /* Start a snapshot search: assisted (visible PRNs, predicted Doppler)
     * when orbits, a position and the time allow, else all 32 PRNs. */
    bool start_search(Job job, uint8_t k_ms);
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
    Text text_warn_{{0, UI_POS_Y(1), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, ""};
    void show_antenna();

    Rect view_rect_{0, UI_POS_Y(2) + 24, screen_width, static_cast<Dim>(screen_height - UI_POS_Y(2) - 24 - UI_POS_Y(1))};
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
