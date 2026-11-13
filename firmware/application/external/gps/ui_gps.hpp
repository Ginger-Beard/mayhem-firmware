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
 * Files: /GPS/ALMANAC.TXT (YUMA), /GPS/EPHEM.RNX (RINEX 2/3 navigation),
 * /GPS/TLE.TXT (sdcard/GPS/README.TXT). The baseband (PGPR) does the signal
 * processing and lends the app M4 RAM for the orbit tables and the solver, so
 * they do not take the M0 heap. */

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

   private:
    Line* lines_;
    size_t rows_;
    const Font* font_;
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
    };
    static constexpr size_t kMax = 32;
    explicit SkyPlot(Rect r)
        : Widget{r} {}
    void clear() { n_ = 0; }
    void add(int16_t az, int16_t el, const char* label, Color color, bool filled);
    void paint(Painter& painter) override;

   private:
    Dot dots_[kMax]{};
    size_t n_{0};
};

/* What lives in the M4 RAM PGPR lends (GpsRxStatusMessage::scratch). */
struct GpsScratch {
    static constexpr size_t kMaxTle = 5;
    Orbit orbits[32];
    Tle tles[kMaxTle];
    Sgp4 search; /* pass search */
    Sgp4 sky;    /* sky plot */
    SolveWork solve;
};

class GpsView;

/* Incremental reader of the orbit files: a few KB per display frame, so the
 * UI stays responsive and nothing large sits on the stack. */
class OrbitLoader {
   public:
    enum Stage : uint8_t { IDLE,
                           ALMANAC,
                           EPHEMERIS,
                           TLES,
                           DONE };
    OrbitLoader() = default;
    OrbitLoader(const OrbitLoader&) = delete;
    OrbitLoader& operator=(const OrbitLoader&) = delete;
    ~OrbitLoader();
    void start(double now_gps);
    /* One frame's worth; true while there is more to do. */
    bool step(GpsView& v);
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
    bool eof_{false};
    double now_gps_{0};
    YumaParser yuma_{0};
    RinexNavParser rinex_{};
    bool open(Stage s);
    void close();
    bool next_line();
    void take(GpsView& v);
};

class PassesView : public View {
   public:
    PassesView(GpsView& main, Rect r);
    void focus() override;
    void refresh();        /* position / time / file lines */
    void update_sky();     /* every few seconds */
    void restart_passes(); /* new position, time or files */
    void frame();          /* a slice of the pass search */

   private:
    GpsView& main_;
    static constexpr size_t kRows = 3;
    TextLines::Line head_[kRows]{};
    TextLines text_head_;
    Button button_pos_;
    Button button_load_;
    Labels labels_off_;
    NumberField field_off_;
    SkyPlot sky_;
    static constexpr size_t kSideRows = 15;
    TextLines::Line side_[kSideRows]{};
    TextLines text_side_;
    static constexpr size_t kPassRows = 4;
    TextLines::Line pass_lines_[kPassRows]{};
    TextLines text_passes_;
    PassFinder finder_{};
    Pass results_[kPassRows]{};
    bool result_ok_[kPassRows]{};
    uint8_t search_idx_{0};
    bool searching_{false};
    void show_passes();
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
};

class FixView : public View {
   public:
    FixView(GpsView& main, Rect r);
    void focus() override;
    void show();

   private:
    GpsView& main_;
    OptionsField field_method_;
    Button button_fix_;
    Button button_save_;
    bool fix_running_shown_{false}; /* button_fix_ shows "Stop" */
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

class GpsView : public View {
   public:
    GpsView(NavigationView& nav);
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

    enum Job : uint8_t { JOB_NONE,
                         JOB_HEARD,
                         JOB_FIX };

    NavigationView& nav_;
    RxRadioState radio_state_{
        1575420000 /* GPS L1 */,
        1750000 /* baseband filter */,
        2048000 /* 2048 samples = 1 ms = one DMA buffer */,
        ReceiverModel::Mode::WidebandFMAudio};
    int32_t utc_off_min_{0}; /* RTC minus UTC, minutes (saved) */
    uint32_t tab_{2};        /* last tab (saved) */
    uint32_t depth_{1};      /* Heard snapshot: 0 = 10 ms, 1 = 20 ms (saved) */
    app_settings::SettingsManager settings_;
    bool bias_was_{false};
    bool rx_on_{false};

    /* Lent M4 RAM: orbit tables, TLEs, propagator states, solver work. */
    GpsScratch* mem_{nullptr};
    uint8_t n_alm_{0}, n_eph_{0}, n_tle_{0};
    double eph_toe_newest_{0};
    OrbitLoader loader_{};
    bool load_wanted_{true};

    saved_position::Position pos_{};
    bool pos_valid_{false};
    Vec3 rx_{};

    /* Baseband status (copied from GpsRxStatusMessage) and the current search. */
    struct Status {
        uint8_t state{GpsRxStatusMessage::IDLE};
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

    /* Last fix. */
    Solution fix_{};
    bool fix_ok_{false};
    bool fix_tried_{false};
    uint32_t fix_fat_{0};
    uint8_t fix_heard_{0};
    double fix_eph_age_s_{0};

    double now_unix() const; /* UTC from the RTC and the offset */
    double now_gps() const { return unix_to_gps(now_unix()); }
    void load_position();
    void start_load();
    /* Start a snapshot search: assisted (visible PRNs, predicted Doppler)
     * when orbits, a position and the time allow, else all 32 PRNs. */
    bool start_search(Job job, uint8_t k_ms);
    /* Stop a running search (the M4 drops it at its next work unit). */
    void stop_search();
    bool searching() const { return job_ != JOB_NONE && !done_; }
    bool assisted_possible() const;
    void on_status(const GpsRxStatusMessage& m);
    void solve_fix();
    int progress_pct() const;

    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "LNA:   VGA:   AMP:", Theme::getInstance()->fg_light->foreground}};
    LNAGainField field_lna_{{UI_POS_X(4), UI_POS_Y(0)}};
    VGAGainField field_vga_{{UI_POS_X(11), UI_POS_Y(0)}};
    RFAmpField field_amp_{{UI_POS_X(18), UI_POS_Y(0)}};
    /* Antenna port power (3.3 V, <= 50 mA): for an ACTIVE antenna only. */
    Checkbox check_bias_{{UI_POS_X(21), UI_POS_Y(0)}, 4, "Ant", true};
    Text text_warn_{{0, UI_POS_Y(1), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, ""};

    Rect view_rect_{0, UI_POS_Y(2) + 24, screen_width, static_cast<Dim>(screen_height - UI_POS_Y(2) - 24 - UI_POS_Y(1))};
    FixView view_fix_;
    HeardView view_heard_;
    PassesView view_passes_;
    TabView tab_view_;

    SignalToken tick_token_{};
    uint8_t sky_tick_{0};

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
