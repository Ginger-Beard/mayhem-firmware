/*
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

#ifndef __UI_DRONE_DETECT_H__
#define __UI_DRONE_DETECT_H__

#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "app_settings.hpp"
#include "saved_position.hpp"
#include "radio_state.hpp"
#include "message.hpp"

#ifdef PRALINE
#include "ui_geomap.hpp"
#include "recent_entries.hpp"
#include "log_file.hpp"
#include "signal.hpp"

#include "dd_model.hpp"
#include "dd_esp.hpp"
#include "dd_demo.hpp"
#include "dd_rf.hpp"
#include "dd_pattern.hpp"
#include "dd_usbq.hpp"
#include "dd_sig.hpp"
#include "dd_survey.hpp"
#include "dd_track.hpp"
#include "dd_zwnet.hpp"
#include "dd_capture.hpp"
#include "ui_record_view.hpp"
#include "ui_spectrum.hpp"
#include "ui_freq_field.hpp"
#include "spec_an_sweep.hpp"
#include "file.hpp"

#include <memory>
#endif

namespace ui::external_app::drone_detect {

/* Menu icon of the built-in app (HackRF Pro, tier 2): the same 16x16 bitmap as
 * the external app header in main.cpp. */
inline constexpr uint8_t bitmap_icon_drone_detect_data[32] = {
    0x00, 0x00, 0x0E, 0x70, 0x1F, 0xF8, 0x0E, 0x70, 0x04, 0x20, 0x07, 0xE0, 0x0F, 0xF0, 0x0F, 0xF0,
    0x07, 0xE0, 0x04, 0x20, 0x0E, 0x70, 0x1F, 0xF8, 0x0E, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
inline constexpr Bitmap bitmap_icon_drone_detect{{16, 16}, bitmap_icon_drone_detect_data};

#ifdef PRALINE

/* One text line of a TextLines widget. */
struct Line {
    static constexpr size_t kCols = 40;
    char text[kCols + 1];
    Color color{Color::white()};
};

/* Fixed lines of text painted straight from char buffers (no std::string per
 * line); cut at the widget's width. */
class TextLines : public Widget {
   public:
    TextLines(Rect parent_rect, Line* lines, size_t rows)
        : Widget{parent_rect}, lines_{lines}, rows_{rows} {}
    TextLines(const TextLines&) = delete;
    TextLines& operator=(const TextLines&) = delete;

    void paint(Painter& painter) override;

   private:
    Line* lines_;
    size_t rows_;
};

/* Persistent options (SettingsManager, "rx_drone_detect"). The first 8
 * bytes are flags, saved together as one number ("opts": one binding instead
 * of eight) and set by the Setup checkboxes through their byte and bit. */
struct DdOptions {
    uint8_t alert{1};
    uint8_t csv{0};  // bit 0: Log (CSV), bit 1: Survey rows too
    uint8_t json{0};
    uint8_t esp{0};       // ESP32 feed
    uint8_t mode{0};      // bit 0: no RF sweeps (Remote ID only), bit 1: no BLE (RF only), bit 2: 2.4 GHz hopper/toy verdicts, bit 3: no pattern scan, bit 4: no Wi-Fi RID, bit 5: no DJI DroneID
    uint8_t bands{0x1F};  // RF sweep bands: bit 0 2.4, 1 5.8, 2 5.3, 3 1.2, 4 900, 5 NO 5.1 (inverted: older settings files sweep it)
    uint8_t region{0};    // 900 MHz band: 0 US 902-928, 1 EU 860-872
    uint8_t beep_low{0};  // beep for low confidence RF classes too (2.4 GHz hoppers / toys, DV/WF)
    int32_t man_lat{0};   // manual own position, 1e-7 degrees, 0/0 = not set
    int32_t man_lon{0};
    uint64_t man_meta{0};  // manual position: bits 0-31 when it was found (FAT date-time, 0 = unknown), bits 32-39 its saved_position::Source (0: picked on the map before sources were kept)
};
constexpr uint64_t kOptsDefault = 0x00001F0000000001ULL; /* the flags above, little endian */
static_assert(offsetof(DdOptions, man_lat) == sizeof(uint64_t), "DdOptions: 8 flag bytes first");

/* What lives in the M4 RAM PDRN lends (DroneDetectStatsMessage::kScratchSig):
 * the selected signature class, its NB rules' dwell state and the survey
 * log's tables. Not the M0 heap. */
/* What the app has seen of one FSK rule (not the Z-Wave one: ZwNets). */
struct FskSeen {
    uint32_t frames;        /* frames this session */
    uint32_t t_win, t_last; /* uptime ms: the listing window's start, the last frame */
    uint32_t t_row;         /* the last FSK_FRM row */
    uint32_t rate_bps;
    int16_t rssi_cdb, off_hz;
    uint16_t dev_hz;
    uint16_t skipped; /* FSK_FRM rows left out since the last one (1 a second at most) */
    uint8_t n_win;    /* frames in the window */
    uint8_t len;
    bool listed, reported, checked;
};

struct SigMemory {
    SigClass cls{};
    SigParser parser{}; /* only while a class loads */
    NbState nb[kSigMaxNb]{};
    NbResult nb_last[kSigMaxNb]{}; /* each NB rule's last hit */
    BleSurvey ble{};
    NrgSurvey nrg{};
    ZwNets zw{}; /* Z-Wave networks heard (home ID, nodes) */
    FskSeen fsk[kSigMaxFsk]{};
    RowBudget budget{};
};

class DetailView;
class SettingsView;
class TrackView;
class TargetView;
class DdCaptureView;
class DebugView;
class Fmt;

/* Drone detect: Remote ID over BLE (PDRN baseband, GFSK mode) and DJI BLE
 * adverts (presence), Remote ID over Wi-Fi heard by the HackRF itself (PDRN
 * DSSS mode: 802.11b 1 Mb/s beacons and NAN frames), the optional ESP32
 * module's WiFi/BLE Remote ID list, the pattern scan (PDRN pattern mode,
 * dd_pattern.hpp: Kind::Pattern) and RF energy sweeps (PDRN sweep mode,
 * spec_an::Sweep, RfEngine) whose confirmed tracks join the same Tracker as
 * Kind::Rf. The scheduler cycles
 *   BLE dwell (channels 37, 39, 38 in turn) -> Wi-Fi dwell (every
 *   tune::kWifiEveryBle-th BLE dwell: one channel of tune::kWifiChannels, the
 *   two halves of the channel in turn) -> pattern dwell (one channel of one
 *   table entry, in turn) -> sweep dwell (one band, in turn) -> BLE ...
 * skipping what is switched off; each BLE dwell is ratio x the Wi-Fi,
 * pattern and sweep dwells after it (>= 50 % BLE airtime, >= 75 % while
 * Remote ID is heard). Every ~40 s a DJI DroneID dwell (PDRN DroneID mode,
 * tune::kDidDwellMs on one 2.4 GHz DroneID centre: Kind::DroneId, presence
 * only) cuts in at a phase boundary, outside the ratio; the cycle then
 * resumes where it was. */
class DroneDetectView : public View {
   public:
    DroneDetectView(NavigationView& nav);
    ~DroneDetectView();

    DroneDetectView(const DroneDetectView&) = delete;
    DroneDetectView& operator=(const DroneDetectView&) = delete;

    void focus() override;
    std::string title() const override { return "Sig Detect"; };

   private:
    friend class DetailView;
    friend class SettingsView;
    friend class DebugView;
    friend class TrackView;
    friend class TargetView;

    static constexpr uint32_t sampling_rate = 4000000;
    static constexpr uint8_t kBle = 0xFF;     /* phase_band_ of a BLE dwell; dump_band_ off */
    static constexpr uint8_t kDumpAll = 0xFE; /* dump_band_: every band's dwell */
    static constexpr uint8_t kPat = 0xFD;     /* phase_band_ of a pattern dwell */
    static constexpr uint8_t kWifi = 0xFC;    /* phase_band_ of a Wi-Fi dwell */
    static constexpr uint8_t kDid = 0xFB;     /* phase_band_ of a DJI DroneID dwell */
    static constexpr uint8_t kTrack = 0xFA;   /* phase_band_ while the track view is open: the cycle is paused */
    static constexpr uint8_t kZw = 0xF9;      /* phase_band_ of an FSK dwell (PDRN MODE_FSK; Z-Wave and the class's FSK rules) */
    static constexpr uint8_t kSetupBands = 6;

    struct BandStat {
        uint16_t looks, slices, ms;
    };

    NavigationView& nav_;
    RxRadioState radio_state_{
        2402000000 /* frequency */,
        4000000 /* bandwidth */,
        sampling_rate,
        ReceiverModel::Mode::WidebandFMAudio};
    DdOptions opt_{};
    uint64_t opts_{kOptsDefault};
    std::string sig_name_{"Drones"}; /* signature class by name (file stem), saved; before settings_, which loads it */
    uint32_t trk_khz_{462160};       /* manual track frequency (kHz), saved; before settings_ */
    uint32_t trk_opt_{0x4405};       /* track view: bits 0-1 bandwidth index, bit 2 beep, bits 8-15 squelch dBFS + 128; saved */
    app_settings::SettingsManager settings_;

    Tracker tracker_{};
    EspFeed esp_{};
#if DD_FULL
    DemoDrones demo_{};
#endif
    /* CSV log (format 2, sdcard/SIGDET/README.TXT): one file per session,
     * LOGS/SIGDET_<date time>.CSV (a new part past kLogPartBytes); rows are
     * written as they come (FatFs keeps the partial sector) and synced every
     * kLogSyncMs, so a pulled battery loses at most that. The File (~0.6 KB)
     * is the only M0 heap the log takes; rows are built on the stack. */
    File* log_{nullptr};
    bool log_open_{false};    /* Log ticked: (re)open on the next frame */
    char log_error_[24]{};    /* why the CSV log could not be opened or written, "" if fine */
    char log_name_[36]{};     /* LOGS/SIGDET_....CSV of this session */
    uint8_t log_part_{0};     /* rollover parts of this session */
    bool log_dirty_{false};   /* rows written since the last sync */
    uint32_t log_sync_ms_{0}; /* last sync */
    uint32_t log_retry_s_{0}; /* after an error: try again at this time (0: no) */
    uint8_t log_fail_{0};     /* errors in a row (reset by a good sync) */
    uint32_t log_rows_{0};    /* rows written (session) */
    uint32_t log_lost_{0};    /* rows lost: no file (error, no memory) */
    uint32_t log_errs_{0};    /* write / sync / open errors */
    uint32_t log_bytes_{0};   /* bytes in the current part */
    uint32_t hb_s_{0};        /* last heartbeat row */
    /* Signature class: index 0 = Drones (built in), else /SIGDET/SIGS file. */
    SigMemory* sig_mem_{nullptr}; /* in PDRN's lent RAM, once known */
    bool sig_file_{false};        /* a file class is selected */
    bool sig_loaded_{false};      /* ... and loaded into sig_mem_ */
    bool sig_scan_pending_{true}; /* list /SIGDET/SIGS on the first frame */
    bool sig_load_failed_{false}; /* the file could not be opened */
    bool sd_legacy_{false};       /* /SIGDET/SIGS missing, /DRONEDET/SIGS read instead (one release) */
    const char* sd_root() const;
    void update_columns();
    bool nb_dwell_{false};     /* NB rules measure this sweep dwell */
    bool async_tx_was_{false}; /* portapack::async_tx_enabled before the app */

    /* Own position from GPSPosData (ESP32 module or shell "gotgps"): only a
     * real fix (on_gps). A GPS that talks but has no fix (the module sends
     * 0/0 then) is remembered apart, for the status line. */
    int32_t gps_lat_{0};
    int32_t gps_lon_{0};
    int32_t gps_alt_m_{0};
    uint8_t gps_sats_{0}; /* satellites in use, as the GPS reports them */
    uint32_t gps_s_{0};
    bool gps_valid_{false};
    uint32_t gps_nofix_s_{0};
    bool gps_nofix_{false};
    uint32_t map_nomem_s_{0}; /* a map could not be opened (heap), shown for a while */
    bool map_nomem_{false};

    /* Scheduler. */
    uint8_t phase_band_{kBle};
    uint16_t phase_dwell_ms_{0};
    uint8_t band_cursor_{BAND_COUNT - 1}; /* RfBand of the current / last sweep dwell */
    uint8_t ble_idx_{2};
    uint8_t channel_{37};
    bool rf_full_{false};
    uint8_t stall_run_{0};
    uint16_t stalls_{0};
    uint16_t slices_{0};
    uint16_t cycle_ms_{0};
    uint32_t cycle_{0};
    systime_t cycle_t0_{0};
    systime_t phase_t0_{0};
    uint32_t captured_rx_{0}; /* SpecAnCaptured / SpecAnSlice messages received (all) */
    uint32_t slice_rx_{0};
    BandStat bstat_[BAND_COUNT]{};
    uint32_t ble_ms_{0}; /* airtime totals since the start (DDSTAT) */
    uint32_t sweep_ms_{0};
    uint32_t pat_ms_{0};
    uint32_t wifi_ms_{0};
    uint32_t zw_ms_{0};     /* Z-Wave airtime */
    uint32_t zw_rx_{0};     /* DroneDetectFskMessages of the Z-Wave rule */
    uint32_t fsk_rx_{0};    /* DroneDetectFskMessages received */
    uint8_t fsk_cursor_{0}; /* the FSK dwell (SigClass::fskg) last visited */
    uint32_t zw_bad_{0};    /* ... failing the header sanity checks */

    /* Wi-Fi Remote ID dwells (PDRN DSSS mode). */
    uint8_t ble_since_wifi_{0};                 /* BLE dwells since the last Wi-Fi dwell */
    uint8_t wifi_cursor_{0};                    /* tune::kWifiChannels entry of the current / last dwell */
    uint8_t wifi_upper_{0};                     /* bit n: channel entry n's next dwell takes the upper (+2 MHz) half */
    int8_t wifi_slice_{-2};                     /* of the current / last dwell */
    uint8_t wifi_mask_{tune::kWifiChannelMask}; /* channel entries scanned (debug key w), not saved */
    bool wifi_legacy_{false};                   /* French / legacy DJI IEs forwarded (debug key l), not saved */
    uint8_t wifi_search_{0};                    /* DSSS search phases - 1 (debug key k: 4, 3, 2), 0 = the image's default; not saved */
    uint32_t wifi_rx_{0};                       /* DroneDetectWifiMessages received */
    uint32_t wifi_odid_{0};                     /* ... with an ODID message pack that the Tracker took */
    uint32_t wifi_rid_s_{0};                    /* when on-board Wi-Fi last brought Remote ID */

    /* DJI DroneID dwells (PDRN DroneID mode, 15.36 Msps). */
    uint8_t did_cursor_{0};     /* tune::kDidChannelsKhz entry of the current / last dwell */
    uint8_t did_resume_{kBle};  /* the phase before it: the cycle goes on from there */
    systime_t did_last_end_{0}; /* when the last DroneID dwell ended (the start: see the constructor) */
    uint32_t did_ms_{0};        /* airtime total (DDSTAT) */
    uint32_t did_rx_{0};        /* DroneDetectDroneIdMessages received */

    /* Pattern scan: table (built-in + /SIGDET/PATTERNS.TXT), dwell cursor. */
    PatternTable pat_table_{};
    PatternScan pat_scan_{pat_table_};
    bool pat_load_pending_{true}; /* read the file on the first frame */
    uint8_t pat_entry_{0};        /* of the current / last pattern dwell */
    uint8_t pat_ch_{0};
    uint16_t pat_dwell_last_{kPatDwellMs};
    uint32_t pat_hits_rx_{0}; /* hit messages received */
    uint16_t partial_{0};     /* sweeps missing a slice (the M4 dropped a reply) */
    uint32_t dwell_seq_{0};   /* sweep dwells since the start: DDSWP / DDLK seq */
    uint8_t dwell_sweeps_{0}; /* complete sweeps of this dwell, kept or not (kDiscardLooks) */
    /* The RF engine's working arrays and the sweep sit in M4 RAM lent by PDRN
     * (rf_memory): no M0 heap. Sweeps pause while a map is open (pause_rf). */
    spec_an::Sweep* sweep_{nullptr};
    RfEngine::Work* work_{nullptr};
    bool rf_paused_{false};
    RfEngine rf_{};

    /* Sweep dump (debug view), not saved. */
    uint8_t dump_band_{kBle};
    bool dump_raw_{false}; /* DDLK: every look of the dumped band(s) */
    bool stat_usb_{false}; /* DDSTAT: counters once a second */
    systime_t dump_t_{0};
    /* Sweep knobs (debug view keys), not saved. */
    bool pace_{tune::kFramePaced};
    uint16_t settle_us_{tune::kSweepSettleUs};
    uint8_t avg_{0}; /* 0: per band (tune::kSweepAvgPeak / kSweepAvgMean) */
#if DD_FULL
    bool dump_sd_{false};
    std::unique_ptr<LogFile> dump_log_{};
#endif

    /* Bring-up switches (debug view), not saved. */
    bool pass_all_{false};
    bool conjugate_{false};
    bool raw_usb_{false};
    bool reset_pending_{false}; /* FLAG_RESET_STATS for the next BLE dwell */
    bool demo_on_{false};
    DroneDetectStatsMessage stats_{}; /* the latest, as sent */
    uint32_t beep_s_{0};
    bool audio_on_{false};

    DetailView* detail_{nullptr};
    TrackView* track_{nullptr};
    DebugView* debug_{nullptr};
    GeoMapView* map_{nullptr}; /* a DdMapView: in PDRN's lent RAM (ui_drone_detect.cpp) */
    Entity::Key map_key_{Entity::invalid_key};

    SignalToken tick_token_{};

    /* Top row as in the stock RX apps: gains and amp (wheel adjustable,
     * written to receiver_model, saved with the app settings), Log. */
    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "LNA:   VGA:   AMP:", Theme::getInstance()->fg_light->foreground}};
    LNAGainField field_lna_{
        {UI_POS_X(4), UI_POS_Y(0)}};
    VGAGainField field_vga_{
        {UI_POS_X(11), UI_POS_Y(0)}};
    RFAmpField field_amp_{
        {UI_POS_X(18), UI_POS_Y(0)}};
    Checkbox check_log_{
        {UI_POS_X(22), UI_POS_Y(0)},
        3,
        "Log",
        true};
    /* Signature class (wheel; files in /SIGDET/SIGS, as Looking Glass's
     * presets) and the survey level of the log. */
    Labels labels_sig_{
        {{UI_POS_X(0), UI_POS_Y(3)}, "Sig", Theme::getInstance()->fg_light->foreground}};
    OptionsField field_sig_{
        {UI_POS_X(4), UI_POS_Y(3)},
        12,
        {{"Drones", 0}}};
    Checkbox check_survey_{
        {UI_POS_X(17), UI_POS_Y(3)},
        6,
        "Survey",
        true};
    Line status_lines_[2]{};
    TextLines text_status_{
        {0, UI_POS_Y(1), UI_POS_MAXWIDTH, UI_POS_HEIGHT(2)},
        status_lines_,
        2};
    RecentEntriesColumns columns_{
        {{"ID", 0},
         {"Src", 4}, /* four source letters, e.g. BWEH */
         {"dB", 4},
         {"Dist", 4}, /* list format: 999m 9.9k 999k */
         {"Age", 3}}};
    RecentEntriesView<Tracker> list_{columns_, tracker_};
    Button button_map_{
        {UI_POS_X_TABLE(4, 0), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Map"};
    Button button_track_{
        {UI_POS_X_TABLE(4, 1), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Target"};
    Button button_setup_{
        {UI_POS_X_TABLE(4, 2), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Setup"};
    Button button_debug_{
        {UI_POS_X_TABLE(4, 3), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Debug"};

    uint32_t now_s() const;
    bool own_position(int32_t& lat, int32_t& lon) const;
    bool gps_fix() const;   /* a fix from the last kGpsMaxAge seconds */
    bool gps_nofix() const; /* a GPS is talking but has no fix (and no fix since) */
    bool map_fits();        /* a map view (DdMapView) can open now (else the status line says so) */
    void apply_options();
    void configure(uint8_t extra_flags = 0);
    bool sweeps_on() const;
    bool patterns_on() const;
    bool wifi_on() const;
    bool wifi_due() const;
    void start_wifi(bool from_sweep);
    void on_wifi(const DroneDetectWifiMessage& message);
    bool droneid_on() const;
    bool droneid_due() const;
    void start_droneid();
    void on_droneid(const DroneDetectDroneIdMessage& message);
    void wifi_raw_line(const DroneDetectWifiMessage& message, bool odid);
    bool sweeping() const { return phase_band_ < BAND_COUNT; }
    void start_ble(bool from_sweep);
    void start_pattern(bool from_sweep);
    void end_pattern(uint16_t ms);
    void load_patterns();
    void on_pattern_hit(const DroneDetectPatternHitMessage& message);
    void report_other(Tracker::Event ev, const Entity& e);
    uint8_t band_id(uint8_t setup_bit) const;
    static uint8_t setup_bit(uint8_t band);
    uint8_t swept_bands() const; /* setup band bits swept (set = on), Drones class */
    uint8_t next_band(bool advance);
    void next_phase();
    void start_sweep(uint8_t band);
    void end_sweep();
    void on_slice(const SpecAnSliceMessage& message);
    void on_rf(RfEngine::Event ev, const RfTrack& t);
    static void on_rf_event(void* ctx, RfEngine::Event ev, const RfTrack& t);
    static void dump_line(void* ctx, const char* line);
    /* USB line that never blocks: queued (dd_usbq.hpp), sent from the
     * queue as the endpoint frees up; a line that does not fit is dropped
     * and counted. Nobody reading the port used to stop the M0 in the USB
     * queue (seen: 11-21 s dwells, lost baseband messages). */
    bool usb_line(const char* line);
    void usb_drain();
    void usb_release();                    /* the queue goes (exit; or a map short of heap) */
    std::unique_ptr<UsbLineQueue> usbq_{}; /* 2 KB of M0 heap, only once a USB output is used, not while a map is open */
    bool usbq_sent_{false};                /* a transfer was started from usbq_ */
    uint32_t usb_dropped_{0};
    void stat_line();
    void put_time(Fmt& f);
    void log_line(const char* line);
    void open_log();
    void close_log();
    void new_log_name();
    void log_failed(const char* what, const char* why);
    void log_tick();
    void row_begin(Fmt& f, const char* event, const char* src);
    /* POS_LOAD / POS_SAVE row: the shared position file (Setup Load / Save). */
    void log_position(const char* event, const saved_position::Position& p);
    /* The manual position as a shared-file record (source, time from man_meta). */
    bool manual_position(saved_position::Position& p) const;
    bool log_on() const { return opt_.csv & 1; }
    bool survey_on() const { return (opt_.csv & 3) == 3; }
    void heartbeat_row();
    uint32_t uptime_ms() const;
    void start_row(const char* event = "START");
    void survey_ble(const DroneDetectRidMessage& message, const AdvInfo& a);
    void survey_ble_row(const char* event, const BleSeen& s);
    void survey_nrg_row(const char* event, const NrgSeen& s);
    void survey_tick();
    void survey_flush();
    /* Signature classes. */
    void scan_classes();
    void select_class(size_t index);
    void load_class();
    bool sig_ready() const { return sig_file_ && sig_loaded_ && sig_mem_; }
    /* A built-in drone detector (RX_RID / RX_DJI / RX_RFCLS) is on: always in
     * the Drones class, in a file class only when it asks for it. */
    bool detector_on(uint8_t rx_bit) const { return !sig_file_ || (sig_ready() && (sig_mem_->cls.rx & rx_bit)); }
    uint32_t rid_skip_{0};      /* Remote ID / DJI frames not taken as detections (class without them) */
    uint8_t mode() const;       /* opt_.mode bits as the class has them */
    uint16_t band_mask() const; /* RfBand bits swept */
    void sig_dwell_end(uint16_t ms);
    void report_sig(Tracker::Event ev, const Entity& e);
    /* Track view (fox hunt). */
    void open_track(const TrackTarget& t);
    bool target_of(const Entity& e, TrackTarget& t) const;
    void end_phase(); /* accounting of the dwell in progress, before a switch */
    void track_radio(const TrackTarget& t, uint8_t ble_channel);
    void track_end();
    void listen(rf::Frequency f); /* hand-off: the stock Audio app (AM) at f, this app closes */
    /* Capture (IQ for the PC tool): this app closes, its capture view opens
     * with the stock capture baseband, RecordView and waterfall. */
    void capture(CaptureContext c);
    void capture_base(CaptureContext& c, const char* kind, const char* label, uint32_t row_ms) const;
    bool capture_of(const Entity& e, CaptureContext& c) const;
    void capture_of_target(const TrackTarget& t, CaptureContext& c) const;
    void on_meter(const DroneDetectMeterMessage& message);
    /* FSK rules (Z-Wave is one). */
    bool zw_on() const { return sig_ready() && sig_mem_->cls.nfskg && !rf_paused_; }
    void start_zw(bool rate_switch, bool keep);
    void fsk_radio(uint8_t group);
    void on_fsk(const DroneDetectFskMessage& message);
    void on_zwave(const DroneDetectFskMessage& message);
    void zw_row(const char* event, const ZwNet& n, const ZwHeader* h, int16_t rssi_cdb);
    void fsk_row(const char* event, uint8_t rule, const DroneDetectFskMessage* m);
    void update_async_tx();
    bool rf_memory(bool on);
    void pause_rf(bool on);
    void set_demo(bool on);
    void open_map(Entity::Key key);
    void refresh_map();
    void refresh_status();
    void beep();
    void report(Tracker::Event ev, const Entity& e);
    void raw_line(const DroneDetectRidMessage& message, bool odid);
    static void on_tracker_event(void* ctx, Tracker::Event ev, const Entity& e);

    void on_frame_sync();
    void on_tick_second();
    void on_rid(const DroneDetectRidMessage& message);
    void on_stats(const DroneDetectStatsMessage& message);
    void on_gps(const GPSPosDataMessage& message);

    MessageHandlerRegistration message_handler_rid{
        Message::ID::DroneDetectRid,
        [this](Message* const p) {
            this->on_rid(*static_cast<const DroneDetectRidMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_stats{
        Message::ID::DroneDetectStats,
        [this](Message* const p) {
            this->on_stats(*static_cast<const DroneDetectStatsMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_hit{
        Message::ID::DroneDetectPatternHit,
        [this](Message* const p) {
            this->on_pattern_hit(*static_cast<const DroneDetectPatternHitMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_wifi{
        Message::ID::DroneDetectWifi,
        [this](Message* const p) {
            this->on_wifi(*static_cast<const DroneDetectWifiMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_droneid{
        Message::ID::DroneDetectDroneId,
        [this](Message* const p) {
            this->on_droneid(*static_cast<const DroneDetectDroneIdMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_fsk{
        Message::ID::DroneDetectFsk,
        [this](Message* const p) {
            this->on_fsk(*static_cast<const DroneDetectFskMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_meter{
        Message::ID::DroneDetectMeter,
        [this](Message* const p) {
            this->on_meter(*static_cast<const DroneDetectMeterMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_gps{
        Message::ID::GPSPosData,
        [this](Message* const p) {
            this->on_gps(*static_cast<const GPSPosDataMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_captured{
        Message::ID::SpecAnCaptured,
        [this](const Message* const p) {
            this->captured_rx_++;
            if (this->sweep_) this->sweep_->on_captured(*reinterpret_cast<const SpecAnCapturedMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_slice{
        Message::ID::SpecAnSlice,
        [this](const Message* const p) {
            this->on_slice(*reinterpret_cast<const SpecAnSliceMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_frame_sync();
        }};
};

/* Everything known about one detection, refreshed every second. */
class DetailView : public View {
   public:
    DetailView(NavigationView& nav, DroneDetectView& main, Entity::Key key);
    ~DetailView();

    void focus() override;
    std::string title() const override { return "Detection"; };

    void refresh();

   private:
    static constexpr size_t kRows = 12;

    DroneDetectView& main_;
    Entity::Key key_;
    Line lines_[kRows]{};
    TextLines text_{
        {0, 0, UI_POS_MAXWIDTH, UI_POS_HEIGHT(kRows)},
        lines_,
        kRows};
    Button button_map_{
        {UI_POS_X_TABLE(4, 0), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Map"};
    Button button_track_{
        {UI_POS_X_TABLE(4, 1), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Track"};
    Button button_cap_{
        {UI_POS_X_TABLE(4, 2), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Cap"};
    Button button_back_{
        {UI_POS_X_TABLE(4, 3), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Back"};
};

/* Track view meter: big level digits and a bar with peak hold, min, max. */
class TrackLevel : public Widget {
   public:
    TrackLevel(Rect parent_rect, const TrackStats& stats)
        : Widget{parent_rect}, stats_{stats} {}
    void paint(Painter& painter) override;
    int16_t mean_cdb{TrackStats::kNone}; /* small text: interval mean (frequency targets) */
    bool clip{false};

   private:
    const TrackStats& stats_;
};

/* Track view history: one column per 100 ms, the last 20 s. */
class TrackHistory : public Widget {
   public:
    TrackHistory(Rect parent_rect, const TrackStats& stats)
        : Widget{parent_rect}, stats_{stats} {}
    void paint(Painter& painter) override;

   private:
    const TrackStats& stats_;
};

/* Fox hunt: one frequency (PDRN MODE_METER: gap-free pulse peak, 20 reports
 * a second) or one BLE advertiser (address, or the class's rule for rotating
 * addresses). The detection cycle pauses while it is open. */
class TrackView : public View {
   public:
    TrackView(NavigationView& nav, DroneDetectView& main, const TrackTarget& target);
    ~TrackView();
    TrackView(const TrackView&) = delete;
    TrackView& operator=(const TrackView&) = delete;

    void focus() override;
    std::string title() const override { return "Track"; };

    const TrackTarget& target() const { return target_; }
    void on_meter(const DroneDetectMeterMessage& m);
    void on_ble(int16_t rssi_cdb);
    void frame(uint32_t now_ms); /* every display frame */
    void second();               /* once a second: status line, TRK row */

   private:
    NavigationView& nav_;
    DroneDetectView& main_;
    TrackTarget target_;
    TrackStats stats_{};
    Geiger geiger_{};
    PacketRate rate_{};
    uint8_t ble_idx_{0};
    uint32_t hop_ms_{0};
    uint32_t hist_paint_ms_{0};
    uint32_t clip_s_{0};
    bool clip_{false};
    bool audio_started_{false};
    DroneDetectMeterMessage last_{};
    uint32_t meter_rx_{0};

    void retune();
    void show_target();
    void update_status();
    void beep_check(uint32_t now_ms);
    void log_row(const char* event);

    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "LNA:   VGA:   AMP:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(20), UI_POS_Y(2)}, "Sq", Theme::getInstance()->fg_light->foreground}};
    LNAGainField field_lna_{{UI_POS_X(4), UI_POS_Y(0)}};
    VGAGainField field_vga_{{UI_POS_X(11), UI_POS_Y(0)}};
    RFAmpField field_amp_{{UI_POS_X(18), UI_POS_Y(0)}};
    Checkbox check_beep_{{UI_POS_X(22), UI_POS_Y(0)}, 4, "Beep", true};
    Line target_line_[1]{};
    TextLines text_target_{{0, UI_POS_Y(1), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, target_line_, 1};
    FrequencyField field_freq_{{UI_POS_X(0), UI_POS_Y(2)}};
    OptionsField field_bw_{{UI_POS_X(13), UI_POS_Y(2)}, 4, {{"890k", 0}, {"220k", 1}, {" 55k", 2}, {" 14k", 3}}};
    OptionsField field_ble_{{UI_POS_X(0), UI_POS_Y(2)}, 9, {{"address", TrackTarget::BLE_ADDR}, {"rule", TrackTarget::BLE_RULE}}};
    NumberField field_sq_{{UI_POS_X(23), UI_POS_Y(2)}, 4, {-120, 0}, 1, ' '};
    Line status_lines_[2]{};
    TextLines text_status_{{0, UI_POS_Y(3), UI_POS_MAXWIDTH, UI_POS_HEIGHT(2)}, status_lines_, 2};
    TrackLevel level_{{0, UI_POS_Y(5), UI_POS_MAXWIDTH, 72}, stats_};
    TrackHistory history_{{0, UI_POS_Y(5) + 76, UI_POS_MAXWIDTH, UI_POS_Y_BOTTOM(3) - UI_POS_Y(5) - 80}, stats_};
    Button button_reset_{
        {UI_POS_X_TABLE(4, 0), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Reset"};
    Button button_listen_{
        {UI_POS_X_TABLE(4, 1), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Listen"};
    Button button_cap_{
        {UI_POS_X_TABLE(4, 2), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Cap"};
    Button button_back_{
        {UI_POS_X_TABLE(4, 3), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Back"};
};

/* Something not in the list: a typed frequency, or one of the survey's last
 * unknowns (energy events, unmatched BLE advertisers: Survey must be on) -
 * to track or to capture. */
class TargetView : public View {
   public:
    TargetView(NavigationView& nav, DroneDetectView& main);
    void focus() override;
    std::string title() const override { return "Target"; };

   private:
    struct Item {
        uint8_t kind; /* 0 manual frequency, 1 energy (NRG), 2 BLE advertiser */
        uint8_t band, pdu, nuuid, mfr_n;
        int8_t db;
        bool has_cid, has_sd;
        uint16_t bw_khz, cid, uuid, sd;
        uint32_t f_khz, age_s, row_ms;
        uint8_t addr[6];
        uint8_t mfr[4];
    };
    static constexpr size_t kItems = 10;

    NavigationView& nav_;
    DroneDetectView& main_;
    Item items_[kItems]{};
    size_t n_{0};

    void show();
    void fill(CaptureContext& c, TrackTarget& t, const Item& it) const;

    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "Target (wheel):", Theme::getInstance()->fg_light->foreground}};
    OptionsField field_{{UI_POS_X(0), UI_POS_Y(1)}, 29, {{"Manual frequency", 0}}};
    Line lines_[6]{};
    TextLines text_{{0, UI_POS_Y(3), UI_POS_MAXWIDTH, UI_POS_HEIGHT(6)}, lines_, 6};
    Button button_track_{
        {UI_POS_X_TABLE(3, 0), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(33), UI_POS_HEIGHT(2)},
        "Track"};
    Button button_cap_{
        {UI_POS_X_TABLE(3, 1), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(33), UI_POS_HEIGHT(2)},
        "Capture"};
    Button button_back_{
        {UI_POS_X_TABLE(3, 2), UI_POS_Y_BOTTOM(3), UI_POS_WIDTH_PERCENT(33), UI_POS_HEIGHT(2)},
        "Back"};
};

/* Capture view (opened in place of Drone Detect, which closes: the stock
 * capture baseband replaces PDRN). The stock widgets and record path - the
 * frequency field, gains, the spectrum bandwidth options, RecordView (IQ +
 * .TXT metadata, aligned 16 KB writes) and the waterfall - plus a duration,
 * the off / on state, a label and an antenna note. Each recording is moved
 * from /SIGDET/CAPTURES/CAPTMP.* into the session folder
 * /SIGDET/CAPTURES/<date>_<time>_<label>/ as P<nn>_OFF / P<nn>_ON with a
 * .SIG sidecar, and logged as CAP_START / CAP_STOP in the session CSV. */
class DdCaptureView : public View {
   public:
    DdCaptureView(NavigationView& nav, const CaptureContext& ctx);
    ~DdCaptureView();
    DdCaptureView(const DdCaptureView&) = delete;
    DdCaptureView& operator=(const DdCaptureView&) = delete;

    void focus() override;
    void set_parent_rect(const Rect new_parent_rect) override;
    std::string title() const override { return "Sig Capture"; };

   private:
    static constexpr Dim header_height = 5 * 16;

    NavigationView& nav_;
    CaptureContext ctx_;
    RxRadioState radio_state_{ReceiverModel::Mode::Capture};
    std::string label_{};
    std::string ant_{};
    CapPairs pairs_{};
    char session_[64]{}; /* the session folder, made at the first recording */
    char base_[16]{};    /* P01_OFF of the recording in progress */
    uint32_t rate_{0};
    uint32_t t0_ms_{0};
    rtc::RTC t0_rtc_{};
    uint8_t pair_{0}, state_{0};
    uint8_t lna_{0}, vga_{0}, amp_{0};

    void on_rate(uint32_t rate);
    void poll();
    bool before_start();
    void prepare_and_start();
    void finish_step();
    void recover();
    void stage(const char* text, const Style* style);
    void show_size();
    void log_row(const char* event, uint32_t duration_ms, uint64_t bytes);
    uint32_t duration_ms() const;

    /* The M0 main stack has ~330 bytes to spare at the depth of an event
     * handler (sysinfo). build-capture2 put a File (FIL with its 512 byte
     * sector buffer), a 1 KB sidecar buffer and the row buffer on the stack
     * inside the tick callback: 1,056 / 2,136 byte frames (-fstack-usage),
     * "Stack Overflow". Everything big is a member now (this view lives on the
     * heap, and Drone Detect is closed), and the finish runs one step per
     * tick, its step on the status line.
     * build-capture3: no SD access from this thread while the capture thread
     * runs. FatFs validate() reads the SD driver state without the volume
     * lock, and the driver is BLK_READING / BLK_WRITING during the other
     * thread's transfer: disk_status() says STA_NOINIT, FR_INVALID_OBJECT.
     * So the Record press only asks (STEP_PREP); the next tick does the folder,
     * the name and the CAP_START row, then starts RecordView; the save steps
     * run after the capture thread has been joined. */
    enum Step : uint8_t {
        STEP_IDLE,
        STEP_PREP,     /* Record pressed: folder, name, CAP_START row, then start */
        STEP_REC,      /* recording: no SD access here */
        STEP_MOVE,     /* stopped: move the data and metadata */
        STEP_SIG,      /* write the .SIG */
        STEP_LOG_STOP, /* CAP_STOP row */
    };
    Step step_{STEP_IDLE};
    bool recovered_{false};
    bool starting_{false}; /* our own record_view_.start() call */
    File file_{};          /* one file at a time */
    char buf_[1024]{};     /* sidecar text / CSV row */
    char pa_[96]{};        /* paths */
    char pb_[96]{};
    char tmp_stem_[12]{}; /* CAPTMP or CAPTMP_GEO */
    uint64_t bytes_{0};
    bool moved_{false};
    bool sig_ok_{false};
    CapSidecar sc_{};
    char sc_file_[24]{};
    char sc_meta_[24]{};

    RSSI rssi_{{UI_POS_X(24), 0, UI_POS_WIDTH_REMAINING(24), 4}};
    Channel channel_{{UI_POS_X(24), 5, UI_POS_WIDTH_REMAINING(24), 4}};
    RxFrequencyField field_frequency_{{UI_POS_X(0), UI_POS_Y(0)}, nav_};
    RFAmpField field_amp_{{16 * 8, UI_POS_Y(0)}};
    LNAGainField field_lna_{{18 * 8, UI_POS_Y(0)}};
    VGAGainField field_vga_{{21 * 8, UI_POS_Y(0)}};
    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(1)}, "Rate:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(11), UI_POS_Y(1)}, "Dur:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(2)}, "Lbl:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(17), UI_POS_Y(2)}, "Ant:", Theme::getInstance()->fg_light->foreground}};
    OptionsField option_rate_{{UI_POS_X(5), UI_POS_Y(1)}, 5, {}};
    OptionsField option_dur_{{UI_POS_X(15), UI_POS_Y(1)}, 3, {{" 5s", 5}, {"10s", 10}, {"30s", 30}, {"man", 0}}};
    OptionsField option_state_{{UI_POS_X(20), UI_POS_Y(1)}, 10, {{"device OFF", 0}, {"device ON", 1}}};
    TextField text_label_{{UI_POS_X(4), UI_POS_Y(2), UI_POS_WIDTH(12), UI_POS_HEIGHT(1)}, ""};
    TextField text_ant_{{UI_POS_X(21), UI_POS_Y(2), UI_POS_WIDTH(9), UI_POS_HEIGHT(1)}, ""};
    RecordView record_view_{
        {UI_POS_X(0), UI_POS_Y(3), screen_width, UI_POS_HEIGHT(1)},
        u"CAPTMP",
        u"/SIGDET/CAPTURES",
        RecordView::FileType::RawS16,
        kWriteSize,
        3};
    Text text_status_{{UI_POS_X(0), UI_POS_Y(4), UI_POS_WIDTH(22), UI_POS_HEIGHT(1)}, ""};
    Button button_dd_{{UI_POS_X(23), UI_POS_Y(4), UI_POS_WIDTH(7), UI_POS_HEIGHT(1)}, "Detect"};
    spectrum::WaterfallView waterfall_{};

    /* No MessageHandlerRegistration in this view: the event loop allows ONE
     * handler per message ID, and the stock widgets above already take
     * DisplayFrameSync, ChannelSpectrumConfig, AudioSpectrum (waterfall),
     * RSSIStatistics, PitchRSSIConfigure (RSSI), ChannelStatistics (Channel),
     * CaptureThreadDone, GPSPosData (RecordView). A frame-sync handler here
     * was the "MsgDblReg" halt of build-capture. Polled once a second
     * instead (a signal, which takes any number of listeners). */
    SignalToken tick_token_{};
    uint32_t dropped_pct_{0}; /* dropped share of this recording, % (from the byte counters) */
    uint64_t recv_bytes_{0};  /* the capture thread's counters, last read (each tick and at our stop) */
    uint64_t drop_bytes_{0};
    uint32_t wall_ms_{0};                         /* start to stop, wall clock */
    static constexpr uint32_t kWriteSize = 16384; /* record_view_'s write buffer */
    uint32_t drop_pct() const;
    uint32_t bb_filter_hz_{0};
};

class SettingsView : public View {
   public:
    SettingsView(NavigationView& nav, DroneDetectView& main);

    void focus() override;
    /* Back from the position picker: sweeps may run again. */
    void on_show() override;
    std::string title() const override { return "SigDet setup"; };

   private:
    DroneDetectView& main_;

    void show_position();
    void load_shared();
    void save_shared();

    /* Gains, amp and Log are on the main view (stock RX app layout). */
    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "Log: LOGS/SIGDET_<time>.CSV", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(8)}, "Manual pos:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(14)}, "RF bands:", Theme::getInstance()->fg_light->foreground}};
    Checkbox check_alert_{
        {UI_POS_X(0), UI_POS_Y(2)},
        14,
        "Beep on drone"};
    Checkbox check_json_{
        {UI_POS_X(0), UI_POS_Y(4)},
        14,
        "USB JSON lines"};
    Checkbox check_esp_{
        {UI_POS_X(0), UI_POS_Y(6)},
        10,
        "ESP32 feed"};
    /* Source and age of the manual position ("Map 2h", "HackRF GPS 3d"). */
    Text text_src_{
        {UI_POS_X(12), UI_POS_Y(8), UI_POS_WIDTH(18), UI_POS_HEIGHT(1)},
        ""};
    Text text_pos_{
        {UI_POS_X(0), UI_POS_Y(9), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)},
        ""};
    /* Load / Save: the position shared by all apps (/SETTINGS/position.ini,
     * saved_position.hpp); the SD card is used only when pressed. */
    Button button_set_{
        {0, UI_POS_Y(10), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "On map"};
    Button button_clear_{
        {static_cast<Coord>(UI_POS_WIDTH_PERCENT(25)), UI_POS_Y(10), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Clear"};
    Button button_load_{
        {static_cast<Coord>(UI_POS_WIDTH_PERCENT(50)), UI_POS_Y(10), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Load"};
    Button button_save_{
        {static_cast<Coord>(UI_POS_WIDTH_PERCENT(75)), UI_POS_Y(10), UI_POS_WIDTH_PERCENT(25), UI_POS_HEIGHT(2)},
        "Save"};
    Text text_msg_{
        {UI_POS_X(0), UI_POS_Y(12), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)},
        ""};
    Checkbox check_rid_{{UI_POS_X(0), UI_POS_Y(13)}, 7, "BLE RID", true};
    Checkbox check_rf_{{UI_POS_X(10), UI_POS_Y(13)}, 8, "RF sweep", true};
    Checkbox check_eu_{{UI_POS_X(21), UI_POS_Y(13)}, 6, "EU 868", true};
    /* Pattern scan (dd_pattern.hpp): built-in RC toy + /SIGDET/PATTERNS.TXT. */
    Checkbox check_pat_{{UI_POS_X(0), UI_POS_Y(18)}, 12, "Pattern scan", true};
    /* Wi-Fi Remote ID on the HackRF itself: 802.11b 1 Mb/s beacons / NAN. */
    Checkbox check_wifi_{{UI_POS_X(0), UI_POS_Y(17)}, 14, "Wi-Fi RID (1M)", true};
    /* DJI DroneID burst detector dwells (presence only; checked on recordings only). */
    Checkbox check_did_{{UI_POS_X(17), UI_POS_Y(18)}, 7, "DroneID", true};
    /* In frequency order, three after the "RF bands:" label and three below
     * (a small checkbox with 3 characters is 42 px wide: six do not fit in
     * one row). */
    Checkbox check_b4_{{UI_POS_X(10), UI_POS_Y(14)}, 3, "900", true};
#if DD_FULL
    Checkbox check_b3_{{UI_POS_X(16), UI_POS_Y(14)}, 3, "1.2", true};
#endif
    Checkbox check_b0_{{UI_POS_X(22), UI_POS_Y(14)}, 3, "2.4", true};
    Checkbox check_b5_{{UI_POS_X(0), UI_POS_Y(15)}, 3, "5.1", true};
#if DD_FULL
    Checkbox check_b2_{{UI_POS_X(6), UI_POS_Y(15)}, 3, "5.3", true};
#endif
    Checkbox check_b1_{{UI_POS_X(12), UI_POS_Y(15)}, 3, "5.8", true};
#if DD_FULL
    Checkbox check_beep_low_{
        {UI_POS_X(0), UI_POS_Y(16)},
        13,
        "Beep low conf",
        true};
    /* 2.4 GHz hopper / toy verdicts: off by default (household Bluetooth and
     * Wi-Fi look the same to an energy sweep). */
    Checkbox check_hop24_{
        {UI_POS_X(17), UI_POS_Y(16)},
        8,
        "2.4G hop",
        true};
#endif
};

/* Bring-up tools: demo drones, raw advert console, demodulator switches. */
class DebugView : public View {
   public:
    DebugView(NavigationView& nav, DroneDetectView& main);
    ~DebugView();

    void focus() override;
    std::string title() const override { return "SigDet debug"; };

    void show_stats();
#if DD_FULL
    void write(const char* line); /* raw frame console */
#endif
    /* Bring-up keys (USB shell "keyboard XX", XX = ASCII hex), so captures
     * can be scripted: o dump off, a dump all bands, 0-6 dump one band (RfBand),
     * r raw looks, s DDSTAT lines, p frame pacing, t settle 100/50/0 us,
     * v FFT average auto/4/2/1, z reset counters, w Wi-Fi channels 6 /
     * 1+6+11, l forward French / legacy DJI Wi-Fi IEs (raw, console),
     * k Wi-Fi search phases default/4/3/2 (cost comparison, DDSTAT). */
    bool on_keyboard(const KeyboardEvent key) override;

   private:
    static constexpr uint8_t kBle = DroneDetectView::kBle;
    DroneDetectView& main_;

#if DD_FULL
    Checkbox check_demo_{
        {UI_POS_X(0), UI_POS_Y(0)},
        4,
        "Demo"};
#endif
    Checkbox check_pass_all_{
        {UI_POS_X(15), UI_POS_Y(0)},
        8,
        "Pass all"};
#if DD_FULL
    /* Demodulate the mirrored spectrum, should I/Q be swapped. */
    Checkbox check_conjugate_{
        {UI_POS_X(0), UI_POS_Y(2)},
        3,
        "Inv"};
#endif
    Checkbox check_usb_{
        {UI_POS_X(15), UI_POS_Y(2)},
        7,
        "USB raw"};
    OptionsField opt_dump_{
        {UI_POS_X(5), UI_POS_Y(4)},
        3,
        {{"off", kBle},
         {"all", DroneDetectView::kDumpAll},
         {"2.4", BAND_24},
         {"5.1", BAND_51},
         {"5.8", BAND_58},
#if DD_FULL
         {"5.3", BAND_53},
         {"1.2", BAND_12},
#endif
         {"915", BAND_915},
         {"868", BAND_868}}};
#if DD_FULL
    Checkbox check_sd_{
        {UI_POS_X(10), UI_POS_Y(4)},
        12,
        "+SD SIGDET",
        true};
#endif
    /* Baseband counters and sweep rates, lines of at most 30 characters. */
    static constexpr size_t kRows = 13;
    Line lines_[kRows]{};
    TextLines text_{
        {0, UI_POS_Y(5), UI_POS_MAXWIDTH, UI_POS_HEIGHT(kRows)},
        lines_,
        kRows};
    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(4)}, "Dump", Theme::getInstance()->fg_light->foreground}};
#if DD_FULL
    Console console_{
        {UI_POS_X(0), UI_POS_Y(18), UI_POS_MAXWIDTH, UI_POS_HEIGHT_REMAINING(19)}};
#endif
};

#else

/* The PDRN baseband and its 4 Msps BLE demodulator are sized for the HackRF
 * Pro (LPC4330); elsewhere the app only says so. */
class DroneDetectView : public View {
   public:
    DroneDetectView(NavigationView& nav);

    void focus() override;
    std::string title() const override { return "Sig Detect"; };

   private:
    NavigationView& nav_;
    Text text_info{
        {UI_POS_X(0), UI_POS_Y(2), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)},
        "Requires HackRF Pro"};
    Button button_back{
        {UI_POS_X_CENTER(8), UI_POS_Y(4), UI_POS_WIDTH(8), UI_POS_HEIGHT(2)},
        "Back"};
};

#endif

} /* namespace ui::external_app::drone_detect */

#endif /*__UI_DRONE_DETECT_H__*/
