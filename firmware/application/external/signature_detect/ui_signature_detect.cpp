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

#include "ui_signature_detect.hpp"

#include "baseband_api.hpp"
#include "portapack.hpp"

#ifdef PRALINE
#include "audio.hpp"
#include "file_path.hpp"
#include "radio.hpp"
#include "rtc_time.hpp"
#include "usb_serial_asyncmsg.hpp"

#include "sigdet_fmt.hpp"
#include "sigdet_ble_vendor.hpp"
#include "sigdet_log.hpp"
#include "apps/analog_audio_app.hpp"
#include "apps/ui_freqman.hpp"
#include "ui_textentry.hpp"
#include "debug.hpp"

#include <cstring>
#include <new>

extern "C" {
#include "usb_serial_endpoints.h"
#include <common/usb.h>
}
#endif

using namespace portapack;

#ifdef PRALINE

namespace ui::external_app::signature_detect {
namespace {

/* Time base of the Tracker: seconds since boot. */
uint32_t clock_s() {
    return chTimeNow() / CH_FREQUENCY;
}

/* Source bits as list letters: B on-board BLE, W ESP32 WiFi, E ESP32 BLE, D demo, R RF sweep, P pattern scan,
 * H on-board (HackRF) Wi-Fi, O on-board DJI DroneID (OcuSync) burst detector. */
void put_sources(Fmt& f, uint8_t sources, size_t max) {
    static constexpr char letters[] = "BWEDRPHO";
    size_t n = 0;
    for (size_t i = 0; i < 8 && n < max; i++)
        if (sources & (1u << i)) {
            f.c(letters[i]);
            n++;
        }
}

/* Distance in at most 4 characters for the list (the detail view has it
 * to the metre): 999m, 9.9k, 999k, "far" beyond (a bad position). */
void put_dist4(Fmt& f, uint32_t m) {
    if (m < 1000)
        f.d(m).c('m');
    else if (m < 10000)
        f.d(m / 1000).c('.').c(static_cast<char>('0' + (m / 100) % 10)).c('k');
    else if (m < 1000000)
        f.d(m / 1000).c('k');
    else
        f.s("far");
}

/* One transfer from the USB line queue on the CDC bulk IN endpoint; false
 * while its single transfer slot is busy. No completion callback: see
 * sigdet_usbq.hpp. Never waits. */
bool usb_send(void*, const uint8_t* data, size_t len) {
    return usb_transfer_schedule(&usb_endpoint_bulk_in, const_cast<uint8_t*>(data), len, nullptr, nullptr) == 0;
}

}  // namespace
}  // namespace ui::external_app::signature_detect

namespace ui {

template <>
void RecentEntriesTable<external_app::signature_detect::Tracker>::draw(
    const Entry& entry,
    const Rect& target_rect,
    Painter& painter,
    const Style& style,
    RecentEntriesColumns& columns) {
    using namespace external_app::signature_detect;

    /* A signature hit has no source letters or distance to show: its label
     * takes the Src column too, its kind tag (TRKR ...) the Dist column. */
    const bool sig = entry.kind == Kind::Sig || entry.kind == Kind::Zwave || entry.kind == Kind::Fsk || entry.kind == Kind::Scan;
    char name[24];
    entity_name(entry, name, sizeof(name));
    size_t col = columns.at(0).second;
    const size_t name_w = sig ? col + 1 + columns.at(1).second : col;
    if (name_w < sizeof(name)) name[name_w] = 0;

    char line[Line::kCols + 1];
    Fmt f{line, sizeof(line)};
    f.s(name).pad(col + 1);
    col += 1 + columns.at(1).second;
    const bool rf = entry.kind == Kind::Rf;
    if (sig) {
        /* the label is there */
    } else if (rf) {
        f.s(rf_class_tag(entry.rf_cls));
    } else if (entry.kind == Kind::Pattern) {
        f.s(entry.cls);
    } else if (entry.kind == Kind::BleVendor) {
        f.s("DJI");
    } else {
        put_sources(f, entry.sources, columns.at(1).second);
    }
    f.pad(col + 1);
    col += 1 + columns.at(2).second;
    if (entry.rssi != kNoRssi)
        f.d(entry.rssi < -99 ? -99 : entry.rssi).c(entry.trend());
    else
        f.c('-');
    f.pad(col + 1);
    col += 1 + columns.at(3).second;
    if (rf)
        f.c("-LMH"[entry.rf_conf & 3]);
    else if (sig)
        f.s(entry.cls); /* kind tag; NB hits carry '?' in the label (low confidence) */
    else if (entry.kind == Kind::Pattern) {
        const uint32_t r = entry.rate10 / 10;
        if (r < 100)
            f.d(r).s("/s");
        else
            f.d(r < 999 ? r : 999);
    } else if (entry.dist_m != kNoDistance)
        put_dist4(f, entry.dist_m);
    else
        f.c('-');
    f.pad(col + 1);
    f.d(clock_s() - entry.last_s).c('s');
    f.pad(target_rect.width() / UI_POS_DEFAULT_WIDTH);

    const Style dim{style.font, style.background, Theme::getInstance()->fg_medium->foreground};
    painter.draw_string(target_rect.location(), entry.stale ? dim : style, line);
}

}  // namespace ui

namespace ui::external_app::signature_detect {

namespace {

using namespace std::literals;

#if SIGDET_FULL
constexpr int32_t kDemoLat = 515007000; /* demo centre without own position */
constexpr int32_t kDemoLon = -1246000;
#endif
/* Setup band bits swept (SIGDET_FULL 0: no 5.3 and 1.2 GHz); bit 5 (5.1 GHz) is
 * stored inverted, so that settings saved before it sweep it too. */
constexpr uint8_t kBandMask = SIGDET_FULL ? 0x3F : 0x33;
constexpr uint8_t kBandInverted = 0x20;
constexpr uint32_t kGpsMaxAge = 30;
/* Every band is swept at 20 Msps; one tune covers 15 MHz there. */
constexpr uint32_t spec_an_sweep_fs = 20000000;
constexpr uint16_t kOneTuneMhz = 15;
constexpr uint32_t kBeepGap = 3;

const char* const kIdTypes[] = {"None", "Serial", "CAA", "UTM", "Session"};
const char* const kUaTypes[16] = {
    "UA ?", "Aeroplane", "Multirotor", "Gyroplane", "VTOL", "Ornithopter", "Glider", "Kite",
    "Free balloon", "Capt balloon", "Airship", "Parachute", "Rocket", "Tethered", "Ground obst", "Other UA"};
const char* const kStatus[] = {"Undeclared", "Ground", "Airborne", "Emergency", "RID failure"};
const char* const kOpLoc[] = {"Takeoff", "Live GNSS", "Fixed", "?"};

const char* pick(const char* const* names, size_t count, size_t i) {
    return i < count ? names[i] : "?";
}

float deg(int32_t e7) {
    return static_cast<float>(e7) / 1e7f;
}

int32_t to_e7(float deg) {
    return static_cast<int32_t>(deg * 1e7f);
}

/* Frequency of a BLE advertising channel (37..39). */
rf::Frequency ble_channel_frequency(uint8_t channel) {
    return 2402000000 + (channel == 38 ? 24000000 : (channel == 39 ? 78000000 : 0));
}

/* Log format 2 (sdcard/SIGDET/README.TXT): 14 columns in every row. */
constexpr const char* kCsvHeader = "uptime_ms,time,event,src,id,label,kind,freq_khz,bw_khz,level,unit,lat,lon,detail";
constexpr uint32_t kLogSyncMs = 2000;         /* rows reach the card within this */
constexpr uint32_t kLogRetrySeconds = 30;     /* after an error: retries back off to this */
constexpr uint8_t kLogNewPartAfter = 3;       /* failed attempts in a row before a new part */
constexpr uint32_t kLogPartBytes = 16u << 20; /* a new part past 16 MB */
constexpr uint32_t kHeartbeatSeconds = 60;
/* The SD folder: /SIGDET; for one release the classes and PATTERNS.TXT are
 * read from /DRONEDET (the app's old name) when /SIGDET/SIGS is missing, and
 * nothing is written there (sd_root()). */
constexpr const char* kSdRoot = "/SIGDET";
constexpr const char* kSdRootOld = "/DRONEDET";
constexpr size_t kMaxClasses = 12; /* files listed on the wheel */
/* Z-Wave dwell in a class with other receivers: a door operation is 0.6-1.3 s
 * of frames ~70 ms apart, so 1.5 s holds several of them whenever the dwell
 * overlaps it; alone in a class the dwells run back to back (100 %). */
constexpr uint16_t kZwDwellMs = 1500;

/* M0 heap a map view takes besides its own object: the map line buffer
 * (480 B), the FatFs name buffer while it opens its files (512 B), GeoPos'
 * strings and lists, the "No map" modal (~0.6 KB) without world_map.bin. */
constexpr size_t kMapHeapExtra = 2048;

/* The heap can give a block of big bytes (0: none) and then one of small
 * bytes: chHeapAlloc returns null where operator new would halt the firmware
 * ("Out of Memory"). Freed again at once, so the view then gets the same
 * blocks. */
bool heap_fits(size_t big, size_t small) {
    void* const b = big ? chHeapAlloc(nullptr, big) : nullptr;
    void* const s = (b || !big) ? chHeapAlloc(nullptr, small) : nullptr;
    if (s) chHeapFree(s);
    if (b) chHeapFree(b);
    return s != nullptr;
}

}  // namespace

/* ---------------------------------------------------------------- SigdetMapView */

/* The stock GeoMapView (~10 KB), placed in the M4 RAM that PSDT lends
 * (SignatureDetectStatsMessage::scratch, above the RF arrays) once the app has
 * it, else on the M0 heap. Opened from the Receive menu, which stays
 * resident under the app (~10 KB of heap with the SD card's external apps),
 * the app leaves about 4 KB of M0 heap: a heap GeoMapView never fits then.
 * NavigationView creates views with new T and deletes them through
 * std::unique_ptr<View>; with View's virtual destructor that delete uses this
 * class's operator delete, so both ends meet here. One map at a time (the
 * slot), a second one would go to the heap. */
class SigdetMapView : public GeoMapView {
   public:
    using GeoMapView::GeoMapView;

    static void* operator new(size_t size) {
        alive_++;
        if (slot_ && !slot_used_ && size <= slot_size_) {
            slot_used_ = true;
            return slot_;
        }
        void* const p = chHeapAlloc(nullptr, size);
        if (!p) chDbgPanic("Out of Memory"); /* as the global operator new; map_fits() comes first */
        return p;
    }
    static void operator delete(void* p) {
        if (!p) return;
        alive_--;
        if (p == slot_)
            slot_used_ = false;
        else
            chHeapFree(p);
    }

    /* The lent memory (nullptr: none); not changed while a view is in it. */
    static void lend(void* mem, size_t size) {
        if (slot_used_) return;
        slot_ = mem;
        slot_size_ = mem ? size : 0;
    }
    /* The next map view goes to the lent memory. */
    static bool lent() { return slot_ && !slot_used_ && sizeof(SigdetMapView) <= slot_size_; }
    /* Map views that exist (main map or the Setup position picker). */
    static uint8_t alive() { return alive_; }

   private:
    static inline void* slot_{nullptr};
    static inline size_t slot_size_{0};
    static inline bool slot_used_{false};
    static inline uint8_t alive_{0};
};
static_assert(sizeof(SigdetMapView) <= SignatureDetectStatsMessage::kScratchMap,
              "GeoMapView grew: raise SignatureDetectStatsMessage::kScratchMap (PSDT's m0_scratch)");
static_assert(sizeof(SigMemory) <= SignatureDetectStatsMessage::kScratchSig,
              "SigMemory grew: raise SignatureDetectStatsMessage::kScratchSig (PSDT's m0_scratch)");
constexpr size_t kSigAt = SignatureDetectStatsMessage::kScratchRf + SignatureDetectStatsMessage::kScratchMap;

/* ---------------------------------------------------------------- TextLines */

void TextLines::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto& s = style();
    const size_t cols = r.width() / UI_POS_DEFAULT_WIDTH;
    for (size_t i = 0; i < rows_; i++) {
        const Line& l = lines_[i];
        size_t n = strlen(l.text);
        if (n > cols) n = cols;
        const Point p{r.left(), static_cast<Coord>(r.top() + i * UI_POS_DEFAULT_HEIGHT)};
        const int w = n ? painter.draw_string(p, s.font, l.color, s.background, std::string_view{l.text, n}) : 0;
        painter.fill_rectangle({p.x() + w, p.y(), r.width() - w, UI_POS_DEFAULT_HEIGHT}, s.background);
    }
}

/* ---------------------------------------------------------------- main view */

SignatureDetectView::SignatureDetectView(NavigationView& nav)
    : nav_{nav},
      settings_{
          "rx_sig_detect",
          app_settings::Mode::RX,
          {{"opts"sv, &opts_},
           {"man_lat"sv, &opt_.man_lat},
           {"man_lon"sv, &opt_.man_lon},
           {"man_meta"sv, &opt_.man_meta},
           {"sig_class"sv, &sig_name_},
           {"trk_khz"sv, &trk_khz_},
           {"trk_opt"sv, &trk_opt_}}} {
    /* First start after the rename (Drone Detect -> Signature Detect): the
     * old settings file, read into the same fields and the radio model. */
    bool settings_loaded = settings_.loaded();
    if (!settings_loaded) {
        app_settings::SettingsManager old{
            "rx_drone_detect",
            app_settings::Mode::RX,
            {{"opts"sv, &opts_},
             {"man_lat"sv, &opt_.man_lat},
             {"man_lon"sv, &opt_.man_lon},
             {"man_meta"sv, &opt_.man_meta},
             {"sig_class"sv, &sig_name_},
             {"trk_khz"sv, &trk_khz_},
             {"trk_opt"sv, &trk_opt_}}};
        settings_loaded = old.loaded();
    }
    /* "opts" is new: an older settings file leaves the defaults; whatever was
     * read, keep every flag in its range. */
    memcpy(&opt_.alert, &opts_, sizeof(opts_));
    opt_.mode &= 0x3F;
    opt_.bands &= 0x3F;
    for (uint8_t* f : {&opt_.alert, &opt_.json, &opt_.esp, &opt_.region, &opt_.beep_low}) *f = *f != 0;
    opt_.csv &= 3;
    async_tx_was_ = portapack::async_tx_enabled;
    baseband::run_image(portapack::spi_flash::image_tag_signature_detect);

    add_children({&labels_, &field_lna_, &field_vga_, &field_amp_, &check_log_, &field_vol_, &text_status_, &labels_sig_, &field_sig_,
                  &check_survey_, &list_, &button_map_, &button_track_, &button_setup_, &button_debug_});
    list_.set_parent_rect({0, UI_POS_Y(4), UI_POS_MAXWIDTH, UI_POS_HEIGHT_REMAINING(7)});
    check_log_.set_value(log_on());
    check_log_.on_select = [this](Checkbox&, bool v) {
        opt_.csv = v ? (opt_.csv | 1) : 0; /* Survey goes with it */
        if (!v) check_survey_.set_value(false);
        apply_options();
    };
    /* Survey rows go into the log: ticking it ticks Log. */
    check_survey_.set_value(survey_on());
    check_survey_.on_select = [this](Checkbox&, bool v) {
        opt_.csv = v ? 3 : (opt_.csv & 1);
        if (v && !check_log_.value()) check_log_.set_value(true);
        apply_options();
    };
    /* A wheel turned through the list changes the class once it rests
     * (kClassSettleMs, on_frame_sync): each change drops every entry and
     * writes a CLASS row (2026-11-14: 19 changes in a session, some 53 ms
     * apart). */
    field_sig_.on_change = [this](size_t index, OptionsField::value_t) {
        sig_pick_ = static_cast<uint8_t>(index);
        sig_pick_ms_ = chTimeNow();
        sig_pick_pending_ = true;
    };

    list_.on_select = [this](const Entity& e) {
        detail_ = nav_.push<DetailView>(*this, e.key());
    };
    button_map_.on_select = [this](Button&) {
        open_map(Entity::invalid_key);
    };
    button_setup_.on_select = [this](Button&) {
        nav_.push<SettingsView>(*this);
    };
    /* Track or capture something not in the list: the typed frequency (the
     * last one, saved) or a survey unknown. */
    button_track_.on_select = [this](Button&) {
        if (track_ || SigdetMapView::alive()) return;
        nav_.push<TargetView>(*this);
    };
    button_debug_.on_select = [this](Button&) {
        debug_ = nav_.push<DebugView>(*this);
    };

    if (!settings_loaded) {
        receiver_model.set_rf_amp(false);
        receiver_model.set_lna(32);
        receiver_model.set_vga(20);
    }
    /* The fields read receiver_model when built, before the defaults above. */
    field_lna_.set_value(receiver_model.lna());
    field_vga_.set_value(receiver_model.vga());
    field_amp_.set_value(receiver_model.rf_amp());

    tracker_.set_notify(on_tracker_event, this);
    rf_.set_notify(on_rf_event, this);
    tick_token_ = rtc_time::signal_tick_second += [this]() {
        on_tick_second();
    };

    /* Each hop and each sweep slice retunes the first LO; without this every
     * retune sleeps 10 ms. */
    radio::set_fast_retune(true);
    receiver_model.set_sampling_rate(sampling_rate);
    receiver_model.enable();
    receiver_model.set_target_frequency(ble_channel_frequency(channel_));
    configure(SignatureDetectConfigMessage::FLAG_RESET_STATS);
    cycle_t0_ = phase_t0_ = chTimeNow();                               /* airtime counts from here, not from boot */
    did_last_end_ = cycle_t0_ - (tune::kDidGapMs - tune::kDidFirstMs); /* the first DroneID dwell after kDidFirstMs */
    next_phase();

    apply_options();
    refresh_status();
}

SignatureDetectView::~SignatureDetectView() {
    memcpy(&opts_, &opt_.alert, sizeof(opts_)); /* settings_ saves it after this */
    /* The survey's last summaries and a STOP row while PSDT's RAM (the
     * tables) is still there; then the file is closed (synced). */
    stop_row("exit");
    close_log();
    sig_mem_ = nullptr;
    rtc_time::signal_tick_second -= tick_token_;
    esp_.enable(false);
    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
    rf_memory(false); /* all three live in PSDT's RAM: not used after this */
    rf_.set_work(nullptr);
    SigdetMapView::lend(nullptr, 0); /* no map is open: it is above this view */
    radio::set_fast_retune(false);
    portapack::async_tx_enabled = async_tx_was_;
    usb_release();
}

/* The USB controller may still be reading a queue buffer (nobody reading
 * the port). Give it 50 ms, then cancel the transfer; then the queue goes
 * (2 KB of heap). The next USB line makes a new one, except while a map is
 * open. */
void SignatureDetectView::usb_release() {
    if (usbq_sent_) {
        for (size_t i = 0; i < 50 && usb_endpoint_is_ready(&usb_endpoint_bulk_in); i++) chThdSleepMilliseconds(1);
        if (usb_endpoint_is_ready(&usb_endpoint_bulk_in)) usb_endpoint_flush(&usb_endpoint_bulk_in);
        usbq_sent_ = false;
    }
    usbq_.reset();
}

void SignatureDetectView::focus() {
    if (tracker_.empty())
        button_setup_.focus();
    else
        list_.focus();
}

uint32_t SignatureDetectView::now_s() const {
    return clock_s();
}

bool SignatureDetectView::gps_fix() const {
    return gps_valid_ && now_s() - gps_s_ <= kGpsMaxAge;
}

bool SignatureDetectView::gps_nofix() const {
    return !gps_fix() && gps_nofix_ && now_s() - gps_nofix_s_ <= kGpsMaxAge;
}

/* A map view (SigdetMapView) goes into PSDT's lent RAM once the first stats
 * message has brought its address (500 ms after the start), else it is one
 * ~10 KB heap block; either way it needs kMapHeapExtra of M0 heap besides.
 * operator new halts the firmware ("Out of Memory") when the heap cannot give
 * it, so ask first (heap_fits). Short of heap with a USB queue (2 KB), the
 * queue goes while the map is open (USB lines are dropped and counted
 * meanwhile) and the heap is asked again. */
bool SignatureDetectView::map_fits() {
    constexpr size_t kMapAt = SignatureDetectStatsMessage::kScratchRf;
    if (stats_.scratch && stats_.scratch_size >= kMapAt + SignatureDetectStatsMessage::kScratchMap)
        SigdetMapView::lend(stats_.scratch + kMapAt, SignatureDetectStatsMessage::kScratchMap);
    const size_t own = SigdetMapView::lent() ? 0 : sizeof(SigdetMapView);
    if (heap_fits(own, kMapHeapExtra)) return true;
    if (usbq_) {
        usb_release();
        if (heap_fits(own, kMapHeapExtra)) return true;
    }
    map_nomem_ = true;
    map_nomem_s_ = now_s();
    refresh_status();
    return false;
}

bool SignatureDetectView::own_position(int32_t& lat, int32_t& lon) const {
    if (gps_fix()) {
        lat = gps_lat_;
        lon = gps_lon_;
        return true;
    }
    if (odid_pos_valid(opt_.man_lat, opt_.man_lon)) {
        lat = opt_.man_lat;
        lon = opt_.man_lon;
        return true;
    }
    return false;
}

void SignatureDetectView::put_pos_src(Fmt& f) const {
    if (gps_fix()) {
        f.s(";pos_src=gps;pos_age_s=").d(now_s() - gps_s_);
        return;
    }
    saved_position::Position p;
    if (!manual_position(p)) return;
    const bool gps = p.source == saved_position::SOURCE_GPS || p.source == saved_position::SOURCE_HACKRF_GPS;
    f.s(";pos_src=").s(p.source == saved_position::SOURCE_MANUAL ? "manual" : (gps ? "saved_gps" : "map"));
    f.s(";pos_age_s=").d(saved_position::age_s(p.time, saved_position::now_fat()));
}

void SignatureDetectView::apply_options() {
    /* The file is opened on the next frame, so the checkbox has been painted
     * before any SD card access (which can take a while or fail). */
    if (log_on() && !log_)
        log_open_ = true;
    else if (!log_on()) {
        stop_row("log_off");
        close_log();
        log_error_[0] = 0;
        log_retry_s_ = 0;
        log_fail_ = 0;
    }
    update_async_tx();

    if (opt_.alert != audio_on_) {
        audio_on_ = opt_.alert;
        if (audio_on_) {
            audio::set_rate(audio::Rate::Hz_24000);
            audio::output::start();
        } else {
            audio::output::stop();
        }
    }

    esp_.enable(opt_.esp != 0);

    /* RF tracks of bands no longer swept go (and leave the list). */
    for (uint8_t b = 0; b < BAND_COUNT; b++)
        if ((mode() & 1) || !(band_mask() & (1u << b))) rf_.forget_band(b);
    rf_.set_hop24(SIGDET_FULL && (opt_.mode & 4));
    update_columns();
    refresh_status();
}

void SignatureDetectView::bind_option(Checkbox& box, uint8_t byte, uint8_t mask, bool invert) {
    uint8_t* const opts = &opt_.alert;
    box.set_value(((opts[byte] & mask) != 0) != invert);
    box.on_select = [this, byte, mask, invert](Checkbox&, bool v) {
        uint8_t* const o = &opt_.alert;
        o[byte] = (v != invert) ? (o[byte] | mask) : (o[byte] & ~mask);
        apply_options();
    };
}

/* The list's 4th column: distance for Remote ID, the kind tag (TRKR, LITE,
 * ...) for signature hits. Drones: "Dist"; a class with Remote ID and other
 * rules: "D/K"; any other class: "Kind". */
void SignatureDetectView::update_columns() {
    const char* name = !sig_file_ ? "Dist" : (detector_on(RX_RID) ? "D/K" : "Kind");
    auto it = columns_.begin() + 3;
    if (it->first == name) return;
    columns_.set(3, name, 4);
    list_.set_dirty();
}

/* CSV log, format 2 (sdcard/SIGDET/README.TXT): a new file per session,
 * LOGS/SIGDET_YYYYMMDDHHMMSS.CSV, the column names as its first line and a
 * START row with the firmware build. A failure keeps the Log box ticked and
 * shows why on the status line (red); a new file is tried every
 * kLogRetrySeconds. Not while a map is open (its heap). */
void SignatureDetectView::open_log() {
    log_open_ = false;
    if (!log_on() || log_) return;
    void* const mem = chHeapAlloc(nullptr, sizeof(File)); /* not operator new: it halts when the heap is short */
    if (!mem) {
        log_failed("", "no memory");
        return;
    }
    log_ = new (mem) File{};
    if (!log_name_[0]) new_log_name();
    ensure_directory(logs_dir);
    const auto error = log_->append(std::filesystem::path{log_name_, log_name_ + strlen(log_name_)});
    if (error.is_valid()) {
        log_failed("open ", error.value().what().c_str());
        return;
    }
    log_error_[0] = 0;
    log_retry_s_ = 0;
    log_bytes_ = log_->size();
    if (log_bytes_ == 0) log_line(kCsvHeader);
    start_row(log_fail_ ? "REOPEN" : "START");
    hb_s_ = now_s();
    refresh_status();
}

/* LOGS/SIGDET_<date time>[_part].CSV: the session's first file, a
 * rollover, or after kLogNewPartAfter failed retries of the same file. */
void SignatureDetectView::new_log_name() {
    const auto t = rtc_time::now();
    Fmt n{log_name_, sizeof(log_name_)};
    n.s("LOGS/SIGDET_").d(t.year()).d2(t.month()).d2(t.day()).d2(t.hour()).d2(t.minute()).d2(t.second());
    if (log_part_) n.c('_').d(log_part_);
    n.s(".CSV");
}

/* An open / write / sync error: the file is closed (what was not synced yet
 * may be lost too), the status line says why in red, and the same file is
 * reopened after 2, 4, 8, 16, then every kLogRetrySeconds s; after
 * kLogNewPartAfter failed attempts in a row a new part is started instead.
 * Rows meanwhile are counted as lost (HB, STOP). */
void SignatureDetectView::log_failed(const char* what, const char* why) {
    Fmt{log_error_, sizeof(log_error_)}.s(what).s(why);
    close_log();
    log_errs_++;
    if (log_fail_ < 255) log_fail_++;
    const uint32_t wait = log_fail_ >= 5 ? kLogRetrySeconds : (1u << log_fail_);
    log_retry_s_ = now_s() + wait;
    refresh_status();
}

/* Synced (FatFs flushes the sector and the directory entry), freed. */
void SignatureDetectView::close_log() {
    if (!log_) return;
    log_->~File(); /* f_close: syncs */
    chHeapFree(log_);
    log_ = nullptr;
    log_dirty_ = false;
}

/* The log stops (app exit, Log unticked): a SIG_DROP (why=stop) for every
 * signature entry still listed and an SCN_DROP (drop=stop) for every scan
 * finding, so that each NEW in the file has its summary; the survey's last
 * summaries; then the STOP row (rows counts itself). The entries stay
 * listed. */
void SignatureDetectView::stop_row(const char* why) {
    if (!log_) return;
    drop_stop_ = true;
    for (const auto& e : tracker_) {
        if (!e.confirmed) continue;
        if (e.kind == Kind::Sig) report_sig(Tracker::EV_DROPPED, e);
        if (e.kind == Kind::Scan) report_scan(Tracker::EV_DROPPED, e);
    }
    drop_stop_ = false;
    survey_flush();
    char row[128];
    Fmt f{row, sizeof(row)};
    row_begin(f, "STOP", "");
    f.s(",,").s(sig_name_.c_str(), Fmt::Esc::Csv).s(",,,,,,,,rows=").d(log_rows_ + 1).s(";lost=").d(log_lost_).s(";why=").s(why);
    f.s(";evictions=").d(tracker_.evictions()).s(";requeues=").d(tracker_.requeues());
    log_line(row);
}

/* One row: written now (FatFs buffers the partial sector), synced by
 * log_tick() within kLogSyncMs. Without an open file (error, not open yet)
 * the row is counted as lost. */
void SignatureDetectView::log_line(const char* line) {
    if (!log_) {
        if (log_on()) log_lost_++;
        return;
    }
    /* In pieces that never take FatFs's direct sector path (sigdet_log.hpp:
     * log_chunk): the row buffer is not word aligned. */
    const size_t n = strlen(line);
    static constexpr char kEol[] = "\r\n";
    File::Result<File::Size> r{static_cast<File::Size>(0)};
    for (int part = 0; part < 2 && !r.is_error(); part++) {
        const char* p = part ? kEol : line;
        size_t left = part ? 2 : n;
        while (left && !r.is_error()) {
            const size_t k = log_chunk(static_cast<uint32_t>(log_->tell()), left);
            r = log_->write(p, k);
            p += k;
            left -= k;
        }
    }
    if (r.is_error()) {
        log_lost_++;
        log_failed("write ", r.error().what().c_str());
        return;
    }
    log_rows_++;
    log_bytes_ += n + 2;
    log_dirty_ = true;
}

/* Once a second: sync, rollover, retry after an error. */
void SignatureDetectView::log_tick() {
    const uint32_t ms = chTimeNow();
    if (log_ && log_dirty_ && ms - log_sync_ms_ >= kLogSyncMs) {
        log_sync_ms_ = ms;
        log_dirty_ = false;
        const auto error = log_->sync();
        if (error.is_valid())
            log_failed("sync ", error.value().what().c_str());
        else
            log_fail_ = 0; /* rows reached the card: the file is healthy again */
    }
    if (log_ && log_bytes_ >= kLogPartBytes && !SigdetMapView::alive()) {
        close_log();
        if (log_part_ < 99) log_part_++;
        new_log_name();
        open_log();
    }
    if (!log_ && log_on() && log_retry_s_ && now_s() >= log_retry_s_ && !SigdetMapView::alive()) {
        if (log_fail_ >= kLogNewPartAfter && log_part_ < 99) { /* the same file keeps failing: a fresh one */
            log_part_++;
            new_log_name();
        }
        open_log();
    }
}

/* uptime_ms,time,event,src (the columns that follow each start with ','). */
void SignatureDetectView::row_begin(Fmt& f, const char* event, const char* src) {
    f.d(static_cast<int32_t>(chTimeNow() * 1000 / CH_FREQUENCY)).c(',');
    const auto t = rtc_time::now();
    f.d(t.year()).c('-').d2(t.month()).c('-').d2(t.day()).c(' ').d2(t.hour()).c(':').d2(t.minute()).c(':').d2(t.second());
    f.c(',').s(event).c(',').s(src);
}

/* POS_LOAD / POS_SAVE: label = source, kind position, lat/lon; detail source
 * (number), alt_m, age_s (-1 unknown), acc_m, sats, file. */
void SignatureDetectView::log_position(const char* event, const saved_position::Position& p) {
    char row[200];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "");
    f.s(",,").s(saved_position::source_name(p.source)).s(",position,,,,,").e7(p.lat_e7).c(',').e7(p.lon_e7);
    f.s(",source=").d(p.source).s(";alt_m=").d(p.alt_m).s(";age_s=").d(saved_position::age_s(p.time, saved_position::now_fat()));
    f.s(";acc_m=").d(static_cast<int32_t>(p.acc_m)).s(";sats=").d(p.sats).s(";file=SETTINGS/position.ini");
    log_line(row);
}

bool SignatureDetectView::manual_position(saved_position::Position& p) const {
    if (!odid_pos_valid(opt_.man_lat, opt_.man_lon)) return false;
    p = saved_position::Position{};
    p.lat_e7 = opt_.man_lat;
    p.lon_e7 = opt_.man_lon;
    p.time = static_cast<uint32_t>(opt_.man_meta);
    p.source = static_cast<uint8_t>(opt_.man_meta >> 32);
    if (p.source == saved_position::SOURCE_NONE) p.source = saved_position::SOURCE_MAP;
    return true;
}

void SignatureDetectView::start_row(const char* event) {
    char row[200];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "");
    f.s(",,").s(sig_name_.c_str(), Fmt::Esc::Csv).s(",,,,,,,,format=2;fw=");
#ifdef VERSION_STRING
    f.s(VERSION_STRING, Fmt::Esc::Kv);
#else
    f.s("?");
#endif
    f.s(";build=").s(__DATE__ " " __TIME__, Fmt::Esc::Kv).s(";log=").s(survey_on() ? "survey" : "detections");
    f.s(";class=").s(sig_name_.c_str(), Fmt::Esc::Kv).s(";part=").d(log_part_);
    f.s(";lna=").d(receiver_model.lna()).s(";vga=").d(receiver_model.vga()).s(";amp=").d(receiver_model.rf_amp());
    log_line(row);
}

/* USB lines (JSON, raw adverts, sweep dumps) need the asyncmsg switch, which
 * the USB shell's "asyncmsg enable" also sets; on as long as one is wanted. */
void SignatureDetectView::update_async_tx() {
    portapack::async_tx_enabled = async_tx_was_ || opt_.json || raw_usb_ || dump_band_ != kBle || stat_usb_;
}

/* The RF working arrays (~2.9 KB) and the sweep (1.4 KB) live in M4 RAM that
 * PSDT lends (address in its stats messages, the first one 500 ms after the
 * start), not on the M0 heap. Without it no sweep dwell starts (BLE only).
 * off: the sweep is ended (the M4 must not be sweeping any more: its engine
 * writes into the sweep's points); a new one is built for the next dwell. */
bool SignatureDetectView::rf_memory(bool on) {
    constexpr size_t kSweepAt = (sizeof(RfEngine::Work) + 7) & ~size_t{7};
    static_assert(kSweepAt + sizeof(spec_an::Sweep) <= SignatureDetectStatsMessage::kScratchRf, "RF arrays + sweep: raise kScratchRf");
    if (on) {
        if (!work_ && stats_.scratch && stats_.scratch_size >= SignatureDetectStatsMessage::kScratchRf) {
            work_ = reinterpret_cast<RfEngine::Work*>(stats_.scratch);
            rf_.set_work(work_);
        }
        if (work_ && !sweep_) sweep_ = new (stats_.scratch + kSweepAt) spec_an::Sweep{};
        return sweep_ != nullptr;
    }
    if (sweep_) {
        sweep_->~Sweep();
        sweep_ = nullptr;
    }
    return false;
}

/* While a map is open (M0 time for drawing; before the map moved into
 * PSDT's lent RAM also ~10 KB of M0 heap): no sweeps,
 * BLE Remote ID only, which is what the map shows. A sweep or DroneID dwell
 * in progress ends now (BLE dwell: the SignatureDetectConfig stops the M4 sweep
 * engine / the DroneID detector). RF tracks and floors are kept. */
void SignatureDetectView::pause_rf(bool on) {
    if (on == rf_paused_) return;
    rf_paused_ = on;
    if (on) {
        if (sweeping() || phase_band_ == kDid || phase_band_ == kHum) next_phase();
        scan_memory(false, Tracker::DROP_GONE); /* a map takes its RAM back */
        rf_memory(false);
    }
    refresh_status();
}

void SignatureDetectView::configure(uint8_t extra_flags) {
    reset_pending_ = reset_pending_ || (extra_flags & SignatureDetectConfigMessage::FLAG_RESET_STATS);
    if (phase_band_ != kBle) return; /* would stop the sweep; the next BLE dwell sends it */
    /* Other adverts (not Remote ID, not DJI) only for BLE signature rules
     * or the survey log: the Drones class without survey is as it was. */
    const bool fwd = (sig_ready() && sig_mem_->cls.nble) || survey_on();
    const uint8_t flags = (pass_all_ ? SignatureDetectConfigMessage::FLAG_PASS_ALL : 0) |
                          (conjugate_ ? SignatureDetectConfigMessage::FLAG_CONJUGATE : 0) |
                          (fwd ? SignatureDetectConfigMessage::FLAG_FWD_ADV : 0) |
                          (reset_pending_ ? SignatureDetectConfigMessage::FLAG_RESET_STATS : 0) | extra_flags;
    reset_pending_ = false;
    baseband::set_signature_detect(sampling_rate, SignatureDetectConfigMessage::MODE_GFSK_BLE, channel_, flags);
}

/* opt_.mode as the class has it: the Drones class takes Setup's boxes; a
 * file class runs what its rules need (sigdet_sig.hpp: BLE rules -> BLE dwells,
 * PAT -> pattern, BAND / NB -> sweeps, RX wifi / droneid), the 2.4 GHz hopper
 * box still from Setup. A file class not loaded yet runs BLE only. */
uint8_t SignatureDetectView::mode() const {
    if (!sig_file_) return opt_.mode;
    const SigClass* c = sig_ready() ? &sig_mem_->cls : nullptr;
    const uint8_t rx = c ? c->rx : 0;
    uint8_t m = opt_.mode & 4;
    if (!c || (!c->band_bits && !c->nscan)) m |= 1;
    if (!(rx & RX_BLE)) m |= 2;
    if (!(rx & RX_PAT)) m |= 8;
    if (!(rx & RX_WIFI)) m |= 16;
    if (!(rx & RX_DID)) m |= 32;
    return m;
}

/* RfBand bits swept: Setup's band boxes (Drones), the class's bands (file). */
uint16_t SignatureDetectView::band_mask() const {
    if (sig_file_) return sig_ready() ? sig_mem_->cls.band_bits : 0;
    uint16_t m = 0;
    const uint8_t bits = swept_bands();
    for (uint8_t c = 0; c < kSetupBands; c++)
        if (bits & (1u << c)) m |= static_cast<uint16_t>(1u << band_id(c));
    return m;
}

bool SignatureDetectView::sweeps_on() const {
    return !rf_paused_ && !(mode() & 1) && (band_mask() || scan_on());
}

/* Pattern dwells: on unless unticked (they also run while a map is open:
 * no M0 heap, 4 Msps like BLE). */
bool SignatureDetectView::patterns_on() const {
    return !(mode() & 8) && pat_table_.size() && !pat_load_pending_;
}

/* Wi-Fi Remote ID dwells: on unless unticked (4 Msps like BLE, no M0 heap). */
bool SignatureDetectView::wifi_on() const {
    return !(mode() & 16) && (wifi_mask_ & ((1u << (sizeof(tune::kWifiChannels) / sizeof(tune::kWifiChannels[0]))) - 1));
}

/* A Wi-Fi dwell follows this BLE dwell (start_ble() counted it): every
 * kWifiEveryBle-th, every one while on-board Wi-Fi hears Remote ID. */
bool SignatureDetectView::wifi_due() const {
    if (!wifi_on()) return false;
    const bool hot = wifi_rid_s_ && now_s() - wifi_rid_s_ < tune::kRidBoostSeconds;
    return ble_since_wifi_ >= (hot ? tune::kWifiEveryBleRid : tune::kWifiEveryBle);
}

/* Setup band bit -> band: 2.4, 5.8, 5.3, 1.2, then 915 or 868 by region,
 * then 5.1. */
uint8_t SignatureDetectView::band_id(uint8_t setup_bit) const {
    if (setup_bit == 5) return BAND_51;
    return setup_bit < BAND_915 ? setup_bit : static_cast<uint8_t>(opt_.region ? BAND_868 : BAND_915);
}

uint8_t SignatureDetectView::setup_bit(uint8_t band) {
    return band < BAND_915 ? band : (band == BAND_51 ? 5 : 4);
}

uint8_t SignatureDetectView::swept_bands() const {
    return (opt_.bands ^ kBandInverted) & kBandMask;
}

/* The enabled band after the cursor (RfBand order: 2.4, 5.8, 5.3, 1.2,
 * 915 / 868, 5.1, then a class's custom bands); BF_ALT bands only every
 * second cycle (band_due: 5.3 on the even ones, 5.1 on the odd ones). Moving
 * past the last band ends a cycle. */
uint8_t SignatureDetectView::next_band(bool advance) {
    uint8_t c = band_cursor_;
    uint32_t cycle = cycle_;
    const uint16_t mask = band_mask();
    for (size_t n = 0; n < 2 * BAND_COUNT; n++) {
        if (++c >= BAND_COUNT) {
            c = 0;
            cycle++;
        }
        if (!(mask & (1u << c)) || !kBands[c].points || !band_due(kBands[c].flags, cycle)) continue;
        if (advance) {
            if (cycle != cycle_) {
                const systime_t now = chTimeNow();
                cycle_ms_ = now - cycle_t0_;
                cycle_t0_ = now;
            }
            band_cursor_ = c;
            cycle_ = cycle;
        }
        return c;
    }
    return BAND_24;
}

/* The cycle: BLE -> (Wi-Fi, when due) -> pattern -> sweep -> BLE ...,
 * skipping what is off (RF only: Wi-Fi -> pattern -> sweep -> Wi-Fi ..., or
 * fewer). A 4 Msps dwell retunes first, then tells the baseband: the few
 * samples of the new channel still in the old dwell's filters only fail
 * their checks. A DJI DroneID dwell (15.36 Msps) cuts in at any boundary
 * when due; after it the cycle goes on from the phase before it. */
void SignatureDetectView::next_phase() {
    enum Next : uint8_t {
        BLE,
        PAT,
        SWEEP,
        WIFI,
        DID,
        ZW,
        HUM
    };
    const uint8_t prev = phase_band_;
    const bool from_sweep = sweeping();
    end_phase();
    /* The new dwell starts now, after the old one's end processing: a sweep
     * dump (end_sweep(), debug "Dump" all) writes ~34 USB lines at ~10 ms
     * each, longer than a 300 ms dwell. Stamped before it (as it was), RF
     * only (sweeps back to back) found every dwell already expired at its
     * first frame: 0..1 slice a dwell, the stall counter rising. */
    phase_t0_ = chTimeNow();
    rf_full_ = false;

    const bool ble = !(mode() & 2);
    const bool pat = patterns_on();
    const bool sweeps = sweeps_on();
    const bool wifi = wifi_on();
    /* After a DroneID dwell: as after the phase before it. The rate is
     * switched back in any case (as after a sweep). */
    const uint8_t from = prev == kDid ? did_resume_ : prev;
    const bool rate_switch = from_sweep || prev == kDid;
    /* Z-Wave dwells (a class with a ZW rule): after Wi-Fi, before the
     * pattern dwell; with nothing else on, back to back. */
    const bool zw = zw_on();
    Next next;
    if (droneid_due()) {
        next = DID;
    } else if (from == kBle) {
        next = wifi_due() ? WIFI : (zw ? ZW : (pat ? PAT : (sweeps ? SWEEP : BLE)));
    } else if (from == kWifi) {
        next = zw ? ZW : (pat ? PAT : (sweeps ? SWEEP : (ble || !wifi ? BLE : WIFI)));
    } else if (from == kZw) {
        next = pat ? PAT : (sweeps ? SWEEP : (ble ? BLE : (wifi ? WIFI : ZW)));
    } else if (from == kPat) {
        next = sweeps ? SWEEP : (ble ? BLE : (wifi ? WIFI : (zw ? ZW : PAT)));
    } else {
        next = ble ? BLE : (wifi ? WIFI : (zw ? ZW : (pat ? PAT : SWEEP)));
    }
    if (next == BLE && !ble) next = wifi ? WIFI : (zw ? ZW : (pat ? PAT : BLE)); /* nothing else left on: BLE after all */
    if (next == SWEEP && !rf_memory(true)) next = ble ? BLE : (wifi ? WIFI : (zw ? ZW : (pat ? PAT : BLE)));
    /* The scan's hum check takes the sweep's turn when one is due (bounded:
     * hum_due()). After it the cycle goes on as after a sweep. */
    if (next == SWEEP && scan_on() && hum_due()) next = HUM;

    if (next == DID) {
        did_resume_ = from;
        /* From a 4 Msps dwell: stop demodulating before the rate goes up
         * (as for a sweep); the SignatureDetectConfig after the retune starts the
         * detector. From a sweep: its SignatureDetectConfig stops the engine. */
        if (!from_sweep) baseband::set_signature_detect(tune::kDidSampleRate, SignatureDetectConfigMessage::MODE_IDLE, channel_, 0);
        start_droneid();
    } else if (next == SWEEP) {
        /* From a 4 Msps dwell: stop demodulating before the sweep raises the
         * sample rate (20 Msps buffers take the 4 Msps demodulator longer
         * than they last, each one a lost buffer, until the SpecAnConfig). */
        if (!from_sweep && prev != kDid) baseband::set_signature_detect(spec_an_sweep_fs, SignatureDetectConfigMessage::MODE_IDLE, channel_, 0);
        if (scan_on())
            start_scan();
        else
            start_sweep(next_band(true));
    } else if (next == HUM) {
        start_hum(rate_switch);
    } else if (next == PAT) {
        start_pattern(rate_switch);
    } else if (next == WIFI) {
        start_wifi(rate_switch);
    } else if (next == ZW) {
        start_zw(rate_switch, prev == kZw);
    } else {
        start_ble(rate_switch);
    }
    refresh_status();
}

/* The dwell in progress ends: its airtime, its end processing (sweep
 * dwell: tracks; pattern dwell: the scan's verdict; DroneID: the detector
 * stops before any rate switch). The track view's pause has nothing to end. */
void SignatureDetectView::end_phase() {
    const uint8_t prev = phase_band_;
    const systime_t now = chTimeNow();
    const uint16_t dt = now - phase_t0_;
    if (prev == kScan) {
        end_scan();
    } else if (prev == kHum) {
        hum_ms_ += dt;
        hum_last_end_ = now;
        end_hum();
    } else if (sweeping()) {
        end_sweep();
    } else if (prev == kBle) {
        ble_ms_ += dt;
    } else if (prev == kWifi) {
        wifi_ms_ += dt;
    } else if (prev == kDid) {
        did_ms_ += dt;
        did_last_end_ = now;
        /* Stop the detector before the rate switch: 4 or 20 Msps buffers are
         * no DroneID stream (a stage B in progress finishes first, M4 side). */
        baseband::set_signature_detect(sampling_rate, SignatureDetectConfigMessage::MODE_IDLE, channel_, 0);
    } else if (prev == kPat) {
        pat_ms_ += dt;
        end_pattern(dt);
    } else if (prev == kZw) {
        zw_ms_ += dt;
    }
}

/* DroneID dwells: on unless unticked; not while a map is open (the map
 * shows Remote ID, which the BLE dwells bring). */
bool SignatureDetectView::droneid_on() const {
    return !(mode() & 32) && !rf_paused_;
}

bool SignatureDetectView::droneid_due() const {
    return droneid_on() && phase_band_ != kDid && chTimeNow() - did_last_end_ >= tune::kDidGapMs;
}

/* One 2.4 GHz DroneID centre (in turn), 15.36 Msps, the LO on the centre:
 * ReceiverModel puts the LO fs/4 below the target, so the target is centre +
 * fs/4 (the burst's unused DC carrier then sits where the Pro's DC residue
 * is, at the band edge after the FPGA's quarter shift). */
void SignatureDetectView::start_droneid() {
    constexpr size_t kN = sizeof(tune::kDidChannelsKhz) / sizeof(tune::kDidChannelsKhz[0]);
    did_cursor_ = static_cast<uint8_t>((did_cursor_ + 1) % kN);
    phase_band_ = kDid;
    phase_dwell_ms_ = tune::kDidDwellMs;
    receiver_model.set_sampling_rate(tune::kDidSampleRate);
    if (sweep_) sweep_->invalidate_rates();
    receiver_model.set_target_frequency(static_cast<rf::Frequency>(tune::kDidChannelsKhz[did_cursor_]) * 1000 + tune::kDidSampleRate / 4);
    const uint8_t flags = reset_pending_ ? SignatureDetectConfigMessage::FLAG_RESET_STATS : 0;
    reset_pending_ = false;
    baseband::set_signature_detect(tune::kDidSampleRate, SignatureDetectConfigMessage::MODE_DRONEID, did_cursor_, flags);
}

/* A DroneID burst from the M4 (presence only): the one DroneID entity
 * (Kind::DroneId, source O), labelled "DroneID?": the detector was checked on
 * recordings only. */
void SignatureDetectView::on_droneid(const SignatureDetectDroneIdMessage& message) {
    did_rx_++;
    constexpr size_t kN = sizeof(tune::kDidChannelsKhz) / sizeof(tune::kDidChannelsKhz[0]);
    const uint32_t khz = tune::kDidChannelsKhz[message.tag < kN ? message.tag : 0];
    const int32_t db = (message.level_cdb - 50) / 100;
    const DroneIdBurst b{static_cast<int8_t>(db < -127 ? -127 : (db > 0 ? 0 : db)), khz, message.zc_milli, message.snr_cdb, message.cfo_hz, message.nsym};
    if (tracker_.ingest_droneid(b, now_s())) list_.set_dirty();
#if SIGDET_FULL
    if (debug_) {
        /* "DID 2444.5 -41 z523 9": centre, dBFS, ZC x 1000, symbols */
        char line[40];
        Fmt f{line, sizeof(line)};
        f.s("DID ").d(khz / 1000).c('.').d(khz % 1000 / 100).c(' ').d(db).s(" z").d(message.zc_milli).c(' ').d(message.nsym);
        debug_->write(line);
    }
#endif
    if (raw_usb_) {
        /* "DDDID,t_ms,centre_khz,level_cdb,snr_cdb,cfo_hz,zc_milli,cp_milli,dur_us,nsym" */
        using I = int32_t;
        const I v[] = {I(message.t_ms), I(khz), message.level_cdb, message.snr_cdb, message.cfo_hz, message.zc_milli,
                       message.cp_milli, message.dur_us, message.nsym};
        char usb[96];
        Fmt f{usb, sizeof(usb)};
        f.s("DDDID");
        for (const I x : v) f.c(',').d(x);
        usb_line(usb);
    }
}

void SignatureDetectView::start_ble(bool from_sweep) {
    static constexpr uint8_t kChannels[3] = {37, 39, 38};
    ble_idx_ = (ble_idx_ + 1) % 3;
    channel_ = kChannels[ble_idx_];
    phase_band_ = kBle;
    phase_dwell_ms_ = tune::kBleOnlyMs;
    if (ble_since_wifi_ < 0xFF) ble_since_wifi_++;
    if (sweeps_on()) {
        /* At least ratio x what follows until the next BLE dwell. */
        const uint8_t ratio = tracker_.rid_within(now_s(), tune::kRidBoostSeconds) ? tune::kBleRatioRid : tune::kBleRatio;
        const uint16_t pat = patterns_on() ? pat_dwell_last_ : 0;
        const uint16_t wifi = (wifi_due() ? tune::kWifiDwellMs : 0) + (zw_on() ? kZwDwellMs : 0);
        const uint16_t ms = ratio * (kBands[next_band(false)].dwell_cs * 10u + pat + wifi);
        phase_dwell_ms_ = ms > tune::kBleMinMs ? ms : tune::kBleMinMs;
    }
    /* The rate switch, again on every dwell while the last one could not
     * be programmed (I2C error): not silently on a wrong sample clock. */
    if (from_sweep || !portapack::clock_manager.sampling_frequency_ok()) {
        receiver_model.set_sampling_rate(sampling_rate);
        if (sweep_) sweep_->invalidate_rates();
    }
    receiver_model.set_target_frequency(ble_channel_frequency(channel_));
    configure();
}

/* One Wi-Fi channel of the table (the next enabled one), the half of it not
 * taken last time: retune so the slice is centred at channel + slice (target
 * = that + fs/4, sigdet_dsss.hpp), then tell the baseband. */
void SignatureDetectView::start_wifi(bool from_sweep) {
    constexpr size_t kN = sizeof(tune::kWifiChannels) / sizeof(tune::kWifiChannels[0]);
    size_t c = wifi_cursor_;
    for (size_t i = 0; i < kN; i++) {
        c = (c + 1) % kN;
        if (wifi_mask_ & (1u << c)) break;
    }
    wifi_cursor_ = static_cast<uint8_t>(c);
    const bool upper = (wifi_upper_ >> c) & 1;
    wifi_upper_ ^= static_cast<uint8_t>(1u << c);
    wifi_slice_ = upper ? tune::kWifiSliceMhz : -tune::kWifiSliceMhz;
    ble_since_wifi_ = 0;
    phase_band_ = kWifi;
    phase_dwell_ms_ = tune::kWifiDwellMs;
    if (from_sweep || !portapack::clock_manager.sampling_frequency_ok()) { /* see start_ble() */
        receiver_model.set_sampling_rate(sampling_rate);
        if (sweep_) sweep_->invalidate_rates();
    }
    const int32_t target_mhz = tune::kWifiChannels[c].mhz + wifi_slice_ + static_cast<int32_t>(sampling_rate / 4 / 1000000);
    receiver_model.set_target_frequency(static_cast<rf::Frequency>(target_mhz) * 1000000);
    const uint8_t flags = (conjugate_ ? SignatureDetectConfigMessage::FLAG_CONJUGATE : 0) |
                          (reset_pending_ ? SignatureDetectConfigMessage::FLAG_RESET_STATS : 0) |
                          (wifi_legacy_ ? SignatureDetectConfigMessage::FLAG_WIFI_LEGACY : 0) |
                          static_cast<uint8_t>((wifi_search_ << SignatureDetectConfigMessage::SEARCH_SHIFT) & SignatureDetectConfigMessage::SEARCH_MASK);
    reset_pending_ = false;
    baseband::set_signature_detect_dsss(tune::kWifiChannels[c].number, wifi_slice_, flags);
}

/* One channel of one pattern table entry (PatternScan's turn). */
void SignatureDetectView::start_pattern(bool from_sweep) {
    size_t e = 0, c = 0;
    if (!pat_scan_.next(e, c)) {
        start_ble(from_sweep);
        return;
    }
    const PatternEntry& pe = pat_table_.entry(e);
    pat_entry_ = static_cast<uint8_t>(e);
    pat_ch_ = static_cast<uint8_t>(c);
    pat_dwell_last_ = pe.dwell_ms;
    phase_band_ = kPat;
    phase_dwell_ms_ = pe.dwell_ms;
    if (from_sweep || !portapack::clock_manager.sampling_frequency_ok()) { /* see start_ble() */
        receiver_model.set_sampling_rate(sampling_rate);
        if (sweep_) sweep_->invalidate_rates();
    }
    receiver_model.set_target_frequency(static_cast<rf::Frequency>(pe.ch_khz[c]) * 1000);
    const uint8_t flags = (conjugate_ ? SignatureDetectConfigMessage::FLAG_CONJUGATE : 0) |
                          (reset_pending_ ? SignatureDetectConfigMessage::FLAG_RESET_STATS : 0);
    reset_pending_ = false;
    const uint8_t pat_flags = (pe.invert ? SignatureDetectConfigMessage::PAT_INVERT : 0) |
                              static_cast<uint8_t>(pe.rate << SignatureDetectConfigMessage::PAT_RATE_SHIFT);
    baseband::set_signature_detect_pattern(sampling_rate, pattern_tag(e, c), flags, pe.sync, pe.sync_bits, pe.max_errors, pe.payload_bits, pat_flags);
}

/* The pattern dwell is over: at the end of an entry's pass the scan says
 * whether it is (still) there; its entity is created, refreshed or dropped. */
void SignatureDetectView::end_pattern(uint16_t ms) {
    const size_t e = pat_entry_;
    const auto ev = pat_scan_.end_dwell(e, pat_ch_, ms);
    if (ev == PatternScan::EV_NONE) return;
    const uint8_t mac[6] = {0, 0, 'P', 'T', static_cast<uint8_t>(e), 0};
    if (ev == PatternScan::EV_DROP) {
        tracker_.drop_pattern(mac);
        list_.set_dirty();
        return;
    }
    const PatternState& st = pat_scan_.state(e);
    const int32_t db = (st.rssi_max_cdb - 50) / 100;
    Entity* x = tracker_.ingest_pattern(mac, static_cast<int8_t>(db < -127 ? -127 : db), now_s());
    if (!x) return;
    const PatternEntry& pe = pat_table_.entry(e);
    x->pat = static_cast<uint8_t>(e);
    x->freq_khz = st.best_khz;
    x->rate10 = pat_scan_.rate10(e);
    memcpy(x->label, pe.name, sizeof(pe.name) < sizeof(x->label) ? sizeof(pe.name) : sizeof(x->label));
    memcpy(x->cls, pe.cls, sizeof(x->cls));
    tracker_.announce(*x, now_s()); /* EV_CONFIRMED the first time: log, beep */
    list_.set_dirty();
}

void SignatureDetectView::on_pattern_hit(const SignatureDetectPatternHitMessage& message) {
    pat_hits_rx_++;
    pat_scan_.hit(message.tag, message.rssi_cdb);
    if (raw_usb_) {
        /* "DDPAT,t_ms,tag,rssi_cdb,errors,phase,payload_bits,PAYLOADHEX" */
        char line[80];
        Fmt f{line, sizeof(line)};
        const uint8_t pl[8] = {static_cast<uint8_t>(message.payload_hi >> 24), static_cast<uint8_t>(message.payload_hi >> 16),
                               static_cast<uint8_t>(message.payload_hi >> 8), static_cast<uint8_t>(message.payload_hi),
                               static_cast<uint8_t>(message.payload_lo >> 24), static_cast<uint8_t>(message.payload_lo >> 16),
                               static_cast<uint8_t>(message.payload_lo >> 8), static_cast<uint8_t>(message.payload_lo)};
        f.s("DDPAT,").d(message.t_ms).c(',').d(message.tag).c(',').d(message.rssi_cdb).c(',').d(message.errors).c(',');
        f.d(message.phase).c(',').d(message.payload_bits).c(',').hex(pl, 8);
        usb_line(line);
    }
}

/* /SIGDET/PATTERNS.TXT (format: sigdet_pattern.hpp), read once, bounded. */
void SignatureDetectView::load_patterns() {
    pat_load_pending_ = false;
    File f;
    char path[32];
    Fmt{path, sizeof(path)}.s(sd_root()).s("/PATTERNS.TXT");
    if (f.open(std::filesystem::path{path, path + strlen(path)}).is_valid()) {
        pat_scan_.reset();
        return; /* no file: the built-in entry only */
    }
    pat_table_.begin_file();
    char buf[64];
    for (size_t total = 0; total < 16384;) {
        const auto r = f.read(buf, sizeof(buf));
        if (!r || r.value() == 0) break;
        pat_table_.feed(buf, r.value());
        total += r.value();
    }
    pat_table_.finish();
    pat_scan_.reset();
}

void SignatureDetectView::start_sweep(uint8_t band) {
    const BandDef& d = kBands[band];
    phase_band_ = band;
    phase_dwell_ms_ = d.dwell_cs * 10u;
    slices_ = 0;
    dwell_sweeps_ = 0;
    dwell_seq_++;
    rf_.begin(band);
    nb_dwell_ = sig_ready() && sig_mem_->cls.nnb;
    if (nb_dwell_) nb_begin(sig_mem_->cls, sig_mem_->nb, band, rf_);
    if (stall_run_ >= tune::kStallReset) {
        stall_run_ = 0;
        sweep_->invalidate_rates();
    }
    spec_an::Sweep::Params p{};
    p.start = d.lo_mhz * 1000000ULL;
    p.span = (d.hi_mhz - d.lo_mhz) * 1000000ULL;
    p.points = d.points;
    p.avg = avg_ ? avg_ : ((d.flags & BF_PEAK) ? tune::kSweepAvgPeak : tune::kSweepAvgMean);
    p.settle_us = settle_us_;
    p.detector = (d.flags & BF_PEAK) ? spec_an::Detector::Peak : spec_an::Detector::Average;
    /* A band within one tune (868) repeats its capture every DMA buffer:
     * unpaced, its replies would flood the queue. */
    p.frame_paced = pace_ || (d.hi_mhz - d.lo_mhz) < kOneTuneMhz;
    /* The last rate switch failed (I2C): the sweep must set its rate again. */
    if (!portapack::clock_manager.sampling_frequency_ok()) sweep_->invalidate_rates();
    sweep_->start(p);
    /* The tune slices of this dwell: the 1.2 GHz video verdicts are checked
     * against them (a per slice front end artefact), RF rows say where in
     * its slice a track sits. */
    const auto& pl = sweep_->plan();
    rf_.set_slices(pl.f_bin0, static_cast<uint32_t>((static_cast<uint64_t>(pl.bins_per_slice) * pl.fs) >> pl.log2n));
}

void SignatureDetectView::end_sweep() {
    sweep_->stop();
    const uint8_t b = phase_band_;
    const uint16_t ms = chTimeNow() - phase_t0_;
    sweep_ms_ += ms;
    bstat_[b] = {rf_.looks(), slices_, ms};
    if (slices_) {
        stall_run_ = 0;
    } else {
        stalls_++;
        stall_run_++;
    }
    /* A band switched off during its dwell leaves no new tracks. */
    if (sweeps_on() && (band_mask() & (1u << b))) {
        rf_.finish();
        sig_dwell_end(ms);
    }

    /* Bench dump of the dwell: USB (asyncmsg) and optionally the SD card.
     * One band at most once a second; all bands every dwell (captures). */
    const systime_t now = chTimeNow();
    if (dump_band_ != kDumpAll && (b != dump_band_ || now - dump_t_ < tune::kDumpMinGapMs)) return;
    dump_t_ = now;
#if SIGDET_FULL
    if (dump_sd_ && !dump_log_) {
        ensure_directory(u"/SIGDET");
        char name[24];
        Fmt{name, sizeof(name)}.s("/SIGDET/DUMP_").d(now_s() % 10000).s(".TXT");
        dump_log_ = std::make_unique<LogFile>();
        if (dump_log_->append(std::filesystem::path{name}).is_valid()) dump_log_.reset();
    }
#endif
    rf_.dump(dump_line, this, dwell_seq_, now, ms, slices_);
}

/* ---------------------------------------------------------------- interference scan */

/* A class with SCAN lines sweeps its scan ranges, window by window, instead
 * of bands (sigdet_scan.hpp). */
bool SignatureDetectView::scan_on() const {
    return !rf_paused_ && sig_ready() && sig_mem_->cls.nscan;
}

/* The engine goes into the map's part of PSDT's lent RAM (free while no map
 * is open; a map or the track view pauses the RF and takes it back), its
 * per-window work into the RF engine's arrays (rf_memory first). */
bool SignatureDetectView::scan_memory(bool on, uint8_t why) {
    constexpr size_t kScanAt = SignatureDetectStatsMessage::kScratchRf;
    static_assert(sizeof(ScanEngine) <= SignatureDetectStatsMessage::kScratchMap, "ScanEngine grew past the map's part of PSDT's lent RAM");
    static_assert(sizeof(ScanEngine::Work) <= sizeof(RfEngine::Work), "ScanEngine::Work must fit in the RF engine's arrays");
    if (!on) {
        if (scan_) {
            /* SCN_DROP rows while the engine can still describe each finding
             * (report_scan), then the tables go. */
            tracker_.drop_kind(Kind::Scan, static_cast<Tracker::DropWhy>(why));
            scan_->~ScanEngine();
            scan_ = nullptr;
            list_.set_dirty();
        }
        return false;
    }
    if (scan_) return true;
    if (SigdetMapView::alive() || !work_ || !sig_ready() || !sig_mem_->cls.nscan) return false;
    if (!stats_.scratch || stats_.scratch_size < kScanAt + SignatureDetectStatsMessage::kScratchMap) return false;
    scan_ = new (stats_.scratch + kScanAt) ScanEngine{};
    scan_->set_work(reinterpret_cast<ScanEngine::Work*>(work_));
    scan_->set_notify(on_scan_event, this);
    if (!scan_->configure(sig_mem_->cls.scan, sig_mem_->cls.nscan)) {
        scan_->~ScanEngine();
        scan_ = nullptr;
        return false;
    }
    scan_w_ = 0;
    scan_t0_ = now_s();
    pass_t0_ = 0; /* the first pass of a new engine is timed from its window 0 */
    scan_gen_++;
    hum_scan_pass_ = 0;
    scan_gain_[0] = static_cast<uint8_t>(receiver_model.lna());
    scan_gain_[1] = static_cast<uint8_t>(receiver_model.vga());
    scan_gain_[2] = receiver_model.rf_amp() ? 1 : 0;
    return true;
}

/* One window of the scan: a sweep of `points` points over its span, `looks`
 * complete looks, then the engine's analysis (end_scan). The window after a
 * window keeps the sample rate: no look is thrown away for settling. */
void SignatureDetectView::start_scan() {
    const bool from_scan = phase_band_ == kScan;
    phase_band_ = kScan;
    phase_dwell_ms_ = 60;
    slices_ = 0;
    dwell_sweeps_ = 0;
    dwell_seq_++;
    if (!scan_memory(true) || !sweep_) return; /* the dwell times out: next phase */
    if (scan_w_ >= scan_->windows()) scan_w_ = 0;
    if (scan_w_ == 0) { /* a pass starts: the one before took this long (the cycle of a scan class) */
        const systime_t now = chTimeNow();
        if (pass_t0_) {
            pass_ms_ = now - pass_t0_;
            cycle_ms_ = static_cast<uint16_t>(pass_ms_ > 65535 ? 65535 : pass_ms_);
        }
        pass_t0_ = now ? now : 1;
    }
    ScanWindowPlan w;
    scan_->plan(scan_w_, w);
    scan_->begin(scan_w_);
    /* Same range as the window before: same sample rate, nothing to settle. */
    if (from_scan && w.range == scan_range_) dwell_sweeps_ = tune::kDiscardLooks;
    scan_range_ = w.range;
    /* Timeout only (the looks end it): 4 looks at one a frame for a window
     * within one tune, ~4 ms a look otherwise. */
    phase_dwell_ms_ = static_cast<uint16_t>(80 + w.looks * 40);
    if (stall_run_ >= tune::kStallReset) {
        stall_run_ = 0;
        sweep_->invalidate_rates();
    }
    spec_an::Sweep::Params p{};
    p.start = w.start_hz;
    p.span = w.span_hz;
    p.points = w.points;
    p.avg = avg_ ? avg_ : tune::kSweepAvgPeak;
    p.settle_us = settle_us_;
    p.detector = spec_an::Detector::Peak;
    p.rbw_idx = w.rbw_idx;
    p.frame_paced = pace_ || w.span_hz < kOneTuneMhz * 1000000ULL;
    /* The engine maps points to frequencies as planned: never move the start
     * (a range must begin high enough for its first tune, README SCAN). */
    p.raise_start = false;
    if (!portapack::clock_manager.sampling_frequency_ok()) sweep_->invalidate_rates();
    sweep_->start(p);
}

void SignatureDetectView::end_scan() {
    if (sweep_) sweep_->stop();
    const uint16_t ms = chTimeNow() - phase_t0_;
    sweep_ms_ += ms;
    if (slices_) {
        stall_run_ = 0;
    } else {
        stalls_++;
        stall_run_++;
    }
    if (!scan_) return;
    /* The floor baseline belongs to the gains it was learnt with. */
    const uint8_t g[3] = {static_cast<uint8_t>(receiver_model.lna()), static_cast<uint8_t>(receiver_model.vga()), static_cast<uint8_t>(receiver_model.rf_amp() ? 1 : 0)};
    if (memcmp(g, scan_gain_, sizeof(g))) {
        memcpy(scan_gain_, g, sizeof(g));
        scan_->reset_baseline();
    }
    scan_->finish(now_s() - scan_t0_);
    scan_windows_++;
    if (scan_w_ + 1u >= scan_->windows()) scan_passes_++;
    if (dump_band_ == kDumpAll) scan_->dump(dump_line, this);
    scan_w_ = static_cast<uint16_t>(scan_w_ + 1u >= scan_->windows() ? 0 : scan_w_ + 1u);
}

/* A hum dwell is due: the scan has a candidate (the strongest steady or
 * drifting emitter not checked within kHumRecheckPasses), a pass has been
 * completed since the last hum dwell, and kHumGapMs have passed since it
 * ended. */
bool SignatureDetectView::hum_due() {
    if (!scan_ || scan_->pass() == hum_scan_pass_) return false;
    if (chTimeNow() - hum_last_end_ < tune::kHumGapMs) return false;
    uint8_t uid = 0;
    uint32_t khz = 0, width_hz = 0;
    if (!scan_->hum_candidate(uid, khz, width_hz)) return false;
    hum_uid_ = uid;
    hum_khz_ = khz;
    return true;
}

/* The meter at the candidate's centre (target = centre: ReceiverModel puts
 * it at +fs/4), boxcar 2^kHumLog2d (14 kHz), FLAG_HUM: the M4 runs the
 * ladder in its idle Wi-Fi slots and sends one SignatureDetectHumMessage per
 * ~1 s window. */
void SignatureDetectView::start_hum(bool rate_switch) {
    phase_band_ = kHum;
    phase_dwell_ms_ = tune::kHumDwellMs;
    hum_win_ = 0;
    hum_lv_ = sigdet::HumLevels{};
    hum_scan_pass_ = scan_ ? scan_->pass() : 0;
    hum_gen_ = scan_gen_;
    hum_dwells_++;
    if (rate_switch || !portapack::clock_manager.sampling_frequency_ok()) { /* see start_ble() */
        receiver_model.set_sampling_rate(sampling_rate);
        if (sweep_) sweep_->invalidate_rates();
    }
    receiver_model.set_target_frequency(static_cast<rf::Frequency>(hum_khz_) * 1000);
    baseband::set_signature_detect(sampling_rate, SignatureDetectConfigMessage::MODE_METER, sigdet::kHumLog2d, SignatureDetectConfigMessage::FLAG_HUM);
}

/* One ladder window. The second one ends the dwell (the next frame moves
 * on); a late one (the dwell already over) is ignored. */
void SignatureDetectView::on_hum(const SignatureDetectHumMessage& message) {
    hum_rx_++;
    if (raw_usb_) {
        /* "DDHUM,t_ms,window,blocks,level_cdb,clip,wait_buf,h60 x12,h50 x12" */
        char line[200];
        Fmt f{line, sizeof(line)};
        f.s("DDHUM,").d(message.t_ms).c(',').d(message.window).c(',').d(message.levels.blocks).c(',').d(message.level_cdb);
        f.c(',').d(message.clip).c(',').d(message.wait_buffers);
        for (size_t k = 0; k < sigdet::kHumHarm; k++) f.c(',').d(message.levels.h60[k]);
        for (size_t k = 0; k < sigdet::kHumHarm; k++) f.c(',').d(message.levels.h50[k]);
        usb_line(line);
    }
    if (phase_band_ != kHum) return;
    hum_lv_ = message.levels;
    hum_win_ = message.window;
    hum_level_cdb_ = message.level_cdb;
    hum_clip_ = message.clip;
    hum_wait_ = message.wait_buffers;
    if (sigdet::hum_clip_warn(message.clip, message.levels.blocks)) clip_s_ = now_s();
    if (hum_win_ >= sigdet::kHumWindows) rf_full_ = true;
    refresh_status();
}

/* The hum dwell is over: the second window's verdict goes to the engine
 * (Hum50 / Hum60: an SE_NEW event, the list entry and its SCN_NEW row) and
 * into an SCN_HUM row. Without that window (a lost message, the dwell cut
 * short) the candidate comes up again; twice in a row without one, it is
 * marked checked (no finding) so that it cannot hold the others up. */
void SignatureDetectView::end_hum() {
    if (!scan_ || hum_gen_ != scan_gen_) return; /* the engine (and its uids) went meanwhile */
    if (hum_win_ < sigdet::kHumWindows) {
        hum_lost_++;
        if (hum_lost_uid_ == hum_uid_) {
            hum_lost_uid_ = 0;
            scan_->hum_result(hum_uid_, sigdet::HumVerdict{});
            hum_row(sigdet::HumVerdict{}, true);
        } else {
            hum_lost_uid_ = hum_uid_;
        }
        return;
    }
    hum_lost_uid_ = 0;
    const sigdet::HumVerdict v = sigdet::hum_verdict(hum_lv_);
    hum_checks_++;
    /* A saturated ADC flattens the envelope: no verdict ("clipped"), the
     * candidate is marked checked like a NoHum (it comes up again after
     * kHumRecheckPasses) but the row says why. Display / log only: the gains
     * stay as the user set them. */
    const bool clipped = sigdet::hum_clip_heavy(hum_clip_, hum_lv_.blocks);
    if (clipped) {
        hum_clipped_++;
        scan_->hum_result(hum_uid_, sigdet::HumVerdict{});
        hum_row(sigdet::HumVerdict{}, false, true);
        return;
    }
    if (v.family) hum_found_++;
    scan_->hum_result(hum_uid_, v);
    hum_row(v, false);
}

/* SCN_HUM: every hum check with Log ticked when it found mains modulation,
 * with Survey ticked (rate limited like SCN_SRV) when it did not. id SC<uid>,
 * label, kind noise, centre, channel width, mean channel power (dBFS);
 * detail: class, type, conf, why, mains_hz, harmonics (of that family over
 * the bar), other (of the other family), best_db, h60 / h50 (the twelve
 * levels, dB over the references, '/' separated), blocks, clip, wait_buf. */
void SignatureDetectView::hum_row(const sigdet::HumVerdict& v, bool lost, bool clipped) {
    if (!log_) return;
    if (!v.family && !clipped && !(survey_on() && sig_mem_ && sig_mem_->budget.take(uptime_ms()))) return;
    char label[kSigLabelLen];
    {
        Fmt l{label, sizeof(label)};
        if (v.family)
            l.s("Hum").d(v.family).c(' ');
        else
            l.s(clipped ? "Clip " : "NoHum ");
        l.d(static_cast<int32_t>(hum_khz_ / 1000)).c('.').d(static_cast<int32_t>(hum_khz_ % 1000 / 100)).d(static_cast<int32_t>(hum_khz_ % 100 / 10)).d(static_cast<int32_t>(hum_khz_ % 10));
    }
    char row[400];
    Fmt f{row, sizeof(row)};
    row_begin(f, "SCN_HUM", "R");
    f.s(",SC").d(hum_uid_).c(',').s(label, Fmt::Esc::Csv).s(",noise,").d(static_cast<int32_t>(hum_khz_)).s(",14,");
    if (!lost) f.d(hum_level_cdb_ / 100);
    f.s(",dBFS,");
    int32_t lat, lon;
    if (own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    f.s(",class=").s(sig_name_.c_str(), Fmt::Esc::Kv).s(";type=").s(v.family ? scan_type_tag(ST_HUM) : "-").s(";conf=").c(v.family ? 'H' : '-');
    f.s(";why=").s(lost ? "no_result" : (clipped ? "clipped" : (v.family ? "mains_modulation" : "no_mains_pattern")));
    if (!lost) {
        f.s(";mains_hz=").d(v.family).s(";harmonics=").d(v.count).s(";other=").d(v.other).s(";best_db=").d(v.best_db);
        f.s(";h60=");
        for (size_t k = 0; k < sigdet::kHumHarm; k++) f.s(k ? "/" : "").d(hum_lv_.h60[k]);
        f.s(";h50=");
        for (size_t k = 0; k < sigdet::kHumHarm; k++) f.s(k ? "/" : "").d(hum_lv_.h50[k]);
        f.s(";blocks=").d(hum_lv_.blocks).s(";clip=").d(static_cast<int32_t>(hum_clip_)).s(";wait_buf=").d(hum_wait_);
    }
    put_pos_src(f);
    log_line(row);
}

void SignatureDetectView::on_scan_event(void* ctx, const ScanEvent& e) {
    static_cast<SignatureDetectView*>(ctx)->scan_event(e);
}

namespace {

/* List label of a scan finding (up to 19 characters). */
void scan_label(const ScanEvent& e, char* out, size_t cap) {
    Fmt f{out, cap};
    const auto mhz3 = [&f](uint32_t khz) { f.d(static_cast<int32_t>(khz / 1000)).c('.').d(static_cast<int32_t>(khz % 1000 / 100)).d(static_cast<int32_t>(khz % 100 / 10)).d(static_cast<int32_t>(khz % 10)); };
    switch (e.type) {
        case ST_COMB:
        case ST_PCOMB:
            f.s(e.type == ST_COMB ? "Comb " : "PComb ");
            if (e.param >= 1000000)
                f.d(static_cast<int32_t>(e.param / 1000000)).c('.').d(static_cast<int32_t>(e.param % 1000000 / 100000)).d(static_cast<int32_t>(e.param % 100000 / 10000)).d(static_cast<int32_t>(e.param % 10000 / 1000)).c('M');
            else
                f.d(static_cast<int32_t>(e.param / 1000)).c('.').d(static_cast<int32_t>(e.param % 1000 / 100)).c('k');
            break;
        case ST_FLOOR:
            /* MHz with a decimal below 100 MHz (HF windows start at 1.5, 3.1 ...). */
            f.s("Floor+").d(e.level_cdb / 100).c(' ').d(static_cast<int32_t>(e.lo_khz / 1000));
            if (e.lo_khz < 100000) f.c('.').d(static_cast<int32_t>(e.lo_khz % 1000 / 100));
            f.c('M');
            break;
        case ST_DRIFT:
            f.s("Drift ");
            mhz3(e.f_khz);
            break;
        case ST_HUM:
            f.s("Hum").d(static_cast<int32_t>(e.param)).c(' ');
            mhz3(e.f_khz);
            break;
        default:
            f.s(scan_type_tag(e.type));
            break;
    }
}

const char* scan_why(uint8_t type) {
    switch (type) {
        case ST_COMB:
        case ST_PCOMB:
            return "regular_lines";
        case ST_FLOOR:
            return "floor_rise";
        case ST_DRIFT:
            return "drift";
        case ST_HUM:
            return "mains_modulation";
        case ST_STEADY:
            return "steady";
        case ST_BURST:
            return "burst";
        case ST_BLOCK:
            return "block";
        case ST_WIDE:
            return "wide";
        case ST_CLOCK:
            return "round_spacing";
        default:
            return "";
    }
}

}  // namespace

/* A scan finding: reported types become list entries (Kind::Scan, kept
 * alive by the engine's per-pass updates, gone with its DROP), survey types
 * a SCN_SRV row when the survey log is on. */
void SignatureDetectView::scan_event(const ScanEvent& e) {
    if (e.ev == SE_SURVEY) {
        if (survey_on() && log_ && sig_mem_ && sig_mem_->budget.take(uptime_ms())) scan_row("SCN_SRV", e);
        return;
    }
    const uint8_t mac[6] = {0, 0, 'S', 'C', e.uid, 0};
    if (e.ev == SE_DROP) {
        scan_ev_ = &e; /* the row keeps the finding's numbers (spacing, teeth, passes ...) */
        tracker_.drop_pattern(mac); /* by address: EV_DROPPED, the SCN_DROP row */
        scan_ev_ = nullptr;
        list_.set_dirty();
        return;
    }
    char label[kSigLabelLen];
    scan_label(e, label, sizeof(label));
    const int32_t db = e.level_cdb / 100;
    scan_ev_ = &e;
    Entity* x = tracker_.ingest_sig(mac, SRC_RF, static_cast<int8_t>(db > 127 ? 127 : (db < -127 ? -127 : db)), e.uid, SK_NOISE, sig_kind_tag(SK_NOISE), label, now_s(), Kind::Scan);
    scan_ev_ = nullptr;
    if (!x) return;
    x->freq_khz = e.f_khz;
    if ((e.type == ST_COMB || e.type == ST_PCOMB) && e.param && e.anchor_khz) {
        /* Track walks to a tooth, not to the middle of the extent (often a
         * gap between teeth): the tooth nearest the centre. */
        const int64_t sp = e.param;
        const int64_t d = (static_cast<int64_t>(e.f_khz) - e.anchor_khz) * 1000;
        const int64_t k = (d >= 0 ? d + sp / 2 : d - sp / 2) / sp;
        const int64_t tooth = static_cast<int64_t>(e.anchor_khz) * 1000 + k * sp;
        if (tooth > 0) x->freq_khz = static_cast<uint32_t>((tooth + 500) / 1000);
    }
    x->bw_khz = e.hi_khz > e.lo_khz ? e.hi_khz - e.lo_khz : e.res_hz / 1000;
    x->rf_cls = e.type;
    x->rf_conf = e.conf;
    list_.set_dirty();
}

/* SCN_NEW / SCN_DROP / SCN_SRV: id SC<uid>, label, kind noise, centre and
 * width, level dB over the floor; detail: class, type, conf, why, the range
 * and what was measured. */
void SignatureDetectView::scan_row(const char* event, const ScanEvent& e, const char* drop_why) {
    char label[kSigLabelLen];
    scan_label(e, label, sizeof(label));
    char row[300];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "R");
    f.s(",SC").d(e.uid).c(',').s(label, Fmt::Esc::Csv).s(",noise,").d(static_cast<int32_t>(e.f_khz)).c(',').d(static_cast<int32_t>(e.hi_khz - e.lo_khz));
    f.c(',').d(e.level_cdb / 100).s(",dB>floor,");
    int32_t lat, lon;
    if (own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    f.s(",class=").s(sig_name_.c_str(), Fmt::Esc::Kv).s(";type=").s(scan_type_tag(e.type)).s(";conf=").c("-LMH"[e.conf & 3]).s(";why=").s(scan_why(e.type));
    f.s(";lo_khz=").d(static_cast<int32_t>(e.lo_khz)).s(";hi_khz=").d(static_cast<int32_t>(e.hi_khz));
    if (e.type == ST_COMB || e.type == ST_PCOMB || e.type == ST_CLOCK) f.s(";spacing_hz=").d(static_cast<int32_t>(e.param)).s(";teeth=").d(e.count);
    if (e.type == ST_FLOOR) f.s(";rise_cdb=").d(static_cast<int32_t>(e.param)).s(";windows=").d(e.count);
    if (e.type == ST_DRIFT || e.type == ST_STEADY || e.type == ST_BURST) f.s(";range_hz=").d(static_cast<int32_t>(e.param)).s(";rate_hz_min=").d(e.rate).s(";seen=").d(e.count);
    if (e.type == ST_HUM) f.s(";mains_hz=").d(static_cast<int32_t>(e.param)).s(";harmonics=").d(e.count);
    if (e.type == ST_BLOCK || e.type == ST_WIDE) f.s(";ripple_cdb=").d(static_cast<int32_t>(e.param)).s(";fill_pct=").d(e.count);
    f.s(";passes=").d(e.passes).s(";duty=").d(e.duty).s(";width_pts=").d(e.width_pts).s(";pt_hz=").d(static_cast<int32_t>(e.res_hz));
    if (e.type == ST_COMB || e.type == ST_PCOMB) f.s(";tooth_khz=").d(static_cast<int32_t>(e.anchor_khz));
    if (drop_why) f.s(";drop=").s(drop_why);
    put_pos_src(f);
    log_line(row);
}

/* A scan finding listed (EV_CONFIRMED: the event in hand) or gone. */
void SignatureDetectView::report_scan(Tracker::Event ev, const Entity& e) {
    if (ev != Tracker::EV_CONFIRMED && ev != Tracker::EV_DROPPED) return;
    const bool drop = ev == Tracker::EV_DROPPED;
    ScanEvent s{};
    if (scan_ev_) {
        s = *scan_ev_;
    } else if (drop && scan_ && scan_->describe(e.mac[4], s)) {
        /* a class change or the log stopping: the engine's numbers as they stand */
    } else {
        s.uid = e.mac[4];
        s.type = e.rf_cls;
        s.conf = e.rf_conf;
        s.f_khz = e.freq_khz;
        s.lo_khz = e.freq_khz - e.bw_khz / 2;
        s.hi_khz = s.lo_khz + e.bw_khz;
        s.level_cdb = static_cast<int16_t>((e.rssi_max != kNoRssi ? e.rssi_max : 0) * 100);
    }
    if (log_on() || drop_stop_) scan_row(drop ? "SCN_DROP" : "SCN_NEW", s, drop ? Tracker::drop_why_name(drop_stop_ ? Tracker::DROP_STOP : tracker_.drop_why()) : nullptr);
    if (opt_.json && !drop_stop_) {
        char label[kSigLabelLen];
        scan_label(s, label, sizeof(label));
        char row[200];
        Fmt f{row, sizeof(row)};
        f.s("{\"event\":\"scan_").s(drop ? "drop" : "new").s("\",\"class\":\"").s(sig_name_.c_str(), Fmt::Esc::Json).s("\",\"type\":\"").s(scan_type_tag(s.type));
        f.s("\",\"label\":\"").s(label, Fmt::Esc::Json).s("\",\"freq_khz\":").d(static_cast<int32_t>(s.f_khz)).s(",\"conf\":").d(s.conf).c('}');
        usb_line(row);
    }
}

void SignatureDetectView::dump_line(void* ctx, const char* line) {
    auto* self = static_cast<SignatureDetectView*>(ctx);
    self->usb_line(line);
#if SIGDET_FULL
    if (self->dump_log_) self->dump_log_->write_raw(line);
#endif
}

/* Queued, never waits (sigdet_usbq.hpp): a memcpy and at most one transfer
 * start. When the host does not keep up, lines are dropped and counted
 * (usb_dropped_: DDSTAT, debug view). */
bool SignatureDetectView::usb_line(const char* line) {
    if (!portapack::async_tx_enabled || !portapack::usb_serial.serial_connected()) return false;
    if (!usbq_ && !SigdetMapView::alive()) {
        /* Not operator new: it panics when the heap is short. Not while a
         * map is open: the map view's later allocations (line buffer, file
         * names, modal) need that heap. */
        void* const mem = chHeapAlloc(nullptr, sizeof(UsbLineQueue));
        if (mem) usbq_.reset(new (mem) UsbLineQueue{});
    }
    if (!usbq_ || !usbq_->push(line)) {
        usb_dropped_++;
        return false;
    }
    usb_drain();
    return true;
}

void SignatureDetectView::usb_drain() {
    if (!usbq_ || !portapack::usb_serial.serial_connected()) return;
    if (usbq_->drain(usb_send, nullptr)) usbq_sent_ = true;
}

void SignatureDetectView::on_slice(const SpecAnSliceMessage& message) {
    slice_rx_++;
    if (!sweeping() || !sweep_ || !sweep_->sweeping()) return; /* late reply */
    const int16_t* const src = sweep_->accept(message);
    if (!src) return;
    slices_++;
    if (!message.last) return;
    if (!sweep_->sweep_complete()) {
        partial_++;
        return;
    }
    if (dwell_sweeps_ < tune::kDiscardLooks) {
        dwell_sweeps_++; /* still settling from the rate switch */
        return;
    }
    if (phase_band_ == kScan) {
        if (!scan_ || !scan_->add_look(src)) rf_full_ = true;
        return;
    }
    if (dump_raw_ && (dump_band_ == kDumpAll || dump_band_ == phase_band_)) {
        /* "DDLK,seq,band,look,ref,CHARS": ref = the look's lowest point
         * (cdBFS), then one character per point, '!' + (cdBFS - ref) / 50,
         * i.e. 0.5 dB steps up to ref + 46.5 dB (clipped above; '!'..'~').
         * Replayed on the host through RfEngine. Costs the M0 ~25 ms a look
         * on USB: fewer looks per dwell while it runs. */
        const uint16_t n = kBands[phase_band_].points;
        int16_t ref = src[0];
        for (size_t x = 1; x < n; x++)
            if (src[x] < ref) ref = src[x];
        char line[32 + RfEngine::kMaxPoints];
        Fmt f{line, sizeof(line)};
        f.s("DDLK,").d(dwell_seq_).c(',').s(kBands[phase_band_].name).c(',').d(rf_.looks()).c(',').d(ref).c(',');
        for (size_t x = 0; x < n; x++) {
            const int32_t v = (src[x] - ref) / 50;
            f.c(static_cast<char>('!' + (v > 93 ? 93 : v)));
        }
        usb_line(line);
    }
    if (!rf_.add_look(src)) rf_full_ = true;
    if (nb_dwell_ && sig_mem_) nb_look(sig_mem_->cls, sig_mem_->nb, src, rf_);
}

void SignatureDetectView::set_demo(bool on) {
    demo_on_ = on;
#if SIGDET_FULL
    if (!on) return;
    int32_t lat, lon;
    if (!own_position(lat, lon)) {
        lat = kDemoLat;
        lon = kDemoLon;
    }
    demo_.start(lat, lon);
#endif
}

void SignatureDetectView::on_frame_sync() {
    usb_drain();
    /* The signature tables live in PSDT's lent RAM: from its first stats
     * message (~500 ms) on. */
    if (!sig_mem_ && stats_.scratch && stats_.scratch_size >= SignatureDetectStatsMessage::kScratchSize)
        sig_mem_ = new (stats_.scratch + kSigAt) SigMemory{};
    if (sig_scan_pending_ && !SigdetMapView::alive()) scan_classes();
    if (sig_pick_pending_ && chTimeNow() - sig_pick_ms_ >= kClassSettleMs) {
        sig_pick_pending_ = false;
        if (sig_pick_ != sig_index_) select_class(sig_pick_);
    }
    if (sig_file_ && !sig_loaded_ && !sig_load_failed_ && sig_mem_ && !SigdetMapView::alive()) load_class();
    if (pat_load_pending_ && !sig_file_) load_patterns();
    if (log_open_ && !SigdetMapView::alive() && !sig_scan_pending_) open_log(); /* not into the heap an open map still needs */
    if (sweeping()) sweep_->on_frame();
    /* Dwells by the clock: the frame rate differs between LCDs. While the
     * track view is open the cycle stays paused (kTrack). */
    const systime_t now = chTimeNow();
    if (phase_band_ == kTrack) {
        if (track_) track_->frame(uptime_ms());
    } else if (now - phase_t0_ >= phase_dwell_ms_ || rf_full_) {
        next_phase();
    }

    /* The ESP32 module's list is Remote ID: as on-board Remote ID, a
     * detection only in a class that has it (Drones, RX rid). */
    Summary s;
    if (esp_.poll(s, now)) {
        if (detector_on(RX_RID))
            tracker_.ingest_summary(s, now_s());
        else
            rid_skip_++;
    }
}

void SignatureDetectView::on_tick_second() {
    const uint32_t now = now_s();

#if SIGDET_FULL
    if (demo_on_) {
        /* Same path as received frames: on_rid -> odid_find_ble -> Tracker. */
        SignatureDetectRidMessage m{};
        m.source = SignatureDetectRidMessage::SOURCE_SHELL;
        m.pdu_type = 0x42; /* ADV_NONCONN_IND, random address */
        m.adv_len = DemoDrones::kAdvLen;
        for (size_t i = 0; i < DemoDrones::kFramesPerSecond; i++) {
            m.chan = channel_;
            m.rssi_cdb = demo_.frame(now, i, m.addr, m.adv) * 100;
            on_rid(m);
        }
    }
#endif

    int32_t lat = 0, lon = 0;
    const bool own = own_position(lat, lon);
    tracker_.tick(now, own, lat, lon);

    log_tick();
    if (log_) {
        survey_tick();
        if (now - hb_s_ >= kHeartbeatSeconds) heartbeat_row();
    }

    refresh_status();
    list_.set_dirty();
    if (detail_) detail_->refresh();
    if (map_ && (now & 1)) refresh_map();
    if (debug_) debug_->show_stats();
    if (track_) track_->second();
    if (stat_usb_) stat_line();
}

/* "DDSTAT,..." once a second (debug key s): every counter of the debug view
 * and the scheduler's airtime, for scripted measurements. Fields in order:
 * t_s cpu max_exec_us lag_us start_us drops drops_ble drops_sweep drops_switch
 * late hop qf qf_rid qf_stats qf_lock q_peak rid_ok rid_fwd dup rcap aa ok crc
 * skip ble_ms sweep_ms cycle_ms partial stalls heap_core heap_free esp_state
 * esp_count, then per band (2.4 5.8 5.3 1.2 915 868) slices looks ms of its
 * last dwell, then pace settle_us avg usb_dropped, then pat_cands pat_matches
 * pat_hits pat_fwd pat_capped pat_rcap drops_pat pat_ms pat_hits_rx vendor_ok
 * vendor_fwd vendor_dup captured_rx slice_rx, then Wi-Fi: triggers frames
 * hdr_bad filtered fcs_bad fcs_ok beacon probe action rid fwd sync_lost
 * hdr_2m legacy dup rcap busy qf overflows backlog max_step_us drops_wifi
 * wifi_ms wifi_rx wifi_odid, then the DSSS cost: cycles per Search / Sync /
 * Lock group and per CFO pass, search phases, Search groups shed to 2 phases,
 * then bus health: i2c_faults i2c_timeouts i2c_recoveries i2c_recovery_failed
 * i2c_nacks clk_errors, then 5.1 slices looks ms, then DroneID: cands evals
 * dets skips capped b_max_us b_mean_us drops_did qf rx did_ms. */
void SignatureDetectView::stat_line() {
    using I = int32_t;
    const auto& m = stats_;
    size_t heap_free = 0;
    chHeapStatus(nullptr, &heap_free);
    const I v[] = {I(now_s()), m.cpu_pct, m.max_exec_us, m.max_lag_us, m.max_start_us, m.drops, m.drops_ble, m.drops_sweep,
                   m.drops_switch, m.drops_late, m.drops_hop, m.q_fail, m.qf_rid, m.qf_stats, m.qf_lock, m.q_peak,
                   I(m.rid_ok), I(m.rid_fwd), m.dup_suppressed, m.rate_capped, I(m.aa_hits), I(m.ok), I(m.crc_bad), m.hop_skips,
                   I(ble_ms_), I(sweep_ms_), cycle_ms_, partial_, stalls_, I(chCoreStatus()), I(heap_free),
                   I(esp_.state()), esp_.count()};
    char buf[720]; /* ~140 fields */
    Fmt f{buf, sizeof(buf)};
    f.s("DDSTAT");
    for (const I x : v) f.c(',').d(x);
    for (size_t i = 0; i < BAND_51; i++) f.c(',').d(bstat_[i].slices).c(',').d(bstat_[i].looks).c(',').d(bstat_[i].ms);
    f.c(',').d(pace_).c(',').d(settle_us_).c(',').d(avg_).c(',').d(usb_dropped_);
    /* Pattern scan and DJI adverts (appended; older parsers ignore them). */
    const I p[] = {I(m.pat_cands), I(m.pat_matches), I(m.pat_hits), I(m.pat_fwd), m.pat_capped, m.pat_rcap, m.drops_pat,
                   I(pat_ms_), I(pat_hits_rx_), I(m.vendor_ok), I(m.vendor_fwd), m.vendor_dup,
                   I(captured_rx_), I(slice_rx_)};
    for (const I x : p) f.c(',').d(x);
    /* Wi-Fi Remote ID (appended). */
    const I w[] = {I(m.wifi_triggers), I(m.wifi_frames), I(m.wifi_hdr_bad), I(m.wifi_filtered), I(m.wifi_fcs_bad),
                   I(m.wifi_fcs_ok), I(m.wifi_beacon), I(m.wifi_probe), I(m.wifi_action), I(m.wifi_rid), I(m.wifi_fwd),
                   m.wifi_sync_lost, m.wifi_hdr_2m, m.wifi_legacy, m.wifi_dup, m.wifi_rcap, m.wifi_busy, m.wifi_qf,
                   m.wifi_overflows, m.wifi_backlog, m.wifi_max_step_us, m.drops_wifi, I(wifi_ms_), I(wifi_rx_), I(wifi_odid_),
                   m.wifi_cyc_search, m.wifi_cyc_sync, m.wifi_cyc_lock, m.wifi_cyc_cfo, m.wifi_phases, I(m.wifi_shed)};
    for (const I x : w) f.c(',').d(x);
    /* Bus health (appended): I2C0 faults timeouts recoveries recovery_failed
     * nacks, failed sample-rate programmings, the ESP feed's back-offs. */
    const auto& i2c = portapack::i2c0.stats();
    const I h[] = {I(i2c.faults), I(i2c.timeouts), I(i2c.recoveries), I(i2c.recovery_failed), I(i2c.nacks),
                   I(portapack::clock_manager.sampling_frequency_errors())};
    for (const I x : h) f.c(',').d(x);
    /* Bands added later (appended): 5.1 slices looks ms. */
    for (size_t i = BAND_51; i < BAND_FIXED; i++) f.c(',').d(bstat_[i].slices).c(',').d(bstat_[i].looks).c(',').d(bstat_[i].ms);
    /* DJI DroneID (appended): candidates, tested by stage B, detected, not
     * captured (snapshot busy / lost buffer), over the stage B rate cap, stage
     * B max / mean us, drops in DroneID mode, detections lost to the queue,
     * detections received, DroneID airtime ms. */
    const I d[] = {I(m.did_cands), I(m.did_evals), I(m.did_dets), m.did_skips, m.did_capped, m.did_b_max_us, m.did_b_mean_us,
                   m.drops_did, m.did_qf, I(did_rx_), I(did_ms_)};
    for (const I x : d) f.c(',').d(x);
    /* Interference scan and its hum dwells (appended): windows, passes,
     * hum dwells, checks judged, mains found, dwells without a result, hum
     * windows received, hum airtime ms; M4: hum dwells deferred for a busy
     * Wi-Fi slot, ladder windows sent. */
    const I s[] = {I(scan_windows_), I(scan_passes_), hum_dwells_, hum_checks_, hum_found_, hum_lost_, I(hum_rx_), I(hum_ms_),
                   m.hum_deferred, m.hum_windows};
    for (const I x : s) f.c(',').d(x);
    usb_line(buf);
}

void SignatureDetectView::on_rid(const SignatureDetectRidMessage& message) {
    const size_t adv_len = message.adv_len <= SignatureDetectRidMessage::ADV_MAX ? message.adv_len : SignatureDetectRidMessage::ADV_MAX;
    size_t len = 0;
    uint8_t counter = 0;
    const uint8_t* odid = odid_find_ble(message.adv, adv_len, &len, &counter);
    if ((SIGDET_FULL && debug_) || raw_usb_) raw_line(message, odid != nullptr);
    /* Remote ID is a detection only in a class that has it (Drones, RX rid):
     * elsewhere the frame is an ordinary advertiser (rules, survey). */
    if (odid && !detector_on(RX_RID)) {
        odid = nullptr;
        rid_skip_++;
    }
    /* Track view on a BLE target: its packets feed the meter; nothing else
     * of the (target-only or pass-all) stream goes to rules or survey. */
    if (track_ && phase_band_ == kTrack) {
        const TrackTarget& t = track_->target();
        bool hit = false;
        if (t.kind == TrackTarget::BLE_ADDR) {
            hit = memcmp(message.addr, t.addr, 6) == 0;
        } else if (t.kind == TrackTarget::BLE_RULE && sig_ready()) {
            AdvInfo a;
            adv_parse(message.adv, adv_len, a);
            hit = sig_ble_match(sig_mem_->cls, message.pdu_type, a) == t.rule;
        }
        if (hit) track_->on_ble(message.rssi_cdb);
        if (!odid) return;
    }
    if (!odid) {
        /* A DJI advert (the M4 forwards those; pass-all brings every advert),
         * or any other (FLAG_FWD_ADV): the class's BLE rules, else the survey. */
        if (message.source != SignatureDetectRidMessage::SOURCE_BLE && message.source != SignatureDetectRidMessage::SOURCE_BLE_VENDOR &&
            message.source != SignatureDetectRidMessage::SOURCE_BLE_OTHER) return;
        const uint8_t* name = nullptr;
        size_t name_len = 0;
        const uint8_t why = detector_on(RX_DJI) ? sigdet::dji_ble_match(message.pdu_type, message.addr, message.adv, adv_len, &name, &name_len) : 0;
        if (why) {
            tracker_.ingest_vendor(message.addr, static_cast<int8_t>((message.rssi_cdb - 50) / 100), why, message.pdu_type & 0x0F, name, name_len, now_s());
            list_.set_dirty();
            return;
        }
        if (message.source == SignatureDetectRidMessage::SOURCE_BLE_VENDOR && detector_on(RX_DJI)) return;
        AdvInfo a;
        adv_parse(message.adv, adv_len, a);
        const int rule = sig_ready() ? sig_ble_match(sig_mem_->cls, message.pdu_type, a) : -1;
        if (rule >= 0) {
            const BleRule& r = sig_mem_->cls.ble[rule];
            if (tracker_.ingest_sig(message.addr, SRC_BLE, static_cast<int8_t>((message.rssi_cdb - 50) / 100), static_cast<uint8_t>(rule), r.kind,
                                    sig_kind_tag(r.kind), r.label, now_s()))
                list_.set_dirty();
            return;
        }
        /* Not from the BLE dwell a file class runs before it has loaded (or
         * a class without BLE): its rows would be logged under that class. */
        if (survey_on() && !(mode() & 2)) survey_ble(message, a);
        return;
    }

    uint8_t source = SRC_BLE;
    switch (message.source) {
        case SignatureDetectRidMessage::SOURCE_SHELL:
            source = SRC_DEMO;
            break;
        case SignatureDetectRidMessage::SOURCE_ESP32_BLE:
            source = SRC_ESP_BLE;
            break;
        case SignatureDetectRidMessage::SOURCE_WIFI_BEACON:
        case SignatureDetectRidMessage::SOURCE_WIFI_NAN:
            source = SRC_ESP_WIFI;
            break;
        default:
            break;
    }
    tracker_.ingest_odid(source, message.addr, static_cast<int8_t>((message.rssi_cdb - 50) / 100), false, odid, len, now_s());
}

/* A Wi-Fi Remote ID frame from the M4 (only Remote ID ever arrives, see
 * sigdet_wifi_rid.hpp): the ODID message pack of a beacon IE or a NAN service
 * info goes the same way as the BLE frames, into the Tracker with source H;
 * the legacy kinds (bench switch) only to the console / USB. Level: dBFS
 * of the channel, estimated from the 4 MHz slice (+4 dB, sigdet_dsss.hpp). */
void SignatureDetectView::on_wifi(const SignatureDetectWifiMessage& message) {
    static_assert(sigdet::SignatureDetectWifiRid::KIND_ODID_IE == ODID_WIFI_IE && sigdet::SignatureDetectWifiRid::KIND_NAN == ODID_WIFI_NAN, "kinds");
    const auto& r = message.rid;
    wifi_rx_++;
    size_t len = 0;
    uint8_t counter = 0;
    const uint8_t* pack = odid_find_wifi_payload(r.kind, r.payload, r.payload_len, &len, &counter);
    if ((SIGDET_FULL && debug_) || raw_usb_) wifi_raw_line(message, pack != nullptr);
    if (!pack) return;
    if (!detector_on(RX_RID)) {
        rid_skip_++;
        return;
    }
    if (tracker_.ingest_odid(SRC_HACKRF_WIFI, r.addr, static_cast<int8_t>((r.rssi_cdb - 50) / 100), false, pack, len, now_s())) {
        wifi_odid_++;
        wifi_rid_s_ = now_s();
        list_.set_dirty();
    }
}

/* Debug console / USB raw line for every Wi-Fi Remote ID frame. */
void SignatureDetectView::wifi_raw_line(const SignatureDetectWifiMessage& message, bool odid) {
    const auto& r = message.rid;
    static constexpr char kKind[5] = {'?', 'I', 'N', 'F', 'D'}; /* ODID IE, NAN, French, DJI */
    const char kind = kKind[r.kind < 5 ? r.kind : 0];
#if SIGDET_FULL
    if (debug_) {
        /* "W6- 025249440001 -41 I #12": channel and half, transmitter, dB, kind, counter. */
        char line[40];
        Fmt f{line, sizeof(line)};
        f.c('W').d(r.channel).c(r.slice_mhz < 0 ? '-' : '+').c(' ').hex(r.addr, 6).c(' ').d((r.rssi_cdb - 50) / 100).c(' ').c(kind);
        if (odid) f.s(" #").d(r.counter);
        debug_->write(line);
    }
#else
    (void)odid;
#endif
    if (raw_usb_) {
        /* "DDWIFI,t_ms,channel,slice,rssi_cdb,FC,kind,MAC,len,PAYLOADHEX" (at most 96 bytes of hex). */
        static constexpr size_t kHexMax = 96;
        char usb[64 + 2 * kHexMax];
        Fmt f{usb, sizeof(usb)};
        f.s("DDWIFI,").d(r.t_ms).c(',').d(r.channel).c(',').d(r.slice_mhz).c(',').d(r.rssi_cdb).c(',');
        f.hex(&r.frame_type, 1).c(',').c(kind).c(',').hex(r.addr, 6).c(',').d(r.payload_len).c(',');
        f.hex(r.payload, r.payload_len < kHexMax ? r.payload_len : kHexMax);
        usb_line(usb);
    }
}

void SignatureDetectView::on_stats(const SignatureDetectStatsMessage& message) {
    /* All but the Message header (its const id is the same anyway). */
    memcpy(reinterpret_cast<uint8_t*>(&stats_) + sizeof(Message), reinterpret_cast<const uint8_t*>(&message) + sizeof(Message),
           sizeof(stats_) - sizeof(Message));
}

/* What the ESP32 module sends (i2cdev_ppmod forwards it as is): 200/200 (the
 * message default) until its GPS has said anything, then the NMEA parser's
 * values even without a fix, i.e. 0/0 with 0 satellites while the GPS is
 * searching. Only a real position is an own position; 0/0, out of range or
 * NaN (the range test below is written so that NaN fails it: NaN lat with a
 * valid lon used to pass as lat 0) is "a GPS without a fix", and ends the use
 * of an earlier fix at once instead of after kGpsMaxAge. */
void SignatureDetectView::on_gps(const GPSPosDataMessage& message) {
    const float la = message.lat;
    const float lo = message.lon;
    const bool in_range = la >= -90.0f && la <= 90.0f && lo >= -180.0f && lo <= 180.0f;
    const int32_t lat = in_range ? to_e7(la) : 0;
    const int32_t lon = in_range ? to_e7(lo) : 0;
    if (in_range && odid_pos_valid(lat, lon)) {
        gps_lat_ = lat;
        gps_lon_ = lon;
        gps_alt_m_ = message.altitude;
        gps_sats_ = message.satinuse;
        gps_s_ = now_s();
        gps_valid_ = true;
        gps_nofix_ = false;
        return;
    }
    gps_valid_ = false;
    if (la >= 199.0f && lo >= 199.0f) return; /* 200/200: no GPS data at all */
    gps_nofix_ = true;
    gps_nofix_s_ = now_s();
}

void SignatureDetectView::on_tracker_event(void* ctx, Tracker::Event ev, const Entity& e) {
    static_cast<SignatureDetectView*>(ctx)->report(ev, e);
}

void SignatureDetectView::beep() {
    const uint32_t now = now_s();
    if (beep_s_ && now - beep_s_ < kBeepGap) return;
    beep_s_ = now;
    baseband::request_audio_beep(1400, 24000, 120);
}

/* CSV row and USB JSON line (Sky-Spy style keys) for a detection event. */
void SignatureDetectView::put_time(Fmt& f) {
    const auto t = rtc_time::now();
    f.d(t.year()).c('-').d2(t.month()).c('-').d2(t.day()).c(' ').d2(t.hour()).c(':').d2(t.minute()).c(':').d2(t.second()).c(',');
}

void SignatureDetectView::on_rf_event(void* ctx, RfEngine::Event ev, const RfTrack& t) {
    static_cast<SignatureDetectView*>(ctx)->on_rf(ev, t);
}

static_assert(offsetof(RfTrack, bw_khz) == offsetof(RfTrack, f_khz) + 4 && offsetof(Entity, bw_khz) == offsetof(Entity, freq_khz) + 4 &&
                  offsetof(RfTrack, flat) == offsetof(RfTrack, cls) + 3 && offsetof(Entity, rf_flat) == offsetof(Entity, rf_cls) + 3,
              "RfTrack / Entity blocks");

/* RF tracks: confirmed ones are Tracker entities; every change of state is
 * a CSV row (RFNEW candidate, RFCONF, RFDROP) and a USB JSON line. */
void SignatureDetectView::on_rf(RfEngine::Event ev, const RfTrack& t) {
    /* The sweep's drone classifiers only in a class that has them (Drones,
     * RX rfcls): elsewhere no RF tracks are listed or logged. */
    if (!detector_on(RX_RFCLS)) return;
    const uint8_t mac[6] = {0, 0, 'R', 'F', t.band, t.uid};
    if (ev == RfEngine::EV_DROP) {
        tracker_.drop_rf(mac);
    } else if (ev != RfEngine::EV_NEW) {
        Entity* e = tracker_.ingest_rf(mac, t.level, now_s());
        if (e) {
            memcpy(&e->freq_khz, &t.f_khz, 8); /* freq_khz, bw_khz */
            memcpy(&e->rf_cls, &t.cls, 4);     /* cls, conf, duty, flat */
            memcpy(e->label, t.label, sizeof(t.label));
        }
    }
    list_.set_dirty();
    if (ev == RfEngine::EV_UPD) return;
    if (ev == RfEngine::EV_CONF && opt_.alert && (t.conf >= CONF_MED || opt_.beep_low)) {
        baseband::request_audio_beep(500 + 100 * t.cls, 24000, 200);
    }

    /* The same fields for the CSV row (in its self_id column, commas made
     * ';') and the JSON line. */
    using I = int32_t;
    static constexpr const char* kEv[3] = {"new", "conf", "drop"};
    const char* const str[] = {rf_class_tag(t.cls), t.label, kBands[t.band].name};
    const char* const* ps = str;
    const I v[] = {I(t.f_khz), I(t.bw_khz), t.level, t.duty, t.flat, t.conf};
    const I* pv = v;
    char det[140];
    Fmt{det, sizeof(det)}.t(
        "\"rf_class\":\"$\",\"label\":\"$\",\"band\":\"$\",\"freq_khz\":#,\"bw_khz\":#,\"level\":#,"
        "\"duty\":#,\"flat\":#,\"conf\":#",
        pv, ps);
    const char* evn = kEv[ev == RfEngine::EV_NEW ? 0 : (ev == RfEngine::EV_CONF ? 1 : 2)];
    char buf[256];
    if (log_on()) { /* log_line() counts it as lost while the file is in error */
        static constexpr const char* kEvRow[3] = {"RF_NEW", "RF_CONF", "RF_DROP"};
        Fmt f{buf, sizeof(buf)};
        row_begin(f, kEvRow[ev == RfEngine::EV_NEW ? 0 : (ev == RfEngine::EV_CONF ? 1 : 2)], "R");
        f.c(',').s(kBands[t.band].name).c(':').d(t.uid).c(',').s(t.label, Fmt::Esc::Csv).c(',').s(rf_class_tag(t.cls));
        f.c(',').d(t.f_khz).c(',').d(t.bw_khz).c(',').d(t.level).s(",dB>floor,,,band=").s(kBands[t.band].name);
        f.s(";conf=").c("-LMH"[t.conf & 3]).s(";duty=").d(t.duty).s(";flat_db=").d1(t.flat);
        /* Where the centre sits in the sweep's tune slices (step: slice
         * index, step_off_khz: from the slice's lower edge; the LO is at
         * lo_off_khz): a per slice artefact repeats at one offset. */
        uint16_t step = 0;
        uint32_t off = 0;
        if (t.band == phase_band_ && sweep_ && rf_.slice_of(t.f_khz, step, off)) {
            const auto& pl = sweep_->plan();
            f.s(";step=").d(step).s(";step_off_khz=").d(static_cast<int32_t>(off));
            f.s(";step_khz=").d(static_cast<int32_t>(((static_cast<uint64_t>(pl.bins_per_slice) * pl.fs) >> pl.log2n) / 1000));
            f.s(";lo_off_khz=").d(static_cast<int32_t>(((static_cast<int64_t>(-pl.bin_first) * pl.fs) >> pl.log2n) / 1000));
        }
        log_line(buf);
    }
    if (opt_.json) {
        Fmt f{buf, sizeof(buf)};
        f.s("{\"event\":\"rf_").s(evn).s("\",").s(det).c('}');
        usb_line(buf);
    }
}

void SignatureDetectView::report(Tracker::Event ev, const Entity& e) {
    if (ev == Tracker::EV_CONFIRMED && e.rid && opt_.alert) beep();
    if (ev == Tracker::EV_CONFIRMED) list_.set_dirty();
    if (e.kind == Kind::Pattern || e.kind == Kind::BleVendor || e.kind == Kind::DroneId) {
        report_other(ev, e);
        return;
    }
    if (e.kind == Kind::Sig) {
        report_sig(ev, e);
        return;
    }
    if (e.kind == Kind::Scan) {
        report_scan(ev, e);
        return;
    }
    if (e.kind == Kind::Fsk) {
        /* The rule's frames stopped (kExpireSeconds): a summary row. */
        if (ev == Tracker::EV_DROPPED) fsk_row("FSK_SUM", e.mac[4], nullptr);
        return;
    }
    if (e.kind == Kind::Zwave) {
        /* The network went quiet (kExpireSeconds): a summary row. */
        if (ev == Tracker::EV_DROPPED && sig_mem_) {
            ZwNet* n = sig_mem_->zw.find(e.mac + 2);
            if (n) zw_row("ZW_SUM", *n, nullptr, e.rssi_max != kNoRssi ? static_cast<int16_t>(e.rssi_max * 100) : -12000);
        }
        return;
    }
    if (e.kind == Kind::Rf || (!log_on() && !opt_.json)) return;

    using I = int32_t;
    constexpr I N = Fmt::kNone;
    RidRecord none{};
    const RidRecord& r = e.rid ? *e.rid : none;
    const bool loc = r.msg_seen & MSG_LOCATION;
    const bool pos = odid_pos_valid(r.lat_e7, r.lon_e7);
    const bool op = odid_pos_valid(r.op_lat_e7, r.op_lon_e7);
    const bool sys = r.msg_seen & MSG_SYSTEM;
    char name[24], src[6], mac[18];
    entity_name(e, name, sizeof(name));
    Fmt fs{src, sizeof(src)};
    put_sources(fs, e.sources, 4);
    static constexpr const char* kEv[3][2] = {{"RID_NEW", "new"}, {"RID_LOC", "loc"}, {"RID_DROP", "drop"}};
    const size_t evi = ev == Tracker::EV_CONFIRMED ? 0 : (ev == Tracker::EV_LOCATION ? 1 : 2);
    /* Empty (N) when unknown; the JSON line takes 0 there. */
    const I v[] = {e.rid ? I(r.id_type) : N, e.rid ? I(r.ua_type) : N, loc ? I(r.status) : N,
                   pos ? r.lat_e7 : N, pos ? r.lon_e7 : N,
                   loc && odid_alt_valid(r.alt_geo_raw) ? odid_alt_dm(r.alt_geo_raw) : N,
                   loc && odid_alt_valid(r.height_raw) ? odid_alt_dm(r.height_raw) : N,
                   loc && odid_speed_h_valid(r) ? odid_speed_h_cms(r) : N,
                   loc && odid_direction_valid(r) ? I(odid_direction_deg(r)) : N,
                   loc && odid_speed_v_valid(r) ? odid_speed_v_cms(r) : N,
                   op ? r.op_lat_e7 : N, op ? r.op_lon_e7 : N,
                   sys && odid_alt_valid(r.op_alt_raw) ? odid_alt_dm(r.op_alt_raw) : N,
                   sys ? I(r.op_loc_type) : N, e.rssi != kNoRssi ? I(e.rssi) : N};
    auto z = [&](size_t i, I div) { return v[i] == N ? 0 : v[i] / div; };
    char buf[360];

    if (log_on()) { /* log_line() counts it as lost while the file is in error */
        /* id: UAS ID (else the address); level, unit; drone lat, lon; the
         * rest as key=value (empty values left out). */
        Fmt f{buf, sizeof(buf)};
        row_begin(f, kEv[evi][0], src);
        Fmt{mac, sizeof(mac)}.hex(e.mac, 6, ':');
        f.c(',').s(name, Fmt::Esc::Csv).s(",,remote_id,,,");
        if (v[14] != N) f.d(v[14]);
        f.c(',').s(e.rssi_dbm ? "dBm" : "dBFS").c(',');
        if (pos) f.e7(r.lat_e7);
        f.c(',');
        if (pos) f.e7(r.lon_e7);
        f.s(",mac=").s(mac);
        static constexpr const char* kKeys[] = {"id_type", "ua_type", "status", nullptr, nullptr, "alt_geo_m", "height_m", "speed_ms",
                                                "heading", "vspeed_ms", "op_lat", "op_lon", "op_alt_m", "op_loc"};
        for (size_t i = 0; i < sizeof(kKeys) / sizeof(kKeys[0]); i++) {
            if (!kKeys[i] || v[i] == N) continue;
            f.c(';').s(kKeys[i]).c('=');
            if (i == 5 || i == 6 || i == 12)
                f.d1(v[i]);
            else if (i == 7 || i == 9)
                f.d2to1(v[i]);
            else if (i == 10 || i == 11)
                f.e7(v[i]);
            else
                f.d(v[i]);
        }
        if (r.operator_id[0]) f.s(";operator_id=").s(r.operator_id, Fmt::Esc::Kv);
        if (r.self_id[0]) f.s(";self_id=").s(r.self_id, Fmt::Esc::Kv);
        log_line(buf);
    }

    if (opt_.json) {
        Fmt{mac, sizeof(mac)}.hex(e.mac, 6, ':', true);
        const I j[] = {z(14, 1), z(3, 1), z(4, 1), z(5, 10), z(10, 1), z(11, 1), z(6, 10), z(7, 100), z(8, 1), r.op_loc_type};
        const char* const str[] = {mac, name, kEv[evi][1], src, r.operator_id};
        const I* pv = j;
        const char* const* ps = str;
        Fmt f{buf, sizeof(buf)};
        f.t("{\"mac\":\"$\",\"rssi\":#,\"drone_lat\":@,\"drone_long\":@,\"drone_altitude\":#,\"pilot_lat\":@,"
            "\"pilot_long\":@,\"basic_id\":\"$\",\"event\":\"$\",\"src\":\"$\",\"height\":#,\"speed\":#,"
            "\"heading\":#,\"op_type\":#,\"operator_id\":\"$\"}",
            pv, ps, Fmt::Esc::Json);
        usb_line(buf);
    }
}

/* Pattern scan detections, DJI BLE presence and DJI DroneID presence: beep
 * (pattern: always; DJI BLE: by name, else only with "Beep low conf"; DroneID:
 * only with "Beep low conf", it was checked on recordings only), a CSV row
 * (event PATNEW / PATDROP / DJINEW / DJIDROP / DIDNEW / DIDDROP, details in
 * the self_id column with commas made ';') and a USB JSON line. */
void SignatureDetectView::report_other(Tracker::Event ev, const Entity& e) {
    const bool pat = e.kind == Kind::Pattern;
    const bool did = e.kind == Kind::DroneId;
    if (ev == Tracker::EV_CONFIRMED && opt_.alert && (pat || (!did && (e.why & sigdet::VENDOR_NAME)) || opt_.beep_low)) beep();
    if (ev == Tracker::EV_LOCATION || (!log_on() && !opt_.json)) return;
    const bool drop = ev == Tracker::EV_DROPPED;
    using I = int32_t;
    char det[220], mac[18];
    const I rssi = e.rssi != kNoRssi ? e.rssi : 0;
    if (pat) {
        const char* const str[] = {e.cls, e.label};
        const I v[] = {I(e.freq_khz), rssi, e.rate10};
        const I* pv = v;
        const char* const* ps = str;
        Fmt{det, sizeof(det)}.t("\"class\":\"$\",\"name\":\"$\",\"freq_khz\":#,\"rssi\":#,\"hits_per_s\":~", pv, ps, Fmt::Esc::Json);
        mac[0] = 0;
    } else if (did) {
        const I v[] = {I(e.freq_khz), rssi, e.did_snr, e.did_zc, e.did_nsym, e.frames};
        const I* pv = v;
        const char* const* none = nullptr;
        Fmt{det, sizeof(det)}.t(
            "\"name\":\"DroneID?\",\"freq_khz\":#,\"rssi\":#,\"snr_cdb\":#,\"zc_milli\":#,\"nsym\":#,\"bursts\":#,"
            "\"note\":\"DJI DroneID presence only, no position; checked on replayed recordings only, never live\"",
            pv, none, Fmt::Esc::Json);
        mac[0] = 0;
    } else {
        Fmt{mac, sizeof(mac)}.hex(e.mac, 6, ':');
        char why[16];
        Fmt w{why, sizeof(why)};
        static constexpr const char* kWhy[3] = {"oui", "name", "cid"}; /* VENDOR_OUI, _NAME, _CID */
        for (size_t i = 0; i < 3; i++)
            if (e.why & (1u << i)) (w.len() ? w.c('+') : w).s(kWhy[i]);
        const char* const str[] = {mac, e.label, why};
        const I v[] = {rssi};
        const I* pv = v;
        const char* const* ps = str;
        Fmt{det, sizeof(det)}.t("\"mac\":\"$\",\"name\":\"$\",\"match\":\"$\",\"rssi\":#,\"note\":\"presence only, no position\"", pv, ps,
                                Fmt::Esc::Json);
    }
    char buf[320];
    if (log_on()) { /* log_line() counts it as lost while the file is in error */
        Fmt f{buf, sizeof(buf)};
        char ev_name[12];
        Fmt{ev_name, sizeof(ev_name)}.s(pat ? "PAT_" : (did ? "DID_" : "DJI_")).s(drop ? "DROP" : "NEW");
        row_begin(f, ev_name, pat ? "P" : (did ? "O" : "B"));
        f.c(',').s(pat ? "" : mac).c(',').s(e.label, Fmt::Esc::Csv).c(',').s(pat ? e.cls : (did ? "droneid" : "dji_ble")).c(',');
        if (pat || did) f.d(e.freq_khz);
        f.s(",,").d(rssi).s(",dBFS,,,");
        /* The JSON fields, as key=value. */
        bool q = false;
        for (const char* p = det; *p; p++) {
            const char ch = *p;
            if (ch == '"') {
                q = !q;
                continue;
            }
            if (q)
                f.c((ch == ',' || ch == ';' || ch == '=') ? '_' : ch);
            else
                f.c(ch == ',' ? ';' : (ch == ':' ? '=' : ch));
        }
        log_line(buf);
    }
    if (opt_.json) {
        Fmt f{buf, sizeof(buf)};
        f.s("{\"event\":\"").s(pat ? "pattern_" : (did ? "droneid_" : "dji_ble_")).s(drop ? "drop" : "new").s("\",").s(det).c('}');
        usb_line(buf);
    }
}

/* ---------------------------------------------------------------- signature classes */

/* The .TXT files of /SIGDET/SIGS, one class per file (as Looking Glass lists
 * /LOOKINGGLASS), on the wheel after the built-in "Drones"; DRONES.TXT only
 * documents the built-in class and is not listed. The class saved by name is
 * selected again. */
const char* SignatureDetectView::sd_root() const {
    return sd_legacy_ ? kSdRootOld : kSdRoot;
}

void SignatureDetectView::scan_classes() {
    sig_scan_pending_ = false;
    {
        static constexpr char kNew[] = "/SIGDET/SIGS", kOld[] = "/DRONEDET/SIGS";
        sd_legacy_ = !file_exists(std::filesystem::path{kNew, kNew + sizeof(kNew) - 1}) &&
                     file_exists(std::filesystem::path{kOld, kOld + sizeof(kOld) - 1});
    }
    char sig_dir[24];
    Fmt{sig_dir, sizeof(sig_dir)}.s(sd_root()).s("/SIGS");
    OptionsField::options_t opts;
    opts.emplace_back("Drones", 0);
    size_t sel = 0;
    scan_root_files(std::filesystem::path{sig_dir, sig_dir + strlen(sig_dir)}, u"*.TXT", [&](const std::filesystem::path& path) {
        if (opts.size() > kMaxClasses || path.empty() || path.native()[0] == u'.') return;
        const std::string stem = path.stem().string();
        if (stem.empty() || stem.size() >= kSigNameLen || stem == "DRONES" || stem == "Drones") return;
        if (stem == sig_name_) sel = opts.size();
        opts.emplace_back(stem, static_cast<int32_t>(opts.size()));
    });
    field_sig_.set_options(std::move(opts));
    if (sel != field_sig_.selected_index()) field_sig_.set_selected_index(sel); /* on_change: a pending pick */
    sig_pick_pending_ = false;                                                  /* the saved class at once */
    select_class(sel);
}

/* Index 0: Drones (built in: Setup's receivers and bands, PATTERNS.TXT);
 * else the file is loaded on the next frame (a wheel turned fast loads only
 * where it stops). */
void SignatureDetectView::select_class(size_t index) {
    sig_index_ = static_cast<uint8_t>(index);
    cycle_ms_ = 0; /* the next cycle (or scan pass) of the new class sets it */
    /* The survey tables stay (first-seen memory included: an advertiser is
     * not "new" again because the class changed); energy entries of the old
     * class's custom bands get their last summary and go. */
    if (sig_mem_) {
        if (log_ && survey_on())
            while (const NrgSeen* e = sig_mem_->nrg.pending(BAND_X0)) survey_nrg_row("NRG_SUM", *e);
        sig_mem_->nrg.forget_bands(BAND_X0);
    }
    /* The old class's detections go first, while its rules (labels, valid)
     * and name are still loaded: their SIG_DROP / ZW_SUM rows name it. */
    tracker_.drop_kind(Kind::Sig);
    tracker_.drop_kind(Kind::Zwave);
    tracker_.drop_kind(Kind::Fsk);
    scan_memory(false, Tracker::DROP_CLASS); /* SCN_DROP rows (Kind::Scan), the scan's tables go */
    /* ... and every built-in detector's (RID_DROP / DJI_DROP / PAT_DROP /
     * DID_DROP rows): the new class lists only what it asks for. */
    tracker_.drop_kind(Kind::RemoteId);
    tracker_.drop_kind(Kind::BleVendor);
    tracker_.drop_kind(Kind::DroneId);
    tracker_.drop_kind(Kind::Pattern);
    tracker_.drop_kind(Kind::Rf);
    list_.set_dirty();
    const auto& opts = field_sig_.options();
    sig_name_ = index < opts.size() ? opts[index].first : std::string{"Drones"};
    const bool file = index != 0;
    sig_file_ = file;
    sig_loaded_ = false;
    sig_load_failed_ = false;
    nb_dwell_ = false;
    /* Custom bands of the class before go; the Drones class reloads its
     * pattern table. */
    for (uint8_t b = BAND_X0; b < BAND_COUNT; b++) {
        set_custom_band(b, 0, 0, 0, 0, "");
        rf_.reset_band(b);
    }
    if (!file) {
        pat_load_pending_ = true;
        pat_table_.begin_file(true);
        pat_table_.finish();
        pat_scan_.reset();
    } else {
        pat_table_.begin_file(false);
        pat_scan_.reset();
    }
    if (sweeping() || phase_band_ == kPat || phase_band_ == kWifi || phase_band_ == kDid || phase_band_ == kHum) next_phase(); /* the old class's dwell ends */
    apply_options();
    if (log_) start_row("CLASS"); /* the class change, with the build, in the log */
}

void SignatureDetectView::load_class() {
    SigMemory& m = *sig_mem_;
    sig_loaded_ = true;
    Fmt{m.cls.name, sizeof(m.cls.name)}.s(sig_name_.c_str());
    char path[48];
    Fmt{path, sizeof(path)}.s(sd_root()).s("/SIGS/").s(sig_name_.c_str()).s(".TXT");
    m.parser.begin(m.cls, &pat_table_);
    File f;
    if (f.open(std::filesystem::path{path, path + strlen(path)}).is_valid()) {
        sig_load_failed_ = true;
    } else {
        char buf[64];
        for (size_t total = 0; total < 32768;) {
            const auto r = f.read(buf, sizeof(buf));
            if (!r || r.value() == 0) break;
            m.parser.feed(buf, r.value());
            total += r.value();
        }
        m.parser.finish();
    }
    pat_scan_.reset();
    pat_load_pending_ = false;
    for (size_t i = 0; i < m.cls.nbands; i++) {
        const SigBand& b = m.cls.bands[i];
        set_custom_band(static_cast<uint8_t>(BAND_X0 + i), b.lo_mhz, b.hi_mhz, b.points, b.dwell_cs, b.name);
    }
    /* m.ble / m.nrg stay across classes (select_class). */
    for (auto& x : m.fsk) x = FskSeen{};
    for (size_t i = 0; i < kSigMaxNb; i++) {
        m.nb_hits[i] = 0;
        m.nb_hit_s[i] = 0;
    }
    fsk_cursor_ = 0;
    apply_options();
    configure(); /* FLAG_FWD_ADV for BLE rules */
}

/* NB rules of the sweep dwell just finished, then the survey's energy events. */
void SignatureDetectView::sig_dwell_end(uint16_t ms) {
    const bool nb = nb_dwell_;
    nb_dwell_ = false;
    if (!sig_mem_ || !rf_.looks()) return;
    const uint8_t band = phase_band_;
    if (nb && sig_ready()) {
        SigClass& c = sig_mem_->cls;
        for (size_t i = 0; i < c.nnb; i++) {
            if (c.nb[i].band != band || !sig_mem_->nb[i].active) continue;
            const NbResult r = nb_eval(c.nb[i], sig_mem_->nb[i], ms);
            if (!r.match) continue;
            sig_mem_->nb_last[i] = r;
            const uint8_t mac[6] = {0, 0, 'N', 'B', static_cast<uint8_t>(i), 0};
            /* Listed after hits in NbRule::dwells dwells (kNbPairSeconds
             * apart at most), then refreshed by each; the Z-Wave energy rule
             * only while Z-Wave frames are being decoded. */
            const uint32_t now = now_s();
            if (now - sig_mem_->nb_hit_s[i] > kNbPairSeconds) sig_mem_->nb_hits[i] = 0;
            if (sig_mem_->nb_hits[i] < 255) sig_mem_->nb_hits[i]++;
            sig_mem_->nb_hit_s[i] = now;
            if (nb_tied_to_zw(c, i) && !(zw_ok_s_ && now - zw_ok_s_ < kNbZwTieSeconds)) continue;
            if (!tracker_.find_addr(mac) && sig_mem_->nb_hits[i] < c.nb[i].dwells) continue;
            const int32_t db = r.max_cdb / 100;
            char label[kSigLabelLen + 2];
            Fmt{label, sizeof(label)}.s(c.nb[i].label).c('?'); /* low confidence: energy only */
            if (tracker_.ingest_sig(mac, SRC_RF, static_cast<int8_t>(db > 127 ? 127 : db), static_cast<uint8_t>(i), c.nb[i].kind,
                                    sig_kind_tag(c.nb[i].kind), label, now_s()))
                list_.set_dirty();
        }
    }
    if (survey_on() && log_) {
        NrgEvent ev[kNrgPerDwell];
        const size_t n = nrg_extract(rf_, ev, kNrgPerDwell);
        const uint32_t now = uptime_ms();
        for (size_t k = 0; k < n; k++) {
            NrgSeen* e = sig_mem_->nrg.add(band, ev[k], now);
            if (e && sig_mem_->budget.take(now)) {
                NrgSurvey::mark(*e, now);
                survey_nrg_row("NRG_NEW", *e);
            }
        }
    }
}

/* A signature rule hit (BLE advert or NB energy): beep (BLE rules with
 * beep=1), CSV row SIG_NEW / SIG_DROP, USB JSON sig_new / sig_drop. */
void SignatureDetectView::report_sig(Tracker::Event ev, const Entity& e) {
    if (ev == Tracker::EV_LOCATION) return;
    const bool nb = (e.sources & SRC_RF) != 0;
    const SigClass* c = sig_ready() ? &sig_mem_->cls : nullptr;
    const BleRule* br = (c && !nb && e.why < c->nble) ? &c->ble[e.why] : nullptr;
    const NbRule* nr = (c && nb && e.why < c->nnb) ? &c->nb[e.why] : nullptr;
    if (ev == Tracker::EV_CONFIRMED && opt_.alert && br && (br->flags & BR_BEEP)) beep();
    const bool logging = log_on() || drop_stop_; /* Log just unticked: the STOP summaries still go into the file */
    if (!logging && !opt_.json) return;
    const bool drop = ev == Tracker::EV_DROPPED;
    const uint8_t valid = br ? br->valid : (nr ? nr->valid : static_cast<uint8_t>(SV_DOC));
    char id[20];
    Fmt fi{id, sizeof(id)};
    if (nb)
        fi.s("NB").d(e.why);
    else
        fi.hex(e.mac, 6, ':');
    char row[300];
    if (logging) { /* log_line() counts it as lost while the file is in error */
        Fmt f{row, sizeof(row)};
        row_begin(f, drop ? "SIG_DROP" : "SIG_NEW", nb ? "R" : "B");
        f.c(',').s(id).c(',').s(e.label, Fmt::Esc::Csv).c(',').s(sig_kind_name(e.pat)).c(',');
        if (nr) f.d(nr->f_khz);
        f.c(',');
        if (nr) f.d(nr->bw_khz);
        f.c(',');
        if (e.rssi_max != kNoRssi) f.d(drop ? e.rssi_max : e.rssi);
        f.c(',').s(nb ? "dB>floor" : "dBFS").c(',');
        int32_t lat, lon;
        if (own_position(lat, lon))
            f.e7(lat).c(',').e7(lon);
        else
            f.c(',');
        f.s(",class=").s(sig_name_.c_str(), Fmt::Esc::Kv).s(";rule=").d(e.why).s(";valid=").s(sig_valid_name(valid));
        f.s(";frames=").d(e.frames).s(";seen_s=").d(e.last_s - e.first_s);
        if (drop) f.s(";max=").d(e.rssi_max).s(";why=").s(Tracker::drop_why_name(drop_stop_ ? Tracker::DROP_STOP : tracker_.drop_why()));
        if (!drop && e.requeued) f.s(";requeued=1");
        if (nr) {
            const NbResult& r = sig_mem_->nb_last[e.why];
            f.s(";conf=low;duty=").d(r.duty).s(";runs=").d(r.runs).s(";burst_ms=").d(r.burst_ms).s(";period_ms=").d(r.period_ms);
        }
        if (br) f.s(";pos=receiver"); /* lat/lon: own position, presence only */
        put_pos_src(f);
        log_line(row);
    }
    if (opt_.json && !drop_stop_) {
        Fmt f{row, sizeof(row)};
        f.s("{\"event\":\"sig_").s(drop ? "drop" : "new").s("\",\"class\":\"").s(sig_name_.c_str(), Fmt::Esc::Json);
        f.s("\",\"label\":\"").s(e.label, Fmt::Esc::Json).s("\",\"kind\":\"").s(sig_kind_name(e.pat)).s("\",\"src\":\"").s(nb ? "R" : "B");
        f.s("\",\"id\":\"").s(id).s("\",\"level\":").d(e.rssi != kNoRssi ? e.rssi : 0).s(",\"valid\":\"").s(sig_valid_name(valid)).s("\"}");
        usb_line(row);
    }
}

/* ---------------------------------------------------------------- survey log */

uint32_t SignatureDetectView::uptime_ms() const {
    return static_cast<uint32_t>(chTimeNow() * 1000ull / CH_FREQUENCY);
}

/* An advert no detector and no rule took: the survey table, a BLE_SEEN row
 * the first time (rate limited), BLE_SUM rows later (survey_tick). */
void SignatureDetectView::survey_ble(const SignatureDetectRidMessage& message, const AdvInfo& a) {
    (void)a;
    if (!sig_mem_ || !log_) return;
    const size_t adv_len = message.adv_len <= SignatureDetectRidMessage::ADV_MAX ? message.adv_len : SignatureDetectRidMessage::ADV_MAX;
    const uint32_t now = uptime_ms();
    BleSurvey& sv = sig_mem_->ble;
    const auto res = sv.add(message.addr, message.pdu_type, message.chan, static_cast<int8_t>((message.rssi_cdb - 50) / 100), message.adv,
                            adv_len, now);
    if (const BleSeen* ev = sv.take_evicted())
        if (sig_mem_->budget.take(now)) survey_ble_row("BLE_SUM", *ev);
    if (res == BleSurvey::REPEAT) {
        sv.mark(sv.entry(), now); /* reported this session already: a summary in 5 min if still heard */
    } else if (res == BleSurvey::NEW && sig_mem_->budget.take(now)) {
        BleSeen& e = sv.entry();
        sv.mark(e, now);
        survey_ble_row("BLE_SEEN", e);
    }
}

namespace {

const char* pdu_name(uint8_t pdu) {
    switch (pdu & 0x0F) {
        case 0:
            return "ADV_IND";
        case 2:
            return "ADV_NONCONN_IND";
        case 4:
            return "SCAN_RSP";
        case 6:
            return "ADV_SCAN_IND";
        default:
            return "?";
    }
}

}  // namespace

void SignatureDetectView::survey_ble_row(const char* event, const BleSeen& s) {
    char row[300];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "B");
    f.c(',').hex(s.addr, 6, ':').s(",,ble,,,").d(s.rssi_max).s(",dBFS,");
    int32_t lat, lon;
    if (own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    f.s(",pdu=");
    bool first = true;
    for (uint8_t t = 0; t < 8; t++)
        if (s.pdus & (1u << t)) {
            if (!first) f.c('+');
            f.s(pdu_name(t));
            first = false;
        }
    f.s(";addr_type=").s((s.pdu & 0x40) ? "random" : "public");
    if (s.has_cid) {
        const uint8_t cid[2] = {static_cast<uint8_t>(s.cid >> 8), static_cast<uint8_t>(s.cid)};
        f.s(";cid=").hex(cid, 2).s(";mfr=").hex(s.mfr, s.mfr_n);
    }
    for (size_t k = 0; k < s.nuuid; k++) {
        const uint8_t u[2] = {static_cast<uint8_t>(s.uuid[k] >> 8), static_cast<uint8_t>(s.uuid[k])};
        f.s(k ? "/" : ";uuid=").hex(u, 2);
    }
    if (s.has_sd) {
        const uint8_t u[2] = {static_cast<uint8_t>(s.sd_uuid >> 8), static_cast<uint8_t>(s.sd_uuid)};
        f.s(";sd=").hex(u, 2).s(";sd_data=").hex(s.sd_data, s.sd_n);
    }
    f.s(";len=").d(s.adv_len).s(";name=").d(s.has_name).s(";n=").d(s.count).s(";n_new=").d(s.count - s.count_emit);
    f.s(";rssi_min=").d(s.rssi_min).s(";rssi_max=").d(s.rssi_max);
    if (s.gap_min_ms != 0xFFFF) f.s(";gap_min_fwd_ms=").d(s.gap_min_ms);
    f.s(";seen_s=").d((s.last_ms - s.first_ms) / 1000).s(";ch=");
    bool any = false;
    for (size_t k = 0; k < 3; k++)
        if (s.chans & (1u << k)) {
            if (any) f.c('+');
            f.d(37 + k);
            any = true;
        }
    put_pos_src(f);
    log_line(row);
}

void SignatureDetectView::survey_nrg_row(const char* event, const NrgSeen& s) {
    char row[260];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "R");
    f.c(',').s(kBands[s.band].name).c(',').s(s.cls != RC_NONE ? rf_class_tag(s.cls) : "").s(",energy,").d(s.f_khz).c(',').d(s.bw_khz);
    f.c(',').d(s.db_max).s(",dB>floor,");
    int32_t lat, lon;
    if (own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    f.s(",band=").s(kBands[s.band].name);
    if (s.band >= BAND_X0) f.s(";lo_mhz=").d(kBands[s.band].lo_mhz).s(";hi_mhz=").d(kBands[s.band].hi_mhz).s(";class=").s(sig_name_.c_str(), Fmt::Esc::Kv);
    f.s(";mean_db=").d(s.mean_max).s(";duty=").d(s.duty_max).s(";dwells=").d(s.dwells);
    f.s(";dwells_new=").d(s.dwells - s.dwells_emit).s(";seen_s=").d((s.last_ms - s.first_ms) / 1000);
    f.s(";conf=").c("-LMH"[s.conf & 3]).s(";pt_khz=").d(kBands[s.band].points > 1 ? (kBands[s.band].hi_mhz - kBands[s.band].lo_mhz) * 1000u / (kBands[s.band].points - 1u) : 0);
    put_pos_src(f);
    log_line(row);
}

/* Once a second: summaries that are due, within the row budget. */
void SignatureDetectView::survey_tick() {
    if (!sig_mem_ || !survey_on()) return;
    const uint32_t now = uptime_ms();
    for (size_t i = 0; i < 3; i++) {
        if (!sig_mem_->budget.take(now)) break;
        const BleSeen* e = sig_mem_->ble.due(now, kSurveyPeriodMs);
        if (!e) {
            sig_mem_->budget.refund();
            break;
        }
        survey_ble_row("BLE_SUM", *e);
    }
    for (size_t i = 0; i < 2; i++) {
        if (!sig_mem_->budget.take(now)) break;
        const NrgSeen* e = sig_mem_->nrg.due(now, kSurveyPeriodMs);
        if (!e) {
            sig_mem_->budget.refund();
            break;
        }
        survey_nrg_row("NRG_SUM", *e);
    }
}

/* Every entry with news since its last row (app exit, Log off): no budget. */
void SignatureDetectView::survey_flush() {
    if (!sig_mem_ || !survey_on()) return;
    while (const BleSeen* e = sig_mem_->ble.pending()) survey_ble_row("BLE_SUM", *e);
    while (const NrgSeen* e = sig_mem_->nrg.pending()) survey_nrg_row("NRG_SUM", *e);
}

/* HB row every kHeartbeatSeconds: is everything alive, and what did each
 * receiver see (the counters are since the start or the last reset). */
void SignatureDetectView::heartbeat_row() {
    hb_s_ = now_s();
    const auto& m = stats_;
    size_t heap_free = 0;
    chHeapStatus(nullptr, &heap_free);
    const auto& i2c = portapack::i2c0.stats();
    using I = int32_t;
    static constexpr const char* kKeys[] = {
        "up_s", "cyc_ms", "cpu", "ble_aa", "ble_ok", "ble_crc", "rid_ok", "rid_fwd", "other_fwd", "other_dup", "other_lost", "dji_ok",
        "drops", "drops_ble", "drops_sweep", "drops_switch", "drops_pat", "drops_wifi", "qf", "qf_rid", "fsk_syncs", "fsk_ok", "fsk_bad", "m0_stack_free_w", "rid_skip", "wifi_frames", "wifi_fcs_ok", "wifi_fcs_bad", "wifi_beacon", "wifi_probe", "wifi_action",
        "wifi_rid", "pat_hits", "pat_rx", "did_cands", "did_dets", "slices", "stalls", "partial", "i2c_faults", "i2c_recov",
        "clk_err", "entities", "heap_core", "heap_free", "rows", "lost", "log_err", "bytes", "srv_drop", "srv_ble", "srv_nrg", "usb_drop",
        "lna", "vga", "amp", "fsk_nochk", "fsk_rx"};
    const I v[] = {I(now_s()), cycle_ms_, m.cpu_pct, I(m.aa_hits), I(m.ok), I(m.crc_bad), I(m.rid_ok), I(m.rid_fwd), I(m.other_fwd),
                   m.other_dup, m.other_qf, I(m.vendor_ok), m.drops, m.drops_ble, m.drops_sweep, m.drops_switch, m.drops_pat, m.drops_wifi, m.q_fail, m.qf_rid, I(m.fsk_syncs), I(m.fsk_ok), I(m.fsk_bad + m.fsk_qf), I(get_free_stack_space()), I(rid_skip_), I(m.wifi_frames), I(m.wifi_fcs_ok),
                   I(m.wifi_fcs_bad), I(m.wifi_beacon), I(m.wifi_probe), I(m.wifi_action), I(m.wifi_rid), I(m.pat_hits),
                   I(pat_hits_rx_), I(m.did_cands), I(m.did_dets), I(slice_rx_), stalls_, partial_, I(i2c.faults), I(i2c.recoveries),
                   I(portapack::clock_manager.sampling_frequency_errors()), I(tracker_.size()), I(chCoreStatus()), I(heap_free),
                   I(log_rows_ + 2), I(log_lost_), I(log_errs_), I(log_bytes_), sig_mem_ ? I(sig_mem_->budget.dropped()) : 0,
                   sig_mem_ ? I(sig_mem_->ble.used()) : 0, sig_mem_ ? I(sig_mem_->nrg.used()) : 0, I(usb_dropped_),
                   I(receiver_model.lna()), I(receiver_model.vga()), I(receiver_model.rf_amp()), I(m.fsk_nochk), I(fsk_rx_)};
    static_assert(sizeof(kKeys) / sizeof(kKeys[0]) == sizeof(v) / sizeof(v[0]), "heartbeat keys");
    /* Two rows (HB: system, BLE, drops; HB2: Wi-Fi, pattern, DroneID,
     * sweep, bus, memory, log), each well under 512 bytes. */
    constexpr size_t kSplit = 25; /* first key of HB2: wifi_frames */
    static_assert(kSplit < sizeof(v) / sizeof(v[0]), "heartbeat split");
    int32_t lat = 0, lon = 0;
    const bool own = own_position(lat, lon);
    for (size_t part = 0; part < 2; part++) {
        char row[512];
        Fmt f{row, sizeof(row)};
        row_begin(f, part ? "HB2" : "HB", "");
        f.s(",,").s(sig_name_.c_str(), Fmt::Esc::Csv).s(",,,,,,");
        if (own)
            f.e7(lat).c(',').e7(lon);
        else
            f.c(',');
        f.c(',');
        const size_t from = part ? kSplit : 0, to = part ? sizeof(v) / sizeof(v[0]) : kSplit;
        for (size_t i = from; i < to; i++) f.s(i != from ? ";" : "").s(kKeys[i]).c('=').d(v[i]);
        if (!part) {
            f.s(";gps=").s(gps_fix() ? "fix" : (gps_nofix() ? "nofix" : "none"));
            f.s(";survey=").d(survey_on());
            if (sig_file_) {
                f.s(";class_ok=").d(sig_ready() && !sig_load_failed_);
                if (sig_ready() && sig_mem_->cls.report.bad) f.s(";class_bad_line=").d(sig_mem_->cls.report.bad_line);
            }
            f.s(";evictions=").d(tracker_.evictions()).s(";requeues=").d(tracker_.requeues());
            put_pos_src(f);
        }
        log_line(row);
    }
    /* HB3 (a class with SCAN lines): the interference scan and its hum
     * dwells: windows, passes, hum dwells, checks judged, mains found, dwells
     * without a result, starts deferred for a busy Wi-Fi slot (M4), ladder
     * windows (M4), hum airtime ms. */
    if (scan_on()) {
        char row[300];
        Fmt f{row, sizeof(row)};
        row_begin(f, "HB3", "");
        f.s(",,").s(sig_name_.c_str(), Fmt::Esc::Csv).s(",,,,,,");
        if (own)
            f.e7(lat).c(',').e7(lon);
        else
            f.c(',');
        f.s(",scan_win=").d(static_cast<int32_t>(scan_windows_)).s(";scan_pass=").d(static_cast<int32_t>(scan_passes_));
        f.s(";hum_dwells=").d(hum_dwells_).s(";hum_checks=").d(hum_checks_).s(";hum_found=").d(hum_found_).s(";hum_lost=").d(hum_lost_);
        f.s(";hum_deferred=").d(m.hum_deferred).s(";hum_windows=").d(m.hum_windows).s(";hum_ms=").d(static_cast<int32_t>(hum_ms_));
        /* pass_ms: the last complete pass, window 0 to window 0 (hum dwells
         * included); floor_full / emit_full: floor rises and new emitters the
         * engine's tables had no slot for; hum_clipped: checks with the ADC
         * saturated (no verdict). */
        f.s(";pass_ms=").d(static_cast<int32_t>(pass_ms_)).s(";hum_clipped=").d(hum_clipped_);
        if (scan_) f.s(";floor_full=").d(static_cast<int32_t>(scan_->floor_full())).s(";emit_full=").d(static_cast<int32_t>(scan_->emit_full()));
        put_pos_src(f);
        log_line(row);
    }
}

/* Debug console / USB raw line for every frame (bring-up tool). */
void SignatureDetectView::raw_line(const SignatureDetectRidMessage& message, bool odid) {
    const size_t adv_len = message.adv_len <= SignatureDetectRidMessage::ADV_MAX ? message.adv_len : SignatureDetectRidMessage::ADV_MAX;
#if SIGDET_FULL
    if (debug_) {
        /* "37 AABBCCDDEEFF -45 1 #12": ODID message type nibble and counter. */
        char line[40];
        Fmt f{line, sizeof(line)};
        f.d(message.chan).c(' ').hex(message.addr, 6).c(' ').d((message.rssi_cdb - 50) / 100).c(' ');
        if (odid && adv_len > 6)
            f.c("0123456789ABCDEF"[message.adv[6] >> 4]).s(" #").d(message.adv[5]);
        else
            f.c('-');
        debug_->write(line);
    }
#else
    (void)odid;
#endif
    if (raw_usb_) {
        /* "DDRID,t_ms,chan,rssi_cdb,cfo_khz,pdu_type,MAC,ADVHEX" */
        char usb[120];
        Fmt f{usb, sizeof(usb)};
        f.s("DDRID,").d(message.t_ms).c(',').d(message.chan).c(',').d(message.rssi_cdb).c(',').d(message.cfo_khz).c(',');
        f.hex(&message.pdu_type, 1).c(',').hex(message.addr, 6).c(',').hex(message.adv, adv_len);
        usb_line(usb);
    }
}

void SignatureDetectView::refresh_status() {
    Line& l0 = status_lines_[0];
    Fmt f{l0.text, sizeof(l0.text)};
    if (phase_band_ == kBle)
        f.s("BLE").d(channel_);
    else if (phase_band_ == kWifi)
        f.s("WiFi").d(tune::kWifiChannels[wifi_cursor_].number).c(wifi_slice_ < 0 ? '-' : '+');
    else if (phase_band_ == kPat)
        f.s("P").d(pat_table_.entry(pat_entry_).ch_khz[pat_ch_] / 1000);
    else if (phase_band_ == kDid)
        f.s("DJI").d(tune::kDidChannelsKhz[did_cursor_] / 1000);
    else if (phase_band_ == kTrack)
        f.s("TRACK");
    else if (phase_band_ == kZw)
        f.s("ZW").d(sig_ready() ? sig_mem_->cls.zw.f_khz / 1000 : 908);
    else if (phase_band_ == kScan)
        f.s("SCN").d(scan_w_).c('p').d(scan_ ? static_cast<int32_t>(scan_->pass()) : 0); /* window, pass */
    else if (phase_band_ == kHum)
        f.s("HUM").d(static_cast<int32_t>(hum_khz_ / 1000)).c('w').d(hum_win_); /* centre MHz, windows so far */
    else
        f.s(kBands[phase_band_].name);
    f.s(" RID ").d(tracker_.active_rid()).c(' ');
    int32_t lat, lon;
    if (gps_fix())
        f.s("GPS ").d(now_s() - gps_s_).c('s');
    else if (gps_nofix()) /* a GPS is there, searching: say so, and what is used instead */
        f.s(own_position(lat, lon) ? "NoFix+Man" : "GPS NoFix");
    else if (own_position(lat, lon))
        f.s("ManPos");
    else
        f.s("NoPos");
    if (sweeps_on()) f.s(" cyc ").d1(cycle_ms_ / 100).c('s');
    l0.color = Color::white();

    Line& l1 = status_lines_[1];
    Fmt g{l1.text, sizeof(l1.text)};
    /* The sample clock could not be set (CLK?): the receive modes run on
     * a wrong rate until a later switch succeeds. I2C ERR: a bus fault in
     * the last 10 s, or the bus did not come back from one. */
    const bool clk_bad = !portapack::clock_manager.sampling_frequency_ok();
    const auto& i2c = portapack::i2c0.stats();
    const bool i2c_bad = !i2c.bus_ok || (i2c.faults && chTimeNow() - i2c.last_fault_time < 10 * CH_FREQUENCY);
    if (clk_bad) g.s("CLK? ");
    /* The ADC saturated in the last 10 s (a hum window or the track meter):
     * levels and the hum check are unreliable; lower LNA / VGA / amp by hand
     * (the app never changes them). */
    const bool clip = clip_s_ && now_s() - clip_s_ < 10;
    if (clip) g.s("CLIP ");
    if (i2c_bad) g.s("I2C ERR ");
    const bool log_bad = log_on() && log_error_[0];
    if (log_bad) g.s("LOG! ").s(log_error_).c(' '); /* why the Log box does not log (a new file is tried every 30 s) */
    /* The class file: missing, or a line it skipped (Debug view: why). */
    const bool cls_bad = sig_file_ && sig_loaded_ && (sig_load_failed_ || (sig_mem_ && sig_mem_->cls.report.bad));
    if (cls_bad) {
        if (sig_load_failed_)
            g.s("SIG: no file ");
        else
            g.s("SIG ln").d(sig_mem_->cls.report.bad_line).s("? ");
    }
    if (map_nomem_ && now_s() - map_nomem_s_ > 5) map_nomem_ = false;
    /* open_map() / Set on map refused (map_fits). What is left to free: the
     * CSV log (0.6 KB and its file) - say so. */
    if (map_nomem_) g.s(log_ ? "MAP: NO MEM, Log off? " : "MAP: NO MEM ");
    static constexpr const char* esp_names[] = {"off", "no module", "starting", "no app", "on ", "lost", "bus err"};
    const auto st = esp_.state();
    /* Nothing while the feed is switched off (Setup): only a feed that is on,
     * or wanted and not working, is worth the room. */
    if (st != EspFeed::State::Off) {
        g.s("ESP ").s(esp_names[static_cast<size_t>(st)]);
        if (st == EspFeed::State::Active) g.d(esp_.count());
        g.c(' ');
    }
    /* BLE dwell drops only: buffers skipped while a sweep FFTs or the rate
     * switches lose no Remote ID (debug view: all of them). */
    g.s("M4 ").d(stats_.cpu_pct).s("% drop ").d(stats_.drops_ble);
    if (log_on() && !log_bad) g.s(log_ ? (survey_on() ? " SRV" : " LOG") : " LOG...");
    if (opt_.json) g.s(" JSON");
    if (demo_on_) g.s(" DEMO");
    l1.color = (log_bad || map_nomem_ || clk_bad || i2c_bad) ? Color::red() : ((demo_on_ || cls_bad || clip) ? Color::yellow() : Color::white());
    text_status_.set_dirty();
}

/* ---------------------------------------------------------------- map */

void SignatureDetectView::open_map(Entity::Key key) {
    if (map_ || !map_fits()) return;
    const Entity* e = tracker_.find_key(key);
    int32_t own_lat = 0, own_lon = 0;
    if (!e && !own_position(own_lat, own_lon)) {
        /* Main view, own position unknown: the newest drone with a position.
         * With an own position (GPS or set by hand) the map centres on that
         * and every drone is a marker: a drone far away (or a test beacon
         * at 0/0) must not carry the map off the user's area. */
        for (const auto& x : tracker_)
            if (x.rid && odid_pos_valid(x.rid->lat_e7, x.rid->lon_e7)) {
                e = &x;
                break;
            }
    }
    int32_t lat = 0, lon = 0;
    char tag[12] = "";
    if (e && e->rid && odid_pos_valid(e->rid->lat_e7, e->rid->lon_e7)) {
        lat = e->rid->lat_e7;
        lon = e->rid->lon_e7;
    } else if (e && e->rid && odid_pos_valid(e->rid->op_lat_e7, e->rid->op_lon_e7)) {
        lat = e->rid->op_lat_e7;
        lon = e->rid->op_lon_e7;
    } else {
        own_position(lat, lon);
    }
    if (e) entity_name(*e, tag, sizeof(tag));
    map_key_ = e ? e->id : Entity::invalid_key;

    /* Without map files GeoMap draws a grid; the markers still show. */
    /* Sweeps and DroneID dwells pause: M0 time for drawing, and the map
     * shows Remote ID. */
    pause_rf(true);
    map_ = nav_.push<SigdetMapView>(
        std::string{tag}, 0, GeoPos::alt_unit::METERS, GeoPos::spd_unit::KMPH,
        deg(lat), deg(lon), 0,
        [this]() {
            map_ = nullptr;
            pause_rf(false);
        });
    refresh_map();
}

void SignatureDetectView::refresh_map() {
    if (!map_) return;
    int32_t lat, lon;
    /* No own position (any more, e.g. the GPS lost its fix and none is set by
     * hand): take the marker off instead of leaving it where it was. The
     * altitude is the GPS's only with a fix (not a stale one for ManPos). */
    if (own_position(lat, lon))
        map_->update_my_position(deg(lat), deg(lon), gps_fix() ? gps_alt_m_ : 0);
    else
        map_->update_my_position(INVALID_LAT_LON, INVALID_LAT_LON, 0);

    map_->clear_markers();
    GeoMarker marker{};
    for (const auto& x : tracker_) {
        if (!x.rid) continue;
        const RidRecord& r = *x.rid;
        const bool loc = r.msg_seen & MSG_LOCATION;
        const uint16_t dir = (loc && odid_direction_valid(r)) ? odid_direction_deg(r) % 360 : 0;
        if (odid_pos_valid(r.lat_e7, r.lon_e7)) {
            if (x.id == map_key_) {
                map_->update_position(
                    deg(r.lat_e7), deg(r.lon_e7), dir,
                    (loc && odid_alt_valid(r.alt_geo_raw)) ? odid_alt_dm(r.alt_geo_raw) / 10 : 0,
                    (loc && odid_speed_h_valid(r)) ? odid_speed_h_cms(r) * 36 / 1000 : 0);
            } else {
                char tag[10];
                entity_name(x, tag, sizeof(tag));
                marker.lat = deg(r.lat_e7);
                marker.lon = deg(r.lon_e7);
                marker.angle = dir;
                marker.tag = tag;
                marker.color = Color::orange();
                map_->store_marker(marker);
            }
        }
        if (odid_pos_valid(r.op_lat_e7, r.op_lon_e7)) {
            marker.lat = deg(r.op_lat_e7);
            marker.lon = deg(r.op_lon_e7);
            marker.angle = 0;
            marker.tag = x.id == map_key_ ? "OP" : "op";
            marker.color = Color::yellow();
            map_->store_marker(marker);
        }
    }
}

/* ---------------------------------------------------------------- detail */

DetailView::DetailView(NavigationView& nav, SignatureDetectView& main, Entity::Key key)
    : main_{main}, key_{key} {
    add_children({&text_, &button_map_, &button_track_, &button_cap_, &button_back_});
    /* Capture this detection's signal: Signature Detect closes, the capture view
     * opens preset to it. */
    button_cap_.on_select = [this, &nav](Button&) {
        auto& main = main_;
        auto& n = nav;
        const Entity* e = main_.tracker_.find_key(key_);
        CaptureContext c{};
        if (!e || !main_.capture_of(*e, c)) return;
        n.pop();
        main.capture(c);
    };
    /* Track this detection (its frequency, its BLE address or rule): this
     * view goes first (heap), the track view then returns to the list. */
    button_track_.on_select = [this, &nav](Button&) {
        auto& main = main_;
        auto& n = nav;
        const Entity* e = main_.tracker_.find_key(key_);
        TrackTarget t{};
        const bool ok = e && main_.target_of(*e, t);
        if (!ok) return;
        n.pop();
        main.open_track(t);
    };
    button_map_.on_select = [this, &nav](Button&) {
        /* The map needs M0 heap (kMapHeapExtra): free this view first (like
         * nav.replace), the map then returns to the list. Nothing of this
         * view may be touched after the pop. */
        auto& main = main_;
        auto& n = nav;
        const Entity::Key key = key_;
        n.pop();
        main.open_map(key);
    };
    button_back_.on_select = [&nav](Button&) {
        nav.pop();
    };
    refresh();
}

DetailView::~DetailView() {
    main_.detail_ = nullptr;
}

void DetailView::focus() {
    button_map_.focus();
}

void DetailView::refresh() {
    const Entity* e = main_.tracker_.find_key(key_);
    if (!e) {
        /* Expired: keep the last values on screen. */
        Line& l = lines_[10];
        Fmt{l.text, sizeof(l.text)}.s("Lost (expired)");
        l.color = Color::red();
        text_.set_dirty();
        return;
    }

    for (auto& l : lines_) {
        l.text[0] = 0;
        l.color = Color::white();
    }
    auto line = [this](size_t i) { return Fmt{lines_[i].text, sizeof(lines_[i].text)}; };

    char name[24];
    entity_name(*e, name, sizeof(name));
    const RidRecord* r = e->rid;
    {
        Fmt f = line(0);
        f.s(name);
        if (r && (r->msg_seen & MSG_BASIC)) f.c(' ').s(pick(kIdTypes, 5, r->id_type));
    }

    if (r) {
        const bool loc = r->msg_seen & MSG_LOCATION;
        const bool sys = r->msg_seen & MSG_SYSTEM;
        const bool pos = odid_pos_valid(r->lat_e7, r->lon_e7);
        const bool op = odid_pos_valid(r->op_lat_e7, r->op_lon_e7);
        {
            Fmt f = line(1);
            f.s(kUaTypes[r->ua_type]);
            if (loc) f.pad(14).s(pick(kStatus, 5, r->status));
        }
        auto position = [&](size_t i, const char* tag, bool valid, int32_t lat, int32_t lon) {
            Fmt f = line(i);
            f.s(tag);
            if (valid)
                f.e7(lat).c(' ').e7(lon);
            else
                f.s("no position");
        };
        position(2, "Dr ", pos, r->lat_e7, r->lon_e7);
        if (loc) {
            using I = int32_t;
            constexpr I D = Fmt::kDash;
            const bool h = odid_alt_valid(r->height_raw);
            const I v[] = {odid_alt_valid(r->alt_geo_raw) ? odid_alt_dm(r->alt_geo_raw) : D,
                           odid_alt_valid(r->alt_baro_raw) ? odid_alt_dm(r->alt_baro_raw) : D,
                           h ? odid_alt_dm(r->height_raw) : D,
                           odid_speed_h_valid(*r) ? odid_speed_h_cms(*r) : D,
                           odid_direction_valid(*r) ? I(odid_direction_deg(*r)) : D,
                           odid_speed_v_valid(*r) ? odid_speed_v_cms(*r) : D};
            const char* const str[] = {h ? (r->height_type ? "AGL" : "TO") : ""};
            const I* pv = v;
            const char* const* ps = str;
            line(4).t(line(3).t("Alt ~ B ~ H ~$\nSpd ^ Hdg # VS ^", pv, ps), pv, ps);
        }
        position(5, "Op ", op, r->op_lat_e7, r->op_lon_e7);
        if (sys || (op && pos)) {
            Fmt f = line(6);
            if (sys) {
                f.s(pick(kOpLoc, 4, r->op_loc_type));
                if (odid_alt_valid(r->op_alt_raw)) f.s(" alt ").d1(odid_alt_dm(r->op_alt_raw));
            }
            if (op && pos) f.s(" ->Dr ").dist(odid_distance_m(r->op_lat_e7, r->op_lon_e7, r->lat_e7, r->lon_e7));
        }
        if (r->operator_id[0]) line(7).s("OpID ").s(r->operator_id);
        if (r->self_id[0]) line(8).s("Self ").s(r->self_id);

        int32_t lat, lon;
        Fmt f = line(9);
        if (!main_.own_position(lat, lon)) {
            f.s(main_.gps_nofix() ? "GPS has no fix, no own pos" : "Own position unknown");
            lines_[9].color = Theme::getInstance()->fg_medium->foreground;
        } else {
            auto to = [&](const char* tag, int32_t la, int32_t lo) {
                f.s(tag).dist(odid_distance_m(lat, lon, la, lo)).c(' ').d(odid_bearing_deg(lat, lon, la, lo));
            };
            if (pos) to("Me>Dr ", r->lat_e7, r->lon_e7);
            if (op) to(" >Op ", r->op_lat_e7, r->op_lon_e7);
        }
    } else if (e->kind == Kind::Rf) {
        /* Class, centre, width, duty, flatness; SNR is the level line. */
        line(1).s("RF ").s(rf_class_tag(e->rf_cls)).s(" conf ").c("-LMH"[e->rf_conf & 3]);
#if SIGDET_FULL
        line(2).s("f ").d(e->freq_khz / 1000).c('.').d(e->freq_khz % 1000 / 100).s(" MHz bw ").d1(e->bw_khz / 100);
        if (e->rf_duty) line(3).s("duty ").d(e->rf_duty).s("% flat ").d1(e->rf_flat).s("dB");
#endif
        /* A '?' in the label: the rule that named it has never seen its real
         * emitter (RfGrade GRADE_DOC: FCC filings / source code, synthetic
         * tests only). */
        if (strchr(e->label, '?')) {
            line(4).s("?: from docs, simulation only");
            lines_[4].color = Theme::getInstance()->fg_medium->foreground;
        }
    } else if (e->kind == Kind::Pattern) {
        /* Class, channel, hit rate, the sync it matched. */
        line(1).s("Pattern scan ").s(e->cls).s(" (1M GFSK)");
        line(2).s("f ").d(e->freq_khz / 1000).c('.').d(e->freq_khz % 1000 / 100).s(" MHz  hits ").d1(e->rate10).s("/s");
        if (e->pat < main_.pat_table_.size()) {
            const PatternEntry& pe = main_.pat_table_.entry(e->pat);
            uint8_t sync[8];
            const size_t nbytes = (pe.sync_bits + 7) / 8;
            for (size_t i = 0; i < nbytes; i++) sync[i] = static_cast<uint8_t>(pe.sync >> (8 * (nbytes - 1 - i)));
            line(3).s("sync ").hex(sync, nbytes).c('/').d(pe.sync_bits).s(" e<=").d(pe.max_errors);
            Fmt f = line(4);
            for (size_t c = 0; c < pe.nch; c++) f.s(c ? " " : "ch ").d(pe.ch_khz[c] / 1000);
        }
        line(5).s("Presence only, no position");
        lines_[5].color = Theme::getInstance()->fg_medium->foreground;
    } else if (e->kind == Kind::BleVendor) {
        /* Name (model), why it matched, address type, PDUs seen. */
        line(1).s("DJI Bluetooth LE advert");
        line(2).s("Presence only, no position");
        lines_[2].color = Color::yellow();
        {
            Fmt f = line(3);
            f.s("Match ");
            if (e->why & sigdet::VENDOR_NAME) f.s("name ");
            if (e->why & sigdet::VENDOR_OUI) f.s("DJI-OUI ");
            if (e->why & sigdet::VENDOR_CID) f.s("DJI-CID");
        }
        {
            Fmt f = line(4);
            f.s((e->why & sigdet::VENDOR_OUI) ? "public addr" : "addr");
            static constexpr const char* kPdu[8] = {" ADV", "", " NONCONN", "", " SCAN_RSP", "", " SCAN_IND", ""};
            for (size_t i = 0; i < 8; i++)
                if (e->pdus & (1u << i)) f.s(kPdu[i]);
        }
        line(5).s("frames ").d(e->frames).s(" first ").d(main_.now_s() - e->first_s).s("s ago");
        if (!(e->why & sigdet::VENDOR_NAME)) {
            line(6).s("No DJI name: may be a DJI");
            line(7).s("remote, camera or gimbal");
        }
    } else if (e->kind == Kind::DroneId) {
        /* Last burst: centre, layout, scores. Then what the detection is
         * worth: checked on recordings only (the '?' of the name). */
        line(1).s("DJI DroneID burst (OcuSync)");
        line(2).s("Presence only, no position");
        lines_[2].color = Color::yellow();
        line(3).s("f ").d(e->freq_khz / 1000).c('.').d(e->freq_khz % 1000 / 100).s(" MHz  bursts ").d(e->frames);
        line(4).d(e->did_nsym).s(e->did_nsym == 9 ? " sym (O2)" : " sym (O1/O4)").s(" ZC ").d(e->did_zc).s("/1000");
        line(5).s("SNR ").d1(e->did_snr / 10).s(" dB  CFO ").d(e->did_cfo).s(" Hz");
        line(6).s("?: validated on recordings");
        line(7).s("(PC: 26/26 real bursts); the");
        line(8).s("port: replays only, never live");
        line(9).s("No decoding (O4: encrypted)");
        for (size_t i = 6; i < 10; i++) lines_[i].color = Theme::getInstance()->fg_medium->foreground;
    } else if (e->kind == Kind::Zwave) {
        /* Network: home ID (the label), nodes, frames; validity of the rule. */
        const ZwNet* n = main_.sig_mem_ ? main_.sig_mem_->zw.find(e->mac + 2) : nullptr;
        line(1).s("Z-Wave network (R1 9.6 kb/s)");
        line(2).s("Presence only, frames decoded");
        lines_[2].color = Color::yellow();
        if (n) {
            line(3).s("nodes ").d(n->node_count()).s("  frames ").d(n->frames);
            Fmt f = line(4);
            f.s("ids");
            for (uint32_t id = 1; id < 256; id++)
                if (n->nodes[id >> 5] & (1u << (id & 31))) f.c(' ').d(static_cast<int32_t>(id));
        }
        if (main_.sig_ready()) line(5).s("valid: ").s(sig_valid_name(main_.sig_mem_->cls.zw.valid)).s("  ").d(main_.sig_mem_->cls.zw.f_khz / 1000).c('.').d(main_.sig_mem_->cls.zw.f_khz % 1000 / 10).s(" MHz");
        line(6).s("Header fields only (log)");
        line(7).s("first ").d(main_.now_s() - e->first_s).s("s ago");
        lines_[6].color = Theme::getInstance()->fg_medium->foreground;
    } else if (e->kind == Kind::Fsk) {
        /* The rule: frequency, rate, what the frames showed, validity. */
        const SigClass* c = main_.sig_ready() ? &main_.sig_mem_->cls : nullptr;
        const uint8_t r = e->mac[4];
        line(1).s("FSK rule ").s(main_.sig_name_.c_str());
        if (c && r < c->nfsk) {
            const FskRuleDef& d = c->fsk[r];
            const FskSeen& x = main_.sig_mem_->fsk[r];
            line(2).s(x.checked ? "Check passed: presence" : "Sync + length only: low conf");
            lines_[2].color = Color::yellow();
            line(3).d(static_cast<int32_t>(d.f_khz / 1000)).c('.').d2(static_cast<int32_t>(d.f_khz % 1000 / 10)).s(" MHz ").d(static_cast<int32_t>(d.rate)).s(" b/s");
            line(4).s("frames ").d(static_cast<int32_t>(x.frames)).s("  len ").d(x.len);
            line(5).s("dev ").d1(x.dev_hz / 100).s("k off ").d1(x.off_hz / 100).s("k");
            line(6).s("rate ").d(static_cast<int32_t>(x.rate_bps)).s(" valid: ").s(sig_valid_name(d.valid));
        }
        line(7).s("first ").d(main_.now_s() - e->first_s).s("s ago");
    } else if (e->kind == Kind::Scan) {
        /* What the scan found and why it counts as interference. */
        line(1).s("Scan ").s(main_.sig_name_.c_str()).c(' ').s(scan_type_tag(e->rf_cls)).s(" conf ").c("-LMH"[e->rf_conf & 3]);
        static constexpr const char* kWhy[] = {"", "Regular lines (comb)", "Comb, not always on", "Noise floor up", "Always on, centre moves",
                                               "Modulated by mains"};
        line(2).s(e->rf_cls < sizeof(kWhy) / sizeof(kWhy[0]) ? kWhy[e->rf_cls] : "");
        lines_[2].color = Color::yellow();
        const uint32_t lo = e->freq_khz - e->bw_khz / 2;
        line(3).d(static_cast<int32_t>(lo / 1000)).c('.').d2(static_cast<int32_t>(lo % 1000 / 10)).s(" - ").d(static_cast<int32_t>((lo + e->bw_khz) / 1000)).c('.').d2(static_cast<int32_t>((lo + e->bw_khz) % 1000 / 10)).s(" MHz");
        line(4).s(e->label);
        line(5).s("Energy sweep: type of signal,");
        line(6).s("not what device sends it");
        for (size_t i = 5; i < 7; i++) lines_[i].color = Theme::getInstance()->fg_medium->foreground;
        line(8).s("first ").d(main_.now_s() - e->first_s).s("s ago");
    } else if (e->kind == Kind::Sig) {
        /* Class, kind, source, the rule and how it was validated. */
        const bool nb = (e->sources & SRC_RF) != 0;
        const SigClass* c = main_.sig_ready() ? &main_.sig_mem_->cls : nullptr;
        line(1).s("Sig ").s(main_.sig_name_.c_str()).c(' ').s(sig_kind_name(e->pat));
        line(2).s(nb ? "NB energy (sweep), low conf" : "BLE advert, presence only");
        lines_[2].color = Color::yellow();
        if (c && nb && e->why < c->nnb) {
            const NbRule& r = c->nb[e->why];
            const NbResult& res = main_.sig_mem_->nb_last[e->why];
            const uint32_t fr = r.f_khz % 1000;
            line(3).s("f ").d(r.f_khz / 1000).c('.').s(fr < 100 ? (fr < 10 ? "00" : "0") : "").d(fr).s(" bw ").d(r.bw_khz).s("k ").s(kBands[r.band].name);
            line(4).s("duty ").d(res.duty).s("% burst ").d(res.burst_ms).s(" per ").d(res.period_ms).s("ms");
            line(5).s("valid: ").s(sig_valid_name(r.valid));
            line(6).s("?: energy at this frequency");
            line(7).s("only; protocol not checked");
            for (size_t i = 6; i < 8; i++) lines_[i].color = Theme::getInstance()->fg_medium->foreground;
        } else if (c && !nb && e->why < c->nble) {
            const BleRule& r = c->ble[e->why];
            Fmt f = line(3);
            f.s("rule ").d(e->why);
            if (r.flags & BR_CID) {
                const uint8_t cid[2] = {static_cast<uint8_t>(r.cid >> 8), static_cast<uint8_t>(r.cid)};
                f.s(" cid ").hex(cid, 2);
            }
            if (r.flags & (BR_UUID | BR_SDATA)) {
                const uint8_t u[2] = {static_cast<uint8_t>(r.uuid >> 8), static_cast<uint8_t>(r.uuid)};
                f.s((r.flags & BR_SDATA) ? " sd " : " uuid ").hex(u, 2);
            }
            line(4).s("valid: ").s(sig_valid_name(r.valid));
        }
        line(8).s("frames ").d(e->frames).s(" first ").d(main_.now_s() - e->first_s).s("s ago");
    } else {
        line(1).s("Drone maker WiFi (ESP32)");
    }

    {
        Fmt f = line(10);
        f.s(e->kind == Kind::Rf ? "SNR " : "RSSI ");
        if (e->rssi != kNoRssi) {
            f.d(e->rssi).s(e->rssi_dbm ? "dBm" : "").s(" max ").d(e->rssi_max).c(' ').c(e->trend());
        } else {
            f.c('-');
        }
        f.s(" age ").d(main_.now_s() - e->last_s).c('s');
        if (e->stale) lines_[10].color = Theme::getInstance()->fg_medium->foreground;
    }
    {
        Fmt f = line(11);
        if (e->kind == Kind::Pattern)
            f.s("table entry ").d(e->pat).s(!main_.pat_table_.has_builtin() ? " (class file)" : (e->pat ? " (PATTERNS.TXT)" : " (built-in)"));
        else if (e->kind == Kind::Sig && (e->sources & SRC_RF))
            f.s("NB rule ").d(e->why).c(' ');
        else
            f.hex(e->mac, 6).c(' ');
        if (e->kind != Kind::Pattern) put_sources(f, e->sources, 4);
        if (r) {
            f.c(' ');
            static constexpr char kMsgLetters[] = "BLASYO";
            for (size_t i = 0; i < 6; i++) f.c((r->msg_seen & (1u << i)) ? kMsgLetters[i] : '-');
        }
    }
    text_.set_dirty();
}

/* ---------------------------------------------------------------- Z-Wave */

/* An FSK dwell (PSDT MODE_FSK): the class's next FSK group (frequency,
 * front end, its rules; Z-Wave is one), 4 Msps, the channel at +fs/4 as for
 * BLE. One group a visit, in turn. keep: the dwell before was the same lone
 * group (a class with nothing else): the receiver runs on, unconfigured. */
void SignatureDetectView::start_zw(bool rate_switch, bool keep) {
    phase_band_ = kZw;
    const uint8_t groups = sig_ready() ? sig_mem_->cls.nfskg : 0;
    const bool alone = (mode() & 2) && !sweeps_on() && !patterns_on() && !wifi_on(); /* no BLE, nothing else */
    phase_dwell_ms_ = alone ? (groups > 1 ? 2500 : 5000) : kZwDwellMs;
    if (keep && groups <= 1) return;
    if (groups) fsk_cursor_ = static_cast<uint8_t>((fsk_cursor_ + 1) % groups);
    if (rate_switch || !portapack::clock_manager.sampling_frequency_ok()) { /* see start_ble() */
        receiver_model.set_sampling_rate(sampling_rate);
        if (sweep_) sweep_->invalidate_rates();
    }
    fsk_radio(fsk_cursor_);
}

/* Tune to FSK group g and hand its plan to the M4 (it copies it). */
void SignatureDetectView::fsk_radio(uint8_t g) {
    if (!sig_ready() || g >= sig_mem_->cls.nfskg) {
        baseband::set_signature_detect_fsk(sampling_rate, 0, 0, nullptr);
        return;
    }
    const FskGroup& x = sig_mem_->cls.fskg[g];
    receiver_model.set_target_frequency(static_cast<rf::Frequency>(x.f_khz) * 1000);
    const uint8_t flags = reset_pending_ ? SignatureDetectConfigMessage::FLAG_RESET_STATS : 0;
    reset_pending_ = false;
    baseband::set_signature_detect_fsk(sampling_rate, g, flags, &x.dwell);
}

/* A frame from the M4: the Z-Wave rule's go to on_zwave (header checks,
 * networks); any other rule's: listed by label once it has min= frames in
 * kFskWindowMs (a rule with a check: every frame passed it; without one:
 * sync and length only, shown as low confidence, '?'). FSK_NEW once a
 * session, FSK_FRM (at most one a second a rule), FSK_SUM when it stops. */
void SignatureDetectView::on_fsk(const SignatureDetectFskMessage& message) {
    fsk_rx_++;
    if (!sig_ready() || message.tag >= sig_mem_->cls.nfsk) return;
    const FskRuleDef& d = sig_mem_->cls.fsk[message.tag];
    if (d.post == FSK_POST_ZWAVE) {
        on_zwave(message);
        return;
    }
    if (track_ && phase_band_ == kTrack) {
        const TrackTarget& t = track_->target();
        if (t.kind == TrackTarget::FSK_RULE && t.rule == message.tag) track_->on_ble(message.rssi_cdb);
    }
    constexpr uint32_t kFskWindowMs = 60000;
    FskSeen& x = sig_mem_->fsk[message.tag];
    const uint32_t now = uptime_ms();
    if (x.n_win && now - x.t_win > kFskWindowMs) x.n_win = 0;
    if (!x.n_win) x.t_win = now;
    if (x.n_win < 255) x.n_win++;
    x.frames++;
    x.t_last = now;
    x.len = message.len;
    x.rssi_cdb = message.rssi_cdb;
    x.off_hz = message.off_hz;
    x.dev_hz = message.dev_hz;
    x.rate_bps = message.rate_bps;
    x.checked = message.flags & SignatureDetectFskMessage::FLAG_CHECKED;
    const bool confirmed_now = !x.listed && x.n_win >= d.min_frames;
    if (confirmed_now) x.listed = true;
    if (!x.listed) return;
    const uint8_t mac[6] = {0, 0, 'F', 'S', message.tag, 0};
    char label[24];
    Fmt{label, sizeof(label)}.s(d.label).s(x.checked ? "" : "?");
    const int32_t db = message.rssi_cdb / 100;
    if (tracker_.ingest_sig(mac, SRC_RF, static_cast<int8_t>(db < -127 ? -127 : db), message.tag, d.kind, sig_kind_tag(d.kind), label, now_s(), Kind::Fsk))
        list_.set_dirty();
    if (!x.reported) {
        x.reported = true;
        fsk_row("FSK_NEW", message.tag, &message);
    }
    if (now - x.t_row >= 1000 || x.frames == 1) {
        fsk_row("FSK_FRM", message.tag, &message);
        x.t_row = now;
        x.skipped = 0;
    } else if (x.skipped < 0xFFFF) {
        x.skipped++;
    }
    if (opt_.alert && confirmed_now && opt_.beep_low) beep();
}

/* FSK_NEW / FSK_FRM / FSK_SUM: id = F<rule>, label, kind, f kHz, bw kHz,
 * level dBFS, own position; detail: rule, frame length, check (ok or none),
 * measured deviation, carrier offset and bit rate, Manchester violations, the
 * frame's first <= 8 bytes in hex (device identifiers: the user's own log,
 * as BLE addresses; never shared unscrubbed), frames so far. */
void SignatureDetectView::fsk_row(const char* event, uint8_t rule, const SignatureDetectFskMessage* m) {
    if (!log_on() || !sig_ready() || rule >= sig_mem_->cls.nfsk) return;
    const FskRuleDef& d = sig_mem_->cls.fsk[rule];
    const FskSeen& x = sig_mem_->fsk[rule];
    char row[256];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "F");
    f.s(",F").d(rule + 1).c(',').s(d.label, Fmt::Esc::Csv).c(',').s(sig_kind_name(d.kind));
    f.c(',').d(static_cast<int32_t>(d.f_khz)).c(',').d(d.bw_khz).c(',').d((m ? m->rssi_cdb : x.rssi_cdb) / 100).s(",dBFS,");
    int32_t lat, lon;
    if (own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    f.c(',');
    f.s("rule=").d(rule + 1).s(";valid=").s(sig_valid_name(d.valid));
    if (m) {
        f.s(";len=").d(m->len).s(";check=").s((m->flags & SignatureDetectFskMessage::FLAG_CHECKED) ? "ok" : "none");
        f.s(";dev_khz=").d1(m->dev_hz / 100).s(";off_khz=").d1(m->off_hz / 100).s(";rate=").d(static_cast<int32_t>(m->rate_bps));
        f.s(";viol=").d(m->violations).s(";hex=").hex(m->hdr, m->nhdr < 8 ? m->nhdr : 8);
        if (x.skipped) f.s(";skipped=").d(x.skipped);
    }
    f.s(";frames=").d(static_cast<int32_t>(x.frames));
    put_pos_src(f);
    log_line(row);
}

/* A checksum-valid frame from the M4: header sanity (zw_parse), the network
 * table (listed from its 2nd valid frame: kZwConfirmMs), a list entry keyed
 * by home ID ("ZW <home ID>": the user's own log and screen, as BLE
 * addresses), ZW_NEW once per network, ZW_FRM per frame. No payload. */
void SignatureDetectView::on_zwave(const SignatureDetectFskMessage& message) {
    zw_rx_++;
    ZwHeader h;
    if (!zw_parse(message.hdr, message.nhdr, message.len, h)) {
        zw_bad_++;
        return;
    }
    if (track_ && phase_band_ == kTrack) {
        const TrackTarget& t = track_->target();
        if (t.kind == TrackTarget::ZW_NET && memcmp(t.addr, h.home, 4) == 0) track_->on_ble(message.rssi_cdb);
    }
    if (!sig_mem_) return;
    bool confirmed_now = false;
    ZwNet* n = sig_mem_->zw.add(h, message.rssi_cdb, uptime_ms(), confirmed_now);
    if (!n || !n->confirmed) return;
    zw_ok_s_ = now_s(); /* a listed network's frame: the Z-Wave energy rule may list (nb_tied_to_zw) */
    const uint8_t mac[6] = {'Z', 'W', h.home[0], h.home[1], h.home[2], h.home[3]};
    char label[24];
    Fmt{label, sizeof(label)}.s("ZW ").hex(h.home, 4);
    const uint8_t kind = sig_ready() ? sig_mem_->cls.zw.kind : static_cast<uint8_t>(SK_HOME);
    const int32_t db = message.rssi_cdb / 100;
    if (tracker_.ingest_sig(mac, SRC_BLE, static_cast<int8_t>(db < -127 ? -127 : db), n->node_count(), kind, sig_kind_tag(kind), label, now_s(),
                            Kind::Zwave))
        list_.set_dirty();
    if (!n->reported) {
        n->reported = true;
        zw_row("ZW_NEW", *n, &h, message.rssi_cdb);
    }
    zw_row("ZW_FRM", *n, &h, message.rssi_cdb);
    if (opt_.alert && confirmed_now && opt_.beep_low) beep();
}

/* ZW_NEW / ZW_FRM / ZW_SUM: id = home ID (hex), label, kind, 908420 kHz,
 * level dBFS, own position; detail: header fields of the frame (src, dst,
 * type, routed, ackreq, seq, len, cc, repeaters, hop) and the network's
 * nodes and frames. */
void SignatureDetectView::zw_row(const char* event, const ZwNet& n, const ZwHeader* h, int16_t rssi_cdb) {
    if (!log_on()) return;
    char row[320];
    Fmt f{row, sizeof(row)};
    row_begin(f, event, "Z");
    const uint8_t kind = sig_ready() ? sig_mem_->cls.zw.kind : static_cast<uint8_t>(SK_HOME);
    f.c(',').hex(n.home, 4).c(',').s(sig_ready() ? sig_mem_->cls.zw.label : "Z-Wave", Fmt::Esc::Csv).c(',').s(sig_kind_name(kind));
    f.c(',').d(sig_ready() ? sig_mem_->cls.zw.f_khz : 908420).s(",200,").d(rssi_cdb / 100).s(",dBFS,");
    int32_t lat, lon;
    if (own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    f.c(',');
    if (h) {
        f.s("src=").d(h->src).s(";dst=").d(h->dst).s(";type=").s(zw_type_name(h->type)).s(";routed=").d(h->routed);
        f.s(";ackreq=").d(h->ackreq).s(";seq=").d(h->seq).s(";len=").d(h->len);
        if (h->cc >= 0) {
            const uint8_t cc = static_cast<uint8_t>(h->cc);
            f.s(";cc=").hex(&cc, 1);
        }
        if (h->routed) {
            f.s(";rep=");
            for (size_t i = 0; i < h->nrep && i < 4; i++) f.s(i ? "/" : "").d(h->rep[i]);
            f.s(";hop=").d(h->hop);
        }
        f.c(';');
    }
    f.s("nodes=").d(n.node_count()).s(";frames=").d(n.frames).s(";node_ids=");
    bool first = true;
    for (uint32_t id = 1; id < 256; id++)
        if (n.nodes[id >> 5] & (1u << (id & 31))) {
            if (!first) f.c('/');
            f.d(static_cast<int32_t>(id));
            first = false;
        }
    put_pos_src(f);
    log_line(row);
}

/* ---------------------------------------------------------------- track (fox hunt) */

/* What the track view locks onto for a detection: a frequency (sweep track,
 * NB rule, pattern channel, DroneID centre, Wi-Fi channel 6) or a BLE
 * advertiser (its address; for a signature rule hit also the rule, for
 * rotating addresses). */
bool SignatureDetectView::target_of(const Entity& e, TrackTarget& t) const {
    t = TrackTarget{};
    entity_name(e, t.label, sizeof(t.label));
    t.kind = TrackTarget::FREQ;
    switch (e.kind) {
        case Kind::Rf:
            t.f_khz = e.freq_khz;
            t.bw = static_cast<uint8_t>(track_bw_for(e.bw_khz));
            return e.freq_khz != 0;
        case Kind::Pattern:
        case Kind::DroneId:
            t.f_khz = e.freq_khz;
            t.bw = 0;
            return e.freq_khz != 0;
        case Kind::Scan:
            /* Walk to it with the meter: its centre, about its width; a comb:
             * one tooth (scan_event put it in freq_khz), a narrow channel. */
            t.f_khz = e.freq_khz;
            t.bw = static_cast<uint8_t>(track_bw_for((e.rf_cls == ST_COMB || e.rf_cls == ST_PCOMB) ? 40 : e.bw_khz));
            return e.freq_khz != 0;
        case Kind::Sig:
            if (e.sources & SRC_RF) {
                if (!sig_ready() || e.why >= sig_mem_->cls.nnb) return false;
                const NbRule& r = sig_mem_->cls.nb[e.why];
                t.f_khz = r.f_khz;
                t.bw = static_cast<uint8_t>(track_bw_for(r.bw_khz));
                return true;
            }
            t.kind = TrackTarget::BLE_ADDR;
            memcpy(t.addr, e.mac, 6);
            t.rule = e.why;
            return true;
        case Kind::Zwave:
            t.kind = TrackTarget::ZW_NET;
            memcpy(t.addr, e.mac + 2, 4);
            return true;
        case Kind::Fsk:
            if (!sig_ready() || e.mac[4] >= sig_mem_->cls.nfsk) return false;
            t.kind = TrackTarget::FSK_RULE;
            t.rule = e.mac[4];
            t.f_khz = sig_mem_->cls.fsk[t.rule].f_khz;
            return true;
        case Kind::BleVendor:
            t.kind = TrackTarget::BLE_ADDR;
            memcpy(t.addr, e.mac, 6);
            return true;
        case Kind::RemoteId:
            if (e.sources & SRC_BLE) {
                t.kind = TrackTarget::BLE_ADDR;
                memcpy(t.addr, e.mac, 6);
                return true;
            }
            if (e.sources & SRC_HACKRF_WIFI) {
                t.f_khz = tune::kWifiChannels[0].mhz * 1000u;
                t.bw = 0;
                return true;
            }
            return false;
        default:
            return false;
    }
}

/* The cycle pauses (the dwell in progress ends as at a switch), 4 Msps, and
 * the track view opens; track_end() resumes the cycle. Not while a map is
 * open; not when the heap cannot take the view (operator new would halt). */
void SignatureDetectView::open_track(const TrackTarget& t) {
    if (track_ || SigdetMapView::alive()) return;
    if (!heap_fits(sizeof(TrackView), 1024)) {
        map_nomem_ = true; /* the status line's "NO MEM" */
        map_nomem_s_ = now_s();
        refresh_status();
        return;
    }
    end_phase();
    phase_band_ = kTrack;
    phase_t0_ = chTimeNow();
    receiver_model.set_sampling_rate(sampling_rate);
    if (sweep_) sweep_->invalidate_rates();
    track_ = nav_.push<TrackView>(*this, t);
    refresh_status();
}

/* Radio and baseband for the target: MODE_METER at f (boxcar 2^log2d), or
 * BLE on one advertising channel, restricted to the address (FLAG_TARGET)
 * or every advert for the rule (pass-all, rate capped on the M4). */
void SignatureDetectView::track_radio(const TrackTarget& t, uint8_t ble_channel) {
    if (t.kind == TrackTarget::FREQ) {
        receiver_model.set_target_frequency(static_cast<rf::Frequency>(t.f_khz) * 1000);
        baseband::set_signature_detect(sampling_rate, SignatureDetectConfigMessage::MODE_METER, kTrackBwLog2[t.bw < kTrackBws ? t.bw : 0], 0);
        return;
    }
    if (t.kind == TrackTarget::ZW_NET || t.kind == TrackTarget::FSK_RULE) {
        /* the rule's dwell (Z-Wave: the ZW rule's) */
        const int r = t.kind == TrackTarget::ZW_NET ? (sig_ready() ? sig_mem_->cls.zw_rule : -1) : t.rule;
        fsk_radio(sig_ready() && r >= 0 && r < sig_mem_->cls.nfsk ? sig_mem_->cls.fsk[r].group : 0xFF);
        return;
    }
    receiver_model.set_target_frequency(ble_channel_frequency(ble_channel));
    if (t.kind == TrackTarget::BLE_ADDR) {
        uint64_t a = 0;
        for (size_t i = 0; i < 6; i++) a = (a << 8) | t.addr[i];
        baseband::set_signature_detect_target(ble_channel, 0, a);
    } else {
        baseband::set_signature_detect(sampling_rate, SignatureDetectConfigMessage::MODE_GFSK_BLE, ble_channel, SignatureDetectConfigMessage::FLAG_PASS_ALL);
    }
}

/* The track view closed: the cycle resumes (a BLE dwell first). */
void SignatureDetectView::track_end() {
    track_ = nullptr;
    if (phase_band_ != kTrack) return;
    phase_t0_ = chTimeNow();
    next_phase();
}

/* Hand-off to the stock Audio app in AM at f (to hear a pulsed source, e.g.
 * the comb's 10 ticks a second), as Looking Glass and Recon do; this app
 * closes (its view goes first, then Audio starts its own baseband). */
void SignatureDetectView::listen(rf::Frequency f) {
    auto settings = receiver_model.settings();
    settings.frequency_app_override = f;
    settings.frequency_step = 1000;
    settings.mode = ReceiverModel::Mode::AMAudio;
    nav_.replace<AnalogAudioView>(settings);
}

void SignatureDetectView::on_meter(const SignatureDetectMeterMessage& message) {
    if (track_) track_->on_meter(message);
}

namespace {

/* Seven segment digit (ch '0'..'9' or '-') in w x h at p, stroke t. */
void draw_digit(Painter& painter, Point p, int w, int h, int t, char ch, Color c) {
    static constexpr uint8_t kSeg[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
    const uint8_t seg = ch == '-' ? 0x40 : (ch >= '0' && ch <= '9' ? kSeg[ch - '0'] : 0);
    const int x = p.x(), y = p.y(), hh = h / 2;
    const Rect r[7] = {{x, y, w, t}, {x + w - t, y, t, hh}, {x + w - t, y + hh, t, hh}, {x, y + h - t, w, t}, {x, y + hh, t, hh}, {x, y, t, hh}, {x, y + hh - t / 2, w, t}};
    for (size_t i = 0; i < 7; i++)
        if (seg & (1u << i)) painter.fill_rectangle(r[i], c);
}

constexpr int32_t kScaleLoCdb = -11000; /* meter scale: -110 .. -10 dBFS */
constexpr int32_t kScaleSpanCdb = 10000;

int scale_x(int32_t cdb, int width) {
    const int32_t v = (cdb - kScaleLoCdb) * width / kScaleSpanCdb;
    return v < 0 ? 0 : (v > width ? width : static_cast<int>(v));
}

}  // namespace

void TrackLevel::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto& st = style();
    painter.fill_rectangle(r, st.background);
    const int16_t lv = stats_.level();
    const Color c = clip ? Color::red() : Color::green();
    /* Level, dB: up to 3 characters of 7-segment digits. */
    char num[8] = "--";
    if (lv != TrackStats::kNone) Fmt{num, sizeof(num)}.d((lv - (lv < 0 ? 50 : -50)) / 100);
    int x = r.left() + 4;
    for (const char* p = num; *p && p < num + 4; p++, x += 34) draw_digit(painter, {x, r.top() + 2}, 26, 46, 5, *p, c);
    /* Peak hold, min, max. */
    auto line = [&](int row, const char* tag, int16_t v) {
        char b[16];
        Fmt f{b, sizeof(b)};
        f.s(tag);
        if (v == TrackStats::kNone)
            f.c('-');
        else
            f.d(v / 100);
        painter.draw_string({r.left() + 144, r.top() + 2 + row * 16}, st.font, Color::white(), st.background, std::string_view{b, f.len()});
    };
    line(0, "peak ", stats_.peak_hold());
    line(1, "min  ", stats_.min());
    line(2, "max  ", stats_.max());
    /* Bar: -110 .. -10 dBFS, the level, the peak hold (white), min / max (yellow ticks). */
    const int by = r.top() + 52, bh = 18, w = r.width();
    const int lx = lv == TrackStats::kNone ? 0 : scale_x(lv, w);
    painter.fill_rectangle({r.left(), by, lx, bh}, c);
    painter.fill_rectangle({r.left() + lx, by, w - lx, bh}, Theme::getInstance()->bg_dark->background);
    if (stats_.peak_hold() != TrackStats::kNone) {
        const int hx = scale_x(stats_.peak_hold(), w - 2);
        painter.fill_rectangle({r.left() + hx, by, 2, bh}, Color::white());
    }
    for (const int16_t v : {stats_.min(), stats_.max()})
        if (v != TrackStats::kNone) painter.fill_rectangle({r.left() + scale_x(v, w - 2), by + bh - 5, 2, 5}, Color::yellow());
}

void TrackHistory::paint(Painter& painter) {
    const auto r = screen_rect();
    const auto& st = style();
    painter.fill_rectangle(r, st.background);
    const int x0 = r.left() + (r.width() - static_cast<int>(TrackStats::kHist)) / 2;
    const int h = r.height() - 10;
    /* Grid every 20 dB (-110 .. -10), labels at the ends. */
    for (int db = -110; db <= -10; db += 20) {
        const int y = r.top() + 10 + h - (db + 110) * h / 100;
        painter.fill_rectangle({x0, y, static_cast<int>(TrackStats::kHist), 1}, Theme::getInstance()->bg_dark->background);
    }
    painter.draw_string({r.left(), r.top()}, ui::font::fixed_5x8, Theme::getInstance()->fg_medium->foreground, st.background,
                        std::string_view{"-10 dB          last 20 s"});
    for (size_t i = 0; i < TrackStats::kHist; i++) {
        const int8_t v = stats_.column(i);
        if (v == -128) continue;
        int bar = (v + 110) * h / 100;
        if (bar < 1) bar = 1;
        if (bar > h) bar = h;
        painter.fill_rectangle({x0 + static_cast<int>(i), r.top() + 10 + h - bar, 1, bar}, Color::green());
    }
}

TrackView::TrackView(NavigationView& nav, SignatureDetectView& main, const TrackTarget& target)
    : nav_{nav}, main_{main}, target_{target} {
    add_children({&labels_, &field_lna_, &field_vga_, &field_amp_, &check_beep_, &text_target_, &field_freq_, &field_bw_, &field_ble_,
                  &field_sq_, &field_vol_, &text_status_, &level_, &history_, &button_reset_, &button_listen_, &button_cap_, &button_back_});
    const bool freq = target_.kind == TrackTarget::FREQ;
    field_freq_.hidden(!freq);
    field_bw_.hidden(!freq);
    field_ble_.hidden(freq || target_.kind == TrackTarget::ZW_NET || target_.kind == TrackTarget::FSK_RULE);
    button_listen_.hidden(!freq);
    field_lna_.set_value(receiver_model.lna());
    field_vga_.set_value(receiver_model.vga());
    field_amp_.set_value(receiver_model.rf_amp());

    field_freq_.set_step(5000);
    field_freq_.set_value(static_cast<rf::Frequency>(target_.f_khz) * 1000);
    field_freq_.on_change = [this](rf::Frequency f) {
        target_.f_khz = static_cast<uint32_t>(f / 1000);
        main_.trk_khz_ = target_.f_khz;
        stats_.clear(main_.uptime_ms());
        retune();
        show_target();
    };
    field_freq_.on_edit = [this]() {
        auto* kv = nav_.push<FrequencyKeypadView>(static_cast<rf::Frequency>(target_.f_khz) * 1000);
        kv->on_changed = [this](rf::Frequency f) { field_freq_.set_value(f); };
    };
    field_bw_.set_selected_index(target_.bw < kTrackBws ? target_.bw : 0, false);
    field_bw_.on_change = [this](size_t i, OptionsField::value_t) {
        target_.bw = static_cast<uint8_t>(i);
        main_.trk_opt_ = (main_.trk_opt_ & ~3u) | static_cast<uint32_t>(i & 3);
        stats_.clear(main_.uptime_ms());
        retune();
        show_target();
    };
    field_ble_.set_by_value(target_.kind);
    field_ble_.on_change = [this](size_t, OptionsField::value_t v) {
        target_.kind = static_cast<TrackTarget::Kind>(v);
        stats_.clear(main_.uptime_ms());
        retune();
        show_target();
    };
    field_sq_.set_value(static_cast<int32_t>((main_.trk_opt_ >> 8) & 0xFF) - 128);
    field_sq_.on_change = [this](int32_t v) {
        main_.trk_opt_ = (main_.trk_opt_ & ~0xFF00u) | (static_cast<uint32_t>(v + 128) & 0xFF) << 8;
    };
    check_beep_.set_value((main_.trk_opt_ & 4) != 0);
    check_beep_.on_select = [this](Checkbox&, bool v) {
        main_.trk_opt_ = v ? (main_.trk_opt_ | 4u) : (main_.trk_opt_ & ~4u);
        if (v && !audio_started_ && !main_.audio_on_) {
            audio::set_rate(audio::Rate::Hz_24000);
            audio::output::start();
            audio_started_ = true;
        }
    };
    if ((main_.trk_opt_ & 4) && !main_.audio_on_) {
        audio::set_rate(audio::Rate::Hz_24000);
        audio::output::start();
        audio_started_ = true;
    }
    button_reset_.on_select = [this](Button&) {
        stats_.reset_minmax(main_.uptime_ms());
        level_.set_dirty();
    };
    button_listen_.on_select = [this](Button&) {
        auto& main = main_;
        auto& n = nav_;
        const rf::Frequency f = static_cast<rf::Frequency>(target_.f_khz) * 1000;
        n.pop();
        main.listen(f);
    };
    button_back_.on_select = [this](Button&) {
        nav_.pop();
    };
    button_cap_.on_select = [this](Button&) {
        auto& main = main_;
        auto& n = nav_;
        CaptureContext c{};
        main_.capture_base(c, target_.kind == TrackTarget::FREQ ? "track" : (target_.kind == TrackTarget::ZW_NET ? "zwave" : (target_.kind == TrackTarget::FSK_RULE ? "fsk" : "ble")), target_.label,
                           main_.uptime_ms());
        main_.capture_of_target(target_, c);
        n.pop();
        main.capture(c);
    };
    stats_.clear(main_.uptime_ms());
    hop_ms_ = main_.uptime_ms();
    retune();
    show_target();
    update_status();
    log_row("TRK_START");
}

TrackView::~TrackView() {
    log_row("TRK_STOP");
    if (audio_started_ && !main_.audio_on_) audio::output::stop();
    main_.track_end();
}

void TrackView::focus() {
    if (target_.kind == TrackTarget::FREQ)
        field_freq_.focus();
    else
        field_ble_.focus();
}

void TrackView::retune() {
    static constexpr uint8_t kChannels[3] = {37, 39, 38};
    main_.track_radio(target_, kChannels[ble_idx_ % 3]);
}

void TrackView::show_target() {
    Line& l = target_line_[0];
    Fmt f{l.text, sizeof(l.text)};
    if (target_.kind == TrackTarget::FREQ) {
        f.s(target_.label).s(" bw ").d(kTrackBwKhz[target_.bw < kTrackBws ? target_.bw : 0]).s("k");
    } else if (target_.kind == TrackTarget::ZW_NET) {
        f.s("Z-Wave net ").hex(target_.addr, 4);
    } else if (target_.kind == TrackTarget::FSK_RULE) {
        f.s("FSK ").s(target_.label);
    } else if (target_.kind == TrackTarget::BLE_ADDR) {
        f.hex(target_.addr, 6, ':');
    } else {
        f.s("rule ").d(target_.rule).c(' ').s(target_.label);
    }
    l.color = Color::white();
    text_target_.set_dirty();
}

void TrackView::on_meter(const SignatureDetectMeterMessage& m) {
    if (target_.kind != TrackTarget::FREQ) return;
    const uint32_t now = main_.uptime_ms();
    meter_rx_++;
    memcpy(reinterpret_cast<uint8_t*>(&last_) + sizeof(Message), reinterpret_cast<const uint8_t*>(&m) + sizeof(Message), sizeof(last_) - sizeof(Message));
    const bool clip = m.clip >= 4; /* a few samples at |x| >= 120 of 127 in 50 ms */
    if (clip) clip_s_ = main_.clip_s_ = main_.now_s();
    if (clip != clip_) {
        clip_ = clip;
        update_status();
    }
    stats_.add(m.peak_cdb, now);
    level_.mean_cdb = m.mean_cdb;
    level_.clip = clip;
    level_.set_dirty();
}

void TrackView::on_ble(int16_t rssi_cdb) {
    if (target_.kind == TrackTarget::FREQ) return;
    const uint32_t now = main_.uptime_ms();
    rate_.add(now);
    stats_.add(rssi_cdb, now);
    /* A BLE packet's level near full scale: the front end is overdriven. */
    const bool clip = rssi_cdb > -300;
    if (clip) clip_s_ = main_.now_s();
    level_.clip = clip;
    level_.set_dirty();
}

void TrackView::frame(uint32_t now_ms) {
    stats_.tick(now_ms);
    /* BLE: the advertising channel in turn, a second each (one channel can
     * sit in a fade). */
    if (target_.kind != TrackTarget::FREQ && target_.kind != TrackTarget::ZW_NET && target_.kind != TrackTarget::FSK_RULE && now_ms - hop_ms_ >= 1000) {
        hop_ms_ = now_ms;
        ble_idx_ = static_cast<uint8_t>((ble_idx_ + 1) % 3);
        retune();
    }
    if (now_ms - hist_paint_ms_ >= 200) {
        hist_paint_ms_ = now_ms;
        history_.set_dirty();
        level_.set_dirty(); /* the peak hold falls */
    }
    beep_check(now_ms);
}

void TrackView::beep_check(uint32_t now_ms) {
    if (!check_beep_.value()) return;
    /* Only on a fresh level: the meter reports every 50 ms, a BLE target
     * counts for a second after its packet. */
    const uint32_t fresh = target_.kind == TrackTarget::FREQ ? 200 : 1000;
    if (stats_.level() == TrackStats::kNone || now_ms - stats_.last_ms() > fresh) return;
    uint16_t hz = 0, ms = 0;
    if (geiger_.due(stats_.level(), static_cast<int16_t>(field_sq_.value() * 100), now_ms, hz, ms))
        baseband::request_audio_beep(hz, 24000, ms);
}

/* Line 0: the pause; line 1: CLIP (red, 2 s after the last), else the pulse
 * readout (frequency) or the packet rate (BLE). */
void TrackView::update_status() {
    const uint32_t now_ms = main_.uptime_ms();
    {
        Line& l = status_lines_[0];
        Fmt f{l.text, sizeof(l.text)};
        f.s("Detection PAUSED  M4 ").d(main_.stats_.cpu_pct).c('%');
        if (target_.kind == TrackTarget::FREQ && last_.mean_cdb > -12000) f.s(" avg ").d(last_.mean_cdb / 100);
        l.color = Color::yellow();
    }
    Line& l = status_lines_[1];
    Fmt f{l.text, sizeof(l.text)};
    const bool clip = clip_s_ && main_.now_s() - clip_s_ < 2;
    if (clip) {
        f.s("CLIP! lower LNA / VGA, AMP off");
        l.color = Color::red();
    } else if (target_.kind == TrackTarget::FREQ) {
        if (last_.period_us)
            f.s("pulse ~").d1(static_cast<int32_t>(last_.period_us / 100)).s(" ms  w ").d1(static_cast<int32_t>(last_.width_us / 100)).s(" ms");
        else
            f.s("no pulse train");
        l.color = Color::white();
    } else {
        const uint32_t r = rate_.rate100(now_ms);
        f.d(static_cast<int32_t>(r / 100)).c('.').d2(static_cast<int32_t>(r % 100)).s(target_.kind == TrackTarget::ZW_NET || target_.kind == TrackTarget::FSK_RULE ? " frm/s  last " : " pkt/s  last ");
        if (stats_.last_ms())
            f.d1(static_cast<int32_t>((now_ms - stats_.last_ms()) / 100)).c('s');
        else
            f.c('-');
        l.color = Color::white();
    }
    text_status_.set_dirty();
}

void TrackView::second() {
    update_status();
    if (main_.log_on()) log_row("TRK");
}

/* TRK row, once a second while the log is on (and TRK_START / TRK_STOP):
 * the target, the strongest level of the second, peak hold / min / max,
 * gains, CLIP, pulses (frequency) or packets (BLE); own position in lat/lon. */
void TrackView::log_row(const char* event) {
    if (!main_.log_on()) return;
    const bool freq = target_.kind == TrackTarget::FREQ;
    char row[300];
    Fmt f{row, sizeof(row)};
    main_.row_begin(f, event, freq ? "M" : "B");
    f.c(',');
    if (freq)
        f.d(target_.f_khz);
    else if (target_.kind == TrackTarget::BLE_ADDR)
        f.hex(target_.addr, 6, ':');
    else if (target_.kind == TrackTarget::ZW_NET)
        f.hex(target_.addr, 4);
    else if (target_.kind == TrackTarget::FSK_RULE)
        f.s("fsk").d(target_.rule + 1);
    else
        f.s("rule").d(target_.rule);
    f.c(',').s(target_.label, Fmt::Esc::Csv).s(",track,");
    if (freq)
        f.d(target_.f_khz).c(',').d(kTrackBwKhz[target_.bw < kTrackBws ? target_.bw : 0]);
    else
        f.c(',');
    f.c(',');
    const int16_t sec = stats_.take_second_max();
    if (sec != TrackStats::kNone) f.d(sec / 100);
    f.s(",dBFS,");
    int32_t lat, lon;
    if (main_.own_position(lat, lon))
        f.e7(lat).c(',').e7(lon);
    else
        f.c(',');
    auto kv = [&](const char* k, int16_t cdb) {
        f.s(k);
        if (cdb != TrackStats::kNone) f.d(cdb / 100);
    };
    f.c(',');
    kv("peak_hold=", stats_.peak_hold());
    kv(";min=", stats_.min());
    kv(";max=", stats_.max());
    f.s(";lna=").d(receiver_model.lna()).s(";vga=").d(receiver_model.vga()).s(";amp=").d(receiver_model.rf_amp());
    f.s(";clip=").d(clip_s_ && main_.now_s() - clip_s_ < 2);
    if (freq) {
        f.s(";mean=").d(last_.mean_cdb / 100).s(";floor=").d(last_.floor_cdb / 100);
        f.s(";pulse_ms=").d1(static_cast<int32_t>(last_.period_us / 100)).s(";width_ms=").d1(static_cast<int32_t>(last_.width_us / 100));
        f.s(";reports=").d(meter_rx_);
    } else {
        const uint32_t r = rate_.rate100(main_.uptime_ms());
        f.s(";pkt_s=").d(static_cast<int32_t>(r / 100)).c('.').d2(static_cast<int32_t>(r % 100));
    }
    main_.put_pos_src(f);
    main_.log_line(row);
}

/* ---------------------------------------------------------------- capture */

namespace {

/* Bytes to a file in pieces that never take FatFs's direct sector path
 * (sigdet_log.hpp): the sidecar and the CAP rows come from stack buffers. */
bool write_safe(File& f, const char* p, size_t n) {
    while (n) {
        const size_t k = log_chunk(static_cast<uint32_t>(f.tell()), n);
        if (f.write(p, k).is_error()) return false;
        p += k;
        n -= k;
    }
    return true;
}

std::filesystem::path to_path(const char* s) {
    return std::filesystem::path{s, s + strlen(s)};
}

constexpr const char* kCapDir = "/SIGDET/CAPTURES";

}  // namespace

/* Common fields: kind, label (also the default capture label), the row it
 * came from, the session log (when open: the capture view appends CAP rows). */
void SignatureDetectView::capture_base(CaptureContext& c, const char* kind, const char* label, uint32_t row_ms) const {
    Fmt{c.target_kind, sizeof(c.target_kind)}.s(kind);
    Fmt{c.target_label, sizeof(c.target_label)}.s(label, Fmt::Esc::Kv);
    cap_sanitize(label, c.label, sizeof(c.label));
    if (!c.label[0]) Fmt{c.label, sizeof(c.label)}.s(kind);
    c.row_uptime_ms = row_ms;
    if (log_ && log_name_[0]) Fmt{c.log_name, sizeof(c.log_name)}.s(log_name_);
}

/* A track target: its frequency (BLE: advertising channel 37, 2 MHz; Z-Wave:
 * its channel, 500 kHz) and the rate for its width. */
void SignatureDetectView::capture_of_target(const TrackTarget& t, CaptureContext& c) const {
    Fmt x{c.target_extra, sizeof(c.target_extra)};
    if (t.kind == TrackTarget::FREQ) {
        c.f_hz = static_cast<uint64_t>(t.f_khz) * 1000;
        c.rate = cap_rate_for(kTrackBwKhz[t.bw < kTrackBws ? t.bw : 0]);
        x.s("bw_khz=").d(kTrackBwKhz[t.bw < kTrackBws ? t.bw : 0]);
    } else if (t.kind == TrackTarget::ZW_NET) {
        c.f_hz = static_cast<uint64_t>(sig_ready() ? sig_mem_->cls.zw.f_khz : 908420) * 1000;
        c.rate = 500000;
        x.s("home=").hex(t.addr, 4);
    } else if (t.kind == TrackTarget::FSK_RULE) {
        const bool ok = sig_ready() && t.rule < sig_mem_->cls.nfsk;
        const uint32_t bw = ok ? sig_mem_->cls.fsk[t.rule].bw_khz : 200;
        c.f_hz = static_cast<uint64_t>(t.f_khz) * 1000;
        c.rate = cap_rate_for(bw);
        x.s("rule=").d(t.rule + 1).s(";bw_khz=").d(static_cast<int32_t>(bw));
    } else {
        c.f_hz = 2402000000ull;
        c.rate = 2000000;
        if (t.kind == TrackTarget::BLE_ADDR)
            x.s("addr=").hex(t.addr, 6, ':').s(";channel=37");
        else
            x.s("rule=").d(t.rule).s(";channel=37");
    }
}

/* A detection of the list. */
bool SignatureDetectView::capture_of(const Entity& e, CaptureContext& c) const {
    TrackTarget t;
    if (!target_of(e, t)) return false;
    static constexpr const char* kKinds[] = {"remote_id", "wifi_oui", "rf", "pattern", "dji_ble", "droneid", "zwave", "sig", "fsk", "scan"};
    const size_t k = static_cast<size_t>(e.kind);
    const char* kind = k < sizeof(kKinds) / sizeof(kKinds[0]) ? kKinds[k] : "?";
    if (e.kind == Kind::Sig) kind = (e.sources & SRC_RF) ? "nb" : "ble";
    char name[24];
    entity_name(e, name, sizeof(name));
    capture_base(c, kind, name, e.first_s * 1000u);
    capture_of_target(t, c);
    /* the rule, the RF class */
    Fmt x{c.target_extra + strlen(c.target_extra), sizeof(c.target_extra) - strlen(c.target_extra)};
    if (e.kind == Kind::Rf) x.s(";class=").s(rf_class_tag(e.rf_cls)).s(";conf=").c("-LMH"[e.rf_conf & 3]);
    if (e.kind == Kind::Sig && sig_ready()) x.s(";class_file=").s(sig_name_.c_str(), Fmt::Esc::Kv).s(";rule_idx=").d(e.why);
    return true;
}

/* This app closes (log STOP, baseband down) and the capture view opens with
 * the stock capture baseband. c lives on this function's stack, so it
 * outlives this view's destruction inside replace(). */
void SignatureDetectView::capture(CaptureContext c) {
    if (!c.f_hz) return;
    nav_.replace<SigdetCaptureView>(c);
}

/* ---------------------------------------------------------------- target picker */

TargetView::TargetView(NavigationView& nav, SignatureDetectView& main)
    : nav_{nav}, main_{main} {
    add_children({&labels_, &field_, &text_, &button_track_, &button_cap_, &button_back_});
    /* Item 0: the typed frequency; then the survey's unknowns, newest first. */
    Item m{};
    m.kind = 0;
    m.f_khz = main_.trk_khz_;
    items_[n_++] = m;
    const uint32_t now = main_.uptime_ms();
    if (main_.sig_mem_) {
        struct Cand {
            uint32_t last;
            Item it;
        };
        Cand c[kItems]{};
        size_t nc = 0;
        auto offer = [&](uint32_t last, const Item& it) {
            if (nc < kItems - 1) {
                c[nc++] = {last, it};
                return;
            }
            size_t old = 0;
            for (size_t i = 1; i < nc; i++)
                if (c[i].last < c[old].last) old = i;
            if (last > c[old].last) c[old] = {last, it};
        };
        for (size_t i = 0; i < kSurveyNrgSlots; i++) {
            const NrgSeen* e = main_.sig_mem_->nrg.slot(i);
            if (!e || !e->reported) continue;
            Item it{};
            it.kind = 1;
            it.band = e->band;
            it.f_khz = e->f_khz;
            it.bw_khz = e->bw_khz;
            it.db = e->db_max;
            it.age_s = (now - e->last_ms) / 1000;
            it.row_ms = e->emit_ms;
            offer(e->last_ms, it);
        }
        for (size_t i = 0; i < kSurveyBleSlots; i++) {
            const BleSeen* e = main_.sig_mem_->ble.slot(i);
            if (!e) continue;
            Item it{};
            it.kind = 2;
            memcpy(it.addr, e->addr, 6);
            it.pdu = e->pdu;
            it.db = e->rssi_max;
            it.has_cid = e->has_cid;
            it.cid = e->cid;
            it.nuuid = e->nuuid;
            it.uuid = e->nuuid ? e->uuid[0] : 0;
            it.has_sd = e->has_sd;
            it.sd = e->sd_uuid;
            it.mfr_n = static_cast<uint8_t>(e->mfr_n < 4 ? e->mfr_n : 4);
            memcpy(it.mfr, e->mfr, it.mfr_n);
            it.age_s = (now - e->last_ms) / 1000;
            it.row_ms = e->emit_ms;
            offer(e->last_ms, it);
        }
        /* newest first */
        for (size_t i = 0; i < nc; i++)
            for (size_t j = i + 1; j < nc; j++)
                if (c[j].last > c[i].last) {
                    const Cand t = c[i];
                    c[i] = c[j];
                    c[j] = t;
                }
        for (size_t i = 0; i < nc && n_ < kItems; i++) items_[n_++] = c[i].it;
    }
    OptionsField::options_t opts;
    for (size_t i = 0; i < n_; i++) {
        const Item& it = items_[i];
        char b[30];
        Fmt f{b, sizeof(b)};
        if (it.kind == 0) {
            f.s("Manual ").d(it.f_khz / 1000).c('.').d2(it.f_khz % 1000 / 10).s(" MHz");
        } else if (it.kind == 1) {
            f.s("NRG ").d(it.f_khz / 1000).c('.').d2(it.f_khz % 1000 / 10).c(' ').s(kBands[it.band].name).s(" +").d(it.db).s("dB ").d(it.age_s).c('s');
        } else {
            f.s("BLE ");
            if (it.has_cid) {
                const uint8_t c2[2] = {static_cast<uint8_t>(it.cid >> 8), static_cast<uint8_t>(it.cid)};
                f.hex(c2, 2);
            } else if (it.has_sd || it.nuuid) {
                const uint16_t u = it.has_sd ? it.sd : it.uuid;
                const uint8_t u2[2] = {static_cast<uint8_t>(u >> 8), static_cast<uint8_t>(u)};
                f.s("u").hex(u2, 2);
            } else {
                f.s("----");
            }
            f.c(' ').d(it.db).s("dB ").d(it.age_s).c('s');
        }
        opts.emplace_back(std::string{b, f.len()}, static_cast<int32_t>(i));
    }
    field_.set_options(std::move(opts));
    field_.on_change = [this](size_t, OptionsField::value_t) { show(); };
    button_track_.on_select = [this](Button&) {
        const size_t i = field_.selected_index();
        if (i >= n_) return;
        CaptureContext c{};
        TrackTarget t{};
        fill(c, t, items_[i]);
        auto& main = main_;
        nav_.pop();
        main.open_track(t);
    };
    button_cap_.on_select = [this](Button&) {
        const size_t i = field_.selected_index();
        if (i >= n_) return;
        CaptureContext c{};
        TrackTarget t{};
        fill(c, t, items_[i]);
        auto& main = main_;
        nav_.pop();
        main.capture(c);
    };
    button_back_.on_select = [this](Button&) { nav_.pop(); };
    show();
}

void TargetView::focus() {
    field_.focus();
}

/* Track target and capture context of an item. */
void TargetView::fill(CaptureContext& c, TrackTarget& t, const Item& it) const {
    t = TrackTarget{};
    if (it.kind == 2) {
        t.kind = TrackTarget::BLE_ADDR;
        memcpy(t.addr, it.addr, 6);
        Fmt{t.label, sizeof(t.label)}.s("BLE unknown");
    } else {
        t.kind = TrackTarget::FREQ;
        t.f_khz = it.f_khz;
        t.bw = static_cast<uint8_t>(it.kind == 1 ? track_bw_for(it.bw_khz) : (main_.trk_opt_ & 3));
        Fmt{t.label, sizeof(t.label)}.s(it.kind == 1 ? "energy" : "manual");
    }
    char label[24];
    Fmt lf{label, sizeof(label)};
    if (it.kind == 0)
        lf.s("manual");
    else if (it.kind == 1)
        lf.s("nrg").d(it.f_khz / 1000);
    else if (it.has_cid) {
        const uint8_t c2[2] = {static_cast<uint8_t>(it.cid >> 8), static_cast<uint8_t>(it.cid)};
        lf.s("ble").hex(c2, 2);
    } else
        lf.s("ble");
    main_.capture_base(c, it.kind == 0 ? "manual" : (it.kind == 1 ? "energy" : "ble"), label, it.row_ms);
    main_.capture_of_target(t, c);
    Fmt x{c.target_extra + strlen(c.target_extra), sizeof(c.target_extra) - strlen(c.target_extra)};
    if (it.kind == 1) {
        x.s(";band=").s(kBands[it.band].name).s(";bw_seen_khz=").d(it.bw_khz).s(";db=").d(it.db);
        c.rate = cap_rate_for(it.bw_khz);
    } else if (it.kind == 2) {
        if (it.has_cid) {
            const uint8_t c2[2] = {static_cast<uint8_t>(it.cid >> 8), static_cast<uint8_t>(it.cid)};
            x.s(";cid=").hex(c2, 2).s(";mfr=").hex(it.mfr, it.mfr_n);
        }
        if (it.nuuid) {
            const uint8_t u2[2] = {static_cast<uint8_t>(it.uuid >> 8), static_cast<uint8_t>(it.uuid)};
            x.s(";uuid=").hex(u2, 2);
        }
        if (it.has_sd) {
            const uint8_t u2[2] = {static_cast<uint8_t>(it.sd >> 8), static_cast<uint8_t>(it.sd)};
            x.s(";sd=").hex(u2, 2);
        }
        x.s(";pdu_hdr=").hex(&it.pdu, 1);
    }
}

void TargetView::show() {
    for (auto& l : lines_) {
        l.text[0] = 0;
        l.color = Color::white();
    }
    const size_t i = field_.selected_index();
    if (i >= n_) return;
    const Item& it = items_[i];
    auto line = [this](size_t k) { return Fmt{lines_[k].text, sizeof(lines_[k].text)}; };
    if (it.kind == 0) {
        line(0).s("Typed frequency (Track view:");
        line(1).s("edit it there; Capture: in");
        line(2).s("the capture view)");
    } else if (it.kind == 1) {
        line(0).s("Energy, no confident label");
        line(1).s("f ").d(it.f_khz / 1000).c('.').d(it.f_khz % 1000).s(" MHz w ").d(it.bw_khz).s("k");
        line(2).s("+").d(it.db).s(" dB over floor, ").d(it.age_s).s(" s ago");
    } else {
        line(0).s("BLE advertiser, no rule");
        line(1).hex(it.addr, 6, ':');
        Fmt f = line(2);
        if (it.has_cid) {
            const uint8_t c2[2] = {static_cast<uint8_t>(it.cid >> 8), static_cast<uint8_t>(it.cid)};
            f.s("cid ").hex(c2, 2).c(' ').hex(it.mfr, it.mfr_n);
        }
        line(3).d(it.db).s(" dBFS, ").d(it.age_s).s(" s ago");
    }
    if (n_ == 1) {
        line(4).s("No unknowns: tick Survey (Log)");
        line(5).s("to collect them while running");
        lines_[4].color = lines_[5].color = Theme::getInstance()->fg_medium->foreground;
    }
    text_.set_dirty();
}

/* ---------------------------------------------------------------- capture view */

SigdetCaptureView::SigdetCaptureView(NavigationView& nav, const CaptureContext& ctx)
    : nav_{nav}, ctx_{ctx} {
    baseband::run_image(portapack::spi_flash::image_tag_capture);
    add_children({&rssi_, &channel_, &field_frequency_, &field_amp_, &field_lna_, &field_vga_, &labels_, &option_rate_, &option_dur_,
                  &option_state_, &text_label_, &text_ant_, &record_view_, &text_status_, &button_dd_, &waterfall_});
    record_view_.set_filename_as_is(true);
    label_ = ctx_.label;
    text_label_.set_text(label_);
    text_label_.on_select = [this](TextField&) {
        text_prompt(nav_, label_, sizeof(ctx_.label) - 1, ENTER_KEYBOARD_MODE_ALPHA, [this](std::string& v) {
            char b[sizeof(ctx_.label)];
            cap_sanitize(v.c_str(), b, sizeof(b));
            label_ = b;
            text_label_.set_text(label_);
        });
    };
    text_ant_.on_select = [this](TextField&) {
        text_prompt(nav_, ant_, 20, ENTER_KEYBOARD_MODE_ALPHA, [this](std::string& v) {
            ant_ = v;
            text_ant_.set_text(ant_);
        });
    };
    option_state_.set_by_value(ctx_.state);
    option_dur_.set_by_value(10);
    option_dur_.on_change = [this](size_t, OptionsField::value_t) { show_size(); };
    freqman_set_bandwidth_option(SPEC_MODULATION, option_rate_);
    option_rate_.on_change = [this](size_t, OptionsField::value_t v) { on_rate(static_cast<uint32_t>(v)); };
    button_dd_.on_select = [this](Button&) {
        nav_.replace<SignatureDetectView>(); /* back to detecting (this view goes first) */
    };
#ifdef PRALINE
    receiver_model.set_narrowband_capture_enabled(true);
#endif
    receiver_model.enable();
    field_frequency_.set_value(ctx_.f_hz);
    option_rate_.set_by_value(static_cast<int32_t>(ctx_.rate));
    if (!rate_) on_rate(static_cast<uint32_t>(option_rate_.selected_index_value()));
    record_view_.on_error = [this](std::string message) { nav_.display_modal("Error", message); };
    record_view_.on_before_start = [this]() { return before_start(); };
    ensure_directory(to_path(kCapDir));
    show_size();
    tick_token_ = rtc_time::signal_tick_second += [this]() { poll(); };
}

SigdetCaptureView::~SigdetCaptureView() {
    rtc_time::signal_tick_second -= tick_token_;
    /* Leaving while recording or saving: stop, then every remaining step at
     * once (small frames: the buffers are members). */
    if (record_view_.is_active()) record_view_.stop(); /* joins the capture thread */
    if (step_ == STEP_PREP) step_ = STEP_IDLE;         /* nothing started yet */
    if (step_ == STEP_REC) step_ = STEP_MOVE;
    for (int i = 0; i < 4 && step_ != STEP_IDLE; i++) finish_step();
    receiver_model.disable();
#ifdef PRALINE
    receiver_model.set_narrowband_capture_enabled(false);
#endif
    baseband::shutdown();
}

void SigdetCaptureView::set_parent_rect(const Rect new_parent_rect) {
    View::set_parent_rect(new_parent_rect);
    waterfall_.set_parent_rect({0, header_height, new_parent_rect.width(), new_parent_rect.height() - header_height});
}

void SigdetCaptureView::focus() {
    record_view_.focus();
}

/* As the stock Capture app: RecordView picks the oversampling and sets the
 * baseband; C8 from 1.5 MHz on, C16 below. */
void SigdetCaptureView::on_rate(uint32_t rate) {
    if (record_view_.is_active()) return;
    waterfall_.stop();
    const auto actual = record_view_.set_sampling_rate(rate);
    receiver_model.set_sampling_rate(actual);
    bb_filter_hz_ = filter_bandwidth_for_sampling_rate(actual);
    receiver_model.set_baseband_bandwidth(bb_filter_hz_);
    record_view_.set_file_type(cap_is_c8(rate) ? RecordView::FileType::RawS8 : RecordView::FileType::RawS16);
    rate_ = rate;
    waterfall_.start();
    show_size();
}

uint32_t SigdetCaptureView::duration_ms() const {
    return static_cast<uint32_t>(option_dur_.selected_index_value()) * 1000u;
}

/* Bytes a second, size of one capture at the chosen duration, free space. */
void SigdetCaptureView::show_size() {
    if (record_view_.is_active()) return;
    const uint32_t bps = cap_bytes_per_s(rate_);
    const uint32_t d = duration_ms() / 1000;
    const uint64_t need = static_cast<uint64_t>(bps) * (d ? d : 60);
    const auto sp = std::filesystem::space(u"");
    char b[32];
    Fmt f{b, sizeof(b)};
    f.d(static_cast<int32_t>(bps / 1000)).s("kB/s ").s(cap_is_c8(rate_) ? "C8 " : "C16 ");
    f.d(static_cast<int32_t>(need / 1000000)).s(d ? "MB " : "MB/min ");
    const bool low = sp.free < need + need / 10;
    if (low)
        f.s("SD FULL?");
    else
        f.d(static_cast<int32_t>(sp.free / 1000000000ull)).s("GB free");
    text_status_.set_style(low || need > kCapWarnBytes ? Theme::getInstance()->fg_red : Theme::getInstance()->fg_light);
    text_status_.set(std::string{b, f.len()});
}

/* The step on the status line (stays until the next one): after a halt the
 * last screen frame / accessibility read shows where it was. */
void SigdetCaptureView::stage(const char* text, const Style* style) {
    text_status_.set_style(style);
    text_status_.set(text);
}

/* Dropped share of the recording from the capture thread's byte counters:
 * 0, else at least 1 %. */
uint32_t SigdetCaptureView::drop_pct() const {
    if (!drop_bytes_ || !recv_bytes_) return 0;
    const uint64_t p = drop_bytes_ * 100u / recv_bytes_;
    return p < 1 ? 1 : (p > 100 ? 100 : static_cast<uint32_t>(p));
}

/* RecordView asks before it starts (its Record button). Not yet: the SD work
 * of a start (folder, name, CAP_START row) runs on the next tick, before the
 * capture thread exists; then that tick starts the recording itself. */
bool SigdetCaptureView::before_start() {
    if (starting_) return true;
    if (step_ == STEP_IDLE) {
        step_ = STEP_PREP;
        stage("rec: preparing", Theme::getInstance()->fg_yellow);
    } else if (step_ != STEP_PREP) {
        stage("busy: saving, try again", Theme::getInstance()->fg_orange);
    }
    return false;
}

/* Once a second, at most one step of SD work, and none while the capture
 * thread runs (see Step): a leftover CAPTMP.* is recovered first; a Record
 * press is prepared and started; a recording ended (RecordView's button, an
 * error, or our duration) is saved in three steps (move, .SIG, CAP_STOP) once
 * the capture thread has been joined (is_active() false). */
void SigdetCaptureView::poll() {
    if (!recovered_) {
        recover(); /* the hook keeps any recording waiting until this is done */
        return;
    }
    if (record_view_.is_active()) {
        if (step_ != STEP_REC) return; /* not ours (cannot happen: the hook) */
        /* the live drop share on the status line (no SD access) */
        uint64_t rx, dr;
        if (record_view_.stream_bytes(rx, dr)) {
            recv_bytes_ = rx;
            drop_bytes_ = dr;
        }
        const uint32_t el = chTimeNow() * 1000u / CH_FREQUENCY - t0_ms_;
        const uint32_t pct = drop_pct();
        Fmt s{buf_, 48};
        s.s(duration_ms() ? "REC auto " : "REC ").d(static_cast<int32_t>(el / 1000)).s("s drop ").d(static_cast<int32_t>(pct)).c('%');
        stage(buf_, pct > 1 ? Theme::getInstance()->fg_red : Theme::getInstance()->fg_yellow);
        if (duration_ms() && el >= duration_ms()) {
            wall_ms_ = el;
            record_view_.stop();
        }
        return;
    }
    if (step_ == STEP_REC && !wall_ms_) wall_ms_ = chTimeNow() * 1000u / CH_FREQUENCY - t0_ms_; /* stopped by the button or an error */
    if (step_ == STEP_PREP) {
        prepare_and_start();
        return;
    }
    if (step_ == STEP_REC) step_ = STEP_MOVE; /* stopped, thread joined */
    if (step_ != STEP_IDLE) finish_step();
}

/* Record was pressed: pair, name, settings, session folder, CAP_START row,
 * all before the capture thread starts; then RecordView creates CAPTMP.* and
 * starts the thread. From here until it stops nothing here touches the SD. */
void SigdetCaptureView::prepare_and_start() {
    stage("rec: CAP_START row", Theme::getInstance()->fg_yellow);
    state_ = static_cast<uint8_t>(option_state_.selected_index_value());
    dropped_pct_ = 0;
    recv_bytes_ = drop_bytes_ = 0;
    wall_ms_ = 0;
    pair_ = pairs_.next(state_);
    rtc_time::now(t0_rtc_);
    lna_ = receiver_model.lna();
    vga_ = receiver_model.vga();
    amp_ = receiver_model.rf_amp();
    if (!session_[0]) {
        /* the session folder, named at the first recording */
        Fmt f{session_, sizeof(session_)};
        f.s(kCapDir).c('/').d(t0_rtc_.year()).d2(t0_rtc_.month()).d2(t0_rtc_.day()).c('_').d2(t0_rtc_.hour()).d2(t0_rtc_.minute()).d2(t0_rtc_.second());
        f.c('_').s(label_.c_str());
        ensure_directory(to_path(session_));
    }
    /* P01_OFF, or P01_OFF_2 if that name exists already */
    for (uint8_t dup = 0; dup < 9; dup++) {
        cap_base_name(pair_, state_, dup, base_, sizeof(base_));
        Fmt{pa_, sizeof(pa_)}.s(session_).c('/').s(base_).s(cap_is_c8(rate_) ? ".C8" : ".C16");
        if (!file_exists(to_path(pa_))) break;
    }
    log_row("CAP_START", 0, 0);
    stage("rec: starting", Theme::getInstance()->fg_yellow);
    starting_ = true;
    record_view_.start(); /* metadata + CAPTMP created here, then the thread */
    starting_ = false;
    if (!record_view_.is_active()) {
        log_row("CAP_FAIL", 0, 0); /* RecordView showed the error */
        step_ = STEP_IDLE;
        stage("REC FAILED to start", Theme::getInstance()->fg_red);
        return;
    }
    t0_ms_ = chTimeNow() * 1000u / CH_FREQUENCY;
    step_ = STEP_REC;
    stage(duration_ms() ? "REC (auto stop)" : "REC", Theme::getInstance()->fg_yellow);
}

/* One step of saving a finished recording. */
void SigdetCaptureView::finish_step() {
    const char* ext = cap_is_c8(rate_) ? ".C8" : ".C16";
    if (step_ == STEP_MOVE) {
        stage("save 1/3: move", Theme::getInstance()->fg_yellow);
        Fmt{tmp_stem_, sizeof(tmp_stem_)}.s("CAPTMP");
        Fmt{pa_, sizeof(pa_)}.s(kCapDir).c('/').s(tmp_stem_).s(ext);
        if (!file_exists(to_path(pa_))) {
            Fmt{tmp_stem_, sizeof(tmp_stem_)}.s("CAPTMP_GEO");
            Fmt{pa_, sizeof(pa_)}.s(kCapDir).c('/').s(tmp_stem_).s(ext);
        }
        bytes_ = 0;
        if (!file_.open(to_path(pa_)).is_valid()) bytes_ = file_.size();
        file_.close();
        Fmt{pb_, sizeof(pb_)}.s(session_).c('/').s(base_).s(ext);
        moved_ = rename_file(to_path(pa_), to_path(pb_)).code() == 0;
        Fmt{pa_, sizeof(pa_)}.s(kCapDir).c('/').s(tmp_stem_).s(".TXT");
        Fmt{pb_, sizeof(pb_)}.s(session_).c('/').s(base_).s(".TXT");
        rename_file(to_path(pa_), to_path(pb_));
        step_ = STEP_SIG;
        return;
    }
    if (step_ == STEP_SIG) {
        stage("save 2/3: .SIG", Theme::getInstance()->fg_yellow);
        Fmt{sc_file_, sizeof(sc_file_)}.s(base_).s(ext);
        Fmt{sc_meta_, sizeof(sc_meta_)}.s(base_).s(".TXT");
        sc_ = CapSidecar{};
        sc_.file = sc_file_;
        sc_.metadata = sc_meta_;
        sc_.center_hz = receiver_model.target_frequency();
        sc_.sample_rate = rate_;
        sc_.c8 = cap_is_c8(rate_);
        sc_.lna = lna_;
        sc_.vga = vga_;
        sc_.amp = amp_;
        sc_.year = static_cast<uint16_t>(t0_rtc_.year());
        sc_.month = static_cast<uint8_t>(t0_rtc_.month());
        sc_.day = static_cast<uint8_t>(t0_rtc_.day());
        sc_.hour = static_cast<uint8_t>(t0_rtc_.hour());
        sc_.minute = static_cast<uint8_t>(t0_rtc_.minute());
        sc_.second = static_cast<uint8_t>(t0_rtc_.second());
        const uint32_t bps = cap_bytes_per_s(rate_);
        sc_.duration_ms = bps ? static_cast<uint32_t>(bytes_ * 1000u / bps) : 0; /* from the data */
        sc_.bytes = bytes_;
        sc_.state = state_;
        sc_.pair = pair_;
        sc_.label = label_.c_str();
        sc_.antenna = ant_.c_str();
        sc_.ctx = &ctx_;
#ifdef VERSION_STRING
        sc_.firmware = VERSION_STRING;
#else
        sc_.firmware = "?";
#endif
        sc_.build = __DATE__ " " __TIME__;
        sc_.uptime_ms = t0_ms_;
        dropped_pct_ = drop_pct();
        sc_.dropped_pct = dropped_pct_;
        sc_.received_bytes = recv_bytes_;
        sc_.dropped_bytes = drop_bytes_;
        sc_.dropped_buffers = static_cast<uint32_t>((drop_bytes_ + kWriteSize - 1) / kWriteSize);
        sc_.write_size = kWriteSize;
        sc_.wall_ms = wall_ms_;
        sc_.bb_filter_hz = bb_filter_hz_;
        sc_.target_hz = static_cast<uint64_t>(receiver_model.target_frequency()) != ctx_.f_hz ? ctx_.f_hz : 0;
        const size_t n = cap_sidecar(buf_, sizeof(buf_), sc_);
        Fmt{pa_, sizeof(pa_)}.s(session_).c('/').s(base_).s(".SIG");
        sig_ok_ = false;
        if (!file_.create(to_path(pa_)).is_valid()) sig_ok_ = write_safe(file_, buf_, n);
        file_.close();
        step_ = STEP_LOG_STOP;
        return;
    }
    if (step_ == STEP_LOG_STOP) {
        stage("save 3/3: CAP_STOP row", Theme::getInstance()->fg_yellow);
        log_row("CAP_STOP", sc_.duration_ms, bytes_);
        step_ = STEP_IDLE;
        /* red: not saved, or samples dropped (> 1 %: no use for timing) */
        Fmt s{buf_, 48};
        s.s(moved_ && sig_ok_ ? "saved " : "SAVE FAILED ").s(base_).c(' ').d1(static_cast<int32_t>(sc_.duration_ms / 100)).s("s ").d(static_cast<int32_t>(bytes_ / 1000000)).s("MB");
        s.s(" drop ").d(static_cast<int32_t>(dropped_pct_)).s("% stk").d(static_cast<int32_t>(get_free_stack_space()));
        stage(buf_, moved_ && sig_ok_ && dropped_pct_ <= 1 ? Theme::getInstance()->fg_green : Theme::getInstance()->fg_red);
    }
}

/* A capture interrupted earlier (halt, power loss) left CAPTMP.* in
 * /SIGDET/CAPTURES: renamed RECOVERED_<n>.* (n = the first free 1..99), so
 * the next recording cannot collide with it and the data is kept. */
void SigdetCaptureView::recover() {
    recovered_ = true;
    static constexpr const char* kStems[2] = {"CAPTMP", "CAPTMP_GEO"};
    static constexpr const char* kExts[3] = {".C16", ".C8", ".TXT"};
    uint8_t n = 0;
    for (const char* st : kStems) {
        bool any = false;
        for (const char* e : kExts) {
            Fmt{pa_, sizeof(pa_)}.s(kCapDir).c('/').s(st).s(e);
            any = any || file_exists(to_path(pa_));
        }
        if (!any) continue;
        for (n = 1; n < 99; n++) {
            bool taken = false;
            for (const char* e : kExts) {
                Fmt{pb_, sizeof(pb_)}.s(kCapDir).s("/RECOVERED_").d(n).s(e);
                taken = taken || file_exists(to_path(pb_));
            }
            if (!taken) break;
        }
        for (const char* e : kExts) {
            Fmt{pa_, sizeof(pa_)}.s(kCapDir).c('/').s(st).s(e);
            Fmt{pb_, sizeof(pb_)}.s(kCapDir).s("/RECOVERED_").d(n).s(e);
            if (file_exists(to_path(pa_))) rename_file(to_path(pa_), to_path(pb_));
        }
    }
    if (n) {
        Fmt{buf_, 40}.s("recovered CAPTMP -> RECOVERED_").d(n);
        stage(buf_, Theme::getInstance()->fg_orange);
    }
}

/* CAP_START / CAP_STOP into the session CSV (Signature Detect's, when its log
 * was on): appended and closed each time, in sector-safe pieces; the row and
 * the File are members (stack). */
void SigdetCaptureView::log_row(const char* event, uint32_t dur_ms, uint64_t bytes) {
    if (!ctx_.log_name[0]) return;
    Fmt f{buf_, sizeof(buf_)};
    f.d(static_cast<int32_t>(chTimeNow() * 1000u / CH_FREQUENCY)).c(',');
    rtc::RTC t;
    rtc_time::now(t);
    f.d(t.year()).c('-').d2(t.month()).c('-').d2(t.day()).c(' ').d2(t.hour()).c(':').d2(t.minute()).c(':').d2(t.second());
    f.c(',').s(event).s(",C,").s(base_).c(',').s(label_.c_str(), Fmt::Esc::Csv).s(",capture,");
    f.d(static_cast<int32_t>(receiver_model.target_frequency() / 1000)).c(',').d(static_cast<int32_t>(rate_ / 1000)).s(",,,,,");
    f.s("state=").s(state_ ? "on" : "off").s(";pair=P").d2(pair_).s(";folder=").s(session_, Fmt::Esc::Kv);
    f.s(";file=").s(base_).s(cap_is_c8(rate_) ? ".C8" : ".C16").s(";rate=").d(static_cast<int32_t>(rate_));
    f.s(";lna=").d(lna_).s(";vga=").d(vga_).s(";amp=").d(amp_).s(";target=").s(ctx_.target_kind, Fmt::Esc::Kv);
    if (dur_ms) f.s(";duration_ms=").d(static_cast<int32_t>(dur_ms)).s(";bytes=").d(static_cast<int32_t>(bytes > 0x7FFFFFFF ? 0x7FFFFFFF : bytes));
    f.s("\r\n");
    if (!file_.append(to_path(ctx_.log_name)).is_valid()) write_safe(file_, buf_, f.len());
    file_.close();
}

/* ---------------------------------------------------------------- settings */

SettingsView::SettingsView(NavigationView& nav, SignatureDetectView& main)
    : main_{main} {
    add_children({&labels_, &check_alert_, &check_json_, &text_src_, &text_pos_, &button_set_, &button_clear_, &button_load_, &button_save_, &text_msg_,
                  &button_drones_});
    /* Option byte and bit of each box (SigdetOptions; unchanged by the split
     * into two pages, so older settings files read the same). */
    main_.bind_option(check_alert_, 0, 1, false);
    main_.bind_option(check_json_, 2, 1, false);
#if SIGDET_FULL
    add_children({&check_beep_low_});
    main_.bind_option(check_beep_low_, 7, 1, false);
#endif
    button_drones_.on_select = [this, &nav](Button&) {
        nav.push<DronesSetupView>(main_);
    };
    button_set_.on_select = [this, &nav](Button&) {
        if (!main_.map_fits()) {
            text_pos_.set(main_.log_ ? "No memory for map: Log off?" : "Not enough memory for the map");
            return;
        }
        int32_t lat = 0, lon = 0;
        main_.own_position(lat, lon);
        main_.pause_rf(true); /* as for the map; on_show() resumes */
        nav.push<SigdetMapView>(
            0, GeoPos::alt_unit::METERS, GeoPos::spd_unit::HIDDEN, deg(lat), deg(lon),
            [this](int32_t, float lat, float lon, int32_t) {
                main_.opt_.man_lat = to_e7(lat);
                main_.opt_.man_lon = to_e7(lon);
                main_.opt_.man_meta = (uint64_t{saved_position::SOURCE_MAP} << 32) | saved_position::now_fat();
                text_msg_.set("");
                show_position();
            });
    };
    button_clear_.on_select = [this](Button&) {
        main_.opt_.man_lat = main_.opt_.man_lon = 0;
        main_.opt_.man_meta = 0;
        text_msg_.set("");
        show_position();
    };
    button_load_.on_select = [this](Button&) { load_shared(); };
    button_save_.on_select = [this](Button&) { save_shared(); };
    show_position();
}

/* Load: the shared position becomes the manual one, with its source and time. */
void SettingsView::load_shared() {
    saved_position::Position p;
    if (!saved_position::load(p)) {
        text_msg_.set("No saved position on SD");
        return;
    }
    main_.opt_.man_lat = p.lat_e7;
    main_.opt_.man_lon = p.lon_e7;
    main_.opt_.man_meta = (uint64_t{p.source} << 32) | p.time;
    main_.log_position("POS_LOAD", p);
    text_msg_.set("Loaded SETTINGS/position.ini");
    show_position();
}

/* Save: the GPS fix while there is one (source GPS, now), else the manual
 * position with the source and time it came with. */
void SettingsView::save_shared() {
    saved_position::Position p;
    if (main_.gps_fix()) {
        p.lat_e7 = main_.gps_lat_;
        p.lon_e7 = main_.gps_lon_;
        p.alt_m = main_.gps_alt_m_;
        p.sats = main_.gps_sats_;
        p.source = saved_position::SOURCE_GPS;
        p.time = saved_position::now_fat();
    } else if (!main_.manual_position(p)) {
        text_msg_.set("No position to save");
        return;
    }
    if (!saved_position::save(p)) {
        text_msg_.set("SD write failed");
        return;
    }
    main_.log_position("POS_SAVE", p);
    char buf[40];
    Fmt f{buf, sizeof(buf)};
    f.s("Saved: ").s(saved_position::source_name(p.source)).s(p.source == saved_position::SOURCE_GPS ? " fix" : "");
    text_msg_.set(buf);
}

void SettingsView::focus() {
    check_alert_.focus();
}

void SettingsView::on_show() {
    main_.pause_rf(false);
}

DronesSetupView::DronesSetupView(NavigationView& nav, SignatureDetectView& main)
    : main_{main} {
    (void)nav;
    add_children({&labels_, &check_rid_, &check_wifi_, &check_did_, &check_pat_, &check_rf_, &check_b4_, &check_b0_, &check_b5_, &check_b1_, &check_eu_,
                  &check_esp_});
#if SIGDET_FULL
    add_children({&check_hop24_, &check_b3_, &check_b2_});
#endif
    /* Option byte and bit of each box (SigdetOptions; inverted: set means off). */
    main_.bind_option(check_rid_, 4, 2, true);
    main_.bind_option(check_rf_, 4, 1, true);
    main_.bind_option(check_pat_, 4, 8, true);
    main_.bind_option(check_wifi_, 4, 16, true);
    main_.bind_option(check_did_, 4, 32, true);
    main_.bind_option(check_b0_, 5, 1, false);
    main_.bind_option(check_b1_, 5, 2, false);
    main_.bind_option(check_b4_, 5, 16, false);
    main_.bind_option(check_b5_, 5, kBandInverted, true);
    main_.bind_option(check_eu_, 6, 1, false);
    main_.bind_option(check_esp_, 3, 1, false);
#if SIGDET_FULL
    main_.bind_option(check_b2_, 5, 4, false);
    main_.bind_option(check_b3_, 5, 8, false);
    main_.bind_option(check_hop24_, 4, 4, false);
#endif
}

void DronesSetupView::focus() {
    check_rid_.focus();
}

void SettingsView::show_position() {
    char buf[32];
    Fmt f{buf, sizeof(buf)};
    const auto& o = main_.opt_;
    saved_position::Position p;
    if (main_.manual_position(p)) {
        f.e7(o.man_lat).c(' ').e7(o.man_lon);
        char src[24], age[8];
        Fmt g{src, sizeof(src)};
        saved_position::format_age(age, sizeof(age), saved_position::age_s(p.time, saved_position::now_fat()));
        g.s(saved_position::source_name(p.source)).c(' ').s(age);
        text_src_.set(src);
    } else {
        f.s("not set");
        text_src_.set("");
    }
    text_pos_.set(buf);
}

/* ---------------------------------------------------------------- debug */

DebugView::DebugView(NavigationView& nav, SignatureDetectView& main)
    : main_{main} {
    (void)nav;
    add_children({&check_pass_all_, &check_usb_, &labels_, &opt_dump_, &text_});
#if SIGDET_FULL
    add_children({&check_demo_, &check_sd_, &console_, &check_conjugate_});
    console_.writeln("Ch MAC          dB T #ctr");
    check_demo_.set_value(main_.demo_on_);
    check_sd_.set_value(main_.dump_sd_);
    check_demo_.on_select = [this](Checkbox&, bool v) {
        main_.set_demo(v);
        main_.refresh_status();
    };
    check_sd_.on_select = [this](Checkbox&, bool v) {
        main_.dump_sd_ = v;
        if (!v) main_.dump_log_.reset();
    };
#endif
    /* Bring-up switches; the baseband takes them now, or at the next BLE dwell. */
    bool* const flags[] = {&main_.pass_all_, &main_.raw_usb_, &main_.conjugate_};
    Checkbox* const boxes[] = {&check_pass_all_, &check_usb_,
#if SIGDET_FULL
                               &check_conjugate_
#endif
    };
    for (size_t i = 0; i < sizeof(boxes) / sizeof(boxes[0]); i++) {
        bool* const flag = flags[i];
        boxes[i]->set_value(*flag);
        boxes[i]->on_select = [this, flag](Checkbox&, bool v) {
            *flag = v;
            main_.configure();
            main_.update_async_tx();
        };
    }
    opt_dump_.set_by_value(main_.dump_band_);
    opt_dump_.on_change = [this](size_t, OptionsField::value_t v) {
        main_.dump_band_ = v;
        main_.update_async_tx();
    };

    show_stats();
}

DebugView::~DebugView() {
    main_.debug_ = nullptr;
}

void DebugView::focus() {
    check_pass_all_.focus();
}

/* Counters since the app started (or key z), max/lag/peak of the last 500 ms:
 *   sync <packets started> ok <CRC good> RID <Remote ID> fw <queued to the M0>
 *   trig <soft checks> len <bad length> crc <bad CRC>
 *   M4 <load>% max <longest execute> lag <latest end>/<latest start> (us)
 *   drop <all>: B <BLE dwell> S <sweep> X <rate switch>
 *       W <Wi-Fi dwell> L <own overrun> H <after a retune>
 *   qf <lost messages>: R <Remote ID> S <stats> lk <queue lock busy> pk <bytes queued peak>
 *   skip <settle buffers> cap <triggers capped> dup <RID repeats> rcap <rate cap>
 *   part <sweeps missing a slice> st <empty dwells> M0 <never allocated>+<free> (bytes)
 * then pattern scan / table / DJI adverts, then Wi-Fi Remote ID (counters
 * only, never frame content; L: legacy IEs forwarded, ch: channel set):
 *   WiFi fr <1M frames seen> ok <FCS good> bad <FCS bad> RID <Remote ID>/<queued>
 *   B <beacons> P <probe resp> A <action> t <triggers> ov <ring overflows> st <longest step us>
 * then sweep slices per second of each band's last dwell (2.4 5.8 5.3 1.2 915 868). */
void DebugView::show_stats() {
    using I = int32_t;
    const auto& s = main_.stats_;
    size_t heap_free = 0;
    chHeapStatus(nullptr, &heap_free);
    const I v[] = {I(s.aa_hits), I(s.ok), I(s.rid_ok), I(s.rid_fwd), I(s.triggers), I(s.len_bad), I(s.crc_bad), s.cpu_pct,
                   s.max_exec_us, s.max_lag_us, s.max_start_us, s.drops, s.drops_ble, s.drops_sweep, s.drops_switch,
                   s.drops_wifi, s.drops_late, s.drops_hop, s.q_fail, s.qf_rid, s.qf_stats, s.qf_lock, s.q_peak,
                   s.hop_skips, s.sync_capped, s.dup_suppressed, s.rate_capped, main_.partial_, main_.stalls_,
                   I(chCoreStatus()), I(heap_free)};
    const I* pv = v;
    const char* const* none = nullptr;
    const char* t =
        "sync # ok # RID #/#\ntrig # len # crc #\nM4 #% max # lag #/#\ndrop #: B # S # X #\n W # L # H #\n"
        "qf #: R # S # lk # pk #\nskip # cap # dup # rcap #\npart # st # M0 #+#";
    for (size_t i = 0; i < 8; i++) t = Fmt{lines_[i].text, sizeof(lines_[i].text)}.t(t, pv, none);
    /* Line 8: pattern scan, line 9: its table (or the first bad line of the
     * file), DJI adverts. */
    {
        const I w[] = {I(s.pat_hits), I(s.pat_fwd), I(s.pat_matches), s.pat_rcap, s.drops_pat, I(main_.pat_ms_ / 1000)};
        const I* pw = w;
        Fmt{lines_[8].text, sizeof(lines_[8].text)}.t("pat # fw # m # rc # dr # #s", pw, none);
        const auto& r = main_.pat_table_.report();
        Fmt f{lines_[9].text, sizeof(lines_[9].text)};
        if (main_.sd_legacy_) f.s("OLD /DRONEDET "); /* classes read from the old folder */
        if (main_.sig_ready()) {
            /* A class file: rules loaded, or its first bad line and why. */
            const SigReport& sr = main_.sig_mem_->cls.report;
            if (sr.bad_line)
                f.s("SIG ln").d(sr.bad_line).c(' ').s(sr.why);
            else
                f.s("SIG b").d(sr.ble).s(" n").d(sr.nb).s(" bd").d(sr.bands).s(" p").d(sr.pat).s(" f").d(main_.sig_mem_->cls.nfsk);
        } else if (r.bad_line)
            f.s("PAT ln").d(r.bad_line).c(' ').s(r.why);
        else
            f.s("tbl ").d(main_.pat_table_.size()).s(r.file ? " file " : " nofile ").d(r.loaded);
        f.s(" DJI ").d(s.vendor_ok).c('/').d(s.vendor_fwd);
        lines_[9].color = r.bad_line ? Color::yellow() : Color::white();
    }
    /* Lines 10-11: Wi-Fi Remote ID. */
    {
        const I w[] = {I(s.wifi_frames), I(s.wifi_fcs_ok), I(s.wifi_fcs_bad), I(s.wifi_rid), I(s.wifi_fwd), I(s.wifi_beacon),
                       I(s.wifi_probe), I(s.wifi_action), I(s.wifi_triggers), s.wifi_overflows, s.wifi_max_step_us};
        const I* pw = w;
        Fmt{lines_[10].text, sizeof(lines_[10].text)}.t("WiFi fr # ok # bad # RID #/#", pw, none);
        Fmt f{lines_[11].text, sizeof(lines_[11].text)};
        f.t("B # P # A # t # ov # st #", pw, none);
        f.s(main_.wifi_legacy_ ? " L" : "").s(main_.wifi_mask_ == tune::kWifiChannelMask ? "" : " ch+");
        lines_[11].color = (s.wifi_overflows || s.wifi_busy) ? Color::yellow() : Color::white();
    }
    /* Line 12: sweep slices per second / 10 of each swept band's last dwell,
     * in RfBand order (full numbers and looks: DDSTAT; seven bands of four
     * digits would not fit); once the I2C bus or the sample clock had a failure, that
     * instead (faults/timeouts, recoveries/failed, clock errors, USB lines
     * dropped). */
    {
        Fmt f{lines_[12].text, sizeof(lines_[12].text)};
        const auto& i2c = portapack::i2c0.stats();
        const uint32_t clk = portapack::clock_manager.sampling_frequency_errors();
        if (i2c.faults || clk) {
            f.s("I2C ").d(i2c.faults).c('/').d(i2c.timeouts).s(" rc ").d(i2c.recoveries).c('/').d(i2c.recovery_failed);
            f.s(" clk ").d(clk).s(" usb ").d(main_.usb_dropped_);
            lines_[12].color = (!i2c.bus_ok || !portapack::clock_manager.sampling_frequency_ok()) ? Color::red() : Color::yellow();
        } else {
            f.s("sw/10");
            for (size_t b = 0; b < BAND_COUNT; b++) {
                const auto& st = main_.bstat_[b];
                if (st.ms) f.c(' ').d(st.slices * 100u / st.ms);
            }
            lines_[12].color = Color::white();
        }
    }
    text_.set_dirty();
}

bool DebugView::on_keyboard(const KeyboardEvent key) {
    auto& m = main_;
    switch (key) {
        case 'o':
            m.dump_band_ = kBle;
            break;
        case 'a':
            m.dump_band_ = SignatureDetectView::kDumpAll;
            break;
        case 'r':
            m.dump_raw_ = !m.dump_raw_;
            break;
        case 's':
            m.stat_usb_ = !m.stat_usb_;
            break;
        case 'p':
            m.pace_ = !m.pace_;
            break;
        case 't':
            m.settle_us_ = m.settle_us_ >= 100 ? 50 : (m.settle_us_ >= 50 ? 0 : 100);
            break;
        case 'v':
            m.avg_ = m.avg_ == 0 ? 4 : (m.avg_ >= 4 ? 2 : (m.avg_ >= 2 ? 1 : 0));
            break;
        case 'z':
            m.configure(SignatureDetectConfigMessage::FLAG_RESET_STATS);
            m.ble_ms_ = m.sweep_ms_ = m.wifi_ms_ = 0;
            m.partial_ = m.stalls_ = 0;
            m.wifi_rx_ = m.wifi_odid_ = 0;
            break;
        case 'w':
            m.wifi_mask_ = m.wifi_mask_ == tune::kWifiChannelMask ? tune::kWifiChannelMaskAll : tune::kWifiChannelMask;
            break;
        case 'l':
            m.wifi_legacy_ = !m.wifi_legacy_; /* from the next Wi-Fi dwell */
            break;
        case 'k':
            m.wifi_search_ = m.wifi_search_ == 0 ? 3 : static_cast<uint8_t>(m.wifi_search_ - 1); /* default, 4, 3, 2 phases */
            break;
        default:
            if (key >= '0' && key < '0' + BAND_COUNT) {
                m.dump_band_ = key - '0';
                break;
            }
            return false;
    }
    opt_dump_.set_by_value(m.dump_band_);
    m.update_async_tx();
    show_stats();
    return true;
}

#if SIGDET_FULL
void DebugView::write(const char* line) {
    console_.writeln(line);
}
#endif

}  // namespace ui::external_app::signature_detect

#else

namespace ui::external_app::signature_detect {

SignatureDetectView::SignatureDetectView(NavigationView& nav)
    : nav_{nav} {
    add_children({&text_info, &button_back});
    button_back.on_select = [this](Button&) {
        nav_.pop();
    };
}

void SignatureDetectView::focus() {
    button_back.focus();
}

}  // namespace ui::external_app::signature_detect

#endif
