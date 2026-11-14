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

/* Detection table of the Signature Detect app: fixed arrays, no heap, integer
 * only, no UI or ChibiOS dependency (host testable). Every source ends up in
 * one of two calls:
 *   ingest_odid()    - one Open Drone ID message or message pack (25 byte
 *                      wire format) from the on-board BLE demodulator, the
 *                      on-board Wi-Fi receiver (beacon IE / NAN pack), the demo
 *                      generator, or later a raw-frame ESP32 command;
 *   ingest_summary() - one decoded entry of the ESP32 module's drone list.
 * Confirmed RF (energy sweep) tracks become Kind::Rf entities through
 * ingest_rf(), with freq_khz / bw_khz / rf_* filled in by the caller; DJI
 * DroneID bursts (PSDT MODE_DRONEID) one Kind::DroneId entity through
 * ingest_droneid(). */

#ifndef __SIGDET_MODEL_HPP__
#define __SIGDET_MODEL_HPP__

#include <cstddef>
#include <cstdint>

#include "odid_lite.hpp"

namespace ui::external_app::signature_detect {

/* Entity::sources bits. */
enum : uint8_t {
    SRC_BLE = 0x01,          // on-board BLE demodulator (PSDT)
    SRC_ESP_WIFI = 0x02,     // ESP32 module: WiFi beacon Remote ID / maker OUI
    SRC_ESP_BLE = 0x04,      // ESP32 module: BLE Remote ID
    SRC_DEMO = 0x08,         // synthesised frames (debug demo)
    SRC_RF = 0x10,           // energy sweep (RfEngine track)
    SRC_PAT = 0x20,          // pattern scan (PSDT MODE_GFSK_PATTERN, sigdet_pattern.hpp)
    SRC_HACKRF_WIFI = 0x40,  // on-board Wi-Fi Remote ID (PSDT MODE_DSSS: 802.11b 1 Mb/s beacon / NAN)
    SRC_DRONEID = 0x80,      // on-board DJI DroneID burst detector (PSDT MODE_DRONEID), letter O (OcuSync)
};
/* The on-board receivers, whose level is dBFS (the ESP32's is dBm). */
constexpr uint8_t SRC_ONBOARD = SRC_BLE | SRC_DEMO | SRC_HACKRF_WIFI | SRC_DRONEID;

/* Message type bits of RidRecord::msg_seen. */
enum : uint8_t {
    MSG_BASIC = 0x01,
    MSG_LOCATION = 0x02,
    MSG_AUTH = 0x04,
    MSG_SELF = 0x08,
    MSG_SYSTEM = 0x10,
    MSG_OPERATOR = 0x20,
};

enum class Kind : uint8_t {
    RemoteId,   // Open Drone ID emitter, has a RidRecord
    WifiOui,    // drone maker WiFi network (ESP32), no position
    Rf,         // energy sweep detection (RfEngine), key mac = 00 00 'R' 'F' band uid
    Pattern,    // pattern scan detection (sigdet_pattern.hpp), key mac = 00 00 'P' 'T' entry 0
    BleVendor,  // drone maker BLE advert, not Remote ID (sigdet_ble_vendor.hpp: DJI): presence only
    DroneId,    // DJI DroneID (OcuSync) bursts: presence only, one entity for all, key mac = 00 00 'D' 'I' 0 0
    Zwave,      // Z-Wave network (sigdet_zwnet.hpp, PSDT MODE_ZWAVE), key mac = 'Z' 'W' home ID; label "ZW <home ID>"
    Sig,        // signature class rule (sigdet_sig.hpp): BLE advert (key: its address) or NB energy (key 00 00 'N' 'B' rule 0); presence only
    Fsk,        // FSK rule (sigdet_sig.hpp, PSDT MODE_FSK): one entity per rule, key mac = 00 00 'F' 'S' rule 0; label = the rule's
    Scan,       // interference scan finding (sigdet_scan.hpp): key mac = 00 00 'S' 'C' uid 0; rf_cls = ScanType, rf_conf = ScanConf, freq_khz / bw_khz
};

constexpr uint32_t kStaleSeconds = 10;
constexpr uint32_t kExpireSeconds = 60;
constexpr uint32_t kConfirmSeconds = 5;      // a RID emitter without Basic ID is announced after this
constexpr uint32_t kLocationLogSeconds = 2;  // at most one location report per entity per 2 s
constexpr int8_t kNoRssi = -128;
constexpr uint32_t kNoDistance = 0xFFFFFFFF;
constexpr size_t kRssiHistory = 8;
constexpr size_t kUasIdMatch = 19;  // the ESP32 list keeps only 19 characters of the UAS ID

struct Entity {
    using Key = uint32_t;
    static constexpr Key invalid_key = 0;

    Key id{invalid_key};
    Key key() const { return id; }

    RidRecord* rid{nullptr};  // Kind::RemoteId only
    uint32_t first_s{0};
    uint32_t last_s{0};
    uint32_t loc_report_s{0};
    uint32_t dist_m{kNoDistance};  // own position -> drone, kept by tick()
    uint32_t freq_khz{0};          // Kind::Rf
    uint32_t bw_khz{0};            // Kind::Rf
    uint16_t frames{0};
    uint8_t mac[6]{};  // BLE AdvA / WiFi BSSID, most significant byte first
    Kind kind{Kind::RemoteId};
    uint8_t sources{0};
    int8_t rssi{kNoRssi};         // dBFS from the on-board receiver, dBm from the ESP32 (rssi_dbm)
    int8_t rssi_max{kNoRssi};     // same unit as rssi
    int8_t hist[kRssiHistory]{};  // strongest per second, [0] = current second
    bool rssi_dbm{false};
    bool stale{false};
    bool confirmed{false};
    bool requeued{false}; /* evicted (its DROP logged) and back within kExpireSeconds: listed again without a NEW */
    uint8_t rf_cls{0};  // Kind::Rf: RfClass, RfConf, duty %, flatness 0.1 dB
    uint8_t rf_conf{0};
    uint8_t rf_duty{0};
    uint8_t rf_flat{0};
    uint8_t pat{0};      // Kind::Pattern: pattern table entry; Kind::Sig: SigKind
    uint8_t why{0};      // Kind::BleVendor: sigdet::VENDOR_* bits seen; Kind::Sig: rule index (BLE or NB by sources)
    uint8_t pdus{0};     // Kind::BleVendor: bit n = PDU type n seen
    uint16_t rate10{0};  // Kind::Pattern: hits per second x10 over the last pass
    uint16_t did_zc{0};  // Kind::DroneId, last burst: ZC root 600 score x 1000, SNR (centi-dB), CFO (Hz), 8 / 9 symbols
    int16_t did_snr{0};
    int16_t did_cfo{0};
    uint8_t did_nsym{0};
    char cls[5]{};     // Kind::Pattern: the entry's class tag (TOY, ...); Kind::Sig: the kind tag (TRKR, ...)
    char label[24]{};  // name for entities without a RidRecord (SSID, band, pattern, BLE local name)

    /* '^' rising, 'v' falling, '=' steady, ' ' unknown. */
    char trend() const;
};

/* One DJI DroneID burst as the PSDT detector reports it (presence only). */
struct DroneIdBurst {
    int8_t rssi;        // burst level, dBFS
    uint32_t freq_khz;  // DroneID centre of the dwell
    uint16_t zc;        // ZC root 600 score x 1000
    int16_t snr;        // centi-dB over the energy floor
    int16_t cfo;        // fractional carrier offset, Hz
    uint8_t nsym;       // 8 (O1 / O4 layout) or 9 (O2)
};
/* Key of the (single) DroneID entity: bursts carry no identity we decode. */
constexpr uint8_t kDroneIdMac[6] = {0, 0, 'D', 'I', 0, 0};

/* One decoded entry of the ESP32 module's drone list (drone_t). */
struct Summary {
    uint8_t source;  // SRC_ESP_BLE or SRC_ESP_WIFI
    Kind kind;       // RemoteId or WifiOui
    int8_t rssi;     // dBm
    uint8_t mac[6];
    char id[21];
    int32_t lat, lon, op_lat, op_lon;
};

class Tracker {
   public:
    static constexpr size_t kMaxEntities = 12;
    static constexpr size_t kMaxRid = 8;

    static constexpr size_t kRecent = 16; /* recently evicted addresses remembered (Tracker::create) */

    enum Event : uint8_t {
        EV_CONFIRMED,  // new detection, announce (beep, log)
        EV_LOCATION,   // position changed, rate limited
        EV_DROPPED,    // expired or evicted (only for confirmed ones); why: drop_why()
    };
    /* Why an EV_DROPPED: valid while it is being handled. */
    enum DropWhy : uint8_t {
        DROP_EXPIRED,  // not heard for kExpireSeconds
        DROP_EVICTED,  // the table was full and it was the stalest (it may still be on the air)
        DROP_CLASS,    // a signature class change (drop_kind)
        DROP_GONE,     // its source said so (pattern scan, interference scan)
        DROP_STOP,     // the log stopped while it was listed (not removed; the app's own rows)
    };
    static const char* drop_why_name(uint8_t why);
    DropWhy drop_why() const { return drop_why_; }
    using Notify = void (*)(void* ctx, Event ev, const Entity& e);

    /* Container interface for RecentEntriesView: entities in display order,
     * newest first. Pointers stay valid only until the next ingest/tick. */
    using value_type = Entity;
    using reference = Entity&;
    using const_reference = const Entity&;
    using iterator = Entity*;
    using const_iterator = const Entity*;
    Entity* begin() { return items_; }
    Entity* end() { return items_ + count_; }
    const Entity* begin() const { return items_; }
    const Entity* end() const { return items_ + count_; }
    bool empty() const { return count_ == 0; }
    size_t size() const { return count_; }
    Entity& front() { return items_[0]; }
    Entity& back() { return items_[count_ ? count_ - 1 : 0]; }

    void set_notify(Notify fn, void* ctx) {
        notify_ = fn;
        notify_ctx_ = ctx;
    }

    /* msg: a 25 byte message or a message pack (odid_apply). Returns the
     * entity or nullptr (invalid message, table full). */
    Entity* ingest_odid(uint8_t source, const uint8_t* mac, int8_t rssi, bool rssi_dbm, const uint8_t* msg, size_t len, uint32_t now);
    Entity* ingest_summary(const Summary& s, uint32_t now);
    /* A confirmed RF track (level: dB over floor); created confirmed, no
     * event. drop_rf() removes it without an event. */
    Entity* ingest_rf(const uint8_t* mac, int8_t level, uint32_t now);
    void drop_rf(const uint8_t* mac);
    /* A pattern scan detection (rssi: dBFS), created unconfirmed: the caller
     * fills in label / cls / pat, then announce()s it (EV_CONFIRMED).
     * drop_pattern() removes it with EV_DROPPED. */
    Entity* ingest_pattern(const uint8_t* mac, int8_t rssi, uint32_t now);
    void announce(Entity& e, uint32_t now) {
        if (!e.confirmed) confirm(e, now);
    }
    void drop_pattern(const uint8_t* mac);
    /* A drone maker BLE advert (not Remote ID): created confirmed (EV_CONFIRMED),
     * dropped by tick() after kExpireSeconds like any other (EV_DROPPED). */
    Entity* ingest_vendor(const uint8_t* mac, int8_t rssi, uint8_t why, uint8_t pdu_type, const uint8_t* name, size_t name_len, uint32_t now);
    /* One DJI DroneID burst: the DroneID entity (kDroneIdMac, label
     * "DroneID?"), created confirmed (EV_CONFIRMED) on the first, dropped by
     * tick() after kExpireSeconds without one (EV_DROPPED). */
    Entity* ingest_droneid(const DroneIdBurst& b, uint32_t now);
    /* A signature rule hit (source SRC_BLE: an advert, mac its address; SRC_RF:
     * NB energy, rssi = dB over floor): created confirmed (EV_CONFIRMED) with
     * label, kind tag and rule, dropped by tick() after kExpireSeconds
     * (EV_DROPPED). An address already held by another kind is left alone. */
    Entity* ingest_sig(const uint8_t* mac, uint8_t source, int8_t rssi, uint8_t rule, uint8_t kind, const char* tag, const char* label, uint32_t now, Kind as = Kind::Sig);
    /* Remote ID emitters heard within the last `seconds`. */
    bool rid_within(uint32_t now, uint32_t seconds) const;

    /* Once a second: ageing, RSSI history, distances, delayed announcements. */
    void tick(uint32_t now, bool own_valid, int32_t own_lat, int32_t own_lon);

    Entity* find_key(Entity::Key key);
    const Entity* find_addr(const uint8_t* mac) {
        return find_mac(mac);
    }
    /* Every entity of this kind goes now, with EV_DROPPED for confirmed ones
     * (a signature class change: their rows must name the class they came
     * from). */
    void drop_kind(Kind kind, DropWhy why = DROP_CLASS);
    /* Evictions so far, and how many of them came back within
     * kExpireSeconds (listed again without a NEW). */
    uint32_t evictions() const { return evictions_; }
    uint32_t requeues() const { return requeues_; }
    /* RID emitters heard within kStaleSeconds. */
    size_t active_rid() const;
    void clear();

   private:
    /* An evicted confirmed entity: heard again within kExpireSeconds it is
     * listed without a fresh EV_CONFIRMED (a crowd larger than the table
     * otherwise turns into NEW / DROP pairs every second). */
    struct Recent {
        uint8_t mac[6];
        Kind kind;
        bool used;
        uint32_t evicted_s, first_s;
    };
    Entity items_[kMaxEntities]{};
    Recent recent_[kRecent]{};
    DropWhy drop_why_{DROP_EXPIRED};
    uint32_t evictions_{0}, requeues_{0};
    RidRecord rids_[kMaxRid]{};
    size_t count_{0};
    uint8_t rid_used_{0};  // bit n: rids_[n] in use
    Entity::Key next_key_{1};
    Notify notify_{nullptr};
    void* notify_ctx_{nullptr};

    Entity* find_mac(const uint8_t* mac);
    Entity* find_uas(const char* uas_id, const Entity* except);
    Entity* create(Kind kind, const uint8_t* mac, uint32_t now);
    bool attach_rid(Entity& e);
    void remove(size_t index, bool report, DropWhy why = DROP_EXPIRED);
    bool evict_oldest(bool rid_only, uint32_t now, bool for_rid);
    void touch(Entity& e, uint8_t source, int8_t rssi, bool rssi_dbm, uint32_t now);
    Entity* merge(Entity* a, Entity* b, uint8_t fresh);
    void confirm(Entity& e, uint32_t now);
    void location(Entity& e, uint32_t now, bool first_fix = false);
    void emit(Event ev, const Entity& e) {
        if (notify_) notify_(notify_ctx_, ev, e);
    }
};

/* Name of an entity for lists: UAS ID (hex for binary ID types), else the
 * label, else the MAC. */
void entity_name(const Entity& e, char* out, size_t cap);

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_MODEL_HPP__*/
