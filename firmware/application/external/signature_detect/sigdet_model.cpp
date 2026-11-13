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

#include "sigdet_model.hpp"
#include "sigdet_fmt.hpp"

#include <cstddef>
#include <cstring>

namespace ui::external_app::signature_detect {

namespace {

bool has_id(const RidRecord& r) {
    return (r.msg_seen & MSG_BASIC) || r.uas_id[0];
}

int8_t max8(int8_t a, int8_t b) {
    return a > b ? a : b;
}

/* Copies the message groups in mask from s to d (one block each, see
 * RidRecord), then fills what d lacks from data that came without message
 * bits (ESP32 summaries). */
void copy_groups(RidRecord& d, const RidRecord& s, uint8_t mask) {
    struct Group {
        uint8_t bit, begin, end;
    };
    static constexpr Group kGroups[] = {
        {MSG_LOCATION, offsetof(RidRecord, lat_e7), offsetof(RidRecord, op_lat_e7)},
        {MSG_SYSTEM, offsetof(RidRecord, op_lat_e7), offsetof(RidRecord, uas_id)},
        {MSG_BASIC, offsetof(RidRecord, uas_id), offsetof(RidRecord, self_id)},
        {MSG_SELF, offsetof(RidRecord, self_id), offsetof(RidRecord, operator_id)},
        {MSG_OPERATOR, offsetof(RidRecord, operator_id), offsetof(RidRecord, mac)},
    };
    for (const auto& g : kGroups)
        if (mask & g.bit) memcpy(reinterpret_cast<uint8_t*>(&d) + g.begin, reinterpret_cast<const uint8_t*>(&s) + g.begin, g.end - g.begin);
    d.msg_seen |= mask & s.msg_seen;

    if (!has_id(d) && s.uas_id[0]) memcpy(d.uas_id, s.uas_id, sizeof(d.uas_id));
    if (!odid_pos_valid(d.lat_e7, d.lon_e7)) {
        d.lat_e7 = s.lat_e7;
        d.lon_e7 = s.lon_e7;
    }
    if (!odid_pos_valid(d.op_lat_e7, d.op_lon_e7)) {
        d.op_lat_e7 = s.op_lat_e7;
        d.op_lon_e7 = s.op_lon_e7;
    }
}

}  // namespace

char Entity::trend() const {
    const int8_t recent = max8(hist[0], hist[1]);
    const int8_t before = max8(max8(hist[2], hist[3]), hist[4]);
    if (recent == kNoRssi || before == kNoRssi) return ' ';
    if (recent > before + 3) return '^';
    if (recent < before - 3) return 'v';
    return '=';
}

Entity* Tracker::find_key(Entity::Key key) {
    for (auto& e : *this)
        if (e.id == key) return &e;
    return nullptr;
}

Entity* Tracker::find_mac(const uint8_t* mac) {
    for (auto& e : *this)
        if (memcmp(e.mac, mac, 6) == 0) return &e;
    return nullptr;
}

Entity* Tracker::find_uas(const char* uas_id, const Entity* except) {
    for (auto& e : *this)
        if (&e != except && e.rid && has_id(*e.rid) && memcmp(e.rid->uas_id, uas_id, kUasIdMatch) == 0) return &e;
    return nullptr;
}

size_t Tracker::active_rid() const {
    size_t n = 0;
    for (const auto& e : *this)
        if (e.rid && !e.stale) n++;
    return n;
}

void Tracker::clear() {
    count_ = 0;
    rid_used_ = 0;
}

bool Tracker::attach_rid(Entity& e) {
    for (size_t i = 0; i < kMaxRid; i++) {
        if (!(rid_used_ & (1u << i))) {
            rid_used_ |= 1u << i;
            rids_[i] = RidRecord{};
            e.rid = &rids_[i];
            e.kind = Kind::RemoteId;
            return true;
        }
    }
    return false;
}

void Tracker::remove(size_t index, bool report) {
    Entity& e = items_[index];
    if (report && e.confirmed) emit(EV_DROPPED, e);
    if (e.rid) rid_used_ &= ~(1u << (e.rid - rids_));
    for (size_t i = index; i + 1 < count_; i++) items_[i] = items_[i + 1];
    count_--;
}

void Tracker::evict_oldest(bool rid_only) {
    size_t victim = count_;
    for (size_t i = 0; i < count_; i++) {
        if (rid_only && !items_[i].rid) continue;
        if (victim == count_ || items_[i].last_s < items_[victim].last_s) victim = i;
    }
    if (victim < count_) remove(victim, true);
}

Entity* Tracker::create(Kind kind, const uint8_t* mac, uint32_t now) {
    if (count_ == kMaxEntities) evict_oldest(false);
    if (kind == Kind::RemoteId && rid_used_ == (1u << kMaxRid) - 1) evict_oldest(true);
    if (count_ == kMaxEntities) return nullptr;

    for (size_t i = count_; i > 0; i--) items_[i] = items_[i - 1];
    count_++;
    Entity& e = items_[0];
    e = Entity{};
    e.id = next_key_++;
    if (next_key_ == Entity::invalid_key) next_key_ = 1;
    e.kind = kind;
    e.first_s = e.last_s = now;
    memcpy(e.mac, mac, 6);
    for (auto& h : e.hist) h = kNoRssi;
    if (kind == Kind::RemoteId && !attach_rid(e)) {
        remove(0, false);
        return nullptr;
    }
    return &e;
}

void Tracker::touch(Entity& e, uint8_t source, int8_t rssi, bool rssi_dbm, uint32_t now) {
    e.last_s = now;
    e.stale = false;
    e.sources |= source;
    if (e.frames < 0xFFFF) e.frames++;

    /* The on-board level (dBFS) and the ESP32's (dBm) do not compare: once the
     * on-board receiver hears the drone, its level is the one shown. */
    if (rssi_dbm && (e.sources & SRC_ONBOARD)) return;
    if (rssi_dbm != e.rssi_dbm) {
        e.rssi_dbm = rssi_dbm;
        e.rssi_max = kNoRssi;
        for (auto& h : e.hist) h = kNoRssi;
    }
    e.rssi = rssi;
    e.rssi_max = max8(e.rssi_max, rssi);
    e.hist[0] = max8(e.hist[0], rssi);
}

/* a and b are the same drone (same UAS ID). b has just been updated with the
 * message types in fresh (0 for a summary). The older entity stays (its key is
 * what list and detail view refer to), the other one is folded into it. */
Entity* Tracker::merge(Entity* a, Entity* b, uint8_t fresh) {
    Entity* keep = a->id < b->id ? a : b;
    Entity* gone = keep == a ? b : a;
    const uint8_t newer = gone == b ? fresh : 0;
    copy_groups(*keep->rid, *gone->rid, gone->rid->msg_seen & (newer | ~keep->rid->msg_seen));

    if (newer) memcpy(keep->mac, gone->mac, 6); /* follow a rotated BLE address */
    keep->sources |= gone->sources;
    keep->confirmed = keep->confirmed || gone->confirmed;
    if (gone->first_s < keep->first_s) keep->first_s = gone->first_s;
    if (gone->last_s > keep->last_s) keep->last_s = gone->last_s;
    if (gone->loc_report_s > keep->loc_report_s) keep->loc_report_s = gone->loc_report_s;
    keep->frames = (keep->frames + gone->frames > 0xFFFF) ? 0xFFFF : keep->frames + gone->frames;
    if (gone->rssi_dbm == keep->rssi_dbm) {
        if (gone == b) keep->rssi = gone->rssi;
        keep->rssi_max = max8(keep->rssi_max, gone->rssi_max);
        for (size_t i = 0; i < kRssiHistory; i++) keep->hist[i] = max8(keep->hist[i], gone->hist[i]);
    }

    const Entity::Key key = keep->id;
    remove(static_cast<size_t>(gone - items_), false);
    return find_key(key);
}

void Tracker::confirm(Entity& e, uint32_t now) {
    e.confirmed = true;
    e.loc_report_s = now;
    emit(EV_CONFIRMED, e);
}

/* first_fix: the drone or operator position just became known after the
 * announcement. Reported at once; later updates stay rate limited. */
void Tracker::location(Entity& e, uint32_t now, bool first_fix) {
    if (!e.confirmed || (!first_fix && now - e.loc_report_s < kLocationLogSeconds)) return;
    e.loc_report_s = now;
    emit(EV_LOCATION, e);
}

Entity* Tracker::ingest_odid(uint8_t source, const uint8_t* mac, int8_t rssi, bool rssi_dbm, const uint8_t* msg, size_t len, uint32_t now) {
    RidRecord probe{};
    const uint32_t valid = odid_apply(probe, msg, len);
    if (!valid) return nullptr;

    Entity* e = find_mac(mac);
    if (!e && (valid & MSG_BASIC)) {
        /* An unknown address announcing a known UAS ID: the drone rotated its
         * BLE address, or it is the same drone heard by another radio. */
        e = find_uas(probe.uas_id, nullptr);
        if (e) memcpy(e->mac, mac, 6);
    }
    if (!e)
        e = create(Kind::RemoteId, mac, now);
    else if (!e->rid && !attach_rid(*e))
        return nullptr;
    if (!e) return nullptr;

    const bool had_pos = odid_pos_valid(e->rid->lat_e7, e->rid->lon_e7);
    const bool had_op = odid_pos_valid(e->rid->op_lat_e7, e->rid->op_lon_e7);
    const uint8_t applied = odid_apply(*e->rid, msg, len);
    touch(*e, source, rssi, rssi_dbm, now);
    if (applied & MSG_BASIC) {
        Entity* other = find_uas(e->rid->uas_id, e);
        if (other) e = merge(other, e, applied);
    }
    const bool first_fix = (!had_pos && odid_pos_valid(e->rid->lat_e7, e->rid->lon_e7)) ||
                           (!had_op && odid_pos_valid(e->rid->op_lat_e7, e->rid->op_lon_e7));
    if (!e->confirmed && has_id(*e->rid))
        confirm(*e, now);
    else if (first_fix || (applied & MSG_LOCATION))
        location(*e, now, first_fix);
    return e;
}

Entity* Tracker::ingest_summary(const Summary& s, uint32_t now) {
    const bool rid = s.kind == Kind::RemoteId;
    Entity* e = find_mac(s.mac);
    if (!e && rid && s.id[0]) e = find_uas(s.id, nullptr);
    if (!e)
        e = create(s.kind, s.mac, now);
    else if (rid && !e->rid && !attach_rid(*e))
        return nullptr;
    if (!e) return nullptr;

    touch(*e, s.source, s.rssi, true, now);
    if (!e->rid) {
        Fmt{e->label, sizeof(e->label)}.s(s.id[0] ? s.id : "drone WiFi");
        if (!e->confirmed) confirm(*e, now);
        return e;
    }

    RidRecord& r = *e->rid;
    const bool first_fix = (odid_pos_valid(s.lat, s.lon) && !odid_pos_valid(r.lat_e7, r.lon_e7)) ||
                           (odid_pos_valid(s.op_lat, s.op_lon) && !odid_pos_valid(r.op_lat_e7, r.op_lon_e7));
    bool moved = false;
    if (s.id[0] && !has_id(r)) memcpy(r.uas_id, s.id, sizeof(r.uas_id));
    if (odid_pos_valid(s.lat, s.lon) && (s.lat != r.lat_e7 || s.lon != r.lon_e7)) {
        r.lat_e7 = s.lat;
        r.lon_e7 = s.lon;
        moved = true;
    }
    if (odid_pos_valid(s.op_lat, s.op_lon)) {
        r.op_lat_e7 = s.op_lat;
        r.op_lon_e7 = s.op_lon;
    }
    if (s.id[0]) {
        Entity* other = find_uas(r.uas_id, e);
        if (other) e = merge(other, e, 0);
    }
    if (!e->confirmed && has_id(*e->rid))
        confirm(*e, now);
    else if (moved || first_fix)
        location(*e, now, first_fix);
    return e;
}

Entity* Tracker::ingest_rf(const uint8_t* mac, int8_t level, uint32_t now) {
    Entity* e = find_mac(mac);
    if (!e) e = create(Kind::Rf, mac, now);
    if (!e) return nullptr;
    e->confirmed = true;
    touch(*e, SRC_RF, level, false, now);
    return e;
}

void Tracker::drop_rf(const uint8_t* mac) {
    Entity* e = find_mac(mac);
    if (e) remove(static_cast<size_t>(e - items_), false);
}

Entity* Tracker::ingest_pattern(const uint8_t* mac, int8_t rssi, uint32_t now) {
    Entity* e = find_mac(mac);
    if (!e) e = create(Kind::Pattern, mac, now);
    if (!e) return nullptr;
    touch(*e, SRC_PAT, rssi, false, now);
    return e;
}

void Tracker::drop_pattern(const uint8_t* mac) {
    Entity* e = find_mac(mac);
    if (e) remove(static_cast<size_t>(e - items_), true);
}

Entity* Tracker::ingest_vendor(const uint8_t* mac, int8_t rssi, uint8_t why, uint8_t pdu_type, const uint8_t* name, size_t name_len, uint32_t now) {
    Entity* e = find_mac(mac);
    if (e && e->kind != Kind::BleVendor) return e; /* a Remote ID emitter's address: that entity wins */
    if (!e) e = create(Kind::BleVendor, mac, now);
    if (!e) return nullptr;
    e->why |= why;
    e->pdus |= static_cast<uint8_t>(1u << (pdu_type & 7));
    if (name && name_len) {
        /* Printable ASCII only (the name goes to the screen, CSV and JSON). */
        size_t n = 0;
        for (size_t i = 0; i < name_len && n + 1 < sizeof(e->label); i++) {
            const char c = static_cast<char>(name[i]);
            e->label[n++] = (c >= 0x20 && c < 0x7F) ? c : '?';
        }
        e->label[n] = 0;
    } else if (!e->label[0]) {
        Fmt{e->label, sizeof(e->label)}.s("DJI device");
    }
    touch(*e, SRC_BLE, rssi, false, now);
    if (!e->confirmed) confirm(*e, now);
    return e;
}

Entity* Tracker::ingest_sig(const uint8_t* mac, uint8_t source, int8_t rssi, uint8_t rule, uint8_t kind, const char* tag, const char* label, uint32_t now, Kind as) {
    Entity* e = find_mac(mac);
    if (e && e->kind != as) return nullptr;
    if (!e) e = create(as, mac, now);
    if (!e) return nullptr;
    e->why = rule;
    e->pat = kind;
    Fmt{e->cls, sizeof(e->cls)}.s(tag);
    Fmt{e->label, sizeof(e->label)}.s(label);
    touch(*e, source, rssi, false, now);
    if (!e->confirmed) confirm(*e, now);
    return e;
}

/* The '?' marks the detector as checked on recordings only (on the PC
 * against recorded real DJI bursts; the firmware port on replayed
 * recordings, never on a live drone): the app treats it as low confidence. */
Entity* Tracker::ingest_droneid(const DroneIdBurst& b, uint32_t now) {
    Entity* e = find_mac(kDroneIdMac);
    if (e && e->kind != Kind::DroneId) return nullptr;
    if (!e) e = create(Kind::DroneId, kDroneIdMac, now);
    if (!e) return nullptr;
    Fmt{e->label, sizeof(e->label)}.s("DroneID?");
    e->freq_khz = b.freq_khz;
    e->did_zc = b.zc;
    e->did_snr = b.snr;
    e->did_cfo = b.cfo;
    e->did_nsym = b.nsym;
    touch(*e, SRC_DRONEID, b.rssi, false, now);
    if (!e->confirmed) confirm(*e, now);
    return e;
}

bool Tracker::rid_within(uint32_t now, uint32_t seconds) const {
    for (const auto& e : *this)
        if (e.rid && now - e.last_s < seconds) return true;
    return false;
}

void Tracker::tick(uint32_t now, bool own_valid, int32_t own_lat, int32_t own_lon) {
    for (size_t i = count_; i-- > 0;) {
        Entity& e = items_[i];
        const uint32_t age = now - e.last_s;
        if (age >= kExpireSeconds) {
            remove(i, true);
            continue;
        }
        e.stale = age >= kStaleSeconds;
        for (size_t k = kRssiHistory - 1; k > 0; k--) e.hist[k] = e.hist[k - 1];
        e.hist[0] = kNoRssi;
        e.dist_m = (own_valid && e.rid && odid_pos_valid(e.rid->lat_e7, e.rid->lon_e7))
                       ? odid_distance_m(own_lat, own_lon, e.rid->lat_e7, e.rid->lon_e7)
                       : kNoDistance;
        if (!e.confirmed && !e.stale && now - e.first_s >= kConfirmSeconds) confirm(e, now);
    }
}

void Tracker::drop_kind(Kind kind) {
    for (size_t i = count_; i-- > 0;)
        if (items_[i].kind == kind) remove(i, true);
}

void entity_name(const Entity& e, char* out, size_t cap) {
    Fmt f{out, cap};
    if (e.rid && has_id(*e.rid)) {
        if (e.rid->binary_id)
            f.hex(reinterpret_cast<const uint8_t*>(e.rid->uas_id), 10);
        else
            f.s(e.rid->uas_id);
    } else if (e.label[0]) {
        f.s(e.label);
    } else {
        f.hex(e.mac, 6);
    }
}

} /* namespace ui::external_app::signature_detect */
