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

#include "sigdet_survey.hpp"
#include "sigdet_sig.hpp"

#include <cstring>

namespace ui::external_app::signature_detect {

bool RowBudget::take(uint32_t now_ms) {
    if (!started_) {
        started_ = true;
        last_ms_ = now_ms;
    }
    const uint32_t add = (now_ms - last_ms_) / kSurveyRefillMs;
    if (add) {
        tokens_ = tokens_ + add > kSurveyBurst ? kSurveyBurst : tokens_ + add;
        last_ms_ += add * kSurveyRefillMs;
    }
    if (!tokens_) {
        dropped_++;
        return false;
    }
    tokens_--;
    return true;
}

/* ---------------------------------------------------------------- BLE */

void BleSurvey::clear() {
    for (auto& s : slots_) s = BleSeen{};
    last_ = slots_;
    seen_.clear();
    evicted_news_ = false;
}

size_t BleSurvey::used() const {
    size_t n = 0;
    for (const auto& s : slots_) n += s.used;
    return n;
}

namespace {

uint16_t hash16(const uint8_t* d, size_t n, uint32_t seed) {
    uint32_t h = 2166136261u ^ seed;
    for (size_t i = 0; i < n; i++) h = (h ^ d[i]) * 16777619u;
    return static_cast<uint16_t>(h ^ (h >> 16));
}

}  // namespace

bool SeenSet::contains(uint16_t h) const {
    for (size_t i = 0; i < n_; i++)
        if (h_[i] == h) return true;
    return false;
}

void SeenSet::add(uint16_t h) {
    if (contains(h)) return;
    h_[next_] = h;
    next_ = static_cast<uint8_t>((next_ + 1) % kSurveySeen);
    if (n_ < kSurveySeen) n_++;
}

/* What the entry does not have yet, from this frame's AdvData. */
void BleSurvey::fill(BleSeen& e, const uint8_t* adv, size_t len) {
    AdvInfo a;
    adv_parse(adv, len, a);
    if (len > e.adv_len) e.adv_len = static_cast<uint8_t>(len);
    if (a.mfr && !e.has_cid) {
        e.has_cid = true;
        e.cid = static_cast<uint16_t>(a.mfr[0] | (a.mfr[1] << 8));
        e.mfr_n = static_cast<uint8_t>(a.mfr_len - 2u > 8u ? 8u : a.mfr_len - 2u);
        memcpy(e.mfr, a.mfr + 2, e.mfr_n);
    }
    for (size_t k = 0; k < a.nuuid && e.nuuid < 2; k++) {
        bool have = false;
        for (size_t j = 0; j < e.nuuid; j++) have |= e.uuid[j] == a.uuid[k];
        if (!have) e.uuid[e.nuuid++] = a.uuid[k];
    }
    if (a.nsd && !e.has_sd) {
        e.has_sd = true;
        e.sd_uuid = a.sd_uuid[0];
        e.sd_n = static_cast<uint8_t>(a.sd_len[0] > 8 ? 8 : a.sd_len[0]);
        memcpy(e.sd_data, a.sd[0], e.sd_n);
    }
    if (a.name) e.has_name = true;
}

BleSurvey::Result BleSurvey::add(const uint8_t* addr, uint8_t pdu, uint8_t chan, int8_t rssi, const uint8_t* adv, size_t len, uint32_t now_ms) {
    BleSeen* e = nullptr;
    BleSeen* spare = nullptr;
    for (auto& s : slots_) {
        if (!s.used) {
            if (!spare) spare = &s;
            continue;
        }
        if (memcmp(s.addr, addr, 6) == 0) {
            e = &s;
            break;
        }
    }
    Result res = SEEN;
    if (!e) {
        if (!spare) {
            /* Full: the one heard longest ago goes, of two equally old the
             * weaker; its summary first (take_evicted) when it has news. */
            spare = &slots_[0];
            for (auto& s : slots_) {
                const uint32_t a = now_ms - s.last_ms, b = now_ms - spare->last_ms;
                if (a > b || (a == b && s.rssi_max < spare->rssi_max)) spare = &s;
            }
            if (spare->count != spare->count_emit) {
                evicted_ = *spare;
                evicted_news_ = true;
            }
        }
        e = spare;
        *e = BleSeen{};
        e->used = true;
        memcpy(e->addr, addr, 6);
        e->pdu = pdu;
        e->rssi_min = e->rssi_max = rssi;
        e->first_ms = now_ms;
        e->gap_min_ms = 0xFFFF;
        const uint16_t h = hash16(addr, 6, 0);
        if (seen_.contains(h)) {
            res = REPEAT;
        } else {
            res = NEW;
            seen_.add(h);
        }
    } else {
        const uint32_t gap = now_ms - e->last_ms;
        if (gap < e->gap_min_ms) e->gap_min_ms = static_cast<uint16_t>(gap > 0xFFFE ? 0xFFFE : gap);
        if (rssi < e->rssi_min) e->rssi_min = rssi;
        if (rssi > e->rssi_max) e->rssi_max = rssi;
    }
    e->pdus |= static_cast<uint8_t>(1u << (pdu & 7));
    fill(*e, adv, len);
    if (chan >= 37 && chan <= 39) e->chans |= static_cast<uint8_t>(1u << (chan - 37));
    if (e->count < 0xFFFF) e->count++;
    e->last_ms = now_ms;
    last_ = e;
    return res;
}

const BleSeen* BleSurvey::due(uint32_t now_ms, uint32_t period_ms) {
    for (auto& s : slots_)
        if (s.used && s.count != s.count_emit && now_ms - s.emit_ms >= period_ms) {
            mark(s, now_ms);
            return &s;
        }
    return nullptr;
}

const BleSeen* BleSurvey::pending() {
    for (auto& s : slots_)
        if (s.used && s.count != s.count_emit) {
            mark(s, s.last_ms);
            return &s;
        }
    return nullptr;
}

/* ---------------------------------------------------------------- energy */

namespace {

uint32_t dist(uint32_t a, uint32_t b) {
    return a > b ? a - b : b - a;
}

}  // namespace

size_t nrg_extract(const RfEngine& rf, NrgEvent* out, size_t max) {
    const size_t n = rf.points();
    const uint16_t looks = rf.looks();
    if (!n || !looks || !max) return 0;
    const uint8_t band = rf.band();
    const uint32_t lo = rf.point_khz(0);
    const uint32_t step = n > 1 ? (rf.point_khz(n - 1) - lo) / (n - 1) : 1;
    size_t count = 0;
    size_t x = 0;
    while (x < n) {
        /* A max hold peak alone is often one noise spike of hundreds of
         * looks: it also needs 2 looks over the occupancy threshold. */
        auto hot = [&](size_t i) { return (rf.maxh_at(i) * 50 >= kNrgMinMaxhCdb && rf.occ_at(i) >= 2) || rf.mean_at(i) >= kNrgMinMeanCdb; };
        if (!hot(x)) {
            x++;
            continue;
        }
        const size_t a = x;
        size_t peak = x;
        int16_t mean_max = -32768;
        uint8_t occ_max = 0;
        while (x < n && hot(x)) {
            if (rf.maxh_at(x) > rf.maxh_at(peak)) peak = x;
            if (rf.mean_at(x) > mean_max) mean_max = rf.mean_at(x);
            if (rf.occ_at(x) > occ_max) occ_max = rf.occ_at(x);
            x++;
        }
        const size_t b = x - 1;
        NrgEvent ev{};
        ev.f_khz = rf.point_khz(peak);
        ev.bw_khz = static_cast<uint16_t>((b - a + 1) * step > 65535 ? 65535 : (b - a + 1) * step);
        ev.maxh_cdb = static_cast<int16_t>(rf.maxh_at(peak) * 50);
        ev.mean_cdb = mean_max;
        ev.duty = static_cast<uint8_t>(occ_max * 100u / looks > 100 ? 100 : occ_max * 100u / looks);
        ev.cls = RC_NONE;
        ev.conf = CONF_NONE;
        /* A segment over it: its class; confidently named ones are no miss. */
        bool skip = false;
        for (size_t i = 0; i < rf.seg_count(); i++) {
            const RfSeg& s = rf.seg(i);
            if (b < s.lo || a > s.hi) continue;
            ev.cls = s.cls;
            ev.conf = s.conf;
            if (rf_reported(s.cls) && s.conf >= CONF_MED) skip = true;
        }
        for (size_t i = 0; i < RfEngine::kMaxTracks && !skip; i++) {
            const RfTrack& t = rf.tracks()[i];
            if (t.uid && t.band == band && t.confirmed && t.conf >= CONF_MED && t.miss == 0 &&
                dist(t.f_khz, ev.f_khz) <= (t.bw_khz / 2 > step ? t.bw_khz / 2 : step))
                skip = true;
        }
        /* BLE advertising channels: the BLE survey has them. */
        static constexpr uint32_t kBleAdv[3] = {2402000, 2426000, 2480000};
        if (band == BAND_24)
            for (const uint32_t c : kBleAdv)
                if (dist(ev.f_khz, c) <= 1200) skip = true;
        if (skip) continue;
        /* Keep the strongest `max`. */
        if (count < max) {
            out[count++] = ev;
        } else {
            size_t weakest = 0;
            for (size_t i = 1; i < count; i++)
                if (out[i].maxh_cdb < out[weakest].maxh_cdb) weakest = i;
            if (ev.maxh_cdb > out[weakest].maxh_cdb) out[weakest] = ev;
        }
    }
    return count;
}

/* Band and 1 MHz cell of an energy event. */
uint16_t nrg_hash(uint8_t band, uint32_t f_khz) {
    const uint32_t cell = f_khz / 1000;
    const uint8_t d[5] = {band, static_cast<uint8_t>(cell), static_cast<uint8_t>(cell >> 8), static_cast<uint8_t>(cell >> 16), 0x4E};
    return hash16(d, sizeof(d), 1);
}

void NrgSurvey::clear() {
    for (auto& s : slots_) s = NrgSeen{};
    seen_.clear();
}

size_t NrgSurvey::used() const {
    size_t n = 0;
    for (const auto& s : slots_) n += s.used;
    return n;
}

NrgSeen* NrgSurvey::add(uint8_t band, const NrgEvent& ev, uint32_t now_ms) {
    NrgSeen* e = nullptr;
    NrgSeen* spare = nullptr;
    for (auto& s : slots_) {
        if (!s.used) {
            if (!spare) spare = &s;
            continue;
        }
        const uint32_t tol = (s.bw_khz > ev.bw_khz ? s.bw_khz : ev.bw_khz) / 2u + 50u;
        if (s.band == band && dist(s.f_khz, ev.f_khz) <= tol) {
            e = &s;
            break;
        }
    }
    auto db = [](int16_t cdb) { return static_cast<int8_t>(cdb / 100 > 127 ? 127 : (cdb / 100 < -127 ? -127 : cdb / 100)); };
    if (!e) {
        if (!spare) {
            /* Full: the one seen longest ago goes (no row: it was either
             * reported already or seen in one dwell only). */
            spare = &slots_[0];
            for (auto& s : slots_)
                if (now_ms - s.last_ms > now_ms - spare->last_ms) spare = &s;
        }
        e = spare;
        *e = NrgSeen{};
        e->used = true;
        e->band = band;
        e->f_khz = ev.f_khz;
        e->bw_khz = ev.bw_khz;
        e->first_ms = now_ms;
        e->db_max = db(ev.maxh_cdb);
        e->mean_max = db(ev.mean_cdb);
        /* Reported before (evicted since): a summary later, no second "new". */
        if (seen_.contains(nrg_hash(band, ev.f_khz))) {
            e->reported = true;
            e->emit_ms = now_ms;
            e->dwells_emit = 1;
        }
    }
    if (db(ev.maxh_cdb) >= e->db_max) {
        e->db_max = db(ev.maxh_cdb);
        e->f_khz = ev.f_khz; /* the strongest sighting's centre */
    }
    if (ev.bw_khz > e->bw_khz) e->bw_khz = ev.bw_khz;
    if (db(ev.mean_cdb) > e->mean_max) e->mean_max = db(ev.mean_cdb);
    if (ev.duty > e->duty_max) e->duty_max = ev.duty;
    e->cls = ev.cls;
    e->conf = ev.conf;
    if (e->dwells < 0xFFFF) e->dwells++;
    e->last_ms = now_ms;
    if (e->reported || e->dwells < 2) return nullptr;
    seen_.add(nrg_hash(band, e->f_khz));
    return e;
}

const NrgSeen* NrgSurvey::due(uint32_t now_ms, uint32_t period_ms) {
    for (auto& s : slots_)
        if (s.used && s.reported && s.dwells != s.dwells_emit && now_ms - s.emit_ms >= period_ms) {
            mark(s, now_ms);
            return &s;
        }
    return nullptr;
}

const NrgSeen* NrgSurvey::pending() {
    for (auto& s : slots_)
        if (s.used && s.reported && s.dwells != s.dwells_emit) {
            mark(s, s.last_ms);
            return &s;
        }
    return nullptr;
}

} /* namespace ui::external_app::signature_detect */
