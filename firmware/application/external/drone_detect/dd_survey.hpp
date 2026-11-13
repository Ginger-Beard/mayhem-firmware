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

/* Survey log of the drone detect app: what was received but matched no
 * detector or signature rule, kept in small fixed tables and written as rate
 * limited CSV rows, so that signatures can be written from the log later.
 *   BleSurvey - BLE advertisers matching no rule: first seen, then a summary
 *               every period while still heard (and when evicted);
 *   NrgSurvey - energy in a sweep dwell that no confident drone classifier
 *               took (no label or a low confidence one): narrow peaks and wide
 *               humps, kept across dwells, reported once seen in two dwells,
 *               then a summary every period.
 * Integer only, no UI or ChibiOS dependency (host testable). Times in ms
 * (uptime). */

#ifndef __DD_SURVEY_HPP__
#define __DD_SURVEY_HPP__

#include <cstddef>
#include <cstdint>

#include "dd_rf.hpp"

namespace ui::external_app::drone_detect {

constexpr size_t kSurveyBleSlots = 32; /* advertisers kept (by address); the one heard longest ago goes */
constexpr size_t kSurveyNrgSlots = 16;
constexpr uint32_t kSurveyPeriodMs = 300000; /* summary rows every 5 min per entry still heard */
constexpr size_t kSurveySeen = 64;           /* first-seen rows remembered (address / energy hashes): no second one this session */
/* Row budget of the survey (token bucket): kSurveyBurst rows at once, then
 * one every kSurveyRefillMs. A row that does not fit is dropped and counted. */
constexpr uint32_t kSurveyBurst = 30;
constexpr uint32_t kSurveyRefillMs = 1000;
constexpr size_t kNrgPerDwell = 6;       /* energy events taken from one dwell at most */
constexpr int16_t kNrgMinMaxhCdb = 1000; /* a peak: max hold >= 10 dB over floor ... */
constexpr int16_t kNrgMinMeanCdb = 400;  /* ... or mean >= 4 dB over floor */

/* The last kSurveySeen hashes that got a first-seen row (a ring): an entry
 * evicted and heard again is not "new" again. */
class SeenSet {
   public:
    bool contains(uint16_t h) const;
    void add(uint16_t h);
    void clear() {
        n_ = next_ = 0;
    }

   private:
    uint16_t h_[kSurveySeen]{};
    uint8_t n_{0}, next_{0};
};

class RowBudget {
   public:
    bool take(uint32_t now_ms);
    void refund() {
        if (tokens_ < kSurveyBurst) tokens_++;
    }
    uint32_t dropped() const { return dropped_; }

   private:
    uint32_t tokens_{kSurveyBurst};
    uint32_t last_ms_{0};
    bool started_{false};
    uint32_t dropped_{0};
};

struct BleSeen {
    uint8_t addr[6];
    uint8_t pdu;   /* PDU header byte 0 of the first frame: type, TxAdd in bit 6 */
    uint8_t pdus;  /* bit n: PDU type n seen (ADV_IND 0, ADV_NONCONN_IND 2, SCAN_RSP 4, ADV_SCAN_IND 6) */
    uint8_t chans; /* bit 0/1/2: channel 37/38/39 */
    uint16_t cid;  /* manufacturer data company ID, valid with has_cid */
    uint16_t uuid[2];
    uint16_t sd_uuid;
    uint8_t nuuid;
    uint8_t mfr_n; /* manufacturer data bytes after the company ID kept (<= 8) */
    uint8_t mfr[8];
    uint8_t sd_n; /* service data bytes after the UUID kept (<= 8) */
    uint8_t sd_data[8];
    uint8_t adv_len;
    bool has_cid, has_sd, has_name, used;
    int8_t rssi_min, rssi_max;
    uint16_t count;      /* frames since the first */
    uint16_t count_emit; /* ... at the last row */
    uint16_t gap_min_ms; /* shortest time between two frames forwarded: the M4 passes an unchanged advert at most once a second, so < 1 s only for changing payloads */
    uint32_t first_ms, last_ms, emit_ms;
};

class BleSurvey {
   public:
    enum Result : uint8_t {
        SEEN,   /* known, updated */
        NEW,    /* new entry, never reported this session: write a first-seen row */
        REPEAT, /* new entry, but its address had its first-seen row already (marked, no row) */
    };
    void clear();
    /* One frame (any PDU type; the entry is the address, SCAN_RSP data
     * fills in what the advert lacked). rssi dBFS. Returns what happened;
     * entry() the entry. */
    Result add(const uint8_t* addr, uint8_t pdu, uint8_t chan, int8_t rssi, const uint8_t* adv, size_t len, uint32_t now_ms);
    BleSeen& entry() { return *last_; }
    /* Slot i, nullptr when empty (the target picker). */
    const BleSeen* slot(size_t i) const { return i < kSurveyBleSlots && slots_[i].used ? &slots_[i] : nullptr; }
    /* After add(): an evicted entry with frames since its last row (summarise
     * it), once; nullptr if none. */
    const BleSeen* take_evicted() {
        const bool e = evicted_news_;
        evicted_news_ = false;
        return e ? &evicted_ : nullptr;
    }
    /* An entry whose summary is due (heard since its last row, period
     * passed), marked as reported; nullptr if none. */
    const BleSeen* due(uint32_t now_ms, uint32_t period_ms);
    /* Every entry with frames since its last row, one per call (final rows at the end). */
    const BleSeen* pending();
    size_t used() const;
    void mark(BleSeen& e, uint32_t now_ms) {
        e.emit_ms = now_ms;
        e.count_emit = e.count;
    }

   private:
    BleSeen slots_[kSurveyBleSlots]{};
    BleSeen evicted_{};
    BleSeen* last_{slots_};
    SeenSet seen_{};
    bool evicted_news_{false};

    static void fill(BleSeen& e, const uint8_t* adv, size_t len);
};

struct NrgSeen {
    uint32_t f_khz;
    uint16_t bw_khz;
    uint8_t band;
    bool used;
    int8_t db_max;     /* max hold over floor, dB */
    int8_t mean_max;   /* mean over floor, dB */
    uint8_t duty_max;  /* % of a dwell's looks */
    uint8_t cls, conf; /* overlapping segment's class and confidence (RC_NONE: none) */
    bool reported;
    uint16_t dwells, dwells_emit;
    uint32_t first_ms, last_ms, emit_ms;
};

/* One energy event of a dwell. */
struct NrgEvent {
    uint32_t f_khz;
    uint16_t bw_khz;
    int16_t maxh_cdb, mean_cdb;
    uint8_t duty, cls, conf;
};

/* Energy events of the dwell just finished (after RfEngine::finish()):
 * runs of points with max hold >= kNrgMinMaxhCdb or mean >= kNrgMinMeanCdb
 * over the floor, not covered by a confidently labelled segment or a
 * confirmed track of medium or high confidence; the strongest kNrgPerDwell.
 * Returns how many. */
size_t nrg_extract(const RfEngine& rf, NrgEvent* out, size_t max);
uint16_t nrg_hash(uint8_t band, uint32_t f_khz);

class NrgSurvey {
   public:
    void clear();
    /* An event of a dwell of `band`; returns the entry when a row is due
     * (seen in a second dwell for the first time: "new"), else nullptr. */
    NrgSeen* add(uint8_t band, const NrgEvent& ev, uint32_t now_ms);
    const NrgSeen* due(uint32_t now_ms, uint32_t period_ms);
    const NrgSeen* pending();
    size_t used() const;
    const NrgSeen* slot(size_t i) const { return i < kSurveyNrgSlots && slots_[i].used ? &slots_[i] : nullptr; }
    static void mark(NrgSeen& e, uint32_t now_ms) {
        e.emit_ms = now_ms;
        e.dwells_emit = e.dwells;
        e.reported = true;
    }

   private:
    NrgSeen slots_[kSurveyNrgSlots]{};
    SeenSet seen_{};
};

} /* namespace ui::external_app::drone_detect */

#endif /*__DD_SURVEY_HPP__*/
