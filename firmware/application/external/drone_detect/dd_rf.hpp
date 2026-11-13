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

/* RF (energy sweep) detector of the drone detect app: integer only, fixed
 * arrays, no UI or ChibiOS dependency (host testable).
 *
 * A dwell sweeps one band (spec_an::Sweep, points in centi-dBFS) over and
 * over. Per complete sweep ("look") add_look() updates, per point:
 *   occ   looks above floor + kOcc*Cdb            (uint8)
 *   sum   (level - floor + 20 dB) in 0.5 dB units  (uint16; mean at the end)
 *   maxh  max of (level - floor) in 0.5 dB        (uint8)
 *   nhit  centres of 0.25..3.2 MHz runs above the threshold (hops)
 *   nwide centres of 3.2..8 MHz runs (oven, pieces of bursts; DD_FULL only)
 * finish() then
 *   1. turns sum into the mean level over floor (cdB) per point,
 *   2. updates the band's floor (8 pieces, lower quartile of the point
 *      means, smoothed across dwells; interpolated per point),
 *   3. extracts wide segments (runs occupied in >= 15 % of looks): centre by
 *      dB-weighted centroid, -10 / -3 dB widths, SNR, flatness (peak minus
 *      median of the central 60 %), duty (occupancy there),
 *   4. hit statistics outside segments, BLE advertising channels and fixed
 *      carriers: clusters (distinct channels), on-grid share, span, thin hits,
 *   5. classifies (segments: analog / digital video by width class (5..80
 *      MHz) and the channel tables, Wi-Fi, oven; hits: 868/915 control
 *      links and 2.4 GHz hoppers and toys by the hop grids), using the
 *      tables declared below (each entry graded, RfGrade),
 *   6. keeps tracks over dwells (candidate -> confirmed -> dropped) and
 *      reports them through the notify callback.
 * dump() writes all of it as DDSWP lines (see dd_rf.cpp for the format). */

#ifndef __DD_RF_HPP__
#define __DD_RF_HPP__

#include <cstddef>
#include <cstdint>

#include "dd_rf_tune.hpp"

namespace ui::external_app::drone_detect {

/* New bands go last: the index is in DDSTAT, the debug view's dump keys and
 * the replay tools. */
enum RfBand : uint8_t {
    BAND_24,
    BAND_58,
    BAND_53,
    BAND_12,
    BAND_915,
    BAND_868,
    BAND_51,
    BAND_X0, /* custom bands of a signature class (dd_sig.hpp BAND lines): filled in at run time */
    BAND_X1,
    BAND_COUNT
};
constexpr uint8_t BAND_FIXED = BAND_X0; /* the built-in bands (DDSTAT, Setup) */

/* BandDef::flags */
enum : uint8_t {
    BF_PEAK = 0x01,   // peak detector (else average)
    BF_ALT = 0x02,    // every second cycle only: the even ones ...
    BF_HOP = 0x04,    // hopper statistics apply
    BF_ODD = 0x08,    // ... or, with BF_ALT, the odd ones (5.3 and 5.1 take turns)
    BF_NOAV = 0x10,   // no analog FPV channels here: a sustained hump is digital video by its width, whatever its shape
    BF_NOCLS = 0x20,  // no drone classifiers (custom bands): floor and per point arrays only, no segments, hits or tracks
};

/* Whether a band with these flags is swept in this cycle. */
constexpr bool band_due(uint8_t flags, uint32_t cycle) {
    return !(flags & BF_ALT) || ((cycle & 1) != 0) == ((flags & BF_ODD) != 0);
}

struct BandDef {
    uint16_t lo_mhz, hi_mhz;
    uint16_t points;
    uint8_t dwell_cs; /* dwell, 10 ms units */
    uint8_t flags;
    char name[4];
};

/* Not const: BAND_X0 / BAND_X1 are set by the selected signature class
 * (set_custom_band); the built-in ones never change. */
extern BandDef kBands[BAND_COUNT];
/* A custom band (BAND_X0 / BAND_X1); points 0 = off. */
void set_custom_band(uint8_t band, uint16_t lo_mhz, uint16_t hi_mhz, uint16_t points, uint8_t dwell_cs, const char* name);

/* Classifier tables (dd_rf.cpp): plain data the engine only looks up, so
 * that a profile file could fill the same structures later. Every entry
 * carries its RfGrade. */

/* Hopper channel grid: base + k x step (k < count), a hit's centre within
 * tol. A band's first grid is its primary one (RfHits::on_grid: the ELRS 915
 * / 868 rule, the 2.4 GHz toy rule, as before); a further grid names a
 * hopper when it holds min_pct of the hits over min_distinct channels and
 * beats every other grid of the band by tune::kGridMarginPct. */
struct HopGrid {
    uint32_t base_khz;
    uint16_t step_khz;
    uint16_t tol_khz;
    uint8_t count;
    uint8_t band;
    uint8_t grade;
    uint8_t min_distinct;
    uint8_t min_pct;                  // named at CONF_MED (CONF_LOW for GRADE_DOC) ...
    uint8_t high_distinct, high_pct;  // ... CONF_HIGH from these (0: never)
    char name[5];
};
extern const HopGrid kHopGrids[];
extern const size_t kHopGridCount;

/* Video channel centres of one width class: offset + k x step MHz within
 * [lo, hi] (+- tol), tested on RfSeg::fm_khz only where that centre can be
 * trusted (RfEngine::video_grid: fine points, a signal narrower than a floor
 * piece; today that leaves the 5 MHz class at 2.4 GHz). A hint: it names a
 * sustained flat top and may turn a Wi-Fi-grid DV/WF into digital video,
 * never raises the confidence. */
struct VideoGrid {
    uint16_t lo_100khz, hi_100khz;
    uint16_t offset_khz, step_khz, tol_khz;
    uint8_t band;
    uint8_t width_mhz; /* tune::kWidthMhz value */
    uint8_t grade;
    char name[4];
};
extern const VideoGrid kVideoGrids[];
extern const size_t kVideoGridCount;

enum RfClass : uint8_t {
    RC_NONE,
    RC_ANALOG,   // analog FPV video
    RC_DIGITAL,  // digital FPV / DJI-like video
    RC_DIGWIFI,  // digital video or busy Wi-Fi (on the Wi-Fi grid, sustained)
    RC_CTRL,     // 868/915 MHz control / telemetry hopper (ELRS, FHSS)
    RC_HOP24,    // 2.4 GHz hopper (BT Classic looks the same)
    RC_TOY,      // 2.4 GHz few-channel toy-like emitter
    RC_WIFI,     // suppressed: Wi-Fi traffic
    RC_OVEN,     // suppressed: microwave oven
    RC_COUNT
};

enum RfConf : uint8_t {
    CONF_NONE,
    CONF_LOW,
    CONF_MED,
    CONF_HIGH
};

/* Three letter class tag for lists and logs. */
const char* rf_class_tag(uint8_t cls);
/* Shown (and kept as a track) at all. */
constexpr bool rf_reported(uint8_t cls) {
    return cls >= RC_ANALOG && cls <= RC_TOY;
}

struct RfSeg {
    uint16_t lo, hi;  // occupancy run, points
    uint16_t l10, r10, l3, r3;
    uint32_t f_khz;  // centroid
    /* Midpoint of the -10 dB edges, each interpolated between two points:
     * the centre for channel-grid tests. The dB centroid leans towards the
     * lower side when the signal has lifted one floor piece (seen in
     * simulation: up to 0.9 MHz for 20 MHz at 2.4 GHz). */
    uint32_t fm_khz;
    uint32_t w10_khz, w3_khz;
    int16_t snr;   // peak mean over floor, cdB
    uint8_t hold;  // peak max-hold over floor, 0.5 dB
    uint8_t flat;  // 0.1 dB
    uint8_t duty;  // %
    uint8_t cls, conf;
    char label[12];
};

struct RfHits {
    static constexpr size_t kMaxGrids = 4; /* hop grids of one band */
    uint16_t looks_any;                    // looks with a 0.25..3.2 MHz hit (anywhere)
    uint16_t thin;                         // < 250 kHz runs
    uint16_t hits;                         // counted hits after exclusions
    uint16_t on_grid;                      // ... on the band's primary grid (= grid_hits[0])
    uint16_t grid_hits[kMaxGrids];         /* ... on each hop grid of the band, kHopGrids order */
    uint16_t wide;                         // 3.2..8 MHz hits counted in the oven range
    uint8_t distinct;
    uint8_t wide_looks;  // looks share (%) of wide hits in the oven range
    uint32_t span_khz;
    uint32_t f_khz;  // hit-weighted centre
    uint32_t wide_spread_khz;
    uint8_t level;  // max-hold at the hits, 0.5 dB
    uint8_t cls, conf;
    char label[12];
};

/* cls..flat and f_khz..bw_khz in the order of Entity's rf_cls.. and
 * freq_khz.. (copied as blocks). */
struct RfTrack {
    uint8_t uid;  // 0 = free
    uint8_t band;
    uint8_t hits, miss;
    bool confirmed;
    int8_t level;  // dB over floor
    uint8_t cls, conf, duty, flat;
    uint32_t f_khz;
    uint32_t bw_khz;
    char label[12];
    /* Duty classes: looks of the track's dwells and the occupied share of
     * them (looks x duty / 100), halved together past 1000 looks. */
    uint16_t acc_looks, acc_occ;
    uint8_t acc_duty() const { return acc_looks ? static_cast<uint8_t>((acc_occ * 100u) / acc_looks) : 0; }
};

class RfEngine {
   public:
    static constexpr size_t kMaxPoints = 320;
    static constexpr size_t kMaxTracks = 8;

    enum Event : uint8_t {
        EV_NEW,   // candidate
        EV_CONF,  // confirmed
        EV_UPD,   // confirmed, seen again
        EV_DROP,  // confirmed, gone
    };
    using Notify = void (*)(void* ctx, Event ev, const RfTrack& t);
    using Sink = void (*)(void* ctx, const char* line);

    /* The per point working arrays of a dwell (about 3 KB). The owner
     * provides the memory (set_work) and may take it back between dwells
     * (set_work(nullptr)), e.g. while a big view needs the heap: floors and
     * tracks stay in the engine. */
    struct Work {
        int16_t floor[kMaxPoints];
        uint16_t sum[kMaxPoints]; /* sum .. nhit (.. nwide) cleared as one block */
        uint8_t occ[kMaxPoints];
        uint8_t maxh[kMaxPoints];
        uint8_t nhit[kMaxPoints];
#if DD_FULL
        uint8_t nwide[kMaxPoints];
#endif
        RfSeg segs[tune::kMaxSegs];
    };
    void set_work(Work* work) {
        w_ = work;
        looks_ = 0;
        nsegs_ = 0;
    }
    bool has_work() const { return w_ != nullptr; }

    void set_notify(Notify fn, void* ctx) {
        notify_ = fn;
        notify_ctx_ = ctx;
    }

    /* Needs the working memory (set_work); false without it. */
    bool begin(uint8_t band);
    /* One complete sweep of `points` levels (centi-dBFS). False once the
     * dwell holds kMaxLooks. */
    bool add_look(const int16_t* cdb);
    void finish();
    /* Tracks of a band no longer swept are dropped. */
    void forget_band(uint8_t band) { drop_tracks(band, 0); }
    /* A band redefined (custom bands): tracks and floor go. */
    void reset_band(uint8_t band) {
        drop_tracks(band, 0);
        floor_ok_ &= static_cast<uint16_t>(~(1u << band));
    }
    /* 2.4 GHz hopper / toy verdicts (low confidence: Bluetooth Classic, BLE
     * and Wi-Fi fragments in a home give the same hit statistics; off by
     * default in the app). Hit statistics are measured and dumped either way. */
    void set_hop24(bool on) { hop24_ = on; }

    void dump(Sink sink, void* ctx, uint32_t seq, uint32_t t_ms, uint16_t ms, uint16_t slices) const;

#ifdef DD_HOST_REPLAY
    /* Host replay of a DDSWP dump (no per look data): the dwell's arrays as
     * the dump has them (F floor, A mean, O occ, M max hold, N/W hits), so
     * finish() then runs on them as on the device. begin(band) first. */
    void load_dwell(uint16_t looks, uint16_t thin, uint16_t looks_any, const uint8_t* F, const uint8_t* A, const uint8_t* O, const uint8_t* M, const uint8_t* N, const uint8_t* W) {
        looks_ = looks;
        hits_.thin = thin;
        hits_.looks_any = looks_any;
        for (size_t x = 0; x < points_; x++) {
            w_->floor[x] = static_cast<int16_t>(F[x] * 50 - 12800 + 25);
            w_->sum[x] = static_cast<uint16_t>(A[x] * looks + looks / 2);
            w_->occ[x] = O[x];
            w_->maxh[x] = M[x];
            w_->nhit[x] = N[x];
#if DD_FULL
            w_->nwide[x] = W[x];
#else
            (void)W;
#endif
        }
        floor_ok_ |= 1u << band_;
    }
#endif

    uint8_t band() const { return band_; }
    uint16_t looks() const { return looks_; }
    uint8_t seg_count() const { return nsegs_; }
    const RfSeg& seg(size_t i) const { return w_->segs[i]; }
    const RfHits& hits() const { return hits_; }
    const RfTrack* tracks() const { return tracks_; }
    uint32_t point_khz(uint32_t x) const;
    int16_t floor_at(size_t x) const { return w_->floor[x]; }
    int16_t mean_at(size_t x) const { return static_cast<int16_t>(w_->sum[x]); } /* after finish() */
    uint8_t occ_at(size_t x) const { return w_->occ[x]; }                        /* looks above the threshold */
    uint8_t maxh_at(size_t x) const { return w_->maxh[x]; }                      /* max hold over floor, 0.5 dB */
    uint16_t points() const { return points_; }
    /* Nearest point of a frequency of the dwell's band (clamped). */
    uint32_t point_of(uint32_t f_khz) const {
        const uint32_t x = khz_to_point(f_khz);
        return points_ && x >= points_ ? points_ - 1u : x;
    }
    bool floor_valid(uint8_t band) const { return floor_ok_ & (1u << band); }

   private:
    uint8_t band_{0};
    uint16_t points_{0};
    uint16_t looks_{0};
    int16_t thr_{0};
    uint32_t lo_khz_{0};
    uint32_t q8_{0}; /* point spacing, kHz << 8 */
    uint16_t floor_ok_{0};
    uint8_t nsegs_{0};
    uint8_t next_uid_{1};
    bool hop24_{true};

    Work* w_{nullptr};
    /* A band's floors_ are written before floor_ok_ marks them valid. */
    int16_t floors_[BAND_COUNT][tune::kFloorSegs];
    RfHits hits_{};
    RfTrack tracks_[kMaxTracks]{};

    Notify notify_{nullptr};
    void* notify_ctx_{nullptr};

    void interpolate_floor();
    void update_floor(const int16_t* v, int16_t ref_from_floor);
    void find_segments();
    void measure(RfSeg& s);
    void classify(RfSeg& s);
    void hit_stats();
    void classify_hits();
    const VideoGrid* video_grid(const RfSeg& s, size_t wc) const;
    void digital(RfSeg& s, size_t wc, uint8_t conf, const VideoGrid* grid);
    const HopGrid* named_grid(uint8_t& conf) const;
    void track(uint8_t cls, uint8_t conf, uint32_t f_khz, uint32_t bw_khz, int16_t level_cdb, uint8_t duty, uint8_t flat, const char* label);
    uint8_t duty_need(uint8_t pct) const;
    void drop_tracks(uint8_t band, uint8_t min_miss);
    bool excluded(size_t x) const;
    uint32_t khz_to_point(uint32_t f_khz) const;
    void emit(Event ev, const RfTrack& t) {
        if (notify_) notify_(notify_ctx_, ev, t);
    }
};

} /* namespace ui::external_app::drone_detect */

#endif /*__DD_RF_HPP__*/
