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

/* General interference scan of the Signature Detect app (class file "SCAN"
 * lines): a frequency-agnostic sweep over one or two ranges, window by window,
 * with feature detectors that report a type, a frequency range and a reason.
 * Integer only, fixed arrays, no UI or ChibiOS dependency (host testable).
 *
 * A range is cut into windows of `points` points `res_hz` apart (the default
 * wide range: 320 points 81.5 kHz apart = 26 MHz, the 915 band's geometry).
 * The app sweeps one window at a time (spec_an::Sweep), hands each look to
 * add_look() and calls finish() after `looks` looks. Per window:
 *   1. per point dB mean, max hold and min hold of the looks;
 *   2. floor: lower quartile of each eighth of the window, each capped at the
 *      second lowest eighth + kFloorCapCdb, interpolated between the eighths;
 *   3. runs of points over the floor (mean >= kOccMean, or >= kOccLook in 2
 *      looks), measured (centroid, width, duty, always on, flatness, edges)
 *      and typed: line, hump, block (flat top + steep edges: a designed
 *      digital transmission), wide (anything else >= 1 MHz);
 *   4. narrow prominent lines (from the mean, and from the max hold for
 *      pulsed ones), and a comb search over them.
 * Across windows and passes:
 *   - combs (kMaxCombs): spacing + extent, reported once seen in 2 passes;
 *     a round spacing (a multiple of 100 kHz) is the receiver's own clock or
 *     the occupied channels of a raster (survey only, "RND?");
 *   - emitters (kMaxEmitters): always-on and intermittent emissions up to
 *     1 MHz wide, their centre history: drift is reported, steady and
 *     intermittent ones only reach the survey (an energy sweep cannot tell a
 *     steady interferer from a licensed transmitter);
 *   - window floors against the session baseline (the lowest floor since the
 *     start or the last gain change): broadband floor rise;
 *   - hum check candidates: the strongest steady emitter not yet checked; the
 *     app runs the meter's mains ladder on it (sigdet_hum.hpp) and passes the
 *     result to hum_result().
 * Every report and survey row carries its reason (ScanWhy). The limits are in
 * the app's README (SCAN lines). */

#ifndef __SIGDET_SCAN_HPP__
#define __SIGDET_SCAN_HPP__

#include <cstddef>
#include <cstdint>

#include "sigdet_hum.hpp"

namespace ui::external_app::signature_detect {

namespace scan_tune {
/* Levels: looks are kept as 0.5 dB steps (uint8, 0 = -128 dBFS). */
constexpr int32_t kU8OffsetCdb = 12800;
constexpr uint8_t kOccMean = 12;    /* point occupied: dB mean >= floor + 6 dB ... */
constexpr uint8_t kOccLook = 20;    /* ... or >= floor + 10 dB in kOccLooks looks */
constexpr uint8_t kOccLooks = 2;    /* (1 when the window has a single look) */
constexpr uint8_t kFloorCapCdb = 8; /* an eighth's floor at most 4 dB over the second lowest */
constexpr uint8_t kLineProm = 12;   /* line: 6 dB over the lower of its two flanks (+-1..2 points) */
constexpr uint8_t kLineMaxW = 3;    /* a line: at most 3 points within 6 dB of its peak (a CW is 2-3 in the wide scan: BH lobe, bins ~ points) */
constexpr uint8_t kCombMinTeeth = 6;
constexpr int32_t kCombTol = 6;             /* a tooth within 6/16 point of the fitted grid */
constexpr uint8_t kCombAlwaysPct = 75;      /* teeth on in >= 75 % of looks: continuous, else pulsed */
constexpr uint8_t kCombFillPct = 70;        /* matched teeth / grid places between first and last */
constexpr uint8_t kCombStepDb = 12;         /* median level step between neighbour teeth <= 6 dB */
constexpr uint16_t kCombMinSp16 = 48;       /* spacing >= 3 points (1/16 point units) */
constexpr uint8_t kCombSpanDiv = 5;         /* spacing <= window / 5 */
constexpr uint16_t kCombSameTol = 150;      /* the same comb: spacing within 1.5 % (1/10000) */
constexpr uint32_t kRoundGridHz = 100000;   /* round spacing (own clock or a channel raster): a multiple of 100 kHz ... */
constexpr uint16_t kRoundTolPpm = 6000;     /* ... within 0.6 % */
constexpr uint32_t kBlockMinHz = 1000000;   /* block / wide: runs of 1 MHz or more */
constexpr uint8_t kBlockRipple = 8;         /* block: plateau p10..p90 within 4 dB */
constexpr uint8_t kBlockFillPct = 80;       /* block: >= 80 % of the run within 3 dB of the plateau */
constexpr uint32_t kEmitMaxHz = 1000000;    /* emitters: runs up to 1 MHz */
constexpr uint8_t kSteadyPct = 75;          /* steady: seen in >= 75 % of the passes since first seen */
constexpr uint8_t kSteadyMin = 3;           /* ... and in 3 passes at least */
constexpr uint8_t kBurstPct = 50;           /* intermittent: < 50 % ... */
constexpr uint8_t kBurstMinPasses = 6;      /* ... over 6 passes at least */
constexpr uint8_t kDriftMin = 6;            /* drift: seen in 6 passes at least, */
constexpr uint8_t kDriftPts = 2;            /* centre range >= 2 points and >= the emission's width, */
constexpr uint8_t kDriftNetPts16 = 24;      /* first to last >= 1.5 points and >= 3/4 of the range, */
constexpr uint8_t kDriftStepPts16 = 24;     /* no step between sightings over 1.5 points */
constexpr uint8_t kEmitDropMiss = 6;        /* an emitter goes after 6 passes unseen (~75 s at the measured 12.5 s a pass) */
constexpr int16_t kHumMinCdb = 10 * 100;    /* hum check candidates: steady, >= 10 dB over the floor */
constexpr uint16_t kHumRecheckPasses = 48;  /* a checked emitter again after 48 passes (~10 min at 12.5 s a pass; was 100 for ~6 s) */
/* Emitter table: per frequency group (HF, 30-300 MHz, 300-1000 MHz, above
 * 1 GHz; only the groups the ranges reach count) a reserve of
 * kMaxEmitters / (2 x groups) slots that the other groups cannot take; the
 * rest is shared. A newcomer in a group at or over its reserve may take the
 * place of the weakest unreported emitter of its group when it is
 * kEmitReplaceCdb stronger. (Before: the windows run upwards and entries seen
 * in this pass could not be evicted, so nothing above ~185 MHz or in HF got
 * a slot in a busy place.) */
constexpr int16_t kEmitReplaceCdb = 300;
constexpr uint32_t kFmLoKhz = 87500;       /* FM broadcast: never an emitter (licensed carriers on a 100 / 200 kHz raster; */
constexpr uint32_t kFmHiKhz = 108000;      /* they starved the table and every hum check) */
constexpr uint32_t kOwnClockHz = 10000000; /* the receiver's own clock lines: lines on multiples of 10 MHz (30 / 40 / 80 / 120 MHz seen) */
constexpr uint8_t kOwnClockMaxW = 5;       /* ... a line: a run at most 5 points wide (a strong CW's skirt), centre within one point of the multiple */
constexpr uint8_t kCombContPct = 50;       /* a comb is continuous (Comb) only when seen in >= 50 % of its passes when listed */
constexpr uint16_t kRasterTolMinHz = 1000; /* teeth on a 100 kHz grid: within max(0.3 point, 1 kHz), at most 25 kHz, */
constexpr uint16_t kRasterTolMaxHz = 25000; /* 5 of 6 of them (and 5 at least) */
constexpr uint8_t kRiseCdbX2 = 8;           /* floor rise: >= 4 dB (0.5 dB units) ... */
constexpr uint8_t kRisePasses = 2;          /* ... in 2 passes running (adjacent windows) */
constexpr uint8_t kRisePassesOne = 3;       /* ... in 3 passes running (one window alone) */
constexpr uint8_t kFloorMinU8 = 76;         /* floors under -90 dBFS are quantisation, not noise: not judged */
constexpr uint8_t kRiseLearn = 3;           /* baseline from 3 passes before any rise is reported */
constexpr uint8_t kRiseBlockPct = 40;       /* windows with blocks over 40 % of their points are not judged */
constexpr uint8_t kCombDropMiss = 3;        /* a continuous comb goes after 3 passes unseen ... */
constexpr uint8_t kPulsedDropMiss = 40;     /* ... a pulsed one (seen in some passes only) after 40 */
}  // namespace scan_tune

struct ScanRange {
    uint32_t lo_khz, hi_khz;
    uint32_t res_hz; /* point spacing */
    uint16_t points; /* per window, 16..320 */
    uint8_t looks;   /* per window visit, 1..8 */
    uint8_t rbw_idx; /* spec_an::Sweep::Params::rbw_idx: 0 auto, else FFT 128 << idx */
};

struct ScanWindowPlan {
    uint64_t start_hz;
    uint64_t span_hz;
    uint16_t points;
    uint8_t looks;
    uint8_t rbw_idx;
    uint8_t range;
    uint16_t index; /* within the range */
};

/* What was found. Reported types first; the rest reach the survey only. */
enum ScanType : uint8_t {
    ST_NONE,
    ST_COMB,   /* comb of lines (continuous) */
    ST_PCOMB,  /* comb of lines that are not always on (pulsed / gated) */
    ST_FLOOR,  /* broadband floor rise */
    ST_DRIFT,  /* always-on emission whose centre moves */
    ST_HUM,    /* emission modulated at mains harmonics (hum check) */
    ST_STEADY, /* survey: always on, stable (licensed carrier or interference: cannot tell) */
    ST_BURST,  /* survey: intermittent (traffic) */
    ST_BLOCK,  /* survey: flat-topped wide block with steep edges (digital service) */
    ST_WIDE,   /* survey: wide emission without a flat top */
    ST_CLOCK,  /* survey: comb on a round spacing: the receiver's own clock, or a channel raster */
    ST_COUNT
};
constexpr bool scan_reported(uint8_t t) {
    return t >= ST_COMB && t <= ST_HUM;
}
const char* scan_type_tag(uint8_t t); /* 4 letters for lists and logs */

enum ScanConf : uint8_t {
    SC_NONE,
    SC_LOW,
    SC_MED,
    SC_HIGH
};

enum ScanEv : uint8_t {
    SE_NEW,    /* reported for the first time */
    SE_UPD,    /* reported, seen again (once a pass at most) */
    SE_DROP,   /* reported, gone */
    SE_SURVEY, /* not reported: a survey row (once per emitter / comb / window) */
};

struct ScanEvent {
    uint8_t ev, type, conf;
    uint8_t uid;    /* 1..255, per reported item (list key) */
    uint8_t range;  /* ScanRange index */
    uint32_t f_khz; /* centre */
    uint32_t lo_khz, hi_khz;
    uint32_t param;    /* comb: spacing Hz; floor: rise cdB; drift: centre range Hz; hum: family Hz */
    int32_t rate;      /* drift: Hz per minute (first to last), 0 otherwise */
    uint16_t count;    /* comb: teeth; floor: windows; hum: harmonics over the bar; drift/steady/burst: passes seen */
    uint16_t passes;   /* passes since first seen */
    int16_t level_cdb; /* over the floor (comb: mean tooth; floor: rise) */
    uint8_t duty;      /* emitters: % of looks in the last window; combs: % of passes seen since first seen (presence) */
    uint8_t width_pts;   /* emitters: widest run, points */
    uint32_t res_hz;     /* the range's point spacing */
    uint32_t anchor_khz; /* comb: a tooth (lo..hi is the extent; Track tunes to the tooth nearest the centre) */
};

class ScanEngine {
   public:
    static constexpr size_t kMaxRanges = 2;
    static constexpr size_t kMaxPoints = 320;
    static constexpr size_t kMaxLooks = 8;
    static constexpr size_t kMaxWindows = 256;
    static constexpr size_t kMaxEmitters = 48; /* 32 until 2026-11-14 (M4 lent RAM: the map's part, see scan_memory) */
    static constexpr size_t kMaxCombs = 8;
    static constexpr size_t kMaxFloors = 4;
    static constexpr size_t kMaxLines = 40;
    static constexpr size_t kPieces = 8;

    using Notify = void (*)(void* ctx, const ScanEvent& e);
    using Sink = void (*)(void* ctx, const char* line);

    /* Per window working memory (2880 B), owned by the caller like
     * RfEngine::Work; may be taken back between windows. */
    struct Work {
        uint8_t look[kMaxLooks][kMaxPoints];
        uint8_t mean[kMaxPoints];
    };
    void set_work(Work* w) {
        w_ = w;
        looks_ = 0;
    }
    bool has_work() const { return w_ != nullptr; }
    void set_notify(Notify fn, void* ctx) {
        notify_ = fn;
        ctx_ = ctx;
    }

    /* Ranges are validated and clamped; returns the number kept. Resets
     * everything (tables, baseline, pass count). */
    size_t configure(const ScanRange* ranges, size_t n);
    /* Gains changed: the floor baseline starts again (tables stay: their
     * levels are relative to the floor). */
    void reset_baseline();

    size_t windows() const { return nwin_; }
    void plan(size_t w, ScanWindowPlan& p) const;

    /* Collect window w. False without working memory or w out of range. */
    bool begin(size_t w);
    /* One complete sweep of the window (centi-dBFS, `points` values). Returns
     * true while more looks are wanted. */
    bool add_look(const int16_t* cdb);
    bool want_more() const { return active_ && looks_ < range_[cur_range_].looks; }
    /* Analyse the window (any number of looks >= 1); the last window of the
     * ranges also closes the pass. t_s: seconds since the scan started. */
    void finish(uint32_t t_s);

    /* Hum check: the emitter the app should check now (the meter at f_khz),
     * or false. Results by the same uid. */
    bool hum_candidate(uint8_t& uid, uint32_t& f_khz, uint32_t& width_hz) const;
    void hum_result(uint8_t uid, const sigdet::HumVerdict& v);
    /* A listed finding as it stands now, as an SE_DROP event (rows written
     * without an event in hand: class change, log stop). False if the uid is
     * not listed. */
    bool describe(uint8_t uid, ScanEvent& out) const;

    uint32_t pass() const { return pass_; }
    /* Floor rises not followed for want of a slot (kMaxFloors), and new
     * emitters not taken (table full, no slot they may take). */
    uint32_t floor_full() const { return floor_full_; }
    uint32_t emit_full() const { return emit_full_; }
    /* FM broadcast or the receiver's own clock line: never an emitter. */
    bool known_carrier(uint32_t f10, uint32_t width_pts, uint32_t res_hz) const;
    uint16_t floor_u8(size_t w) const { return w < nwin_ ? floor_[w] : 0; }
    uint16_t base_u8(size_t w) const { return w < nwin_ ? base_[w] : 0; }

    /* Last window's analysis, for tests and the debug dump. */
    struct Run {
        uint16_t lo, hi; /* points */
        uint16_t peak;   /* point */
        uint32_t c16;    /* centroid, 1/16 point */
        uint8_t ex_mean; /* peak dB mean over floor, 0.5 dB */
        uint8_t ex_max;  /* max hold over floor anywhere in the run, 0.5 dB */
        uint8_t duty;    /* % of looks with the peak >= floor + 6 dB */
        bool always;     /* min hold at the peak >= floor + 6 dB */
        uint8_t type;    /* ST_STEADY (line / hump), ST_BLOCK, ST_WIDE */
        uint8_t ripple;  /* wide runs: plateau p10..p90, 0.5 dB */
        uint8_t fill;    /* wide runs: % within 3 dB of the plateau */
        bool comb;       /* holds a tooth of the window's comb */
    };
    struct Line {
        uint32_t p16; /* position, 1/16 point */
        uint8_t ex;   /* over floor, 0.5 dB */
        uint8_t duty; /* % of looks >= floor + 6 dB */
        bool pulsed;  /* from the max hold, not always on */
    };
    struct CombFit {
        bool ok;
        bool pulsed;
        uint32_t sp16;    /* spacing, 1/16 point */
        uint32_t first16; /* first tooth */
        uint16_t teeth;   /* matched */
        uint16_t places;  /* grid places from first to last */
        uint8_t level;    /* mean tooth over floor, 0.5 dB */
        uint8_t step;     /* median neighbour step, 0.5 dB */
        uint8_t duty;     /* mean of the teeth's duty, % */
    };
    static constexpr size_t kMaxRuns = 24;
    size_t run_count() const { return nruns_; }
    const Run& run(size_t i) const { return runs_[i]; }
    const CombFit& comb_fit() const { return fit_; }
    uint8_t window_floor() const { return wfloor_; }
    /* The comb search alone, on any line set (tests). */
    static CombFit fit_comb(const Line* lines, size_t n, uint16_t points, bool pulsed);

    void dump(Sink sink, void* ctx) const;

   private:
    struct Emitter {
        uint32_t f10, f10_first, f10_min, f10_max; /* centre, 10 Hz units */
        uint32_t lo10, hi10;                       /* the last run's extent */
        uint32_t t_first, t_last;                  /* seconds */
        uint32_t pass_first, pass_last;
        uint32_t hum_pass; /* pass of the last hum check, 0 = never */
        uint16_t seen;
        uint16_t max_step16; /* largest move between sightings, 1/16 point */
        int16_t level;       /* strongest dB mean over floor, cdB */
        uint8_t uid;         /* 0 = free */
        uint8_t range;
        uint8_t width;  /* widest run, points */
        uint8_t always; /* sightings with the peak on in every look */
        uint8_t duty;   /* last duty % */
        uint8_t type;   /* reported type, ST_NONE */
        uint8_t surveyed;
        bool upd;           /* seen this pass */
        uint8_t hum_family; /* ST_HUM: 50 or 60 */
        uint8_t hum_count;  /* ... harmonics over the bar */
    };
    struct Comb {
        uint32_t sp_hz;
        uint32_t anchor10; /* a tooth, 10 Hz units (the first window's first) */
        uint16_t base_k;   /* teeth between the anchor and the farthest tooth used for sp_hz */
        uint32_t lo_khz, hi_khz;
        uint32_t pass_first, pass_last;
        uint16_t seen;
        uint16_t teeth;
        uint8_t level;
        uint8_t uid; /* 0 = free */
        uint8_t range;
        uint8_t places;  /* grid places of the widest one-window fit (spacing precision) */
        bool pulsed, reported, clock, upd;
        bool grid100;    /* a window's teeth sat on a 100 kHz grid (a raster's channels, or round clock lines) */
    };
    struct Floor {
        uint16_t w_lo, w_hi;
        uint16_t rise; /* cdB */
        uint8_t uid;   /* 0 = free */
        uint8_t miss;
        bool reported;
    };

    ScanRange range_[kMaxRanges]{};
    uint16_t first_[kMaxRanges + 1]{}; /* first window of each range */
    size_t nrange_{0};
    size_t nwin_{0};

    Work* w_{nullptr};
    bool active_{false};
    uint16_t cur_w_{0};
    uint8_t cur_range_{0};
    uint16_t points_{0};
    uint8_t looks_{0};
    uint64_t start_hz_{0};

    uint32_t pass_{0};
    uint32_t t_s_{0};
    uint8_t next_uid_{1};

    uint8_t floor_[kMaxWindows]{};
    uint8_t base_[kMaxWindows]{}; /* second lowest floor seen */
    uint8_t low_[kMaxWindows]{};  /* lowest */
    uint8_t rise_run_[kMaxWindows]{};
    uint8_t base_passes_{0};
    uint8_t block_pct_[kMaxWindows]{};
    uint8_t surveyed_[kMaxWindows / 8]{}; /* a block / wide survey row given for the window */

    Emitter em_[kMaxEmitters]{};
    Comb comb_[kMaxCombs]{};
    Floor fl_[kMaxFloors]{};
    uint8_t groups_{0}; /* bit g: a range reaches frequency group g (emitter reserve) */
    uint32_t floor_full_{0}, emit_full_{0};

    /* Last window */
    uint8_t piece_[kPieces]{};
    uint8_t wfloor_{0};
    Run runs_[kMaxRuns]{};
    size_t nruns_{0};
    CombFit fit_{};

    Notify notify_{nullptr};
    void* ctx_{nullptr};

    uint8_t floor_at(size_t x) const;
    void find_floor();
    uint8_t quiet_floor() const;
    void find_runs();
    void measure_wide(Run& r) const;
    size_t find_lines(Line* out, bool pulsed) const;
    void window_comb();
    void emitters_from_runs();
    Emitter* emitter_slot(uint32_t f10, int16_t level);
    uint16_t comb_on_grid() const;
    bool comb_clock(const Comb& c) const;
    bool on_comb(uint32_t f10, uint8_t range) const;
    void close_pass();
    void floors_close();
    uint8_t new_uid();
    uint32_t f10_of(uint32_t p16) const;
    uint32_t khz_of(uint32_t p16) const { return (f10_of(p16) + 50) / 100; }
    void emit_emitter(uint8_t ev, uint8_t type, uint8_t conf, const Emitter& e);
    void fill_emitter(uint8_t type, uint8_t conf, const Emitter& e, ScanEvent& s) const;
    void comb_event(const Comb& c, ScanEvent& s) const;
    void floor_event(const Floor& f, ScanEvent& s) const;
    void emit(const ScanEvent& e) {
        if (notify_) notify_(ctx_, e);
    }
};

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_SCAN_HPP__*/
