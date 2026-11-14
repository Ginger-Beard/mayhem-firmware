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

#include "sigdet_rf.hpp"
#include "sigdet_fmt.hpp"

#include <cstring>

namespace ui::external_app::signature_detect {

using namespace tune;

BandDef kBands[BAND_COUNT] = {
    {2398, 2486, 320, 30, BF_PEAK | (SIGDET_FULL ? BF_HOP : 0), "2.4"},
    {5640, 5950, 320, 30, 0, "5.8"},
    {5340, 5640, 320, 30, BF_ALT, "5.3"},
    {1060, 1380, 320, 30, BF_SLICE, "1.2"},
    {902, 928, 320, 25, BF_PEAK | BF_HOP, "915"},
    {860, 872, 160, 15, BF_PEAK | BF_HOP, "868"},
    /* U-NII-1 (GRADE_DOC: no real signal swept yet). DJI aircraft from the
     * Mini 4 Pro / Air 3 on, the O4 air units and Autel EVO Lite transmit at
     * 5157-5245 MHz (FCC filings, fcc-research grids.md 1-2). The 5.8 band's
     * point spacing (~970 kHz) and average detector; ~8 slices a look, so
     * 150 ms holds ~20 looks. Odd cycles only, when 5.3 rests: ~+10 % cycle
     * time. A 60 / 80 MHz mode fills over half of these 100 MHz, and the
     * band's floor then rises under it (see NOTE of the sweep-fix build). */
    {5150, 5250, 104, 15, BF_ALT | BF_ODD | BF_NOAV, "5.1"},
    {0, 0, 0, 0, BF_NOCLS, "X0"},
    {0, 0, 0, 0, BF_NOCLS, "X1"},
};

void set_custom_band(uint8_t band, uint16_t lo_mhz, uint16_t hi_mhz, uint16_t points, uint8_t dwell_cs, const char* name) {
    if (band < BAND_X0 || band >= BAND_COUNT) return;
    BandDef& d = kBands[band];
    d.lo_mhz = lo_mhz;
    d.hi_mhz = hi_mhz;
    d.points = points;
    d.dwell_cs = dwell_cs;
    d.flags = BF_NOCLS | BF_PEAK;
    size_t i = 0;
    for (; name && name[i] && i < sizeof(d.name) - 1; i++) d.name[i] = name[i];
    d.name[i] = 0;
}

/* Hop grids, each band's primary grid first (HopGrid). Grid facts only (not
 * copyrightable; sources named per entry). The further 915 grids need the
 * generic FHSS rule's distinct channels: they only name what was "FHSS" (or
 * a weak "ELRS"). */
const HopGrid kHopGrids[] = {
    /* ExpressLRS FCC915: 903.5 + 0.6k MHz, 40 channels (ExpressLRS FHSS.cpp). */
    {903500, 600, kGridTolKhz, 40, BAND_915, GRADE_PRIOR, kElrsMinDistinct, kElrsMinGridPct, kElrsHighDistinct, kElrsHighGridPct, "ELRS"},
    /* TBS Crossfire 915: 902.75 + 0.5k MHz, 50 channels, 20 dB width ~254
     * kHz (FCC QOS-TXMICRO, QOS-TANGO2 test reports). */
    {902750, 500, kHopGridTolKhz, 50, BAND_915, GRADE_DOC, kFhssMinDistinct, kNamedGridPct, 0, 0, "CRSF"},
    /* FrSky R9 / R9M: 904 + 0.5k MHz, 43 channels, 20 dB width 0.34 MHz (FCC
     * XYFR9M09T test report). */
    {904000, 500, kHopGridTolKhz, 43, BAND_915, GRADE_DOC, kFhssMinDistinct, kNamedGridPct, 0, 0, "R9"},
    /* ExpressLRS EU868: 863.275 + 0.525k MHz, 13 channels (FHSS.cpp). The
     * Crossfire / R9 868 plans have no source in the research (FCC filings
     * cover 915 only): none here. */
    {863275, 525, kGridTolKhz, 13, BAND_868, GRADE_PRIOR, kElrsEuMinDistinct, kElrsMinGridPct, kElrsHighDistinct, kElrsHighGridPct, "ELRS"},
#if SIGDET_FULL
    /* Toys tune 2400 + n MHz: the toy rule. */
    {2400000, 1000, kToyGridTolKhz, 84, BAND_24, GRADE_PRIOR, 0, kToyMinGridPct, 0, 0, "Toy"},
    /* ExpressLRS 2.4 GHz: 2400.4 + n MHz, 80 channels (FHSS.cpp); names a
     * 2.4 GHz hopper, so only with the 2.4 GHz hopper verdicts on. */
    {2400400, 1000, kElrs24TolKhz, 80, BAND_24, GRADE_DOC, kElrs24MinDistinct, kNamedGridPct, 0, 0, "ELRS"},
#endif
};
const size_t kHopGridCount = sizeof(kHopGrids) / sizeof(kHopGrids[0]);

/* DJI OcuSync / O3 / O4 video centres by width class (FCC test reports,
 * fcc-research grids.md 1: any x.5 MHz centre in the class's range, not the
 * old 15 MHz list). 1.4 and 3 MHz modes are narrow hits, not segments. */
const VideoGrid kVideoGrids[] = {
    {24045, 24695, 500, 1000, kVideoGridTolKhz, BAND_24, 5, GRADE_DOC, "DJI"},
    {24055, 24765, 500, 1000, kVideoGridTolKhz, BAND_24, 10, GRADE_DOC, "DJI"},
    {24105, 24725, 500, 1000, kVideoGridTolKhz, BAND_24, 20, GRADE_DOC, "DJI"},
    {24225, 24525, 500, 1000, kVideoGridTolKhz, BAND_24, 40, GRADE_DOC, "DJI"},
    {24325, 24425, 500, 1000, kVideoGridTolKhz, BAND_24, 60, GRADE_DOC, "DJI"},
    /* 5.8 GHz: x.5 MHz as well, but inert at this band's ~970 kHz points
     * (tune::kGridMaxPtKhz). The 60 class holds the 80 MHz mode's range.
     * 5.1 GHz centres are integer MHz, as Wi-Fi's: no grid entry. */
    {57305, 58445, 500, 1000, kVideoGridTolKhz, BAND_58, 10, GRADE_DOC, "DJI"},
    {57355, 58395, 500, 1000, kVideoGridTolKhz, BAND_58, 20, GRADE_DOC, "DJI"},
    {57455, 58295, 500, 1000, kVideoGridTolKhz, BAND_58, 40, GRADE_DOC, "DJI"},
    {57655, 58095, 500, 1000, kVideoGridTolKhz, BAND_58, 60, GRADE_DOC, "DJI"},
};
const size_t kVideoGridCount = sizeof(kVideoGrids) / sizeof(kVideoGrids[0]);

namespace {

/* Analog FPV channel tables (MHz), first match wins on a tie. */
struct AnalogTable {
    char name[3];
    uint8_t band;
    uint16_t mhz[8];
};
const AnalogTable kAnalog[] = {
    {"R", BAND_58, {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}},
    {"F", BAND_58, {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}},
    {"A", BAND_58, {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}},
    {"B", BAND_58, {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}},
    {"E", BAND_58, {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}},
#if SIGDET_FULL
    {"L", BAND_53, {5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621}},
    {"1G", BAND_12, {1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360}},
#endif
    {"2G", BAND_24, {2414, 2432, 2450, 2468, 0, 0, 0, 0}},
};

/* Digital FPV centres, 100 kHz units: DJI V1/Vista/Walksnail, O3, O4 (20 and
 * 40/60 MHz), HDZero on raceband; DJI OcuSync at 2.4 GHz. */
const uint16_t kDigital58[] = {56580, 56600, 56690, 56770, 56950, 57050, 57320, 57350, 57410, 57690, 57700, 57900,
                               57950, 58050, 58060, 58150, 58390, 58400, 58430, 58550, 58760, 58780, 58800, 59020,
                               59120, 59140, 59170};
const uint16_t kDigital24[] = {23995, 24145, 24295, 24445, 24595, 24745};

#if SIGDET_FULL
/* BLE advertising channels, excluded from the 2.4 GHz hit statistics. */
const uint32_t kBleAdvKhz[3] = {2402000, 2426000, 2480000};
#endif

constexpr const char kTags[RC_COUNT][4] = {"-", "AV", "DV", "DW", "CL", "H24", "TOY", "WFI", "OVN"};
constexpr int16_t kRelRef = -1600; /* histogram base for levels over floor */

/* Not inlined: called in many places (code size). */
__attribute__((noinline)) uint32_t dist(uint32_t a, uint32_t b) {
    return a > b ? a - b : b - a;
}

__attribute__((noinline)) uint8_t sat8(int32_t v) {
    return v < 0 ? 0 : (v > 255 ? 255 : static_cast<uint8_t>(v));
}

/* Value at `pct` percent of v[a..b), 0.5 dB resolution over 64 dB from ref. */
int16_t pctl(const int16_t* v, size_t a, size_t b, int16_t ref, uint8_t pct) {
    uint16_t h[128] = {};
    for (size_t i = a; i < b; i++) h[sat8(((v[i] - ref) * 41) >> 11) & 0x7F]++;
    const size_t target = ((b - a) * pct) / 100;
    size_t acc = 0, k = 0;
    while (k < 127 && (acc += h[k]) <= target) k++;
    return static_cast<int16_t>(ref + k * 50 + 25);
}

/* f on base + k step (0 <= k < count) within tol. */
bool on_grid(uint32_t f, uint32_t base, uint32_t step, uint32_t count, uint32_t tol) {
    if (f + tol < base) return false;
    const uint32_t k = (f + step / 2 - base) / step;
    return k < count && dist(f, base + k * step) <= tol;
}

bool on_list(const uint16_t* list, size_t n, uint32_t f, uint32_t tol) {
    for (size_t i = 0; i < n; i++)
        if (dist(f, list[i] * 100u) <= tol) return true;
    return false;
}

bool on_wifi_grid(uint8_t band, uint32_t f) {
    if (band == BAND_24) return on_grid(f, 2412000, 5000, 13, kWifi24TolKhz);
    if (band == BAND_58 || band == BAND_53 || band == BAND_51) return on_grid(f, f < 5735000 ? 5180000 : 5745000, 10000, 60, kWifi5TolKhz);
    return false;
}

/* On a digital FPV table channel (kDigital24 / kDigital58). */
bool on_dig_table(uint8_t band, uint32_t f) {
    return band == BAND_24 ? on_list(kDigital24, sizeof(kDigital24) / 2, f, kDigTolKhz)
                           : (band == BAND_58 && on_list(kDigital58, sizeof(kDigital58) / 2, f, kDigTolKhz));
}

static_assert(kWidthMhz[0] == 5, "width class 0 is the 5 MHz class");

/* The band's idx-th hop grid (0: primary), or nullptr. */
const HopGrid* band_grid(uint8_t band, size_t idx) {
    for (const HopGrid& g : kHopGrids)
        if (g.band == band && !idx--) return &g;
    return nullptr;
}

/* A grid's verdict confidence: low for GRADE_DOC, else medium, high from its
 * high_distinct / high_pct. */
uint8_t grid_conf(const HopGrid& g, uint8_t distinct, uint32_t pct) {
    if (g.grade == GRADE_DOC) return CONF_LOW;
    return (g.high_distinct && distinct >= g.high_distinct && pct >= g.high_pct) ? CONF_HIGH : CONF_MED;
}

/* "ELRS 915", a GRADE_DOC grid "CRSF? 915". */
void grid_label(Fmt& l, const HopGrid& g, const char* band) {
    l.s(g.name);
    if (g.grade == GRADE_DOC) l.c('?');
    l.c(' ').s(band);
}

uint8_t family(uint8_t cls) {
    return cls == RC_DIGWIFI ? uint8_t{RC_DIGITAL} : (cls == RC_TOY ? uint8_t{RC_HOP24} : cls);
}

}  // namespace

const char* rf_class_tag(uint8_t cls) {
    return kTags[cls < RC_COUNT ? cls : 0];
}

uint32_t RfEngine::point_khz(uint32_t x) const {
    return lo_khz_ + ((x * q8_) >> 8);
}

uint32_t RfEngine::khz_to_point(uint32_t f) const {
    return f <= lo_khz_ ? 0 : (((f - lo_khz_) << 8) + q8_ / 2) / q8_;
}

bool RfEngine::begin(uint8_t band) {
    band_ = band;
    const BandDef& d = kBands[band];
    points_ = d.points;
    looks_ = 0;
    nsegs_ = 0;
    hits_ = RfHits{};
    if (!w_) return false;
    lo_khz_ = d.lo_mhz * 1000u;
    q8_ = ((d.hi_mhz - d.lo_mhz) * 1000u << 8) / (points_ - 1);
    thr_ = (d.flags & BF_PEAK) ? kOccPeakCdb : kOccAvgCdb;
    /* sum .. nhit (.. nwide) are adjacent */
    memset(w_->sum, 0, reinterpret_cast<uint8_t*>(w_->nhit + (SIGDET_FULL ? 2 : 1) * kMaxPoints) - reinterpret_cast<uint8_t*>(w_->sum));
    if (floor_valid(band)) interpolate_floor();
    return true;
}

/* Piece s of W points has its floor at its centre; linear in between. */
void RfEngine::interpolate_floor() {
    const int16_t* f = floors_[band_];
    const int32_t w = points_ / kFloorSegs;
    for (int32_t x = 0; x < points_; x++) {
        int32_t s = (x - w / 2) / w;
        if (x < w / 2) s = 0;
        if (s >= kFloorSegs - 1) {
            w_->floor[x] = f[kFloorSegs - 1];
            continue;
        }
        const int32_t t = x - (s * w + w / 2);
        w_->floor[x] = static_cast<int16_t>(f[s] + (t > 0 ? (f[s + 1] - f[s]) * t / w : 0));
    }
}

/* New floor pieces from v (levels; relative to the current floor when
 * relative is set): lower quartile of each piece, smoothed. */
void RfEngine::update_floor(const int16_t* v, int16_t relative) {
    int16_t* f = floors_[band_];
    const size_t w = points_ / kFloorSegs;
    for (size_t s = 0; s < kFloorSegs; s++) {
        const size_t a = s * w;
        if (!relative) {
            int16_t lo = v[a];
            for (size_t i = a; i < a + w; i++)
                if (v[i] < lo) lo = v[i];
            f[s] = pctl(v, a, a + w, lo, kFloorPctl);
            continue;
        }
        const int16_t p = pctl(v, a, a + w, kRelRef, kFloorPctl);
        f[s] += (p < -kFloorFastDownCdb) ? p : p / (1 << kFloorShift);
    }
    /* A piece filled by one signal: keep it near the median of the others. */
    int16_t sorted[kFloorSegs];
    memcpy(sorted, f, sizeof(sorted));
    for (size_t i = 1; i < kFloorSegs; i++)
        for (size_t j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
            const int16_t t = sorted[j];
            sorted[j] = sorted[j - 1];
            sorted[j - 1] = t;
        }
    const int16_t cap = sorted[kFloorSegs / 2] + kFloorMaxAboveMedCdb;
    for (size_t s = 0; s < kFloorSegs; s++)
        if (f[s] > cap) f[s] = cap;
    floor_ok_ |= 1u << band_;
}

bool RfEngine::add_look(const int16_t* cdb) {
    if (!w_ || looks_ >= kMaxLooks) return false;
    if (!floor_valid(band_)) {
        update_floor(cdb, 0);
        interpolate_floor();
    }
    looks_++;

    size_t start = 0;
    bool in_run = false, any = false;
    for (size_t x = 0; x <= points_; x++) {
        bool on = false;
        if (x < points_) {
            const int32_t rel = cdb[x] - w_->floor[x];
            w_->sum[x] += sat8(((rel + kSumOffsetCdb) * 41) >> 11);
            const uint8_t m = sat8((rel * 41) >> 11);
            if (m > w_->maxh[x]) w_->maxh[x] = m;
            on = rel >= thr_;
            if (on && w_->occ[x] < 255) w_->occ[x]++;
        }
        if (on && !in_run) {
            start = x;
            in_run = true;
        } else if (!on && in_run) {
            in_run = false;
            const uint32_t w = ((x - start) * q8_) >> 8;
            const size_t c = (start + x - 1) / 2;
            if (w < kHitThinKhz || x - start < kHitMinPts) {
                if (hits_.thin < 0xFFFF) hits_.thin++;
            } else if (w <= kHitNormKhz) {
                if (w_->nhit[c] < 255) w_->nhit[c]++;
                any = true;
            }
#if SIGDET_FULL
            else if (w <= kHitWideKhz && w_->nwide[c] < 255) {
                w_->nwide[c]++;
            }
#endif
        }
    }
    if (any) hits_.looks_any++;
    return looks_ < kMaxLooks;
}

void RfEngine::finish() {
    if (!looks_ || !w_) return; /* nothing measured (stalled sweep): tracks keep their state */

    /* Means over floor, cdB, in place of the sums. */
    const uint32_t inv = (50u << 16) / looks_;
    int16_t* mean = reinterpret_cast<int16_t*>(w_->sum);
    for (size_t x = 0; x < points_; x++)
        mean[x] = static_cast<int16_t>(static_cast<int32_t>((w_->sum[x] * inv) >> 16) - kSumOffsetCdb);

    for (auto& t : tracks_)
        if (t.uid && t.band == band_ && t.miss < 255) t.miss++;

    nsegs_ = 0;
    if (kBands[band_].flags & BF_NOCLS) { /* custom band: floor only (signature NB rules, survey) */
        update_floor(mean, 1);
        return;
    }
    find_segments();
    if (kBands[band_].flags & BF_SLICE) {
        for (size_t i = 0; i < nsegs_; i++) {
            RfSeg& sg = w_->segs[i];
            if ((sg.cls == RC_ANALOG || sg.cls == RC_DIGITAL || sg.cls == RC_DIGWIFI) && slice_artefact(sg)) {
                sg.cls = RC_NONE;
                sg.conf = CONF_NONE;
                slice_gated_++;
            }
        }
    }
    if (kBands[band_].flags & BF_HOP) {
        hit_stats();
        classify_hits();
    }
    for (size_t i = 0; i < nsegs_; i++) {
        const RfSeg& s = w_->segs[i];
        if (rf_reported(s.cls)) track(s.cls, s.conf, s.f_khz, s.w10_khz, s.snr, s.duty, s.flat, s.label);
    }
    if (rf_reported(hits_.cls))
        track(hits_.cls, hits_.conf, hits_.f_khz, hits_.span_khz, hits_.level * 50, 0, 0, hits_.label);
    drop_tracks(band_, kDropMisses);

    update_floor(mean, 1);
}

/* Runs of points occupied in >= kSegMinOccPct of the looks. A run may hold
 * more than one emitter (adjacent Wi-Fi channels): what lies outside the
 * -10 dB span of its strongest one is looked at again, less that one's
 * skirt. */
void RfEngine::find_segments() {
    const uint32_t min_occ = looks_ * kSegMinOccPct / 100;
    const uint8_t need = min_occ < 2 ? 2 : static_cast<uint8_t>(min_occ);
    uint16_t todo[4][3];
    size_t x = 0;
    while (x < points_) {
        if (w_->occ[x] < need) {
            x++;
            continue;
        }
        size_t b = x, gap = 0;
        for (size_t y = x + 1; y < points_; y++) {
            if (w_->occ[y] >= need) {
                b = y;
                gap = 0;
            } else if (++gap > kSegGapPts) {
                break;
            }
        }
        size_t n = 0;
        todo[n][0] = x;
        todo[n][1] = b;
        todo[n++][2] = 0;
        x = b + 1;
        while (n && nsegs_ < kMaxSegs) {
            n--;
            size_t lo = todo[n][0], hi = todo[n][1];
            const uint8_t side = todo[n][2]; /* 1: parent on the right, 2: on the left */
            /* Off with the parent's skirt: down from the parent's side. */
            const int16_t* mean = reinterpret_cast<const int16_t*>(w_->sum);
            while (side == 1 && hi > lo && mean[hi - 1] < mean[hi]) hi--;
            while (side == 2 && lo < hi && mean[lo + 1] < mean[lo]) lo++;
            while (lo < hi && w_->occ[lo] < need) lo++;
            while (hi > lo && w_->occ[hi] < need) hi--;
            if ((((hi - lo + 1) * q8_) >> 8) < kWideMinKhz) continue;
            RfSeg& s = w_->segs[nsegs_];
            s = RfSeg{};
            s.lo = lo;
            s.hi = hi;
            measure(s);
            classify(s);
            nsegs_++;
            if (n < 3 && s.l10 > lo + 1) {
                todo[n][0] = lo;
                todo[n][1] = s.l10 - 1;
                todo[n++][2] = 1;
            }
            if (n < 3 && s.r10 + 1u < hi) {
                todo[n][0] = s.r10 + 1;
                todo[n][1] = hi;
                todo[n++][2] = 2;
            }
        }
    }
}

void RfEngine::measure(RfSeg& s) {
    const int16_t* mean = reinterpret_cast<const int16_t*>(w_->sum);
    size_t pk = s.lo;
    for (size_t x = s.lo; x <= s.hi; x++)
        if (mean[x] > mean[pk]) pk = x;
    const int16_t peak = mean[pk];
    s.snr = peak;

    /* The "-10 dB" span is taken at most kShapeMarginCdb above the floor:
     * below ~13 dB SNR a full 10 dB down would reach the floor clamp and
     * take in the skirt, so a flat top read as peaked (w3/w10 small): a 97 %
     * duty 20 MHz Wi-Fi-like emitter at 10 dB SNR was confirmed as analog
     * FPV (A7 5745, high) on the device. */
    int32_t drop10 = peak - kShapeMarginCdb;
    if (drop10 > 1000) drop10 = 1000;
    if (drop10 < 300) drop10 = 300;
    for (size_t k = 0; k < 2; k++) {
        int32_t lim = peak - (k ? 300 : drop10);
        if (lim < kSpanFloorCdb) lim = kSpanFloorCdb;
        size_t l = pk, r = pk;
        while (l > s.lo && mean[l - 1] >= lim) l--;
        while (r < s.hi && mean[r + 1] >= lim) r++;
        (k ? s.l3 : s.l10) = l;
        (k ? s.r3 : s.r10) = r;
    }
    s.w10_khz = ((s.r10 - s.l10 + 1) * q8_) >> 8;
    s.w3_khz = ((s.r3 - s.l3 + 1) * q8_) >> 8;

    uint32_t ws = 0, xs = 0;
    for (size_t x = s.l10; x <= s.r10; x++) {
        const uint32_t w = mean[x] > 0 ? mean[x] >> 4 : 0;
        ws += w;
        xs += w * x;
        if (w_->maxh[x] > s.hold) s.hold = w_->maxh[x];
    }
    const uint32_t c16 = ws ? (xs * 16) / ws : pk * 16;
    s.f_khz = lo_khz_ + ((c16 * q8_) >> 12);

    /* fm: each -10 dB edge between its last point at or above the level and
     * the next one out (1/16 point; a whole point when that one is above the
     * level too, at the run's end; none when even the peak is below the
     * clamped level, a segment barely over the floor). */
    int32_t lim = peak - drop10;
    if (lim < kSpanFloorCdb) lim = kSpanFloorCdb;
    auto edge = [&](size_t in, size_t out) -> int32_t {
        const int32_t a = mean[in] - lim, b = mean[in] - mean[out];
        if (a <= 0) return 0;
        return a >= b ? 16 : (a * 16) / b; /* here b > a > 0 */
    };
    const int32_t el = s.l10 * 16 - (s.l10 > 0 ? edge(s.l10, s.l10 - 1) : 0);
    const int32_t er = s.r10 * 16 + (s.r10 + 1u < points_ ? edge(s.r10, s.r10 + 1) : 0);
    s.fm_khz = lo_khz_ + ((static_cast<uint32_t>(el + er) / 2 * q8_) >> 12);

    const size_t n = s.r10 - s.l10 + 1;
    const size_t cut = n * (100 - kCentralPct) / 200;
    const size_t ca = s.l10 + cut, cb = s.r10 + 1 - cut;
    s.flat = sat8((peak - pctl(mean, ca, cb, kRelRef, 50)) / 10);
    uint32_t occ = 0;
    for (size_t x = ca; x < cb; x++) occ += w_->occ[x];
    const uint32_t duty = (occ * 100) / (looks_ * (cb - ca));
    s.duty = duty > 100 ? 100 : duty;
}

/* A duty class's threshold for one dwell: kDutySlackPct lower, so a short
 * dwell (~10 looks) with a missed look or two keeps its class. The track
 * accumulates the duty of every such dwell and confirms only at the full
 * threshold over kConfirmLooks looks (track()). */
bool RfEngine::slice_of(uint32_t f_khz, uint16_t& index, uint32_t& off_khz) const {
    if (!slice_step_hz_) return false;
    const int64_t d = static_cast<int64_t>(f_khz) * 1000 - slice_f0_hz_;
    if (d < 0) return false;
    const int64_t j = d / slice_step_hz_;
    index = static_cast<uint16_t>(j > 65535 ? 65535 : j);
    off_khz = static_cast<uint32_t>((d - j * slice_step_hz_) / 1000);
    return true;
}

/* A video verdict that is the receiver's: the sweep puts each tune slice's
 * piece of spectrum side by side, and a front end response that is not flat
 * across the slice (seen 2026-11-14 at 1.2 GHz with LNA 40 / VGA 38-52 /
 * amp on: 5-8 MHz wide "AV / DV" humps ~4.3 MHz above the LO of 2-5 slices
 * at once, centres 14.6 MHz apart: the tune step) repeats at the same offset
 * in other slices. The segment's peak offset is looked up in every other
 * slice of the band: when at least two of them hold half its level there
 * (+-1 point), it is dropped. A real transmitter repeats at one slice step
 * only by chance (and then in one other slice, not two). Only in BF_SLICE
 * bands (1.2 GHz): where Wi-Fi fills many slices (2.4 / 5 GHz) the same
 * test would drop real video. */
bool RfEngine::slice_artefact(const RfSeg& s) const {
    if (!slice_step_hz_ || points_ < 3) return false;
    const int16_t* mean = reinterpret_cast<const int16_t*>(w_->sum);
    size_t pk = s.lo;
    for (size_t x = s.lo; x <= s.hi; x++)
        if (mean[x] > mean[pk]) pk = x;
    const int64_t f_pk = static_cast<int64_t>(point_khz(pk)) * 1000;
    const int64_t step = slice_step_hz_;
    const int64_t d = f_pk - slice_f0_hz_;
    if (d < 0) return false;
    const int64_t own = d / step, off = d - own * step;
    const int64_t lo = static_cast<int64_t>(point_khz(0)) * 1000, hi = static_cast<int64_t>(point_khz(points_ - 1)) * 1000;
    size_t echoes = 0;
    for (int64_t j = 0;; j++) {
        const int64_t f = slice_f0_hz_ + j * step + off;
        if (f > hi) break;
        if (j == own || f < lo) continue;
        const size_t x = khz_to_point(static_cast<uint32_t>(f / 1000));
        if (x >= points_) continue;
        int16_t v = mean[x];
        if (x > 0 && mean[x - 1] > v) v = mean[x - 1];
        if (x + 1 < points_ && mean[x + 1] > v) v = mean[x + 1];
        if (2 * static_cast<int32_t>(v) >= s.snr) echoes++;
    }
    return echoes >= 2;
}

uint8_t RfEngine::duty_need(uint8_t pct) const {
    return static_cast<uint8_t>(pct > kDutySlackPct ? pct - kDutySlackPct : 0);
}

void RfEngine::classify(RfSeg& s) {
    const uint32_t f = s.f_khz;
    const int32_t mhz = (f + 500) / 1000;
    const uint32_t shape = s.w10_khz ? (s.w3_khz * 100u) / s.w10_khz : 0;
    const bool noav = kBands[band_].flags & BF_NOAV;
    const bool peaked = s.flat * 10 >= kAnalogMinFlatCdb || shape <= kAnalogMaxShapePct;
    const bool flattop = noav || (s.flat * 10 < kDigMaxFlatCdb && shape >= kDigMinShapePct);
    Fmt l{s.label, sizeof(s.label)};

    /* A Wi-Fi channel's width on the Wi-Fi grid is Wi-Fi or digital video,
     * whatever its shape: real OFDM emitters read humped at ~1 MHz points (a
     * 97 % duty 20 MHz emitter on ch 149 measured 2-3 dB "flat", w3/w10
     * 35-55 %, i.e. analog by shape). Analog FPV that wide on an A band
     * channel (= Wi-Fi 149..173) is then reported as DV/WF, not missed. A
     * sustained one on a DJI centre (x.5 MHz, where resolved) is DJI-like
     * video instead (VideoGrid: a hint, low confidence). */
    size_t wc10 = kWidthClasses;
    for (size_t k = 0; k < kWidthClasses; k++)
        if (kWidthMhz[k] >= 20 && s.w10_khz >= kWidthLoKhz[k] && s.w10_khz <= kWidthHiKhz[k]) wc10 = k;
    if (wc10 < kWidthClasses && on_wifi_grid(band_, f)) {
        if (s.duty < kWifiMaxDuty) {
            s.cls = RC_WIFI;
            l.s("WiFi ").d(mhz);
            return;
        }
        const bool table = on_dig_table(band_, f);
        const VideoGrid* grid = video_grid(s, wc10);
        if ((table || grid) && s.duty >= duty_need(kDigMinDuty)) {
            digital(s, wc10, table ? CONF_MED : CONF_LOW, grid);
        } else {
            s.cls = RC_DIGWIFI;
            s.conf = CONF_LOW;
            l.s("DV/WF ").d(mhz);
        }
        return;
    }

    /* 5 MHz class (GRADE_DOC), ahead of the analog test that took such a
     * signal when it read peaked: flat enough (w3/w10), no wider skirt than
     * an OFDM block's, but a skirt: a piece of a wider signal that the floor
     * cut off has none (w3 = w10; seen in simulation, 40 MHz at 2.4 GHz).
     * Sustained, at the full kDigMinDuty within the dwell (no slack: two
     * household emitters of this width at 70 % in the replay corpus):
     * digital video, low; else nothing. Only here: the width classes below
     * start at 10 MHz, as before. */
    if (s.w3_khz >= kWidthLoKhz[0] && s.w3_khz <= kWidthHiKhz[0] && s.w10_khz <= kW5MaxW10Khz && shape >= kW5MinShapePct &&
        s.w3_khz < s.w10_khz) {
        if (s.duty >= kDigMinDuty) digital(s, 0, CONF_LOW, video_grid(s, 0));
        return;
    }

    if (!noav && s.duty >= duty_need(kAnalogMinDuty) && peaked && s.w10_khz >= kAnalogMinW10Khz && s.w10_khz <= kAnalogMaxW10Khz) {
        s.cls = RC_ANALOG;
        const AnalogTable* best = nullptr;
        size_t best_i = 0;
        uint32_t best_d = kAnalogTolKhz + 1;
        for (const auto& t : kAnalog) {
            if (t.band != band_) continue;
            for (size_t i = 0; i < 8; i++) {
                const uint32_t d = dist(f, t.mhz[i] * 1000u);
                if (d < best_d) {
                    best_d = d;
                    best = &t;
                    best_i = i;
                }
            }
        }
        if (best) {
            s.conf = s.snr >= kHighSnrCdb ? CONF_HIGH : CONF_MED;
            l.s(best->name).d(best_i + 1).c(' ').d(best->mhz[best_i]);
        } else {
            s.conf = CONF_LOW;
            l.s("FPV? ").d(mhz);
        }
        return;
    }

    /* Width class (10 MHz and up) of the -3 dB width, else of the -10 dB
     * width. */
    size_t wc = kWidthClasses;
    for (size_t k = 0; k < 2 * (kWidthClasses - 1) && wc == kWidthClasses; k++) {
        const uint32_t w = k < kWidthClasses - 1 ? s.w3_khz : s.w10_khz;
        const size_t c = 1 + k % (kWidthClasses - 1);
        if (w >= kWidthLoKhz[c] && w <= kWidthHiKhz[c]) wc = c;
    }
    const bool wifi = wc < kWidthClasses && on_wifi_grid(band_, f);
    if (wifi && s.duty < kWifiMaxDuty) {
        s.cls = RC_WIFI;
        l.s("WiFi ").d(mhz);
        return;
    }
    if (flattop && wc < kWidthClasses && s.duty >= kWifiMaxDuty) {
        const bool table = on_dig_table(band_, f);
        const VideoGrid* grid = video_grid(s, wc);
        if (s.duty >= duty_need(kDigMinDuty) && (table || grid || !wifi)) {
            digital(s, wc, table ? CONF_MED : CONF_LOW, grid);
        } else if (wifi) {
            s.cls = RC_DIGWIFI;
            s.conf = CONF_LOW;
            l.s("DV/WF ").d(mhz);
        }
        return;
    }

    if (band_ == BAND_24 && s.duty >= kOvenMinPct && s.duty <= kOvenMaxPct && f >= kOvenLoKhz && f <= kOvenHiKhz &&
        s.w10_khz >= kOvenMinSpreadKhz) {
        s.cls = RC_OVEN;
        l.s("Oven");
    }
}

/* The video grid entry of this band and width class whose channel the
 * segment's edge midpoint is on. Only where that centre can be trusted:
 * points of at most kGridMaxPtKhz, and a signal (-10 dB) within
 * kGridMaxPiecePct of a floor piece. A wider one lifts the floor piece it
 * fills, tilting what is left over the floor: in simulation the edge
 * midpoint of a 9 / 18 MHz signal at 2.4 GHz then read up to 0.45 / 0.4 MHz
 * off, and saturated 20 MHz Wi-Fi on 2422 MHz came out "DJI?" in 40 % of
 * seeds; a 5 MHz one stays within 0.3 MHz. */
const VideoGrid* RfEngine::video_grid(const RfSeg& s, size_t wc) const {
    const uint32_t f = s.fm_khz;
    const uint32_t piece_khz = ((points_ / kFloorSegs) * q8_) >> 8;
    if ((q8_ >> 8) > kGridMaxPtKhz || s.w10_khz * 100u > piece_khz * kGridMaxPiecePct) return nullptr;
    for (const VideoGrid& g : kVideoGrids) {
        if (g.band != band_ || g.width_mhz != kWidthMhz[wc]) continue;
        if (f + g.tol_khz < g.lo_100khz * 100u || f > g.hi_100khz * 100u + g.tol_khz) continue;
        const uint32_t r = (f + g.step_khz - g.offset_khz) % g.step_khz;
        if ((r < g.step_khz - r ? r : g.step_khz - r) <= g.tol_khz) return &g;
    }
    return nullptr;
}

/* Digital video of width class wc: "DIG20 5805", a GRADE_DOC width class
 * "DIG5? 2440" (low), on a video grid "DJI?20 2423" (the grid never raises
 * the confidence). */
void RfEngine::digital(RfSeg& s, size_t wc, uint8_t conf, const VideoGrid* grid) {
    const bool doc_width = kWidthGrade[wc] == GRADE_DOC;
    s.cls = RC_DIGITAL;
    s.conf = doc_width ? uint8_t{CONF_LOW} : conf;
    Fmt l{s.label, sizeof(s.label)};
    l.s(grid ? grid->name : "DIG");
    if (grid && grid->grade == GRADE_DOC) l.c('?');
    l.d(kWidthMhz[wc]);
    if (!grid && doc_width) l.c('?');
    l.c(' ').d(static_cast<int32_t>((s.f_khz + 500) / 1000));
}

bool RfEngine::excluded(size_t x) const {
    if (w_->nhit[x] * 100u >= kFixedPct * looks_) return true;
    for (size_t i = 0; i < nsegs_; i++) {
        const RfSeg& s = w_->segs[i];
        const size_t a = s.lo < s.l10 ? s.lo : s.l10;
        const size_t b = s.hi > s.r10 ? s.hi : s.r10;
        if (x + 1 >= a && x <= b + 1) return true;
    }
#if SIGDET_FULL
    if (band_ == BAND_24)
        for (const uint32_t c : kBleAdvKhz)
            if (dist(point_khz(x), c) <= kBleExclKhz) return true;
#endif
    return false;
}

void RfEngine::hit_stats() {
    RfHits& h = hits_;
    uint32_t f_min = 0, f_max = 0, w_min = 0, w_max = 0, dl = 0;
    size_t x = 0;
    while (x < points_) {
        if (w_->nhit[x] && !excluded(x)) {
            uint32_t n = 0, xs = 0;
            size_t gap = 0;
            for (; x < points_ && gap <= 1; x++) {
                if (w_->nhit[x] && !excluded(x)) {
                    n += w_->nhit[x];
                    xs += w_->nhit[x] * x;
                    if (w_->maxh[x] > h.level) h.level = w_->maxh[x];
                    gap = 0;
                } else {
                    gap++;
                }
            }
            const uint32_t f = lo_khz_ + ((((xs * 16) / n) * q8_) >> 12);
            if (!h.distinct) f_min = f;
            f_max = f;
            if (h.distinct < 255) h.distinct++;
            h.hits += n;
            size_t i = 0;
            for (const HopGrid& g : kHopGrids) {
                if (g.band != band_ || i >= RfHits::kMaxGrids) continue;
                if (on_grid(f, g.base_khz, g.step_khz, g.count, g.tol_khz)) h.grid_hits[i] += n;
                i++;
            }
            if (f >= kLoraDlLoKhz && f <= kLoraDlHiKhz) dl += n;
        } else {
            x++;
        }
    }
    h.on_grid = h.grid_hits[0];
    h.span_khz = f_max - f_min;
    h.f_khz = (f_min + f_max) / 2;
    if (h.hits && band_ == BAND_915 && dl * 100 >= kLoraDlRejectPct * h.hits) h.hits = 0; /* LoRaWAN downlinks */

#if SIGDET_FULL
    if (band_ != BAND_24) return;
    for (x = 0; x < points_; x++) {
        const uint32_t f = point_khz(x);
        if (!w_->nwide[x] || f < kOvenLoKhz || f > kOvenHiKhz || excluded(x)) continue;
        if (!h.wide) w_min = f;
        w_max = f;
        h.wide += w_->nwide[x];
    }
    const uint32_t share = (h.wide * 100u) / looks_;
    h.wide_looks = share > 100 ? 100 : share;
    h.wide_spread_khz = w_max - w_min;
#else
    (void)w_min;
    (void)w_max;
#endif
}

void RfEngine::classify_hits() {
    RfHits& h = hits_;
    Fmt l{h.label, sizeof(h.label)};
#if SIGDET_FULL
    if (band_ == BAND_24) {
        bool oven = h.wide_looks >= kOvenMinPct && h.wide_looks <= kOvenMaxPct && h.wide_spread_khz >= kOvenMinSpreadKhz;
        for (size_t i = 0; i < nsegs_; i++) oven = oven || w_->segs[i].cls == RC_OVEN;
        if (oven) {
            h.cls = RC_OVEN;
            l.s("Oven");
            return;
        }
    }
#endif
    if (h.hits < kMinHits) return;
    const uint32_t grid = (h.on_grid * 100u) / h.hits;
    const uint8_t d = h.distinct;
    h.conf = CONF_LOW;
    uint8_t conf = CONF_LOW;
    const HopGrid* named = named_grid(conf);
#if SIGDET_FULL
    if (band_ == BAND_24) {
        if (!hop24_) {
            h.conf = CONF_NONE;
            return;
        }
        if (d >= kHop24MinDistinct) {
            h.cls = RC_HOP24;
            if (named) {
                h.conf = conf; /* low: every 2.4 GHz hopper verdict is */
                grid_label(l, *named, kBands[band_].name);
                return;
            }
            l.s("Hop24 ");
        } else if (d >= 2 && d <= kToyMaxDistinct && grid >= kToyMinGridPct) {
            h.cls = RC_TOY;
            l.s("Toy ");
        } else {
            return;
        }
        l.d(d).s("ch");
        return;
    }
#endif
    if (h.thin * 100u >= kThinRejectPct * (h.thin + h.hits)) return; /* LoRaWAN 125k, meters */
    const uint32_t span = band_ == BAND_868 ? kElrsEuMinSpanKhz : kElrsMinSpanKhz;
    if (h.span_khz < span) return;
    /* A further grid that stands clear of the others, else the primary
     * grid's rule (ELRS), else generic FHSS. */
    const HopGrid* p = band_grid(band_, 0);
    if (!named && p && d >= p->min_distinct && grid >= p->min_pct) {
        named = p;
        conf = grid_conf(*p, d, grid);
    }
    h.cls = RC_CTRL;
    if (named) {
        h.conf = conf;
        grid_label(l, *named, kBands[band_].name);
    } else if (d >= kFhssMinDistinct) {
        l.s("FHSS ").s(kBands[band_].name);
    } else {
        h.cls = RC_NONE;
    }
}

/* The band's further hop grid (not the primary) that names the hits: its own
 * minimum distinct channels and share, and kGridMarginPct over every other
 * grid of the band. Its confidence goes to conf. */
const HopGrid* RfEngine::named_grid(uint8_t& conf) const {
    const RfHits& h = hits_;
    if (!h.hits) return nullptr;
    const HopGrid* best = nullptr;
    uint32_t best_pct = 0, other = 0;
    size_t i = 0;
    for (const HopGrid& g : kHopGrids) {
        if (g.band != band_ || i >= RfHits::kMaxGrids) continue;
        const uint32_t pct = (h.grid_hits[i] * 100u) / h.hits;
        if (i > 0 && h.distinct >= g.min_distinct && pct >= g.min_pct && pct > best_pct) {
            if (best_pct > other) other = best_pct;
            best = &g;
            best_pct = pct;
        } else if (pct > other) {
            other = pct;
        }
        i++;
    }
    if (!best || best_pct < other + kGridMarginPct) return nullptr;
    conf = grid_conf(*best, h.distinct, best_pct);
    return best;
}

void RfEngine::track(uint8_t cls, uint8_t conf, uint32_t f_khz, uint32_t bw_khz, int16_t level_cdb, uint8_t duty, uint8_t flat, const char* label) {
    const bool hopper = family(cls) == RC_CTRL || family(cls) == RC_HOP24;
    RfTrack* t = nullptr;
    for (auto& c : tracks_) {
        if (!c.uid || c.band != band_ || !c.miss || family(c.cls) != family(cls)) continue;
        const uint32_t tol = c.bw_khz / 4 > kMatchKhz ? c.bw_khz / 4 : kMatchKhz;
        if (hopper || dist(f_khz, c.f_khz) <= tol) {
            t = &c;
            break;
        }
    }
    Event ev = EV_NEW;
    if (t) {
        if (t->hits < 255) t->hits++;
        ev = EV_UPD;
    } else {
        for (auto& c : tracks_) {
            if (!c.uid) {
                t = &c;
                break;
            }
            if (!c.confirmed && (!t || c.miss > t->miss)) t = &c;
        }
        if (!t) return; /* table full of confirmed tracks */
        *t = RfTrack{};
        t->uid = next_uid_++;
        if (!next_uid_) next_uid_ = 1;
        t->band = band_;
        t->hits = 1;
    }
    t->miss = 0;
    t->cls = cls;
    t->conf = conf;
    const uint8_t fam = family(cls);
    const bool duty_class = fam == RC_ANALOG || fam == RC_DIGITAL;
    if (duty_class) {
        if (t->acc_looks > 1000) {
            t->acc_looks /= 2;
            t->acc_occ /= 2;
        }
        t->acc_looks += looks_;
        t->acc_occ += (looks_ * duty + 50) / 100;
    }
    t->f_khz = f_khz;
    t->bw_khz = bw_khz;
    t->level = static_cast<int8_t>(level_cdb / 100 > 127 ? 127 : level_cdb / 100);
    t->duty = duty;
    t->flat = flat;
    memcpy(t->label, label, sizeof(t->label));
    /* Duty over all the track's looks: analog >= kAnalogMinDuty, digital
     * >= kDigMinDuty, DV/WF (the Wi-Fi grid) >= kWifiMaxDuty. */
    const uint8_t need_duty = cls == RC_ANALOG ? kAnalogMinDuty : (cls == RC_DIGITAL ? kDigMinDuty : kWifiMaxDuty);
    const bool duty_ok = !duty_class || (t->acc_looks >= kConfirmLooks && t->acc_duty() >= need_duty);
    if (!t->confirmed && duty_ok && t->hits >= (conf == CONF_LOW ? kConfirmDwellsLow : kConfirmDwells)) {
        t->confirmed = true;
        ev = EV_CONF;
    }
    if (ev != EV_UPD || t->confirmed) emit(ev, *t);
}

void RfEngine::drop_tracks(uint8_t band, uint8_t min_miss) {
    for (auto& t : tracks_) {
        if (!t.uid || t.band != band || t.miss < min_miss) continue;
        if (t.confirmed) emit(EV_DROP, t);
        t.uid = 0;
    }
}

/* DDSWP lines, one dwell (all numbers decimal unless noted):
 *   DDSWP,seq,band,H,t_ms,lo_khz,hi_khz,points,looks,dwell_ms,slices,thr_cdb
 *   DDSWP,seq,band,<A>,first,HEX  per point arrays, 64 points (2 hex digits each) per line:
 *     F floor: (cdBFS + 12800) / 50     A mean over floor: (cdB + 2000) / 50
 *     O looks occupied                  M max hold over floor / 50 cdB
 *     N 0.25..3.2 MHz hit centres       W 3.2..8 MHz hit centres (SIGDET_FULL)
 *   DDSWP,seq,band,S,i,lo,hi,l10,r10,l3,r3,f_khz,w10_khz,w3_khz,snr_cdb,hold_halfdb,flat_decidb,duty,class,conf,label
 *   DDSWP,seq,band,K,looks_any,thin,hits,distinct,on_grid,span_khz,f_khz,wide,wide_pct,wide_spread_khz,level_halfdb,class,conf,label
 *   DDSWP,seq,band,T,uid,class,conf,confirmed,hits,miss,f_khz,bw_khz,level_db,duty,flat_decidb,acc_duty,acc_looks,label
 *   DDSWP,seq,band,E,lines (this one included) */
void RfEngine::dump(Sink sink, void* ctx, uint32_t seq, uint32_t t_ms, uint16_t ms, uint16_t slices) const {
    if (!w_) return;
    using I = int32_t;
    char buf[160];
    I lines = 0;
    auto head = [&]() {
        Fmt f{buf, sizeof(buf)};
        f.s("DDSWP,").d(seq).c(',').s(kBands[band_].name).c(',');
        lines++;
        return f;
    };
    auto line = [&](const char* tmpl, const I* v, const char* const* str) {
        head().t(tmpl, v, str);
        sink(ctx, buf);
    };
    {
        const I v[] = {I(t_ms), I(lo_khz_), I(point_khz(points_ - 1)), points_, looks_, ms, slices, thr_};
        line("H,#,#,#,#,#,#,#,#", v, nullptr);
    }
    const int16_t* mean = reinterpret_cast<const int16_t*>(w_->sum);
#if SIGDET_FULL
    const uint8_t* const bytes[4] = {w_->occ, w_->maxh, w_->nhit, w_->nwide};
    constexpr size_t kArrays = 6;
#else
    const uint8_t* const bytes[3] = {w_->occ, w_->maxh, w_->nhit};
    constexpr size_t kArrays = 5;
#endif
    for (size_t a = 0; a < kArrays; a++) {
        for (size_t x0 = 0; x0 < points_; x0 += 64) {
            Fmt f = head();
            f.c("FAOMNW"[a]).c(',').d(x0).c(',');
            for (size_t x = x0; x < x0 + 64 && x < points_; x++) {
                const uint8_t v = a == 0 ? sat8((w_->floor[x] + 12800) / 50) : (a == 1 ? sat8((mean[x] + kSumOffsetCdb) / 50) : bytes[a - 2][x]);
                f.hex(&v, 1);
            }
            sink(ctx, buf);
        }
    }
    for (size_t i = 0; i < nsegs_; i++) {
        const RfSeg& s = w_->segs[i];
        const I v[] = {I(i), s.lo, s.hi, s.l10, s.r10, s.l3, s.r3, I(s.f_khz), I(s.w10_khz), I(s.w3_khz), s.snr, s.hold, s.flat, s.duty, s.conf};
        const char* const str[] = {rf_class_tag(s.cls), s.label};
        line("S,#,#,#,#,#,#,#,#,#,#,#,#,#,#,$,#,$", v, str);
    }
    if (kBands[band_].flags & BF_HOP) {
        const RfHits& h = hits_;
        const I v[] = {h.looks_any, h.thin, h.hits, h.distinct, h.on_grid, I(h.span_khz), I(h.f_khz), h.wide, h.wide_looks, I(h.wide_spread_khz), h.level, h.conf};
        const char* const str[] = {rf_class_tag(h.cls), h.label};
        line("K,#,#,#,#,#,#,#,#,#,#,#,$,#,$", v, str);
    }
    for (const auto& t : tracks_) {
        if (!t.uid || t.band != band_) continue;
        const I v[] = {t.uid, t.conf, t.confirmed, t.hits, t.miss, I(t.f_khz), I(t.bw_khz), t.level, t.duty, t.flat, t.acc_duty(), t.acc_looks};
        const char* const str[] = {rf_class_tag(t.cls), t.label};
        line("T,#,$,#,#,#,#,#,#,#,#,#,#,#,$", v, str);
    }
    const I v[] = {lines + 1};
    line("E,#", v, nullptr);
}

} /* namespace ui::external_app::signature_detect */
