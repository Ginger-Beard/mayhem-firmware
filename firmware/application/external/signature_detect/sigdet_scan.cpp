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

#include "sigdet_scan.hpp"

#include <cstring>

#include "sigdet_fmt.hpp"

namespace ui::external_app::signature_detect {

using namespace scan_tune;

namespace {

uint8_t to_u8(int16_t cdb) {
    const int32_t v = (static_cast<int32_t>(cdb) + kU8OffsetCdb + 25) / 50;
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

uint8_t sub0(int32_t a, int32_t b) {
    const int32_t d = a - b;
    return static_cast<uint8_t>(d < 0 ? 0 : (d > 255 ? 255 : d));
}

uint32_t absdiff(uint32_t a, uint32_t b) {
    return a > b ? a - b : b - a;
}

/* Smallest v with at least q of n values <= v (values 0..255): a counting
 * search, no sort buffer. get(i) gives the i-th value. */
template <class Get>
uint8_t quantile(size_t n, size_t q, Get get) {
    uint32_t lo = 0, hi = 255;
    while (lo < hi) {
        const uint32_t mid = (lo + hi) / 2;
        size_t c = 0;
        for (size_t i = 0; i < n; i++)
            if (get(i) <= mid) c++;
        if (c >= q)
            hi = mid;
        else
            lo = mid + 1;
    }
    return static_cast<uint8_t>(lo);
}

}  // namespace

const char* scan_type_tag(uint8_t t) {
    static const char* const kTags[ST_COUNT] = {"----", "COMB", "PCMB", "FLOR", "DRFT", "HUM", "STDY", "BRST", "BLOK", "WIDE", "RND?"};
    return t < ST_COUNT ? kTags[t] : "----";
}

size_t ScanEngine::configure(const ScanRange* ranges, size_t n) {
    nrange_ = 0;
    nwin_ = 0;
    for (size_t i = 0; i < n && nrange_ < kMaxRanges; i++) {
        ScanRange r = ranges[i];
        if (r.points < 16) r.points = 16;
        if (r.points > kMaxPoints) r.points = kMaxPoints;
        if (r.looks < 1) r.looks = 1;
        if (r.looks > kMaxLooks) r.looks = kMaxLooks;
        if (r.res_hz < 1000 || r.res_hz > 1000000 || r.hi_khz <= r.lo_khz) continue;
        const uint64_t span = static_cast<uint64_t>(r.points - 1) * r.res_hz;
        const uint64_t width = static_cast<uint64_t>(r.hi_khz - r.lo_khz) * 1000;
        uint64_t nw = (width + span - 1) / span;
        if (nw > kMaxWindows - nwin_) nw = kMaxWindows - nwin_;
        if (!nw) break;
        range_[nrange_] = r;
        first_[nrange_] = static_cast<uint16_t>(nwin_);
        nwin_ += static_cast<size_t>(nw);
        nrange_++;
    }
    first_[nrange_] = static_cast<uint16_t>(nwin_);

    pass_ = 1;
    t_s_ = 0;
    active_ = false;
    memset(floor_, 0, sizeof(floor_));
    memset(rise_run_, 0, sizeof(rise_run_));
    memset(block_pct_, 0, sizeof(block_pct_));
    memset(surveyed_, 0, sizeof(surveyed_));
    for (auto& e : em_) e = Emitter{};
    for (auto& c : comb_) c = Comb{};
    for (auto& f : fl_) f = Floor{};
    nruns_ = 0;
    fit_ = CombFit{};
    reset_baseline();
    return nrange_;
}

void ScanEngine::reset_baseline() {
    memset(base_, 0, sizeof(base_));
    memset(low_, 0, sizeof(low_));
    memset(rise_run_, 0, sizeof(rise_run_));
    base_passes_ = 0;
}

void ScanEngine::plan(size_t w, ScanWindowPlan& p) const {
    p = ScanWindowPlan{};
    if (w >= nwin_) return;
    size_t r = 0;
    while (r + 1 < nrange_ && w >= first_[r + 1]) r++;
    const ScanRange& g = range_[r];
    const uint64_t span = static_cast<uint64_t>(g.points - 1) * g.res_hz;
    p.index = static_cast<uint16_t>(w - first_[r]);
    p.start_hz = static_cast<uint64_t>(g.lo_khz) * 1000 + p.index * span;
    p.span_hz = span;
    p.points = g.points;
    p.looks = g.looks;
    p.rbw_idx = g.rbw_idx;
    p.range = static_cast<uint8_t>(r);
}

bool ScanEngine::begin(size_t w) {
    active_ = false;
    if (!w_ || w >= nwin_) return false;
    ScanWindowPlan p;
    plan(w, p);
    cur_w_ = static_cast<uint16_t>(w);
    cur_range_ = p.range;
    points_ = p.points;
    start_hz_ = p.start_hz;
    looks_ = 0;
    active_ = true;
    return true;
}

bool ScanEngine::add_look(const int16_t* cdb) {
    if (!active_ || !w_ || looks_ >= range_[cur_range_].looks) return false;
    uint8_t* dst = w_->look[looks_];
    for (size_t x = 0; x < points_; x++) dst[x] = to_u8(cdb[x]);
    looks_++;
    return looks_ < range_[cur_range_].looks;
}

uint32_t ScanEngine::f10_of(uint32_t p16) const {
    const uint64_t res = range_[cur_range_].res_hz;
    return static_cast<uint32_t>(start_hz_ / 10 + (static_cast<uint64_t>(p16) * res + 80) / 160);
}

uint8_t ScanEngine::new_uid() {
    for (size_t tries = 0; tries < 256; tries++) {
        const uint8_t u = next_uid_;
        next_uid_ = static_cast<uint8_t>(next_uid_ == 255 ? 1 : next_uid_ + 1);
        bool used = false;
        for (const auto& e : em_) used |= e.uid == u;
        for (const auto& c : comb_) used |= c.uid == u;
        for (const auto& f : fl_) used |= f.uid == u;
        if (!used) return u;
    }
    return 0;
}

/* ---- one window -------------------------------------------------------- */

void ScanEngine::finish(uint32_t t_s) {
    t_s_ = t_s;
    const bool last = cur_w_ + 1u >= nwin_;
    if (active_ && w_ && looks_) {
        for (size_t x = 0; x < points_; x++) {
            uint32_t s = 0;
            for (size_t l = 0; l < looks_; l++) s += w_->look[l][x];
            w_->mean[x] = static_cast<uint8_t>((s + looks_ / 2) / looks_);
        }
        find_floor();
        floor_[cur_w_] = quiet_floor();
        find_runs();
        window_comb();
        emitters_from_runs();

        /* Block / wide survey rows, once per window and session. */
        const uint8_t bit = static_cast<uint8_t>(1u << (cur_w_ & 7));
        for (size_t i = 0; i < nruns_ && !(surveyed_[cur_w_ >> 3] & bit); i++) {
            const Run& r = runs_[i];
            if (r.type != ST_BLOCK && r.type != ST_WIDE) continue;
            surveyed_[cur_w_ >> 3] |= bit;
            ScanEvent e{};
            e.ev = SE_SURVEY;
            e.type = r.type;
            e.conf = SC_LOW;
            e.range = cur_range_;
            e.f_khz = khz_of(r.c16);
            e.lo_khz = khz_of(r.lo * 16u);
            e.hi_khz = khz_of(r.hi * 16u);
            e.level_cdb = static_cast<int16_t>(r.ex_mean * 50);
            e.duty = r.duty;
            e.width_pts = static_cast<uint8_t>(r.hi - r.lo + 1 > 255 ? 255 : r.hi - r.lo + 1);
            e.param = r.ripple * 50u;
            e.count = r.fill;
            e.res_hz = range_[cur_range_].res_hz;
            emit(e);
        }
    }
    active_ = false;
    if (last) close_pass();
}

void ScanEngine::find_floor() {
    const size_t pw = points_ / kPieces;
    uint8_t sorted[kPieces];
    for (size_t k = 0; k < kPieces; k++) {
        const size_t a = k * pw;
        const size_t n = (k + 1 == kPieces) ? points_ - a : pw;
        const uint8_t* m = w_->mean + a;
        piece_[k] = quantile(n, (n + 3) / 4, [m](size_t i) { return m[i]; });
        sorted[k] = piece_[k];
    }
    for (size_t i = 1; i < kPieces; i++)
        for (size_t j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
            const uint8_t t = sorted[j];
            sorted[j] = sorted[j - 1];
            sorted[j - 1] = t;
        }
    wfloor_ = sorted[1];
    const uint32_t cap = sorted[1] + kFloorCapCdb;
    for (size_t k = 0; k < kPieces; k++)
        if (piece_[k] > cap) piece_[k] = static_cast<uint8_t>(cap);
}

/* The floor of the quietest look (second lowest eighth's lower quartile, the
 * lowest over the looks): what the floor rise detector compares. Bursty
 * traffic (Wi-Fi, LTE uplink) seldom fills all looks 4 ms apart, while a
 * continuous noise source lifts every look. Noise gated by the mains (bursts
 * of ~3 ms every 8.3 / 10 ms) can leave one look clean: a stated limit. */
uint8_t ScanEngine::quiet_floor() const {
    const size_t pw = points_ / kPieces;
    uint8_t best = 255;
    for (size_t l = 0; l < looks_; l++) {
        const uint8_t* v = w_->look[l];
        uint8_t lo1 = 255, lo2 = 255;
        for (size_t k = 0; k < kPieces; k++) {
            const size_t a = k * pw;
            const size_t n = (k + 1 == kPieces) ? points_ - a : pw;
            const uint8_t* m = v + a;
            const uint8_t q = quantile(n, (n + 3) / 4, [m](size_t i) { return m[i]; });
            if (q < lo1) {
                lo2 = lo1;
                lo1 = q;
            } else if (q < lo2) {
                lo2 = q;
            }
        }
        if (lo2 < best) best = lo2;
    }
    return best ? best : 1; /* 0 means "not measured" */
}

uint8_t ScanEngine::floor_at(size_t x) const {
    const size_t pw = points_ / kPieces;
    const size_t half = pw / 2;
    if (x <= half) return piece_[0];
    const size_t k = (x - half) / pw;
    if (k + 1 >= kPieces) return piece_[kPieces - 1];
    const int32_t a = piece_[k], b = piece_[k + 1];
    const int32_t u = static_cast<int32_t>(x - half - k * pw);
    return static_cast<uint8_t>(a + ((b - a) * u + (b > a ? static_cast<int32_t>(pw) / 2 : -static_cast<int32_t>(pw) / 2)) / static_cast<int32_t>(pw));
}

void ScanEngine::find_runs() {
    nruns_ = 0;
    const uint8_t need = looks_ > 1 ? kOccLooks : 1;
    size_t start = 0;
    bool in = false;
    uint32_t block_pts = 0;
    for (size_t x = 0; x <= points_; x++) {
        bool on = false;
        if (x < points_) {
            const uint8_t fl = floor_at(x);
            if (sub0(w_->mean[x], fl) >= kOccMean) {
                on = true;
            } else {
                uint8_t c = 0;
                for (size_t l = 0; l < looks_; l++) c += sub0(w_->look[l][x], fl) >= kOccLook;
                on = c >= need;
            }
        }
        if (on && !in) {
            start = x;
            in = true;
            continue;
        }
        if (on || !in) continue;
        in = false;

        Run r{};
        r.lo = static_cast<uint16_t>(start);
        r.hi = static_cast<uint16_t>(x - 1);
        uint32_t sw = 0, sxw = 0;
        uint8_t best = 0;
        for (size_t p = start; p < x; p++) {
            const uint8_t fl = floor_at(p);
            const uint8_t e = sub0(w_->mean[p], fl);
            if (e > best || p == start) {
                best = e;
                r.peak = static_cast<uint16_t>(p);
            }
            for (size_t l = 0; l < looks_; l++) {
                const uint8_t m = sub0(w_->look[l][p], fl);
                if (m > r.ex_max) r.ex_max = m;
            }
            sw += e + 1u;
            sxw += (e + 1u) * static_cast<uint32_t>(p) * 16u;
        }
        r.ex_mean = best;
        r.c16 = (sxw + sw / 2) / sw;
        {
            const uint8_t fl = floor_at(r.peak);
            uint8_t on_looks = 0, mn = 255;
            for (size_t l = 0; l < looks_; l++) {
                const uint8_t e = sub0(w_->look[l][r.peak], fl);
                on_looks += e >= kOccMean;
                if (e < mn) mn = e;
            }
            r.duty = static_cast<uint8_t>(on_looks * 100u / looks_);
            r.always = mn >= kOccMean;
        }
        const uint32_t width_hz = (static_cast<uint32_t>(r.hi - r.lo) + 1) * range_[cur_range_].res_hz;
        r.type = ST_STEADY;
        if (width_hz >= kBlockMinHz) {
            measure_wide(r);
            r.type = (r.ripple <= kBlockRipple && r.fill >= kBlockFillPct) ? ST_BLOCK : ST_WIDE;
            if (r.type == ST_BLOCK) block_pts += r.hi - r.lo + 1u;
        }
        if (nruns_ < kMaxRuns) {
            runs_[nruns_++] = r;
        } else {
            /* Full: the weakest goes. */
            size_t wi = 0;
            for (size_t i = 1; i < nruns_; i++)
                if (runs_[i].ex_mean < runs_[wi].ex_mean) wi = i;
            if (r.ex_mean > runs_[wi].ex_mean) runs_[wi] = r;
        }
    }
    block_pct_[cur_w_] = static_cast<uint8_t>(block_pts * 100u / points_);
}

void ScanEngine::measure_wide(Run& r) const {
    const size_t w = r.hi - r.lo + 1u;
    const size_t a = r.lo + w / 5, n = w - 2 * (w / 5);
    auto ex = [this, a](size_t i) { return sub0(w_->mean[a + i], floor_at(a + i)); };
    const uint8_t p10 = quantile(n, (n + 9) / 10, ex);
    const uint8_t p50 = quantile(n, (n + 1) / 2, ex);
    const uint8_t p90 = quantile(n, (n * 9 + 9) / 10, ex);
    r.ripple = static_cast<uint8_t>(p90 - p10);
    size_t near = 0;
    for (size_t p = r.lo; p <= r.hi; p++)
        if (sub0(w_->mean[p], floor_at(p)) + 6 >= p50) near++;
    r.fill = static_cast<uint8_t>(near * 100u / w);
}

size_t ScanEngine::find_lines(Line* out, bool pulsed) const {
    size_t n = 0;
    auto v = [this, pulsed](size_t x) -> uint8_t {
        if (!pulsed) return w_->mean[x];
        uint8_t m = 0;
        for (size_t l = 0; l < looks_; l++)
            if (w_->look[l][x] > m) m = w_->look[l][x];
        return m;
    };
    for (size_t x = 1; x + 1 < points_; x++) {
        const uint8_t b = v(x);
        const uint8_t fl = floor_at(x);
        const uint8_t e = sub0(b, fl);
        if (e < (pulsed ? kOccLook : kOccMean)) continue;
        const uint8_t a = v(x - 1), c = v(x + 1);
        if (b < a || b <= c) continue;
        const uint8_t left = x >= 2 ? (a < v(x - 2) ? a : v(x - 2)) : a;
        const uint8_t right = x + 2 < points_ ? (c < v(x + 2) ? c : v(x + 2)) : c;
        const uint8_t flank = left > right ? left : right;
        if (sub0(b, flank) < kLineProm) continue;
        /* Narrow: at most kLineMaxW points within 6 dB of the peak. */
        size_t wl = x, wr = x;
        while (wl > 0 && v(wl - 1) + kLineProm > b) wl--;
        while (wr + 1 < points_ && v(wr + 1) + kLineProm > b) wr++;
        if (wr - wl + 1 > kLineMaxW) continue;
        uint8_t mn = 255, on = 0;
        for (size_t l = 0; l < looks_; l++) {
            if (w_->look[l][x] < mn) mn = w_->look[l][x];
            on += sub0(w_->look[l][x], fl) >= kOccMean;
        }
        if (pulsed && sub0(mn, fl) >= kOccMean) continue; /* always on: a continuous line */
        /* Parabolic peak: delta = (a - c) / (2 (a - 2b + c)), 1/16 point. */
        const int32_t den = static_cast<int32_t>(a) - 2 * b + c;
        int32_t d16 = den ? (8 * (static_cast<int32_t>(a) - c)) / den : 0;
        if (d16 > 8) d16 = 8;
        if (d16 < -8) d16 = -8;
        const Line ln{static_cast<uint32_t>(static_cast<int32_t>(x) * 16 + d16), e, static_cast<uint8_t>(on * 100u / looks_), pulsed};
        if (n < kMaxLines) {
            out[n++] = ln;
        } else {
            size_t wi = 0;
            for (size_t i = 1; i < n; i++)
                if (out[i].ex < out[wi].ex) wi = i;
            if (ln.ex > out[wi].ex) out[wi] = ln;
        }
    }
    for (size_t i = 1; i < n; i++)
        for (size_t j = i; j > 0 && out[j - 1].p16 > out[j].p16; j--) {
            const Line t = out[j];
            out[j] = out[j - 1];
            out[j - 1] = t;
        }
    return n;
}

namespace {

/* A comb grid: place k at (a256 + k x d256) / 16, in the lines' 1/16 point
 * units (1/256 point keeps the fitted spacing's fraction). 32 bit: positions
 * stay under 320 x 256 (the M0 divides 64 bit numbers slowly). */
struct Grid {
    int32_t a256, d256;
};

/* Lines within `radius` places of place 0 and `tol` (1/16 point) of their
 * place; idx gets the place or INT16_MIN. Returns the number matched. */
size_t grid_match(const ScanEngine::Line* L, size_t n, const Grid& g, int32_t radius, int32_t tol, int16_t* idx) {
    size_t matched = 0;
    int32_t prev = INT32_MIN;
    for (size_t m = 0; m < n; m++) {
        idx[m] = INT16_MIN;
        const int32_t off = static_cast<int32_t>(L[m].p16) * 16 - g.a256;
        const int32_t k = (off >= 0 ? off + g.d256 / 2 : off - g.d256 / 2) / g.d256;
        if (k > radius || k < -radius || k == prev) continue;
        const int32_t r = off - k * g.d256;
        if ((r < 0 ? -r : r) > tol * 16) continue;
        idx[m] = static_cast<int16_t>(k);
        prev = k;
        matched++;
    }
    return matched;
}

/* Least squares place -> position over the matched lines. */
bool grid_fit(const ScanEngine::Line* L, size_t n, const int16_t* idx, Grid& g) {
    int64_t m = 0, sn = 0, sp = 0;
    for (size_t i = 0; i < n; i++)
        if (idx[i] != INT16_MIN) {
            m++;
            sn += idx[i];
            sp += L[i].p16;
        }
    if (m < 2) return false;
    int64_t snn = 0, snp = 0;
    for (size_t i = 0; i < n; i++)
        if (idx[i] != INT16_MIN) {
            const int64_t dn = idx[i] * m - sn;
            const int64_t dp = static_cast<int64_t>(L[i].p16) * m - sp;
            snn += dn * dn;
            snp += dn * dp;
        }
    if (!snn) return false;
    const int64_t d256 = (16 * snp + snn / 2) / snn;
    if (d256 <= 0 || d256 > INT32_MAX / 4) return false;
    g.d256 = static_cast<int32_t>(d256);
    g.a256 = static_cast<int32_t>((16 * sp - d256 * sn) / m);
    return true;
}

}  // namespace

ScanEngine::CombFit ScanEngine::fit_comb(const Line* L, size_t n, uint16_t points, bool pulsed) {
    CombFit best{};
    if (n < kCombMinTeeth) return best;
    const uint32_t max_sp = static_cast<uint32_t>(points) * 16u / kCombSpanDiv;
    int16_t idx[kMaxLines];
    for (size_t i = 0; i + 1 < n; i++) {
        for (size_t step = 1; step <= 2 && i + step < n; step++) {
            const uint32_t d = (L[i + step].p16 - L[i].p16) / static_cast<uint32_t>(step);
            if (d < kCombMinSp16 || d > max_sp) continue;
            /* Grow the grid outwards from line i, refitting as it goes: a
             * harmonic comb is exactly periodic, so every tooth must sit
             * within kCombTol of the fitted grid. */
            Grid g{static_cast<int32_t>(L[i].p16) * 16, static_cast<int32_t>(d) * 16};
            static constexpr int32_t kRadius[] = {2, 5, 12, 1000};
            size_t matched = 0;
            bool ok = true;
            for (size_t round = 0; round < 4 && ok; round++) {
                matched = grid_match(L, n, g, kRadius[round], round ? kCombTol : kCombTol + 2, idx);
                if (round < 3) ok = grid_fit(L, n, idx, g);
            }
            if (!ok || matched < kCombMinTeeth) continue;
            const uint32_t sp16 = static_cast<uint32_t>((g.d256 + 8) / 16);
            if (sp16 < kCombMinSp16 || sp16 > max_sp) continue;
            int32_t nmin = INT32_MAX, nmax = INT32_MIN;
            for (size_t m = 0; m < n; m++)
                if (idx[m] != INT16_MIN) {
                    if (idx[m] < nmin) nmin = idx[m];
                    if (idx[m] > nmax) nmax = idx[m];
                }
            const uint16_t places = static_cast<uint16_t>(nmax - nmin + 1);
            if (matched * 100u < kCombFillPct * static_cast<uint32_t>(places)) continue;
            if (matched < best.teeth || (matched == best.teeth && places >= best.places)) continue;

            /* Levels: mean, and the median step between neighbour teeth. */
            uint32_t lev = 0, duty = 0;
            uint8_t steps[kMaxLines];
            size_t ns = 0;
            int32_t last_k = INT32_MIN;
            uint8_t last_ex = 0;
            for (size_t m = 0; m < n; m++) {
                if (idx[m] == INT16_MIN) continue;
                lev += L[m].ex;
                duty += L[m].duty;
                if (last_k != INT32_MIN && idx[m] == last_k + 1) steps[ns++] = static_cast<uint8_t>(L[m].ex > last_ex ? L[m].ex - last_ex : last_ex - L[m].ex);
                last_k = idx[m];
                last_ex = L[m].ex;
            }
            uint8_t med = 0;
            if (ns) {
                for (size_t a = 1; a < ns; a++)
                    for (size_t b = a; b > 0 && steps[b - 1] > steps[b]; b--) {
                        const uint8_t t = steps[b];
                        steps[b] = steps[b - 1];
                        steps[b - 1] = t;
                    }
                med = steps[ns / 2];
            }
            if (med > kCombStepDb) continue;
            const int32_t first = (g.a256 + nmin * g.d256 + 8) / 16;
            best.ok = true;
            best.duty = static_cast<uint8_t>(duty / matched);
            best.pulsed = pulsed || best.duty < kCombAlwaysPct;
            best.sp16 = sp16;
            best.first16 = static_cast<uint32_t>(first < 0 ? 0 : first);
            best.teeth = static_cast<uint16_t>(matched);
            best.places = places;
            best.level = static_cast<uint8_t>(lev / matched);
            best.step = med;
        }
    }
    return best;
}

void ScanEngine::window_comb() {
    Line lines[kMaxLines];
    size_t n = find_lines(lines, false);
    fit_ = fit_comb(lines, n, points_, false);
    if (!fit_.ok && looks_ > 1) {
        n = find_lines(lines, true);
        fit_ = fit_comb(lines, n, points_, true);
    }
    if (!fit_.ok) return;

    /* Runs that hold a tooth belong to the comb, not to the emitter table. */
    const uint32_t tol = fit_.sp16 / 8 > 8 ? fit_.sp16 / 8 : 8;
    const uint32_t last16 = fit_.first16 + (fit_.places - 1u) * fit_.sp16;
    for (size_t i = 0; i < nruns_; i++) {
        Run& r = runs_[i];
        const uint32_t p = r.peak * 16u;
        if (p + tol < fit_.first16 || p > last16 + tol) continue;
        const uint32_t off = p + fit_.sp16 - fit_.first16; /* >= 0 */
        const uint32_t m = off % fit_.sp16;
        r.comb = m <= tol || fit_.sp16 - m <= tol;
    }

    const ScanRange& g = range_[cur_range_];
    const uint32_t sp_hz = static_cast<uint32_t>((static_cast<uint64_t>(fit_.sp16) * g.res_hz + 8) / 16);
    const uint32_t lo_khz = khz_of(fit_.first16), hi_khz = khz_of(last16);

    const uint32_t f10_first = f10_of(fit_.first16);
    const uint32_t span_khz = static_cast<uint32_t>(static_cast<uint64_t>(g.points - 1) * g.res_hz / 1000);

    Comb* hit = nullptr;
    for (auto& c : comb_) {
        if (!c.uid || c.range != cur_range_) continue;
        /* The same spacing, near the comb's extent (2 windows) or anywhere on
         * its grid (a gated comb shows in a few windows a pass, rarely next
         * to each other): the same comb. A one-window fit far away that is
         * off the grid by more than an eighth of the spacing is another. */
        if (static_cast<uint64_t>(absdiff(c.sp_hz, sp_hz)) * 10000u > static_cast<uint64_t>(c.sp_hz) * kCombSameTol) continue;
        const uint32_t near_khz = 2 * span_khz;
        if (lo_khz > c.hi_khz + near_khz || hi_khz + near_khz < c.lo_khz) {
            const int64_t df = static_cast<int64_t>(f10_first) * 10 - static_cast<int64_t>(c.anchor10) * 10;
            const int64_t sp = c.sp_hz;
            const int64_t k = (df >= 0 ? df + sp / 2 : df - sp / 2) / sp;
            const int64_t err = df - k * sp;
            if ((err < 0 ? -err : err) * 8 > sp) continue;
        }
        hit = &c;
        break;
    }
    if (!hit) {
        for (auto& c : comb_)
            if (!c.uid) {
                hit = &c;
                break;
            }
        if (!hit) {
            /* Full: the oldest unreported one goes. */
            for (auto& c : comb_)
                if (!c.reported && (!hit || c.pass_last < hit->pass_last)) hit = &c;
            if (!hit) return;
        }
        *hit = Comb{};
        hit->uid = new_uid();
        if (!hit->uid) return;
        hit->sp_hz = sp_hz;
        hit->anchor10 = f10_first;
        hit->lo_khz = lo_khz;
        hit->hi_khz = hi_khz;
        hit->range = cur_range_;
        hit->pass_first = pass_;
        hit->pulsed = fit_.pulsed;
    } else {
        /* Long baseline: teeth far from the first ones seen pin the spacing
         * down (one window's fit is good to ~0.1 %, not enough to tell a
         * round spacing or to follow the grid across the range). Only when
         * the tooth count between them is unambiguous. */
        const int64_t df = static_cast<int64_t>(f10_first) * 10 - static_cast<int64_t>(hit->anchor10) * 10;
        const int64_t sp = hit->sp_hz;
        const int64_t k = (df >= 0 ? df + sp / 2 : df - sp / 2) / sp;
        const int64_t kabs = k < 0 ? -k : k;
        const int64_t err = df - k * sp;
        if (kabs > hit->base_k && (err < 0 ? -err : err) * 4 < sp) {
            hit->sp_hz = static_cast<uint32_t>(((df < 0 ? -df : df) + kabs / 2) / kabs);
            hit->base_k = static_cast<uint16_t>(kabs > 65535 ? 65535 : kabs);
        }
        if (fit_.teeth > hit->teeth && hit->base_k < fit_.teeth) hit->sp_hz = sp_hz;
    }
    if (lo_khz < hit->lo_khz) hit->lo_khz = lo_khz;
    if (hi_khz > hit->hi_khz) hit->hi_khz = hi_khz;
    if (fit_.teeth > hit->teeth) hit->teeth = fit_.teeth;
    if (fit_.level > hit->level) hit->level = fit_.level;
    if (!fit_.pulsed) hit->pulsed = false;
    if (hit->pass_last != pass_) {
        hit->seen++;
        hit->pass_last = pass_;
    }
    hit->upd = true;
}

/* A round spacing: a multiple of 100 kHz within 0.6 %. Either the
 * receiver's own clock harmonics (lines on round frequencies, 0.5 / 1 /
 * 2.5 MHz apart) or the occupied channels of a raster (FM broadcast every
 * 200 kHz: any subset 2, 3 ... channels apart is a geometric comb). A
 * switching supply's comb seldom has a round spacing; one that does is
 * missed (stated in the README). One window's spacing is good to ~0.5 %,
 * which sets the tolerance. */
bool ScanEngine::comb_clock(const Comb& c) const {
    if (c.sp_hz < kRoundGridHz) return false;
    const uint64_t k = (c.sp_hz + kRoundGridHz / 2) / kRoundGridHz;
    const uint64_t round = k * kRoundGridHz;
    return static_cast<uint64_t>(absdiff(c.sp_hz, static_cast<uint32_t>(round))) * 1000000u <= round * kRoundTolPpm;
}

/* A frequency on a known comb's grid (within 0.6 point), inside its extent. */
bool ScanEngine::on_comb(uint32_t f10, uint8_t range) const {
    const uint32_t res = range_[range].res_hz;
    for (const auto& c : comb_) {
        if (!c.uid || c.range != range || !c.sp_hz) continue;
        const uint64_t f_hz = static_cast<uint64_t>(f10) * 10;
        if (f_hz + c.sp_hz < static_cast<uint64_t>(c.lo_khz) * 1000 || f_hz > static_cast<uint64_t>(c.hi_khz) * 1000 + c.sp_hz) continue;
        const int64_t df = static_cast<int64_t>(f_hz) - static_cast<int64_t>(c.anchor10) * 10;
        int64_t m = df % static_cast<int64_t>(c.sp_hz);
        if (m < 0) m += c.sp_hz;
        const int64_t dev = m < static_cast<int64_t>(c.sp_hz) - m ? m : static_cast<int64_t>(c.sp_hz) - m;
        if (dev * 10 <= static_cast<int64_t>(res) * 6) return true;
    }
    return false;
}

void ScanEngine::emitters_from_runs() {
    const ScanRange& g = range_[cur_range_];
    for (size_t i = 0; i < nruns_; i++) {
        const Run& r = runs_[i];
        if (r.type != ST_STEADY || r.comb) continue;
        const uint32_t width = r.hi - r.lo + 1u;
        if (width * g.res_hz > kEmitMaxHz) continue;
        const uint32_t f10 = f10_of(r.c16);
        if (on_comb(f10, cur_range_)) continue;
        /* The same emitter: the run overlaps where it was last seen, give or
         * take half a point. Neighbouring channels (FM: 2.45 points apart)
         * stay apart; a carrier drifting less than its width a pass is
         * followed; a faster one becomes a new emitter (stated limit). */
        const uint32_t lo10 = f10_of(r.lo * 16u), hi10 = f10_of(r.hi * 16u);
        const uint32_t half10 = g.res_hz / 20;
        Emitter* hit = nullptr;
        for (auto& e : em_) {
            if (!e.uid || e.range != cur_range_) continue;
            if (lo10 > e.hi10 + half10 || hi10 + half10 < e.lo10) continue;
            if (!hit || absdiff(e.f10, f10) < absdiff(hit->f10, f10)) hit = &e;
        }
        const int16_t level = static_cast<int16_t>(r.ex_mean * 50);
        if (hit && hit->upd) {
            /* Seen twice in one pass (a window edge): the stronger run wins. */
            if (level > hit->level) hit->level = level;
            if (width > hit->width) hit->width = static_cast<uint8_t>(width > 255 ? 255 : width);
            continue;
        }
        if (!hit) {
            for (auto& e : em_)
                if (!e.uid) {
                    hit = &e;
                    break;
                }
            if (!hit) {
                /* Full: the least seen, longest unseen, unreported one goes. */
                for (auto& e : em_) {
                    if (scan_reported(e.type) || e.upd) continue;
                    if (!hit || e.pass_last < hit->pass_last || (e.pass_last == hit->pass_last && e.seen < hit->seen)) hit = &e;
                }
                if (!hit) continue;
            }
            *hit = Emitter{};
            hit->uid = new_uid();
            if (!hit->uid) continue;
            hit->f10 = hit->f10_first = hit->f10_min = hit->f10_max = f10;
            hit->t_first = t_s_;
            hit->pass_first = pass_;
            hit->range = cur_range_;
        } else {
            const uint32_t step16 = static_cast<uint32_t>((static_cast<uint64_t>(absdiff(hit->f10, f10)) * 160u) / g.res_hz);
            if (step16 > hit->max_step16) hit->max_step16 = static_cast<uint16_t>(step16 > 65535 ? 65535 : step16);
            hit->f10 = f10;
            if (f10 < hit->f10_min) hit->f10_min = f10;
            if (f10 > hit->f10_max) hit->f10_max = f10;
        }
        hit->lo10 = lo10;
        hit->hi10 = hi10;
        hit->seen++;
        hit->pass_last = pass_;
        hit->t_last = t_s_;
        if (level > hit->level) hit->level = level;
        if (width > hit->width) hit->width = static_cast<uint8_t>(width > 255 ? 255 : width);
        if (r.always && hit->always < 255) hit->always++;
        hit->duty = r.duty;
        hit->upd = true;
    }
}

/* ---- end of a pass ----------------------------------------------------- */

void ScanEngine::emit_emitter(uint8_t ev, uint8_t type, uint8_t conf, const Emitter& e) {
    const ScanRange& g = range_[e.range];
    ScanEvent s{};
    s.ev = ev;
    s.type = type;
    s.conf = conf;
    s.uid = e.uid;
    s.range = e.range;
    s.f_khz = (e.f10 + 50) / 100;
    const uint32_t half_khz = e.width * g.res_hz / 2000u;
    s.lo_khz = s.f_khz > half_khz ? s.f_khz - half_khz : 0;
    s.hi_khz = s.f_khz + half_khz;
    s.param = (e.f10_max - e.f10_min) * 10u;
    if (e.t_last > e.t_first) {
        const int64_t d10 = static_cast<int64_t>(e.f10) - static_cast<int64_t>(e.f10_first);
        s.rate = static_cast<int32_t>(d10 * 10 * 60 / static_cast<int64_t>(e.t_last - e.t_first));
    }
    s.count = e.seen;
    s.passes = static_cast<uint16_t>(pass_ - e.pass_first + 1);
    s.level_cdb = e.level;
    s.duty = e.duty;
    s.width_pts = e.width;
    s.res_hz = g.res_hz;
    emit(s);
}

void ScanEngine::close_pass() {
    for (auto& e : em_) {
        if (!e.uid) continue;
        const ScanRange& g = range_[e.range];
        const uint32_t span = pass_ - e.pass_first + 1;
        const uint32_t pres = e.seen * 100u / span;
        if (e.upd) {
            if (e.type == ST_NONE && e.seen >= kDriftMin && pres >= kSteadyPct) {
                const uint64_t range16 = static_cast<uint64_t>(e.f10_max - e.f10_min) * 160u / g.res_hz;
                const uint64_t net16 = static_cast<uint64_t>(absdiff(e.f10, e.f10_first)) * 160u / g.res_hz;
                /* It must leave its own footprint (a wide hump's centre
                 * jitters by a point or two), mostly in one direction, in
                 * small steps. */
                const uint64_t need16 = (e.width > kDriftPts ? e.width : kDriftPts) * 16u;
                if (range16 >= need16 && net16 >= kDriftNetPts16 && net16 * 4 >= range16 * 3 && e.max_step16 <= kDriftStepPts16) {
                    e.type = ST_DRIFT;
                    emit_emitter(SE_NEW, ST_DRIFT, SC_MED, e);
                }
            } else if (scan_reported(e.type)) {
                emit_emitter(SE_UPD, e.type, e.type == ST_HUM ? SC_HIGH : SC_MED, e);
            }
            if (!e.surveyed && e.seen >= kSteadyMin && pres >= kSteadyPct) {
                e.surveyed = ST_STEADY;
                emit_emitter(SE_SURVEY, ST_STEADY, SC_LOW, e);
            }
        } else if (pass_ - e.pass_last >= kEmitDropMiss) {
            if (scan_reported(e.type)) emit_emitter(SE_DROP, e.type, SC_MED, e);
            e = Emitter{};
            continue;
        }
        if (!e.surveyed && span >= kBurstMinPasses && pres < kBurstPct && e.seen >= 2) {
            e.surveyed = ST_BURST;
            emit_emitter(SE_SURVEY, ST_BURST, SC_LOW, e);
        }
        e.upd = false;
    }

    for (auto& c : comb_) {
        if (!c.uid) continue;
        const ScanRange& g = range_[c.range];
        if (!c.reported) c.clock = comb_clock(c);
        ScanEvent s{};
        s.uid = c.uid;
        s.range = c.range;
        s.type = c.clock ? ST_CLOCK : (c.pulsed ? ST_PCOMB : ST_COMB);
        s.conf = c.clock ? SC_LOW : SC_MED;
        s.f_khz = (c.lo_khz + c.hi_khz) / 2;
        s.lo_khz = c.lo_khz;
        s.hi_khz = c.hi_khz;
        s.param = c.sp_hz;
        s.count = c.teeth;
        s.passes = static_cast<uint16_t>(pass_ - c.pass_first + 1);
        s.level_cdb = static_cast<int16_t>(c.level * 50);
        s.duty = static_cast<uint8_t>(c.seen * 100u / s.passes);
        s.res_hz = g.res_hz;
        if (c.upd) {
            if (!c.reported && c.seen >= 2) {
                c.reported = true;
                s.ev = c.clock ? SE_SURVEY : SE_NEW;
                emit(s);
            } else if (c.reported && !c.clock) {
                s.ev = SE_UPD;
                emit(s);
            }
            c.upd = false;
        } else if (pass_ - c.pass_last >= (c.pulsed ? kPulsedDropMiss : kCombDropMiss)) {
            if (c.reported && !c.clock) {
                s.ev = SE_DROP;
                emit(s);
            }
            c = Comb{};
        }
    }

    floors_close();
    pass_++;
    if (base_passes_ < 255) base_passes_++;
}

void ScanEngine::floors_close() {
    const bool judge = base_passes_ >= kRiseLearn;
    for (size_t w = 0; w < nwin_; w++) {
        if (!floor_[w]) {
            rise_run_[w] = 0;
            continue;
        }
        /* A floor under kFloorMinU8 is the ADC's last bit, not the
         * receiver's noise (gains far down): not judged. */
        if (judge && base_[w] >= kFloorMinU8 && block_pct_[w] < kRiseBlockPct && floor_[w] >= base_[w] + kRiseCdbX2) {
            if (rise_run_[w] < 255) rise_run_[w]++;
        } else {
            rise_run_[w] = 0;
        }
        /* Baseline: the second lowest floor seen (one unlucky low estimate
         * does not set it). */
        if (!low_[w] || floor_[w] < low_[w]) {
            base_[w] = low_[w];
            low_[w] = floor_[w];
        } else if (!base_[w] || floor_[w] < base_[w]) {
            base_[w] = floor_[w];
        }
    }

    bool matched[kMaxFloors] = {};
    for (size_t r = 0; r < nrange_; r++) {
        for (size_t w = first_[r]; w < first_[r + 1];) {
            if (rise_run_[w] < kRisePasses) {
                w++;
                continue;
            }
            size_t e = w;
            uint32_t rise = 0;
            while (e < first_[r + 1] && rise_run_[e] >= kRisePasses) {
                const uint32_t d = floor_[e] - base_[e];
                if (d > rise) rise = d;
                e++;
            }
            const uint16_t lo = static_cast<uint16_t>(w), hi = static_cast<uint16_t>(e - 1);
            w = e;
            /* One window alone needs a pass more. */
            if (lo == hi && rise_run_[lo] < kRisePassesOne) continue;
            Floor* f = nullptr;
            size_t fi = 0;
            for (size_t i = 0; i < kMaxFloors; i++)
                if (fl_[i].uid && fl_[i].w_lo <= hi && fl_[i].w_hi >= lo) {
                    f = &fl_[i];
                    fi = i;
                    break;
                }
            if (!f)
                for (size_t i = 0; i < kMaxFloors; i++)
                    if (!fl_[i].uid) {
                        f = &fl_[i];
                        fi = i;
                        *f = Floor{};
                        f->uid = new_uid();
                        break;
                    }
            if (!f || !f->uid) continue;
            matched[fi] = true;
            f->w_lo = lo;
            f->w_hi = hi;
            f->rise = static_cast<uint16_t>(rise * 50);
            f->miss = 0;
            ScanWindowPlan a, b;
            plan(lo, a);
            plan(hi, b);
            ScanEvent s{};
            s.ev = f->reported ? SE_UPD : SE_NEW;
            s.type = ST_FLOOR;
            s.conf = hi > lo ? SC_MED : SC_LOW;
            s.uid = f->uid;
            s.range = static_cast<uint8_t>(r);
            s.lo_khz = static_cast<uint32_t>(a.start_hz / 1000);
            s.hi_khz = static_cast<uint32_t>((b.start_hz + b.span_hz) / 1000);
            s.f_khz = (s.lo_khz + s.hi_khz) / 2;
            s.param = f->rise;
            s.level_cdb = static_cast<int16_t>(f->rise);
            s.count = static_cast<uint16_t>(hi - lo + 1);
            s.res_hz = range_[r].res_hz;
            f->reported = true;
            emit(s);
        }
    }
    for (size_t i = 0; i < kMaxFloors; i++) {
        Floor& f = fl_[i];
        if (!f.uid || matched[i]) continue;
        if (++f.miss < 2) continue;
        if (f.reported) {
            ScanWindowPlan a, b;
            plan(f.w_lo, a);
            plan(f.w_hi, b);
            ScanEvent s{};
            s.ev = SE_DROP;
            s.type = ST_FLOOR;
            s.conf = f.w_hi > f.w_lo ? SC_MED : SC_LOW;
            s.uid = f.uid;
            s.range = a.range;
            s.lo_khz = static_cast<uint32_t>(a.start_hz / 1000);
            s.hi_khz = static_cast<uint32_t>((b.start_hz + b.span_hz) / 1000);
            s.f_khz = (s.lo_khz + s.hi_khz) / 2;
            s.param = f.rise;
            s.level_cdb = static_cast<int16_t>(f.rise);
            s.count = static_cast<uint16_t>(f.w_hi - f.w_lo + 1);
            emit(s);
        }
        f = Floor{};
    }
}

/* ---- hum check --------------------------------------------------------- */

bool ScanEngine::hum_candidate(uint8_t& uid, uint32_t& f_khz, uint32_t& width_hz) const {
    const Emitter* best = nullptr;
    for (const auto& e : em_) {
        if (!e.uid || e.type == ST_HUM) continue;
        if (e.surveyed != ST_STEADY && e.type != ST_DRIFT) continue;
        if (e.level < kHumMinCdb) continue;
        if (e.hum_pass && pass_ - e.hum_pass < kHumRecheckPasses) continue;
        if (!best || e.level > best->level) best = &e;
    }
    if (!best) return false;
    uid = best->uid;
    f_khz = (best->f10 + 50) / 100;
    width_hz = best->width * range_[best->range].res_hz;
    return true;
}

void ScanEngine::hum_result(uint8_t uid, const sigdet::HumVerdict& v) {
    for (auto& e : em_) {
        if (e.uid != uid || !uid) continue;
        e.hum_pass = pass_;
        if (v.family && e.type != ST_HUM) {
            e.type = ST_HUM;
            const ScanRange& g = range_[e.range];
            ScanEvent s{};
            s.ev = SE_NEW;
            s.type = ST_HUM;
            s.conf = SC_HIGH;
            s.uid = e.uid;
            s.range = e.range;
            s.f_khz = (e.f10 + 50) / 100;
            const uint32_t half_khz = e.width * g.res_hz / 2000u;
            s.lo_khz = s.f_khz > half_khz ? s.f_khz - half_khz : 0;
            s.hi_khz = s.f_khz + half_khz;
            s.param = v.family;
            s.count = v.count;
            s.passes = static_cast<uint16_t>(pass_ - e.pass_first + 1);
            s.level_cdb = e.level;
            s.duty = e.duty;
            s.width_pts = e.width;
            s.res_hz = g.res_hz;
            emit(s);
        }
        return;
    }
}

/* ---- debug dump -------------------------------------------------------- */

/* SCAN lines (decimal):
 *   SCAN,W,pass,window,range,start_khz,points,looks,floor_halfdb,runs,comb_teeth,comb_sp_hz
 *   SCAN,R,i,lo,hi,peak,f_khz,ex_mean_halfdb,ex_max_halfdb,duty,always,type,ripple,fill,comb
 *   SCAN,E,uid,range,f_khz,fmin_khz,fmax_khz,seen,pass_first,pass_last,level_cdb,width,always,type,surveyed
 *   SCAN,C,uid,range,sp_hz,lo_khz,hi_khz,seen,teeth,level_halfdb,pulsed,reported,clock */
void ScanEngine::dump(Sink sink, void* ctx) const {
    char line[160];
    {
        Fmt f{line, sizeof(line)};
        f.s("SCAN,W,").d(pass_).c(',').d(cur_w_).c(',').d(cur_range_).c(',').d(static_cast<int32_t>(start_hz_ / 1000)).c(',');
        f.d(points_).c(',').d(looks_).c(',').d(wfloor_).c(',').d(nruns_).c(',').d(fit_.ok ? fit_.teeth : 0).c(',');
        f.d(fit_.ok ? static_cast<int32_t>((static_cast<uint64_t>(fit_.sp16) * range_[cur_range_].res_hz + 8) / 16) : 0);
        sink(ctx, line);
    }
    for (size_t i = 0; i < nruns_; i++) {
        const Run& r = runs_[i];
        Fmt f{line, sizeof(line)};
        f.s("SCAN,R,").d(i).c(',').d(r.lo).c(',').d(r.hi).c(',').d(r.peak).c(',').d(khz_of(r.c16)).c(',').d(r.ex_mean).c(',');
        f.d(r.ex_max).c(',').d(r.duty).c(',').d(r.always).c(',').s(scan_type_tag(r.type)).c(',').d(r.ripple).c(',').d(r.fill).c(',').d(r.comb);
        sink(ctx, line);
    }
    for (const auto& e : em_) {
        if (!e.uid) continue;
        Fmt f{line, sizeof(line)};
        f.s("SCAN,E,").d(e.uid).c(',').d(e.range).c(',').d((e.f10 + 50) / 100).c(',').d((e.f10_min + 50) / 100).c(',');
        f.d((e.f10_max + 50) / 100).c(',').d(e.seen).c(',').d(e.pass_first).c(',').d(e.pass_last).c(',').d(e.level).c(',');
        f.d(e.width).c(',').d(e.always).c(',').s(scan_type_tag(e.type)).c(',').s(scan_type_tag(e.surveyed));
        sink(ctx, line);
    }
    for (const auto& c : comb_) {
        if (!c.uid) continue;
        Fmt f{line, sizeof(line)};
        f.s("SCAN,C,").d(c.uid).c(',').d(c.range).c(',').d(c.sp_hz).c(',').d(c.lo_khz).c(',').d(c.hi_khz).c(',').d(c.seen).c(',');
        f.d(c.teeth).c(',').d(c.level).c(',').d(c.pulsed).c(',').d(c.reported).c(',').d(c.clock);
        sink(ctx, line);
    }
}

} /* namespace ui::external_app::signature_detect */
