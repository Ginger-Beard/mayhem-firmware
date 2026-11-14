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

#ifndef __GPS_AUTO_H__
#define __GPS_AUTO_H__

/* "Autonomous" method, the M0 side (no UI, host tested): takes the parity-
 * checked subframes and the once-a-second measurement sets the PGPR tracking
 * loops publish (gps_trk_shared.hpp), and
 *  - assembles subframes 1-3 per satellite into ephemerides (same issue in
 *    all three; replaces an almanac orbit or an older ephemeris in the table),
 *  - takes the full GPS week from subframe 1 and the time of week from every
 *    subframe: GPS time without the hand-set clock,
 *  - collects the almanac pages (SF4/5) and the SF5 page 25 time of
 *    applicability, opportunistically (a full set takes 12.5 minutes),
 *  - builds the PVT input list from a measurement set.
 * All state is a plain struct kept in the M4 RAM the image lends the app. */

#include <cstdint>

#include "gps_orbit.hpp"
#include "gps_pvt.hpp"
#include "gps_trk_shared.hpp"

namespace ui::external_app::gps {

constexpr int kAutoSlots = gps_trk::kMaxCh;

struct EphCollect {
    uint8_t prn;   /* 0: free */
    uint8_t have;  /* bit k: subframe k + 1 stored (same issue) */
    uint8_t fresh; /* a complete set not yet written to the SD cache */
    uint8_t pad;
    uint32_t tow1;      /* TOW of the stored subframe 1's start, s */
    uint32_t sf[3][10]; /* subframes 1-3, 24 data bits per word */
};

struct AlmPages {
    uint32_t have;         /* bit p - 1: SV p's page stored */
    uint32_t health_known; /* bit p - 1: health from a page 25 */
    uint32_t words[32][8]; /* words 3..10 of SV p's almanac page */
    uint32_t toa;          /* s of the almanac week (page 25) */
    uint32_t wna8;
    uint8_t have_p25;
    uint8_t written; /* a complete set was written to the card */
    uint8_t pad[2];
};

struct AutoState {
    EphCollect col[kAutoSlots];
    AlmPages alm;
    gps_trk::MeasSet meas; /* the latest measurement set (a copy) */
    gps_trk::ChInfo ch[gps_trk::kMaxCh];
    uint8_t ch_n;
    uint8_t have_meas;
    uint8_t pad[2];
    uint32_t sf_read;  /* subframe ring: slots read */
    uint32_t meas_seq; /* last measurement set copied */
    uint32_t sf_total, sf_eph;
    int32_t week;       /* full GPS week from subframe 1, 0 unknown */
    uint32_t tow_s;     /* latest subframe start TOW */
    uint64_t tow_at_sample; /* not used by the solver: display only */
    PvtWork pvt;
};

void auto_reset(AutoState& a);

/* One parity-checked subframe. Updates week / TOW / almanac pages; when it
 * completes a satellite's subframes 1-3 the ephemeris replaces table[prn-1]
 * unless that already holds an ephemeris with the same or a later toe.
 * Returns the PRN of a new ephemeris, else 0. */
int auto_take_subframe(AutoState& a, const gps_trk::SubframeSlot& s, int32_t week_hint, Orbit* table);

/* Copy what the M4 published (seqlock / ring rules of gps_trk_shared.hpp).
 * Returns true when a new measurement set was copied into a.meas. Subframes
 * are handed to auto_take_subframe() through `take` (prn of a new ephemeris
 * reported through `on_eph`). */
template <typename F>
bool auto_poll(AutoState& a, const gps_trk::Shared* sh, int32_t week_hint, Orbit* table, F&& on_eph) {
    if (!sh) return false;
    /* channel table */
    for (int tries = 0; tries < 3; tries++) {
        const uint32_t s1 = sh->ch_seq;
        if (s1 & 1u) continue;
        for (int i = 0; i < gps_trk::kMaxCh; i++) a.ch[i] = sh->ch[i];
        a.ch_n = sh->active;
        if (sh->ch_seq == s1) break;
    }
    /* subframes */
    const uint32_t w = sh->sf_write;
    if (w - a.sf_read > static_cast<uint32_t>(gps_trk::kSfSlots)) a.sf_read = w - gps_trk::kSfSlots; /* overrun: the oldest are gone */
    while (a.sf_read != w) {
        const gps_trk::SubframeSlot slot = sh->sf[a.sf_read % gps_trk::kSfSlots];
        a.sf_read++;
        const int prn = auto_take_subframe(a, slot, week_hint, table);
        if (prn) on_eph(prn);
    }
    /* measurements */
    const uint32_t ms = sh->meas_seq;
    if (ms == a.meas_seq || ms == 0) return false;
    a.meas = sh->meas[(ms - 1) & 1u];
    const bool ok = sh->meas_seq == ms && a.meas.seq == ms;
    a.meas_seq = ms;
    if (ok) a.have_meas = 1;
    return ok;
}

/* PVT inputs from a.meas: channels with subframe sync and a verified signal
 * whose orbit is valid; eph_only: only satellites with an ephemeris.
 * week: the GPS week of the measurement. Returns the count; *n_eph: how many
 * of them have an ephemeris. */
int auto_build_pvt(AutoState& a, const Orbit* table, bool eph_only, int32_t week, int* n_eph);

/* Receive-time guess for the PVT list built by auto_build_pvt (n entries):
 * the latest transmit time + 75 ms, relative to a.pvt.t_ref. */
double auto_t_rx_guess(const AutoState& a, int n);

/* Almanac: all pages for the healthy satellites seen in page 25, plus the
 * toa: decode them into recs (32, by PRN - 1). wn_full: the full almanac
 * week (from wna8 and the current week). */
bool auto_almanac_complete(const AutoState& a);

/* Aided search of a newly risen satellite: from its orbit (almanac or
 * ephemeris), the receiver position and the true GPS time t_rx at some sample,
 * the code phase (chips) of the signal arriving at that sample and the
 * geometric Doppler (Hz, without the receiver clock's frequency error). */
void auto_predict(const Orbit& o, const Vec3& rx, double t_rx, double* code_chips, double* doppler_geo_hz);
/* half-width of the aided code search (windows of 15 half chips either side):
 * the orbit's position error over the light time, from the almanac's age */
int auto_aided_windows(bool ephemeris, double age_days);
int auto_almanac_decode(const AutoState& a, int32_t week_now, AlmRec* recs);

}  // namespace ui::external_app::gps

#endif /*__GPS_AUTO_H__*/
