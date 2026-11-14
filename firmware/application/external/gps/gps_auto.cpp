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

#include "gps_auto.hpp"
#include "gps_nav.hpp"

#include <cstring>

namespace ui::external_app::gps {

void auto_reset(AutoState& a) {
    /* everything, the ring counters too: each Get fix starts new tracking loops,
     * whose shared block starts counting from 0 again */
    std::memset(static_cast<void*>(&a), 0, sizeof(a));
}

namespace {

EphCollect* slot_for(AutoState& a, uint8_t prn) {
    EphCollect* free_slot = nullptr;
    for (auto& c : a.col) {
        if (c.prn == prn) return &c;
        if (!c.prn && !free_slot) free_slot = &c;
    }
    if (free_slot) {
        std::memset(static_cast<void*>(free_slot), 0, sizeof(*free_slot));
        free_slot->prn = prn;
        return free_slot;
    }
    /* all taken: reuse a slot whose set was already saved */
    for (auto& c : a.col)
        if (!c.fresh) {
            std::memset(static_cast<void*>(&c), 0, sizeof(c));
            c.prn = prn;
            return &c;
        }
    return nullptr;
}

/* SF4 page -> almanac SV (pages 2-5 and 7-10 carry SV 25-32) */
int sf4_page_sv(int page) {
    static constexpr int8_t k[26] = {0, 0, 25, 26, 27, 28, 0, 29, 30, 31, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    return page >= 1 && page <= 25 ? k[page] : 0;
}

}  // namespace

int auto_take_subframe(AutoState& a, const gps_trk::SubframeSlot& s, int32_t week_hint, Orbit* table) {
    a.sf_total++;
    const uint32_t* w = s.words;
    const int id = nav::sf_id(w);
    a.tow_s = nav::sf_tow_start(w);
    if (id == 1) a.week = nav::full_week(nav::sf1_week10(w), week_hint > 0 ? week_hint : a.week);
    if (id >= 1 && id <= 3) {
        EphCollect* c = slot_for(a, s.prn);
        if (!c) return 0;
        /* a new issue (IODE / IODC) restarts the set */
        const int k = id - 1;
        std::memcpy(c->sf[k], w, sizeof(c->sf[k]));
        c->have |= static_cast<uint8_t>(1u << k);
        if (id == 1) c->tow1 = a.tow_s;
        if (c->have != 7) return 0;
        const int iodc = nav::sf1_iodc(c->sf[0]), i2 = nav::sf2_iode(c->sf[1]), i3 = nav::sf3_iode(c->sf[2]);
        if (i2 != i3 || (iodc & 0xFF) != i2) { /* mixed issues: keep the newest subframe only */
            c->have = static_cast<uint8_t>(1u << k);
            return 0;
        }
        const int32_t wk = a.week > 0 ? a.week : week_hint;
        Orbit o;
        if (!nav::decode_ephemeris(c->sf[0], c->sf[1], c->sf[2], s.prn, wk, o)) return 0;
        Orbit& t = table[s.prn - 1];
        const bool newer = !t.valid || !t.ephemeris || o.toe > t.toe ||
                           (o.toe == t.toe && (o.af0 != t.af0 || o.m0 != t.m0)); /* same toe, new upload */
        if (!c->fresh && t.valid && t.ephemeris && !newer) return 0;
        c->fresh = 1;
        a.sf_eph++;
        if (!newer) return 0;
        t = o;
        return s.prn;
    }
    /* SF4 / SF5: almanac pages */
    const int page = static_cast<int>((a.tow_s / 30u) % 25u) + 1;
    int sv = 0;
    if (id == 5 && page <= 24) sv = page;
    if (id == 4) sv = sf4_page_sv(page);
    if (sv) {
        AlmRec r;
        int svid = 0;
        if (nav::decode_almanac_page(w, r, &svid) && svid == sv) {
            for (int k = 0; k < 8; k++) a.alm.words[sv - 1][k] = w[k + 2];
            a.alm.have |= 1u << (sv - 1);
        }
        return 0;
    }
    if (id == 5 && page == 25) {
        uint32_t toa, wna;
        if (nav::decode_page25_sf5(w, &toa, &wna)) {
            if (a.alm.have_p25 && (toa != a.alm.toa || wna != a.alm.wna8)) a.alm.have = 0; /* a new almanac: start again */
            a.alm.toa = toa;
            a.alm.wna8 = wna;
            a.alm.have_p25 = 1;
            /* health of SV 1-24: 6 bits each from bit 91 */
            for (int sv2 = 1; sv2 <= 24; sv2++) {
                const uint32_t h = nav::field(w, 91 + 6 * (sv2 - 1) + 6 * ((sv2 - 1) / 4), 6);
                (void)h;
                a.alm.health_known |= 1u << (sv2 - 1);
            }
        }
    }
    return 0;
}

int auto_build_pvt(AutoState& a, const Orbit* table, bool eph_only, int32_t week, int* n_eph) {
    int n = 0, ne = 0;
    /* reference epoch: the whole second of the first synced channel's transmit time */
    uint32_t ref_ms = 0;
    for (int i = 0; i < a.meas.n; i++)
        if (a.meas.ch[i].flags & gps_trk::F_FRAME) {
            ref_ms = a.meas.ch[i].tx_ms - a.meas.ch[i].tx_ms % 1000u;
            break;
        }
    a.pvt.t_ref = week * kWeekS + ref_ms / 1000u;
    for (int i = 0; i < a.meas.n && n < kMaxPvt; i++) {
        const gps_trk::MeasCh& m = a.meas.ch[i];
        if ((m.flags & (gps_trk::F_FRAME | gps_trk::F_VERIFIED)) != (gps_trk::F_FRAME | gps_trk::F_VERIFIED)) continue;
        if (m.prn < 1 || m.prn > 32) continue;
        const Orbit& o = table[m.prn - 1];
        if (!o.valid || o.health) continue;
        if (eph_only && !o.ephemeris) continue;
        PvtMeas& p = a.pvt.in[n++];
        p.orbit = &o;
        int32_t dms = static_cast<int32_t>(m.tx_ms - ref_ms);
        if (dms > 302400000) dms -= 604800000; /* week rollover between the two */
        if (dms < -302400000) dms += 604800000;
        p.dt_tx = (dms + m.tx_frac / 4294967296.0) * 1e-3;
        p.doppler_hz = m.doppler_hz;
        p.cn0 = m.cn0_x10 / 10.0f;
        p.prn = m.prn;
        ne += o.ephemeris;
    }
    if (n_eph) *n_eph = ne;
    return n;
}

double auto_t_rx_guess(const AutoState& a, int n) {
    double best = -1e9;
    for (int i = 0; i < n; i++)
        if (a.pvt.in[i].dt_tx > best) best = a.pvt.in[i].dt_tx;
    return n > 0 ? best + 0.075 : 0;
}

bool auto_almanac_complete(const AutoState& a) {
    if (!a.alm.have_p25) return false;
    /* every SV page seen at least once in a full cycle: 24 SF5 + 8 SF4 pages
     * (dummy pages of unused SVs carry SV ID 0 and are not stored: those are
     * not required) */
    return __builtin_popcount(a.alm.have) >= 24;
}

int auto_almanac_decode(const AutoState& a, int32_t week_now, AlmRec* recs) {
    int n = 0;
    int32_t wna = static_cast<int32_t>(a.alm.wna8);
    /* full almanac week: the one nearest the current week with these 8 bits */
    int32_t full = week_now - ((week_now - wna) & 0xFF);
    if (week_now - full > 128) full += 256;
    for (int sv = 1; sv <= 32; sv++) {
        recs[sv - 1] = AlmRec{};
        if (!((a.alm.have >> (sv - 1)) & 1u)) continue;
        uint32_t w[10] = {};
        for (int k = 0; k < 8; k++) w[k + 2] = a.alm.words[sv - 1][k];
        AlmRec r;
        int svid;
        if (!nav::decode_almanac_page(w, r, &svid) || svid != sv) continue;
        r.toe = full * kWeekS + a.alm.toa;
        recs[sv - 1] = r;
        n++;
    }
    return n;
}

}  // namespace ui::external_app::gps
