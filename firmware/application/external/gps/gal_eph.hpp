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

#ifndef __GAL_EPH_H__
#define __GAL_EPH_H__

/* Galileo ephemeris, clock and GST-GPS offset from the I/NAV words the app's
 * decoder (gal_inav.hpp) delivers (header only, no ChibiOS: host tested).
 * Written from the Galileo OS SIS ICD issue 2.1: word types 1-4 (tables
 * 40-43: ephemeris, SISA, clock), 5 (table 44: BGD, signal health, data
 * validity), 10 (table 49: GGTO); scale factors tables 65, 68, 70, 74.
 *
 * Word types 1-4 are kept raw per channel with their IODnav; an ephemeris is
 * built only when all four carry the same IODnav, word type 4's SVID is the
 * tracked PRN and word type 5 (health, BGD) was seen in the same lock. It
 * goes into a small table by PRN as an Orbit with ephemeris = 2 (sat_position
 * then uses Galileo's GM), BGD(E1,E5b) as the single-frequency E1 group delay
 * (ICD eq. 17), times in GPS-week seconds (GST week + 1024 = GPS week, same
 * time of week). Everything lives in the RAM the tracking image lends. */

#include <cmath>
#include <cstdint>
#include <cstring>

#include "gal_e1.hpp"
#include "gal_inav.hpp"
#include "gps_orbit.hpp"
#include "gps_pvt.hpp"
#include "gps_trk_shared.hpp"

namespace gal_eph {

using ui::external_app::gps::kPi;
using ui::external_app::gps::kWeekS;
using ui::external_app::gps::Orbit;

constexpr int kCh = gal_inav::Work::kCh;
constexpr int kSlots = 6;                  /* Galileo ephemerides kept (by PRN) */
constexpr double kMaxAgeS = 4.0 * 3600.0;  /* ephemeris use either side of toe (ICD: 4 h fit) */
constexpr double kGgtoMaxAgeS = 86400 * 3; /* a GGTO older than this is not applied */

/* word bits: bit i of the 128-bit word (MSB first, word type in bits 0-5) */
inline uint32_t ubits(const uint8_t* w, int at, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | ((w[(at + i) >> 3] >> (7 - ((at + i) & 7))) & 1u);
    return v;
}
inline int32_t sbits(const uint8_t* w, int at, int n) {
    const uint32_t u = ubits(w, at, n);
    return n < 32 && (u >> (n - 1)) ? static_cast<int32_t>(u | (~0u << n)) : static_cast<int32_t>(u);
}
inline uint16_t iod_of(const uint8_t* w) {
    return static_cast<uint16_t>(ubits(w, 6, 10));
}

constexpr double p2(int e) { /* 2^e, exact */
    return e >= 0 ? static_cast<double>(1ull << e) : 1.0 / static_cast<double>(1ull << -e);
}

/* word types 1-4 and 5 of one channel's satellite */
struct Collect {
    uint8_t prn;
    uint8_t have;    /* bit k: word type k + 1 stored */
    uint8_t have5;   /* word type 5 seen in this lock */
    uint8_t hs, dvs; /* E1-B signal health (2 bits), data validity (1 bit) from word type 5 */
    uint8_t pad;
    int16_t bgd_b;  /* BGD(E1,E5b), 2^-32 s */
    uint16_t epoch; /* the channel's start count */
    uint16_t pad2;
    uint8_t w[4][16]; /* raw words 1-4 */
};

struct Eph {
    Orbit o; /* ephemeris = 2: Galileo */
    uint16_t iod;
    uint8_t prn; /* 0: free */
    uint8_t sisa;
    uint8_t hs, dvs;
    uint16_t confirms; /* the same IODnav assembled again since */
};

/* GST - GPS (ICD eq. 23): dt = A0G + A1G (TOW - t0G + 604800 (WN - WN0G)) */
struct Ggto {
    int16_t a0;  /* 2^-35 s */
    int16_t a1;  /* 2^-51 s/s (12 bits) */
    uint8_t t0;  /* 3600 s */
    uint8_t wn0; /* 6-bit week */
    uint8_t valid;
    uint8_t prn;
    double t_got; /* GPS s when decoded */
};

struct Store {
    Collect col[kCh];
    Eph eph[kSlots];
    Ggto ggto;
    uint16_t built, rejected, iod_waits; /* counters for the log */
};

enum Event : uint8_t {
    EV_NONE = 0,
    EV_WORD,      /* a word type 1-5 stored */
    EV_NEW,       /* a new ephemeris (or a new IODnav) built */
    EV_SAME,      /* the same IODnav assembled again */
    EV_IOD_WAIT,  /* words 1-4 complete but with different IODnav: waiting for the rest of the new batch */
    EV_BAD_SVID,  /* word type 4's SVID is not the tracked PRN: dropped */
    EV_BAD_VALUE, /* orbit values implausible: dropped */
    EV_GGTO,      /* GST-GPS offset taken from word type 10 */
    EV_GGTO_NONE, /* word type 10 says the GGTO is not available */
};

/* Orbit from words 1-4 (same IODnav) and word type 5's BGD / health.
 * now_gps: continuous GPS s (for the week of toe / toc). False if the
 * values are not a plausible Galileo MEO orbit. */
inline bool build(const uint8_t w1[16], const uint8_t w2[16], const uint8_t w3[16], const uint8_t w4[16], int16_t bgd_b, uint8_t hs, uint8_t dvs, double now_gps, Orbit& o) {
    o = Orbit{};
    const double toe_sow = ubits(w1, 16, 14) * 60.0;
    o.m0 = sbits(w1, 30, 32) * p2(-31) * kPi;
    o.e = ubits(w1, 62, 32) * p2(-33);
    o.sqrt_a = ubits(w1, 94, 32) * p2(-19);
    o.omega0 = sbits(w2, 16, 32) * p2(-31) * kPi;
    o.i0 = sbits(w2, 48, 32) * p2(-31) * kPi;
    o.omega = sbits(w2, 80, 32) * p2(-31) * kPi;
    o.idot = static_cast<float>(sbits(w2, 112, 14) * p2(-43) * kPi);
    o.omega_dot = static_cast<float>(sbits(w3, 16, 24) * p2(-43) * kPi);
    o.dn = static_cast<float>(sbits(w3, 40, 16) * p2(-43) * kPi);
    o.cuc = static_cast<float>(sbits(w3, 56, 16) * p2(-29));
    o.cus = static_cast<float>(sbits(w3, 72, 16) * p2(-29));
    o.crc = static_cast<float>(sbits(w3, 88, 16) * p2(-5));
    o.crs = static_cast<float>(sbits(w3, 104, 16) * p2(-5));
    o.cic = static_cast<float>(sbits(w4, 22, 16) * p2(-29));
    o.cis = static_cast<float>(sbits(w4, 38, 16) * p2(-29));
    const double toc_sow = ubits(w4, 54, 14) * 60.0;
    o.af0 = static_cast<float>(sbits(w4, 68, 31) * p2(-34));
    o.af1 = static_cast<float>(sbits(w4, 99, 21) * p2(-46));
    o.af2 = static_cast<float>(sbits(w4, 120, 6) * p2(-59));
    o.tgd = static_cast<float>(bgd_b * p2(-32)); /* single-frequency E1: dt_SV - BGD(E1,E5b) */
    /* the week of toe / toc: the one that puts them nearest now */
    const double wk = static_cast<double>(static_cast<int64_t>(now_gps / kWeekS));
    auto nearest = [&](double sow) {
        double t = wk * kWeekS + sow;
        if (t - now_gps > kWeekS / 2) t -= kWeekS;
        if (now_gps - t > kWeekS / 2) t += kWeekS;
        return t;
    };
    o.toe = nearest(toe_sow);
    o.toc = nearest(toc_sow);
    o.prn = static_cast<uint8_t>(ubits(w4, 16, 6));
    o.health = (hs != 0 || dvs != 0) ? 1 : 0;
    o.ephemeris = 2;
    o.valid = 1;
    return o.sqrt_a > 5000.0 && o.sqrt_a < 6000.0 && o.e < 0.25;
}

inline const Eph* find(const Store& s, int prn) {
    for (const Eph& e : s.eph)
        if (e.prn == prn) return &e;
    return nullptr;
}

/* the Orbit to use for prn at now_gps: healthy, within the fit time; else nullptr */
inline const Orbit* usable(const Store& s, int prn, double now_gps) {
    const Eph* e = find(s, prn);
    if (!e || e->o.health) return nullptr;
    const double d = now_gps - e->o.toe;
    if (d > kMaxAgeS || d < -kMaxAgeS) return nullptr;
    return &e->o;
}

/* GST - GPS in seconds at GPS time t (continuous GPS s), if a valid GGTO
 * younger than kGgtoMaxAgeS is known */
inline bool ggto_at(const Store& s, double t, double* dt) {
    const Ggto& g = s.ggto;
    if (!g.valid || t - g.t_got > kGgtoMaxAgeS || g.t_got - t > kGgtoMaxAgeS) return false;
    const int32_t wk = static_cast<int32_t>(t / kWeekS);
    const double tow = t - wk * kWeekS;
    int32_t dw = ((wk - 1024) - g.wn0) & 63; /* GST week mod 64 - WN0G, |true difference| <= 31 */
    if (dw > 31) dw -= 64;
    *dt = g.a0 * p2(-35) + g.a1 * p2(-51) * (tow - g.t0 * 3600.0 + kWeekS * dw);
    return true;
}

/* One decoded page of channel ch (gal_inav::Page, CRC good or not) of the
 * channel's start count `epoch`. Returns what happened (Event); *slot: the
 * table entry of a new / confirmed ephemeris. */
inline Event take(Store& s, int ch, uint16_t epoch, const gal_inav::Page& p, double now_gps, int* slot = nullptr) {
    if (!p.crc_ok || ch < 0 || ch >= kCh) return EV_NONE;
    Collect& c = s.col[ch];
    if (c.prn != p.prn || c.epoch != epoch) { /* another satellite or a new lock: start over */
        std::memset(static_cast<void*>(&c), 0, sizeof(c));
        c.prn = p.prn;
        c.epoch = epoch;
    }
    const uint8_t* w = p.word;
    switch (p.word_type) {
        case 1:
        case 2:
        case 3:
        case 4:
            std::memcpy(c.w[p.word_type - 1], w, 16);
            c.have |= static_cast<uint8_t>(1u << (p.word_type - 1));
            break;
        case 5:
            c.bgd_b = static_cast<int16_t>(sbits(w, 57, 10));
            c.hs = static_cast<uint8_t>(ubits(w, 69, 2));
            c.dvs = static_cast<uint8_t>(ubits(w, 72, 1));
            c.have5 = 1;
            break;
        case 10: {
            const uint32_t a0 = ubits(w, 86, 16), a1 = ubits(w, 102, 12), t0 = ubits(w, 114, 8), wn0 = ubits(w, 122, 6);
            if (a0 == 0xFFFF && a1 == 0xFFF && t0 == 0xFF && wn0 == 0x3F) return EV_GGTO_NONE;
            s.ggto.a0 = static_cast<int16_t>(sbits(w, 86, 16));
            s.ggto.a1 = static_cast<int16_t>(sbits(w, 102, 12));
            s.ggto.t0 = static_cast<uint8_t>(t0);
            s.ggto.wn0 = static_cast<uint8_t>(wn0);
            s.ggto.valid = 1;
            s.ggto.prn = p.prn;
            s.ggto.t_got = now_gps;
            return EV_GGTO;
        }
        default:
            return EV_NONE;
    }
    if (c.have != 15 || !c.have5) return EV_WORD;
    const uint16_t iod = iod_of(c.w[0]);
    if (iod_of(c.w[1]) != iod || iod_of(c.w[2]) != iod || iod_of(c.w[3]) != iod) {
        s.iod_waits++;
        return p.word_type <= 4 ? EV_IOD_WAIT : EV_WORD;
    }
    if (p.word_type == 5) /* health refreshed: the stored ephemeris of this batch takes it */
        for (Eph& e : s.eph)
            if (e.prn == c.prn && e.iod == iod && e.o.valid) {
                e.hs = c.hs;
                e.dvs = c.dvs;
                e.o.health = (c.hs || c.dvs) ? 1 : 0;
                e.o.tgd = static_cast<float>(c.bgd_b * p2(-32));
                return EV_WORD;
            }
    /* a batch complete: this PRN's entry, else a free one, else the oldest toe */
    int k = -1;
    for (int i = 0; i < kSlots; i++)
        if (s.eph[i].prn == c.prn) k = i;
    if (k >= 0 && s.eph[k].iod == iod && s.eph[k].o.valid) {
        if (s.eph[k].confirms < 65535) s.eph[k].confirms++;
        if (slot) *slot = k;
        return EV_SAME;
    }
    Orbit o;
    const bool ok = build(c.w[0], c.w[1], c.w[2], c.w[3], c.bgd_b, c.hs, c.dvs, now_gps, o);
    if (o.prn != c.prn) {
        s.rejected++;
        return EV_BAD_SVID;
    }
    if (!ok) {
        s.rejected++;
        return EV_BAD_VALUE;
    }
    if (k < 0)
        for (int i = 0; i < kSlots && k < 0; i++)
            if (!s.eph[i].prn) k = i;
    if (k < 0) {
        k = 0;
        for (int i = 1; i < kSlots; i++)
            if (s.eph[i].o.toe < s.eph[k].o.toe) k = i;
    }
    Eph& e = s.eph[k];
    e.o = o;
    e.iod = iod;
    e.prn = c.prn;
    e.sisa = static_cast<uint8_t>(ubits(c.w[2], 120, 8));
    e.hs = c.hs;
    e.dvs = c.dvs;
    e.confirms = 0;
    s.built++;
    if (slot) *slot = k;
    return EV_NEW;
}

/* why a Galileo channel is not in the PVT input (log) */
enum Why : uint8_t {
    W_USED = 0,
    W_IDLE,        /* no channel, or not tracking */
    W_NO_CS,       /* secondary code not in sync */
    W_WEAK,        /* E1C C/N0 below kMinCn0x10 */
    W_NO_TIME,     /* no page time of week in this lock yet */
    W_TIME_UNCONF, /* the page time not yet confirmed by a second time page */
    W_NO_EPH,      /* no ephemeris decoded, unhealthy, or out of its fit time */
    W_FULL,        /* the PVT input is full (GPS first) */
};
constexpr uint16_t kMinCn0x10 = 250; /* E1C 25 dB-Hz */

/* Galileo entries of the measurement set m appended to the PVT input after
 * its n0 GPS ones (w.t_ref set by them): transmit time from the decoder's page
 * time of the same lock, the decoded ephemeris; ggto_s (GST - GPS, s; 0 when
 * not known) is subtracted so the transmit times are GPS time. why[ch]: the
 * reason per channel. Returns the new count. */
inline int add_pvt(ui::external_app::gps::PvtWork& w, int n0, const gal_acq::MeasSet& m, const gal_inav::Work& nav, const Store& s, double now_gps, double ggto_s, uint8_t* why) {
    int n = n0;
    const int64_t ref_ms = static_cast<int64_t>(w.t_ref - static_cast<double>(static_cast<int64_t>(w.t_ref / kWeekS)) * kWeekS) * 1000;
    for (int i = 0; i < kCh && i < gal_acq::kMaxTrk; i++) {
        const gal_acq::MeasCh& c = m.ch[i];
        const gal_inav::ChanState& cs = nav.ch[i];
        uint8_t r = W_USED;
        uint32_t ms = 0, fr = 0;
        const Orbit* o = nullptr;
        if (!c.prn || c.state != gps_trk::TRK_TRACK)
            r = W_IDLE;
        else if (!(c.flags & gal_acq::TF_CS))
            r = W_NO_CS;
        else if (c.cn0c_x10 < kMinCn0x10)
            r = W_WEAK;
        else if (cs.prn != c.prn || cs.epoch != c.sym_epoch || !gal_inav::tx_time(cs, c.sym, c.frac, &ms, &fr))
            r = W_NO_TIME;
        else if (!cs.t_agree)
            r = W_TIME_UNCONF;
        else if (!(o = usable(s, c.prn, now_gps)))
            r = W_NO_EPH;
        else if (n >= ui::external_app::gps::kMaxPvt)
            r = W_FULL;
        why[i] = r;
        if (r != W_USED) continue;
        int64_t dms = static_cast<int64_t>(ms) - ref_ms;
        if (dms > 302400000) dms -= 604800000;
        if (dms < -302400000) dms += 604800000;
        ui::external_app::gps::PvtMeas& p = w.in[n++];
        p.orbit = o;
        p.dt_tx = (static_cast<double>(dms) + fr / 4294967296.0) * 1e-3 - ggto_s;
        p.doppler_hz = c.doppler_hz;
        p.cn0 = c.cn0c_x10 / 10.0f;
        p.prn = c.prn;
        p.sys = 1;
    }
    return n;
}

/* GPS + Galileo against GPS alone: whether the mixed solution may replace
 * the GPS-only one (never make a GPS fix worse) */
enum Gate : uint8_t {
    G_IN = 0,    /* used */
    G_FAILED,    /* the GPS + Galileo solve failed */
    G_FEW_GAL,   /* fewer Galileo left after the checks than the mode needs (2 with the offset solved) */
    G_UNCHECKED, /* no redundancy to check it (2 spare satellites when GPS alone failed) */
    G_GAL_RESID, /* Galileo residuals over kGateGalRmsM */
    G_MOVED,     /* the position moved more than the GPS-only error allows */
    G_RMS,       /* the residuals grew */
};
constexpr double kGateGalRmsM = 15.0;
constexpr double kGateMoveMinM = 15.0;

inline double horiz_m(const ui::external_app::gps::Vec3& a, const ui::external_app::gps::Vec3& b) {
    const ui::external_app::gps::Observer ob = ui::external_app::gps::make_observer(b);
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const double e = -ob.slo * dx + ob.clo * dy, nn = -ob.sla * ob.clo * dx - ob.sla * ob.slo * dy + ob.cla * dz;
    return std::sqrt(e * e + nn * nn);
}

/* gps: the GPS-only solution (gps_ok: it was accepted by its own checks),
 * mix: GPS + Galileo (mix_ok), isb: the offset was solved, gal_rms: rms of
 * the Galileo residuals in mix */
inline Gate gate(bool gps_ok, const ui::external_app::gps::PvtSolution& gps, bool mix_ok, const ui::external_app::gps::PvtSolution& mix, bool isb, double gal_rms) {
    if (!mix_ok || !mix.ok || !std::isfinite(mix.pos.x + mix.pos.y + mix.pos.z + mix.resid_rms_m + gal_rms)) return G_FAILED;
    if (mix.used_gal < (isb ? 2 : 1)) return G_FEW_GAL;
    const int unknowns = isb ? 5 : 4;
    if (mix.used < unknowns + (gps_ok ? 1 : 2)) return G_UNCHECKED;
    if (gal_rms > kGateGalRmsM) return G_GAL_RESID;
    if (gps_ok) {
        const double lim = 3.0 * ui::external_app::gps::pvt_err_m(gps.hdop);
        if (horiz_m(mix.pos, gps.pos) > (lim > kGateMoveMinM ? lim : kGateMoveMinM)) return G_MOVED;
        if (gps.checked && mix.resid_rms_m > 1.5 * gps.resid_rms_m + 3.0) return G_RMS;
    }
    return G_IN;
}

}  // namespace gal_eph

#endif /*__GAL_EPH_H__*/
