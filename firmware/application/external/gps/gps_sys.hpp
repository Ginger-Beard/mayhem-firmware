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

#ifndef __GPS_SYS_H__
#define __GPS_SYS_H__

/* Satellite systems other than GPS (no UI, host-tested): the "System" wheel's
 * choices, the letters rows carry (G E C R S), and the prediction almanac
 * /GPS/GNSSALM.TXT for Galileo, BeiDou, GLONASS and SBAS. That file holds one
 * satellite per line in the GPS almanac model (made on a PC from a public
 * broadcast navigation file, mayhem-build tools/gnss_alm.py):
 *   sys prn health sqrtA e i0 Omega0 OmegaDot omega M0 week toa af0 af1 [k]
 * It gives positions for the sky plot and the satellites up, not for a fix.
 * The file is not kept in RAM: the app reads it again every 30 s and keeps
 * only each satellite's elevation and azimuth (SysSky, ~5 bytes each). */

#include <cstdint>
#include <cstddef>

#include "gps_orbit.hpp"

namespace ui::external_app::gps {

enum GnssSys : uint8_t {
    SYS_GPS = 0,
    SYS_GAL, /* Galileo E1 (same frequency as GPS L1) */
    SYS_BDS, /* BeiDou: B1C on L1 (BDS-3 MEO / IGSO), B1I at 1561.098 MHz (all) */
    SYS_GLO, /* GLONASS L1OF: 1598-1606 MHz, its own tuning */
    SYS_SBAS,
    SYS_N
};

/* The wheel ("System" on Heard and Passes). */
enum SysView : uint8_t {
    VIEW_ALL_L1 = 0, /* GPS + Galileo + BeiDou B1C + SBAS: one tuning at 1575.42 MHz */
    VIEW_GPS,
    VIEW_GAL,
    VIEW_BDS,
    VIEW_GLO,
    VIEW_SBAS,
    VIEW_N
};

inline GnssSys view_sys(uint8_t v) {
    return v == VIEW_GPS ? SYS_GPS : (v == VIEW_GAL ? SYS_GAL : (v == VIEW_BDS ? SYS_BDS : (v == VIEW_GLO ? SYS_GLO : SYS_SBAS)));
}
inline char sys_letter(uint8_t s) {
    return s < SYS_N ? "GECRS"[s] : '?';
}
inline const char* sys_name(uint8_t s) {
    static const char* const k[] = {"GPS", "Galileo", "BeiDou", "GLONASS", "SBAS"};
    return s < SYS_N ? k[s] : "?";
}
/* On L1 (1575.42 MHz): GPS, Galileo E1, SBAS, and BeiDou B1C, which only the
 * BDS-3 medium and inclined orbit satellites send (C19-C58; not the GEOs). */
inline bool sys_on_l1(uint8_t sys, uint8_t prn) {
    if (sys == SYS_GLO) return false;
    if (sys == SYS_BDS) return prn >= 19 && prn <= 58;
    return true;
}

/* One GNSSALM.TXT line into an almanac Orbit (prn as in the system: SBAS
 * 120-158). False for comments, GPS lines (ALMANAC.TXT has GPS) and bad
 * lines. */
bool gnssalm_line(const char* s, size_t n, Orbit& o, uint8_t& sys);

/* The other systems' satellites now: elevation and azimuth, refreshed from
 * GNSSALM.TXT every kRefreshS by the app (frame handler, one line per unit). */
struct SysSat {
    uint8_t sys;
    uint8_t prn;
    uint8_t health; /* 0 usable */
    int8_t el;      /* degrees */
    uint8_t az2;    /* azimuth / 2 (degrees) */
};

struct SysSky {
    static constexpr size_t kMax = 128;
    static constexpr uint32_t kRefreshS = 30;
    SysSat sat[kMax];
    uint8_t n;         /* satellites in the file (not GPS) */
    uint8_t file;      /* 0 not read yet, 1 read, 2 missing, 3 no usable line */
    uint8_t up[SYS_N]; /* usable and above 0 deg now */
    uint8_t total[SYS_N];
    uint32_t at;        /* unix s of this sky, 0 none */
    double toa_newest;  /* GPS s: the file's newest record (its age) */
    uint16_t bad_lines; /* lines that did not parse */
    uint16_t line_len;
    char line[168]; /* the line being read (a satellite line is ~140 characters) */
};

}  // namespace ui::external_app::gps

#endif /*__GPS_SYS_H__*/
