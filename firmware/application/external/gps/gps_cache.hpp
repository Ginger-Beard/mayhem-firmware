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

#ifndef __GPS_CACHE_H__
#define __GPS_CACHE_H__

/* /GPS/EPH_AIR.TXT: ephemerides the app decoded off the air, so the next
 * start within their validity (4 h) is precise at once. The parity-checked
 * subframes are stored as they were broadcast (exact; no float formatting on
 * the M0): one ephemeris = three lines
 *   S <prn> <week> <subframe 1..3> <TOW of its start, s> <10 data words, hex>
 * (under 100 characters: the orbit loader's line buffer). Read after
 * EPHEM.RNX; per satellite the record closest to now wins. */

#include <cstddef>
#include <cstdint>

#include "gps_fmt.hpp"

namespace ui::external_app::gps {

inline size_t eph_air_line(char* buf, size_t size, uint8_t prn, int32_t week, uint8_t sfid, uint32_t tow, const uint32_t w[10]) {
    Fmt f{buf, size};
    f.s("S ").u(prn).c(' ').u(static_cast<uint32_t>(week)).c(' ').u(sfid).c(' ').u(tow);
    static const char* const hex = "0123456789ABCDEF";
    for (int k = 0; k < 10; k++) {
        f.c(' ');
        for (int d = 5; d >= 0; d--) f.c(hex[(w[k] >> (4 * d)) & 15u]);
    }
    f.c('\n');
    return f.len();
}

inline bool eph_air_parse(const char* s, uint8_t* prn, int32_t* week, uint8_t* sfid, uint32_t* tow, uint32_t w[10]) {
    if (s[0] != 'S' || s[1] != ' ') return false;
    s += 2;
    uint32_t v[4];
    for (int k = 0; k < 4; k++) {
        while (*s == ' ') s++;
        if (*s < '0' || *s > '9') return false;
        uint32_t x = 0;
        while (*s >= '0' && *s <= '9') x = x * 10u + static_cast<uint32_t>(*s++ - '0');
        v[k] = x;
    }
    for (int k = 0; k < 10; k++) {
        while (*s == ' ') s++;
        uint32_t x = 0;
        int nd = 0;
        for (;; s++, nd++) {
            const char c = *s;
            uint32_t d;
            if (c >= '0' && c <= '9')
                d = static_cast<uint32_t>(c - '0');
            else if (c >= 'A' && c <= 'F')
                d = static_cast<uint32_t>(c - 'A' + 10);
            else if (c >= 'a' && c <= 'f')
                d = static_cast<uint32_t>(c - 'a' + 10);
            else
                break;
            x = (x << 4) | d;
        }
        if (nd != 6) return false;
        w[k] = x;
    }
    if (v[0] < 1 || v[0] > 32 || v[2] < 1 || v[2] > 3) return false;
    *prn = static_cast<uint8_t>(v[0]);
    *week = static_cast<int32_t>(v[1]);
    *sfid = static_cast<uint8_t>(v[2]);
    *tow = v[3];
    return true;
}

}  // namespace ui::external_app::gps

#endif /*__GPS_CACHE_H__*/
