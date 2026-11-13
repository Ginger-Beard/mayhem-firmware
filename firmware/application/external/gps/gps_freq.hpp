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

/* The downlink table /GPS/SATFREQ.TXT: one downlink per line, matched to a
 * TLE by NORAD catalogue number (or by name when the number is "-").
 *
 *   # NORAD  NAME     DOWNLINK  MODE  BW_KHZ  NOTE (rest of the line)
 *   25544    ISS      145.800   NFM   12.5    voice / SSTV, when active
 *
 * DOWNLINK in MHz with a decimal point ("437.800") or in Hz ("437800000").
 * MODE: NFM, AM, WFM, USB, LSB, CW (a CW beacon, heard as a 700 Hz tone in
 * USB) or DATA (digital: followed and metered, not decoded). BW_KHZ: the
 * signal's width (chooses the receive filter) or "-" for the mode's default.
 * '#' starts a comment line. No heap, no file access (host tested). */

#ifndef __GPS_FREQ_H__
#define __GPS_FREQ_H__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::gps {

enum FreqMode : uint8_t { FMODE_NFM,
                          FMODE_AM,
                          FMODE_WFM,
                          FMODE_USB,
                          FMODE_LSB,
                          FMODE_CW,
                          FMODE_DATA };

struct SatFreq {
    uint32_t catnr{0}; /* 0: match by name */
    uint64_t hz{0};
    uint32_t bw_hz{0}; /* 0: the mode's default */
    uint8_t mode{FMODE_NFM};
    char name[13]{};
    char note[25]{};
};

enum class FreqLine : uint8_t { OK,
                                SKIP, /* blank or comment */
                                BAD };

/* One line of the table. On BAD, *why (if given) names the field. */
FreqLine parse_satfreq(const char* line, SatFreq& out, const char** why = nullptr);

/* Does a table entry belong to this TLE (catalogue number, or the entry's
 * name as the start of the TLE name, ignoring case, when it has no number)? */
bool satfreq_matches(const SatFreq& f, uint32_t catnr, const char* tle_name);

const char* freq_mode_name(uint8_t mode); /* "NFM" ... */
/* "HF" < 30 MHz, "VHF" < 300 MHz, "UHF" < 1 GHz, "L" < 2 GHz, "S" < 4 GHz, else "C" */
const char* band_name(uint64_t hz);
/* "437.800" (kHz resolution; Hz shown only when not a whole kHz: "137.9125") */
void put_mhz(char* buf, size_t size, uint64_t hz);

}  // namespace ui::external_app::gps

#endif /*__GPS_FREQ_H__*/
