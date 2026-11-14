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

#ifndef __GAL_E1_H__
#define __GAL_E1_H__

/* Galileo E1 aided search: what the GPS app and its tracking image share
 * (the search itself: baseband/gal_acq.hpp). The app reads a satellite's
 * codes from /GPS/GALE1.BIN into Codes (in the tracking image's RAM) before
 * CMD_GAL and reads Result back. */

#include <cstdint>

namespace gal_acq {

constexpr int kChips = 4092;
constexpr int kCodeBytes = 512; /* 4092 bits + 4 padding */
/* /GPS/GALE1.BIN: a 16-byte header ("GALE1", version, count, bytes per
 * code), then per PRN 1-50 E1-B and E1-C, 512 bytes each, MSB first */
constexpr uint32_t kFileHeader = 16;
constexpr uint32_t kFilePrnBytes = 2 * kCodeBytes;

struct Codes {
    uint8_t b[kCodeBytes]; /* E1B (data) */
    uint8_t c[kCodeBytes]; /* E1C (pilot) */
};

struct Result {
    uint8_t prn{0};
    uint8_t done{0};
    uint8_t detected{0};
    uint8_t blocks{0};
    float metric{0};     /* peak / mean over the cells */
    float side_db{0};    /* peak over the higher of its +-1.5-sample neighbours (dB): small = ambiguous */
    float cn0{0};        /* dB-Hz, E1B + E1C */
    float code_phase{0}; /* chips at the block's first sample */
    float doppler{0};    /* Hz */
    int16_t offset{0};   /* half samples from the prediction */
};

}  // namespace gal_acq

#endif /*__GAL_E1_H__*/
