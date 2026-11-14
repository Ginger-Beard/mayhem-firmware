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

/* Galileo E1 tracking (baseband/gal_trk.hpp, tracking image): what the image
 * publishes for the app, in its own RAM (GpsRxStatusMessage::gal_track; valid
 * while the image runs). The M4 writes, the app copies under the seqlock
 * (odd while the M4 writes). */
constexpr int kMaxTrk = 4; /* Galileo channels at most */

enum TrkFlags : uint8_t {
    TF_PLL = 1,       /* phase lock indicator good */
    TF_CS = 2,        /* E1C secondary code (CS25) in sync: 4-quadrant PLL, a verified Galileo signal */
    TF_EVER = 4,      /* secondary code found at least once since the hand-over */
    TF_SIDE = 8,      /* the side-peak monitor's last window saw a very early / late tap above the prompt */
    TF_RELEASED = 16, /* given up (lost and not found again, or freed for GPS) */
    TF_REACQ = 32,    /* lost: waiting for, or in, a re-acquisition search */
};

enum TrkRelease : uint8_t {
    TR_NONE = 0,
    TR_LOST = 1, /* lost and not found again within the give-up time */
    TR_POOL = 2, /* freed for a GPS channel (the M4 time is shared) */
    TR_OFF = 3,  /* Galileo switched off by the app */
};

struct TrkInfo {
    uint8_t prn;
    uint8_t state;     /* gps_trk::ChState */
    uint8_t flags;     /* TrkFlags */
    uint8_t reacq;     /* re-acquisitions so far */
    uint16_t cn0c_x10; /* C/N0 of E1C (pilot), dB-Hz x10 */
    uint16_t cn0b_x10; /* C/N0 of E1B (data) */
    float doppler_hz;
    uint32_t lock_ms;     /* time in TRK_TRACK since the last (re)start */
    float code_err;       /* last DLL discriminator, chips */
    uint16_t code_x16;    /* prompt code phase, chips x 16 (0..65471) */
    int8_t pld_x100;      /* PLL lock indicator, x100 */
    uint8_t mode;         /* 0 FLL pull-in, 1 PLL wide, 2 PLL narrow, 3 waiting for a search */
    uint16_t side_e_x100; /* last side-peak window: |VE|^2 / |P|^2 x100 (very early tap, +0.75 chip) */
    uint16_t side_l_x100; /* |VL|^2 / |P|^2 x100 (very late tap, -0.75 chip) */
    uint16_t side_jumps;  /* code moved by 0.75 chip onto the main peak */
    uint8_t cs_phase;     /* secondary code chip of the next epoch (0..24), with TF_CS */
    uint8_t release;      /* TrkRelease, with TF_RELEASED */
    uint32_t epochs;      /* 4 ms code epochs since the start */
    uint16_t cs_errs;     /* secondary code chips that disagreed while in sync */
    uint8_t cs_syncs;     /* times the secondary code was found */
    uint8_t fl_fixes;     /* 125 Hz false locks corrected (the carrier turning half a cycle per 4 ms) */
};

struct TrkShared {
    volatile uint32_t seq;
    TrkInfo ch[kMaxTrk];
    uint8_t max_ch; /* Galileo channels allowed (the app's setting) */
    uint8_t active;
    uint16_t handovers;      /* aided-search detections handed to a channel */
    uint16_t refused;        /* detections not handed over: no room in the M4 time */
    uint16_t preempted;      /* channels freed for GPS */
    uint16_t reacq_searches; /* re-acquisition searches run */
    uint8_t last_ho_prn;     /* the last hand-over decision (diagnostics) */
    uint8_t last_ho_result;  /* TrkHandover */
    uint16_t pool_x10;       /* channel pool in use, GPS-channel equivalents x10 */
    uint16_t pool_budget_x10;
};

enum TrkHandover : uint8_t {
    HO_NONE = 0,
    HO_TRACKED = 1, /* a channel took it */
    HO_NO_ROOM = 2, /* the pool (M4 time) has no room: GPS first */
    HO_ALREADY = 3, /* that PRN already has a channel */
    HO_NOT_DETECTED = 4,
    HO_OFF = 5, /* the app asked for the search only (Heard), or no channel allowed */
};

}  // namespace gal_acq

#endif /*__GAL_E1_H__*/
