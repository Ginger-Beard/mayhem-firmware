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

#ifndef __GPS_TRK_SHARED_H__
#define __GPS_TRK_SHARED_H__

/* What the PGPR tracking loops (M4) publish for the GPS app (M0), in M4 RAM
 * (GpsRxStatusMessage::track points here while the image runs). The M4 is
 * the only writer; the M0 copies under the sequence counters:
 *  - channel table: seqlock (odd while the M4 writes; copy, then check the
 *    counter is the same and even);
 *  - subframes: a ring of kSfSlots, sf_write counts slots written; a slot is
 *    complete when written (the counter is raised after the slot);
 *  - measurements: two sets, meas_seq counts sets; set (meas_seq - 1) & 1 is
 *    the newest, the M4 writes the other one next (one second later). */

#include <cstdint>

namespace gps_trk {

constexpr int kMaxCh = 12;
constexpr int kSfSlots = 16;

enum ChState : uint8_t {
    TRK_IDLE = 0,
    TRK_PULLIN, /* frequency pull-in after a hand-over or a re-acquisition */
    TRK_TRACK,  /* carrier and code loops running */
    TRK_SEARCH, /* lost: local search around the last code phase / Doppler */
    TRK_FAILED  /* given up (search found nothing for a while) */
};

enum ChFlags : uint8_t {
    F_PLL = 1,       /* phase lock indicator good */
    F_BIT = 2,       /* bit sync found */
    F_FRAME = 4,     /* subframe sync confirmed: transmit time (TOW) known */
    F_VERIFIED = 8,  /* PLL lock and C/N0 >= 28 dB-Hz for a second: a real signal */
    F_MEAS_GAP = 16, /* (measurement) a missed DMA buffer in the last second */
    F_EVER = 32,     /* verified (or bit-synced) at least once since the hand-over: a real satellite */
    F_RELEASED = 64, /* never verified within the verification timeout: channel given up (detection only) */
    F_AIDED = 128,   /* started by the aided search for a newly risen satellite */
};

/* why a channel last lost lock (ChInfo::why, the log's EV rows) */
enum LossWhy : uint8_t {
    LOSS_NONE = 0,
    LOSS_CN0 = 1,    /* C/N0 below 25 dB-Hz (no bit sync yet: not coasted) */
    LOSS_COAST = 2,  /* faded: coasted with the NCO held, the signal did not come back in time */
    LOSS_PULLIN = 3, /* the frequency pull-in did not lock in time */
    LOSS_NOLOCK = 4, /* tracked 6 s without phase lock */
    LOSS_START = 5,  /* started by a search (aided search, hand-over offset search) */
};

/* what a re-acquisition kept (ChInfo::keep): the bit edge and the subframe
 * phase follow from the code epochs counted across the loss */
enum KeepFlags : uint8_t {
    KEEP_BITS = 1,  /* bit edge kept */
    KEEP_FRAME = 2, /* subframe phase and time of week kept (the framer expects the next TLM) */
    KEEP_OK = 4,    /* the bit-sync histogram found the same edge */
    KEEP_BAD = 8,   /* it found another edge: the kept timing was dropped */
};

struct ChInfo {
    uint8_t prn;
    uint8_t state; /* ChState */
    uint8_t flags; /* ChFlags */
    uint8_t reacq; /* re-acquisitions so far */
    uint16_t cn0_x10;
    uint8_t mode;    /* loop mode: 0 FLL pull-in, 1 PLL wide + FLL assist, 2 PLL narrow, 3 search, 4 hand-over offset search, 5 coasting (faded: NCO held) */
    int8_t pld_x100; /* PLL lock indicator cos(2 phi), x100 (smoothed) */
    float doppler_hz;
    uint32_t lock_ms;     /* time in TRK_TRACK since the last (re)start */
    uint32_t subframes;   /* parity-good subframes */
    uint32_t parity_errs; /* words that failed parity while in a frame */
    uint32_t tow_s;       /* last decoded TOW (start of a subframe), s of week; valid with F_FRAME */
    uint32_t bits;        /* data bits decided since bit sync */
    uint32_t words_ok;    /* words that passed parity */
    uint16_t code_x64;    /* prompt code phase, chips x 64 (0..65471) */
    uint16_t sweeps;      /* search sweeps since the last loss of lock */
    float code_err;       /* last DLL discriminator, chips (code lock: near 0) */
    uint16_t half_fixes;  /* +-500 Hz false locks corrected (the carrier turning half a cycle per ms) */
    uint8_t why;          /* LossWhy of the last loss of lock */
    uint8_t keep;         /* KeepFlags: what the last re-acquisition kept of the bit / frame timing */
};

struct SubframeSlot {
    uint8_t prn;
    uint8_t sfid;
    uint8_t ch;
    uint8_t pad;
    uint32_t tow_s;     /* start of this subframe, s of week */
    uint32_t words[10]; /* 24 data bits each (d1 in bit 23), parity checked */
};

struct MeasCh {
    uint8_t prn;
    uint8_t flags; /* ChFlags (F_FRAME required for a pseudorange) */
    uint16_t cn0_x10;
    uint32_t tx_ms;   /* satellite time of the signal at the measurement sample, whole ms of week */
    uint32_t tx_frac; /* + fraction of a ms, 2^-32 ms */
    float doppler_hz; /* carrier Doppler incl. the receiver clock */
    uint32_t lock_ms;
};

struct MeasSet {
    uint32_t seq;
    uint8_t n;
    uint8_t pad[3];
    uint64_t sample; /* sample index (2.048 Msps, since the image started) of the measurement */
    uint32_t missed; /* DMA buffers missed so far */
    MeasCh ch[kMaxCh];
};

struct Shared {
    volatile uint32_t ch_seq;
    ChInfo ch[kMaxCh];
    volatile uint32_t sf_write;
    SubframeSlot sf[kSfSlots];
    volatile uint32_t meas_seq;
    MeasSet meas[2];
    /* M4 load of the tracking (DWT cycles per 1 ms buffer) */
    uint32_t cycles_avg;
    uint32_t cycles_max;
    uint32_t buffers;
    uint32_t missed;
    uint8_t active; /* channels in use */
    /* the last hand-over (diagnostics): age of the search result when the loops
     * started, ms, and how many hand-overs so far */
    uint32_t handover_age_ms;
    uint32_t handovers;
    /* input while tracking, one buffer in 64 (diagnostics): AC rms per
     * component x10 (LSB), DC x10, samples at full scale per mille */
    uint16_t level_x10;
    int16_t dc_i_x10, dc_q_x10;
    uint16_t clip_pm;
    /* unconfirmed hand-over (Autonomous): channels released because they never
     * verified within the timeout (PRN ring, newest at (released - 1) & 7) and
     * spare detections started in the freed channels */
    uint16_t released;
    uint16_t refills;
    uint8_t rel_prn[8];
    uint16_t rel_ms[8]; /* time from the start of that channel to the release, ms */
    /* searches for newly risen satellites (app predictions while tracking) */
    uint16_t aided_starts;
    /* hand-over across an image switch: the code offset common to all channels
     * (half chips from the predicted code phase; valid when xs_found) and the
     * ms from the hand-over until a channel locked on it */
    int16_t xs_off;
    uint8_t xs_found;
    uint8_t xs_pad;
    uint32_t xs_ms;
};

}  // namespace gps_trk

#endif /*__GPS_TRK_SHARED_H__*/
