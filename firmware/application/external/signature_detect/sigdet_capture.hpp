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

/* Capture of an unknown signal for the PC tool (sigdb): what to capture (a
 * detection, the track target, a survey unknown, a typed frequency), the
 * sample rate to suggest, the off / on pairing, the names in the session
 * folder and the .SIG sidecar (format: sdcard/SIGDET/README.TXT
 * "Captures"). The IQ and its .TXT metadata come from the stock RecordView.
 * Integer only, no UI or ChibiOS dependency (host testable). */

#ifndef __SIGDET_CAPTURE_HPP__
#define __SIGDET_CAPTURE_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::signature_detect {

constexpr uint32_t kCapSigFormat = 1;
constexpr uint32_t kCapWarnBytes = 500u * 1000u * 1000u; /* warn above this per capture */

/* Handed from Signature Detect to its capture view (by value). */
struct CaptureContext {
    uint64_t f_hz{0};
    uint32_t rate{500000};     /* suggested stock bandwidth option (= sample rate) */
    uint8_t state{0};          /* 0 off (baseline), 1 on (device active) */
    char label[16]{};          /* default label: [A-Za-z0-9_-] */
    char target_kind[12]{};    /* manual, rf, nb, ble, zwave, pattern, droneid, remote_id, energy, ... */
    char target_label[24]{};   /* what it was called */
    char target_extra[80]{};   /* key=value;... (rule, band, cid, uuid, addr, home ...) */
    char log_name[36]{};       /* session CSV, "" when the log was off */
    uint32_t row_uptime_ms{0}; /* the row / detection it came from, 0 none */
};

/* Stock spectrum bandwidth option (= capture sample rate, freqman SPEC) for
 * an emitter bw_khz wide: 2.5 x its width, at least 500 kHz, at most
 * 4000 kHz (the stock table: C16 to 1 MHz, C8 to 2.25 MHz on a fast card,
 * above 2.5 MHz the M4 may drop samples). */
uint32_t cap_rate_for(uint32_t bw_khz);
/* Bytes per second of a capture at rate: C16 (rate < 1.5 MHz, as the stock
 * app chooses) 4 bytes a sample, C8 2. */
uint32_t cap_bytes_per_s(uint32_t rate);
bool cap_is_c8(uint32_t rate);

/* Label: letters, digits, '-' and '_' kept, others '_', at most cap - 1. */
void cap_sanitize(const char* in, char* out, size_t cap);

/* Off / on pairs of a session: a capture joins the current pair unless that
 * pair already holds its state, then a new pair starts (OFF, ON -> P01 both;
 * OFF again -> P02). */
class CapPairs {
   public:
    uint8_t next(uint8_t state);
    uint8_t current() const { return pair_; }

   private:
    uint8_t pair_{0};
    uint8_t have_{0}; /* bit 0 off, bit 1 on in the current pair */
};

/* "P01_OFF" / "P01_ON" (+ "_2".. when taken). */
void cap_base_name(uint8_t pair, uint8_t state, uint8_t dup, char* out, size_t cap);

struct CapSidecar {
    const char* file;     /* data file name, e.g. P01_OFF.C16 */
    const char* metadata; /* stock metadata file name, P01_OFF.TXT */
    uint64_t center_hz;
    uint32_t sample_rate;
    bool c8;
    uint8_t lna, vga, amp;
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint32_t duration_ms;
    uint64_t bytes;
    uint8_t state;
    uint8_t pair;
    const char* label;
    const char* antenna;
    const CaptureContext* ctx;
    const char* firmware;
    const char* build;
    uint32_t uptime_ms;       /* at the start of the capture */
    uint32_t dropped_pct;     /* dropped_bytes / received_bytes, %, 1 if any was dropped, 0..99 */
    uint64_t received_bytes;  /* bytes the baseband streamed for the file (written + dropped) */
    uint64_t dropped_bytes;   /* ... dropped for want of an empty buffer (the SD writes fell behind) */
    uint32_t dropped_buffers; /* dropped_bytes in write buffers (rounded up) */
    uint32_t write_size;      /* the capture thread's write buffer, bytes */
    uint32_t wall_ms;         /* recording start to stop, wall clock (duration_ms is from the data) */
    uint32_t bb_filter_hz;    /* baseband (anti-alias) filter */
    uint64_t target_hz;       /* the target's own frequency when the capture is tuned off it, else 0 */
};

/* The .SIG sidecar text (key=value lines, CRLF); returns its length. */
size_t cap_sidecar(char* out, size_t cap, const CapSidecar& s);

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_CAPTURE_HPP__*/
