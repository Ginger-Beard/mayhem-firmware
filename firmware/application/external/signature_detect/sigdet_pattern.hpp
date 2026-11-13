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

/* Pattern scan of the Signature Detect app: a table of 1 Mb/s GFSK emitters
 * known by a fixed sync word (RC toys, simple control links), the M4's
 * pattern receiver (PSDT MODE_GFSK_PATTERN, sigdet::PatternDemod) dwelling on
 * each of their channels in turn, and the decision when hits make a
 * detection. Integer only, fixed arrays, no UI or ChibiOS dependency (host
 * testable).
 *
 * Table: one built-in entry (the RC toy FCC ID SLU23T24T23, recorded on the
 * bench), then up to kPatFileEntries from the SD card file
 * /SIGDET/PATTERNS.TXT, one entry per line, FREQMAN style:
 *
 *   name=RC toy (SLU23T24T23),class=TOY,ch=2429/2449,sync=5527151B97,err=2
 *
 * Fields (comma separated key=value, any order; '#' starts a comment line):
 *   name=    shown in lists and logs, up to 23 characters, no comma (required)
 *   class=   short class tag, up to 4 characters (default PAT)
 *   ch=      channel centres in MHz, '/' separated, up to 8, 10..7250, at
 *            most 3 decimals (required)
 *   sync=    sync word in hex as on air, first bit sent = MSB of the first
 *            digit, 2..16 digits (required)
 *   bits=    sync length in bits, 8..64 (default 4 x the hex digits): the
 *            LAST `bits` bits of the hex are used
 *   err=     bit errors allowed, 0..3; the receiver allows at most
 *            ceil(bits / 16) - 1 (40 bits: 2) and clamps it (default 0)
 *   rate=    bit rate; only 1M is supported (default 1M)
 *   inv=     1: bit 1 = negative deviation (default 0: positive)
 *   dwell=   ms per channel visit, 20..250 (default 60)
 *   payload= bits after the sync returned with each hit, 0..64 (default 0)
 * Lines are at most kPatLineMax characters; a longer, unknown or malformed
 * line is skipped and counted (the first bad line and why is kept for the
 * Debug view). A missing file is fine. Parsing stops after kPatFileEntries
 * good entries or kPatMaxLines lines. */

#ifndef __SIGDET_PATTERN_HPP__
#define __SIGDET_PATTERN_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::signature_detect {

constexpr size_t kPatFileEntries = 16;                  // entries read from PATTERNS.TXT at most
constexpr size_t kPatMaxEntries = 1 + kPatFileEntries;  // + the built-in one
constexpr size_t kPatMaxChannels = 8;                   // channels per entry
constexpr size_t kPatLineMax = 160;                     // longer lines are rejected (bounded parse)
constexpr size_t kPatMaxLines = 200;                    // lines read at most
constexpr size_t kPatNameLen = 24;                      // incl. the terminator
constexpr size_t kPatClassLen = 5;

/* ---- detection (sigdet_pattern.cpp, ui_signature_detect.cpp) ---- */
/* Default dwell per channel visit: the toy sends groups of 2-3 packets 6.4 ms
 * apart every 36.84 ms, so any 60 ms window holds at least 2 packets. */
constexpr uint16_t kPatDwellMs = 60;
constexpr uint16_t kPatDwellMinMs = 20;
constexpr uint16_t kPatDwellMaxMs = 250;
/* Hits needed within the last two passes (a pass: one visit to each of an
 * entry's channels) for a detection. The recordings: the toy gives ~67 hits/s
 * per channel (4 per 60 ms visit, 8 per pass), the 2.4 GHz recordings
 * none in 181 channel-seconds (ambient, Wi-Fi, BLE, 802.15.4); a lone noise
 * hit (~1 per 5 min of dwell at 40 bits / 2 errors, from the binomial
 * tail) can never make 3. */
constexpr uint8_t kPatConfirmHits = 3;
/* Passes in a row without a hit: the detection is dropped. */
constexpr uint8_t kPatDropPasses = 3;

struct PatternEntry {
    char name[kPatNameLen];
    char cls[kPatClassLen];
    uint8_t nch;
    uint32_t ch_khz[kPatMaxChannels];
    uint64_t sync; /* right aligned, last bit sent in bit 0 */
    uint8_t sync_bits;
    uint8_t max_errors;
    uint8_t payload_bits;
    uint8_t rate; /* sigdet::PatternRate: 0 = 1 Mb/s */
    bool invert;
    uint16_t dwell_ms;
};

/* Per entry detection state. */
struct PatternState {
    uint16_t hits_pass;   /* this pass so far */
    uint16_t hits_prev;   /* the previous pass */
    uint16_t ms_pass;     /* dwell time of this pass so far */
    int16_t rssi_max_cdb; /* strongest hit this pass */
    uint32_t best_khz;    /* channel of the strongest hit this pass */
    uint8_t misses;       /* passes in a row without a hit */
    bool detected;
};

class PatternTable {
   public:
    struct Report {
        uint8_t loaded;    /* entries from the file */
        uint8_t bad;       /* lines rejected */
        uint16_t bad_line; /* first rejected line (1 based), 0 if none */
        bool file;         /* the file was found */
        bool truncated;    /* stopped at kPatFileEntries / kPatMaxLines */
        char why[20];      /* why bad_line was rejected */
    };

    PatternTable();

    /* File contents in pieces of any size, then finish() at the end.
     * keep_builtin false: the table starts empty (a signature class file's
     * PAT lines, added with add()). */
    void begin_file(bool keep_builtin = true);
    /* One parsed entry (a signature class's PAT line); false when full. */
    bool add(const PatternEntry& e);
    /* Entry 0 is the built-in toy entry. */
    bool has_builtin() const { return builtin_; }
    void feed(const char* data, size_t n);
    void finish();
    const Report& report() const { return report_; }

    size_t size() const { return count_; }
    const PatternEntry& entry(size_t i) const { return entries_[i]; }

    /* Parses one line into e (exposed for tests); nullptr if fine, else why. */
    static const char* parse_line(const char* line, size_t len, PatternEntry& e);

   private:
    PatternEntry entries_[kPatMaxEntries]{};
    size_t count_{0};
    bool builtin_{true};
    Report report_{};
    char line_[kPatLineMax + 1]{};
    size_t line_len_{0};
    bool line_long_{false};
    uint16_t lineno_{0};

    void end_line();
    void reject(const char* why);
};

/* Tag of a dwell (SignatureDetectConfigMessage::channel, back in each hit). */
constexpr uint8_t pattern_tag(size_t entry, size_t channel) {
    return static_cast<uint8_t>((entry << 3) | (channel & 7));
}
constexpr size_t tag_entry(uint8_t tag) {
    return tag >> 3;
}
constexpr size_t tag_channel(uint8_t tag) {
    return tag & 7;
}

/* Visits every channel of every entry in turn and decides detections. */
class PatternScan {
   public:
    enum Event : uint8_t {
        EV_NONE,
        EV_DETECT,  /* first confirmed (announce) */
        EV_REFRESH, /* still there: update the entity */
        EV_DROP,    /* kPatDropPasses passes without a hit */
    };

    explicit PatternScan(const PatternTable& table)
        : table_{table} {}

    void reset();
    /* The next (entry, channel) to dwell on; false if the table is empty. */
    bool next(size_t& entry, size_t& channel);
    /* A hit of the dwell with this tag. */
    void hit(uint8_t tag, int16_t rssi_cdb);
    /* The dwell (entry, channel) that next() gave has ended after ms. At the
     * end of an entry's pass returns what to do with its entity. */
    Event end_dwell(size_t entry, size_t channel, uint16_t ms);

    const PatternState& state(size_t entry) const { return st_[entry]; }
    /* Hits per second over the last pass, x10. */
    uint16_t rate10(size_t entry) const { return rate10_[entry]; }

   private:
    const PatternTable& table_;
    PatternState st_[kPatMaxEntries]{};
    uint16_t rate10_[kPatMaxEntries]{};
    size_t cur_entry_{0};
    size_t cur_channel_{0};
    bool started_{false};
};

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_PATTERN_HPP__*/
