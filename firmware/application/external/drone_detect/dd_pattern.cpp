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

#include "dd_pattern.hpp"

#include <cstring>

namespace ui::external_app::drone_detect {

namespace {

/* The RC toy FCC ID SLU23T24T23 as recorded on the bench (2026-11-11): two
 * channels, GFSK 1 Mb/s, ~3 bytes 0x55 then a constant 148 bit body
 * 27 15 1B 97 28 D7 ...; the sync is the last preamble byte and the first
 * 4 body bytes. 40 bits / 2 errors: 67/s per channel on the toy recordings,
 * none on the ambient ones (dd_pattern_test.cpp). */
const PatternEntry kBuiltin = {
    "RC toy (SLU23T24T23)",
    "TOY",
    2,
    {2429000, 2449000},
    0x5527151B97ull,
    40,
    2,
    0,
    0,
    false,
    kPatDwellMs};

bool is_space(char c) {
    return c == ' ' || c == '\t';
}

/* Trimmed [b, e). */
void trim(const char*& b, const char*& e) {
    while (b < e && is_space(*b)) b++;
    while (e > b && is_space(e[-1])) e--;
}

bool equals(const char* b, const char* e, const char* key) {
    const size_t n = strlen(key);
    return static_cast<size_t>(e - b) == n && memcmp(b, key, n) == 0;
}

/* Decimal 0..max, whole token. */
bool parse_uint(const char* b, const char* e, uint32_t max, uint32_t& out) {
    if (b == e || e - b > 6) return false;
    uint32_t v = 0;
    for (; b < e; b++) {
        if (*b < '0' || *b > '9') return false;
        v = v * 10 + (*b - '0');
    }
    if (v > max) return false;
    out = v;
    return true;
}

/* MHz with up to 3 decimals -> kHz, 10..7250 MHz. */
bool parse_mhz(const char* b, const char* e, uint32_t& khz) {
    const char* dot = b;
    while (dot < e && *dot != '.') dot++;
    uint32_t whole = 0, frac = 0;
    if (!parse_uint(b, dot, 7250, whole)) return false;
    if (dot < e) {
        const char* f = dot + 1;
        if (f == e || e - f > 3) return false;
        uint32_t scale = 100;
        for (; f < e; f++, scale /= 10) {
            if (*f < '0' || *f > '9') return false;
            frac += (*f - '0') * scale;
        }
    }
    khz = whole * 1000 + frac;
    return khz >= 10000 && khz <= 7250000;
}

bool parse_hex(const char* b, const char* e, uint64_t& out, uint8_t& digits) {
    if (b == e || e - b > 16) return false;
    uint64_t v = 0;
    digits = static_cast<uint8_t>(e - b);
    for (; b < e; b++) {
        const char c = *b;
        uint32_t d;
        if (c >= '0' && c <= '9')
            d = c - '0';
        else if (c >= 'a' && c <= 'f')
            d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F')
            d = c - 'A' + 10;
        else
            return false;
        v = (v << 4) | d;
    }
    out = v;
    return true;
}

}  // namespace

PatternTable::PatternTable() {
    entries_[0] = kBuiltin;
    count_ = 1;
}

const char* PatternTable::parse_line(const char* line, size_t len, PatternEntry& e) {
    e = PatternEntry{};
    memcpy(e.cls, "PAT", 4);
    e.dwell_ms = kPatDwellMs;
    bool have_name = false, have_sync = false;
    uint8_t hex_digits = 0;
    uint32_t bits = 0;
    const char* p = line;
    const char* const end = line + len;
    while (p < end) {
        const char* f = p;
        while (p < end && *p != ',') p++;
        const char* fe = p;
        if (p < end) p++; /* the comma */
        trim(f, fe);
        if (f == fe) continue;
        const char* eq = f;
        while (eq < fe && *eq != '=') eq++;
        if (eq == fe) return "no '=' in a field";
        const char* kb = f;
        const char* ke = eq;
        const char* vb = eq + 1;
        const char* ve = fe;
        trim(kb, ke);
        trim(vb, ve);
        uint32_t v = 0;
        if (equals(kb, ke, "name")) {
            const size_t n = ve - vb;
            if (n == 0 || n >= kPatNameLen) return "name length";
            memcpy(e.name, vb, n);
            e.name[n] = 0;
            have_name = true;
        } else if (equals(kb, ke, "class")) {
            const size_t n = ve - vb;
            if (n == 0 || n >= kPatClassLen) return "class length";
            memcpy(e.cls, vb, n);
            e.cls[n] = 0;
        } else if (equals(kb, ke, "ch")) {
            e.nch = 0;
            const char* c = vb;
            while (c < ve) {
                const char* cb = c;
                while (c < ve && *c != '/') c++;
                const char* ce = c;
                if (c < ve) c++;
                trim(cb, ce);
                if (e.nch == kPatMaxChannels) return "over 8 channels";
                if (!parse_mhz(cb, ce, e.ch_khz[e.nch])) return "bad channel";
                e.nch++;
            }
        } else if (equals(kb, ke, "sync")) {
            if (!parse_hex(vb, ve, e.sync, hex_digits) || hex_digits < 2) return "bad sync hex";
            have_sync = true;
        } else if (equals(kb, ke, "bits")) {
            if (!parse_uint(vb, ve, 64, bits) || bits < 8) return "bits not 8..64";
        } else if (equals(kb, ke, "err")) {
            if (!parse_uint(vb, ve, 3, v)) return "err not 0..3";
            e.max_errors = static_cast<uint8_t>(v);
        } else if (equals(kb, ke, "rate")) {
            if (!equals(vb, ve, "1M")) return "rate: only 1M";
        } else if (equals(kb, ke, "inv")) {
            if (!parse_uint(vb, ve, 1, v)) return "inv not 0/1";
            e.invert = v != 0;
        } else if (equals(kb, ke, "dwell")) {
            if (!parse_uint(vb, ve, kPatDwellMaxMs, v) || v < kPatDwellMinMs) return "dwell not 20..250";
            e.dwell_ms = static_cast<uint16_t>(v);
        } else if (equals(kb, ke, "payload")) {
            if (!parse_uint(vb, ve, 64, v)) return "payload not 0..64";
            e.payload_bits = static_cast<uint8_t>(v);
        } else {
            return "unknown key";
        }
    }
    if (!have_name) return "no name";
    if (!have_sync) return "no sync";
    if (e.nch == 0) return "no ch";
    const uint32_t avail = 4u * hex_digits;
    if (bits == 0) bits = avail;
    if (bits > avail) return "bits > hex digits";
    e.sync_bits = static_cast<uint8_t>(bits);
    if (bits < 64) e.sync &= (1ull << bits) - 1;
    /* The receiver finds every match only up to ceil(bits / 16) - 1 errors. */
    const uint8_t emax = static_cast<uint8_t>((bits + 15) / 16 - 1);
    if (e.max_errors > emax) e.max_errors = emax;
    return nullptr;
}

void PatternTable::begin_file(bool keep_builtin) {
    builtin_ = keep_builtin;
    count_ = keep_builtin ? 1 : 0;
    report_ = Report{};
    report_.file = true;
    line_len_ = 0;
    line_long_ = false;
    lineno_ = 0;
}

bool PatternTable::add(const PatternEntry& e) {
    if (count_ >= kPatMaxEntries || report_.loaded >= kPatFileEntries) {
        report_.truncated = true;
        return false;
    }
    entries_[count_++] = e;
    report_.loaded++;
    return true;
}

void PatternTable::reject(const char* why) {
    if (report_.bad < 255) report_.bad++;
    if (!report_.bad_line) {
        report_.bad_line = lineno_;
        const size_t n = strlen(why);
        const size_t m = n < sizeof(report_.why) - 1 ? n : sizeof(report_.why) - 1;
        memcpy(report_.why, why, m);
        report_.why[m] = 0;
    }
}

void PatternTable::end_line() {
    lineno_++;
    const bool long_line = line_long_;
    size_t n = line_len_;
    line_len_ = 0;
    line_long_ = false;
    if (report_.truncated) return;
    if (lineno_ > kPatMaxLines || report_.loaded >= kPatFileEntries) {
        report_.truncated = true;
        return;
    }
    if (long_line) {
        reject("line too long");
        return;
    }
    if (n && line_[n - 1] == '\r') n--;
    const char* b = line_;
    const char* e = line_ + n;
    trim(b, e);
    if (b == e || *b == '#') return;
    PatternEntry pe;
    const char* why = parse_line(b, e - b, pe);
    if (why) {
        reject(why);
        return;
    }
    entries_[count_++] = pe;
    report_.loaded++;
}

void PatternTable::feed(const char* data, size_t n) {
    for (size_t i = 0; i < n; i++) {
        const char c = data[i];
        if (c == '\n') {
            end_line();
        } else if (line_len_ < kPatLineMax) {
            line_[line_len_++] = c;
        } else {
            line_long_ = true;
        }
    }
}

void PatternTable::finish() {
    if (line_len_ || line_long_) end_line();
}

/* ---------------------------------------------------------------- scan */

void PatternScan::reset() {
    for (auto& s : st_) s = PatternState{};
    for (auto& r : rate10_) r = 0;
    cur_entry_ = cur_channel_ = 0;
    started_ = false;
}

bool PatternScan::next(size_t& entry, size_t& channel) {
    const size_t n = table_.size();
    if (!n) return false;
    if (!started_) {
        started_ = true;
        cur_entry_ = 0;
        cur_channel_ = 0;
    } else if (++cur_channel_ >= table_.entry(cur_entry_ < n ? cur_entry_ : 0).nch) {
        cur_channel_ = 0;
        if (++cur_entry_ >= n) cur_entry_ = 0;
    }
    if (cur_entry_ >= n) cur_entry_ = 0;
    entry = cur_entry_;
    channel = cur_channel_;
    if (channel == 0) {
        PatternState& s = st_[entry];
        s.hits_pass = 0;
        s.ms_pass = 0;
        s.rssi_max_cdb = -32768;
        s.best_khz = 0;
    }
    return true;
}

void PatternScan::hit(uint8_t tag, int16_t rssi_cdb) {
    const size_t e = tag_entry(tag), c = tag_channel(tag);
    if (e >= table_.size() || c >= table_.entry(e).nch) return;
    PatternState& s = st_[e];
    if (s.hits_pass < 0xFFFF) s.hits_pass++;
    if (rssi_cdb > s.rssi_max_cdb) {
        s.rssi_max_cdb = rssi_cdb;
        s.best_khz = table_.entry(e).ch_khz[c];
    }
}

PatternScan::Event PatternScan::end_dwell(size_t entry, size_t channel, uint16_t ms) {
    if (entry >= table_.size()) return EV_NONE;
    PatternState& s = st_[entry];
    s.ms_pass = static_cast<uint16_t>(s.ms_pass + ms);
    if (channel + 1 < table_.entry(entry).nch) return EV_NONE;

    /* End of the entry's pass. */
    const uint32_t hits = s.hits_pass;
    rate10_[entry] = static_cast<uint16_t>(s.ms_pass ? (hits * 10000u) / s.ms_pass : 0);
    const uint32_t window = hits + s.hits_prev;
    s.hits_prev = static_cast<uint16_t>(hits);
    if (hits)
        s.misses = 0;
    else if (s.misses < 255)
        s.misses++;
    if (!s.detected) {
        if (window >= kPatConfirmHits && hits) {
            s.detected = true;
            return EV_DETECT;
        }
        return EV_NONE;
    }
    if (hits) return EV_REFRESH;
    if (s.misses >= kPatDropPasses) {
        s.detected = false;
        return EV_DROP;
    }
    return EV_NONE;
}

} /* namespace ui::external_app::drone_detect */
