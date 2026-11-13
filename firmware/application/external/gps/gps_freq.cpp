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

#include "gps_freq.hpp"
#include "gps_fmt.hpp"

namespace ui::external_app::gps {

namespace {

constexpr uint64_t kMinHz = 1000000u;      /* the HackRF tunes from 1 MHz */
constexpr uint64_t kMaxHz = 7250000000ull; /* ... to 7.25 GHz */

bool is_space(char c) {
    return c == ' ' || c == '\t';
}

char upper(char c) {
    return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
}

/* the next blank-separated token: [*s, *e); false at the end of the line */
bool token(const char*& p, const char*& s, const char*& e) {
    while (*p && is_space(*p)) p++;
    if (!*p) return false;
    s = p;
    while (*p && !is_space(*p)) p++;
    e = p;
    return true;
}

bool tok_eq(const char* s, const char* e, const char* word) {
    for (; s < e; s++, word++)
        if (!*word || upper(*s) != *word) return false;
    return *word == 0;
}

/* Unsigned decimal with up to `dec` fraction digits, scaled by 10^dec:
 * "145.8" with dec 6 -> 145800000. false on anything else or overflow. */
bool parse_scaled(const char* s, const char* e, int dec, uint64_t& out) {
    uint64_t v = 0;
    int frac = -1; /* digits after the point, -1: no point yet */
    bool digits = false;
    for (; s < e; s++) {
        if (*s == '.') {
            if (frac >= 0) return false;
            frac = 0;
            continue;
        }
        if (*s < '0' || *s > '9') return false;
        if (frac >= 0 && ++frac > dec) {
            if (*s != '0') return false; /* more precision than kept */
            continue;
        }
        v = v * 10 + static_cast<uint64_t>(*s - '0');
        if (v > 100000000000000ull) return false;
        digits = true;
    }
    if (!digits) return false;
    for (int k = frac < 0 ? 0 : frac; k < dec; k++) v *= 10;
    out = v;
    return true;
}

}  // namespace

FreqLine parse_satfreq(const char* line, SatFreq& out, const char** why) {
    const char* dummy;
    const char*& w = why ? *why : dummy;
    out = SatFreq{};
    w = "";
    const char* p = line;
    while (*p && is_space(*p)) p++;
    if (!*p || *p == '#' || *p == '\r' || *p == '\n') return FreqLine::SKIP;
    const char *s, *e;
    /* NORAD number or "-" */
    token(p, s, e);
    if (e - s == 1 && *s == '-') {
        out.catnr = 0;
    } else {
        uint64_t v;
        if (e - s > 6 || !parse_scaled(s, e, 0, v) || v == 0) {
            w = "NORAD number";
            return FreqLine::BAD;
        }
        out.catnr = static_cast<uint32_t>(v);
    }
    /* name: one word */
    if (!token(p, s, e)) {
        w = "name";
        return FreqLine::BAD;
    }
    for (size_t k = 0; s + k < e && k + 1 < sizeof(out.name); k++) out.name[k] = s[k];
    /* downlink: MHz with a point, else Hz */
    if (!token(p, s, e)) {
        w = "frequency";
        return FreqLine::BAD;
    }
    bool point = false;
    for (const char* q = s; q < e; q++) point = point || *q == '.';
    uint64_t hz;
    if (!parse_scaled(s, e, point ? 6 : 0, hz) || hz < kMinHz || hz > kMaxHz) {
        w = "frequency";
        return FreqLine::BAD;
    }
    out.hz = hz;
    /* mode */
    if (!token(p, s, e)) {
        w = "mode";
        return FreqLine::BAD;
    }
    static const char* const kModes[] = {"NFM", "AM", "WFM", "USB", "LSB", "CW", "DATA"};
    bool found = false;
    for (uint8_t m = 0; m < sizeof(kModes) / sizeof(kModes[0]) && !found; m++)
        if (tok_eq(s, e, kModes[m])) {
            out.mode = m;
            found = true;
        }
    if (!found) {
        w = "mode";
        return FreqLine::BAD;
    }
    /* bandwidth in kHz, or "-" */
    if (!token(p, s, e)) {
        w = "bandwidth";
        return FreqLine::BAD;
    }
    if (!(e - s == 1 && *s == '-')) {
        uint64_t bw;
        if (!parse_scaled(s, e, 3, bw) || bw == 0 || bw > 20000000ull) { /* 0..20 MHz */
            w = "bandwidth";
            return FreqLine::BAD;
        }
        out.bw_hz = static_cast<uint32_t>(bw);
    }
    /* note: the rest, blanks trimmed */
    while (*p && is_space(*p)) p++;
    size_t n = 0;
    while (p[n] && p[n] != '\r' && p[n] != '\n' && n + 1 < sizeof(out.note)) {
        out.note[n] = p[n];
        n++;
    }
    while (n && is_space(out.note[n - 1])) n--;
    out.note[n] = 0;
    return FreqLine::OK;
}

bool satfreq_matches(const SatFreq& f, uint32_t catnr, const char* tle_name) {
    if (f.catnr) return f.catnr == catnr;
    if (!f.name[0] || !tle_name) return false;
    for (size_t k = 0; f.name[k]; k++)
        if (upper(tle_name[k]) != upper(f.name[k])) return false; /* also stops at the end of tle_name */
    return true;
}

const char* freq_mode_name(uint8_t mode) {
    static const char* const kNames[] = {"NFM", "AM", "WFM", "USB", "LSB", "CW", "DATA"};
    return mode < sizeof(kNames) / sizeof(kNames[0]) ? kNames[mode] : "?";
}

const char* band_name(uint64_t hz) {
    if (hz < 30000000u) return "HF";
    if (hz < 300000000u) return "VHF";
    if (hz < 1000000000u) return "UHF";
    if (hz < 2000000000u) return "L";
    if (hz < 4000000000ull) return "S";
    return "C";
}

void put_mhz(char* buf, size_t size, uint64_t hz) {
    Fmt f{buf, size};
    f.u(static_cast<uint32_t>(hz / 1000000u)).c('.');
    const uint32_t rest = static_cast<uint32_t>(hz % 1000000u);
    if (rest % 1000u == 0)
        f.u(rest / 1000u, 3, '0');
    else if (rest % 100u == 0)
        f.u(rest / 100u, 4, '0');
    else
        f.u(rest, 6, '0');
}

}  // namespace ui::external_app::gps
