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

#include "sigdet_capture.hpp"
#include "sigdet_fmt.hpp"

namespace ui::external_app::signature_detect {

namespace {

/* The stock spectrum bandwidth options (freqman_db.cpp SPEC), up to 4 MHz. */
constexpr uint32_t kRates[] = {12500, 16000, 25000, 32000, 50000, 75000, 100000, 150000, 250000, 500000, 600000, 750000,
                               1000000, 1250000, 1500000, 1750000, 2000000, 2250000, 2500000, 3000000, 3500000, 4000000};

}  // namespace

uint32_t cap_rate_for(uint32_t bw_khz) {
    uint64_t want = static_cast<uint64_t>(bw_khz) * 2500u;
    if (want < 500000) want = 500000;
    for (const uint32_t r : kRates)
        if (r >= want) return r;
    return kRates[sizeof(kRates) / sizeof(kRates[0]) - 1];
}

bool cap_is_c8(uint32_t rate) {
    return rate >= 1500000;
}

uint32_t cap_bytes_per_s(uint32_t rate) {
    return rate * (cap_is_c8(rate) ? 2u : 4u);
}

void cap_sanitize(const char* in, char* out, size_t cap) {
    size_t n = 0;
    for (; in && *in && n + 1 < cap; in++) {
        const char c = *in;
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
        out[n++] = ok ? c : '_';
    }
    if (cap) out[n] = 0;
}

uint8_t CapPairs::next(uint8_t state) {
    const uint8_t bit = state ? 2 : 1;
    if (pair_ == 0 || (have_ & bit)) {
        if (pair_ < 99) pair_++;
        have_ = 0;
    }
    have_ |= bit;
    return pair_;
}

void cap_base_name(uint8_t pair, uint8_t state, uint8_t dup, char* out, size_t cap) {
    Fmt f{out, cap};
    f.c('P').d2(pair).s(state ? "_ON" : "_OFF");
    if (dup) f.c('_').d(dup + 1);
}

size_t cap_sidecar(char* out, size_t cap, const CapSidecar& s) {
    Fmt f{out, cap};
    auto eol = [&]() { f.s("\r\n"); };
    auto u64 = [&](uint64_t v) {
        char t[21];
        size_t n = 0;
        do {
            t[n++] = static_cast<char>('0' + v % 10);
            v /= 10;
        } while (v);
        while (n) f.c(t[--n]);
    };
    const CaptureContext* c = s.ctx;
    f.s("format=").d(static_cast<int32_t>(kCapSigFormat));
    eol();
    f.s("file=").s(s.file, Fmt::Esc::Kv);
    eol();
    f.s("metadata=").s(s.metadata, Fmt::Esc::Kv);
    eol();
    f.s("center_hz=");
    u64(s.center_hz);
    eol();
    f.s("sample_rate=").d(static_cast<int32_t>(s.sample_rate));
    eol();
    f.s("sample_format=").s(s.c8 ? "cs8" : "cs16");
    eol();
    f.s("bb_filter_hz=").d(static_cast<int32_t>(s.bb_filter_hz));
    eol();
    f.s("lna=").d(s.lna).s("\r\nvga=").d(s.vga).s("\r\namp=").d(s.amp);
    eol();
    f.s("time=").d(s.year).c('-').d2(s.month).c('-').d2(s.day).c('T').d2(s.hour).c(':').d2(s.minute).c(':').d2(s.second);
    eol();
    f.s("uptime_ms=");
    u64(s.uptime_ms);
    eol();
    f.s("duration_ms=");
    u64(s.duration_ms);
    eol();
    f.s("bytes=");
    u64(s.bytes);
    eol();
    f.s("dropped_pct=").d(static_cast<int32_t>(s.dropped_pct));
    eol();
    f.s("state=").s(s.state ? "on" : "off");
    eol();
    f.s("pair=P").d2(s.pair);
    eol();
    f.s("label=").s(s.label, Fmt::Esc::Kv);
    eol();
    f.s("target_kind=").s(c ? c->target_kind : "manual", Fmt::Esc::Kv);
    eol();
    f.s("target_label=").s(c ? c->target_label : "", Fmt::Esc::Kv);
    eol();
    /* already key=value;... */
    f.s("target_extra=");
    if (c)
        for (const char* p = c->target_extra; *p; p++) f.c((*p == ',' || *p == '\r' || *p == '\n') ? '_' : *p);
    if (s.target_hz) {
        if (c && c->target_extra[0]) f.c(';');
        f.s("target_hz=");
        u64(s.target_hz);
    }
    eol();
    f.s("survey_log=").s(c ? c->log_name : "", Fmt::Esc::Kv);
    eol();
    f.s("survey_uptime_ms=");
    u64(c ? c->row_uptime_ms : 0);
    eol();
    f.s("firmware=").s(s.firmware, Fmt::Esc::Kv);
    eol();
    f.s("build=").s(s.build, Fmt::Esc::Kv);
    eol();
    f.s("antenna=").s(s.antenna, Fmt::Esc::Kv);
    eol();
    return f.len();
}

} /* namespace ui::external_app::signature_detect */
