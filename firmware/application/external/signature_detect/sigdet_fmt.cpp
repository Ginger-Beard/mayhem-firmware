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

#include "sigdet_fmt.hpp"
#include "odid_lite.hpp"

namespace ui::external_app::signature_detect {

Fmt::Fmt(char* buf, size_t cap)
    : start_{buf}, p_{buf}, end_{buf + cap - 1} {
    *p_ = 0;
}

Fmt& Fmt::c(char ch) {
    if (p_ < end_) *p_++ = ch;
    *p_ = 0;
    return *this;
}

Fmt& Fmt::s(const char* str, Esc esc) {
    for (; *str; str++) {
        char ch = *str;
        if (esc == Esc::Kv) {
            if (ch == ',' || ch == ';' || ch == '=' || ch == '"' || static_cast<unsigned char>(ch) < 0x20 || ch == 0x7F) ch = '_';
        } else if (esc == Esc::Csv && ch == ',')
            ch = ';';
        else if (esc != Esc::None && (ch == '"' || (esc == Esc::Json && ch == '\\')))
            ch = '\'';
        c(ch);
    }
    return *this;
}

Fmt& Fmt::s(const char* str) {
    return s(str, Esc::None);
}

Fmt& Fmt::d(int32_t v) {
    char tmp[11];
    size_t n = 0;
    uint32_t u = v < 0 ? 0u - static_cast<uint32_t>(v) : static_cast<uint32_t>(v);
    do {
        tmp[n++] = static_cast<char>('0' + u % 10);
        u /= 10;
    } while (u);
    if (v < 0) c('-');
    while (n) c(tmp[--n]);
    return *this;
}

Fmt& Fmt::d2(int32_t v) {
    if (v < 10) c('0');
    return d(v);
}

Fmt& Fmt::d1(int32_t tenths) {
    if (tenths < 0) {
        c('-');
        tenths = -tenths;
    }
    return d(tenths / 10).c('.').c(static_cast<char>('0' + tenths % 10));
}

Fmt& Fmt::d2to1(int32_t hundredths) {
    /* Round half away from zero to tenths. */
    return d1((hundredths + (hundredths < 0 ? -5 : 5)) / 10);
}

Fmt& Fmt::e7(int32_t v) {
    char tmp[16];
    odid_format_e7(v, tmp, sizeof(tmp));
    return s(tmp);
}

Fmt& Fmt::hex(const uint8_t* b, size_t n, char sep, bool lower) {
    const char* digits = lower ? "0123456789abcdef" : "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++) {
        if (i && sep) c(sep);
        c(digits[b[i] >> 4]);
        c(digits[b[i] & 0x0F]);
    }
    return *this;
}

Fmt& Fmt::dist(uint32_t m) {
    if (m < 10000) return d(m).c('m');
    if (m < 100000) return d(m / 1000).c('.').c(static_cast<char>('0' + (m / 100) % 10)).c('k');
    return d(m / 1000).c('k');
}

const char* Fmt::t(const char* tmpl, const int32_t*& v, const char* const*& str, Esc esc) {
    for (char ch; (ch = *tmpl) && ch != '\n'; tmpl++) {
        if (ch == '$') {
            s(*str++, esc);
            continue;
        }
        if (ch != '#' && ch != '~' && ch != '^' && ch != '@') {
            c(ch);
            continue;
        }
        const int32_t x = *v++;
        if (x == kNone) continue;
        if (x == kDash) {
            c('-');
            continue;
        }
        if (ch == '#')
            d(x);
        else if (ch == '~')
            d1(x);
        else if (ch == '^')
            d2to1(x);
        else
            e7(x);
    }
    return *tmpl ? tmpl + 1 : tmpl;
}

Fmt& Fmt::pad(size_t col) {
    while (len() < col && p_ < end_) c(' ');
    return *this;
}

} /* namespace ui::external_app::signature_detect */
