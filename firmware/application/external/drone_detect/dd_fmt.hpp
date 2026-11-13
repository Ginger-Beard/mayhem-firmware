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

#ifndef __DD_FMT_HPP__
#define __DD_FMT_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::drone_detect {

/* Bounded text builder for the screen, the CSV log and the USB lines: integer
 * only, no printf, no heap. Output is always NUL terminated and silently cut
 * at the buffer's end. */
class Fmt {
   public:
    /* How s() treats characters that would break the output format. */
    enum class Esc : uint8_t {
        None,
        Csv,   // ',' -> ';'   '"' -> '\''
        Json,  // '"' and '\\' -> '\''
        Kv,    // ',' ';' '=' '"' and control characters -> '_' (a value in the log's key=value detail column)
    };

    Fmt(char* buf, size_t cap); /* out of line: built in many places */

    Fmt& c(char ch);
    Fmt& s(const char* str, Esc esc);
    Fmt& s(const char* str); /* Esc::None, one argument less at each call */
    Fmt& d(int32_t v);
    /* 0..99 as two digits. */
    Fmt& d2(int32_t v);
    /* Tenths as "-12.3" (decimetres -> metres, ...). */
    Fmt& d1(int32_t tenths);
    /* Hundredths shown with one decimal: cm/s -> "21.0" m/s. */
    Fmt& d2to1(int32_t hundredths);
    /* 1e-7 degrees as "-12.3456789". */
    Fmt& e7(int32_t v);
    Fmt& hex(const uint8_t* b, size_t n, char sep = 0, bool lower = false);
    /* Metres as "850m", "12.3k" or "123k". */
    Fmt& dist(uint32_t m);
    /* Spaces up to column col (counted from the start of the buffer). */
    Fmt& pad(size_t col);
    /* Copies tmpl up to its end or a '\n' and returns what follows. The next
     * value of v goes in at '#' (d), '~' (d1), '^' (d2to1), '@' (e7), kNone
     * leaving the field empty; the next string of str at '$' (escaped with
     * esc). One table of values instead of a call per field keeps long lines
     * small in code. */
    static constexpr int32_t kNone = INT32_MIN;
    static constexpr int32_t kDash = INT32_MIN + 1; /* the field is '-' */
    const char* t(const char* tmpl, const int32_t*& v, const char* const*& str, Esc esc = Esc::None);
    const char* t(const char* tmpl, const int32_t* v) {
        const char* const* none = nullptr;
        return t(tmpl, v, none);
    }

    size_t len() const { return static_cast<size_t>(p_ - start_); }
    const char* str() const { return start_; }

   private:
    char* start_;
    char* p_;
    char* end_;
};

} /* namespace ui::external_app::drone_detect */

#endif /*__DD_FMT_HPP__*/
