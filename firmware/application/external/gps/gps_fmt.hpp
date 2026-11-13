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

#ifndef __GPS_FMT_H__
#define __GPS_FMT_H__

/* Text into a fixed char buffer, no heap (always terminated, cut when full). */

#include <cstddef>
#include <cstdint>

namespace ui::external_app::gps {

class Fmt {
   public:
    Fmt(char* buf, size_t size)
        : b_{buf}, n_{size} {
        if (n_) b_[0] = 0;
    }
    Fmt& c(char ch) {
        if (i_ + 1 < n_) {
            b_[i_++] = ch;
            b_[i_] = 0;
        }
        return *this;
    }
    Fmt& s(const char* str) {
        while (str && *str) c(*str++);
        return *this;
    }
    Fmt& d(int32_t v) {
        if (v < 0) {
            c('-');
            return u(static_cast<uint32_t>(-static_cast<int64_t>(v)));
        }
        return u(static_cast<uint32_t>(v));
    }
    Fmt& u(uint32_t v, int width = 0, char fill = ' ') {
        char t[11];
        int k = 0;
        do {
            t[k++] = static_cast<char>('0' + v % 10);
            v /= 10;
        } while (v && k < 10);
        while (width-- > k) c(fill);
        while (k) c(t[--k]);
        return *this;
    }
    Fmt& d2(uint32_t v) { return u(v, 2, '0'); }
    /* v / 10^dec with dec decimals (v already scaled). */
    Fmt& fixed(int32_t v, int dec) {
        if (v < 0) {
            c('-');
            v = -v;
        }
        int32_t p = 1;
        for (int k = 0; k < dec; k++) p *= 10;
        u(static_cast<uint32_t>(v / p));
        if (dec) {
            c('.');
            u(static_cast<uint32_t>(v % p), dec, '0');
        }
        return *this;
    }
    /* Pad with blanks to column col. */
    Fmt& pad(size_t col) {
        while (i_ < col) c(' ');
        return *this;
    }
    size_t len() const { return i_; }

   private:
    char* b_;
    size_t n_;
    size_t i_{0};
};

}  // namespace ui::external_app::gps

#endif /*__GPS_FMT_H__*/
