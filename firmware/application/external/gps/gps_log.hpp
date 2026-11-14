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

#ifndef __GPS_LOG_H__
#define __GPS_LOG_H__

/* Diagnostics log of the GPS app (the "Log" box): rows are formatted into a
 * RAM ring (in the M4 RAM the image lends the app, not the M0 heap or stack)
 * wherever they happen, and written to the card later, from the display
 * frame handler, in pieces that never cross a 512-byte sector boundary and
 * stay below 512 bytes, so FatFs always copies them through its own aligned
 * sector buffer (its direct multi-sector path is never taken). A full ring
 * drops rows (counted) instead of blocking. Header only, host tested. */

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ui::external_app::gps {

constexpr size_t kLogRing = 4096; /* ~2 s of rows while tracking 8 channels */
constexpr size_t kLogRow = 480;   /* one row, < one sector with its CR LF */
constexpr size_t kLogSector = 512;

struct LogRing {
    char buf[kLogRing];
    uint32_t head; /* bytes pushed, ever (index = head % kLogRing) */
    uint32_t tail; /* bytes taken */
    uint32_t rows, dropped;
    char row[kLogRow + 2]; /* the row being built */
};

/* One row (no line end; CR LF added). False (counted) when it does not fit. */
inline bool log_push(LogRing& r, const char* s, size_t n) {
    if (n > kLogRow) n = kLogRow;
    if (kLogRing - (r.head - r.tail) < n + 2) {
        r.dropped++;
        return false;
    }
    for (size_t k = 0; k < n; k++) r.buf[(r.head + k) % kLogRing] = s[k];
    r.buf[(r.head + n) % kLogRing] = '\r';
    r.buf[(r.head + n + 1) % kLogRing] = '\n';
    r.head += static_cast<uint32_t>(n + 2);
    r.rows++;
    return true;
}

inline size_t log_pending(const LogRing& r) {
    return r.head - r.tail;
}

/* The next piece to write at file position pos: contiguous in the ring, to
 * the end of the current sector at most, and below 512 bytes. */
inline size_t log_piece(const LogRing& r, uint32_t pos, const char** p) {
    size_t n = r.head - r.tail;
    const size_t at = r.tail % kLogRing;
    if (n > kLogRing - at) n = kLogRing - at; /* to the ring's end */
    const size_t room = kLogSector - pos % kLogSector;
    const size_t cap = room < kLogSector ? room : kLogSector - 1;
    if (n > cap) n = cap;
    *p = r.buf + at;
    return n;
}

inline void log_taken(LogRing& r, size_t n) {
    r.tail += static_cast<uint32_t>(n);
}

/* The header (a constant text, written before any queued row so it never
 * fills the ring): the next piece of it at file position pos, from header
 * offset done, by the same sector rule. 0 when it is all written. */
inline size_t log_head_piece(const char* head, size_t len, size_t done, uint32_t pos, const char** p) {
    if (done >= len) return 0;
    size_t n = len - done;
    const size_t room = kLogSector - pos % kLogSector;
    const size_t cap = room < kLogSector ? room : kLogSector - 1;
    if (n > cap) n = cap;
    *p = head + done;
    return n;
}

}  // namespace ui::external_app::gps

#endif /*__GPS_LOG_H__*/
