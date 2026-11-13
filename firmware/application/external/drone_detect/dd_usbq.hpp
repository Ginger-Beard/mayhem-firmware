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

#ifndef __DD_USBQ_HPP__
#define __DD_USBQ_HPP__

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ui::external_app::drone_detect {

/* The app's USB serial lines, never blocking the caller (pure logic, host
 * tested in rf_test/dd_usbq_test.cpp).
 *
 * The stock path (SUSBD1's output queue) cannot be used without blocking:
 * its notify hook starts one USB transfer per byte written and, while the
 * bulk IN endpoint's only transfer slot is busy (host slow or not reading),
 * sleeps in 1 ms steps until it is free, whatever timeout the writer gave.
 *
 * Here lines go into one of two buffers ("\r\n" appended); drain() hands
 * the filled one, whole lines only, to send(), which starts one transfer
 * from it and returns false when none can start now (the slot is busy, by
 * our previous transfer or the shell's). The endpoint has a single transfer
 * slot, so a transfer that starts proves the previous one, from the other
 * buffer, has completed: the buffers swap and the other one is filled next.
 * A buffer is thus never written while the USB controller may still read it,
 * with no completion callback (which would point into the app's code after
 * it exits).
 *
 * A line that does not fit is dropped and counted: the newest goes, queued
 * lines stay whole and in order. A transfer never ends on a full 64 byte
 * packet (the endpoint sends no zero length packet, so the host would hold
 * the data until more came): its last byte waits for the next transfer. */
class UsbLineQueue {
   public:
    static constexpr size_t kBufSize = 1024;
    static constexpr size_t kPacket = 64; /* full speed bulk max packet */

    /* Starts a transfer of len bytes at data, which stay untouched until a
     * later send() returns true; false: cannot start one now. */
    using Send = bool (*)(void* ctx, const uint8_t* data, size_t len);

    bool push(const char* line) {
        const size_t n = strlen(line);
        if (n + 2 > kBufSize - len_) {
            dropped_++;
            return false;
        }
        uint8_t* const p = buf_[fill_] + len_;
        memcpy(p, line, n);
        p[n] = '\r';
        p[n + 1] = '\n';
        len_ += n + 2;
        return true;
    }

    /* true when a transfer was started. */
    bool drain(Send send, void* ctx) {
        if (len_ == 0) return false;
        const size_t n = (len_ % kPacket == 0) ? len_ - 1 : len_;
        if (n == 0 || !send(ctx, buf_[fill_], n)) return false;
        const uint8_t carry = buf_[fill_][len_ - 1];
        const bool carried = n != len_;
        fill_ ^= 1;
        len_ = 0;
        if (carried) buf_[fill_][len_++] = carry;
        sent_++;
        return true;
    }

    size_t pending() const { return len_; }
    uint32_t dropped() const { return dropped_; }
    uint32_t sent() const { return sent_; }

   private:
    uint8_t buf_[2][kBufSize];
    size_t len_{0};
    uint8_t fill_{0};
    uint32_t dropped_{0};
    uint32_t sent_{0};
};

} /* namespace ui::external_app::drone_detect */

#endif /*__DD_USBQ_HPP__*/
