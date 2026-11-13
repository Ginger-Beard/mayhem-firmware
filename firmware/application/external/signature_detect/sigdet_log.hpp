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

#ifndef __SIGDET_LOG_HPP__
#define __SIGDET_LOG_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::signature_detect {

/* Bytes of a log row to hand FatFs in one f_write at file position pos.
 * f_write sends whole sectors straight from the caller's buffer when, after
 * filling the current partial sector, 512 bytes or more remain; the LPC43xx
 * SD driver's DMA needs a word aligned buffer (sdc_lld.c), and a row buffer
 * at an odd offset then reaches the card shifted by 1-3 bytes (seen on the
 * card: HB rows, the only ones over 512 bytes, damaged at sector starts).
 * Writes that never cross a sector boundary and stay under 512 bytes always
 * go through FatFs's own (aligned) sector buffer. */
constexpr size_t kLogSector = 512;
constexpr size_t log_chunk(uint32_t pos, size_t left) {
    const size_t room = kLogSector - pos % kLogSector; /* to the sector's end, 1..512 */
    const size_t cap = room < kLogSector ? room : kLogSector - 1;
    return left < cap ? left : cap;
}

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_LOG_HPP__*/
