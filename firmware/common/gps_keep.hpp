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

#ifndef __GPS_KEEP_H__
#define __GPS_KEEP_H__

/* The top of the M4 RAM that survives a switch between the GPS app's
 * baseband images (HackRF Pro only).
 *
 * The GPS images (L1 acquisition, L1 tracking, ...) are linked with an M4
 * RAM shortened by kSize (baseband/CMakeLists.txt, LD_M4_RAM_LEN), so their
 * start-up code (.data, .bss, stacks, heap) never touches this block: when
 * the app shuts one image down and starts the next, what is here stays.
 * It holds the RAM the images lend the app (orbit tables, solver work,
 * Autonomous state, the diagnostics log ring), the search results the
 * tracking image takes over from the acquisition image, and the block the
 * tracking loops publish in. The app clears the lent part once, when the
 * first image reports it; the images never touch it. */

#include <cstdint>
#include <cstddef>

#include "message.hpp"
#include "gps_rx_result.hpp"
#include "gps_trk_shared.hpp"

namespace gps_keep {

constexpr uint32_t kSize = 28 * 1024;       /* also GPS_KEEP_SIZE in baseband/CMakeLists.txt */
constexpr uint32_t kM4RamSize = 128 * 1024; /* the M4's RAM on the LPC4330 (local SRAM bank 1) */
constexpr uintptr_t kBase = 0x10000000u + kM4RamSize - kSize;

struct Block {
    uint8_t scratch[GpsRxStatusMessage::kScratchSize]; /* lent to the app */
    gps_rx::Result results[32];                        /* the last search's results, by PRN - 1 */
    gps_trk::Shared shared;                            /* the tracking loops' output */
};
static_assert(sizeof(Block) <= kSize, "the kept block outgrew GPS_KEEP_SIZE");
static_assert((GpsRxStatusMessage::kScratchSize & 7u) == 0, "the lent RAM must stay 8-byte aligned");

#if defined(GPS_KEEP_SIZE)
static_assert(GPS_KEEP_SIZE == kSize, "GPS_KEEP_SIZE (CMake) and gps_keep::kSize disagree");
#endif

inline Block* block() {
    return reinterpret_cast<Block*>(kBase);
}

}  // namespace gps_keep

#endif /*__GPS_KEEP_H__*/
