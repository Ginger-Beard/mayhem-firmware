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

#ifndef __SAVED_POSITION_H__
#define __SAVED_POSITION_H__

#include <cstddef>
#include <cstdint>

/* One position shared by every app: /SETTINGS/position.ini, written with the
 * same loader as the apps' own .ini files (app_settings.hpp). An app saves a
 * position it knows (a GPS fix, a point picked on the map, ...) and any other
 * app can load it later; Mayhem runs one app at a time, so a file is the way
 * to hand a position over. It is never a live fix: show its source and age.
 * The file is read or written only when called (no RAM is kept).
 *
 * Keys (all integers, one "key=value" per line):
 *   lat_e7, lon_e7  latitude / longitude in 1e-7 degree (WGS84)
 *   alt_m           altitude in metres (0 = unknown)
 *   source          1 GPS (external module, e.g. the ESP32 add-on or the USB
 *                   shell "gotgps"), 2 the HackRF itself as GPS receiver,
 *                   3 typed in by hand, 4 picked on a map
 *   time            when the position was found: FAT date-time (bits 31-25
 *                   year-1980, 24-21 month, 20-16 day, 15-11 hour, 10-5
 *                   minute, 4-0 seconds/2), 0 = unknown
 *   acc_m           estimated horizontal error in metres, 0 = unknown
 *   sats            satellites used, 0 = unknown / not a GPS fix
 */
namespace saved_position {

enum Source : uint8_t {
    SOURCE_NONE = 0,
    SOURCE_GPS = 1,
    SOURCE_HACKRF_GPS = 2,
    SOURCE_MANUAL = 3,
    SOURCE_MAP = 4,
};

struct Position {
    int32_t lat_e7{0};
    int32_t lon_e7{0};
    int32_t alt_m{0};
    uint8_t source{SOURCE_NONE};
    uint8_t sats{0};
    uint32_t time{0};
    uint32_t acc_m{0};

    bool valid() const;
};

/* false: no file, unreadable, or not a valid position (p is then cleared). */
bool load(Position& p);
/* false: the SD card could not be written. */
bool save(const Position& p);

/* The RTC now, as a FAT date-time. */
uint32_t now_fat();
/* Seconds from `then` to `now` (FAT date-times), -1 if unknown. */
int32_t age_s(uint32_t then, uint32_t now);
/* "GPS", "HackRF GPS", "Manual", "Map" or "?". */
const char* source_name(uint8_t source);
/* "45s", "12m", "3h", "5d" or "?" into buf (at least 6 bytes). */
void format_age(char* buf, size_t size, int32_t age);

}  // namespace saved_position

#endif /*__SAVED_POSITION_H__*/
