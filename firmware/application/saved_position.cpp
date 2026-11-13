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

#include "saved_position.hpp"

#include "app_settings.hpp"
#include "rtc_time.hpp"

namespace saved_position {

namespace {

constexpr std::string_view kStore = "position"sv; /* /SETTINGS/position.ini */

SettingBindings bindings(Position& p) {
    return {{"lat_e7"sv, &p.lat_e7},
            {"lon_e7"sv, &p.lon_e7},
            {"alt_m"sv, &p.alt_m},
            {"source"sv, &p.source},
            {"time"sv, &p.time},
            {"acc_m"sv, &p.acc_m},
            {"sats"sv, &p.sats}};
}

/* Days since 1980-01-01 (proleptic Gregorian). */
int32_t days_since_1980(int32_t y, int32_t m, int32_t d) {
    y -= m <= 2;
    const int32_t era = y / 400;
    const int32_t yoe = y - era * 400;
    const int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 723120; /* 723120 = 719468 (to 1970-01-01) + 3652 (1970 to 1980) */
}

/* FAT date-time -> 1 + seconds since 1980-01-01 (so 0 can mean "not a date"). */
uint32_t fat_seconds(uint32_t fat) {
    const int32_t m = (fat >> 21) & 0x0F;
    const int32_t d = (fat >> 16) & 0x1F;
    if (m < 1 || m > 12 || d < 1) return 0;
    const int32_t days = days_since_1980(1980 + static_cast<int32_t>(fat >> 25), m, d);
    return static_cast<uint32_t>(days) * 86400u + ((fat >> 11) & 0x1F) * 3600u + ((fat >> 5) & 0x3F) * 60u + (fat & 0x1F) * 2u + 1u;
}

}  // namespace

bool Position::valid() const {
    return source != SOURCE_NONE && lat_e7 >= -900000000 && lat_e7 <= 900000000 &&
           lon_e7 >= -1800000000 && lon_e7 <= 1800000000 && (lat_e7 != 0 || lon_e7 != 0);
}

bool load(Position& p) {
    p = Position{};
    auto b = bindings(p);
    if (load_settings(kStore, b) && p.valid()) return true;
    p = Position{};
    return false;
}

bool save(const Position& p) {
    Position copy = p;
    return save_settings(kStore, bindings(copy));
}

uint32_t now_fat() {
    const auto t = rtc_time::now();
    const uint32_t y = t.year() >= 1980 ? t.year() - 1980u : 0u;
    return (y << 25) | (static_cast<uint32_t>(t.month()) << 21) | (static_cast<uint32_t>(t.day()) << 16) |
           (static_cast<uint32_t>(t.hour()) << 11) | (static_cast<uint32_t>(t.minute()) << 5) | (t.second() / 2u);
}

int32_t age_s(uint32_t then, uint32_t now) {
    const uint32_t a = fat_seconds(then), b = fat_seconds(now);
    if (!a || !b || b < a) return -1;
    const uint32_t d = b - a;
    return d > 0x7FFFFFFFu ? -1 : static_cast<int32_t>(d);
}

const char* source_name(uint8_t source) {
    switch (source) {
        case SOURCE_GPS:
            return "GPS";
        case SOURCE_HACKRF_GPS:
            return "HackRF GPS";
        case SOURCE_MANUAL:
            return "Manual";
        case SOURCE_MAP:
            return "Map";
        default:
            return "?";
    }
}

void format_age(char* buf, size_t size, int32_t age) {
    if (!buf || size < 6) return;
    if (age < 0) {
        buf[0] = '?';
        buf[1] = 0;
        return;
    }
    char unit = 's';
    int32_t v = age;
    if (v >= 86400) {
        v /= 86400;
        unit = 'd';
    } else if (v >= 3600) {
        v /= 3600;
        unit = 'h';
    } else if (v >= 60) {
        v /= 60;
        unit = 'm';
    }
    if (v > 9999) v = 9999;
    char tmp[5];
    size_t n = 0;
    do {
        tmp[n++] = static_cast<char>('0' + v % 10);
        v /= 10;
    } while (v && n < sizeof(tmp));
    size_t i = 0;
    while (n) buf[i++] = tmp[--n];
    buf[i++] = unit;
    buf[i] = 0;
}

}  // namespace saved_position
