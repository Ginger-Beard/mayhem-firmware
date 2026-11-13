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

#include "sigdet_demo.hpp"
#include "odid_lite.hpp"

#include <cstring>

namespace ui::external_app::signature_detect {

namespace {

constexpr int64_t kE7 = 10000000;
constexpr int64_t kMetresPerDeg = 111195;

void put16(uint8_t* p, uint32_t v) {
    p[0] = v;
    p[1] = v >> 8;
}

void put32(uint8_t* p, uint32_t v) {
    put16(p, v);
    put16(p + 2, v >> 16);
}

void put_text(uint8_t* p, const char* s, size_t n) {
    for (size_t i = 0; i < n && s[i]; i++) p[i] = s[i];
}

/* Wire encoding of an altitude / height in decimetres. */
uint16_t alt_raw(int32_t dm) {
    return static_cast<uint16_t>((dm + 10000) / 5);
}

/* Position north_m / east_m metres from lat/lon. */
void offset(int32_t lat, int32_t lon, int32_t north_m, int32_t east_m, int32_t* out_lat, int32_t* out_lon) {
    int32_t c = odid_cos_deg_q15(lat / kE7);
    if (c < 1000) c = 1000;
    *out_lat = lat + static_cast<int32_t>(north_m * kE7 / kMetresPerDeg);
    *out_lon = lon + static_cast<int32_t>(east_m * kE7 * 32768 / (kMetresPerDeg * c));
}

struct Drone {
    const char* uas_id;
    const char* operator_id;
    const char* self_id;
    uint8_t ua_type;
    uint8_t op_loc_type;
    uint8_t mac_tag;
    uint8_t rot[4];  // second message of each second, by now_s % 4
};

const Drone kDrones[2] = {
    {"DEMOA1581F5000001", "FIN87astrdge12k8", "Demo: circling", 2, 1, 0x0A, {0, 4, 5, 3}},
    {"DEMOB4D4C0000002", "GBR-OP-DEMO0002", "Demo: hovering", 4, 0, 0x0B, {0, 4, 0, 5}},
};

}  // namespace

int8_t DemoDrones::frame(uint32_t now_s, size_t index, uint8_t* mac, uint8_t* adv) {
    const size_t n = index >= 2 ? 1 : 0;
    const Drone& d = kDrones[n];
    const uint8_t type = (index & 1) ? d.rot[now_s % 4] : 1;

    mac[0] = 0xC0;
    mac[1] = 0xDE;
    mac[2] = 0x00;
    mac[3] = 0x00;
    mac[4] = d.mac_tag;
    mac[5] = n ? static_cast<uint8_t>(now_s / 30) : 0x01;

    adv[0] = 0x1E;
    adv[1] = 0x16;
    adv[2] = 0xFA;
    adv[3] = 0xFF;
    adv[4] = 0x0D;
    adv[5] = counter_[n][type]++;
    uint8_t* m = adv + 6;
    memset(m, 0, ODID_MSG_SIZE);

    /* Geometry: A at bearing `angle` on a 200 m circle, B fixed 400 m NE. */
    const int32_t angle = static_cast<int32_t>((now_s * 6) % 360);
    int32_t lat, lon, op_lat, op_lon;
    uint16_t dir;
    int32_t speed_cms, alt_dm, height_dm;
    int8_t rssi;
    if (n == 0) {
        offset(lat_, lon_, (200 * odid_cos_deg_q15(angle)) >> 15, (200 * odid_cos_deg_q15(angle - 90)) >> 15, &lat, &lon);
        op_lat = lat_;
        op_lon = lon_;
        dir = static_cast<uint16_t>((angle + 90) % 360);
        speed_cms = 2094; /* 2 pi 200 m / 60 s */
        alt_dm = 1200;
        height_dm = 1000;
        rssi = static_cast<int8_t>(-55 + (odid_cos_deg_q15(angle) >> 13));
    } else {
        offset(lat_, lon_, 283, 283, &lat, &lon);
        offset(lat_, lon_, 250, 250, &op_lat, &op_lon);
        dir = 0;
        speed_cms = 0;
        alt_dm = 600;
        height_dm = 400;
        rssi = -72;
    }

    switch (type) {
        case 0: /* Basic ID: serial number */
            m[0] = 0x02;
            m[1] = (1 << 4) | d.ua_type;
            put_text(m + 2, d.uas_id, 20);
            break;
        case 1: { /* Location / vector, airborne, height over take-off */
            const bool ew = dir >= 180;
            m[0] = 0x12;
            m[1] = (2 << 4) | (ew ? 0x02 : 0);
            m[2] = static_cast<uint8_t>(dir - (ew ? 180 : 0));
            m[3] = static_cast<uint8_t>(speed_cms / 25);
            m[4] = 0;
            put32(m + 5, lat);
            put32(m + 9, lon);
            put16(m + 13, alt_raw(alt_dm - 20));
            put16(m + 15, alt_raw(alt_dm));
            put16(m + 17, alt_raw(height_dm));
            m[19] = 0x4A;
            m[20] = 0x43;
            put16(m + 21, (now_s % 3600) * 10);
            m[23] = 1;
            break;
        }
        case 3: /* Self-ID */
            m[0] = 0x32;
            put_text(m + 2, d.self_id, 23);
            break;
        case 4: /* System: operator position */
            m[0] = 0x42;
            m[1] = d.op_loc_type;
            put32(m + 2, op_lat);
            put32(m + 6, op_lon);
            put16(m + 10, 1);
            put16(m + 18, alt_raw(200));
            put32(m + 20, 250000000 + now_s);
            break;
        default: /* 5: Operator ID */
            m[0] = 0x52;
            put_text(m + 2, d.operator_id, 20);
            break;
    }
    return rssi;
}

} /* namespace ui::external_app::signature_detect */
