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

// Small, heap-free, integer-only Open Drone ID (ASTM F3411) decoder.
// Byte layouts follow opendroneid-core-c (opendroneid.h / opendroneid.c).

#ifndef __ODID_LITE_HPP__
#define __ODID_LITE_HPP__

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ui::external_app::drone_detect {

constexpr size_t ODID_MSG_SIZE = 25;

// One tracked Remote ID emitter; stores raw wire encodings, converts on demand.
// Fields are packed (bitfields for small enums) to keep sizeof <= 128, and
// grouped by message type in one run of bytes each (Location, System, Basic
// ID, Self-ID, Operator ID, in that order), so a group copies as one block.
struct RidRecord {
    /* Location / vector (type 1) */
    int32_t lat_e7, lon_e7;  // 0/0 = unknown
    uint16_t alt_baro_raw, alt_geo_raw, height_raw, ts_raw;
    uint8_t dir_raw, speed_h_raw;
    int8_t speed_v_raw;
    uint8_t status;           // 0..4 (raw 4-bit value stored)
    uint8_t height_type : 1;  // 0 over takeoff, 1 AGL
    uint8_t ew : 1;           // direction East/West flag (+180)
    uint8_t speed_mult : 1;   // speed multiplier flag
    uint8_t horiz_acc : 4, vert_acc : 4;
    uint8_t baro_acc : 4, speed_acc : 4;
    uint8_t ts_acc : 4;
    /* System (type 4) */
    int32_t op_lat_e7, op_lon_e7;  // 0/0 = unknown
    uint32_t sys_timestamp;        // s since 2019-01-01
    uint16_t op_alt_raw;
    uint16_t area_count;
    uint16_t area_ceiling_raw, area_floor_raw;
    uint8_t area_radius_raw;  // x10 m
    uint8_t op_loc_type : 2;  // 0 takeoff, 1 live GNSS, 2 fixed
    uint8_t class_type : 3;
    uint8_t eu_category : 4, eu_class : 4;
    /* Basic ID (type 0) */
    char uas_id[21];  // NUL terminated; for binary ID types (3,4) stored as received
    uint8_t id_type : 4, ua_type : 4;
    uint8_t binary_id : 1;  // uas_id holds raw bytes (ID types 3,4)
    /* Self-ID (type 3) */
    char self_id[24];
    uint8_t desc_type;
    /* Operator ID (type 5) */
    char operator_id[21];
    uint8_t operator_id_type;
    /* Not tied to one message */
    uint8_t mac[6];
    uint8_t msg_seen;  // bit n set when message type n (0..5) has been applied
    uint8_t proto_version;
};
static_assert(sizeof(RidRecord) <= 128, "RidRecord too large");

// Find BLE Service Data (0x16), UUID 0xFFFA, app code 0x0D in AD structures.
// Returns pointer to ODID payload (byte after counter), nullptr if absent/malformed.
const uint8_t* odid_find_ble(const uint8_t* adv, size_t len, size_t* payload_len, uint8_t* counter);

// 802.11 vendor IE body: ie points at the OUI (after the length byte).
// Accepts OUI FA:0B:BC and Parrot 90:3A:E6, type 0x0D. Returns pointer after counter.
const uint8_t* odid_find_wifi_ie(const uint8_t* ie, size_t ie_len, size_t* payload_len, uint8_t* counter);

// Payload of a Wi-Fi Remote ID frame forwarded by the drone detect baseband
// (dd::DroneDetectWifiRid): kind ODID_WIFI_IE is a vendor IE body (OUI on,
// as odid_find_wifi_ie takes it), ODID_WIFI_NAN a NAN service info (counter,
// then the message pack). Returns the message pack, nullptr otherwise.
constexpr uint8_t ODID_WIFI_IE = 1;
constexpr uint8_t ODID_WIFI_NAN = 2;
const uint8_t* odid_find_wifi_payload(uint8_t kind, const uint8_t* p, size_t len, size_t* payload_len, uint8_t* counter);

// Apply a single message or a message pack (type 0xF) to rec.
// Returns bitmask of message types applied (bit n = type n); 0 if nothing valid.
// Auth (type 2) messages are only flagged (bit 2), their content is not stored.
uint32_t odid_apply(RidRecord& rec, const uint8_t* msg, size_t len);

// Altitude: raw 0 = invalid. Result in decimetres: raw*5 - 10000.
bool odid_alt_valid(uint16_t raw);
int32_t odid_alt_dm(uint16_t raw);

// Horizontal speed in cm/s: mult0 raw*25, mult1 raw*75 + 6375.
// Invalid is 255 m/s, i.e. mult1 with raw 255 (see odid_lite.cpp).
bool odid_speed_h_valid(const RidRecord& r);
int32_t odid_speed_h_cms(const RidRecord& r);

// Vertical speed in cm/s: raw*50. 63 m/s (raw 126) is invalid.
bool odid_speed_v_valid(const RidRecord& r);
int32_t odid_speed_v_cms(const RidRecord& r);

// Direction in degrees: dir_raw + (ew ? 180 : 0). Values > 360 are invalid.
bool odid_direction_valid(const RidRecord& r);
uint16_t odid_direction_deg(const RidRecord& r);

// Valid when not (0,0) and within +-90 / +-180 degrees.
bool odid_pos_valid(int32_t lat_e7, int32_t lon_e7);

// Formats a 1e-7 degree value as "-123.4567890" (cap >= 14). Returns length.
size_t odid_format_e7(int32_t v_e7, char* out, size_t cap);

// Integer-only equirectangular distance (metres) and initial bearing (degrees 0..359).
uint32_t odid_distance_m(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7);
uint16_t odid_bearing_deg(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7);

// cos(deg) in Q15 for any integer number of degrees (1 degree table).
int32_t odid_cos_deg_q15(int32_t deg);

}  // namespace ui::external_app::drone_detect

#endif /*__ODID_LITE_HPP__*/
