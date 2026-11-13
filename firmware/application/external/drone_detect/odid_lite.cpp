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

#include "odid_lite.hpp"

namespace ui::external_app::drone_detect {

namespace {

constexpr uint8_t kMsgBasicId = 0;
constexpr uint8_t kMsgLocation = 1;
constexpr uint8_t kMsgAuth = 2;
constexpr uint8_t kMsgSelfId = 3;
constexpr uint8_t kMsgSystem = 4;
constexpr uint8_t kMsgOperatorId = 5;
constexpr uint8_t kMsgPack = 0xF;
constexpr size_t kPackMax = 9;

uint16_t rd_u16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t rd_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int32_t rd_i32(const uint8_t* p) {
    return static_cast<int32_t>(rd_u32(p));
}

// Copy up to n bytes (stops at first NUL), replace non-printable with '?',
// strip trailing spaces, NUL terminate. dst must hold n+1 bytes.
void copy_text(char* dst, const uint8_t* src, size_t n) {
    size_t i = 0;
    for (; i < n && src[i] != 0; i++) {
        uint8_t c = src[i];
        dst[i] = (c >= 0x20 && c <= 0x7E) ? static_cast<char>(c) : '?';
    }
    while (i > 0 && dst[i - 1] == ' ')
        i--;
    dst[i] = '\0';
    // Clear the remainder so the record is deterministic.
    for (size_t j = i + 1; j <= n; j++)
        dst[j] = '\0';
}

void apply_basic_id(RidRecord& r, const uint8_t* m) {
    r.id_type = m[1] >> 4;
    r.ua_type = m[1] & 0x0F;
    if (r.id_type == 3 || r.id_type == 4) {
        r.binary_id = 1;
        memcpy(r.uas_id, &m[2], 20);
        r.uas_id[20] = '\0';
    } else {
        r.binary_id = 0;
        copy_text(r.uas_id, &m[2], 20);
    }
}

void apply_location(RidRecord& r, const uint8_t* m) {
    r.speed_mult = m[1] & 1;
    r.ew = (m[1] >> 1) & 1;
    r.height_type = (m[1] >> 2) & 1;
    r.status = m[1] >> 4;
    r.dir_raw = m[2];
    r.speed_h_raw = m[3];
    r.speed_v_raw = static_cast<int8_t>(m[4]);
    r.lat_e7 = rd_i32(&m[5]);
    r.lon_e7 = rd_i32(&m[9]);
    r.alt_baro_raw = rd_u16(&m[13]);
    r.alt_geo_raw = rd_u16(&m[15]);
    r.height_raw = rd_u16(&m[17]);
    r.horiz_acc = m[19] & 0x0F;
    r.vert_acc = m[19] >> 4;
    r.speed_acc = m[20] & 0x0F;
    r.baro_acc = m[20] >> 4;
    r.ts_raw = rd_u16(&m[21]);
    r.ts_acc = m[23] & 0x0F;
}

void apply_system(RidRecord& r, const uint8_t* m) {
    r.op_loc_type = m[1] & 0x03;
    r.class_type = (m[1] >> 2) & 0x07;
    r.op_lat_e7 = rd_i32(&m[2]);
    r.op_lon_e7 = rd_i32(&m[6]);
    r.area_count = rd_u16(&m[10]);
    r.area_radius_raw = m[12];
    r.area_ceiling_raw = rd_u16(&m[13]);
    r.area_floor_raw = rd_u16(&m[15]);
    r.eu_class = m[17] & 0x0F;
    r.eu_category = m[17] >> 4;
    r.op_alt_raw = rd_u16(&m[18]);
    r.sys_timestamp = rd_u32(&m[20]);
}

// Apply exactly one 25-byte non-pack message.
uint32_t apply_single(RidRecord& r, const uint8_t* m) {
    uint8_t type = m[0] >> 4;
    switch (type) {
        case kMsgBasicId:
            apply_basic_id(r, m);
            break;
        case kMsgLocation:
            apply_location(r, m);
            break;
        case kMsgAuth:
            break;  // flag only
        case kMsgSelfId:
            r.desc_type = m[1];
            copy_text(r.self_id, &m[2], 23);
            break;
        case kMsgSystem:
            apply_system(r, m);
            break;
        case kMsgOperatorId:
            r.operator_id_type = m[1];
            copy_text(r.operator_id, &m[2], 20);
            break;
        default:
            return 0;
    }
    r.proto_version = m[0] & 0x0F;
    r.msg_seen |= static_cast<uint8_t>(1u << type);
    return 1u << type;
}

// Cos table, Q15, 0..90 degrees in 1 degree steps.
const uint16_t kCosQ15[91] = {
    32768, 32763, 32748, 32723, 32688, 32643, 32588, 32524, 32449, 32365, 32270, 32166, 32052, 31928, 31795,
    31651, 31499, 31336, 31164, 30983, 30792, 30592, 30382, 30163, 29935, 29698, 29452, 29197, 28932, 28660,
    28378, 28088, 27789, 27482, 27166, 26842, 26510, 26170, 25822, 25466, 25102, 24730, 24351, 23965, 23571,
    23170, 22763, 22348, 21926, 21498, 21063, 20622, 20174, 19720, 19261, 18795, 18324, 17847, 17364, 16877,
    16384, 15886, 15384, 14876, 14365, 13848, 13328, 12803, 12275, 11743, 11207, 10668, 10126, 9580, 9032,
    8481, 7927, 7371, 6813, 6252, 5690, 5126, 4560, 3993, 3425, 2856, 2286, 1715, 1144, 572, 0};

// atan(i/64) in millidegrees, i = 0..64.
const uint16_t kAtan64[65] = {
    0, 895, 1790, 2684, 3576, 4467, 5356, 6242, 7125, 8005, 8881, 9752, 10620, 11482, 12339, 13191, 14036,
    14876, 15709, 16535, 17354, 18166, 18970, 19767, 20556, 21337, 22109, 22874, 23629, 24376, 25115, 25844,
    26565, 27277, 27979, 28673, 29358, 30033, 30700, 31357, 32005, 32645, 33275, 33896, 34509, 35112, 35707,
    36293, 36870, 37439, 37999, 38550, 39094, 39629, 40156, 40675, 41186, 41689, 42184, 42672, 43152, 43625,
    44091, 44549, 45000};

constexpr int64_t kE7 = 10000000;
// Metres per degree of latitude (mean), 111195 m.
constexpr int64_t kMetresPerDeg = 111195;

uint32_t isqrt_u64(uint64_t x) {
    uint64_t res = 0;
    uint64_t bit = 1ULL << 62;
    while (bit > x)
        bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<uint32_t>(res);
}

// cos of latitude (e7 degrees) in Q15, linear interpolation.
int32_t cos_lat_q15(int64_t lat_e7) {
    if (lat_e7 < 0) lat_e7 = -lat_e7;
    if (lat_e7 >= 90 * kE7) return 0;
    uint32_t idx = static_cast<uint32_t>(lat_e7 / kE7);
    int64_t frac = lat_e7 % kE7;
    int32_t a = kCosQ15[idx];
    int32_t b = kCosQ15[idx + 1];
    return a + static_cast<int32_t>(((b - a) * frac) / kE7);
}

// East/north displacement in centimetres from point 1 to point 2.
void displacement_cm(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, int64_t* east_cm, int64_t* north_cm) {
    int64_t dlat = static_cast<int64_t>(lat2) - lat1;
    int64_t dlon = static_cast<int64_t>(lon2) - lon1;
    if (dlon > 180 * kE7) dlon -= 360 * kE7;
    if (dlon < -180 * kE7) dlon += 360 * kE7;
    int64_t mid = (static_cast<int64_t>(lat1) + lat2) / 2;
    *north_cm = dlat * kMetresPerDeg * 100 / kE7;
    *east_cm = dlon * kMetresPerDeg * 100 / kE7 * cos_lat_q15(mid) / 32768;
}

}  // namespace

const uint8_t* odid_find_ble(const uint8_t* adv, size_t len, size_t* payload_len, uint8_t* counter) {
    if (adv == nullptr) return nullptr;
    size_t i = 0;
    while (i < len) {
        size_t l = adv[i];
        if (l == 0) break;                    // padding / end of AD data
        if (i + 1 + l > len) return nullptr;  // truncated structure
        // Need type(1) + uuid(2) + appcode(1) + counter(1) = 5 bytes.
        if (l >= 5 && adv[i + 1] == 0x16 && adv[i + 2] == 0xFA && adv[i + 3] == 0xFF && adv[i + 4] == 0x0D) {
            if (counter) *counter = adv[i + 5];
            if (payload_len) *payload_len = l - 5;
            return &adv[i + 6];
        }
        i += 1 + l;
    }
    return nullptr;
}

const uint8_t* odid_find_wifi_ie(const uint8_t* ie, size_t ie_len, size_t* payload_len, uint8_t* counter) {
    if (ie == nullptr || ie_len < 5) return nullptr;
    bool oui_ok = (ie[0] == 0xFA && ie[1] == 0x0B && ie[2] == 0xBC) ||
                  (ie[0] == 0x90 && ie[1] == 0x3A && ie[2] == 0xE6);
    if (!oui_ok || ie[3] != 0x0D) return nullptr;
    if (counter) *counter = ie[4];
    if (payload_len) *payload_len = ie_len - 5;
    return &ie[5];
}

const uint8_t* odid_find_wifi_payload(uint8_t kind, const uint8_t* p, size_t len, size_t* payload_len, uint8_t* counter) {
    if (p == nullptr) return nullptr;
    if (kind == ODID_WIFI_IE) return odid_find_wifi_ie(p, len, payload_len, counter);
    if (kind != ODID_WIFI_NAN || len < 1 + ODID_MSG_SIZE) return nullptr;
    if (counter) *counter = p[0];
    if (payload_len) *payload_len = len - 1;
    return p + 1;
}

uint32_t odid_apply(RidRecord& rec, const uint8_t* msg, size_t len) {
    if (msg == nullptr || len < ODID_MSG_SIZE) return 0;
    uint8_t type = msg[0] >> 4;
    if (type != kMsgPack)
        return apply_single(rec, msg);

    // Message pack: [hdr][size=25][count][count * 25 bytes]
    if (len < 3 || msg[1] != ODID_MSG_SIZE) return 0;
    size_t count = msg[2];
    if (count < 1 || count > kPackMax) return 0;
    if (len < 3 + count * ODID_MSG_SIZE) return 0;
    // Reference rejects the whole pack if it holds anything but types 0..5
    // (this also rules out nesting).
    for (size_t i = 0; i < count; i++) {
        if ((msg[3 + i * ODID_MSG_SIZE] >> 4) > kMsgOperatorId) return 0;
    }
    uint32_t applied = 0;
    for (size_t i = 0; i < count; i++)
        applied |= apply_single(rec, &msg[3 + i * ODID_MSG_SIZE]);
    return applied;
}

bool odid_alt_valid(uint16_t raw) {
    return raw != 0;
}

int32_t odid_alt_dm(uint16_t raw) {
    return static_cast<int32_t>(raw) * 5 - 10000;
}

// The spec's "invalid" value is 255 m/s which encodes as mult=1, raw=255.
// (mult=0, raw=255 is a legitimate 63.75 m/s.)
bool odid_speed_h_valid(const RidRecord& r) {
    return !(r.speed_mult != 0 && r.speed_h_raw == 255);
}

int32_t odid_speed_h_cms(const RidRecord& r) {
    int32_t raw = r.speed_h_raw;
    return r.speed_mult ? raw * 75 + 6375 : raw * 25;
}

bool odid_speed_v_valid(const RidRecord& r) {
    return r.speed_v_raw != 126;
}

int32_t odid_speed_v_cms(const RidRecord& r) {
    return static_cast<int32_t>(r.speed_v_raw) * 50;
}

bool odid_direction_valid(const RidRecord& r) {
    return odid_direction_deg(r) <= 360;
}

uint16_t odid_direction_deg(const RidRecord& r) {
    return static_cast<uint16_t>(r.dir_raw + (r.ew ? 180 : 0));
}

bool odid_pos_valid(int32_t lat_e7, int32_t lon_e7) {
    if (lat_e7 == 0 && lon_e7 == 0) return false;
    return lat_e7 >= -900000000 && lat_e7 <= 900000000 && lon_e7 >= -1800000000 && lon_e7 <= 1800000000;
}

size_t odid_format_e7(int32_t v_e7, char* out, size_t cap) {
    if (out == nullptr || cap == 0) return 0;
    char tmp[16];
    size_t n = 0;
    int64_t v = v_e7;
    bool neg = v < 0;
    if (neg) v = -v;
    uint32_t ip = static_cast<uint32_t>(v / kE7);
    uint32_t fp = static_cast<uint32_t>(v % kE7);
    if (neg) tmp[n++] = '-';
    char ib[4];
    size_t k = 0;
    do {
        ib[k++] = static_cast<char>('0' + ip % 10);
        ip /= 10;
    } while (ip != 0);
    while (k > 0)
        tmp[n++] = ib[--k];
    tmp[n++] = '.';
    for (int d = 6; d >= 0; d--) {
        uint32_t p = 1;
        for (int j = 0; j < d; j++)
            p *= 10;
        tmp[n++] = static_cast<char>('0' + (fp / p) % 10);
    }
    size_t w = n < cap - 1 ? n : cap - 1;
    memcpy(out, tmp, w);
    out[w] = '\0';
    return w;
}

uint32_t odid_distance_m(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7) {
    int64_t e, n;
    displacement_cm(lat1_e7, lon1_e7, lat2_e7, lon2_e7, &e, &n);
    uint64_t sq = static_cast<uint64_t>(e * e) + static_cast<uint64_t>(n * n);
    uint64_t cm = isqrt_u64(sq);
    return static_cast<uint32_t>((cm + 50) / 100);
}

uint16_t odid_bearing_deg(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7) {
    int64_t e, n;
    displacement_cm(lat1_e7, lon1_e7, lat2_e7, lon2_e7, &e, &n);
    if (e == 0 && n == 0) return 0;
    uint64_t ae = e < 0 ? -e : e;
    uint64_t an = n < 0 ? -n : n;
    // Angle within first octant pair: atan(min/max) in millidegrees.
    uint64_t hi = ae > an ? ae : an;
    uint64_t lo = ae > an ? an : ae;
    uint32_t pos = static_cast<uint32_t>((lo * 64 * 256) / hi);  // Q8 index into table
    uint32_t i = pos >> 8;
    uint32_t f = pos & 0xFF;
    uint32_t a = kAtan64[i];
    uint32_t b = i < 64 ? kAtan64[i + 1] : a;
    uint32_t t = a + (((b - a) * f) >> 8);  // atan(lo/hi), millidegrees, 0..45000
    // Angle from the north axis in the first quadrant (mdeg).
    uint32_t q = (ae > an) ? 90000 - t : t;
    uint32_t brg;
    if (e >= 0 && n >= 0)
        brg = q;
    else if (e >= 0 && n < 0)
        brg = 180000 - q;
    else if (e < 0 && n < 0)
        brg = 180000 + q;
    else
        brg = 360000 - q;
    uint32_t deg = (brg + 500) / 1000;
    if (deg >= 360) deg -= 360;
    return static_cast<uint16_t>(deg);
}

int32_t odid_cos_deg_q15(int32_t deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    if (deg <= 90) return kCosQ15[deg];
    if (deg <= 180) return -static_cast<int32_t>(kCosQ15[180 - deg]);
    if (deg <= 270) return -static_cast<int32_t>(kCosQ15[deg - 180]);
    return kCosQ15[360 - deg];
}

}  // namespace ui::external_app::drone_detect
