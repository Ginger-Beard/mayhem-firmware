/*
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

/* Wi-Fi Remote ID frames heard on the HackRF itself (drone detect, PDRN DSSS
 * mode): what the M4 forwards to the M0 for one frame, and which frames
 * qualify. Shared by both cores and the host tests (header only, no
 * ChibiOS).
 *
 * PRIVACY (a design rule): only frames that carry Remote ID are forwarded,
 * and of those only the transmitter address and the Remote ID payload.
 * Everything else the receiver decodes (ordinary networks' beacons, probe
 * responses, action frames) is reduced to counters on the M4: no SSID, no
 * address, no payload leaves dd_wifi_rid_find(), in any mode. A frame
 * qualifies with a valid FCS (checked by the receiver) and:
 *   - beacon / probe response with a vendor IE of OUI FA:0B:BC (ASTM) or
 *     90:3A:E6 (Parrot), type 0x0D: payload = the IE body from the OUI on
 *     (OUI, type, message counter, message pack), as odid_find_wifi_ie()
 *     takes it;
 *   - NAN service discovery frame (public action, Wi-Fi Alliance OUI type
 *     0x13) to 51:6F:9A:01:00:00 with a service descriptor of the ODID
 *     service ID 88:69:19:9D:92:09: payload = its service info (message
 *     counter, message pack);
 *   - only with `legacy` (a bench switch, off by default): a beacon with the
 *     French (6A:5C:35) or legacy DJI (26:37:12) drone vendor IE: payload =
 *     the IE body, raw (no parser on the M0; shown on the debug console). */

#ifndef __DD_WIFI_RID_H__
#define __DD_WIFI_RID_H__

#include <cstddef>
#include <cstdint>

namespace dd {

/* One forwarded frame (the body of DroneDetectWifiMessage). Only the first
 * wire_size() bytes travel: the M4 pushes that prefix, the M0 reads no
 * further than payload_len. */
struct DroneDetectWifiRid {
    static constexpr uint8_t KIND_ODID_IE = 1; /* vendor IE body (OUI, type, counter, pack) */
    static constexpr uint8_t KIND_NAN = 2;     /* NAN service info (counter, pack) */
    static constexpr uint8_t KIND_FRENCH = 3;  /* legacy, raw IE body */
    static constexpr uint8_t KIND_DJI = 4;     /* legacy, raw IE body */
    /* An IE body and a NAN service info have one length byte: <= 255. */
    static constexpr size_t PAYLOAD_MAX = 255;

    uint32_t t_ms;       /* baseband time of the frame's end */
    int16_t rssi_cdb;    /* channel power estimate (slice power + 4 dB), centi-dB re int8 full scale */
    uint8_t channel;     /* Wi-Fi channel number of the dwell */
    int8_t slice_mhz;    /* slice centre - channel centre, MHz */
    uint8_t frame_type;  /* frame control byte 0: 0x80 beacon, 0x50 probe response, 0xD0 action */
    uint8_t kind;        /* KIND_* */
    uint8_t addr[6];     /* transmitter address (addr2), first byte on air first */
    uint8_t counter;     /* ODID message counter (0 for the legacy kinds) */
    uint8_t payload_len; /* bytes in payload */
    uint8_t payload[PAYLOAD_MAX];

    static constexpr size_t header_size() { return 18; }
    size_t wire_size() const { return header_size() + payload_len; }
};
static_assert(offsetof(DroneDetectWifiRid, payload) == DroneDetectWifiRid::header_size(), "DroneDetectWifiRid: payload last, after an 18 byte header");

/* Frame control byte 0 of the frames the DSSS receiver keeps. */
constexpr uint8_t kWifiFcBeacon = 0x80;
constexpr uint8_t kWifiFcProbeResp = 0x50;
constexpr uint8_t kWifiFcAction = 0xD0;

constexpr size_t kWifiHeader = 24;   /* management frame header */
constexpr size_t kWifiFixed = 12;    /* beacon / probe response fixed fields */
constexpr size_t kWifiFcs = 4;       /* trailing frame check sequence */
constexpr size_t kOdidMsgBytes = 25; /* one ODID message */

namespace wifi_detail {

inline bool same(const uint8_t* a, const uint8_t* b, size_t n) {
    for (size_t i = 0; i < n; i++)
        if (a[i] != b[i]) return false;
    return true;
}

inline void copy_payload(DroneDetectWifiRid& out, const uint8_t* p, size_t n) {
    if (n > DroneDetectWifiRid::PAYLOAD_MAX) n = DroneDetectWifiRid::PAYLOAD_MAX;
    for (size_t i = 0; i < n; i++) out.payload[i] = p[i];
    out.payload_len = static_cast<uint8_t>(n);
}

/* Beacon / probe response: the ODID vendor IE (or, with legacy, a French or
 * DJI one; ODID wins when both are there). */
inline uint8_t find_ie(const uint8_t* body, size_t len, bool legacy, DroneDetectWifiRid& out) {
    static constexpr uint8_t kAstm[3] = {0xFA, 0x0B, 0xBC}, kParrot[3] = {0x90, 0x3A, 0xE6};
    static constexpr uint8_t kFrench[3] = {0x6A, 0x5C, 0x35}, kDji[3] = {0x26, 0x37, 0x12};
    const uint8_t* legacy_ie = nullptr;
    size_t legacy_len = 0;
    uint8_t legacy_kind = 0;
    size_t off = 0;
    while (off + 2 <= len) {
        const uint8_t id = body[off];
        const size_t n = body[off + 1];
        const uint8_t* ie = body + off + 2;
        if (off + 2 + n > len) break; /* malformed: stop */
        if (id == 0xDD && n >= 4) {
            if ((same(ie, kAstm, 3) || same(ie, kParrot, 3)) && ie[3] == 0x0D && n >= 5 + kOdidMsgBytes) {
                out.kind = DroneDetectWifiRid::KIND_ODID_IE;
                out.counter = ie[4];
                copy_payload(out, ie, n);
                return out.kind;
            }
            if (legacy && !legacy_ie) {
                if (same(ie, kFrench, 3)) legacy_kind = DroneDetectWifiRid::KIND_FRENCH;
                if (same(ie, kDji, 3)) legacy_kind = DroneDetectWifiRid::KIND_DJI;
                if (legacy_kind) {
                    legacy_ie = ie;
                    legacy_len = n;
                }
            }
        }
        off += 2 + n;
    }
    if (!legacy_ie) return 0;
    out.kind = legacy_kind;
    out.counter = 0;
    copy_payload(out, legacy_ie, legacy_len);
    return out.kind;
}

/* NAN service discovery frame body (after the 24 byte header): public
 * action 4 / 9, OUI 50:6F:9A type 0x13, then attributes (id, length LE16,
 * body); the service descriptor (id 3) of the ODID service carries the
 * message counter and pack in its service info. Optional fields of the
 * descriptor by its control byte: binding bitmap (b6, 2 bytes), matching
 * filter (b2), service response filter (b3), service info (b4), each but the
 * first with a length byte. */
inline uint8_t find_nan(const uint8_t* body, size_t len, DroneDetectWifiRid& out) {
    static constexpr uint8_t kNanHead[6] = {0x04, 0x09, 0x50, 0x6F, 0x9A, 0x13};
    static constexpr uint8_t kOdidService[6] = {0x88, 0x69, 0x19, 0x9D, 0x92, 0x09};
    if (len < sizeof(kNanHead) || !same(body, kNanHead, sizeof(kNanHead))) return 0;
    size_t off = sizeof(kNanHead);
    while (off + 3 <= len) {
        const uint8_t id = body[off];
        const size_t n = body[off + 1] | (body[off + 2] << 8);
        const uint8_t* a = body + off + 3;
        if (off + 3 + n > len) break;
        if (id == 0x03 && n >= 9 && same(a, kOdidService, 6)) {
            const uint8_t control = a[8];
            size_t at = 9;
            if (control & 0x40) at += 2;
            if (control & 0x04) at = at < n ? at + 1 + a[at] : n;
            if (control & 0x08) at = at < n ? at + 1 + a[at] : n;
            if (!(control & 0x10) || at >= n) return 0;
            const size_t si = a[at];
            if (at + 1 + si > n || si < 1 + kOdidMsgBytes) return 0;
            out.kind = DroneDetectWifiRid::KIND_NAN;
            out.counter = a[at + 1];
            copy_payload(out, a + at + 1, si);
            return out.kind;
        }
        off += 3 + n;
    }
    return 0;
}

} /* namespace wifi_detail */

/* mpdu: a whole MPDU with a valid FCS (len includes the FCS), as the DSSS
 * receiver delivers it. Fills out's kind, frame_type, addr, counter and
 * payload and returns the kind when the frame carries Remote ID (see the top
 * of this file), else returns 0 and leaves out untouched. */
inline uint8_t dd_wifi_rid_find(const uint8_t* mpdu, size_t len, bool legacy, DroneDetectWifiRid& out) {
    static constexpr uint8_t kNanCluster[6] = {0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00};
    if (len < kWifiHeader + kWifiFcs) return 0;
    const uint8_t fc = mpdu[0];
    const size_t end = len - kWifiFcs;
    uint8_t kind = 0; /* find_*() write into out only when they find one */
    if (fc == kWifiFcBeacon || fc == kWifiFcProbeResp) {
        if (end < kWifiHeader + kWifiFixed) return 0;
        kind = wifi_detail::find_ie(mpdu + kWifiHeader + kWifiFixed, end - kWifiHeader - kWifiFixed, legacy, out);
    } else if (fc == kWifiFcAction && wifi_detail::same(mpdu + 4, kNanCluster, 6)) {
        kind = wifi_detail::find_nan(mpdu + kWifiHeader, end - kWifiHeader, out);
    }
    if (!kind) return 0;
    out.frame_type = fc;
    for (size_t i = 0; i < 6; i++) out.addr[i] = mpdu[10 + i];
    return kind;
}

} /* namespace dd */

#endif /*__DD_WIFI_RID_H__*/
