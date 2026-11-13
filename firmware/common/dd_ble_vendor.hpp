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

/* Drone maker Bluetooth LE adverts that are not Open Drone ID (drone detect
 * app): DJI aircraft advertise over plain BLE 4 (a real DJI Mini 4 Pro sends
 * a scan response named "DJI-MINI4-Pro-98F8" from E4:7A:2C:AB:98:F9 on
 * channel 38, RFUAV recording via the sdrmm fixtures). That says "a DJI device
 * is near", with a model name and RSSI, but no position.
 *
 * The one table of what counts, used by both cores: the M4 (PDRN) decides
 * what to forward, the M0 app why it matched. Header only, no ChibiOS, host
 * testable; cheap enough to run on every CRC valid advert (a few compares
 * and one walk over at most 31 bytes). */

#ifndef __DD_BLE_VENDOR_H__
#define __DD_BLE_VENDOR_H__

#include <cstddef>
#include <cstdint>

namespace dd {

/* IEEE MA-L assignments to DJI companies (official registry
 * standards-oui.ieee.org/oui/oui.csv, retrieved 2026-11-11; no DJI entries
 * in MA-M or MA-S). Matched only for public addresses (TxAdd 0): a random
 * static address has its top two bits set and could start with E4, 34, ...
 * by chance. */
constexpr uint8_t kDjiOuis[][3] = {
    {0xE4, 0x7A, 0x2C}, /* SZ DJI TECHNOLOGY CO.,LTD (the Mini 4 Pro capture) */
    {0x60, 0x60, 0x1F}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x48, 0x1C, 0xB9}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x34, 0xD2, 0x62}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x58, 0xB8, 0x58}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x04, 0xA8, 0x5A}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x0C, 0x9A, 0xE6}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x8C, 0x58, 0x23}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x4C, 0x43, 0xF6}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x88, 0x29, 0x85}, /* SZ DJI TECHNOLOGY CO.,LTD */
    {0x9C, 0x5A, 0x8A}, /* DJI BAIWANG TECHNOLOGY CO LTD */
    {0xC8, 0xA1, 0x62}, /* DJI BAIWANG TECHNOLOGY CO LTD */
    {0xEC, 0x72, 0xF7}, /* DJI BAIWANG TECHNOLOGY CO LTD */
    {0x34, 0x91, 0xF0}, /* DJI BAIWANG TECHNOLOGY CO LTD */
    {0x24, 0x91, 0x23}, /* SZ DJI Ronin Technology Co., Ltd. (gimbals) */
    {0xF8, 0x40, 0x68}, /* SZ DJI Ronin Technology Co., Ltd. (gimbals) */
    {0x20, 0x1F, 0x55}, /* DJI Osmo Technology Co., Ltd. (cameras) */
};

/* Bluetooth SIG company identifier of DJI in Manufacturer Specific Data
 * (assigned_numbers/company_identifiers/company_identifiers.yaml, retrieved
 * 2026-11-11: "value: 0x08AA, name: 'SZ DJI TECHNOLOGY CO.,LTD'"; the only
 * DJI entry). On air little endian: AA 08. */
constexpr uint16_t kDjiCompanyId = 0x08AA;

/* Local name prefix (AD 0x08 / 0x09) of DJI aircraft: "DJI-MINI4-Pro-98F8"
 * is the only name with real-signal evidence; no published capture shows
 * Mavic / Spark / Air style BLE names, so none are guessed here. "DJI-" with
 * the hyphen: DJI's own pattern there (model, then the address' last bytes). */
constexpr char kDjiNamePrefix[] = "DJI-";

/* Why an advert matched (bits). */
enum : uint8_t {
    VENDOR_OUI = 0x01,  /* public AdvA in kDjiOuis */
    VENDOR_NAME = 0x02, /* local name starts with kDjiNamePrefix */
    VENDOR_CID = 0x04,  /* Manufacturer Specific Data with kDjiCompanyId */
};

/* hdr0: PDU header byte 0 (type in bits 0-3, TxAdd in bit 6); addr: AdvA,
 * most significant byte first; adv / len: AdvData. Returns VENDOR_* bits (0:
 * not DJI); name / name_len point at the local name inside adv when there
 * is one (whether it matched or not). AdvData only follows AdvA in
 * ADV_IND (0), ADV_NONCONN_IND (2), SCAN_RSP (4) and ADV_SCAN_IND (6). */
inline uint8_t dji_ble_match(uint8_t hdr0, const uint8_t* addr, const uint8_t* adv, size_t len, const uint8_t** name, size_t* name_len) {
    const uint8_t type = hdr0 & 0x0F;
    *name = nullptr;
    *name_len = 0;
    if (type != 0 && type != 2 && type != 4 && type != 6) return 0;
    uint8_t why = 0;
    if (!(hdr0 & 0x40)) {
#pragma GCC unroll 1
        for (const auto& o : kDjiOuis)
            if (addr[0] == o[0] && addr[1] == o[1] && addr[2] == o[2]) {
                why |= VENDOR_OUI;
                break;
            }
    }
    size_t i = 0;
    while (i + 1 < len) {
        const size_t ad_len = adv[i];
        if (ad_len == 0 || i + 1 + ad_len > len) break;
        const uint8_t ad_type = adv[i + 1];
        const uint8_t* const d = adv + i + 2;
        const size_t n = ad_len - 1;
        if (ad_type == 0x08 || ad_type == 0x09) {
            *name = d;
            *name_len = n;
            const size_t p = sizeof(kDjiNamePrefix) - 1;
            bool match = n >= p;
            for (size_t k = 0; match && k < p; k++) match = d[k] == static_cast<uint8_t>(kDjiNamePrefix[k]);
            if (match) why |= VENDOR_NAME;
        } else if (ad_type == 0xFF && n >= 2 && (d[0] | (d[1] << 8)) == kDjiCompanyId) {
            why |= VENDOR_CID;
        }
        i += 1 + ad_len;
    }
    return why;
}

} /* namespace dd */

#endif /*__DD_BLE_VENDOR_H__*/
