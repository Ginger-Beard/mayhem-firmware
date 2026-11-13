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

/* Z-Wave networks of the Signature Detect app (frames from PSDT MODE_ZWAVE):
 * header fields of a checksum-valid frame (never the payload), sanity checks
 * the 8 bit XOR checksum cannot give, and a small table of networks by home
 * ID: a network counts once 2 valid frames of it arrived within
 * kZwConfirmMs (a single corrupted frame can pass the checksum: seen once in
 * 61 on the recording). Integer only, no UI or ChibiOS (host testable). */

#ifndef __SIGDET_ZWNET_HPP__
#define __SIGDET_ZWNET_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::signature_detect {

constexpr uint32_t kZwConfirmMs = 120000;
constexpr size_t kZwNets = 4;

struct ZwHeader {
    uint8_t home[4];
    uint8_t src, dst, len;
    uint8_t type; /* header type: 1 singlecast, 2 multicast, 3 ack, 8 routed (frame control low nibble) */
    bool routed, ackreq;
    uint8_t seq;
    uint8_t nrep, hop; /* routed: repeaters, hop index */
    uint8_t rep[4];
    int16_t cc; /* command class byte, -1 none (acks, routed acks, out of the header bytes) */
};

/* Header fields of a frame (hdr: its first nhdr bytes, len its length
 * byte); false when they cannot be a frame (unknown header type, node 0,
 * a routing header out of range). */
bool zw_parse(const uint8_t* hdr, uint8_t nhdr, uint8_t len, ZwHeader& h);
const char* zw_type_name(uint8_t type);

struct ZwNet {
    uint8_t home[4];
    uint32_t nodes[8]; /* bit n: node n seen as source or destination */
    uint16_t frames;
    uint16_t frames_emit; /* at the last summary row */
    int16_t rssi_max;
    uint32_t first_ms, last_ms;
    bool used, confirmed, reported;
    uint8_t node_count() const;
};

class ZwNets {
   public:
    void clear();
    /* A valid frame: its network (created, or the oldest unconfirmed one
     * replaced); became_confirmed when this frame confirmed it. nullptr only
     * if the table is full of confirmed networks heard within kZwConfirmMs. */
    ZwNet* add(const ZwHeader& h, int16_t rssi_cdb, uint32_t now_ms, bool& became_confirmed);
    ZwNet* find(const uint8_t* home);

   private:
    ZwNet n_[kZwNets]{};
};

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_ZWNET_HPP__*/
