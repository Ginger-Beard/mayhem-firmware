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

#include "sigdet_zwnet.hpp"

#include <cstring>

namespace ui::external_app::signature_detect {

/* G.9959 classic frame: home ID (4), source, frame control (2: routed, ack
 * request, low power, speed modified, header type | beaming, sequence),
 * length, destination, then for a routed frame a routing header (status,
 * repeaters << 4 | hop, repeater IDs), then the command class. */
bool zw_parse(const uint8_t* hdr, uint8_t nhdr, uint8_t len, ZwHeader& h) {
    h = ZwHeader{};
    if (nhdr < 9 || len < 10) return false;
    memcpy(h.home, hdr, 4);
    h.src = hdr[4];
    h.routed = (hdr[5] & 0x80) != 0;
    h.ackreq = (hdr[5] & 0x40) != 0;
    h.type = hdr[5] & 0x0F;
    h.seq = hdr[6] & 0x0F;
    h.len = len;
    h.dst = hdr[8];
    h.cc = -1;
    if (h.type != 1 && h.type != 2 && h.type != 3 && h.type != 8) return false;
    if (h.src == 0 || h.src > 232 || h.dst == 0) return false;
    size_t p = 9;
    if (h.routed) {
        if (nhdr < 11 || len < 12) return false;
        h.nrep = hdr[10] >> 4;
        h.hop = hdr[10] & 0x0F;
        if (h.nrep == 0 || h.nrep > 4) return false;
        if (h.hop > h.nrep && h.hop != 0x0F) return false;
        for (size_t i = 0; i < h.nrep && 11 + i < nhdr; i++) h.rep[i] = hdr[11 + i];
        p = 11 + h.nrep;
    }
    if (h.type != 3 && p + 1 < len && p < nhdr) h.cc = hdr[p];
    return true;
}

const char* zw_type_name(uint8_t type) {
    switch (type) {
        case 1:
            return "singlecast";
        case 2:
            return "multicast";
        case 3:
            return "ack";
        case 8:
            return "routed";
        default:
            return "?";
    }
}

uint8_t ZwNet::node_count() const {
    uint32_t n = 0;
    for (uint32_t w : nodes)
        for (; w; w &= w - 1) n++;
    return static_cast<uint8_t>(n > 255 ? 255 : n);
}

void ZwNets::clear() {
    for (auto& n : n_) n = ZwNet{};
}

ZwNet* ZwNets::find(const uint8_t* home) {
    for (auto& n : n_)
        if (n.used && memcmp(n.home, home, 4) == 0) return &n;
    return nullptr;
}

ZwNet* ZwNets::add(const ZwHeader& h, int16_t rssi_cdb, uint32_t now_ms, bool& became_confirmed) {
    became_confirmed = false;
    ZwNet* n = find(h.home);
    if (n && !n->confirmed && now_ms - n->last_ms > kZwConfirmMs) {
        *n = ZwNet{}; /* its single frame is too old to count */
        n = nullptr;
    }
    if (!n || !n->used) {
        ZwNet* spare = nullptr;
        for (auto& c : n_) {
            if (!c.used) {
                spare = &c;
                break;
            }
            /* an unconfirmed one first, the oldest; else a confirmed one gone quiet */
            const bool quiet = now_ms - c.last_ms > kZwConfirmMs;
            if (!c.confirmed || quiet) {
                if (!spare || (spare->confirmed && !c.confirmed) || c.last_ms < spare->last_ms) spare = &c;
            }
        }
        if (!spare) return nullptr;
        n = spare;
        *n = ZwNet{};
        n->used = true;
        memcpy(n->home, h.home, 4);
        n->first_ms = now_ms;
        n->rssi_max = rssi_cdb;
    }
    if (n->frames < 0xFFFF) n->frames++;
    n->last_ms = now_ms;
    if (rssi_cdb > n->rssi_max) n->rssi_max = rssi_cdb;
    n->nodes[h.src >> 5] |= 1u << (h.src & 31);
    if (h.dst != 0xFF) n->nodes[h.dst >> 5] |= 1u << (h.dst & 31);
    if (!n->confirmed && n->frames >= 2) {
        n->confirmed = true;
        became_confirmed = true;
    }
    return n;
}

} /* namespace ui::external_app::signature_detect */
