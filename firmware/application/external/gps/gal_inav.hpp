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

#ifndef __GAL_INAV_H__
#define __GAL_INAV_H__

/* Galileo E1-B I/NAV page decoding (header only, no ChibiOS: the host tests
 * run it). Written from the Galileo OS SIS ICD (issue 2.1, sections 4.1.4 and
 * 4.3: FEC K = 7 rate 1/2, G1 = 171o then G2 = 133o with G2 inverted, block
 * interleaver 30 columns x 8 rows, page part = 10-symbol synchronisation
 * pattern 0101100000 + 240 coded symbols = 250 symbols of 4 ms; a nominal
 * page = an even part (Data 1/2, 112 bits) and an odd part (Data 2/2 16 bits,
 * OSNMA, SAR, spare, CRC-24Q over 196 bits, SSP)). The 128-bit word (Data
 * 1/2 + 2/2) starts with its 6-bit word type.
 *
 * The tracking image writes one soft symbol per 4 ms epoch (E1-B prompt
 * after the pilot's secondary code is in sync) into a ring per channel; the
 * decoder finds the page boundary from two synchronisation patterns 250
 * symbols apart (either polarity: the data polarity is not known), then
 * de-interleaves and Viterbi-decodes every part, pairs even and odd parts
 * and checks the CRC. Its state and work memory live in RAM the tracking
 * image lends (Work), not on the app's heap or stack. */

#include <cstdint>
#include <cstring>

namespace gal_inav {

constexpr int kPart = 250; /* symbols per page part */
constexpr int kSync = 10;
constexpr int kCoded = 240;
constexpr int kBits = 120;               /* decoded bits per part, 6 of them tail */
constexpr uint16_t kSyncPattern = 0x160; /* 0101100000, first symbol in bit 9 */
constexpr uint8_t kG1 = 0171, kG2 = 0133;

inline int parity7(uint32_t x) {
    x &= 0x7Fu;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return static_cast<int>(x & 1u);
}

/* the encoder (tests; the decoder's branch outputs): register bit 6 = the
 * input bit, bits 5..0 the previous six (bit 5 the newest) */
inline void encode(const uint8_t* bits, int n, uint8_t* sym) {
    uint32_t s = 0;
    for (int k = 0; k < n; k++) {
        const uint32_t r = (static_cast<uint32_t>(bits[k] & 1u) << 6) | s;
        sym[2 * k] = static_cast<uint8_t>(parity7(r & kG1));
        sym[2 * k + 1] = static_cast<uint8_t>(parity7(r & kG2) ^ 1);
        s = r >> 1;
    }
}

/* the two output bits (G1 in bit 1, inverted G2 in bit 0) for register r = input << 6 | state */
struct BranchTable {
    uint8_t out[128];
    constexpr BranchTable()
        : out{} {
        for (int r = 0; r < 128; r++) {
            int a = r & kG1, b = r & kG2, pa = 0, pb = 0;
            for (int k = 0; k < 7; k++) pa ^= (a >> k) & 1, pb ^= (b >> k) & 1;
            out[r] = static_cast<uint8_t>((pa << 1) | (pb ^ 1));
        }
    }
};
constexpr BranchTable kBranch{};

/* transmission order -> encoder order: written in 30 columns of 8, read in rows */
inline int deinterleave_index(int p) {
    return (p % 30) * 8 + p / 30;
}

/* CRC-24Q (as GPS L5 / SBAS) over n bits, MSB first */
inline uint32_t crc24q(const uint8_t* bits, int n) {
    uint32_t crc = 0;
    for (int i = 0; i < n; i++) {
        const uint32_t fb = ((crc >> 23) & 1u) ^ (bits[i] & 1u);
        crc = (crc << 1) & 0xFFFFFFu;
        if (fb) crc ^= 0x864CFBu;
    }
    return crc;
}

/* what the app keeps per channel, and the Viterbi work, in lent RAM */
struct ChanState {
    uint32_t read;  /* symbols of the ring taken so far (the ring's write count) */
    uint32_t next;  /* with page sync: symbol count of the next part's first symbol */
    uint16_t epoch; /* the channel's start count the state belongs to */
    uint8_t prn;
    int8_t pol; /* +1 / -1: data polarity found with the synchronisation */
    uint8_t sync_ok;
    uint8_t have_even;
    uint8_t misses; /* synchronisation patterns missed in a row */
    uint16_t pages_ok, pages_bad, parts;
    uint8_t even[kBits]; /* the last even part's bits */
};

struct Work {
    static constexpr int kCh = 4;
    ChanState ch[kCh];
    int32_t metric[2][64];
    uint64_t surv[kBits]; /* decision bits per step and state */
    int8_t coded[kCoded];
    uint8_t bits[kBits];
    uint8_t page[2 * kBits];
};

/* Viterbi on 240 soft symbols in encoder order (> 0: bit 0), ending in state
 * 0 (the tail); 120 bits out */
inline void viterbi(Work& w, const int8_t* in, uint8_t* out) {
    int32_t* cur = w.metric[0];
    int32_t* nxt = w.metric[1];
    for (int s = 0; s < 64; s++) cur[s] = s ? -1000000 : 0;
    for (int k = 0; k < kBits; k++) {
        const int32_t x1 = in[2 * k], x2 = in[2 * k + 1];
        const int32_t bm[4] = {x1 + x2, x1 - x2, -x1 + x2, -x1 - x2}; /* by the expected bits (G1, G2) */
        uint32_t dlo = 0, dhi = 0;
        for (int ns = 0; ns < 64; ns++) {
            const uint32_t s0 = (static_cast<uint32_t>(ns) << 1) & 63u, r0 = ((static_cast<uint32_t>(ns) >> 5) << 6) | s0;
            const int32_t m0 = cur[s0] + bm[kBranch.out[r0]], m1 = cur[s0 | 1u] + bm[kBranch.out[r0 | 1u]];
            const bool b = m1 > m0;
            nxt[ns] = b ? m1 : m0;
            if (b) {
                if (ns < 32)
                    dlo |= 1u << ns;
                else
                    dhi |= 1u << (ns - 32);
            }
        }
        const uint64_t dec = (static_cast<uint64_t>(dhi) << 32) | dlo;
        w.surv[k] = dec;
        int32_t* t = cur;
        cur = nxt;
        nxt = t;
    }
    uint32_t s = 0;
    for (int k = kBits - 1; k >= 0; k--) {
        out[k] = static_cast<uint8_t>(s >> 5);
        const uint32_t b = static_cast<uint32_t>((w.surv[k] >> s) & 1u);
        s = ((s << 1) & 63u) | b;
    }
}

/* one decoded page: the 128-bit word (16 bytes), its type, and the
 * parts' fields the log shows */
struct Page {
    uint8_t prn;
    uint8_t word_type;
    uint8_t crc_ok;
    uint8_t alert;
    uint8_t word[16];
    uint32_t sym_end; /* the ring's symbol count at the end of the odd part */
};

inline uint32_t get_bits(const uint8_t* b, int at, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | (b[at + i] & 1u);
    return v;
}

/* GST week and time of week carried by word types 0 (time field 2), 5 and 6:
 * false for the others */
inline bool word_gst(const Page& p, uint32_t* wn, uint32_t* tow) {
    uint8_t b[128];
    for (int i = 0; i < 128; i++) b[i] = (p.word[i >> 3] >> (7 - (i & 7))) & 1u;
    switch (p.word_type) {
        case 0:
            if (get_bits(b, 6, 2) != 2) return false;
            *wn = get_bits(b, 96, 12);
            *tow = get_bits(b, 108, 20);
            return true;
        case 5:
            *wn = get_bits(b, 73, 12);
            *tow = get_bits(b, 85, 20);
            return true;
        case 6:
            *wn = 0xFFFF;
            *tow = get_bits(b, 105, 20);
            return true;
        default:
            return false;
    }
}

/* Take new symbols of one channel's ring (ring: n_ring symbols, `write`
 * symbols written in all, the channel's start count `epoch`) and decode at
 * most one page part. Returns true with *page set when a page (even + odd)
 * completed (CRC good or not). */
inline bool step(Work& w, int ch, uint8_t prn, const int8_t* ring, uint32_t n_ring, uint32_t write, uint16_t epoch, Page* page) {
    ChanState& c = w.ch[ch];
    if (c.prn != prn || c.epoch != epoch || write < c.read || write - c.read > n_ring) { /* new channel, restart or overrun */
        std::memset(static_cast<void*>(&c), 0, sizeof(c));
        c.prn = prn;
        c.epoch = epoch;
        c.read = write > n_ring - kPart ? write - (n_ring - kPart) : 0;
    }
    auto sym = [&](uint32_t i) { return ring[i % n_ring]; };
    auto sync_at = [&](uint32_t i, int* pol) { /* symbols i .. i + 9: the pattern, either polarity, <= 1 error */
                                               int e0 = 0, e1 = 0;
                                               for (int k = 0; k < kSync; k++) {
                                                   const int bit = sym(i + k) < 0 ? 1 : 0, want = (kSyncPattern >> (9 - k)) & 1;
                                                   e0 += bit != want;
                                                   e1 += bit == want;
                                               }
                                               if (e0 <= 1)
                                                   *pol = 1;
                                               else if (e1 <= 1)
                                                   *pol = -1;
                                               else
                                                   return false;
                                               return true;
    };
    if (!c.sync_ok) { /* look for two patterns 250 symbols apart */
        while (c.read + kPart + kSync <= write) {
            int p1 = 0, p2 = 0;
            const uint32_t i = c.read;
            c.read++;
            if (sym(i) == 0 || !sync_at(i, &p1) || !sync_at(i + kPart, &p2) || p1 != p2) continue;
            c.next = i;
            c.sync_ok = 1;
            c.pol = static_cast<int8_t>(p1);
            break;
        }
        if (!c.sync_ok) return false;
    }
    /* the part starting at c.next: complete once its 250 symbols and the next pattern are in */
    if (c.next + kPart + kSync > write) return false;
    const uint32_t s0 = c.next;
    int pol = 0;
    if (!sync_at(s0, &pol) || pol != c.pol) {
        if (++c.misses >= 2) { /* lost the page boundary */
            c.sync_ok = 0;
            c.read = s0 + 1;
            c.have_even = 0;
            return false;
        }
    } else {
        c.misses = 0;
    }
    for (int p = 0; p < kCoded; p++) {
        int v = sym(s0 + kSync + static_cast<uint32_t>(p)) * c.pol;
        w.coded[deinterleave_index(p)] = static_cast<int8_t>(v > 127 ? 127 : (v < -127 ? -127 : v));
    }
    viterbi(w, w.coded, w.bits);
    c.parts++;
    c.next = s0 + kPart;
    c.read = c.next;
    const int odd = w.bits[0];
    if (!odd) {
        std::memcpy(c.even, w.bits, kBits);
        c.have_even = 1;
        return false;
    }
    if (!c.have_even) return false;
    c.have_even = 0;
    /* CRC over even: E/O, type, data 1/2 (114 bits) + odd: E/O, type, data 2/2, OSNMA, SAR, spare (82 bits) */
    std::memcpy(w.page, c.even, 114);
    std::memcpy(w.page + 114, w.bits, 82);
    const uint32_t crc = crc24q(w.page, 196);
    const uint32_t got = get_bits(w.bits, 82, 24);
    page->prn = prn;
    page->crc_ok = crc == got ? 1 : 0;
    page->alert = c.even[1];
    std::memset(page->word, 0, sizeof(page->word));
    for (int i = 0; i < 112; i++) page->word[i >> 3] |= static_cast<uint8_t>((c.even[2 + i] & 1u) << (7 - (i & 7)));
    for (int i = 0; i < 16; i++) page->word[(112 + i) >> 3] |= static_cast<uint8_t>((w.bits[2 + i] & 1u) << (7 - ((112 + i) & 7)));
    page->word_type = static_cast<uint8_t>(page->word[0] >> 2);
    page->sym_end = s0 + kPart;
    if (page->crc_ok)
        c.pages_ok++;
    else
        c.pages_bad++;
    return true;
}

}  // namespace gal_inav

#endif /*__GAL_INAV_H__*/
