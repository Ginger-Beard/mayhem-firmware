#!/usr/bin/env python3
#
# Copyright (C) 2026 PortaPack Mayhem contributors
#
# This file is part of PortaPack.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#
"""gal_e1_codes.py: /GPS/GALE1.BIN, the Galileo E1-B / E1-C primary codes for the GPS app.

Usage: gal_e1_codes.py C7_E1B.txt C8_E1C.txt OUT.BIN [GNSS-SDR Galileo_E1.h]

Source: the electronic attachments of the Galileo Open Service Signal-In-Space Interface Control
Document (OS SIS ICD, Issue 2.1, November 2023, (c) European Union 2023), annexes C.7 (E1-B) and C.8
(E1-C) primary codes, published by the EU on the European GNSS Service Centre website. The ICD's
terms of use allow use for information, standardisation, research and development and commercial
purposes; reproduction must keep its terms and the copyright notice, and altering it needs written
permission. The codes are therefore NOT distributed with the firmware: each user makes this file from
the attachments downloaded from the GSC (one line per PRN: "E1B__Code_No_01;<1023 hex digits>").

Output (little endian): 16-byte header b"GALE1\\0", version 1, count 50, bytes per code 512 (uint16),
6 reserved; then per PRN 1-50: E1-B 512 bytes, E1-C 512 bytes: 4092 chips MSB first (the hex digits as
given, first chip = most significant bit), 4 zero bits of padding.
With the GNSS-SDR header (GPL; only read to cross-check, nothing copied) every code is compared.
"""

import re
import struct
import sys


def read_att(path, tag):
    codes = {}
    for ln in open(path, errors='replace'):
        m = re.match(r'\s*%s__Code_No_(\d+)\s*;\s*([0-9A-Fa-f]+)' % tag, ln)
        if m:
            codes[int(m.group(1))] = m.group(2).upper()
    return codes


def to_bytes(hexs):
    if len(hexs) != 1023:
        raise ValueError('a code must have 1023 hex digits (4092 chips), got %d' % len(hexs))
    bits = bin(int(hexs, 16))[2:].zfill(4092) + '0000'
    return int(bits, 2).to_bytes(512, 'big')


def gnss_sdr_codes(path):
    """the E1B / E1C string arrays of GNSS-SDR's Galileo_E1.h, in order (cross-check only)"""
    txt = open(path, errors='replace').read()
    out = []
    for m in re.finditer(r'\{([^{}]*)\}', txt):
        strs = re.findall(r'"([0-9A-Fa-f]+)"', m.group(1))
        if len(strs) >= 50:
            # one array: each entry may be split over several adjacent literals
            entries = re.split(r'"\s*,', m.group(1))
            codes = [''.join(re.findall(r'"([0-9A-Fa-f]+)', e)).upper() for e in entries]
            codes = [c for c in codes if c]
            if len(codes) == 50:
                out.append(codes)
    return out


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    b = read_att(sys.argv[1], 'E1B')
    c = read_att(sys.argv[2], 'E1C')
    if sorted(b) != list(range(1, 51)) or sorted(c) != list(range(1, 51)):
        sys.exit('gal_e1_codes: expected PRN 1-50 in both files (E1B %d, E1C %d)' % (len(b), len(c)))
    if len(sys.argv) > 4:
        arrays = gnss_sdr_codes(sys.argv[4])
        ok = 0
        for arr in arrays:
            if arr == [b[p] for p in range(1, 51)]:
                ok |= 1
            if arr == [c[p] for p in range(1, 51)]:
                ok |= 2
        print('cross-check with GNSS-SDR: E1B %s, E1C %s' % ('identical' if ok & 1 else 'DIFFERENT',
                                                              'identical' if ok & 2 else 'DIFFERENT'))
        if ok != 3:
            sys.exit(1)
    out = bytearray(b'GALE1\0' + struct.pack('<BBH6x', 1, 50, 512))
    for p in range(1, 51):
        out += to_bytes(b[p]) + to_bytes(c[p])
    open(sys.argv[3], 'wb').write(out)
    print('%s: %d bytes (50 PRNs x E1B + E1C x 512 B, 16-byte header)' % (sys.argv[3], len(out)))


if __name__ == '__main__':
    main()
