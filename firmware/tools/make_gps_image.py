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

"""Pack a GPS baseband image (.bin from objcopy) into the GPS app's card format.

Usage: make_gps_image.py IMAGE.bin TAG ID VERSION_MD5 OUT.BIN [MAX_SIZE]
  TAG          the baseband tag, 4 characters (PGRA, PGRT)
  ID           1 = L1 acquisition, 2 = L1 tracking (firmware/common/gps_image.hpp)
  VERSION_MD5  the firmware's VERSION_MD5, e.g. 0x1234abcd
  MAX_SIZE     refuse an image larger than this (m4_code), default 65536

Format (firmware/common/gps_image.hpp): a 32-byte little-endian header
(magic "GPSI", header version 1, id, firmware VERSION_MD5, image size,
checksum = sum of the image's 32-bit words), then the image padded to a
multiple of 4 bytes.
"""

import struct
import sys


def main():
    if len(sys.argv) < 6:
        sys.exit(__doc__)
    src, tag, ident, version, out = sys.argv[1:6]
    limit = int(sys.argv[6], 0) if len(sys.argv) > 6 else 65536
    if len(tag) != 4:
        sys.exit("make_gps_image: the tag must have 4 characters")
    data = open(src, "rb").read()
    data += b"\0" * (-len(data) % 4)
    if not data or len(data) > limit:
        sys.exit("make_gps_image: %s is %d bytes (limit %d)" % (src, len(data), limit))
    checksum = sum(struct.unpack("<%dI" % (len(data) // 4), data)) & 0xFFFFFFFF
    header = struct.pack("<IHBBIII4sII", 0x49535047, 1, int(ident, 0), 0, int(version, 0) & 0xFFFFFFFF,
                         len(data), checksum, tag.encode("ascii"), 0, 0)
    assert len(header) == 32
    with open(out, "wb") as f:
        f.write(header)
        f.write(data)
    print("%s: %s id %s, %d bytes, version %s, checksum 0x%08x" % (out, tag, ident, len(data), version, checksum))


if __name__ == "__main__":
    main()
