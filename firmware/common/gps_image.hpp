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

#ifndef __GPS_IMAGE_H__
#define __GPS_IMAGE_H__

/* The GPS app's baseband images on the SD card (HackRF Pro).
 *
 * The app's M0 code (~110 KB) is far larger than an external app's share of
 * m4_code, so the app stays in the firmware and loads its own baseband
 * images from the card into m4_code, as the .ppma loader does with a bundled
 * image, then starts them (baseband::run_prepared_image). One image per job
 * keeps each within the 64 KiB: L1 acquisition (snapshot search) and L1
 * tracking (the loops, aided and lost-channel searches); later images for
 * other tunings. They are build products of this firmware version, packed by
 * firmware/tools/make_gps_image.py, shipped in the .ppfw.tar's APPS folder
 * and copied to /APPS with the .ppma apps.
 *
 * File: a 32-byte header, then the image (the baseband's .bin, padded to a
 * multiple of 4 bytes). The app refuses an image of another firmware version
 * (VERSION_MD5, as the .ppma check), another job or a bad checksum. */

#include <cstdint>
#include <cstddef>

namespace gps_image {

constexpr uint32_t kMagic = 0x49535047u; /* "GPSI" */
constexpr uint16_t kHeaderVersion = 1;

enum Id : uint8_t {
    ID_NONE = 0,
    ID_L1A = 1, /* L1 acquisition: snapshot search (GPS C/A) */
    ID_L1T = 2, /* L1 tracking: loops, aided search, lost-channel search */
};

struct Header {
    uint32_t magic;
    uint16_t header_version;
    uint8_t id;
    uint8_t reserved0;
    uint32_t version_md5; /* the firmware's VERSION_MD5 */
    uint32_t size;        /* bytes of image after the header, a multiple of 4 */
    uint32_t checksum;    /* sum of the image's 32-bit words (little endian), mod 2^32 */
    char tag[4];          /* the baseband's tag (PGRA, PGRT) */
    uint32_t reserved[2];
};
static_assert(sizeof(Header) == 32, "the image header is 32 bytes");

inline uint32_t checksum_add(uint32_t sum, const void* p, size_t bytes) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i + 3 < bytes; i += 4)
        sum += static_cast<uint32_t>(b[i]) | (static_cast<uint32_t>(b[i + 1]) << 8) | (static_cast<uint32_t>(b[i + 2]) << 16) |
               (static_cast<uint32_t>(b[i + 3]) << 24);
    return sum;
}

/* path on the card (the APPS folder, next to the .ppma apps) */
inline const char16_t* path(Id id) {
    switch (id) {
        case ID_L1A:
            return u"APPS/GPS_L1A.BIN";
        case ID_L1T:
            return u"APPS/GPS_L1T.BIN";
        default:
            return u"";
    }
}

inline const char* name(Id id) {
    switch (id) {
        case ID_L1A:
            return "GPS_L1A";
        case ID_L1T:
            return "GPS_L1T";
        default:
            return "none";
    }
}

/* why an image was refused (for the screen and the log) */
enum Check : uint8_t {
    OK = 0,
    NO_CARD_OR_FILE, /* the file could not be opened */
    READ_ERROR,
    NOT_AN_IMAGE,  /* bad magic / header version */
    WRONG_JOB,     /* the id differs */
    WRONG_VERSION, /* another firmware's image */
    TOO_LARGE,     /* larger than m4_code */
    BAD_CHECKSUM,
};

inline const char* check_text(Check c) {
    static const char* const k[] = {"ok", "missing", "read error", "not an image", "wrong image", "other firmware version", "too large", "damaged"};
    return c <= BAD_CHECKSUM ? k[c] : "?";
}

/* the header against what the app wants; size_limit: m4_code's size */
inline Check check_header(const Header& h, Id want, uint32_t version_md5, uint32_t size_limit) {
    if (h.magic != kMagic || h.header_version != kHeaderVersion) return NOT_AN_IMAGE;
    if (h.id != want) return WRONG_JOB;
    if (h.version_md5 != version_md5) return WRONG_VERSION;
    if (h.size == 0 || (h.size & 3u) || h.size > size_limit) return TOO_LARGE;
    return OK;
}

}  // namespace gps_image

#endif /*__GPS_IMAGE_H__*/
