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

#ifndef __SIGDET_DEMO_HPP__
#define __SIGDET_DEMO_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::signature_detect {

/* Bench test without a transmitter: two made-up Remote ID drones as raw BLE
 * AdvData (1E 16 FA FF 0D counter + one 25 byte ODID message), to be fed to
 * the same parser as received frames.
 *  - "A" circles 200 m around the centre (60 s per lap, 120 m), its operator
 *    stands at the centre (Live GNSS), fixed address.
 *  - "B" hovers 400 m to the north-east, operator at its take-off point; it
 *    changes its BLE address every 30 s (exercises the UAS ID merge). */
class DemoDrones {
   public:
    static constexpr size_t kFramesPerSecond = 4;
    static constexpr size_t kAdvLen = 31;

    void start(int32_t lat_e7, int32_t lon_e7) {
        lat_ = lat_e7;
        lon_ = lon_e7;
    }

    /* Frame index (0..kFramesPerSecond-1) of second now_s. Fills mac (MSB
     * first) and adv[kAdvLen]; returns the level to report in dB. */
    int8_t frame(uint32_t now_s, size_t index, uint8_t* mac, uint8_t* adv);

   private:
    int32_t lat_{0};
    int32_t lon_{0};
    uint8_t counter_[2][6]{};
};

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_DEMO_HPP__*/
