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

#ifndef __GPS_RX_RESULT_H__
#define __GPS_RX_RESULT_H__

#include <cstdint>

/* One PRN's acquisition result, written by the PGPR baseband (M4) into its
 * own RAM and read by the GPS app (M0) through GpsRxStatusMessage::results. */
namespace gps_rx {

struct Result {
    float metric{0};     /* peak / mean cell (detection: > threshold for the snapshot length) */
    float code_phase{0}; /* chips at the snapshot's first sample (refined if refined) */
    float doppler{0};    /* Hz, incl. the receiver clock offset */
    float cn0{0};        /* dB-Hz estimate */
    uint8_t prn{0};
    uint8_t searched{0};
    uint8_t detected{0};
    uint8_t refined{0};
};

}  // namespace gps_rx

#endif /*__GPS_RX_RESULT_H__*/
