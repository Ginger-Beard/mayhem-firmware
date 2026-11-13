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

#ifndef __PROC_GPS_RX_H__
#define __PROC_GPS_RX_H__

/* PGPR: GPS L1 C/A receiver (HackRF Pro). Receive only.
 * On GpsRxConfig CMD_SEARCH: capture K ms at 2.048 Msps into a 2-bit
 * snapshot, then search the requested PRNs (gps_acq.hpp), one work unit per
 * DMA buffer, and refine the detected ones; progress and noise statistics go
 * to the app in GpsRxStatus messages, the results stay in M4 RAM.
 * With `track` set (or on CMD_TRACK) the confirmed PRNs are handed to the
 * real-time tracking loops (gps_trk.hpp, up to max_ch channels), which run
 * on every DMA buffer and publish channels, parity-checked subframes and
 * once-a-second transmit-time measurements in M4 RAM (gps_trk::Shared). */

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "message.hpp"
#include "gps_acq.hpp"
#include "gps_trk.hpp"

#include <cstdint>

class GpsRxProcessor : public BasebandProcessor {
   public:
    GpsRxProcessor();
    GpsRxProcessor(const GpsRxProcessor&) = delete;
    GpsRxProcessor& operator=(const GpsRxProcessor&) = delete;

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    static constexpr uint32_t kStatusCycles = 20000000; /* 100 ms at 200 MHz */

    /* Written by the event thread, applied by the baseband thread. */
    volatile bool pending_{false};
    struct Req {
        uint8_t cmd{GpsRxConfigMessage::CMD_STOP};
        uint8_t k_ms{20}, half_bins{12}, refine{1};
        uint32_t prn_mask{0};
        int16_t center[32]{};
        uint8_t confirm{1};
        uint8_t track{0};
        uint8_t max_ch{8};
    } req_{};

    static constexpr uint8_t kSettleBuffers = 20; /* 20 ms skipped before a capture: LO / gain settling after enable */
    uint8_t state_{GpsRxStatusMessage::IDLE};
    uint8_t settle_{0};
    uint32_t seq_{0};
    uint32_t units_{0};
    uint64_t unit_cycles_sum_{0};
    uint32_t last_status_{0};
    /* Time for the A -> B interval in samples of the sample clock (DMA buffers
     * handled + missed): the code phases are measured in that clock too, so a
     * TCXO error cancels in the confirmation rule. */
    uint64_t samples_{0};
    uint16_t missed_seen_{0};
    GpsRxStatusMessage status_{};
    /* tracking: the loops live in the snapshot's RAM while they run */
    gps_trk::Tracker* trk_{nullptr};
    uint8_t max_ch_{8};
    uint16_t last_rms_x10_{0}, last_clip_{0};
    uint8_t last_k_{0};
    uint32_t trk_cycles_sum_{0}, trk_cycles_max_{0}, trk_buffers_{0};

    void apply();
    void send_status();
    void start_tracking(uint32_t prn_mask);
    void stop_tracking();

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{gps_acq::kFs, this, baseband::Direction::Receive};
};

#endif /*__PROC_GPS_RX_H__*/
