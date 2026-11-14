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

/* The GPS app's L1 baseband images (HackRF Pro), loaded by the app from the
 * SD card (common/gps_image.hpp). Receive only. One source, two images:
 *
 * GPS_L1A (PGRA, acquisition): on GpsRxConfig CMD_SEARCH capture K ms at
 * 2.048 Msps into a 2-bit snapshot, search the requested PRNs (gps_acq.hpp),
 * one work unit per DMA buffer, refine the detected ones; progress and noise
 * statistics go to the app in GpsRxStatus messages, the results to the M4
 * RAM block kept across the images (gps_keep.hpp) when the search is done.
 *
 * GPS_L1T (PGRT, tracking): on CMD_TRACK the detected PRNs of those results
 * go to the real-time tracking loops (gps_trk.hpp, up to max_ch channels),
 * which run on every DMA buffer and publish channels, parity-checked
 * subframes and once-a-second transmit-time measurements in the kept block
 * (gps_trk::Shared); CMD_ADD starts an aided search for a newly risen
 * satellite; lost channels search locally. CMD_GAL: an aided Galileo E1
 * search (gal_acq.hpp); a detection may go to a Galileo channel (gal_trk.hpp)
 * when the app allows and the channel pool has room. */

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "message.hpp"
#include "gps_acq.hpp"
#include "gps_keep.hpp"
#include "gps_image.hpp"
#if defined(BASEBAND_gps_l1t)
#include "gps_trk.hpp"
#include "gal_acq.hpp"
#endif

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
#if defined(BASEBAND_gps_l1t)
    static constexpr uint8_t kImage = gps_image::ID_L1T;
#else
    static constexpr uint8_t kImage = gps_image::ID_L1A;
#endif

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
        uint32_t age_ms{0};
        uint8_t add_prn{0}, add_windows{0};
        float add_code{0}, add_doppler{0};
        uint64_t add_at{0};
        uint8_t gal_max{0};
    } req_{};

    uint8_t state_{GpsRxStatusMessage::IDLE};
    uint32_t seq_{0};
    uint32_t last_status_{0};
    /* Time in samples of the sample clock (DMA buffers handled + missed): the
     * code phases are measured in that clock too, so a TCXO error cancels. */
    uint64_t samples_{0};
    uint16_t missed_seen_{0};
    GpsRxStatusMessage status_{};
    uint64_t results_sample_{0}; /* first sample of the result snapshot (this image's clock) */

#if defined(BASEBAND_gps_l1t)
    static constexpr uint32_t kVerifyTimeoutMs = 10000; /* unconfirmed hand-over: verify within 10 s or release */
    /* The tracking image's sample count starts here, so that the result
     * snapshot, taken before this image started, still has a positive index
     * (its age arrives with CMD_TRACK). */
    static constexpr uint64_t kSampleBase = static_cast<uint64_t>(gps_acq::kFs) * 600u;
    gps_trk::Tracker* trk_{nullptr};
    uint8_t max_ch_{8};
    uint32_t trk_cycles_sum_{0}, trk_cycles_max_{0}, trk_buffers_{0};
    void start_tracking(uint32_t prn_mask, uint32_t age_ms);
    void stop_tracking();
    /* the Galileo searcher (the app's aided searches, re-acquisitions of lost
     * Galileo channels): a unit only while the buffer's tracking + Galileo
     * cycles stay under the budget (of 204k per 1 ms buffer): the loops come first */
    static constexpr uint32_t kGalBudget = 180000;
    static constexpr uint32_t kGalUnit = 20000; /* the largest unit (~17k cycles: rf_test/gps m4bench_trk GALSEARCH), with margin */
    uint32_t gal_cycles_{0};                    /* the app's current search's M4 cycles (diagnostics) */
#else
    static constexpr uint8_t kSettleBuffers = 20; /* 20 ms skipped before a capture: LO / gain settling after enable */
    uint8_t settle_{0};
    uint32_t units_{0};
    uint64_t unit_cycles_sum_{0};
    uint16_t last_rms_x10_{0}, last_clip_{0};
    uint8_t last_k_{0};
    bool results_kept_{false}; /* this search's results copied to the kept block */
#endif

    void apply();
    void send_status();

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{gps_acq::kFs, this, baseband::Direction::Receive};
};

#endif /*__PROC_GPS_RX_H__*/
