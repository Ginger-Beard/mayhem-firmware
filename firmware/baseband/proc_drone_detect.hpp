/*
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

#ifndef __PROC_DRONE_DETECT_H__
#define __PROC_DRONE_DETECT_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"

#include "spec_an_engine.hpp"
#include "dd_dsp.hpp"
#include "dd_meter.hpp"
#include "dd_fsk_rx.hpp"
#include "dd_dsss.hpp"
#include "dd_droneid.hpp"
#include "dd_ble_vendor.hpp"
#include "dd_wifi_rid.hpp"

#include "message.hpp"

#include <cstdint>
#include <cstddef>

/* Drone detect baseband (PDRN). One image, modes chosen at run time:
 * - GFSK / BLE (DroneDetectConfig, MODE_GFSK_BLE): 4 Msps, the channel tuned
 *   by the M0 through ReceiverModel (so at +fs/4 here), dd::GfskDemod. Remote ID
 *   adverts (or every CRC valid advert with FLAG_PASS_ALL) go to the M0 as
 *   DroneDetectRidMessage, counters every 500 ms as DroneDetectStatsMessage.
 *   Adverts that are not Remote ID but match the DJI table (dd_ble_vendor.hpp)
 *   go as DroneDetectRidMessage with SOURCE_BLE_VENDOR, thinned to about one
 *   a second per advert (presence only).
 * - GFSK / pattern (MODE_GFSK_PATTERN): the same 4 Msps front end with a sync
 *   word search, dd::PatternDemod; each hit goes to the M0 as
 *   DroneDetectPatternHitMessage (at most kPatternRateMaxHits per 100 ms).
 * - Wi-Fi Remote ID (MODE_DSSS): the same 4 Msps, the M0 tuned to one half of
 *   a 2.4 GHz Wi-Fi channel (slice -2 or +2 MHz, dd_dsss.hpp), dd::DsssDemod
 *   (802.11b 1 Mb/s). Its work per sample depends on its state, so each DMA
 *   buffer goes into its input ring and is demodulated in steps while this
 *   buffer's cycle budget lasts (kDsssLimitCycles): a SYNC burst is spread
 *   over the next buffers. FCS valid frames are handed to the event thread
 *   (two slots, no lock), which keeps only Remote ID (dd_wifi_rid.hpp: ODID
 *   vendor IE or NAN; French / legacy DJI IEs with FLAG_WIFI_LEGACY), counts
 *   the rest by type and forgets it, and queues DroneDetectWifiMessage (the
 *   used prefix only), at most kWifiRateMaxFrames per 100 ms.
 * - DJI DroneID (MODE_DRONEID, HackRF Pro): 15.36 Msps with the LO on a
 *   DroneID centre, dd::droneid::Detector (dd_droneid.hpp): stage A (energy,
 *   on-time, snapshot capture) on every buffer here, stage B (CP fold + ZC
 *   root 600 test, ~0.7 M cycles) on each candidate in the event thread, at
 *   most kDidEvalsPerSecond a second; a detection goes to the M0 as
 *   DroneDetectDroneIdMessage. Presence only, no decoding. Its memory is the
 *   pattern and Wi-Fi receivers' (mode arena in the .cpp): they are taken
 *   down for a DroneID dwell and rebuilt after it, counters kept.
 * - Sweep: entered with SpecAnConfig and served by spec_an::Engine exactly as
 *   in proc_wideband_spectrum, so the M0 drives it with spec_an::Sweep. Left
 *   with any DroneDetectConfig.
 * - Burst (later): room in the mode numbering, nothing here yet. */
class DroneDetectProcessor : public BasebandProcessor {
   public:
    DroneDetectProcessor();
    DroneDetectProcessor(const DroneDetectProcessor&) = delete;
    DroneDetectProcessor& operator=(const DroneDetectProcessor&) = delete;

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    static constexpr uint32_t gfsk_fs = dd::kGfskSampleRate;
    static constexpr uint32_t stats_period_cycles = 100000000; /* 500 ms at 200 MHz */
    static constexpr uint32_t cycles_per_ms = 200000;
    static constexpr uint32_t cycles_per_us = 200;
    static constexpr uint32_t slow_exec_cycles = 400 * cycles_per_us;
    /* Buffers not demodulated after a channel change: the M0 retunes just
     * before it sends the DroneDetectConfig, so this one holds the synthesisers'
     * settling and samples of the old channel. 512 us of a 1.2 s dwell. */
    static constexpr uint8_t hop_settle_buffers = 1;
    /* Drops seen this many buffers after a channel change count as the hop's. */
    static constexpr uint8_t hop_window_buffers = 16;
    /* Drops this many buffers after a phase change (BLE <-> sweep <-> idle, i.e. a
     * sample rate switch) count as the switch's. */
    static constexpr uint8_t switch_window_buffers = 8;
    /* DSSS dwell: no execute() runs past this (a step starts only if it can
     * end by then): 350 of the 512 us of a 2048 sample buffer, the rest left
     * to the DMA interrupt, the event thread's frame hand-off and the M0's
     * messages. Measured per 2048 samples: Search ~55 k cycles in mode 3
     * (~42 k on 2 phases, ~62 k on 4: too much), Lock ~60 k, so the backlog
     * a frame's Sync leaves drains slowly; mode 3 sheds to 2 phases while it
     * is long (dd_dsss.hpp kShedOn). Beyond that the backlog is dropped,
     * counted as wifi_overflows; DMA buffers are never missed. */
    static constexpr uint32_t dsss_limit_cycles = 350 * cycles_per_us;
    /* The longest demodulator step, measured on the HackRF Pro (DDSTAT
     * wifi_max_step_us, cyc_*): <= 29 us, the CFO pass (~5.3 k cycles); a
     * Search or Lock block of 32 groups ~3.5-4.4 k, 8 Sync groups ~3.4 k.
     * With margin: 32 us, so execute() stays within dsss_limit_cycles. */
    static constexpr uint32_t dsss_step_bound_cycles = 32 * cycles_per_us;
    /* FCS valid frames waiting for the event thread: a frame takes >= 1.4 ms
     * on air, the event thread a few k cycles per frame. */
    static constexpr uint8_t wifi_slots = 2;
    /* DroneID stage B: at most this many candidates tested a second (each
     * ~0.7 M cycles, ~0.9 M with the 20 % the M4 ran over the DSSS model:
     * <= 11 % of the M4). Household 2.4 GHz gives 8-11 a second, next to a
     * DJI O4 downlink 15-20, DJI enhanced Wi-Fi ~45 (prototype, bench). */
    static constexpr uint16_t did_evals_per_second = 24;
    /* Buffers not fed after the rate switch and retune into a DroneID dwell
     * (1 ms of a 4.5 s dwell). */
    static constexpr uint8_t did_settle_buffers = 8;
    /* power_to_cdb(128^2): a full scale int8 sample. */
    static constexpr int32_t kDidFullScaleCdb = 4214;
    enum Phase : uint8_t {
        PHASE_IDLE,
        PHASE_BLE,
        PHASE_SWEEP,
        PHASE_PATTERN,
        PHASE_WIFI,
        PHASE_DRONEID,
        PHASE_COUNT
    };

    /* Written by the event thread, applied by the baseband thread at the start
     * of its next buffer. */
    volatile bool config_pending_{false};
    uint8_t req_mode_{DroneDetectConfigMessage::MODE_IDLE};
    uint8_t req_channel_{37};
    uint8_t req_flags_{0};
    int8_t req_slice_{0};
    dd::PatternConfig req_pattern_{};

    uint8_t mode_{DroneDetectConfigMessage::MODE_IDLE};
    bool pass_all_{false};
    bool vendor_seen_{false}; /* forward_vendor(): the last advert matched the DJI table */
    bool fwd_adv_{false};
    uint32_t req_target_hi_{0}, req_target_lo_{0}; /* FLAG_FWD_ADV: other adverts too (SOURCE_BLE_OTHER) */

    /* Counters, baseband thread. */
    uint32_t rid_ok_{0};
    uint16_t drops_base_{0};
    uint16_t missed_seen_{0}; /* m4_buffer_missed at the last execute() */
    uint16_t drops_hop_{0};
    uint16_t drops_late_{0};
    uint16_t slow_execs_{0};
    uint16_t hop_skips_{0};
    uint16_t q_fail_{0};
    uint16_t dup_suppressed_{0};          /* Remote ID frames dropped as repeats */
    uint16_t rate_capped_{0};             /* frames dropped by the forward rate cap */
    uint16_t drops_phase_[PHASE_COUNT]{}; /* by Phase, outside the switch window */
    uint16_t drops_switch_{0};
    uint16_t qf_rid_{0};
    uint16_t qf_stats_{0};
    uint16_t qf_lock_{0};
    uint16_t q_peak_{0};
    uint32_t rid_fwd_{0};
    uint32_t pat_fwd_{0};
    uint16_t pat_rcap_{0};
    uint32_t pat_window_start_{0};
    uint32_t pat_window_count_{0};
    uint32_t vendor_ok_{0};
    uint32_t vendor_fwd_{0};
    uint16_t vendor_dup_{0};
    uint32_t other_fwd_{0};
    uint16_t other_dup_{0};
    uint16_t other_qf_{0};
    /* Wi-Fi (MODE_DSSS). Baseband thread: */
    uint8_t wifi_channel_{0};
    int8_t wifi_slice_{0};
    bool wifi_legacy_{false};
    uint16_t wifi_busy_{0};
    uint16_t wifi_backlog_max_{0};
    uint32_t wifi_step_max_{0};
    uint32_t wifi_step_last_{0};
    /* Cost by state (execute_dsss): cycles and groups (CFO: passes) since
     * the last stats message; demodulator counters at the last step. */
    uint32_t wifi_cyc_[4]{};
    uint32_t wifi_grp_[4]{};
    uint32_t wifi_seen_[4]{};
    volatile uint8_t wifi_head_{0}; /* slots filled (baseband thread) */
    volatile uint8_t wifi_tail_{0}; /* slots done (event thread) */
    /* ... event thread (read by send_stats): */
    uint32_t wifi_beacon_{0};
    uint32_t wifi_probe_{0};
    uint32_t wifi_action_{0};
    uint32_t wifi_rid_{0};
    uint32_t wifi_fwd_{0};
    /* DroneID (MODE_DRONEID). Baseband thread: */
    const int8_t* did_prev_{nullptr}; /* the previous DMA buffer of the dwell, read in place for the pre-trigger */
    uint8_t did_tag_{0};
    uint32_t did_cands_{0}; /* the detectors of earlier dwells (each dwell starts a fresh one) */
    uint32_t did_skips_{0};
    /* ... hand-off to the event thread: set here with the snapshot frozen,
     * cleared by the event thread after stage B; no leaving the mode while set. */
    volatile bool did_busy_{false};
    /* ... event thread: */
    uint32_t did_evals_{0};
    uint32_t did_dets_{0};
    uint16_t did_capped_{0};
    uint16_t did_qf_{0};
    uint32_t did_window_start_{0};
    uint16_t did_window_count_{0};
    uint32_t did_b_max_{0}; /* cycles, since the last stats message */
    uint32_t did_b_sum_{0};
    uint16_t did_b_n_{0};
    volatile bool did_reset_{false}; /* FLAG_RESET_STATS: the event thread clears its counters */
    uint16_t wifi_legacy_seen_{0};
    uint16_t wifi_dup_{0};
    uint16_t wifi_rcap_{0};
    uint16_t wifi_qf_{0};
    volatile bool wifi_reset_{false}; /* FLAG_RESET_STATS: the event thread clears its counters */
    uint8_t phase_{PHASE_IDLE};       /* of the last execute() */
    uint8_t since_switch_{0xFF};      /* buffers since the phase changed, saturating */
    uint8_t settle_left_{0};          /* buffers still to discard */
    uint8_t since_hop_{0xFF};         /* buffers since the last channel change, saturating */
    bool last_late_{false};           /* the last execute() ended past its deadline */
    uint32_t max_exec_cycles_{0};     /* since the last stats message */
    uint32_t max_lag_samples_{0};     /* ditto, at the end of execute() */
    uint32_t max_start_lag_samples_{0};
    uint32_t rate_khz_{0};

    /* Time keeping from the DWT cycle counter. */
    uint32_t last_cycles_{0};
    uint32_t ms_frac_cycles_{0};
    uint32_t t_ms_{0};
    uint32_t stats_cycles_{0};
    uint32_t busy_bb_{0};           /* cycles in execute(), baseband thread */
    volatile uint32_t bb_total_{0}; /* ... running total (wraps): the event thread subtracts the time it was preempted */
    volatile uint32_t busy_ev_{0};  /* cycles of sweep FFTs, event thread */
    uint32_t busy_ev_base_{0};

    spec_an::Engine sweep_engine{};

    DroneDetectRidMessage rid_message{};
    DroneDetectStatsMessage stats_message{};
    DroneDetectPatternHitMessage hit_message{};
    DroneDetectWifiMessage wifi_message{};   /* event thread */
    DroneDetectDroneIdMessage did_message{}; /* event thread */
    /* Track view: MODE_METER, and the BLE target (FLAG_TARGET). */
#ifdef PRALINE
    dd::PowerMeter meter_{};
    DroneDetectMeterMessage meter_message{};
#endif
    bool target_on_{false};
    uint8_t target_[6]{};
    uint32_t tgt_window_start_{0};
    uint32_t tgt_window_count_{0};
#ifdef PRALINE
    void execute_meter(const buffer_c8_t& buffer);
    /* FSK frame rules (MODE_FSK; Z-Wave R1 is one). */
    dd::FskRx fsk_{};
    dd::FskDwell req_fsk_{};
    DroneDetectFskMessage fsk_message{};
    uint16_t fsk_qf_{0};
    static void fsk_callback(void* ctx, const dd::FskFrame& f);
#endif
    dd::RidDedup dedup_{};
    dd::VendorDedup vendor_dedup_{};
    dd::OtherDedup other_dedup_{};
    dd::WifiDedup wifi_dedup_{}; /* event thread */

    void apply_config();
    void keep_time();
    void send_stats(uint32_t elapsed);
    void account_drops();
    template <typename T>
    bool push(const T& message);
    static uint32_t dma_lag_samples(const buffer_c8_t& buffer);
    void on_packet(const dd::BlePacket& packet);
    static void packet_callback(void* context, const dd::BlePacket& packet);
    void on_hit(const dd::PatternHit& hit);
    static void hit_callback(void* context, const dd::PatternHit& hit);
    bool forward_vendor(const dd::BlePacket& packet, const uint8_t* adv, size_t adv_len);
    void execute_dsss(const buffer_c8_t& buffer, uint32_t start);
    void on_wifi_frame(const dd::DsssFrame& frame);
    static void wifi_callback(void* context, const dd::DsssFrame& frame);
    void drain_wifi();
    void enter_droneid();
    void leave_droneid();
    void execute_droneid(const buffer_c8_t& buffer);
    void stage_b(); /* event thread */
    void send_adv(const dd::BlePacket& packet, const uint8_t* adv, size_t adv_len, uint8_t source);
    static const uint8_t* find_remote_id(const uint8_t* adv, size_t len);

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{gfsk_fs, this, baseband::Direction::Receive};
};

#endif /*__PROC_DRONE_DETECT_H__*/
