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

#include "proc_drone_detect.hpp"
#include "portapack_shared_memory.hpp"

#include "event_m4.hpp"
#include "audio_dma.hpp"
#include "portapack_dma.hpp"
#include "hal.h"

#include <cstring>
#include <new>

/* In .bss rather than in the (heap allocated) processor, so the linker's
 * memory report accounts for it. */
static dd::GfskDemod gfsk_demod;

#ifdef PRALINE
/* Wi-Fi Remote ID receiver. Search on 2 phases + 2 interpolated, shedding to
 * 2 phases under load (dd_dsss.hpp kSearchPhases 3): the 4 phase search
 * measured 30 cycles a sample on this M4 (25 by instruction count), more
 * than the DSSS budget, so the input ring overflowed 2-30 times a second of
 * Wi-Fi dwell on the bench; mode 3 is ~1 dB behind it at the threshold
 * (host). 18.0 KB, 16 KB of it the input ring. */
using DsssDemodFw = dd::DsssDemod<dd::kDsssFirmwareSearch>;

/* Mode arena: the Wi-Fi Remote ID receiver (18.0 KB) and the pattern receiver
 * (5.9 KB) live here between DroneID dwells; a DroneID dwell takes the same
 * memory for the DroneID detector (23.8 KB, its 11680 sample snapshot most of
 * it) together with the two receivers' counters, and gives it back rebuilt
 * when it ends. Only one mode runs at a time, so the detector costs no static
 * RAM. Nothing outside the baseband thread touches the receivers; stage B
 * (event thread) reads the detector, so the mode is not left while it runs
 * (did_busy_). */
struct RxModes {
    DsssDemodFw dsss{};
    dd::PatternDemod pattern{};
};
struct DroneIdMode {
    dd::droneid::Detector det{};
    dd::PatternDemod::Stats pattern_stats{};
    DsssDemodFw::Stats dsss_stats{};
    int dsss_phases{0};
};
static_assert(sizeof(DroneIdMode) <= sizeof(RxModes), "the DroneID detector must fit in the Wi-Fi and pattern receivers' memory");
alignas(8) static uint8_t mode_arena[sizeof(RxModes)];
static_assert(alignof(RxModes) <= 8 && alignof(DroneIdMode) <= 8, "mode arena alignment");
static RxModes* rx_modes = nullptr;     /* the receivers, between DroneID dwells */
static DroneIdMode* did_mode = nullptr; /* the DroneID detector, during a DroneID dwell */

static dd::PatternDemod& pattern_rx() {
    return rx_modes->pattern;
}
static DsssDemodFw& dsss_rx() {
    return rx_modes->dsss;
}
/* The receivers' counters, also while a DroneID dwell has their memory. */
static const dd::PatternDemod::Stats& pattern_stats() {
    return did_mode ? did_mode->pattern_stats : rx_modes->pattern.stats();
}
static const DsssDemodFw::Stats& dsss_stats() {
    return did_mode ? did_mode->dsss_stats : rx_modes->dsss.stats();
}
static int dsss_phases() {
    return did_mode ? did_mode->dsss_phases : rx_modes->dsss.search_phases();
}
static void clear_rx_stats() {
    if (did_mode) {
        did_mode->pattern_stats = dd::PatternDemod::Stats{};
        did_mode->dsss_stats = DsssDemodFw::Stats{};
    } else {
        rx_modes->pattern.clear_stats();
        rx_modes->dsss.clear_stats();
    }
}

/* FCS valid frames for the event thread (two slots, wifi_head_/wifi_tail_). */
struct WifiSlot {
    dd::DsssFrame frame;
    uint32_t t_ms;
    uint8_t channel;
    int8_t slice;
    bool legacy;
};
static WifiSlot wifi_slot[2];
#else
/* HackRF One: the app is a "Requires HackRF Pro" stub and never starts PDRN,
 * whose image must still fit the H1's 32 KB M4 code region: no pattern mode,
 * no DSSS receiver, no DroneID detector. */
struct NoPatternDemod {
    using Callback = dd::PatternDemod::Callback;
    dd::PatternConfig cfg{};
    dd::PatternDemod::Stats st{};
    bool configure(const dd::PatternConfig& c) {
        cfg = c;
        return false;
    }
    void set_conjugate(bool) {}
    void clear_stats() {}
    const dd::PatternConfig& config() const { return cfg; }
    const dd::PatternDemod::Stats& stats() const { return st; }
    void execute(const int8_t*, size_t, Callback, void*) {}
};
static NoPatternDemod pattern_demod;
struct NoDsssDemod {
    dd::DsssDemod<dd::kDsssFirmwareSearch>::Stats st{};
    void configure(const dd::DsssTaps&, int, bool, bool, bool, int, const dd::DsssTrigger&, int = 0) {}
    int search_phases() const { return 0; }
    void clear_stats() {}
    const dd::DsssDemod<dd::kDsssFirmwareSearch>::Stats& stats() const { return st; }
    size_t push(const int8_t*, size_t) { return 0; }
    size_t backlog() const { return 0; }
    template <class E>
    void process(E&&, uint32_t, uint32_t, dd::DsssDemod<dd::kDsssFirmwareSearch>::Callback, void*) {}
};
static NoDsssDemod dsss_demod;
static NoPatternDemod& pattern_rx() {
    return pattern_demod;
}
static NoDsssDemod& dsss_rx() {
    return dsss_demod;
}
static const dd::PatternDemod::Stats& pattern_stats() {
    return pattern_demod.stats();
}
static const dd::DsssDemod<dd::kDsssFirmwareSearch>::Stats& dsss_stats() {
    return dsss_demod.stats();
}
static int dsss_phases() {
    return 0;
}
static void clear_rx_stats() {}
#endif

/* Working memory lent to the app: the RF detector's per point arrays and the
 * spec_an::Sweep (whose points this image's sweep engine writes anyway), about
 * 4.3 KB, then room for the app's map view (~10 KB, only while one is open):
 * M0 heap that the app, opened from a menu that stays underneath it, does not
 * have. The M4 itself never reads or writes it outside the sweep's Shared
 * block; its address goes out in every stats message. Layout:
 * DroneDetectStatsMessage::kScratchRf / kScratchSize. */
static uint8_t m0_scratch[DroneDetectStatsMessage::kScratchSize] __attribute__((aligned(8)));

DroneDetectProcessor::DroneDetectProcessor() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_cycles_ = DWT->CYCCNT;
    drops_base_ = shared_memory.m4_buffer_missed;
    missed_seen_ = drops_base_;
#ifdef PRALINE
    rx_modes = new (mode_arena) RxModes{};
#endif
}

void DroneDetectProcessor::execute(const buffer_c8_t& buffer) {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t start_lag = dma_lag_samples(buffer);

    if (config_pending_) apply_config();
    account_drops();

    if (sweep_engine.active()) {
        sweep_engine.execute(buffer);
    } else if (mode_ == DroneDetectConfigMessage::MODE_GFSK_BLE) {
        if (settle_left_) {
            settle_left_--;
            hop_skips_++;
        } else {
            gfsk_demod.execute(reinterpret_cast<const int8_t*>(buffer.p), buffer.count, packet_callback, this);
        }
    } else if (mode_ == DroneDetectConfigMessage::MODE_GFSK_PATTERN) {
        if (settle_left_) {
            settle_left_--;
            hop_skips_++;
        } else {
            pattern_rx().execute(reinterpret_cast<const int8_t*>(buffer.p), buffer.count, hit_callback, this);
        }
    } else if (mode_ == DroneDetectConfigMessage::MODE_DSSS) {
        if (settle_left_) {
            settle_left_--;
            hop_skips_++;
        } else {
            execute_dsss(buffer, start);
        }
    }
#ifdef PRALINE
    else if (mode_ == DroneDetectConfigMessage::MODE_ZWAVE) {
        if (settle_left_) {
            settle_left_--;
            hop_skips_++;
        } else {
            zwave_.feed(reinterpret_cast<const int8_t*>(buffer.p), buffer.count, zwave_callback, this);
        }
    } else if (mode_ == DroneDetectConfigMessage::MODE_METER) {
        if (settle_left_) {
            settle_left_--;
            hop_skips_++;
        } else {
            execute_meter(buffer);
        }
    }
#endif
    else if (mode_ == DroneDetectConfigMessage::MODE_DRONEID) {
        if (settle_left_) {
            settle_left_--;
            hop_skips_++;
            did_prev_ = nullptr;
        } else {
            execute_droneid(buffer);
        }
    }

    /* Deadline: the DMA completing the next buffer. Past it, its wake-up
     * finds this thread busy and that buffer is lost. */
    const uint8_t phase = sweep_engine.active()                                  ? PHASE_SWEEP
                          : mode_ == DroneDetectConfigMessage::MODE_GFSK_BLE     ? PHASE_BLE
                          : mode_ == DroneDetectConfigMessage::MODE_GFSK_PATTERN ? PHASE_PATTERN
                          : mode_ == DroneDetectConfigMessage::MODE_DSSS         ? PHASE_WIFI
                          : mode_ == DroneDetectConfigMessage::MODE_DRONEID      ? PHASE_DRONEID
                                                                                 : PHASE_IDLE;
    if (phase != phase_) {
        phase_ = phase;
        since_switch_ = 0;
    } else if (since_switch_ < 0xFF) {
        since_switch_++;
    }

    const uint32_t lag = dma_lag_samples(buffer);
    last_late_ = lag >= buffer.count;
    if (lag > max_lag_samples_) max_lag_samples_ = lag;
    if (start_lag > max_start_lag_samples_) max_start_lag_samples_ = start_lag;
    rate_khz_ = buffer.sampling_rate / 1000;
    if (since_hop_ < 0xFF) since_hop_++;

    const uint32_t cycles = DWT->CYCCNT - start;
    busy_bb_ += cycles;
    bb_total_ = bb_total_ + cycles;
    if (cycles > max_exec_cycles_) max_exec_cycles_ = cycles;
    if (cycles > slow_exec_cycles) slow_execs_++;
    keep_time();
}

/* The DMA fills a ring of 4 transfers of buffer.count samples; this buffer is
 * the one it completed last, so it is writing the next one now, and how far
 * it has got there is how long ago this buffer was complete. Inside this
 * buffer again: 3 transfers further. */
uint32_t DroneDetectProcessor::dma_lag_samples(const buffer_c8_t& buffer) {
    const int32_t ring_bytes = 4 * buffer.count * sizeof(*buffer.p);
    const uint32_t dst = LPC_GPDMA->CH[portapack::sgpio_gpdma_channel_number].DESTADDR;
    int32_t at = static_cast<int32_t>(dst - reinterpret_cast<uint32_t>(buffer.p));
    if (at < 0) at += ring_bytes;
    const uint32_t samples = static_cast<uint32_t>(at) / sizeof(*buffer.p);
    return (samples >= buffer.count) ? samples - buffer.count : samples + 3 * buffer.count;
}

/* New misses since the last buffer, by cause: this thread overran the
 * previous buffer's deadline, or a channel change was just made. The rest
 * (drops - late - hop) were caused outside execute(). And by phase: that of
 * the last execute(), or the switch window after a phase change. */
void DroneDetectProcessor::account_drops() {
    const uint16_t missed = shared_memory.m4_buffer_missed;
    const uint16_t delta = missed - missed_seen_;
    missed_seen_ = missed;
    if (!delta) return;
#ifdef PRALINE
    /* The DroneID detector needs a gap free stream: a capture across the
     * gap is dropped, and the buffer before this one is not the previous. */
    if (mode_ == DroneDetectConfigMessage::MODE_DRONEID && did_mode) {
        did_mode->det.gate.discontinuity();
        did_prev_ = nullptr;
    }
#endif
    if (since_switch_ < switch_window_buffers)
        drops_switch_ += delta;
    else
        drops_phase_[phase_] += delta;
    if (last_late_)
        drops_late_ += delta;
    else if (since_hop_ < hop_window_buffers)
        drops_hop_ += delta;
}

void DroneDetectProcessor::keep_time() {
    const uint32_t now = DWT->CYCCNT;
    const uint32_t delta = now - last_cycles_;
    last_cycles_ = now;

    ms_frac_cycles_ += delta;
    if (ms_frac_cycles_ >= cycles_per_ms) {
        t_ms_ += ms_frac_cycles_ / cycles_per_ms;
        ms_frac_cycles_ %= cycles_per_ms;
    }

    stats_cycles_ += delta;
    if (stats_cycles_ >= stats_period_cycles) {
        send_stats(stats_cycles_);
        stats_cycles_ = 0;
    }
}

template <typename T>
bool DroneDetectProcessor::push(const T& message) {
    /* Never waits: a full queue (the M0 busy) or the queue's lock held by the
     * event thread (sweep replies) costs the message, counted. */
    auto& q = shared_memory.application_queue;
    const size_t queued = q.len();
    if (queued > q_peak_) q_peak_ = static_cast<uint16_t>(queued);
    if (q.push(message)) return true;
    q_fail_++;
    constexpr size_t capacity = sizeof(shared_memory.application_queue_data);
    if (queued + sizeof(message) + 8 < capacity) qf_lock_++;
    return false;
}

void DroneDetectProcessor::send_stats(uint32_t elapsed) {
    const uint32_t ev = busy_ev_;
    const uint32_t busy = busy_bb_ + (ev - busy_ev_base_);
    busy_ev_base_ = ev;
    busy_bb_ = 0;

    const auto& s = gfsk_demod.stats();
    stats_message.aa_hits = s.aa_hits;
    stats_message.crc_bad = s.crc_bad;
    stats_message.ok = s.ok;
    stats_message.rid_ok = rid_ok_;
    stats_message.drops = shared_memory.m4_buffer_missed - drops_base_;
    stats_message.cpu_pct = static_cast<uint16_t>((static_cast<uint64_t>(busy) * 100) / elapsed);
    const bool pattern = mode_ == DroneDetectConfigMessage::MODE_GFSK_PATTERN;
    const bool wifi = mode_ == DroneDetectConfigMessage::MODE_DSSS;
    const bool droneid = mode_ == DroneDetectConfigMessage::MODE_DRONEID;
    stats_message.chan = pattern ? pattern_rx().config().tag : (wifi ? wifi_channel_ : (droneid ? did_tag_ : gfsk_demod.channel()));
    stats_message.mode = sweep_engine.active() ? DroneDetectConfigMessage::MODE_SWEEP : mode_;
    stats_message.triggers = s.triggers;
    stats_message.len_bad = s.len_bad;
    stats_message.max_exec_us = static_cast<uint16_t>(max_exec_cycles_ / cycles_per_us);
    const uint32_t khz = rate_khz_ ? rate_khz_ : 1;
    stats_message.max_lag_us = static_cast<uint16_t>((max_lag_samples_ * 1000) / khz);
    stats_message.max_start_us = static_cast<uint16_t>((max_start_lag_samples_ * 1000) / khz);
    stats_message.slow_execs = slow_execs_;
    stats_message.hop_skips = hop_skips_;
    stats_message.drops_hop = drops_hop_;
    stats_message.drops_late = drops_late_;
    stats_message.sync_capped = static_cast<uint16_t>(s.capped);
    stats_message.q_fail = q_fail_;
    stats_message.dup_suppressed = dup_suppressed_;
    stats_message.rate_capped = rate_capped_;
    stats_message.scratch = m0_scratch;
    stats_message.scratch_size = sizeof(m0_scratch);
    stats_message.drops_ble = drops_phase_[PHASE_BLE];
    stats_message.drops_sweep = drops_phase_[PHASE_SWEEP];
    stats_message.drops_switch = drops_switch_ + drops_phase_[PHASE_IDLE];
    stats_message.qf_rid = qf_rid_;
    stats_message.qf_stats = qf_stats_;
    stats_message.qf_lock = qf_lock_;
    stats_message.q_peak = q_peak_;
    stats_message.rid_fwd = rid_fwd_;
    const auto& ps = pattern_stats();
    stats_message.pat_cands = ps.cands;
    stats_message.pat_matches = ps.matches;
    stats_message.pat_hits = ps.hits;
    stats_message.pat_fwd = pat_fwd_;
    stats_message.pat_capped = static_cast<uint16_t>(ps.capped);
    stats_message.pat_rcap = pat_rcap_;
    stats_message.drops_pat = drops_phase_[PHASE_PATTERN];
    stats_message.vendor_dup = vendor_dup_;
    stats_message.vendor_ok = vendor_ok_;
    stats_message.vendor_fwd = vendor_fwd_;
    stats_message.other_fwd = other_fwd_;
    stats_message.other_dup = other_dup_;
    stats_message.other_qf = other_qf_;
#ifdef PRALINE
    const auto& zs = zwave_.stats();
    stats_message.zw_syncs = zs.syncs;
    stats_message.zw_ok = zs.ok;
    stats_message.zw_bad = static_cast<uint16_t>(zs.bad_len + zs.bad_sum);
    stats_message.zw_qf = zw_qf_;
#endif
    const auto& ws = dsss_stats();
    stats_message.wifi_triggers = ws.triggers;
    stats_message.wifi_frames = ws.sfd;
    stats_message.wifi_hdr_bad = ws.hdr_bad;
    stats_message.wifi_filtered = ws.filtered;
    stats_message.wifi_fcs_bad = ws.fcs_bad;
    stats_message.wifi_fcs_ok = ws.ok;
    stats_message.wifi_beacon = wifi_beacon_;
    stats_message.wifi_probe = wifi_probe_;
    stats_message.wifi_action = wifi_action_;
    stats_message.wifi_rid = wifi_rid_;
    stats_message.wifi_fwd = wifi_fwd_;
    stats_message.wifi_sync_lost = static_cast<uint16_t>(ws.sync_lost);
    stats_message.wifi_hdr_2m = static_cast<uint16_t>(ws.hdr_2m);
    stats_message.wifi_legacy = wifi_legacy_seen_;
    stats_message.wifi_dup = wifi_dup_;
    stats_message.wifi_rcap = wifi_rcap_;
    stats_message.wifi_busy = wifi_busy_;
    stats_message.wifi_qf = wifi_qf_;
    stats_message.wifi_overflows = static_cast<uint16_t>(ws.overflows);
    stats_message.wifi_backlog = wifi_backlog_max_;
    stats_message.wifi_max_step_us = static_cast<uint16_t>(wifi_step_max_ / cycles_per_us);
    stats_message.drops_wifi = drops_phase_[PHASE_WIFI];
    {
        uint16_t* const cyc[4] = {&stats_message.wifi_cyc_search, &stats_message.wifi_cyc_sync, &stats_message.wifi_cyc_lock, &stats_message.wifi_cyc_cfo};
        for (size_t k = 0; k < 4; k++) {
            const uint32_t c = wifi_grp_[k] ? wifi_cyc_[k] / wifi_grp_[k] : 0;
            *cyc[k] = static_cast<uint16_t>(c > 0xFFFF ? 0xFFFF : c);
            wifi_cyc_[k] = wifi_grp_[k] = 0;
        }
        stats_message.wifi_phases = static_cast<uint8_t>(dsss_phases());
        stats_message.wifi_shed = ws.g_shed;
    }
    {
        uint32_t cands = did_cands_, skips = did_skips_;
#ifdef PRALINE
        if (did_mode) {
            cands += did_mode->det.gate.candidates();
            skips += did_mode->det.gate.busy_skips();
        }
#endif
        stats_message.did_cands = cands;
        stats_message.did_skips = static_cast<uint16_t>(skips > 0xFFFF ? 0xFFFF : skips);
        /* event thread counters: read as they are (32 bit reads) */
        stats_message.did_evals = did_evals_;
        stats_message.did_dets = did_dets_;
        stats_message.did_capped = did_capped_;
        stats_message.did_qf = did_qf_;
        const uint32_t bmax = did_b_max_ / cycles_per_us, bn = did_b_n_;
        stats_message.did_b_max_us = static_cast<uint16_t>(bmax > 0xFFFF ? 0xFFFF : bmax);
        stats_message.did_b_mean_us = static_cast<uint16_t>(bn ? (did_b_sum_ / bn) / cycles_per_us : 0);
        did_b_max_ = did_b_sum_ = 0;
        did_b_n_ = 0;
        stats_message.drops_did = drops_phase_[PHASE_DRONEID];
    }
    wifi_backlog_max_ = 0;
    wifi_step_max_ = 0;
    max_exec_cycles_ = 0;
    max_lag_samples_ = 0;
    max_start_lag_samples_ = 0;
    q_peak_ = 0;
    if (!push(stats_message)) qf_stats_++;
}

void DroneDetectProcessor::apply_config() {
    const uint8_t mode = req_mode_;
#ifdef PRALINE
    /* A DroneID detector whose snapshot stage B is still testing (event
     * thread, a few ms) is neither reset nor taken down: the change waits
     * (config_pending_ stays set, no buffer is fed meanwhile). */
    if (did_mode && did_busy_) return;
#endif
    config_pending_ = false;
    const uint8_t channel = req_channel_;
    const uint8_t flags = req_flags_;
#ifdef PRALINE
    if (mode != DroneDetectConfigMessage::MODE_DRONEID && did_mode) leave_droneid();
    if (mode == DroneDetectConfigMessage::MODE_DRONEID && !did_mode) enter_droneid();
#endif

    pass_all_ = (flags & DroneDetectConfigMessage::FLAG_PASS_ALL) != 0;
    fwd_adv_ = (flags & DroneDetectConfigMessage::FLAG_FWD_ADV) != 0;
    target_on_ = mode == DroneDetectConfigMessage::MODE_GFSK_BLE && (flags & DroneDetectConfigMessage::FLAG_TARGET) != 0;
    if (target_on_) {
        const uint64_t t = (static_cast<uint64_t>(req_target_hi_) << 32) | req_target_lo_;
        for (size_t i = 0; i < 6; i++) target_[i] = static_cast<uint8_t>(t >> (8 * (5 - i)));
    }
    gfsk_demod.set_conjugate((flags & DroneDetectConfigMessage::FLAG_CONJUGATE) != 0);
#ifdef PRALINE
    if (rx_modes)
#endif
        pattern_rx().set_conjugate((flags & DroneDetectConfigMessage::FLAG_CONJUGATE) != 0);
    if (flags & DroneDetectConfigMessage::FLAG_RESET_STATS) {
        gfsk_demod.clear_stats();
        clear_rx_stats();
        did_cands_ = did_skips_ = 0;
        did_reset_ = true; /* the event thread's DroneID counters */
        pat_fwd_ = vendor_ok_ = vendor_fwd_ = 0;
        pat_rcap_ = vendor_dup_ = 0;
        vendor_dedup_.reset();
        other_dedup_.reset();
        other_fwd_ = 0;
        other_dup_ = other_qf_ = 0;
#ifdef PRALINE
        zwave_.clear_stats();
        zw_qf_ = 0;
#endif
        rid_ok_ = 0;
        drops_base_ = shared_memory.m4_buffer_missed;
        drops_hop_ = drops_late_ = 0;
        slow_execs_ = hop_skips_ = q_fail_ = 0;
        dup_suppressed_ = rate_capped_ = 0;
        for (auto& d : drops_phase_) d = 0;
        drops_switch_ = 0;
        qf_rid_ = qf_stats_ = qf_lock_ = 0;
        rid_fwd_ = 0;
        dedup_.reset();
        wifi_busy_ = 0;
        wifi_reset_ = true; /* the event thread's counters */
    }
    /* Every Wi-Fi dwell is a retune (channel and slice half). Conjugate (I/Q
     * swapped): the mirrored slice is the other half's, conjugated, whose
     * DBPSK decisions are the same: just the taps of -slice. */
    if (mode == DroneDetectConfigMessage::MODE_DSSS) {
        const bool conj = (flags & DroneDetectConfigMessage::FLAG_CONJUGATE) != 0;
        const dd::DsssTaps* taps = dd::dsss_taps(conj ? -req_slice_ : req_slice_);
        const uint8_t search = (flags & DroneDetectConfigMessage::SEARCH_MASK) >> DroneDetectConfigMessage::SEARCH_SHIFT;
        if (taps) dsss_rx().configure(*taps, 2, false, true, true, 3, dd::kDsssTriggerFirmware, search ? search + 1 : 0);
        wifi_channel_ = channel;
        wifi_slice_ = req_slice_;
        wifi_legacy_ = (flags & DroneDetectConfigMessage::FLAG_WIFI_LEGACY) != 0;
        settle_left_ = hop_settle_buffers;
        since_hop_ = 0;
        mode_ = taps ? mode : DroneDetectConfigMessage::MODE_IDLE;
        return;
    }
    /* Every pattern dwell comes with its own sync word and channel: a retune. */
    /* Every DroneID dwell is a retune (and a rate switch): a fresh detector,
     * whose floor builds up over the first 16 buffers (2 ms). */
    if (mode == DroneDetectConfigMessage::MODE_DRONEID) {
#ifdef PRALINE
        auto& gate = did_mode->det.gate;
        did_cands_ += gate.candidates();
        did_skips_ += gate.busy_skips();
        gate.reset();
        did_prev_ = nullptr;
        did_tag_ = channel;
        settle_left_ = did_settle_buffers;
        since_hop_ = 0;
        mode_ = mode;
#else
        mode_ = DroneDetectConfigMessage::MODE_IDLE;
#endif
        return;
    }
#ifdef PRALINE
    /* Z-Wave: a fresh receiver (a retune), its counters kept. */
    if (mode == DroneDetectConfigMessage::MODE_ZWAVE) {
        if (mode_ != mode) zwave_.reset();
        settle_left_ = hop_settle_buffers;
        since_hop_ = 0;
        mode_ = mode;
        return;
    }
    /* Track meter: a fresh boxcar and pulse detector (a retune or a
     * bandwidth change). */
    if (mode == DroneDetectConfigMessage::MODE_METER) {
        meter_.configure(channel);
        settle_left_ = hop_settle_buffers;
        since_hop_ = 0;
        mode_ = mode;
        return;
    }
#else
    /* The app is a stub off the HackRF Pro: no track meter or Z-Wave in
     * this image (its 32 KB). */
    if (mode == DroneDetectConfigMessage::MODE_ZWAVE || mode == DroneDetectConfigMessage::MODE_METER) {
        mode_ = DroneDetectConfigMessage::MODE_IDLE;
        return;
    }
#endif
    if (mode == DroneDetectConfigMessage::MODE_GFSK_PATTERN) {
        pattern_rx().configure(req_pattern_);
        settle_left_ = hop_settle_buffers;
        since_hop_ = 0;
        pat_window_count_ = 0;
        mode_ = mode;
        return;
    }
    /* A new channel means a retune: whatever is in the filters is stale, and
     * the synthesisers are still settling. */
    if (mode != mode_ || channel != gfsk_demod.channel()) {
        gfsk_demod.configure(channel);
        settle_left_ = hop_settle_buffers;
        since_hop_ = 0;
    }
    mode_ = mode;
}

#ifdef PRALINE
/* A checksum-valid Z-Wave frame: its header to the M0. */
void DroneDetectProcessor::zwave_callback(void* ctx, const dd::ZwaveFrame& f) {
    auto* self = static_cast<DroneDetectProcessor*>(ctx);
    auto& m = self->zw_message;
    m.t_ms = self->t_ms_;
    m.rssi_cdb = dd::PowerMeter::to_cdb(f.power);
    m.len = f.len;
    m.nhdr = f.nhdr;
    m.violations = f.violations;
    memcpy(m.hdr, f.hdr, sizeof(m.hdr));
    if (!self->push(m)) self->zw_qf_++;
}

/* Track meter (MODE_METER): every buffer through the boxcar; a report about
 * every 50 ms. */
void DroneDetectProcessor::execute_meter(const buffer_c8_t& buffer) {
    meter_.feed(reinterpret_cast<const int8_t*>(buffer.p), buffer.count);
    dd::MeterReport r;
    if (!meter_.take(r)) return;
    meter_message.t_ms = t_ms_;
    meter_message.peak_cdb = dd::PowerMeter::to_cdb(r.peak);
    meter_message.mean_cdb = dd::PowerMeter::to_cdb(r.mean);
    meter_message.floor_cdb = dd::PowerMeter::to_cdb(r.floor);
    meter_message.pulses = r.pulses;
    meter_message.clip = r.clip;
    meter_message.period_us = r.period_us;
    meter_message.last_period_us = r.last_period_us;
    meter_message.width_us = r.width_us;
    meter_message.log2d = meter_.log2d();
    push(meter_message);
}
#endif

void DroneDetectProcessor::packet_callback(void* context, const dd::BlePacket& packet) {
    static_cast<DroneDetectProcessor*>(context)->on_packet(packet);
}

/* Walks the AD structures for Service Data - 16 bit UUID 0xFFFA (ASTM Remote
 * ID) with application code 0x0D (Open Drone ID). Returns the AD structure
 * (at its length byte), or nullptr. */
const uint8_t* DroneDetectProcessor::find_remote_id(const uint8_t* adv, size_t len) {
    size_t i = 0;
    while (i + 1 < len) {
        const size_t ad_len = adv[i];
        if (ad_len == 0 || i + 1 + ad_len > len) break;
        if (adv[i + 1] == 0x16 && ad_len >= 4 &&
            adv[i + 2] == 0xFA && adv[i + 3] == 0xFF && adv[i + 4] == 0x0D)
            return adv + i;
        i += 1 + ad_len;
    }
    return nullptr;
}

void DroneDetectProcessor::on_packet(const dd::BlePacket& packet) {
    const uint8_t type = packet.pdu_type();
    const uint8_t* adv = packet.pdu + 8; /* header (2) + AdvA (6) */
    const size_t adv_len = packet.length - 6;

    /* AdvData only follows AdvA in these four. */
    const bool has_adv_data = type == dd::ADV_IND || type == dd::ADV_NONCONN_IND ||
                              type == dd::ADV_SCAN_IND || type == dd::SCAN_RSP;
    /* Track view, one advertiser: every advert of it (any PDU type), no
     * deduplication, nothing else; 20 per 100 ms at most. */
    if (target_on_) {
        for (size_t i = 0; i < 6; i++)
            if (packet.pdu[7 - i] != target_[i]) return;
        if (static_cast<uint32_t>(t_ms_ - tgt_window_start_) >= dd::kRateWindowMs) {
            tgt_window_start_ = t_ms_;
            tgt_window_count_ = 0;
        }
        if (tgt_window_count_ >= 20) return;
        tgt_window_count_++;
        send_adv(packet, adv, has_adv_data ? adv_len : 0, DroneDetectRidMessage::SOURCE_BLE);
        return;
    }
    const uint8_t* rid_ad = has_adv_data ? find_remote_id(adv, adv_len) : nullptr;
    const bool rid = rid_ad != nullptr;
    if (rid) rid_ok_++;
    if (!rid && !pass_all_) {
        if (!has_adv_data) return;
        if (forward_vendor(packet, adv, adv_len)) {
            send_adv(packet, adv, adv_len, DroneDetectRidMessage::SOURCE_BLE_VENDOR);
        } else if (fwd_adv_ && !vendor_seen_) {
            /* Any other advert (signature rules, survey log): one per address,
             * PDU type and payload a second, rate capped. */
            uint8_t addr[6];
            for (size_t i = 0; i < 6; i++) addr[i] = packet.pdu[7 - i];
            uint16_t sum = 0;
            for (size_t i = 0; i < adv_len; i++) sum = static_cast<uint16_t>(sum * 31 + adv[i]);
            if (other_dedup_.duplicate(addr, packet.pdu_type(), static_cast<uint8_t>(adv_len), sum, t_ms_)) {
                other_dup_++;
            } else if (!other_dedup_.rate_ok(t_ms_)) {
                other_qf_++;
            } else {
                send_adv(packet, adv, adv_len, DroneDetectRidMessage::SOURCE_BLE_OTHER);
            }
        }
        return;
    }

    /* A transmitter repeats each payload on several advertising events: forward
     * the first, not the repeats (pass-all shows everything). The AD structure
     * is length, 0x16, FA FF, 0D, counter, then the 25 byte ODID message whose
     * type is the high nibble of its first byte. */
    if (rid && !pass_all_ && rid_ad[0] >= 6) {
        uint16_t sum = 0;
        for (size_t i = 0; i < adv_len; i++) sum += adv[i];
        uint8_t addr[6];
        for (size_t i = 0; i < 6; i++) addr[i] = packet.pdu[7 - i];
        if (dedup_.duplicate(addr, rid_ad[6] >> 4, rid_ad[5], sum, t_ms_)) {
            dup_suppressed_++;
            return;
        }
    }
    if (!dedup_.rate_ok(t_ms_)) {
        rate_capped_++;
        return;
    }
    send_adv(packet, adv, adv_len, DroneDetectRidMessage::SOURCE_BLE);
}

/* A DJI advert (dd_ble_vendor.hpp): counted, and true when it is to be
 * forwarded (not a repeat within kVendorDedupWindowMs, under its rate cap). */
bool DroneDetectProcessor::forward_vendor(const dd::BlePacket& packet, const uint8_t* adv, size_t adv_len) {
    uint8_t addr[6];
    for (size_t i = 0; i < 6; i++) addr[i] = packet.pdu[7 - i];
    const uint8_t* name;
    size_t name_len;
    vendor_seen_ = false;
    if (!dd::dji_ble_match(packet.pdu[0], addr, adv, adv_len, &name, &name_len)) return false;
    vendor_seen_ = true; /* a DJI advert, forwarded or not: never an "other" one */
    vendor_ok_++;
    uint16_t sum = 0;
    for (size_t i = 0; i < adv_len; i++) sum += adv[i];
    if (vendor_dedup_.duplicate(addr, packet.pdu_type(), 0, sum, t_ms_)) {
        vendor_dup_++;
        return false;
    }
    if (!vendor_dedup_.rate_ok(t_ms_)) {
        rate_capped_++;
        return false;
    }
    return true;
}

void DroneDetectProcessor::send_adv(const dd::BlePacket& packet, const uint8_t* adv, size_t adv_len, uint8_t source) {
    /* Other adverts (survey, signature rules) never crowd out Remote ID,
     * DJI, pattern hits, Wi-Fi frames, sweep replies or stats: only while the
     * application queue is at most a quarter full. */
    if (source == DroneDetectRidMessage::SOURCE_BLE_OTHER &&
        shared_memory.application_queue.len() + sizeof(rid_message) > sizeof(shared_memory.application_queue_data) / 4) {
        other_qf_++;
        return;
    }
    rid_message.t_ms = t_ms_;
    rid_message.rssi_cdb = packet.rssi_cdb;
    rid_message.cfo_khz = packet.cfo_khz;
    rid_message.chan = packet.channel;
    rid_message.pdu_type = packet.pdu[0];
    rid_message.source = source;
    for (size_t i = 0; i < 6; i++) rid_message.addr[i] = packet.pdu[7 - i];
    rid_message.adv_len = adv_len;
    memcpy(rid_message.adv, adv, adv_len);
    if (push(rid_message)) {
        if (source == DroneDetectRidMessage::SOURCE_BLE_VENDOR)
            vendor_fwd_++;
        else if (source == DroneDetectRidMessage::SOURCE_BLE_OTHER)
            other_fwd_++;
        else
            rid_fwd_++;
    } else if (source == DroneDetectRidMessage::SOURCE_BLE_OTHER) {
        other_qf_++;
    } else {
        qf_rid_++;
    }
}

void DroneDetectProcessor::hit_callback(void* context, const dd::PatternHit& hit) {
    static_cast<DroneDetectProcessor*>(context)->on_hit(hit);
}

/* A pattern hit: the M0 counts them, so only the rate is capped here. */
void DroneDetectProcessor::on_hit(const dd::PatternHit& hit) {
    if (static_cast<uint32_t>(t_ms_ - pat_window_start_) >= dd::kRateWindowMs) {
        pat_window_start_ = t_ms_;
        pat_window_count_ = 0;
    }
    if (pat_window_count_ >= dd::kPatternRateMaxHits) {
        pat_rcap_++;
        return;
    }
    pat_window_count_++;
    hit_message.t_ms = t_ms_;
    hit_message.payload_hi = static_cast<uint32_t>(hit.payload >> 32);
    hit_message.payload_lo = static_cast<uint32_t>(hit.payload);
    hit_message.rssi_cdb = hit.rssi_cdb;
    hit_message.tag = hit.tag;
    hit_message.errors = hit.errors;
    hit_message.phase = hit.phase;
    hit_message.payload_bits = hit.payload_bits;
    if (push(hit_message))
        pat_fwd_++;
    else
        qf_rid_++;
}

/* ---------------------------------------------------------------- Wi-Fi */

/* The buffer into the receiver's ring, then steps while the budget lasts
 * (elapsed: cycles since this execute() began). The time between two calls
 * of elapsed() is one step: the longest is reported. */
void DroneDetectProcessor::execute_dsss(const buffer_c8_t& buffer, uint32_t start) {
    auto& dsss = dsss_rx();
    dsss.push(reinterpret_cast<const int8_t*>(buffer.p), buffer.count);
    const size_t backlog = dsss.backlog();
    if (backlog > wifi_backlog_max_) wifi_backlog_max_ = static_cast<uint16_t>(backlog);
    wifi_step_last_ = DWT->CYCCNT - start;
    {
        const auto& s = dsss.stats();
        wifi_seen_[0] = s.g_search;
        wifi_seen_[1] = s.g_sync;
        wifi_seen_[2] = s.g_lock;
        wifi_seen_[3] = s.cfo_passes;
    }
    dsss.process(
        [this, start, &dsss]() {
            const uint32_t e = DWT->CYCCNT - start;
            const uint32_t step = e - wifi_step_last_;
            if (step > wifi_step_max_) wifi_step_max_ = step;
            wifi_step_last_ = e;
            /* A step works in one state only: charge it to the one whose
             * count moved (the first call: nothing moved yet). */
            const auto& s = dsss.stats();
            const uint32_t now[4] = {s.g_search, s.g_sync, s.g_lock, s.cfo_passes};
            for (size_t k = 0; k < 4; k++) {
                const uint32_t d = now[k] - wifi_seen_[k];
                if (d) {
                    wifi_cyc_[k] += step;
                    wifi_grp_[k] += d;
                    break;
                }
            }
            for (size_t k = 0; k < 4; k++) wifi_seen_[k] = now[k];
            return e;
        },
        dsss_limit_cycles, dsss_step_bound_cycles, wifi_callback, this);
}

void DroneDetectProcessor::wifi_callback(void* context, const dd::DsssFrame& frame) {
    static_cast<DroneDetectProcessor*>(context)->on_wifi_frame(frame);
}

/* Baseband thread: an FCS valid beacon / probe response / action frame
 * into a free slot (by words: GCC would call newlib-nano's byte memcpy),
 * then wake the event thread. Both slots busy: dropped, counted. */
__attribute__((optimize("no-tree-loop-distribute-patterns"))) void DroneDetectProcessor::on_wifi_frame(const dd::DsssFrame& frame) {
#ifdef PRALINE
    const uint8_t head = wifi_head_;
    if (static_cast<uint8_t>(head - wifi_tail_) >= wifi_slots) {
        wifi_busy_++;
        return;
    }
    WifiSlot& s = wifi_slot[head % wifi_slots];
    const size_t words = (frame.length + 3u) / 4u;
    const uint32_t* src = reinterpret_cast<const uint32_t*>(frame.mpdu);
    uint32_t* dst = reinterpret_cast<uint32_t*>(s.frame.mpdu);
    for (size_t i = 0; i < words; i++) dst[i] = src[i];
    s.frame.length = frame.length;
    s.frame.rssi_cdb = frame.rssi_cdb;
    s.t_ms = t_ms_;
    s.channel = wifi_channel_;
    s.slice = wifi_slice_;
    s.legacy = wifi_legacy_;
    __asm__ volatile("" ::: "memory"); /* the slot before the index (one core: a compiler barrier) */
    wifi_head_ = head + 1;
    EventDispatcher::events_flag(EVT_MASK_SPECTRUM);
#else
    (void)frame;
#endif
}

/* Event thread: every waiting slot is classified, counted, forwarded if it
 * carries Remote ID, then wiped (no copy of an ordinary frame outlives
 * this). */
__attribute__((optimize("no-tree-loop-distribute-patterns"))) void DroneDetectProcessor::drain_wifi() {
    if (wifi_reset_) {
        wifi_reset_ = false;
        wifi_beacon_ = wifi_probe_ = wifi_action_ = wifi_rid_ = wifi_fwd_ = 0;
        wifi_legacy_seen_ = wifi_dup_ = wifi_rcap_ = wifi_qf_ = 0;
        wifi_dedup_.reset();
    }
#ifdef PRALINE
    while (wifi_tail_ != wifi_head_) {
        WifiSlot& s = wifi_slot[wifi_tail_ % wifi_slots];
        const uint8_t fc = s.frame.mpdu[0];
        if (fc == dd::kWifiFcBeacon)
            wifi_beacon_++;
        else if (fc == dd::kWifiFcProbeResp)
            wifi_probe_++;
        else
            wifi_action_++;
        auto& r = wifi_message.rid;
        const uint8_t kind = dd::dd_wifi_rid_find(s.frame.mpdu, s.frame.length, s.legacy, r);
        if (kind) {
            if (kind == dd::DroneDetectWifiRid::KIND_ODID_IE || kind == dd::DroneDetectWifiRid::KIND_NAN)
                wifi_rid_++;
            else
                wifi_legacy_seen_++;
            r.t_ms = s.t_ms;
            r.rssi_cdb = s.frame.rssi_cdb;
            r.channel = s.channel;
            r.slice_mhz = s.slice;
            uint16_t sum = 0;
            for (size_t i = 0; i < r.payload_len; i++) sum += r.payload[i];
            if (wifi_dedup_.duplicate(r.addr, kind, r.counter, sum, s.t_ms)) {
                wifi_dup_++;
            } else if (!wifi_dedup_.rate_ok(s.t_ms)) {
                wifi_rcap_++;
            } else if (shared_memory.application_queue.push_prefix(wifi_message, wifi_message.wire_size())) {
                wifi_fwd_++;
            } else {
                wifi_qf_++;
            }
        }
        uint32_t* w = reinterpret_cast<uint32_t*>(s.frame.mpdu);
        for (size_t i = 0; i < (s.frame.length + 3u) / 4u; i++) w[i] = 0;
        __asm__ volatile("" ::: "memory");
        wifi_tail_ = wifi_tail_ + 1;
    }
#endif
}

/* ---------------------------------------------------------------- DroneID */

/* The receivers' memory becomes the DroneID detector's; their counters go
 * with it. */
void DroneDetectProcessor::enter_droneid() {
#ifdef PRALINE
    const dd::PatternDemod::Stats ps = rx_modes->pattern.stats();
    const DsssDemodFw::Stats ds = rx_modes->dsss.stats();
    const int phases = rx_modes->dsss.search_phases();
    rx_modes->~RxModes();
    rx_modes = nullptr;
    did_mode = new (mode_arena) DroneIdMode{};
    did_mode->pattern_stats = ps;
    did_mode->dsss_stats = ds;
    did_mode->dsss_phases = phases;
    did_mode->det.gate.reset();
    did_prev_ = nullptr;
#endif
}

/* ... and back: the receivers rebuilt (their next dwell configures them),
 * counters restored. Never while stage B runs (apply_config). */
void DroneDetectProcessor::leave_droneid() {
#ifdef PRALINE
    did_cands_ += did_mode->det.gate.candidates();
    did_skips_ += did_mode->det.gate.busy_skips();
    const dd::PatternDemod::Stats ps = did_mode->pattern_stats;
    const DsssDemodFw::Stats ds = did_mode->dsss_stats;
    did_mode->~DroneIdMode();
    did_mode = nullptr;
    rx_modes = new (mode_arena) RxModes{};
    rx_modes->pattern.restore_stats(ps);
    rx_modes->dsss.restore_stats(ds);
    did_prev_ = nullptr;
#endif
}

/* Stage A on this buffer (the previous one read in place for a capture's
 * pre-trigger part); a candidate's frozen snapshot goes to the event thread.
 * Nothing is fed while a mode change waits for stage B. */
void DroneDetectProcessor::execute_droneid(const buffer_c8_t& buffer) {
#ifdef PRALINE
    const int8_t* const p = reinterpret_cast<const int8_t*>(buffer.p);
    if (!config_pending_ && buffer.count == dd::droneid::kBuffer) {
        if (did_mode->det.gate.feed(p, did_prev_)) {
            did_busy_ = true;
            EventDispatcher::events_flag(EVT_MASK_SPECTRUM);
        }
    }
    did_prev_ = p;
#else
    (void)buffer;
#endif
}

/* Event thread: stage B on the frozen snapshot (at most
 * did_evals_per_second a second), a detection to the M0, then the snapshot
 * goes back to stage A. */
void DroneDetectProcessor::stage_b() {
#ifdef PRALINE
    if (did_reset_) {
        did_reset_ = false;
        did_evals_ = did_dets_ = 0;
        did_capped_ = did_qf_ = 0;
    }
    if (!did_busy_ || !did_mode) return;
    auto& det = did_mode->det;
    const dd::droneid::Candidate c = det.gate.candidate();
    const uint32_t a = det.gate.snapshot_start();
    const uint32_t now = t_ms_;
    if (now - did_window_start_ >= 1000) {
        did_window_start_ = now;
        did_window_count_ = 0;
    }
    if (did_window_count_ >= did_evals_per_second) {
        did_capped_++;
    } else {
        did_window_count_++;
        const uint32_t t0 = DWT->CYCCNT;
        const uint32_t bb0 = bb_total_;
        const dd::droneid::Result r = det.stage_b.evaluate(det.gate.snapshot(), c.end - a, c.start - a, c.end - a);
        const uint32_t cyc = (DWT->CYCCNT - t0) - (bb_total_ - bb0);
        did_evals_++;
        if (cyc > did_b_max_) did_b_max_ = cyc;
        did_b_sum_ += cyc;
        did_b_n_++;
        if (r.det) {
            did_dets_++;
            const uint32_t d = c.end - c.start;
            const uint32_t pw = dd::droneid::power_sum(det.gate.snapshot() + 2 * (c.start - a), d) / d;
            did_message.t_ms = t_ms_;
            did_message.level_cdb = static_cast<int16_t>(dd::power_to_cdb(pw) - kDidFullScaleCdb);
            did_message.snr_cdb = static_cast<int16_t>(dd::power_to_cdb(c.mean_pw_q4) - dd::power_to_cdb(c.floor_q4));
            const float cfo = r.cfo_frac_hz;
            did_message.cfo_hz = static_cast<int16_t>(cfo > 32000.f ? 32000.f : (cfo < -32000.f ? -32000.f : cfo));
            did_message.zc_milli = static_cast<uint16_t>(r.zc * 1000.f + 0.5f);
            did_message.cp_milli = static_cast<uint16_t>(r.cp * 1000.f + 0.5f);
            did_message.dur_us = static_cast<uint16_t>((static_cast<uint64_t>(d) * 1000000u) / dd::droneid::kSampleRate);
            did_message.nsym = r.nsym;
            did_message.tag = did_tag_;
            if (!shared_memory.application_queue.push(did_message)) did_qf_++;
        }
    }
    det.gate.release();
    __asm__ volatile("" ::: "memory"); /* the release before the hand-off flag (one core: a compiler barrier) */
    did_busy_ = false;
#endif
}

void DroneDetectProcessor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::DroneDetectConfig: {
            const auto& m = *reinterpret_cast<const DroneDetectConfigMessage*>(message);
            sweep_engine.stop();
            baseband_thread.set_sampling_rate(m.sampling_rate ? m.sampling_rate : gfsk_fs);
            req_mode_ = m.mode;
            req_channel_ = m.channel;
            req_flags_ = m.flags;
            req_slice_ = m.slice_mhz;
            req_target_hi_ = m.sync_hi;
            req_target_lo_ = m.sync_lo;
            if (m.mode == DroneDetectConfigMessage::MODE_GFSK_PATTERN) {
                req_pattern_.sync = (static_cast<uint64_t>(m.sync_hi) << 32) | m.sync_lo;
                req_pattern_.sync_bits = m.sync_bits;
                req_pattern_.max_errors = m.max_errors;
                req_pattern_.payload_bits = m.payload_bits;
                req_pattern_.invert = (m.pat_flags & DroneDetectConfigMessage::PAT_INVERT) != 0;
                req_pattern_.rate = static_cast<dd::PatternRate>((m.pat_flags >> DroneDetectConfigMessage::PAT_RATE_SHIFT) & 3);
                req_pattern_.tag = m.channel;
            }
            config_pending_ = true;
            break;
        }

        case Message::ID::SpecAnConfig:
            sweep_engine.configure(*reinterpret_cast<const SpecAnConfigMessage*>(message), nullptr, baseband_thread);
            /* Leave GFSK mode, so the next DroneDetectConfig counts as a
             * mode change: demodulator reset, settle buffer skipped. */
            req_mode_ = DroneDetectConfigMessage::MODE_SWEEP;
            req_flags_ &= ~DroneDetectConfigMessage::FLAG_RESET_STATS;
            config_pending_ = true;
            break;

        /* Alert beeps of the app (baseband::request_audio_beep). */
        case Message::ID::AudioBeep: {
            const auto& m = *reinterpret_cast<const AudioBeepMessage*>(message);
            audio::dma::beep_start(m.freq, m.sample_rate, m.duration_ms);
            break;
        }

        case Message::ID::RequestSignal:
            if (reinterpret_cast<const RequestSignalMessage*>(message)->signal == RequestSignalMessage::Signal::BeepStopRequest)
                audio::dma::beep_stop();
            break;

        /* EVT_MASK_SPECTRUM: sweep FFTs due, Wi-Fi frames handed over, or a
         * DroneID candidate for stage B. The load counted is this thread's own
         * time: the baseband thread's while it preempted this one is not. */
        case Message::ID::UpdateSpectrum: {
            const uint32_t start = DWT->CYCCNT;
            const uint32_t bb0 = bb_total_;
            stage_b();
            drain_wifi();
            if (sweep_engine.active()) sweep_engine.process_pending();
            busy_ev_ = busy_ev_ + ((DWT->CYCCNT - start) - (bb_total_ - bb0));
            break;
        }

        default:
            break;
    }
}

int main() {
    audio::dma::init_audio_out(); /* for the alert beeps */
    EventDispatcher event_dispatcher{std::make_unique<DroneDetectProcessor>()};
    event_dispatcher.run();
    return 0;
}
