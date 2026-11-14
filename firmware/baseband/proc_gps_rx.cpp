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

#include "proc_gps_rx.hpp"
#include "portapack_shared_memory.hpp"
#include "event_m4.hpp"
#include "hal.h"

#include <cstring>
#include <new>

/* The block kept across the GPS images (top of the M4 RAM, outside this
 * image's RAM): the RAM lent to the app, the search results, the tracking
 * output. Never cleared here: the app clears its part once. */
static gps_keep::Block& keep = *gps_keep::block();

#if defined(BASEBAND_gps_l1t)
static const gps_trk::CarrierTable carrier_table;
/* a fresh Tracker for each hand-over (it clears the shared block's counters) */
alignas(8) static uint8_t tracker_mem[sizeof(gps_trk::Tracker)];
#else
/* Big working memory, outside the processor object (bss). */
alignas(8) static gps_acq::Snapshot snapshot;
static gps_acq::Acquirer acquirer{snapshot};
static gps_acq::Session session{snapshot, acquirer};
#endif

GpsRxProcessor::GpsRxProcessor() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* the first status goes out with the first buffer: the app waits for it
     * after an image switch (Autonomous: the tracking image's CMD_TRACK) */
    last_status_ = DWT->CYCCNT - 6 * kStatusCycles;
    missed_seen_ = shared_memory.m4_buffer_missed;
    /* the tracking output starts empty with each image, as it did when it was
     * part of the image's own RAM */
    std::memset(static_cast<void*>(&keep.shared), 0, sizeof(keep.shared));
#if defined(BASEBAND_gps_l1t)
    samples_ = kSampleBase;
#endif
}

#if defined(BASEBAND_gps_l1t)

void GpsRxProcessor::start_tracking(uint32_t prn_mask, uint32_t age_ms) {
    const gps_acq::Result* res = keep.results;
    int n = 0;
    for (int i = 0; i < 32; i++)
        if (((prn_mask >> i) & 1u) && res[i].detected && !res[i].once) n++;
    if (!n) return;
    stop_tracking();
    trk_ = new (tracker_mem) gps_trk::Tracker(&keep.shared, &carrier_table);
    trk_->set_max_active(max_ch_);
    /* unconfirmed detections (no second snapshot): the loops verify them, and
     * one that never verifies gives its channel back (to a spare detection) */
    trk_->params().verify_timeout_ms = req_.confirm ? 0 : kVerifyTimeoutMs;
    /* code phases refer to the first sample of the result snapshot, taken by
     * the acquisition image age_ms ago; samples_ is the index of the next
     * buffer's first sample */
    const uint64_t now = samples_;
    const uint64_t age = static_cast<uint64_t>(age_ms) * (gps_acq::kFs / 1000u);
    const uint64_t at = age < now ? now - age : 0;
    results_sample_ = at;
    trk_->handover(res, prn_mask, at, now);
    keep.shared.handover_age_ms = static_cast<uint32_t>((now - at) * 1000u / gps_acq::kFs);
    keep.shared.handovers++;
    trk_->publish();
    trk_cycles_sum_ = trk_cycles_max_ = trk_buffers_ = 0;
    state_ = GpsRxStatusMessage::TRACKING;
    send_status();
}

void GpsRxProcessor::stop_tracking() {
    if (!trk_) return;
    trk_->reset(samples_);
    trk_->publish();
    trk_->~Tracker();
    trk_ = nullptr;
}

void GpsRxProcessor::apply() {
    pending_ = false;
    switch (req_.cmd) {
        case GpsRxConfigMessage::CMD_ADD: /* the loops keep running */
            if (trk_ && req_.add_prn >= 1 && req_.add_prn <= 32) {
                /* apply() runs before this buffer goes to the loops: it starts at samples_ - kN */
                trk_->start_aided(req_.add_prn, req_.add_code, req_.add_doppler, req_.add_at, samples_ - gps_acq::kN, req_.add_windows);
                trk_->publish();
            }
            return;
        case GpsRxConfigMessage::CMD_TRACK:
            max_ch_ = req_.max_ch ? req_.max_ch : 8;
            seq_++;
            start_tracking(req_.prn_mask, req_.age_ms);
            return;
        default: /* stop (a search is the acquisition image's job) */
            stop_tracking();
            state_ = GpsRxStatusMessage::IDLE;
            send_status();
            return;
    }
}

#else /* acquisition */

void GpsRxProcessor::apply() {
    pending_ = false;
    if (req_.cmd != GpsRxConfigMessage::CMD_SEARCH) { /* stop (tracking is the tracking image's job) */
        session.stop();
        state_ = GpsRxStatusMessage::IDLE;
        send_status();
        return;
    }
    seq_++;
    units_ = 0;
    unit_cycles_sum_ = 0;
    results_kept_ = false;
    gps_acq::Session::Params p;
    p.prn_mask = req_.prn_mask;
    std::memcpy(p.center, req_.center, sizeof(p.center));
    p.half_bins = req_.half_bins;
    p.k_ms = req_.k_ms;
    p.refine = req_.refine;
    p.confirm = req_.confirm;
    session.start(p);
    settle_ = kSettleBuffers;
    state_ = GpsRxStatusMessage::CAPTURING;
    send_status();
}

#endif

void GpsRxProcessor::send_status() {
    status_.state = state_;
    status_.image = kImage;
#if defined(BASEBAND_gps_l1t)
    status_.level = GpsRxStatusMessage::LEVEL_OK;
    status_.confirming = 0;
    status_.done_prn = status_.total_prn = 0;
    status_.k_ms = 0;
    status_.rms_x10 = static_cast<uint16_t>(keep.shared.level_x10);
    status_.clip_permille = keep.shared.clip_pm;
    status_.units = 0;
    status_.unit_cycles = 0;
    status_.dt_ms = 0;
#else
    status_.level = session.level();
    status_.confirming = session.confirming();
    status_.done_prn = static_cast<uint8_t>(acquirer.done_count());
    status_.total_prn = static_cast<uint8_t>(acquirer.requested());
    last_k_ = static_cast<uint8_t>(snapshot.k());
    last_rms_x10_ = static_cast<uint16_t>(snapshot.rms() * 10.0f);
    const uint32_t f = snapshot.filled();
    last_clip_ = static_cast<uint16_t>(f ? (snapshot.clipped() * 1000u) / f : 0);
    status_.k_ms = last_k_;
    status_.rms_x10 = last_rms_x10_;
    status_.clip_permille = last_clip_;
    status_.units = units_;
    status_.unit_cycles = units_ ? static_cast<uint32_t>(unit_cycles_sum_ / units_) : 0;
    status_.dt_ms = static_cast<int32_t>(session.dt() * 1000.0);
    {
        const int64_t at = static_cast<int64_t>(std::llround(session.t_results() * gps_acq::kFs)) - static_cast<int64_t>(gps_acq::kN);
        results_sample_ = at > 0 ? static_cast<uint64_t>(at) : 0;
    }
    if (state_ == GpsRxStatusMessage::DONE && !results_kept_) { /* the results outlive this image */
        std::memcpy(static_cast<void*>(keep.results), session.results(), sizeof(keep.results));
        results_kept_ = true;
    }
#endif
    status_.seq = seq_;
    status_.track = &keep.shared;
    status_.results = keep.results;
    status_.scratch = keep.scratch;
    status_.scratch_size = sizeof(keep.scratch);
    status_.sample_now = samples_;
    status_.results_sample = results_sample_;
    shared_memory.application_queue.push(status_);
    last_status_ = DWT->CYCCNT;
}

void GpsRxProcessor::execute(const buffer_c8_t& buffer) {
    const uint16_t missed = shared_memory.m4_buffer_missed;
    samples_ += static_cast<uint64_t>(buffer.count) * (1u + static_cast<uint16_t>(missed - missed_seen_));
    missed_seen_ = missed;
    if (pending_) apply();
#if defined(BASEBAND_gps_l1t)
    if (!trk_) {
        if (DWT->CYCCNT - last_status_ > 5 * kStatusCycles) send_status(); /* idle: twice a second */
        return;
    }
    /* the tracking loops: every buffer, ~11.5k cycles per channel + ~23k */
    const uint32_t t0 = DWT->CYCCNT;
    trk_->process(reinterpret_cast<const int8_t*>(buffer.p), samples_ - buffer.count);
    const uint32_t dt = DWT->CYCCNT - t0;
    if ((trk_buffers_ & 63u) == 0) { /* the input level, one buffer in 64 (~12k cycles) */
        const int8_t* x = reinterpret_cast<const int8_t*>(buffer.p);
        int32_t si = 0, sq = 0;
        uint32_t ss = 0, clip = 0;
        for (size_t k = 0; k < buffer.count; k++) {
            const int32_t i = x[2 * k], q = x[2 * k + 1];
            si += i;
            sq += q;
            ss += static_cast<uint32_t>(i * i + q * q);
            clip += (i >= 127 || i <= -127 || q >= 127 || q <= -127) ? 1u : 0u;
        }
        const float n = static_cast<float>(buffer.count);
        const float mi = si / n, mq = sq / n;
        const float ac = ss / (2.0f * n) - 0.5f * (mi * mi + mq * mq);
        keep.shared.level_x10 = static_cast<uint16_t>(ac > 0 ? std::sqrt(ac) * 10.0f : 0.0f);
        keep.shared.dc_i_x10 = static_cast<int16_t>(mi * 10.0f);
        keep.shared.dc_q_x10 = static_cast<int16_t>(mq * 10.0f);
        keep.shared.clip_pm = static_cast<uint16_t>(clip * 1000u / buffer.count);
    }
    trk_cycles_sum_ += dt;
    if (dt > trk_cycles_max_) trk_cycles_max_ = dt;
    trk_buffers_++;
    if (DWT->CYCCNT - last_status_ > kStatusCycles * 2) { /* 5 per second */
        keep.shared.cycles_avg = trk_buffers_ ? trk_cycles_sum_ / trk_buffers_ : 0;
        keep.shared.cycles_max = trk_cycles_max_;
        trk_cycles_sum_ = trk_cycles_max_ = trk_buffers_ = 0;
        trk_->publish();
        send_status();
    }
#else
    if (state_ == GpsRxStatusMessage::IDLE || state_ == GpsRxStatusMessage::DONE) {
        if (DWT->CYCCNT - last_status_ > 5 * kStatusCycles) send_status(); /* idle: twice a second */
        return;
    }
    if (settle_) {
        settle_--;
        return;
    }
    if (session.want_samples()) {
        session.put(reinterpret_cast<const int8_t*>(buffer.p), buffer.count, static_cast<double>(samples_) / gps_acq::kFs);
        if (session.phase() == gps_acq::Session::DONE)
            state_ = GpsRxStatusMessage::DONE; /* level too low */
        else if (!session.want_samples())
            state_ = GpsRxStatusMessage::SEARCHING;
        if (!session.want_samples()) send_status();
        return;
    }
    /* one work unit per DMA buffer (the samples are not needed now) */
    const uint32_t t0 = DWT->CYCCNT;
    session.step();
    unit_cycles_sum_ += DWT->CYCCNT - t0;
    units_++;
    const uint8_t before = state_;
    const auto ph = session.phase();
    if (ph == gps_acq::Session::DONE)
        state_ = GpsRxStatusMessage::DONE; /* Autonomous: the app then starts the tracking image */
    else if (session.want_samples())
        state_ = GpsRxStatusMessage::CAPTURING;
    else
        state_ = acquirer.refining() ? GpsRxStatusMessage::REFINING : GpsRxStatusMessage::SEARCHING;
    if (state_ != before || DWT->CYCCNT - last_status_ > kStatusCycles) send_status();
#endif
}

void GpsRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::GpsRxConfig) {
        const auto& m = *reinterpret_cast<const GpsRxConfigMessage*>(message);
        if (m.cmd == GpsRxConfigMessage::CMD_ADD) { /* only the aided-search fields: the search settings stay */
            req_.cmd = m.cmd;
            req_.add_prn = m.add_prn;
            req_.add_windows = m.add_windows;
            req_.add_code = m.add_code;
            req_.add_doppler = m.add_doppler;
            req_.add_at = m.add_at;
            pending_ = true;
            return;
        }
        req_.cmd = m.cmd;
        req_.k_ms = m.k_ms;
        req_.half_bins = m.half_bins;
        req_.refine = m.refine;
        req_.prn_mask = m.prn_mask;
        std::memcpy(req_.center, m.center, sizeof(req_.center));
        req_.confirm = m.confirm;
        req_.track = m.track;
        req_.max_ch = m.max_ch;
        req_.age_ms = m.age_ms;
        pending_ = true;
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<GpsRxProcessor>()};
    event_dispatcher.run();
    return 0;
}
