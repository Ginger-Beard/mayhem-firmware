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

/* Big working memory, outside the processor object (bss). The tracking
 * loops are constructed in the snapshot's storage while they run (the
 * search is idle then; the snapshot is re-created before the next one). */
alignas(8) static gps_acq::Snapshot snapshot;
static_assert(sizeof(gps_trk::Tracker) <= sizeof(gps_acq::Snapshot), "tracker must fit in the snapshot RAM");
static gps_trk::Shared trk_shared;
static const gps_trk::CarrierTable carrier_table;
static gps_acq::Acquirer acquirer{snapshot};
static gps_acq::Session session{snapshot, acquirer};
/* M4 RAM lent to the app (orbit tables, solver work space); never touched here. */
static uint8_t m0_scratch[GpsRxStatusMessage::kScratchSize] __attribute__((aligned(8)));

GpsRxProcessor::GpsRxProcessor() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_status_ = DWT->CYCCNT;
    missed_seen_ = shared_memory.m4_buffer_missed;
}

void GpsRxProcessor::start_tracking(uint32_t prn_mask) {
    const gps_acq::Result* res = session.results();
    /* the PRNs to track, strongest first */
    int order[32];
    int n = 0;
    for (int i = 0; i < 32; i++)
        if (((prn_mask >> i) & 1u) && res[i].detected && !res[i].once) order[n++] = i;
    for (int a = 1; a < n; a++)
        for (int b = a; b > 0 && res[order[b]].cn0 > res[order[b - 1]].cn0; b--) {
            const int t = order[b];
            order[b] = order[b - 1];
            order[b - 1] = t;
        }
    if (!n) return;
    session.stop();
    if (!trk_) trk_ = new (&snapshot) gps_trk::Tracker(&trk_shared, &carrier_table);
    trk_->set_max_active(max_ch_);
    /* code phases refer to the first sample of the result snapshot; samples_
     * is the index of the next buffer's first sample (put() got samples_ / fs
     * after this buffer was counted) */
    const uint64_t now = samples_;
    const uint64_t at = static_cast<uint64_t>(std::llround(session.t_results() * gps_acq::kFs)) - gps_acq::kN;
    trk_->reset(now);
    for (int k = 0; k < n && k < trk_->max_active(); k++)
        trk_->start(order[k] + 1, res[order[k]].code_phase, res[order[k]].doppler, at, now);
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
    new (&snapshot) gps_acq::Snapshot(); /* the storage is the snapshot again */
}

void GpsRxProcessor::apply() {
    pending_ = false;
    if (req_.cmd == GpsRxConfigMessage::CMD_STOP) {
        stop_tracking();
        session.stop();
        state_ = GpsRxStatusMessage::IDLE;
        send_status();
        return;
    }
    max_ch_ = req_.max_ch ? req_.max_ch : 8;
    if (req_.cmd == GpsRxConfigMessage::CMD_TRACK) {
        start_tracking(req_.prn_mask);
        return;
    }
    stop_tracking(); /* a new search */
    seq_++;
    units_ = 0;
    unit_cycles_sum_ = 0;
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

void GpsRxProcessor::send_status() {
    status_.state = state_;
    status_.level = session.level();
    status_.confirming = session.confirming();
    status_.done_prn = static_cast<uint8_t>(acquirer.done_count());
    status_.total_prn = static_cast<uint8_t>(acquirer.requested());
    if (!trk_) { /* the snapshot's storage holds the tracker while it runs */
        last_k_ = static_cast<uint8_t>(snapshot.k());
        last_rms_x10_ = static_cast<uint16_t>(snapshot.rms() * 10.0f);
        const uint32_t f = snapshot.filled();
        last_clip_ = static_cast<uint16_t>(f ? (snapshot.clipped() * 1000u) / f : 0);
    }
    status_.k_ms = last_k_;
    status_.rms_x10 = last_rms_x10_;
    status_.clip_permille = last_clip_;
    status_.seq = seq_;
    status_.units = units_;
    status_.unit_cycles = units_ ? static_cast<uint32_t>(unit_cycles_sum_ / units_) : 0;
    status_.track = &trk_shared;
    status_.results = session.results();
    status_.scratch = m0_scratch;
    status_.scratch_size = sizeof(m0_scratch);
    shared_memory.application_queue.push(status_);
    last_status_ = DWT->CYCCNT;
}

void GpsRxProcessor::execute(const buffer_c8_t& buffer) {
    const uint16_t missed = shared_memory.m4_buffer_missed;
    samples_ += static_cast<uint64_t>(buffer.count) * (1u + static_cast<uint16_t>(missed - missed_seen_));
    missed_seen_ = missed;
    if (pending_) apply();
    if (trk_) {
        /* the tracking loops: every buffer, ~11.5k cycles per channel + ~23k */
        const uint32_t t0 = DWT->CYCCNT;
        trk_->process(reinterpret_cast<const int8_t*>(buffer.p), samples_ - buffer.count);
        const uint32_t dt = DWT->CYCCNT - t0;
        trk_cycles_sum_ += dt;
        if (dt > trk_cycles_max_) trk_cycles_max_ = dt;
        trk_buffers_++;
        if (DWT->CYCCNT - last_status_ > kStatusCycles * 2) { /* 5 per second */
            trk_shared.cycles_avg = trk_buffers_ ? trk_cycles_sum_ / trk_buffers_ : 0;
            trk_shared.cycles_max = trk_cycles_max_;
            trk_cycles_sum_ = trk_cycles_max_ = trk_buffers_ = 0;
            trk_->publish();
            send_status();
        }
        return;
    }
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
    if (ph == gps_acq::Session::DONE) {
        state_ = GpsRxStatusMessage::DONE;
        if (req_.track) { /* "Autonomous": the DONE status first (the app's coarse fix), then the loops */
            send_status();
            uint32_t mask = 0;
            const gps_acq::Result* r = session.results();
            for (int i = 0; i < 32; i++)
                if (r[i].detected && !r[i].once) mask |= 1u << i;
            if (mask) start_tracking(mask);
            return;
        }
    } else if (session.want_samples())
        state_ = GpsRxStatusMessage::CAPTURING;
    else
        state_ = acquirer.refining() ? GpsRxStatusMessage::REFINING : GpsRxStatusMessage::SEARCHING;
    if (state_ != before || DWT->CYCCNT - last_status_ > kStatusCycles) send_status();
}

void GpsRxProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::GpsRxConfig) {
        const auto& m = *reinterpret_cast<const GpsRxConfigMessage*>(message);
        req_.cmd = m.cmd;
        req_.k_ms = m.k_ms;
        req_.half_bins = m.half_bins;
        req_.refine = m.refine;
        req_.prn_mask = m.prn_mask;
        std::memcpy(req_.center, m.center, sizeof(req_.center));
        req_.confirm = m.confirm;
        req_.track = m.track;
        req_.max_ch = m.max_ch;
        pending_ = true;
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<GpsRxProcessor>()};
    event_dispatcher.run();
    return 0;
}
