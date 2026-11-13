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

/* Big working memory, outside the processor object (bss). */
static gps_acq::Snapshot snapshot;
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

void GpsRxProcessor::apply() {
    pending_ = false;
    if (req_.cmd == GpsRxConfigMessage::CMD_STOP) {
        session.stop();
        state_ = GpsRxStatusMessage::IDLE;
        send_status();
        return;
    }
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
    status_.k_ms = static_cast<uint8_t>(snapshot.k());
    status_.rms_x10 = static_cast<uint16_t>(snapshot.rms() * 10.0f);
    const uint32_t f = snapshot.filled();
    status_.clip_permille = static_cast<uint16_t>(f ? (snapshot.clipped() * 1000u) / f : 0);
    status_.seq = seq_;
    status_.units = units_;
    status_.unit_cycles = units_ ? static_cast<uint32_t>(unit_cycles_sum_ / units_) : 0;
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
        state_ = GpsRxStatusMessage::DONE;
    else if (session.want_samples())
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
        pending_ = true;
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<GpsRxProcessor>()};
    event_dispatcher.run();
    return 0;
}
