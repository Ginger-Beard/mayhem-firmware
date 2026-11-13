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
/* M4 RAM lent to the app (orbit tables, solver work space); never touched here. */
static uint8_t m0_scratch[GpsRxStatusMessage::kScratchSize] __attribute__((aligned(8)));

GpsRxProcessor::GpsRxProcessor() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_status_ = DWT->CYCCNT;
}

void GpsRxProcessor::apply() {
    pending_ = false;
    if (req_.cmd == GpsRxConfigMessage::CMD_STOP) {
        acquirer.stop();
        state_ = GpsRxStatusMessage::IDLE;
        send_status();
        return;
    }
    seq_++;
    units_ = 0;
    unit_cycles_sum_ = 0;
    snapshot.start(req_.k_ms);
    state_ = GpsRxStatusMessage::CAPTURING;
    send_status();
}

void GpsRxProcessor::send_status() {
    status_.state = state_;
    status_.done_prn = static_cast<uint8_t>(acquirer.done_count());
    status_.total_prn = static_cast<uint8_t>(acquirer.requested());
    status_.k_ms = static_cast<uint8_t>(snapshot.k());
    status_.rms_x10 = static_cast<uint16_t>(snapshot.rms() * 10.0f);
    const uint32_t f = snapshot.filled();
    status_.clip_permille = static_cast<uint16_t>(f ? (snapshot.clipped() * 1000u) / f : 0);
    status_.seq = seq_;
    status_.units = units_;
    status_.unit_cycles = units_ ? static_cast<uint32_t>(unit_cycles_sum_ / units_) : 0;
    status_.results = acquirer.results();
    status_.scratch = m0_scratch;
    status_.scratch_size = sizeof(m0_scratch);
    shared_memory.application_queue.push(status_);
    last_status_ = DWT->CYCCNT;
}

void GpsRxProcessor::execute(const buffer_c8_t& buffer) {
    if (pending_) apply();
    switch (state_) {
        case GpsRxStatusMessage::CAPTURING: {
            snapshot.put(reinterpret_cast<const int8_t*>(buffer.p), buffer.count);
            if (snapshot.full()) {
                gps_acq::Acquirer::Request r;
                r.prn_mask = req_.prn_mask;
                std::memcpy(r.center, req_.center, sizeof(r.center));
                r.half_bins = req_.half_bins > 20 ? 20 : req_.half_bins;
                r.refine = req_.refine;
                acquirer.start(r);
                state_ = GpsRxStatusMessage::SEARCHING;
                send_status();
            }
            break;
        }
        case GpsRxStatusMessage::SEARCHING:
        case GpsRxStatusMessage::REFINING: {
            /* one work unit per DMA buffer (the samples are not needed now) */
            const uint32_t t0 = DWT->CYCCNT;
            const bool more = acquirer.step();
            unit_cycles_sum_ += DWT->CYCCNT - t0;
            units_++;
            state_ = !more ? GpsRxStatusMessage::DONE : (acquirer.refining() ? GpsRxStatusMessage::REFINING : GpsRxStatusMessage::SEARCHING);
            if (!more || DWT->CYCCNT - last_status_ > kStatusCycles) send_status();
            break;
        }
        default:
            if (DWT->CYCCNT - last_status_ > 5 * kStatusCycles) send_status(); /* idle: twice a second */
            break;
    }
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
        pending_ = true;
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<GpsRxProcessor>()};
    event_dispatcher.run();
    return 0;
}
