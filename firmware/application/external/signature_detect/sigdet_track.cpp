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

#include "sigdet_track.hpp"

namespace ui::external_app::signature_detect {

size_t track_bw_for(uint32_t bw_khz) {
    for (size_t i = kTrackBws; i-- > 0;)
        if (kTrackBwKhz[i] >= bw_khz) return i;
    return 0;
}

/* ---------------------------------------------------------------- stats */

void TrackStats::reset_minmax(uint32_t now_ms) {
    min_ = max_ = level_;
    hold_ = level_;
    hold_ms_ = now_ms;
}

void TrackStats::clear(uint32_t now_ms) {
    level_ = hold_ = min_ = max_ = sec_max_ = kNone;
    for (auto& h : hist_) h = -128;
    col_max_ = -128;
    head_ = 0;
    last_ms_ = 0;
    hold_ms_ = col_ms_ = now_ms;
    started_ = true;
}

void TrackStats::add(int16_t cdb, uint32_t now_ms) {
    if (!started_) clear(now_ms);
    level_ = cdb;
    last_ms_ = now_ms;
    if (min_ == kNone || cdb < min_) min_ = cdb;
    if (max_ == kNone || cdb > max_) max_ = cdb;
    if (sec_max_ == kNone || cdb > sec_max_) sec_max_ = cdb;
    if (hold_ == kNone || cdb >= hold_) {
        hold_ = cdb;
        hold_ms_ = now_ms;
    }
    const int32_t db = cdb / 100;
    const int8_t v = static_cast<int8_t>(db < -127 ? -127 : (db > 127 ? 127 : db));
    if (v > col_max_) col_max_ = v;
}

void TrackStats::push_column(int8_t v) {
    hist_[head_] = v;
    head_ = (head_ + 1) % kHist;
}

void TrackStats::tick(uint32_t now_ms) {
    if (!started_) clear(now_ms);
    /* Peak hold: falls from the moment it was set. */
    if (hold_ != kNone) {
        const uint32_t dt = now_ms - hold_ms_;
        if (dt >= 100) {
            const int32_t fall = static_cast<int32_t>(dt) * kDecayCdbPerS / 1000;
            int32_t h = hold_ - fall;
            if (level_ != kNone && h < level_) h = level_;
            hold_ = static_cast<int16_t>(h < -12000 ? -12000 : h);
            hold_ms_ = now_ms;
        }
    }
    /* One column per kColumnMs (several if the caller was late, at most a
     * whole strip). */
    size_t n = 0;
    while (now_ms - col_ms_ >= kColumnMs && n < kHist) {
        push_column(col_max_);
        col_max_ = -128;
        col_ms_ += kColumnMs;
        n++;
    }
    if (n == kHist) col_ms_ = now_ms;
}

int16_t TrackStats::take_second_max() {
    const int16_t v = sec_max_;
    sec_max_ = kNone;
    return v;
}

/* ---------------------------------------------------------------- beep */

bool Geiger::due(int16_t level_cdb, int16_t squelch_cdb, uint32_t now_ms, uint16_t& hz, uint16_t& ms) {
    if (level_cdb == TrackStats::kNone || level_cdb < squelch_cdb) return false;
    int32_t over = level_cdb - squelch_cdb;
    if (over > kSpanCdb) over = kSpanCdb;
    const uint32_t interval = kSlowMs - static_cast<uint32_t>(over) * (kSlowMs - kFastMs) / kSpanCdb;
    if (started_ && now_ms - last_ms_ < interval) return false;
    started_ = true;
    last_ms_ = now_ms;
    hz = static_cast<uint16_t>(500 + over * 2000 / kSpanCdb);
    ms = static_cast<uint16_t>(interval < 120 ? 25 : 40);
    return true;
}

/* ---------------------------------------------------------------- BLE rate */

void PacketRate::add(uint32_t now_ms) {
    t_[head_] = now_ms;
    head_ = (head_ + 1) % kMax;
    if (n_ < kMax) n_++;
}

uint32_t PacketRate::rate100(uint32_t now_ms) const {
    uint32_t c = 0, oldest = now_ms;
    for (size_t i = 0; i < n_; i++)
        if (now_ms - t_[i] < kWindowMs) {
            c++;
            if (now_ms - t_[i] > now_ms - oldest) oldest = t_[i];
        }
    if (c == kMax) {
        /* the ring holds less than the window: the span of what it holds */
        const uint32_t span = now_ms - oldest;
        return span ? (kMax - 1) * 100000u / span : 0;
    }
    return c * 100000u / kWindowMs;
}

} /* namespace ui::external_app::signature_detect */
