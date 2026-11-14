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

/* The aided Galileo E1 search (Heard "E1" box, off by default): while the
 * tracking loops run and the GPS fix is precise, one Galileo satellite at a
 * time, highest first: its code phase and Doppler predicted from
 * GNSSALM.TXT's orbit and clock, the fix and GPS time at the fix's sample,
 * plus the receiver clock term measured on the GPS channels; its codes read
 * from /GPS/GALE1.BIN (1 KB) into the tracking image's buffer; CMD_GAL; the
 * image searches in the M4 time the loops leave (gal_acq.hpp) and reports.
 * Heard only: nothing of Galileo enters the fix yet. GPS is unchanged when
 * the box is off (no command, no file read, no extra M4 work). */

#include "ui_gps.hpp"

#include "baseband_api.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

namespace ui::external_app::gps {

#if defined(PRALINE)

namespace {
constexpr uint32_t kGalHalfWin = 16;                /* samples either side of the prediction (~8 chips) */
constexpr systime_t kGalTimeout = MS2ST(60000);     /* a search that never reports */
constexpr systime_t kGalRoundPause = MS2ST(120000); /* all searched: again after this */
constexpr float kGalMinEl = 5.0f;
}  // namespace

void GpsView::gal_reset() {
    gal_pending_ = false;
    if (!mem_) return;
    std::memset(mem_->gal_state, 0, sizeof(mem_->gal_state));
    std::memset(mem_->gal_cn0, 0, sizeof(mem_->gal_cn0));
    std::memset(mem_->gal_metric_x10, 0, sizeof(mem_->gal_metric_x10));
    gal_round_t_ = chTimeNow();
}

uint8_t GpsView::gal_heard() const {
    if (!mem_) return 0;
    uint8_t n = 0;
    for (int p = 1; p <= 36; p++) n += mem_->gal_state[p] == 2;
    return n;
}

/* the tracking image finished the search */
void GpsView::gal_collect() {
    gal_pending_ = false;
    if (!gal_result_ || !mem_) return;
    gal_acq::Result r;
    std::memcpy(static_cast<void*>(&r), gal_result_, sizeof(r));
    if (r.prn != gal_prn_ || !r.done || gal_prn_ > 36) return;
    mem_->gal_state[gal_prn_] = r.detected ? 2 : 3;
    mem_->gal_cn0[gal_prn_] = static_cast<uint8_t>(r.cn0 > 0 ? (r.cn0 > 99 ? 99 : r.cn0 + 0.5f) : 0);
    mem_->gal_metric_x10[gal_prn_] = static_cast<uint16_t>(r.metric * 10.0f > 65535 ? 65535 : r.metric * 10.0f);
    log_gal(r);
    view_heard_.show();
}

/* frame handler (a background job): collect, or start the next search */
void GpsView::gal_step() {
    if (!gal_on_ || !mem_ || image_ != gps_image::ID_L1T || !image_ready_ || !tracking_ || !gal_codes_ || !gal_result_) return;
    if (gal_pending_) {
        if (gal_status_seq_ != gal_seq_sent_)
            gal_collect();
        else if (chTimeNow() - gal_sent_t_ > kGalTimeout) { /* never reported: give it up */
            gal_pending_ = false;
            if (gal_prn_ <= 36) mem_->gal_state[gal_prn_] = 3;
        }
        return;
    }
    if (gal_status_busy_ || !fix_ok_ || fix_mode_ != FIX_PRECISE || !fix_have_time_ || sys_active_ || !mem_->n_gal) return;
    if (gal_file_ >= 2) return; /* GALE1.BIN missing or bad: the Heard page says so */
    const AutoState& a = mem_->autos;
    constexpr uint64_t kFsHz = 2048000;
    const uint64_t age = a.meas.sample > fix_sample_ ? a.meas.sample - fix_sample_ : 0;
    if (age > 3u * kFsHz) return; /* a recent fix: the time at fix_sample_ is what the prediction needs */
    /* the next satellite: up (SysSky), not searched in this round, highest first */
    const SysSky& k = mem_->sys;
    int best = -1;
    float best_el = 0;
    for (size_t i = 0; i < mem_->n_gal; i++) {
        const uint8_t prn = mem_->gal[i].prn;
        if (prn < 1 || prn > 36 || mem_->gal_state[prn]) continue;
        for (size_t j = 0; j < k.n; j++) {
            const SysSat& s = k.sat[j];
            if (s.sys != SYS_GAL || s.prn != prn || s.health || s.el < kGalMinEl) continue;
            if (best < 0 || s.el > best_el) {
                best = static_cast<int>(i);
                best_el = s.el;
            }
        }
    }
    if (best < 0) { /* all searched: another round after a pause */
        if (chTimeNow() - gal_round_t_ > kGalRoundPause) {
            for (int p = 1; p <= 36; p++)
                if (mem_->gal_state[p] == 3) mem_->gal_state[p] = 0; /* the heard ones keep their result */
            gal_round_t_ = chTimeNow();
            if (!gal_heard())
                for (int p = 1; p <= 36; p++) mem_->gal_state[p] = 0;
        }
        return;
    }
    /* the receiver clock's Doppler term: median over the verified GPS channels */
    float diffs[gps_trk::kMaxCh];
    int nd = 0;
    for (int i = 0; i < gps_trk::kMaxCh; i++) {
        const gps_trk::ChInfo& c = a.ch[i];
        if (c.prn >= 1 && c.prn <= 32 && c.state == gps_trk::TRK_TRACK && (c.flags & gps_trk::F_VERIFIED))
            diffs[nd++] = c.doppler_hz + mem_->geo[c.prn - 1].rr * static_cast<float>(1575.42e6 / kC);
    }
    if (!nd) return;
    for (int x = 1; x < nd; x++)
        for (int y = x; y > 0 && diffs[y] < diffs[y - 1]; y--) std::swap(diffs[y], diffs[y - 1]);
    const float clk = diffs[nd / 2];
    const GalAlm& g = mem_->gal[best];
    const Orbit o = gal_orbit(g);
    double cp = 0, dg = 0;
    gal_predict(o, fix_.pos, fix_trx_, &cp, &dg);
    /* the codes, into the tracking image's buffer */
    void* const m = chHeapAlloc(nullptr, sizeof(File)); /* not operator new: it halts on a short heap */
    if (!m) return;
    File* f = new (m) File{};
    bool ok = false;
    if (f->open(u"GPS/GALE1.BIN").is_valid()) {
        gal_file_ = 2;
    } else {
        char hdr[gal_acq::kFileHeader];
        const auto r = f->read(hdr, sizeof(hdr));
        if (!r || r.value() != sizeof(hdr) || std::memcmp(hdr, "GALE1", 5) != 0 || f->size() < gal_acq::kFileHeader + 50u * gal_acq::kFilePrnBytes) {
            gal_file_ = 3;
        } else if (f->seek(gal_acq::kFileHeader + (g.prn - 1u) * gal_acq::kFilePrnBytes)) {
            const auto rr = f->read(gal_codes_, gal_acq::kFilePrnBytes);
            ok = rr && rr.value() == gal_acq::kFilePrnBytes;
            gal_file_ = ok ? 1 : 3;
        }
    }
    f->~File();
    chHeapFree(m);
    if (!ok) {
        view_heard_.show();
        return;
    }
    GpsRxConfigMessage msg;
    msg.cmd = GpsRxConfigMessage::CMD_GAL;
    msg.add_prn = g.prn;
    msg.add_windows = kGalHalfWin;
    msg.add_code = static_cast<float>(cp);
    msg.add_doppler = static_cast<float>(dg + clk);
    msg.add_at = fix_sample_;
    baseband::gps_rx_config(msg);
    gal_pending_ = true;
    gal_seq_sent_ = gal_status_seq_;
    gal_prn_ = g.prn;
    gal_sent_t_ = chTimeNow();
    gal_pred_el_ = best_el;
    gal_pred_clk_ = clk;
    gal_pred_cp_ = cp;
    gal_pred_dop_ = dg + clk;
    gal_pred_age_ms_ = static_cast<uint32_t>(age * 1000u / kFsHz);
    mem_->gal_state[g.prn] = 1;
    view_heard_.show();
}

#endif /* PRALINE */

}  // namespace ui::external_app::gps
