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
 * A detection goes to a Galileo tracking channel (gal_trk.hpp) when
 * rx_gps.ini's gal_trk allows (default 4, 0 = search only) and the channel
 * pool has room: GPS keeps its channels, Galileo gets what the M4 time
 * leaves. Nothing of Galileo enters the fix yet. GPS is unchanged when the
 * box is off (no command, no file read, no extra M4 work; unticking it
 * releases the Galileo channels). */

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
    std::memset(static_cast<void*>(mem_->gal_prev), 0, sizeof(mem_->gal_prev));
    if (gal_nav_) std::memset(gal_nav_, 0, sizeof(gal_inav::Work));
    gal_round_t_ = chTimeNow();
}

uint8_t GpsView::gal_heard() const {
    if (!mem_) return 0;
    uint8_t n = 0;
    for (int p = 1; p <= 36; p++) n += mem_->gal_state[p] == 2;
    return n;
}

const GpsScratch::GalPrev* GpsView::gal_tracked(int prn) const {
    if (!mem_ || !gal_trk_sh_) return nullptr;
    for (const auto& c : mem_->gal_prev)
        if (c.prn == prn && c.state != gps_trk::TRK_IDLE && c.state != gps_trk::TRK_FAILED) return &c;
    return nullptr;
}

uint8_t GpsView::gal_tracking() const {
    if (!mem_ || !gal_trk_sh_) return 0;
    uint8_t n = 0;
    for (const auto& c : mem_->gal_prev) n += c.state != gps_trk::TRK_IDLE && c.state != gps_trk::TRK_FAILED;
    return n;
}

/* one Galileo channel of the tracking image's table (seqlock; false: being written) */
bool GpsView::gal_channel(int i, gal_acq::TrkInfo& c) const {
    if (!gal_trk_sh_) return false;
    for (int k = 0; k < 3; k++) {
        const uint32_t s0 = gal_trk_sh_->seq;
        if (s0 & 1u) continue;
        std::memcpy(static_cast<void*>(&c), const_cast<const gal_acq::TrkInfo*>(&gal_trk_sh_->ch[i]), sizeof(c));
        if (gal_trk_sh_->seq == s0) return true;
    }
    return false;
}

/* the E1 box unticked: the tracking image stops its search and frees the Galileo channels */
void GpsView::gal_off() {
    gal_pending_ = false;
    if (gal_off_sent_ || image_ != gps_image::ID_L1T || !image_ready_ || !tracking_) return;
    GpsRxConfigMessage msg;
    msg.cmd = GpsRxConfigMessage::CMD_GAL;
    msg.add_prn = 0;
    baseband::gps_rx_config(msg);
    gal_off_sent_ = true;
}

/* 5 times a second with the status (UI thread): copy the Galileo channels
 * (seqlock) and log what changed since the last copy */
void GpsView::gal_poll() {
    if (!mem_ || !gal_trk_sh_ || image_ != gps_image::ID_L1T) return;
    bool changed = false;
    for (int i = 0; i < gal_acq::kMaxTrk; i++) {
        gal_acq::TrkInfo c;
        if (!gal_channel(i, c)) continue;
        GpsScratch::GalPrev& p = mem_->gal_prev[i];
        const bool on = c.state != gps_trk::TRK_IDLE && c.state != gps_trk::TRK_FAILED;
        const bool was = p.prn == c.prn && p.state != gps_trk::TRK_IDLE && p.state != gps_trk::TRK_FAILED;
        if (on && !was) {
            log_gal_event(c, "start", static_cast<int32_t>(c.code_x16), static_cast<int32_t>(std::lround(c.doppler_hz)));
            changed = true;
        }
        if (p.prn == c.prn) {
            if ((c.flags & gal_acq::TF_CS) && !(p.flags & gal_acq::TF_CS)) log_gal_event(c, "cs", c.cs_phase, c.cs_syncs);
            if (!(c.flags & gal_acq::TF_CS) && (p.flags & gal_acq::TF_CS) && on) log_gal_event(c, "cs_lost", c.cs_errs, 0);
            if (c.side_jumps != p.side_jumps) log_gal_event(c, "side", c.side_e_x100, c.side_l_x100);
            if (c.fl_fixes != p.fl_fixes) log_gal_event(c, "false_lock", c.fl_fixes, static_cast<int32_t>(std::lround(c.doppler_hz)));
            if (c.state == gps_trk::TRK_SEARCH && p.state != gps_trk::TRK_SEARCH) log_gal_event(c, "lost", static_cast<int32_t>(p.lock_ms), c.reacq);
            if (p.state == gps_trk::TRK_SEARCH && (c.state == gps_trk::TRK_PULLIN || c.state == gps_trk::TRK_TRACK))
                log_gal_event(c, "found", static_cast<int32_t>(c.code_x16), static_cast<int32_t>(std::lround(c.doppler_hz)));
        }
        if ((c.flags & gal_acq::TF_RELEASED) && !((p.flags & gal_acq::TF_RELEASED) && p.prn == c.prn)) {
            log_gal_event(c, "released", c.release, 0);
            if (c.prn >= 1 && c.prn <= 36) mem_->gal_state[c.prn] = 0; /* searched again in a later round */
            changed = true;
        }
        if (c.state != p.state || (c.flags & gal_acq::TF_CS) != (p.flags & gal_acq::TF_CS)) changed = true;
        p.prn = c.prn;
        p.state = c.state;
        p.flags = c.flags;
        p.fl_fixes = c.fl_fixes;
        p.side_jumps = c.side_jumps;
        p.cn0c_x10 = c.cn0c_x10;
        p.lock_ms = c.lock_ms;
    }
    if (changed) view_heard_.show();
}

static_assert(sizeof(gal_inav::Work) <= gal_acq::kNavWorkBytes, "gal_inav::Work outgrew the RAM the tracking image lends");
static_assert(gal_inav::Work::kCh >= gal_acq::kMaxTrk, "one decoder per Galileo channel");

uint16_t GpsView::gal_nav_pages(int prn) const {
    if (!gal_nav_) return 0;
    const gal_inav::Work& w = *static_cast<const gal_inav::Work*>(gal_nav_);
    for (const auto& c : w.ch)
        if (c.prn == prn) return c.pages_ok;
    return 0;
}

/* frame handler (E1 box on, tracking image): the next channel's symbols, at
 * most one page part decoded per frame (~150k M0 instructions) */
void GpsView::gal_nav_step() {
    if (!gal_on_ || !gal_nav_ || !gal_trk_sh_ || image_ != gps_image::ID_L1T || !tracking_) return;
    gal_inav::Work& w = *static_cast<gal_inav::Work*>(gal_nav_);
    for (int k = 0; k < gal_acq::kMaxTrk; k++) {
        const int i = (gal_nav_rr_ + k) % gal_acq::kMaxTrk;
        const uint8_t prn = mem_ ? mem_->gal_prev[i].prn : 0;
        const uint8_t st = mem_ ? mem_->gal_prev[i].state : 0;
        if (!prn || st == gps_trk::TRK_IDLE || st == gps_trk::TRK_FAILED) continue;
        const uint32_t parts = w.ch[i].parts;
        gal_inav::Page pg;
        const bool page = gal_inav::step(w, i, prn, gal_trk_sh_->sym[i], gal_acq::kSymRing, gal_trk_sh_->sym_write[i], gal_trk_sh_->sym_epoch[i], &pg);
        if (page) log_gal_page(pg);
        if (page || w.ch[i].parts != parts) { /* one part decoded: the next channel next frame */
            gal_nav_rr_ = static_cast<uint8_t>((i + 1) % gal_acq::kMaxTrk);
            if (page && pg.crc_ok && w.ch[i].pages_ok == 1) view_heard_.show(); /* " N" on its row */
            return;
        }
    }
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
            const bool room = gal_trk_max_ && gal_tracking() < gal_trk_max_;
            for (int p = 1; p <= 36; p++) /* the heard ones keep their result (heard, not tracked: again while channels are free) */
                if (mem_->gal_state[p] == 3 || (room && mem_->gal_state[p] == 2 && !gal_tracked(p))) mem_->gal_state[p] = 0;
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
    msg.gal_max = static_cast<uint8_t>(gal_trk_max_ > gal_acq::kMaxTrk ? gal_acq::kMaxTrk : gal_trk_max_);
    baseband::gps_rx_config(msg);
    gal_off_sent_ = false;
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
