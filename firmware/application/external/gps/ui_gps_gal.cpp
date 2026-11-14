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
    if (gal_nav_) std::memset(gal_nav_, 0, sizeof(GalNavMem));
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

static_assert(sizeof(GalNavMem) <= gal_acq::kNavWorkBytes, "GalNavMem outgrew the RAM the tracking image lends");
static_assert(gal_inav::Work::kCh >= gal_acq::kMaxTrk, "one decoder per Galileo channel");

uint16_t GpsView::gal_nav_pages(int prn) const {
    if (!gal_nav_) return 0;
    const gal_inav::Work& w = gal_mem()->inav;
    for (const auto& c : w.ch)
        if (c.prn == prn) return c.pages_ok;
    return 0;
}

/* frame handler (E1 box on, tracking image): the next channel's symbols, at
 * most one page part decoded per frame (~150k M0 instructions). Not while
 * the solver runs: it may read the Galileo ephemerides a page would replace
 * (the symbols wait in the 2 s rings). */
void GpsView::gal_nav_step() {
    if (!gal_on_ || !gal_nav_ || !gal_trk_sh_ || image_ != gps_image::ID_L1T || !tracking_ || solving()) return;
    GalNavMem& gm = *gal_mem();
    gal_inav::Work& w = gm.inav;
    for (int k = 0; k < gal_acq::kMaxTrk; k++) {
        const int i = (gal_nav_rr_ + k) % gal_acq::kMaxTrk;
        const uint8_t prn = mem_ ? mem_->gal_prev[i].prn : 0;
        const uint8_t st = mem_ ? mem_->gal_prev[i].state : 0;
        if (!prn || st == gps_trk::TRK_IDLE || st == gps_trk::TRK_FAILED) continue;
        const uint32_t parts = w.ch[i].parts;
        gal_inav::Page pg;
        const bool page = gal_inav::step(w, i, prn, gal_trk_sh_->sym[i], gal_acq::kSymRing, gal_trk_sh_->sym_write[i], gal_trk_sh_->sym_epoch[i], &pg);
        if (page) {
            log_gal_page(pg);
            int slot = -1;
            const uint8_t ev = gal_eph::take(gm.eph, i, gal_trk_sh_->sym_epoch[i], pg, now_gps(), &slot);
            if (ev != gal_eph::EV_NONE && ev != gal_eph::EV_WORD) log_gal_eph(ev, i, slot, prn);
            if (ev == gal_eph::EV_NEW) view_heard_.show();
        }
        if (page || w.ch[i].parts != parts) { /* one part decoded: the next channel next frame */
            gal_nav_rr_ = static_cast<uint8_t>((i + 1) % gal_acq::kMaxTrk);
            if (page && pg.crc_ok && w.ch[i].pages_ok == 1) view_heard_.show(); /* " N" on its row */
            return;
        }
    }
}

uint8_t GpsView::gal_eph_ready(int prn) const {
    return gal_nav_ && gal_eph::usable(gal_mem()->eph, prn, now_gps()) ? 1 : 0;
}

uint8_t GpsView::gal_eph_count() const {
    if (!mem_ || !gal_nav_) return 0;
    uint8_t n = 0;
    for (const auto& c : mem_->gal_prev)
        if (c.prn && c.state != gps_trk::TRK_IDLE && c.state != gps_trk::TRK_FAILED) n += gal_eph_ready(c.prn);
    return n;
}

bool GpsView::gal_in_fix(int prn) const {
    return fix_gal_ && prn >= 1 && prn <= 64 && ((fix_gal_prns_ >> (prn - 1)) & 1u);
}

/* one short line (<= 23 characters: the Heard list) on Galileo in the fix;
 * alone: the Galileo-alone solution when there is one (System wheel on Galileo) */
void GpsView::gal_fix_text(char* b, size_t n, bool alone) const {
    Fmt f{b, n};
    if (alone && alone_n_) {
        f.s("E alone: ").u(alone_n_).s(" sat ");
        if (alone_dm_ == 0xFFFF)
            f.s("no GPS");
        else
            f.u((alone_dm_ + 5u) / 10u).s(" m off");
        return;
    }
    if (fix_gal_) {
        f.s("in fix: G").u(static_cast<uint32_t>(fix_.used - fix_gal_)).s("+E").u(fix_gal_).s(fix_gal_mode_ == 1 ? " GGTO" : " own clk");
        return;
    }
    static const char* const kIn[] = {"", "fix: none tracked", "fix: no sec. code yet", "fix: too weak", "fix: waiting page time", "fix: confirming time", "fix: waiting ephemeris", "fix: no room (GPS)",
                                      "", "fix: 1 sat, no GGTO"};
    static const char* const kGate[] = {"", "fix: out, solve failed", "fix: out, few left", "fix: out, unchecked", "fix: out, residuals", "fix: out, moved fix", "fix: out, rms grew"};
    const uint8_t w = gal_out_why_;
    if (w == 1)
        f.s("fix: off (gal_fix=0)");
    else if (w >= 10 && w < 20 && kIn[w - 10][0])
        f.s(kIn[w - 10]);
    else if (w >= 20 && w < 27)
        f.s(kGate[w - 20]);
    else
        f.s("fix: not in it yet");
}

/* a new GPS measurement set was copied (UI thread): the Galileo set of the
 * same sample, written by the image before the GPS counter moved (logged
 * with the PVT input: gal_add_pvt) */
void GpsView::gal_meas_copy() {
    GalNavMem* gm = gal_mem();
    if (!gm) return;
    gm->meas_ok = 0;
    if (!gal_on_ || !gal_trk_sh_ || image_ != gps_image::ID_L1T || !mem_) return;
    const uint32_t s = gal_trk_sh_->meas_seq;
    if (!s) return;
    std::memcpy(static_cast<void*>(&gm->meas), const_cast<const gal_acq::MeasSet*>(&gal_trk_sh_->meas[(s - 1) & 1u]), sizeof(gm->meas));
    const AutoState& a = mem_->autos;
    gm->meas_ok = gal_trk_sh_->meas_seq == s && gm->meas.seq == a.meas.seq && gm->meas.sample == a.meas.sample;
}

/* PVT input (UI thread, before the solver starts): Galileo entries after the n
 * GPS ones when Galileo may enter the fix. GST - GPS from word type 10 when
 * known (one clock), else the offset is solved (needs 2 Galileo); one Galileo
 * satellite and no GGTO: left out. Returns the new count. */
int GpsView::gal_add_pvt(int n) {
    solve_gal_n_ = 0;
    solve_gal_mode_ = 0;
    solve_gal_alone_ = 0;
    GalNavMem* gm = gal_mem();
    if (!gal_on_ || !gm || image_ != gps_image::ID_L1T || !mem_) return n;
    if (!gal_fix_on_) {
        gal_out_why_ = 1;
        return n;
    }
    if (!gm->meas_ok) {
        gal_out_why_ = 10 + gal_eph::W_IDLE;
        return n;
    }
    AutoState& a = mem_->autos;
    double ggto = 0;
    const double t = a.pvt.t_ref;
    const bool have_ggto = gal_eph::ggto_at(gm->eph, t, &ggto);
    const int n2 = gal_eph::add_pvt(a.pvt, n, gm->meas, gm->inav, gm->eph, t, have_ggto ? ggto : 0.0, gm->why);
    log_gal_meas(); /* with the reasons just found */
    const int k = n2 - n;
    /* the channel that got furthest, for the screen when none is used */
    uint8_t best = gal_eph::W_IDLE;
    for (int i = 0; i < gal_inav::Work::kCh; i++)
        if (gm->why[i] != gal_eph::W_IDLE && (best == gal_eph::W_IDLE || gm->why[i] > best)) best = gm->why[i];
    if (!k) {
        gal_out_why_ = static_cast<uint8_t>(10 + best);
        return n;
    }
    if (!have_ggto && k < 2) { /* its own clock offset would take the one satellite's information */
        gal_out_why_ = 10 + 9;
        return n;
    }
    solve_gal_n_ = static_cast<uint8_t>(k);
    solve_gal_mode_ = have_ggto ? 1 : 2;
    solve_ggto_ns_ = static_cast<float>(ggto * 1e9);
    solve_gal_alone_ = (sys_view_ == VIEW_GAL && k >= 4) ? 1 : 0;
    return n2;
}

/* solver thread, right after the GPS + Galileo solve: the residuals the gate
 * and the log need (the Galileo-alone solve reuses the work space) */
void GpsView::gal_solve_stats() {
    GalNavMem* gm = gal_mem();
    if (!gm) return;
    const PvtWork& w = mem_->autos.pvt;
    double sg = 0, se = 0;
    int ng = 0, ne = 0, k = 0;
    for (int i = 0; i < solve_pvt_n_; i++) {
        const bool gal = w.in[i].sys != 0;
        if (gal && k < gal_inav::Work::kCh) {
            gm->mix_prn[k] = w.in[i].prn;
            const double r = w.resid[i] * 10.0;
            gm->mix_res_dm[k++] = static_cast<int16_t>(!solve_mix_ok_ || !w.use[i] ? 32767 : (r > 32766 ? 32766 : (r < -32766 ? -32766 : std::lround(r))));
        }
        if (!solve_mix_ok_ || !w.use[i]) continue;
        (gal ? se : sg) += w.resid[i] * w.resid[i];
        (gal ? ne : ng)++;
    }
    for (; k < gal_inav::Work::kCh; k++) gm->mix_prn[k] = 0;
    gm->mix_gal_rms = ne ? static_cast<float>(std::sqrt(se / ne)) : 0.0f;
    gm->mix_gps_rms = ng ? static_cast<float>(std::sqrt(sg / ng)) : 0.0f;
}

/* UI thread, after the solver: whether GPS + Galileo replaces GPS alone */
bool GpsView::gal_finish(bool gps_ok) {
    GalNavMem* gm = gal_mem();
    if (!gm || !solve_gal_n_) {
        alone_n_ = 0;
        return false;
    }
    const gal_eph::Gate g = gal_eph::gate(gps_ok, mem_->pvt, solve_mix_ok_, gm->mix, solve_gal_mode_ == 2, gm->mix_gal_rms);
    gm->gate = g;
    const bool use = g == gal_eph::G_IN;
    gal_out_why_ = use ? 0 : static_cast<uint8_t>(20 + g);
    fix_gal_prns_ = 0;
    if (use)
        for (int k = 0; k < gal_inav::Work::kCh; k++)
            if (gm->mix_prn[k] && gm->mix_res_dm[k] != 32767) fix_gal_prns_ |= 1ull << (gm->mix_prn[k] - 1);
    alone_n_ = solve_alone_ok_ ? gm->alone.used : 0;
    if (solve_alone_ok_) {
        const double h = gps_ok ? gal_eph::horiz_m(gm->alone.pos, mem_->pvt.pos) * 10.0 : -1.0;
        alone_dm_ = static_cast<uint16_t>(h < 0 ? 0xFFFF : (h > 65000 ? 65000 : h));
    }
    log_gal_fix(gps_ok, use);
    return use;
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
