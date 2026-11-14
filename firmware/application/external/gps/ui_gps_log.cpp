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

/* The GPS app's diagnostics log (Log box on the Fix tab): /LOGS/GPS_<date>_
 * <time>.CSV, a new file per app start. Columns: sdcard/GPS/README.TXT
 * ("Diagnostics log"). Rows are built in the lent M4 RAM (GpsScratch::log,
 * not the M0 heap or stack) wherever the event happens on the UI thread,
 * and written only from the display frame handler: a few pieces per frame,
 * each inside one 512-byte sector (FatFs's own buffer), f_sync every 3 s.
 * A write error closes the file, counts it, shows "LOG ERROR" on the Fix tab
 * and retries every 10 s; nothing halts. The per-second summary also goes
 * to the USB serial shell when `asyncmsg enable` was given there. */

#include "ui_gps.hpp"
#include "gps_nav.hpp"

#include "file_path.hpp"
#include "portapack.hpp"
#include "rtc_time.hpp"
#include "usb_serial_asyncmsg.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace portapack;

namespace ui::external_app::gps {

#if defined(PRALINE)

namespace {
constexpr systime_t kLogSyncTicks = MS2ST(3000);
constexpr systime_t kLogRetryTicks = MS2ST(10000);
constexpr int kLogPiecesPerFrame = 3;

/* text into a CSV field: no commas, no line ends */
void put_text(Fmt& f, const char* s) {
    for (; s && *s; s++) f.c(*s == ',' ? ';' : (*s == '\r' || *s == '\n' ? ' ' : *s));
}
void put_hex(Fmt& f, uint32_t v, int digits) {
    static const char* const hex = "0123456789ABCDEF";
    for (int d = digits - 1; d >= 0; d--) f.c(hex[(v >> (4 * d)) & 15u]);
}
uint32_t text_hash(uint32_t h, const char* s) {
    for (; *s; s++) h = (h ^ static_cast<uint8_t>(*s)) * 16777619u;
    return h * 16777619u;
}
int32_t e7(double deg) {
    return static_cast<int32_t>(std::lround(deg * 1e7));
}
}  // namespace

/* the size of the File the open log holds on the heap (read from the ELF for the notes) */
extern const uint32_t gps_log_file_bytes;
const uint32_t gps_log_file_bytes = sizeof(File);

/* rows are queued once the file has been opened the first time (the header
 * and START row come first); while it is closed after an error they wait in
 * the ring (dropped and counted when it is full) */
bool GpsView::log_active() const {
    return log_on_ && mem_ != nullptr && log_started_;
}

/* A row starts with uptime ms, UTC (s; from the satellites once GPS time is
 * known, else RTC - Zone) and its type. */
Fmt GpsView::log_row(const char* type) {
    Fmt f{mem_->log.row, sizeof(mem_->log.row) - 2};
    f.u(static_cast<uint32_t>(chTimeNow())).c(',').u(static_cast<uint32_t>(gps_to_unix(now_gps()))).c(',').s(type);
    return f;
}

void GpsView::log_end(const Fmt& f) {
    log_push(mem_->log, mem_->log.row, f.len());
}

void GpsView::log_header() {
    static const char* const kHead[] = {
        "ms,utc,type,fields (one row type per line below; README.TXT 'Diagnostics log')",
        "#START,fw,lna,vga,amp,ant_bias,zone_min,pos_valid,pos_src,pos_age_s,pos_acc_m,lat_e7,lon_e7,alt_m,alm_n,alm_age_dx10,eph_n,tle_n,rtc_local,rtc_utc,heap_free,scratch,method,tab,file",
        "#SEC,job,done,tracking,search,pgpr_state,level,confirming,done_prn,total_prn,pct,rms_x10,clip_pm,unit_cyc,units,seq,dt_ab_ms,solving,solve_kind,solve_pending,fix_ok,fix_mode,fix_tried,heard,cand,est_m,ttff_s,ttff_coarse_s,tracked,synced,eph,gps_time,gps_off_ms,heap_free,log_rows,log_dropped,log_errs,cached,cache_errs,n_eph,n_alm,ant",
        "#M4,cyc_avg,cyc_max,buffers,missed,active,handovers,handover_age_ms,sf_write,meas_seq,level_x10,dc_i_x10,dc_q_x10,clip_pm",
        "#CH,ch,prn,state,mode,flags,cn0_x10,pld_x100,doppler_x10,code_x64,code_err_x1000,lock_ms,bits,words_ok,parity_errs,subframes,tow,reacq,sweeps,half_fixes",
        "#SR,seq,search,k_ms,prn,searched,detected,once,refined,metric_x100,thr_x100,cn0_x10,doppler,code_x1000,verdict,b_metric_x100,b_doppler,b_code_x1000",
        "#HO,handovers,age_ms,prns",
        "#REL,prn,after_ms,released,refills (channel freed: detection never verified)",
        "#ADD,prn,el_deg,code_x1000,doppler,clock_hz,windows,fix_age_ms,orbit,aided_starts (aided search: newly risen satellite)",
        "#SF,prn,ch,sfid,page,tow,w1..w10 (24-bit data words hex)",
        "#EPH,event,prn,have,fresh,in_table,toe_age_s,iode",
        "#MEAS,seq,sample,missed,n,prn:flags:tx_ms:tx_frac_hex:doppler:cn0_x10 ...",
        "#SOLVE,kind,n,prns,t_offset_s,prior_lat_e7,prior_lon_e7,max_rms_m,max_dt_s,alm_age_dx10,snapshot_age_ms",
        "#FIX,kind,ok,why,used,dropped,rms_m_x10,hdop_x10,vdop_x10,clock_m,time_corr_ms_or_drift_ppb,lat_e7,lon_e7,alt_m,speed_cm_s,masked,trop_m_x10,iono_m_x10,iono_src",
        "#IONO,source (none / air / cache),age_h,a0..a3 (x1e9 s ...),b0..b3",
        "#SCREEN,tab,fix rows 0-10 joined by |",
        "#HEARD,heard rows (status | list)",
        "#CLK,event (tow / pvt: GPS time in use; set: RTC written by Set clock),gps_minus_rtc_ms,rtc_utc,gps_utc,zone_min",
        "#ERR,what,detail",
        "#LOG,event,errors (the log file reopened after the Log box was ticked again)",
    };
    for (const char* h : kHead) log_push(mem_->log, h, std::strlen(h));
}

void GpsView::log_start_row() {
    Fmt f = log_row("START");
    char lbl[8];
    std::memcpy(lbl, VERSION_STRING, std::min(sizeof(lbl) - 1, std::strlen(VERSION_STRING)));
    lbl[std::min(sizeof(lbl) - 1, std::strlen(VERSION_STRING))] = 0;
    f.c(',').s(lbl).c(',').u(static_cast<uint32_t>(receiver_model.lna())).c(',').u(static_cast<uint32_t>(receiver_model.vga())).c(',').u(receiver_model.rf_amp() ? 1u : 0u);
    f.c(',').u(portapack::get_antenna_bias() ? 1u : 0u).c(',').d(utc_off_min_).c(',').u(pos_valid_ ? 1u : 0u);
    if (pos_valid_) {
        f.c(',').u(pos_.source).c(',').d(saved_position::age_s(pos_.time, saved_position::now_fat())).c(',').u(pos_.acc_m).c(',').d(pos_.lat_e7).c(',').d(pos_.lon_e7).c(',').d(pos_.alt_m);
    } else {
        f.s(",,,,,,");
    }
    f.c(',').u(n_alm_).c(',').d(static_cast<int32_t>(std::lround(alm_age_days() * 10))).c(',').u(n_eph_).c(',').u(n_tle_);
    const auto t = rtc_time::now();
    f.c(',').u(t.year()).d2(t.month()).d2(t.day()).d2(t.hour()).d2(t.minute()).d2(t.second());
    f.c(',').u(static_cast<uint32_t>(now_unix()));
    size_t heap = 0;
    chHeapStatus(nullptr, &heap);
    f.c(',').u(static_cast<uint32_t>(heap)).c(',').u(static_cast<uint32_t>(sizeof(GpsScratch)));
    f.c(',').u(view_fix_.method()).c(',').u(tab_view_.selected()).c(',').s(log_name_);
    log_end(f);
}

/* from the display frame handler: open, per-second rows, screen changes,
 * a few pieces to the card, sync */
void GpsView::log_frame() {
    if (!log_on_ || !mem_) {
        if (log_file_) log_close();
        return;
    }
    const systime_t now = chTimeNow();
    if (!log_file_ && !loading() && now - log_retry_t_ >= kLogRetryTicks) log_open();
    if (!log_started_) return;
    if (now - log_sec_t_ >= MS2ST(1000)) {
        log_sec_t_ = now;
        log_second();
    }
    log_screen();
    if (!log_file_) return;
    for (int k = 0; k < kLogPiecesPerFrame && log_pending(mem_->log); k++) {
        const char* p;
        const size_t n = log_piece(mem_->log, log_pos_, &p);
        const auto r = log_file_->write(p, n);
        if (r.is_error()) {
            log_fail("write");
            return;
        }
        log_taken(mem_->log, n);
        log_pos_ += static_cast<uint32_t>(n);
        log_dirty_ = true;
    }
    if (log_dirty_ && now - log_sync_t_ >= kLogSyncTicks) {
        log_sync_t_ = now;
        log_dirty_ = false;
        if (log_file_->sync().is_valid()) log_fail("sync");
    }
}

/* LOGS/GPS_<yyyymmdd>_<hhmmss>.CSV (RTC time), one per app start; the same
 * file is reopened (appended) after an error */
void GpsView::log_open() {
    log_retry_t_ = chTimeNow();
    const char* const prev_err = log_err_;              /* set when the file was closed by an error */
    void* const m = chHeapAlloc(nullptr, sizeof(File)); /* not operator new: it halts on a short heap */
    if (!m) {
        log_err_ = "no heap";
        log_errs_++;
        return;
    }
    log_file_ = new (m) File{};
    if (!log_name_[0]) {
        const auto t = rtc_time::now();
        Fmt n{log_name_, sizeof(log_name_)};
        n.s("LOGS/GPS_").u(t.year()).d2(t.month()).d2(t.day()).c('_').d2(t.hour()).d2(t.minute()).d2(t.second()).s(".CSV");
    }
    ensure_directory(logs_dir);
    const auto e = log_file_->append(std::filesystem::path{log_name_, log_name_ + std::strlen(log_name_)});
    if (e.is_valid()) {
        log_fail("open");
        return;
    }
    log_pos_ = static_cast<uint32_t>(log_file_->size());
    log_err_ = nullptr;
    if (!log_started_) {
        log_started_ = true;
        log_header();
        log_start_row();
    } else if (prev_err) { /* after a card error */
        Fmt f = log_row("ERR");
        f.s(",reopened after ").s(prev_err).s(",errors ").u(log_errs_);
        log_end(f);
    } else { /* the Log box was unticked and ticked again: not an error */
        Fmt f = log_row("LOG");
        f.s(",reopened,errors ").u(log_errs_);
        log_end(f);
    }
}

void GpsView::log_close() {
    if (!log_file_) return;
    log_file_->~File(); /* f_close: syncs */
    chHeapFree(log_file_);
    log_file_ = nullptr;
    log_dirty_ = false;
}

void GpsView::log_fail(const char* what) {
    log_err_ = what;
    log_errs_++;
    log_close();
    log_retry_t_ = chTimeNow();
    view_fix_.show();
}

void GpsView::log_second() {
    const auto& st = status_;
    {
        Fmt f = log_row("SEC");
        f.c(',').u(job_).c(',').u(done_).c(',').u(tracking_).c(',').u(search_kind_).c(',').u(st.state).c(',').u(st.level).c(',').u(st.confirming);
        f.c(',').u(st.done_prn).c(',').u(st.total_prn).c(',').u(static_cast<uint32_t>(progress_pct())).c(',').u(st.rms_x10).c(',').u(st.clip_permille);
        f.c(',').u(st.unit_cycles).c(',').u(st.units).c(',').u(st.seq).c(',').d(status_dt_ms_);
        f.c(',').u(solving() ? 1u : 0u).c(',').u(solve_kind_).c(',').u(solve_pending_).c(',').u(fix_ok_).c(',').u(fix_mode_).c(',').u(fix_tried_);
        f.c(',').u(fix_heard_).c(',').u(fix_cand_).c(',').u(fix_est_m_).c(',').u(ttff_s_).c(',').u(ttff_coarse_s_);
        f.c(',').u(auto_tracked_).c(',').u(auto_synced_).c(',').u(auto_eph_).c(',').u(gps_time_ok_).c(',').d(static_cast<int32_t>(std::lround(gps_off_s_ * 1000.0)));
        size_t heap = 0;
        chHeapStatus(nullptr, &heap);
        f.c(',').u(static_cast<uint32_t>(heap)).c(',').u(mem_->log.rows).c(',').u(mem_->log.dropped).c(',').u(log_errs_);
        f.c(',').u(cache_written_).c(',').u(cache_errs_).c(',').u(n_eph_).c(',').u(n_alm_).c(',').u(portapack::get_antenna_bias() ? 1u : 0u);
        log_end(f);
        mem_->log.row[f.len()] = 0;
        UsbSerialAsyncmsg::asyncmsg(static_cast<const char*>(mem_->log.row)); /* only with `asyncmsg enable` and a client connected */
    }
    if (!trk_sh_ || !tracking_) return;
    const gps_trk::Shared& sh = *trk_sh_;
    {
        Fmt f = log_row("M4");
        f.c(',').u(sh.cycles_avg).c(',').u(sh.cycles_max).c(',').u(sh.buffers).c(',').u(sh.missed).c(',').u(sh.active);
        f.c(',').u(sh.handovers).c(',').u(sh.handover_age_ms).c(',').u(sh.sf_write).c(',').u(sh.meas_seq);
        f.c(',').u(sh.level_x10).c(',').d(sh.dc_i_x10).c(',').d(sh.dc_q_x10).c(',').u(sh.clip_pm);
        log_end(f);
    }
    /* channels the loops gave back (unconfirmed detections that never verified) */
    while (log_rel_seen_ != sh.released) {
        if (static_cast<uint16_t>(sh.released - log_rel_seen_) > 8) log_rel_seen_ = static_cast<uint16_t>(sh.released - 8);
        const uint16_t k = log_rel_seen_++;
        Fmt f = log_row("REL");
        f.c(',').c('G').d2(sh.rel_prn[k & 7u]).c(',').u(sh.rel_ms[k & 7u]).c(',').u(sh.released).c(',').u(sh.refills);
        log_end(f);
    }
    const AutoState& a = mem_->autos;
    for (int i = 0; i < gps_trk::kMaxCh; i++) {
        const gps_trk::ChInfo& c = a.ch[i];
        if (!c.prn) continue;
        Fmt f = log_row("CH");
        f.c(',').u(static_cast<uint32_t>(i)).c(',').u(c.prn).c(',').u(c.state).c(',').u(c.mode).c(',');
        f.s(c.flags & gps_trk::F_PLL ? "L" : "-").s(c.flags & gps_trk::F_BIT ? "B" : "-").s(c.flags & gps_trk::F_FRAME ? "T" : "-").s(c.flags & gps_trk::F_VERIFIED ? "V" : "-");
        f.s(c.flags & gps_trk::F_EVER ? "E" : "-").s(c.flags & gps_trk::F_RELEASED ? "R" : "-").s(c.flags & gps_trk::F_AIDED ? "A" : "-");
        f.c(',').u(c.cn0_x10).c(',').d(c.pld_x100).c(',').d(static_cast<int32_t>(std::lround(c.doppler_hz * 10.0f))).c(',').u(c.code_x64);
        f.c(',').d(static_cast<int32_t>(std::lround(c.code_err * 1000.0f))).c(',').u(c.lock_ms).c(',').u(c.bits).c(',').u(c.words_ok).c(',').u(c.parity_errs);
        f.c(',').u(c.subframes).c(',').u(c.tow_s).c(',').u(c.reacq).c(',').u(c.sweeps).c(',').u(c.half_fixes);
        log_end(f);
    }
    /* ephemerides held, every 5 s */
    if (++log_eph_tick_ >= 5) {
        log_eph_tick_ = 0;
        Fmt f = log_row("EPH");
        f.s(",held,,,,,,");
        for (const auto& c : a.col) {
            if (!c.prn) continue;
            const Orbit& o = mem_->orbits[c.prn - 1];
            f.c(' ').c('G').d2(c.prn).c(':').u(c.have).c(':').u(c.fresh).c(':').u(o.valid && o.ephemeris ? 1u : 0u);
        }
        log_end(f);
    }
}

/* the Fix tab's rows and the Heard tab's status rows, when they change */
void GpsView::log_screen() {
    const TextLines::Line* fl = view_fix_.lines();
    uint32_t h = 2166136261u;
    for (size_t r = 0; r < view_fix_.rows(); r++) h = text_hash(h, fl[r].text);
    h ^= tab_view_.selected();
    if (h != log_fix_hash_) {
        log_fix_hash_ = h;
        Fmt f = log_row("SCREEN");
        f.c(',').u(tab_view_.selected()).c(',');
        for (size_t r = 0; r < view_fix_.rows(); r++) {
            if (r) f.c('|');
            put_text(f, fl[r].text);
        }
        log_end(f);
    }
    const TextLines::Line* hs = view_heard_.status_lines();
    const TextLines::Line* hl = view_heard_.list_lines();
    uint32_t g = 2166136261u;
    for (size_t r = 0; r < view_heard_.status_rows(); r++) g = text_hash(g, hs[r].text);
    for (size_t r = 0; r < view_heard_.list_rows(); r++) g = text_hash(g, hl[r].text);
    if (g != log_heard_hash_) {
        log_heard_hash_ = g;
        Fmt f = log_row("HEARD");
        f.c(',');
        for (size_t r = 0; r < view_heard_.status_rows(); r++) {
            put_text(f, hs[r].text);
            f.c('|');
        }
        for (size_t r = 0; r < view_heard_.list_rows(); r++) {
            if (!hl[r].text[0]) continue;
            put_text(f, hl[r].text);
            f.c('|');
        }
        log_end(f);
    }
}

/* every search result of a finished search: accepted, and rejected with why */
void GpsView::log_results(const gps_rx::Result* r, uint8_t k_ms) {
    if (!log_active() || !r) return;
    static constexpr uint16_t kThr100[20] = {1775, 1042, 782, 646, 561, 503, 460, 427, 401, 379, 361, 346, 333, 322, 311, 303, 295, 287, 281, 275};
    for (int p = 0; p < 32; p++) {
        const auto& x = r[p];
        if (!x.searched) continue;
        Fmt f = log_row("SR");
        f.c(',').u(status_.seq).c(',').u(search_kind_).c(',').u(k_ms).c(',').u(x.prn ? x.prn : static_cast<uint32_t>(p + 1));
        f.c(',').u(x.searched).c(',').u(x.detected).c(',').u(x.once).c(',').u(x.refined);
        f.c(',').u(static_cast<uint32_t>(x.metric * 100.0f + 0.5f)).c(',').u(k_ms >= 1 && k_ms <= 20 ? kThr100[k_ms - 1] : 0u);
        f.c(',').u(static_cast<uint32_t>(x.cn0 * 10.0f + 0.5f)).c(',').d(static_cast<int32_t>(std::lround(x.doppler))).c(',').u(static_cast<uint32_t>(x.code_phase * 1000.0f));
        static const char* const kWhy[4] = {",rejected: not confirmed by the 2nd snapshot", ",rejected: 2nd snapshot did not find it",
                                            ",rejected: 2nd snapshot Doppler moved > 750 Hz", ",rejected: 2nd snapshot code phase off the prediction"};
        f.s(x.detected ? (req_confirm_ ? ",accepted" : ",detected (unconfirmed: the tracker verifies)")
                       : (x.once ? kWhy[x.reject < 4 ? x.reject : 0] : ",rejected: below threshold"));
        if (x.once) f.c(',').u(static_cast<uint32_t>(x.b_metric * 100.0f + 0.5f)).c(',').d(static_cast<int32_t>(std::lround(x.b_doppler))).c(',').u(static_cast<uint32_t>(x.b_code * 1000.0f));
        log_end(f);
    }
}

void GpsView::log_clock(const char* src) {
    if (clock_logged_ && std::fabs(gps_off_s_ - clock_logged_off_) < 0.1) return; /* once, then on changes over 0.1 s */
    clock_logged_ = true;
    clock_logged_off_ = gps_off_s_;
    if (!log_active()) return;
    Fmt f = log_row("CLK");
    f.c(',').s(src).c(',').d(static_cast<int32_t>(std::lround(gps_off_s_ * 1000.0)));
    f.c(',').u(static_cast<uint32_t>(now_unix())).c(',').u(static_cast<uint32_t>(now_best())).c(',').d(utc_off_min_);
    log_end(f);
}

void GpsView::log_clock_set(double rtc_before, int64_t t) {
    clock_logged_off_ = gps_off_s_;
    if (!log_active()) return;
    Fmt f = log_row("CLK");
    f.s(",set,").d(static_cast<int32_t>(std::lround((static_cast<double>(t) - rtc_before) * 1000.0)));
    f.c(',').u(static_cast<uint32_t>(rtc_before)).c(',').u(static_cast<uint32_t>(t)).c(',').d(utc_off_min_);
    log_end(f);
}

void GpsView::log_aided(int prn, float el, double code, double dop, double clk, int windows, uint32_t age_ms, bool eph) {
    if (!log_active()) return;
    Fmt f = log_row("ADD");
    f.c(',').c('G').d2(prn).c(',').d(static_cast<int32_t>(std::lround(el))).c(',').u(static_cast<uint32_t>(code * 1000.0));
    f.c(',').d(static_cast<int32_t>(std::lround(dop))).c(',').d(static_cast<int32_t>(std::lround(clk))).c(',').u(static_cast<uint32_t>(windows));
    f.c(',').u(age_ms).c(',').s(eph ? "eph" : "alm").c(',').u(trk_sh_ ? trk_sh_->aided_starts : 0u);
    log_end(f);
}

void GpsView::log_handover() {
    if (!log_active() || !trk_sh_) return;
    Fmt f = log_row("HO");
    f.c(',').u(trk_sh_->handovers).c(',').u(trk_sh_->handover_age_ms).c(',');
    for (int i = 0; i < gps_trk::kMaxCh; i++)
        if (trk_sh_->ch[i].prn) f.c('G').d2(trk_sh_->ch[i].prn).c(' ');
    log_end(f);
}

/* subframes the app is about to take from the ring (all ten data words) */
void GpsView::log_subframes() {
    if (!log_active() || !trk_sh_) return;
    const AutoState& a = mem_->autos;
    const uint32_t w = trk_sh_->sf_write;
    uint32_t k = a.sf_read;
    if (w - k > static_cast<uint32_t>(gps_trk::kSfSlots)) {
        Fmt f = log_row("ERR");
        f.s(",subframe ring overrun,lost ").u(w - k - gps_trk::kSfSlots);
        log_end(f);
        k = w - gps_trk::kSfSlots;
    }
    for (; k != w; k++) {
        const gps_trk::SubframeSlot& s = trk_sh_->sf[k % gps_trk::kSfSlots];
        Fmt f = log_row("SF");
        const int id = nav::sf_id(s.words);
        f.c(',').u(s.prn).c(',').u(s.ch).c(',').u(static_cast<uint32_t>(id)).c(',').u((s.tow_s / 30u) % 25u + 1u).c(',').u(s.tow_s);
        for (int j = 0; j < 10; j++) {
            f.c(',');
            put_hex(f, s.words[j], 6);
        }
        log_end(f);
    }
}

void GpsView::log_meas() {
    if (!log_active()) return;
    const gps_trk::MeasSet& m = mem_->autos.meas;
    Fmt f = log_row("MEAS");
    f.c(',').u(m.seq).c(',').u(static_cast<uint32_t>(m.sample / 2048u)).s("x2048,").u(m.missed).c(',').u(m.n);
    for (int i = 0; i < m.n && i < gps_trk::kMaxCh; i++) {
        const gps_trk::MeasCh& c = m.ch[i];
        f.c(',').u(c.prn).c(':').u(c.flags).c(':').u(c.tx_ms).c(':');
        put_hex(f, c.tx_frac, 8);
        f.c(':').d(static_cast<int32_t>(std::lround(c.doppler_hz))).c(':').u(c.cn0_x10);
    }
    log_end(f);
}

void GpsView::log_eph_event(const char* what, int prn) {
    if (!log_active() || prn < 1 || prn > 32) return;
    const AutoState& a = mem_->autos;
    const Orbit& o = mem_->orbits[prn - 1];
    uint8_t have = 0, fresh = 0;
    for (const auto& c : a.col)
        if (c.prn == prn) {
            have = c.have;
            fresh = c.fresh;
        }
    Fmt f = log_row("EPH");
    f.c(',').s(what).c(',').u(static_cast<uint32_t>(prn)).c(',').u(have).c(',').u(fresh).c(',').u(o.valid && o.ephemeris ? 1u : 0u);
    f.c(',').d(static_cast<int32_t>(std::lround(now_gps() - o.toe)));
    log_end(f);
}

/* kind: 0 snapshot (coarse / precise by the orbits), 1 PVT coarse (almanac), 2 PVT precise */
void GpsView::log_solve_start(uint8_t kind) {
    if (!log_active()) return;
    Fmt f = log_row("SOLVE");
    static const char* const kKind[] = {"snapshot", "pvt_coarse", "pvt_precise"};
    double lat = 0, lon = 0, h = 0;
    ecef_to_lla(solve_prior_, lat, lon, h);
    if (kind == 0) {
        f.c(',').s(kKind[0]).s(fix_mode_ == FIX_PRECISE ? "_precise" : "_coarse").c(',').u(static_cast<uint32_t>(solve_n_)).c(',');
        for (int i = 0; i < solve_n_; i++) {
            const Measurement& m = mem_->solve.in[i];
            f.c('G').d2(m.orbit->prn).c(':').u(static_cast<uint32_t>(m.code_phase * 1000.0)).c(' ');
        }
        f.c(',').d(static_cast<int32_t>(std::lround(solve_t_ - std::floor(solve_t_ / 86400.0) * 86400.0)));
        f.c(',').d(e7(lat)).c(',').d(e7(lon)).c(',').u(static_cast<uint32_t>(solve_max_rms_)).c(',').u(static_cast<uint32_t>(solve_max_dt_));
    } else {
        f.c(',').s(kKind[kind < 3 ? kind : 1]).c(',').u(static_cast<uint32_t>(solve_pvt_n_)).c(',');
        for (int i = 0; i < solve_pvt_n_; i++) f.c('G').d2(mem_->autos.pvt.in[i].prn).c(' ');
        f.c(',').d(static_cast<int32_t>(std::lround(solve_pvt_trx_ * 1000.0))).s("ms");
        f.c(',').d(e7(lat)).c(',').d(e7(lon)).c(',').u(solve_pvt_eph_ ? 25u : 0u).c(',');
    }
    f.c(',').d(static_cast<int32_t>(std::lround(alm_age_days() * 10)));
    if (kind == 0) f.c(',').u(static_cast<uint32_t>(std::lround(results_age_s() * 1000.0)));
    log_end(f);
}

void GpsView::log_fix(uint8_t kind, bool ok, const char* why) {
    if (!log_active()) return;
    static const char* const kKind[] = {"snapshot", "pvt_coarse", "pvt_precise"};
    Fmt f = log_row("FIX");
    f.c(',').s(kKind[kind < 3 ? kind : 0]).c(',').u(ok ? 1u : 0u).c(',');
    put_text(f, why);
    if (kind == 0) {
        const Solution& s = solve_out_;
        double lat = 0, lon = 0, h = 0;
        if (ok) ecef_to_lla(s.pos, lat, lon, h);
        f.c(',').u(s.used).s(",,").u(static_cast<uint32_t>(s.resid_rms_m * 10.0)).c(',').u(static_cast<uint32_t>(s.hdop * 10.0f)).s(",,");
        f.d(static_cast<int32_t>(std::lround(s.clock_m))).c(',').d(static_cast<int32_t>(std::lround(s.time_corr_s * 1000.0)));
        f.c(',').d(e7(lat)).c(',').d(e7(lon)).c(',').d(static_cast<int32_t>(std::lround(h))).c(',');
    } else {
        const PvtSolution& p = mem_->pvt;
        double lat = 0, lon = 0, h = 0;
        if (ok) ecef_to_lla(p.pos, lat, lon, h);
        f.c(',').u(p.used).c(',').u(p.dropped).c(',').u(static_cast<uint32_t>(p.resid_rms_m * 10.0)).c(',').u(static_cast<uint32_t>(p.hdop * 10.0f));
        f.c(',').u(static_cast<uint32_t>(p.vdop * 10.0f)).c(',').d(static_cast<int32_t>(std::lround(p.clock_m)));
        f.c(',').d(static_cast<int32_t>(std::lround(p.drift_mps / kC * 1e9)));
        f.c(',').d(e7(lat)).c(',').d(e7(lon)).c(',').d(static_cast<int32_t>(std::lround(h))).c(',').u(static_cast<uint32_t>(norm(p.vel) * 100.0));
        f.c(',').u(p.masked).c(',').u(static_cast<uint32_t>(p.trop_m * 10.0f)).c(',').u(static_cast<uint32_t>(p.iono_m * 10.0f)).c(',').u(iono_src_);
    }
    log_end(f);
}

void GpsView::log_iono() {
    if (!log_active()) return;
    static const char* const kSrc[] = {"none", "air", "cache"};
    const AutoState& a = mem_->autos;
    Fmt f = log_row("IONO");
    f.c(',').s(kSrc[iono_src_ < 3 ? iono_src_ : 0]).c(',');
    if (iono_src_ == 2) f.d(static_cast<int32_t>(std::lround((now_gps() - mem_->iono_cache_t) / 3600.0)));
    if (iono_src_) {
        const float* ia = a.pvt.ion_a;
        const float* ib = a.pvt.ion_b;
        for (int k = 0; k < 4; k++) f.c(',').d(static_cast<int32_t>(std::lround(ia[k] * 1e9 * (k == 0 ? 1.0 : 1.0))));
        for (int k = 0; k < 4; k++) f.c(',').d(static_cast<int32_t>(std::lround(ib[k])));
    }
    log_end(f);
}

void GpsView::log_error(const char* what, const char* detail) {
    if (!log_active()) return;
    Fmt f = log_row("ERR");
    f.c(',');
    put_text(f, what);
    f.c(',');
    put_text(f, detail);
    log_end(f);
}

#endif /* PRALINE */

}  // namespace ui::external_app::gps
