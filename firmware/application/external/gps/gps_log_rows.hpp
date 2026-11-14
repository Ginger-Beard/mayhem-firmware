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

#ifndef __GPS_LOG_ROWS_H__
#define __GPS_LOG_ROWS_H__

/* The diagnostics log's header (column names, one line per row type) and the
 * rows that need no GpsView state, header only so the host test checks them
 * against the header: Galileo channels (GTRK, GCH), channel events (EV) and
 * the decision when the Galileo rows are written. ui_gps_log.cpp writes the
 * header straight from flash (gps_log.hpp log_head_piece) before any queued
 * row, so the header no longer fills the 4 KB ring at log start. */

#include <cmath>
#include <cstdint>

#include "gps_fmt.hpp"
#include "gps_trk_shared.hpp"
#include "gal_e1.hpp"

namespace ui::external_app::gps {

inline constexpr char kLogHead[] =
    "ms,utc,type,fields (one row type per line below; README.TXT 'Diagnostics log')\r\n"
    "#START,fw,lna,vga,amp,ant_bias,zone_min,pos_valid,pos_src,pos_age_s,pos_acc_m,lat_e7,lon_e7,alt_m,alm_n,alm_age_dx10,eph_n,tle_n,rtc_local,rtc_utc,heap_free,scratch,method,tab,file,e1,sys_view,iq_conj\r\n"
    "#SEC,job,done,tracking,search,pgpr_state,level,confirming,done_prn,total_prn,pct,rms_x10,clip_pm,unit_cyc,units,seq,dt_ab_ms,solving,solve_kind,solve_pending,fix_ok,fix_mode,fix_tried,heard,cand,est_m,ttff_s,ttff_coarse_s,tracked,synced,eph,gps_time,gps_off_ms,heap_free,log_rows,log_dropped,log_errs,cached,cache_errs,n_eph,n_alm,ant,iq_conj,xs_found,xs_ms,e1,gal_sh\r\n"
    "#M4,cyc_avg,cyc_max,buffers,missed,active,handovers,handover_age_ms,sf_write,meas_seq,level_x10,dc_i_x10,dc_q_x10,clip_pm\r\n"
    "#CH,ch,prn,state,mode,flags,cn0_x10,pld_x100,doppler_x10,code_x64,code_err_x1000,lock_ms,bits,words_ok,parity_errs,subframes,tow,reacq,sweeps,half_fixes\r\n"
    "#EV,ch,prn,event (start / lost / found / coast / coast_end / released / given_up),why (lost: cn0 / coast / pullin / nolock / search),cn0_x10,lock_ms,reacq,keep (bits / frame kept across the loss; ok / bad once bit sync checked it)\r\n"
    "#SR,seq,search,k_ms,prn,searched,detected,once,refined,metric_x100,thr_x100,cn0_x10,doppler,code_x1000,verdict,b_metric_x100,b_doppler,b_code_x1000\r\n"
    "#HO,handovers,age_ms,prns\r\n"
    "#REL,prn,after_ms,released,refills (channel freed: detection never verified)\r\n"
    "#ADD,prn,el_deg,code_x1000,doppler,clock_hz,windows,fix_age_ms,orbit,aided_starts (aided search: newly risen satellite)\r\n"
    "#SF,prn,ch,sfid,page,tow,w1..w10 (24-bit data words hex)\r\n"
    "#EPH,event,prn,have,fresh,in_table,toe_age_s,iode\r\n"
    "#MEAS,seq,sample,missed,n,prn:flags:tx_ms:tx_frac_hex:doppler:cn0_x10 ...\r\n"
    "#SOLVE,kind,n,prns,t_offset_s,prior_lat_e7,prior_lon_e7,max_rms_m,max_dt_s,alm_age_dx10,snapshot_age_ms\r\n"
    "#FIX,kind,ok,why,used,dropped,rms_m_x10,hdop_x10,vdop_x10,clock_m,time_corr_ms_or_drift_ppb,lat_e7,lon_e7,alt_m,speed_cm_s,masked,trop_m_x10,iono_m_x10,iono_src,systems (G / GE),galileo_used\r\n"
    "#IONO,source (none / air / cache),age_h,a0..a3 (x1e9 s ...),b0..b3\r\n"
    "#SCREEN,tab,fix rows 0-10 joined by |\r\n"
    "#HEARD,heard rows (status | list)\r\n"
    "#UI,event (getfix / stop / method / tab / e1 / log / setclock / savepos / heard_search / heard_stop / ant_power),detail\r\n"
    "#CLK,event (tow / pvt: GPS time in use; set: RTC written by Set clock),gps_minus_rtc_ms,rtc_utc,gps_utc,zone_min\r\n"
    "#ERR,what,detail\r\n"
    "#IQ,source (search / tracking: Q negated; skipped_search / skipped_tracking: the check could not run),evidence (search: candidates mirrored; tracking: best C/N0; skipped: the value),iq_conj now (skipped: why),flips\r\n"
    "#LOG,event,errors (the log file reopened after the Log box was ticked again)\r\n"
    "#IMG,image,result,bytes,load_ms,loads (a baseband image loaded from /APPS)\r\n"
    "#SYS,event (view / sky),view,file (1 read 2 missing 3 empty),age_dx10,sats,E up/total,C up/total,R up/total,S up/total,bad_lines\r\n"
    "#GAL,prn,el_deg,pred_code_chips_x100,pred_doppler_hz,clock_term_hz,fix_age_ms,result,metric_x100,cn0_x10,offset_half_samples,side_db_x10,found_doppler,blocks,m4_kcycles,search_s,handover\r\n"
    "#GTRK,max_ch,active,handovers,refused,preempted,reacq_searches,pool_x10,pool_budget_x10,last_ho_prn,last_ho (Galileo channels, 1/s)\r\n"
    "#GCH,ch,prn,state,mode,flags,cn0c_x10,cn0b_x10,pld_x100,doppler_x10,code_x16,code_err_x1000,lock_ms,side_e_x100,side_l_x100,side_jumps,cs_phase,cs_errs,cs_syncs,fl_fixes,reacq,epochs\r\n"
    "#GEV,prn,event (start / cs / cs_lost / side / lost / found / released),a,b (Galileo channel events; see README)\r\n"
    "#GNAV,prn,word_type,crc_ok,alert,pages_ok,pages_bad,gst_week,gst_tow,word (128 bits hex; an I/NAV page of a tracked Galileo satellite)\r\n"
    "#GEPH,event (new / same / iod_wait / bad_svid / bad_value / ggto / ggto_none),prn,iodnav,toe_sow,sisa,e1b_hs,e1b_dvs,bgd_e1e5b_ps,af0_ns,sqrt_a_x1000,built,rejected,iod_waits (or for ggto: a0_ps,a1_x1e15,t0g_h,wn0g,ggto_ps_now)\r\n"
    "#GMEAS,seq,sample,n,prn:why:sym:frac_hex:tx_ms:t_agree:cn0c_x10 ... (why: 0 used 1 idle 2 no CS 3 weak 4 no page time 5 time unconfirmed 6 no ephemeris 7 PVT full)\r\n"
    "#GFIX,mode (ggto / isb),gps_ok,mix_ok,used,gate,n_gps_used,n_gal_used,isb_ps,ggto_ps,rms_gps_only_x10,rms_mix_x10,gal_rms_x10,gps_rms_in_mix_x10,hdop_gps_x10,hdop_mix_x10,moved_dm,prn:resid_dm ...,alone_ok,alone_used,alone_moved_dm (GPS + Galileo against GPS alone)\r\n";
inline constexpr uint32_t kLogHeadLen = sizeof(kLogHead) - 1;

/* The Galileo rows (GTRK, GCH) are written once a second while logging, with
 * the E1 box ticked and the tracking image's Galileo block known (it comes
 * with each status of the tracking image), while the loops run. Without E1
 * there are none: SEC carries e1 and gal_sh to tell which was missing. */
inline bool gal_rows_wanted(bool log_active, bool gal_on, const void* gal_sh, bool tracking) {
    return log_active && gal_on && gal_sh != nullptr && tracking;
}

/* GTRK fields after the type */
inline void row_gtrk(Fmt& f, const gal_acq::TrkShared& t) {
    f.c(',').u(t.max_ch).c(',').u(t.active).c(',').u(t.handovers).c(',').u(t.refused).c(',').u(t.preempted).c(',').u(t.reacq_searches);
    f.c(',').u(t.pool_x10).c(',').u(t.pool_budget_x10).c(',').u(t.last_ho_prn).c(',').u(t.last_ho_result);
}

/* GCH fields after the type: Galileo channel i */
inline void row_gch(Fmt& f, int i, const gal_acq::TrkInfo& c) {
    const auto r = [](float v) { return static_cast<int32_t>(std::lround(v)); };
    f.c(',').u(static_cast<uint32_t>(i)).c(',').c('E').d2(c.prn).c(',').u(c.state).c(',').u(c.mode).c(',');
    f.s(c.flags & gal_acq::TF_PLL ? "L" : "-").s(c.flags & gal_acq::TF_CS ? "C" : "-").s(c.flags & gal_acq::TF_EVER ? "E" : "-");
    f.s(c.flags & gal_acq::TF_SIDE ? "S" : "-").s(c.flags & gal_acq::TF_REACQ ? "Q" : "-");
    f.c(',').u(c.cn0c_x10).c(',').u(c.cn0b_x10).c(',').d(c.pld_x100).c(',').d(r(c.doppler_hz * 10.0f)).c(',').u(c.code_x16);
    f.c(',').d(r(c.code_err * 1000.0f)).c(',').u(c.lock_ms).c(',').u(c.side_e_x100).c(',').u(c.side_l_x100).c(',').u(c.side_jumps);
    f.c(',').u(c.cs_phase).c(',').u(c.cs_errs).c(',').u(c.cs_syncs).c(',').u(c.fl_fixes).c(',').u(c.reacq).c(',').u(c.epochs);
}

/* A channel event from two looks at the channel table (the app polls it ~5
 * times a second): the previous PRN / state / re-acquisition count / mode and
 * the current entry. nullptr: nothing new. One event per look; a new PRN in
 * the slot (or a re-acquisition count that went down: a new tracker) is
 * "start". */
inline const char* ch_event(uint8_t prn0, uint8_t state0, uint8_t reacq0, uint8_t mode0, const gps_trk::ChInfo& c) {
    if (!c.prn) return nullptr;
    if (c.prn != prn0 || c.reacq < reacq0) return "start";
    if (c.reacq > reacq0) return "lost";
    if (c.state == gps_trk::TRK_FAILED && state0 != gps_trk::TRK_FAILED) return (c.flags & gps_trk::F_RELEASED) ? "released" : "given_up";
    if (state0 == gps_trk::TRK_SEARCH && (c.state == gps_trk::TRK_PULLIN || c.state == gps_trk::TRK_TRACK)) return "found";
    if (c.mode == 5 && mode0 != 5 && c.state == gps_trk::TRK_TRACK) return "coast";
    if (mode0 == 5 && c.mode != 5 && c.state == gps_trk::TRK_TRACK) return "coast_end";
    return nullptr;
}

/* EV fields after the type */
inline void row_ev(Fmt& f, int ch, const gps_trk::ChInfo& c, const char* event) {
    static const char* const kWhy[] = {"", "cn0", "coast", "pullin", "nolock", "search"};
    f.c(',').u(static_cast<uint32_t>(ch)).c(',').c('G').d2(c.prn).c(',').s(event).c(',');
    if (event[0] == 'l') f.s(kWhy[c.why < 6 ? c.why : 0]); /* lost */
    f.c(',').u(c.cn0_x10).c(',').u(c.lock_ms).c(',').u(c.reacq).c(',');
    f.s(c.keep & gps_trk::KEEP_BITS ? "B" : "-").s(c.keep & gps_trk::KEEP_FRAME ? "F" : "-");
    f.s(c.keep & gps_trk::KEEP_OK ? "ok" : (c.keep & gps_trk::KEEP_BAD ? "bad" : ""));
}

}  // namespace ui::external_app::gps

#endif /*__GPS_LOG_ROWS_H__*/
