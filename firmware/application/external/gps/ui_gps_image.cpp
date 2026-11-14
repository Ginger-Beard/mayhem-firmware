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

/* The GPS app's baseband images on the SD card (common/gps_image.hpp):
 * /APPS/GPS_L1A.BIN (L1 acquisition) and /APPS/GPS_L1T.BIN (L1 tracking),
 * built with this firmware and copied with the .ppma apps. The app loads
 * one into m4_code (as the .ppma loader does with a bundled image) and
 * starts it. A switch runs in the frame handler (UI thread), with the File
 * on the heap: ~60 KB from the card, ~0.1-0.2 s. What the app keeps in M4
 * RAM (the lent block, the search results, the tracking output) is in the
 * block the images leave alone (gps_keep.hpp), so nothing is lost. */

#include "ui_gps.hpp"

#include "baseband_api.hpp"
#include "memory_map.hpp"
#include "portapack.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

using namespace portapack;

namespace ui::external_app::gps {

#if defined(PRALINE)

namespace {
constexpr systime_t kImageRetryTicks = MS2ST(5000); /* a failed load is not retried every frame */
}

gps_image::Check GpsView::load_image(gps_image::Id id) {
    const systime_t t0 = chTimeNow();
    if (solver_ && solve_kind_ == 1 && solve_gal_n_ && mem_) { /* a GPS + Galileo solve reads the tracking image's RAM: stop it first */
        mem_->autos.pvt.cancel = 1;
        finish_solve(); /* waits for it; logged as cancelled */
    }
    baseband::shutdown(); /* the M4 runs from m4_code: stop it before overwriting (no-op when none runs) */
    image_ = gps_image::ID_NONE;
    image_ready_ = false;
    gal_codes_ = nullptr; /* they lived in the old image's RAM */
    gal_result_ = nullptr;
    gal_trk_sh_ = nullptr;
    gal_nav_ = nullptr;
    gps_image::Check c = gps_image::OK;
    uint32_t size = 0;
    void* const m = chHeapAlloc(nullptr, sizeof(File)); /* not operator new: it halts on a short heap */
    if (!m) {
        c = gps_image::READ_ERROR;
    } else {
        File* f = new (m) File{};
        if (f->open(gps_image::path(id)).is_valid()) {
            c = gps_image::NO_CARD_OR_FILE;
        } else {
            gps_image::Header h{};
            const auto r = f->read(&h, sizeof(h));
            if (!r || r.value() != sizeof(h))
                c = gps_image::NOT_AN_IMAGE;
            else
                c = gps_image::check_header(h, id, VERSION_MD5, memory::map::m4_code.size());
            if (c == gps_image::OK) {
                auto* const dst = reinterpret_cast<uint8_t*>(memory::map::m4_code.base());
                uint32_t sum = 0;
                for (uint32_t done = 0; done < h.size;) {
                    const uint32_t n = std::min<uint32_t>(std::filesystem::max_file_block_size, h.size - done);
                    const auto rr = f->read(dst + done, n);
                    if (!rr || rr.value() != n) {
                        c = gps_image::READ_ERROR;
                        break;
                    }
                    sum = gps_image::checksum_add(sum, dst + done, n);
                    done += n;
                }
                if (c == gps_image::OK && sum != h.checksum) c = gps_image::BAD_CHECKSUM;
                size = h.size;
            }
        }
        f->~File();
        chHeapFree(m);
    }
    if (c == gps_image::OK) {
        baseband::run_prepared_image(memory::map::m4_code.base());
        image_ = id;
    } else {
        image_fail_t_ = chTimeNow();
    }
    image_err_ = c;
    image_err_id_ = id;
    img_last_id_ = id;
    img_last_check_ = c;
    img_last_size_ = size;
    img_last_ms_ = static_cast<uint32_t>(chTimeNow() - t0);
    img_loads_++;
    log_image();
    return c;
}

void GpsView::want_image(gps_image::Id id) {
    if (image_ == id && image_err_ == gps_image::OK) return;
    image_want_ = id;
}

/* frame handler: a due switch; a failed one waits kImageRetryTicks */
void GpsView::image_step() {
    if (image_want_ == gps_image::ID_NONE || image_want_ == image_) return;
    if (image_err_ != gps_image::OK && image_err_id_ == image_want_ && chTimeNow() - image_fail_t_ < kImageRetryTicks) return;
    const gps_image::Id id = image_want_;
    image_want_ = gps_image::ID_NONE;
    if (load_image(id) != gps_image::OK) { /* what waited for it cannot run */
        pending_search_ = false;
        pending_track_mask_ = 0;
        if (job_ != JOB_NONE) {
            job_ = JOB_NONE;
            awaiting_seq_ = false;
            done_ = false;
            tracking_ = false;
            want_job_ = JOB_NONE;
        }
        image_fallback_ram(); /* no image at all yet: Passes still works */
        view_fix_.show();
        view_heard_.show();
    }
}

/* the wanted image reported for the first time */
void GpsView::on_image_ready() {
    image_ready_ = true;
    if (pending_search_ && image_ == gps_image::ID_L1A) {
        pending_search_ = false;
        start_search(pending_job_, pending_kms_, pending_full_);
        view_heard_.show();
        view_fix_.show();
    }
    if (image_ == gps_image::ID_L1T) gal_reset(); /* a new tracking run: the Galileo search starts over */
    if (pending_track_mask_ && image_ == gps_image::ID_L1T) {
        send_track(pending_neg_dop_); /* mirrored by the Dopplers (mirror_check): negated */
        pending_neg_dop_ = false;
    }
}

/* Autonomous: the detections of the acquisition image's search to the
 * tracking loops, with the age of the result snapshot on the app's clock
 * (the tracking image propagates the code phases over it) */
void GpsView::send_track(bool neg_dop) {
    GpsRxConfigMessage m;
    m.cmd = GpsRxConfigMessage::CMD_TRACK;
    m.neg_dop = neg_dop ? 1 : 0;
    m.prn_mask = pending_track_mask_;
    m.confirm = req_confirm_ ? 1 : 0;
    m.max_ch = 8; /* M4: ~62 % with 8 channels on the device */
    const uint32_t now = static_cast<uint32_t>(chTimeNow());
    m.age_ms = results_ms_ok_ ? now - results_ms_ : 0;
    m.conj = iq_conj_ ^ iq_trk_flip_;
    track_mask_ = pending_track_mask_;
    track_t_ = chTimeNow();
    iq_skip_logged_ = false; /* this hand-over's mirrored-spectrum check */
    pending_track_mask_ = 0;
    baseband::gps_rx_config(m);
}

/* the samples arrive mirrored (I/Q orientation; one device run on 14 Nov): the
 * image negates Q from the next command on. The search's evidence (2nd
 * snapshot) sets the acquisition image's correction; the tracking image's flip
 * lasts for that hand-over only (each image start sets the stream up anew, and
 * the search may have seen it the other way). Logged as an IQ row. */
void GpsView::iq_flip(bool trk, uint32_t n, const char* src) {
    if (trk)
        iq_trk_flip_ ^= 1u;
    else
        iq_conj_ ^= 1u;
    iq_flips_++;
    if (!log_active()) return;
    Fmt f = log_row("IQ");
    f.s(src ? src : (trk ? ",tracking," : ",search,")).u(n).c(',').u(trk ? iq_conj_ ^ iq_trk_flip_ : iq_conj_).c(',').u(iq_flips_);
    log_end(f);
}

/* Autonomous: strong satellites handed over, but no channel found the common
 * code offset (or verified) in kIqTrkWait: the tracking image may see the
 * spectrum mirrored (each image start sets up the stream anew). Once per
 * hand-over: flip, and hand the same detections over again, Dopplers negated. */
void GpsView::iq_trk_check() {
    if (iq_trk_retried_ || !trk_sh_ || !results_ || !track_mask_) return;
    if (chTimeNow() - track_t_ < kIqTrkWait) return;
    if (iq_flips_ >= kIqMaxFlips) {
        log_iq_skip("skipped_tracking", "all corrections of this session used", iq_flips_);
        return;
    }
    /* only satellites whose Doppler tells the two orientations apart: one near
     * 0 Hz (incl. the clock term) tracks either way, so it neither proves the
     * orientation right nor vetoes the others' re-hand-over */
    float best = 0;
    for (int p = 0; p < 32; p++)
        if (((track_mask_ >> p) & 1u) && std::fabs(results_[p].doppler) >= kIqMinDop && results_[p].cn0 > best) best = results_[p].cn0;
    if (best < kIqTrkCn0) { /* weak ones may just need longer */
        log_iq_skip("skipped_tracking", "best C/N0 (Doppler over 400 Hz) below 35 (x10)", static_cast<uint32_t>(best * 10.0f + 0.5f));
        return;
    }
    for (int i = 0; i < gps_trk::kMaxCh; i++) {
        const gps_trk::ChInfo& c = trk_sh_->ch[i];
        if (!(c.flags & (gps_trk::F_EVER | gps_trk::F_VERIFIED)) || c.prn < 1 || c.prn > 32) continue;
        if (std::fabs(results_[c.prn - 1].doppler) < kIqMinDop) continue;
        log_iq_skip("skipped_tracking", "a channel verified (prn)", c.prn);
        return;
    }
    iq_trk_retried_ = true;
    iq_flip(true, static_cast<uint32_t>(best + 0.5f));
    pending_track_mask_ = track_mask_;
    send_track(true);
}

#endif /* PRALINE */

}  // namespace ui::external_app::gps
