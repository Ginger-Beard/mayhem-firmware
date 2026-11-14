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

#ifndef __GAL_TRK_H__
#define __GAL_TRK_H__

/* Galileo E1 tracking channel for the GPS app's tracking image (header only,
 * free of ChibiOS: the host tests run this code). Receive only. Included by
 * gps_trk.hpp (the channel shares the GPS tracker's per-block tables and
 * carrier wipe); include gps_trk.hpp, not this file.
 *
 * Signal: E1B (data, one I/NAV symbol per 4 ms code) and E1C (pilot, CS25
 * secondary code, one chip per 4 ms code) on the same carrier phase, each a
 * 4092-chip code with BOC(1,1). The 1.75 MHz front end keeps the two BOC main
 * lobes only: the correlation peak is narrow (|R|^2 0.33 at +-0.25 chip) with
 * side peaks ~3.8 dB down at +-0.75 chip.
 *
 * Correlator: the GPS tracker's ("LUT-bitstream"): per 32-sample block a
 * 32-bit replica word read from the code's BOC half-chip stream at the
 * prompt's half-chip phase (8184 half chips per 8192 samples: the same code
 * rate as C/A at 1 sample per half chip), sum = S - 2 x (8 table look-ups).
 * Per block: E1C prompt, E1B prompt, every other block the E1C early - late
 * pair at +-0.5 half chip (+-0.25 chip; one of the two is the prompt word, the
 * other the word one half chip off, chosen by the phase fraction), every
 * fourth block the very early / very late taps at +-1.5 half chips (the
 * side-peak monitor) with the prompt of the same blocks. Blocks where a
 * replica phase repeats or the 4 ms epoch ends are split exactly with bit
 * masks. The sample-quantised replica averages over the phase fraction
 * within an epoch, so the taps behave as at their nominal offsets.
 *
 * Loops per 4 ms epoch: FLL on the four ~1 ms quarter sums of the E1C prompt
 * (pull-in range +-250 Hz, no secondary code or data inside a code period),
 * then a Costas PLL with FLL assist; once the CS25 secondary code is found
 * on the prompt signs it is wiped and the PLL runs 4-quadrant (the pilot has
 * no data). Carrier-aided first-order DLL on E1C (dot-product discriminator).
 * Side-peak monitor ("bump jumping"): over 0.5 s windows |VE|^2 and |VL|^2
 * against |P|^2 of the same blocks; on a side peak one of them exceeds the
 * prompt (the main peak), three windows in a row (E1C >= 26 dB-Hz) -> the
 * code moves 0.75 chip that way. C/N0 by the M2M4 estimator on each component (200 ms windows).
 * Lost -> the tracking image's aided searcher re-acquires around the
 * propagated code phase (gps_trk::GalService); not found for kGiveUpMs ->
 * released. Structure as in the GPS tracker (Kaplan; Borre et al., "A
 * Software-Defined GPS and Galileo Receiver"); written for this firmware. */

#ifndef __GPS_TRK_H__
#error "include gps_trk.hpp (it includes this file)"
#endif

#include "gal_acq.hpp"

namespace gal_trk {

using gal_acq::kHalf;
using gal_acq::kStreamWords;
constexpr int kEpochN = 8192;  /* samples per 4 ms code period */
constexpr int kQuarter = 2048; /* FLL partial sums */
constexpr int kMaxTrk = gal_acq::kMaxTrk;
/* CS25 secondary code (E1C), first chip first: chip k = bit 24 - k (1 = -1) */
constexpr uint32_t kCs25 = 0x007015B2u; /* 0011100000001010110110010 (ICD), 1 = -1 */
constexpr uint32_t kCsMask = (1u << 25) - 1u;

inline int popcount32(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    x = (x + (x >> 4)) & 0x0F0F0F0Fu;
    return static_cast<int>((x * 0x01010101u) >> 24);
}

/* the 25 chips starting at chip r, first chip in bit 24 */
inline uint32_t cs25_rot(int r) {
    return ((kCs25 << r) | (kCs25 >> (25 - r))) & kCsMask;
}

/* four-quadrant arctangent (rad) from the two-quadrant one */
inline float atan_4q(float y, float x) {
    const float a = gps_trk::atan_2q(y, x);
    if (x >= 0.0f) return a;
    return y >= 0.0f ? a + 3.14159265f : a - 3.14159265f;
}

struct Params {
    float pll_bn_wide{15.0f}; /* Hz, after the pull-in */
    float pll_bn{10.0f};      /* Hz, steady */
    float fll_assist{1.5f};   /* Hz, FLL assist while wide (epoch to epoch) */
    float dll_bn_wide{1.0f};
    float dll_bn{0.25f};
    float side_thr{1.0f}; /* |VE|^2 or |VL|^2 over |P|^2 above this kSideWins windows in a row: side peak */
};

class Channel {
   public:
    static constexpr int kPullinQ = 75;         /* 300 ms FLL on the half-period sums (+-250 Hz) */
    static constexpr int kPullinEpochs = 150;   /* then FLL epoch to epoch (+-62 Hz) up to 600 ms */
    static constexpr int kFllAccQ = 25;         /* half-period pull-in: products of 100 ms per frequency step */
    static constexpr int kFllAcc = 5;           /* epoch pull-in: 20 ms per step */
    static constexpr int kFqWin = 250;          /* false-lock watch: half-period products of 1 s */
    static constexpr float kFqThr = 70.0f;      /* Hz: a residual this large two windows in a row is a 125 Hz false lock */
    static constexpr int kWideEpochs = 250;     /* 1 s PLL wide, at least */
    static constexpr int kPldWin = 25;          /* lock indicator window, epochs (100 ms) */
    static constexpr int kCn0Win = 50;          /* M2M4 window, epochs (200 ms) */
    static constexpr int kSideWin = 125;        /* side-peak monitor window, epochs (0.5 s) */
    static constexpr int kSideWins = 3;         /* windows in a row before a jump */
    static constexpr float kSideMinCn0 = 26.0f; /* E1C dB-Hz: below, no jump */
    static constexpr float kLossCn0 = 23.0f;    /* E1C dB-Hz: three windows below -> lost */
    static constexpr uint32_t kNoLockMs = 6000; /* never phase-locked this long after a (re)start -> lost */
    /* the code discriminator's gain (chips per unit of Re{(Eh - Lh) P*} / |P|^2,
     * Eh - Lh from every other block): measured on the band-limited replica
     * (rf_test/gps gal_trk_test) */
    static constexpr float kDllGain = 0.65f;

    void stop() {
        state_ = gps_trk::TRK_IDLE;
        prn_ = 0;
    }
    gps_trk::ChState state() const { return static_cast<gps_trk::ChState>(state_); }
    uint8_t prn() const { return prn_; }
    bool on() const { return state_ == gps_trk::TRK_PULLIN || state_ == gps_trk::TRK_TRACK; }
    bool in_use() const { return state_ != gps_trk::TRK_IDLE && state_ != gps_trk::TRK_FAILED; }

    /* Start on PRN: half-chip streams (gal_acq stream format; comp 0 E1B, 1
     * E1C), code phase (chips) at the next sample processed, Doppler (Hz). */
    GAL_COLD void start(int prn, const uint32_t* hc_b, const uint32_t* hc_c, double code_phase_chips, float doppler_hz, const Params& p) {
        std::memcpy(hc_[0], hc_b, sizeof(hc_[0]));
        std::memcpy(hc_[1], hc_c, sizeof(hc_[1]));
        prn_ = static_cast<uint8_t>(prn);
        p_ = p;
        reacq_ = 0;
        side_jumps_ = 0;
        cs_syncs_ = 0;
        fl_fixes_ = 0;
        cs_errs_ = 0;
        ever_cs_ = false;
        released_ = rel_reported_ = false;
        release_ = gal_acq::TR_NONE;
        epochs_ = 0;
        set_code(code_phase_chips);
        phc_ = 0;
        restart(doppler_hz);
    }
    /* a re-acquisition found the satellite again: code phase at the next sample */
    GAL_COLD void reacquired(double code_phase_chips, float doppler_hz) {
        set_code(code_phase_chips);
        restart(doppler_hz);
        if (sym_epoch_) (*sym_epoch_)++; /* the symbol timing starts over */
    }
    /* where the E1-B soft symbols go (the tracker's TrkShared rings); a new
     * start counts as a new symbol timing */
    void attach_symbols(int8_t* ring, volatile uint32_t* write, uint16_t* epoch) {
        sym_ = ring;
        sym_write_ = write;
        sym_epoch_ = epoch;
        if (sym_epoch_) (*sym_epoch_)++;
    }
    /* free the channel (reported once by the tracker) */
    void release(uint8_t why) {
        state_ = gps_trk::TRK_FAILED;
        released_ = true;
        rel_reported_ = false;
        release_ = why;
    }
    bool take_release() {
        if (!released_ || rel_reported_) return false;
        rel_reported_ = true;
        return true;
    }
    uint8_t release_reason() const { return release_; }

    /* ---- hot path: one group of blocks (the GPS tracker's shared tables) */
    void run_group(const uint32_t* lut, const uint32_t* sblk, const uint32_t* cs);
    /* samples not correlated (missed DMA buffers, or waiting for a search):
     * propagate the replica and count epochs */
    void skip(uint64_t samples);

    /* lost and waiting for a search: the predicted code phase (chips) at the
     * next sample and the Doppler of the last good lock */
    bool wants_search() const { return state_ == gps_trk::TRK_SEARCH; }
    double code_phase_chips() const { return (m_ + f_ / 4294967296.0) * 0.5; }
    float search_doppler() const { return good_freq_; }
    uint32_t search_ms() const { return search_ms_; }
    const uint32_t* stream(int comp) const { return hc_[comp & 1]; }

    float cn0() const { return cn0c_; }
    float cn0_b() const { return cn0b_; }
    double doppler() const { return freq_int_; }
    uint32_t epochs() const { return epochs_; }
    bool cs_sync() const { return cs_sync_; }
    bool pll_lock() const { return pll_lock_; }
    uint16_t side_jumps() const { return side_jumps_; }
    int cs_phase() const { return cs_k_; }
    /* test access */
    double code_phase_halfchips() const { return m_ + f_ / 4294967296.0; }
    float side_e() const { return side_e_; }
    float side_l() const { return side_l_; }
    float code_err() const { return code_err_; }
    float pld() const { return pld_s_; }
    /* per-epoch callback for tests: E1C and E1B prompt after the secondary code
     * wipe-off (nullptr in the firmware) */
    void (*on_epoch)(void* ctx, int prn, float ic, float qc, float ib, float qb, int cs_sync) = nullptr;
    void* on_epoch_ctx = nullptr;

    GAL_COLD void info(gal_acq::TrkInfo& o) const {
        o.prn = prn_;
        o.state = state_;
        uint8_t f = 0;
        if (pll_lock_) f |= gal_acq::TF_PLL;
        if (cs_sync_) f |= gal_acq::TF_CS;
        if (ever_cs_) f |= gal_acq::TF_EVER;
        if (side_flag_) f |= gal_acq::TF_SIDE;
        if (released_) f |= gal_acq::TF_RELEASED;
        if (state_ == gps_trk::TRK_SEARCH) f |= gal_acq::TF_REACQ;
        o.flags = f;
        o.reacq = reacq_;
        o.cn0c_x10 = static_cast<uint16_t>(cn0c_ > 0 ? cn0c_ * 10.0f + 0.5f : 0);
        o.cn0b_x10 = static_cast<uint16_t>(cn0b_ > 0 ? cn0b_ * 10.0f + 0.5f : 0);
        o.doppler_hz = freq_int_;
        o.lock_ms = state_ == gps_trk::TRK_TRACK ? track_ms_ : 0;
        o.code_err = code_err_;
        o.code_x16 = static_cast<uint16_t>(code_phase_chips() * 16.0);
        const float pl = pld_s_ * 100.0f;
        o.pld_x100 = static_cast<int8_t>(pl > 100.0f ? 100 : (pl < -100.0f ? -100 : pl));
        o.mode = state_ == gps_trk::TRK_SEARCH ? 3 : (state_ == gps_trk::TRK_PULLIN ? 0 : (narrow_ ? 2 : 1));
        o.side_e_x100 = static_cast<uint16_t>(side_e_ * 100.0f > 65535.0f ? 65535.0f : side_e_ * 100.0f);
        o.side_l_x100 = static_cast<uint16_t>(side_l_ * 100.0f > 65535.0f ? 65535.0f : side_l_ * 100.0f);
        o.side_jumps = side_jumps_;
        o.cs_phase = static_cast<uint8_t>(cs_k_);
        o.release = release_;
        o.epochs = epochs_;
        o.cs_errs = cs_errs_;
        o.cs_syncs = cs_syncs_;
        o.fl_fixes = fl_fixes_;
    }

   private:
    uint32_t hc_[2][kStreamWords]{}; /* E1B, E1C */
    /* replica state at the next sample (as gps_trk::Channel, 8184 half chips) */
    uint32_t m_{0}, f_{0}, delta_{0};
    float delta_res_{0};
    uint32_t phc_{0}, wblk_{0};
    float wblk_res_{0};
    /* epoch in progress (Q negated: SMLSDX): 0-1 E1C prompt, 2-3 E1C Eh - Lh
     * (every other block), 4-5 E1B prompt, 6-7 VE, 8-9 VL, 10-11 E1C prompt of
     * the monitor blocks */
    int32_t acc_[12]{};
    int32_t qacc_[3][2]{}; /* E1C prompt sums at the ends of the first three quarters */
    uint8_t q_n_{0};
    uint32_t epoch_samples_{0};
    bool epoch_gap_{false};
    uint32_t epochs_{0};
    int8_t jump_{0}; /* side-peak jump to apply (half chips x2: +-3) */

    Params p_{};
    uint8_t prn_{0};
    uint8_t state_{gps_trk::TRK_IDLE};
    uint8_t reacq_{0};
    float freq_{0}, freq_int_{0}, code_corr_{0}, good_freq_{0};
    uint32_t mode_ms_{0}, track_ms_{0}, search_ms_{0}, nolock_ms_{0};
    float p2_avg_{0};
    float code_err_{0};
    float prev_i_{0}, prev_q_{0}; /* the last epoch's E1C prompt (FLL) */
    bool have_prev_{false};
    float fll_dot_{0}, fll_cross_{0}; /* pull-in sums */
    int fll_n_{0};
    /* lock */
    float pld_num_{0}, pld_den_{0};
    int pld_n_{0};
    float pld_s_{0};
    bool pll_lock_{false}, narrow_{false};
    uint32_t lock_good_{0}, lock_bad_{0};
    /* C/N0 */
    float m2c_{0}, m4c_{0}, m2b_{0}, m4b_{0};
    int cn0_n_{0}, cn0_wins_{0}, cn0_low_{0};
    float cn0c_{0}, cn0b_{0};
    /* secondary code */
    bool cs_sync_{false}, ever_cs_{false};
    int8_t cs_pol_{1};
    uint8_t cs_k_{0}; /* chip index of the epoch in progress */
    uint32_t cs_hist_{0};
    uint8_t cs_hist_n_{0};
    int8_t cs_cand_{-1}; /* rotation found once, waiting for the next 25 chips */
    int8_t cs_cand_pol_{1};
    uint8_t cs_chk_n_{0}, cs_chk_bad_{0};
    uint16_t cs_errs_{0};
    uint8_t cs_syncs_{0};
    /* 125 Hz false lock: 4 ms correlations leave the Costas PLL and the
     * epoch FLL blind to half a cycle per epoch; the quarter sums (1 ms apart)
     * are not, so their phase rotation is watched over 1 s windows */
    float fq_dot_{0}, fq_cross_{0};
    int fq_n_{0};
    uint8_t fq_hits_{0}, fl_fixes_{0};
    /* side-peak monitor */
    float sp_e_{0}, sp_l_{0}, sp_p_{0};
    int sp_n_{0};
    int8_t sp_prev_{0};
    uint8_t sp_run_{0};
    float side_e_{0}, side_l_{0};
    bool side_flag_{false};
    uint16_t side_jumps_{0};
    /* E1-B soft symbols out */
    int8_t* sym_{nullptr};
    volatile uint32_t* sym_write_{nullptr};
    uint16_t* sym_epoch_{nullptr};
    void put_symbol(int8_t v) {
        if (!sym_) return;
        const uint32_t w = *sym_write_;
        sym_[w % static_cast<uint32_t>(gal_acq::kSymRing)] = v;
        gps_trk::mem_barrier();
        *sym_write_ = w + 1;
    }
    /* release */
    bool released_{false}, rel_reported_{false};
    uint8_t release_{0};

    GAL_COLD void set_code(double chips) {
        double h = std::fmod(chips * 2.0, static_cast<double>(kHalf));
        if (h < 0) h += kHalf;
        m_ = static_cast<uint32_t>(h);
        if (m_ >= static_cast<uint32_t>(kHalf)) m_ = 0;
        f_ = static_cast<uint32_t>((h - std::floor(h)) * 4294967296.0);
    }
    GAL_COLD void restart(float doppler_hz) {
        freq_ = freq_int_ = good_freq_ = doppler_hz;
        code_corr_ = 0;
        delta_res_ = wblk_res_ = 0;
        set_nco();
        state_ = gps_trk::TRK_PULLIN;
        mode_ms_ = track_ms_ = search_ms_ = 0;
        p2_avg_ = 0;
        code_err_ = 0;
        have_prev_ = false;
        fll_dot_ = fll_cross_ = 0;
        fll_n_ = 0;
        fq_dot_ = fq_cross_ = 0;
        fq_n_ = 0;
        fq_hits_ = 0;
        pld_num_ = pld_den_ = pld_s_ = 0;
        pld_n_ = 0;
        pll_lock_ = narrow_ = false;
        lock_good_ = lock_bad_ = 0;
        nolock_ms_ = 0;
        m2c_ = m4c_ = m2b_ = m4b_ = 0;
        cn0_n_ = cn0_wins_ = cn0_low_ = 0;
        cn0c_ = cn0b_ = 0;
        cs_sync_ = false;
        cs_pol_ = 1;
        cs_hist_ = 0;
        cs_hist_n_ = 0;
        cs_cand_ = -1;
        cs_chk_n_ = cs_chk_bad_ = 0;
        sp_e_ = sp_l_ = sp_p_ = 0;
        sp_n_ = 0;
        sp_prev_ = 0;
        sp_run_ = 0;
        side_flag_ = false;
        jump_ = 0;
        for (auto& a : acc_) a = 0;
        q_n_ = 0;
        epoch_samples_ = 0;
        epoch_gap_ = true; /* the first epoch starts somewhere inside a code period */
    }
    void set_nco() { /* as gps_trk::Channel: the same chip rate and samples per half chip */
        const float w = freq_ * (32.0f * 4294967296.0f / gps_trk::kFs) + wblk_res_;
        const float wr = std::floor(w + 0.5f);
        wblk_res_ = w - wr;
        wblk_ = static_cast<uint32_t>(static_cast<int32_t>(wr));
        const float d = 4194304.0f - static_cast<float>(2046.0 / 2048.0 / 1575.42e6 * 4294967296.0) * freq_ -
                        static_cast<float>(2.0 / 2048000.0 * 4294967296.0) * code_corr_ + delta_res_;
        const float dr = std::floor(d + 0.5f);
        delta_res_ = d - dr;
        delta_ = static_cast<uint32_t>(static_cast<int32_t>(dr));
    }
    /* move the code by n half halfchips (+-3: the side-peak jump), inside an
     * epoch (never across its end: the epoch count stays exact) */
    GAL_COLD void move_code(int n2) {
        const int64_t span = static_cast<int64_t>(kHalf) << 32;
        int64_t ph = (static_cast<int64_t>(m_) << 32) + f_ + static_cast<int64_t>(n2) * (int64_t{1} << 31);
        if (ph < 0) ph += span;
        if (ph >= span) ph -= span;
        m_ = static_cast<uint32_t>(ph >> 32);
        f_ = static_cast<uint32_t>(ph & 0xFFFFFFFFu);
    }
    void quarter_mark() {
        while (q_n_ < 3 && epoch_samples_ >= static_cast<uint32_t>(kQuarter) * (q_n_ + 1u)) {
            qacc_[q_n_][0] = acc_[0];
            qacc_[q_n_][1] = acc_[1];
            q_n_++;
        }
    }
    void slow_block(const uint32_t* lut, uint32_t s, uint32_t c, int b);
    void add_part(uint32_t c, int b, uint32_t s0, uint32_t p0, uint32_t b0, uint32_t eh0, uint32_t lh0, uint32_t ve0, uint32_t vl0);
    void epoch_end();
    void track_epoch(const float* v, bool valid, const float* qi, const float* qq, bool quarters);
    void secondary(float ic, bool valid);
    void lost();
};

/* replica word of one half chip family from a 64-bit window: shift, then
 * the repeat at step j (bits after j take the previous half chip) */
inline uint32_t fam_word(uint64_t win, int sh, uint32_t lo) {
    const uint32_t w = static_cast<uint32_t>(win >> sh);
    return (w & lo) | ((w << 1) & ~lo);
}

inline uint64_t window64(const uint32_t* hc, uint32_t m) {
    const uint32_t* d = hc + (m >> 5);
    const uint32_t sh5 = m & 31u;
    return (((static_cast<uint64_t>(d[1]) << 32) | d[0]) >> sh5) | ((static_cast<uint64_t>(d[2]) << 32) << (32 - sh5));
}

/* Most blocks: no replica repeat (prompt family at fraction f, half family
 * at f + 1/2), no epoch end. Window bit k = half chip m - 2 + k, so the
 * prompt is win >> 2; early +1 half chip win >> 3, late win >> 1. */
inline void Channel::run_group(const uint32_t* lut, const uint32_t* sblk, const uint32_t* cs) {
    if (jump_ && m_ > 1024u && m_ < static_cast<uint32_t>(kHalf - 1024)) { /* the side-peak jump, mid-epoch */
        move_code(jump_);
        jump_ = 0;
        epoch_gap_ = true; /* this epoch has samples of both code phases */
    }
    uint32_t m = m_, f = f_, d32 = delta_ * 32u, phc = phc_, ns = 0;
    const uint32_t wblk = wblk_;
    int32_t ipc = acc_[0], qpc = acc_[1], idc = acc_[2], qdc = acc_[3], ipb = acc_[4], qpb = acc_[5];
    for (int b = 0; b < gps_trk::kGroupBlk; b++, lut += 128) {
        const uint32_t c = cs[(phc + (1u << 23)) >> 24];
        phc += wblk;
        if (f < d32 || (f - 0x80000000u) < d32 || m >= static_cast<uint32_t>(kHalf - 32)) {
            acc_[0] = ipc, acc_[1] = qpc, acc_[2] = idc, acc_[3] = qdc, acc_[4] = ipb, acc_[5] = qpb;
            m_ = m, f_ = f, phc_ = phc;
            epoch_samples_ += ns;
            ns = 0;
            slow_block(lut, sblk[b], c, b);
            m = m_, f = f_, d32 = delta_ * 32u, phc = phc_;
            ipc = acc_[0], qpc = acc_[1], idc = acc_[2], qdc = acc_[3], ipb = acc_[4], qpb = acc_[5];
            continue;
        }
        const uint64_t wc = window64(hc_[1], m);
        const uint32_t s = sblk[b];
        const uint32_t tp = gps_trk::lut_sum(lut, static_cast<uint32_t>(wc >> 2));
        const uint32_t cp = gps_trk::ssub16(gps_trk::ssub16(s, tp), tp);
        ipc = gps_trk::smlad(cp, c, ipc);
        qpc = gps_trk::smlsdx(cp, c, qpc);
        const bool upper = f >= 0x80000000u; /* early half = the word one half chip early, late half = the prompt */
        if (!(b & 1)) {
            /* (Eh - Lh) / 2 = lut(Lh) - lut(Eh) */
            const uint32_t cd = upper ? gps_trk::ssub16(tp, gps_trk::lut_sum(lut, static_cast<uint32_t>(wc >> 3)))
                                      : gps_trk::ssub16(gps_trk::lut_sum(lut, static_cast<uint32_t>(wc >> 1)), tp);
            idc = gps_trk::smlad(cd, c, idc);
            qdc = gps_trk::smlsdx(cd, c, qdc);
        }
        const uint32_t tb = gps_trk::lut_sum(lut, static_cast<uint32_t>(window64(hc_[0], m) >> 2));
        const uint32_t cb = gps_trk::ssub16(gps_trk::ssub16(s, tb), tb);
        ipb = gps_trk::smlad(cb, c, ipb);
        qpb = gps_trk::smlsdx(cb, c, qpb);
        if ((b & 3) == 3) { /* the side-peak monitor: VE +1.5, VL -1.5 half chips */
            const uint32_t te = gps_trk::lut_sum(lut, static_cast<uint32_t>(wc >> (upper ? 4 : 3)));
            const uint32_t tl = gps_trk::lut_sum(lut, static_cast<uint32_t>(wc >> (upper ? 1 : 0)));
            const uint32_t ce = gps_trk::ssub16(gps_trk::ssub16(s, te), te), cl = gps_trk::ssub16(gps_trk::ssub16(s, tl), tl);
            acc_[6] = gps_trk::smlad(ce, c, acc_[6]);
            acc_[7] = gps_trk::smlsdx(ce, c, acc_[7]);
            acc_[8] = gps_trk::smlad(cl, c, acc_[8]);
            acc_[9] = gps_trk::smlsdx(cl, c, acc_[9]);
            acc_[10] = gps_trk::smlad(cp, c, acc_[10]);
            acc_[11] = gps_trk::smlsdx(cp, c, acc_[11]);
        }
        f -= d32;
        m += 32;
        ns += 32;
    }
    acc_[0] = ipc, acc_[1] = qpc, acc_[2] = idc, acc_[3] = qdc, acc_[4] = ipb, acc_[5] = qpb;
    m_ = m, f_ = f, phc_ = phc;
    epoch_samples_ += ns;
    quarter_mark();
}

/* A block with a repeat of either half-chip family and / or the end of an
 * epoch: every replica word built exactly, the block split with bit masks. */
__attribute__((noinline)) inline void Channel::slow_block(const uint32_t* lut, uint32_t s, uint32_t c, int b) {
    uint32_t m = m_, f = f_;
    const uint32_t delta = delta_, d32 = delta * 32u;
    const uint64_t wc = window64(hc_[1], m), wb = window64(hc_[0], m);
    /* prompt family (P): repeat at step j when f < 32 delta */
    uint32_t lo_p = 0xFFFFFFFFu, adv = 32;
    uint32_t t = 32; /* samples of this block in the current epoch */
    if (f < d32) {
        const uint32_t j = f / delta;
        adv = 31;
        if (j <= 30) {
            lo_p = (2u << j) - 1u;
            if (m + 30 >= static_cast<uint32_t>(kHalf)) t = (kHalf - m <= j) ? kHalf - m : kHalf + 1 - m;
        } else if (m + 31 >= static_cast<uint32_t>(kHalf)) {
            t = kHalf - m;
        }
    } else if (m + 31 >= static_cast<uint32_t>(kHalf)) {
        t = kHalf - m;
    }
    /* half family (Eh, Lh, VE, VL) at fraction f + 1/2: carry, repeat */
    const uint32_t fh = f + 0x80000000u;
    const int carry = f >= 0x80000000u ? 1 : 0;
    uint32_t lo_h = 0xFFFFFFFFu;
    if (fh < d32) {
        const uint32_t j = fh / delta;
        if (j <= 30) lo_h = (2u << j) - 1u;
    }
    const uint32_t wp = fam_word(wc, 2, lo_p), wpb = fam_word(wb, 2, lo_p);
    const uint32_t weh = fam_word(wc, 2 + carry, lo_h), wlh = fam_word(wc, 1 + carry, lo_h);
    const bool mon = (b & 3) == 3;
    const uint32_t tp = gps_trk::lut_sum(lut, wp), tb = gps_trk::lut_sum(lut, wpb);
    const uint32_t teh = gps_trk::lut_sum(lut, weh), tlh = gps_trk::lut_sum(lut, wlh);
    uint32_t tve = 0, tvl = 0, wve = 0, wvl = 0;
    if (mon) {
        wve = fam_word(wc, 3 + carry, lo_h);
        wvl = fam_word(wc, carry, lo_h);
        tve = gps_trk::lut_sum(lut, wve);
        tvl = gps_trk::lut_sum(lut, wvl);
    }
    uint32_t slo = s, tplo = tp, tblo = tb, tehlo = teh, tlhlo = tlh, tvelo = tve, tvllo = tvl;
    if (t < 32) {
        const uint32_t mlo = (1u << t) - 1u;
        slo = gps_trk::lut_sum(lut, mlo);
        tplo = gps_trk::lut_sum(lut, wp & mlo);
        tblo = gps_trk::lut_sum(lut, wpb & mlo);
        tehlo = gps_trk::lut_sum(lut, weh & mlo);
        tlhlo = gps_trk::lut_sum(lut, wlh & mlo);
        if (mon) {
            tvelo = gps_trk::lut_sum(lut, wve & mlo);
            tvllo = gps_trk::lut_sum(lut, wvl & mlo);
        }
    }
    add_part(c, b, slo, tplo, tblo, tehlo, tlhlo, tvelo, tvllo);
    if (t >= 32) {
        epoch_samples_ += 32;
        f -= d32;
        m += adv;
        if (m >= static_cast<uint32_t>(kHalf)) { /* the epoch ended exactly at the block edge */
            m_ = m - kHalf, f_ = f;
            epoch_end();
            return;
        }
        m_ = m, f_ = f;
        return;
    }
    epoch_samples_ += t;
    m_ = m, f_ = f;
    epoch_end();
    /* the rest of the block starts the next epoch */
    f -= d32;
    m_ = m + adv - kHalf, f_ = f;
    add_part(c, b, gps_trk::ssub16(s, slo), gps_trk::ssub16(tp, tplo), gps_trk::ssub16(tb, tblo), gps_trk::ssub16(teh, tehlo), gps_trk::ssub16(tlh, tlhlo),
             gps_trk::ssub16(tve, tvelo), gps_trk::ssub16(tvl, tvllo));
    epoch_samples_ = 32 - t;
}

/* one part of a slow block (sum s0, look-ups of P, B, Eh, Lh, VE, VL) into the accumulators */
__attribute__((noinline)) inline void Channel::add_part(uint32_t c, int b, uint32_t s0, uint32_t p0, uint32_t b0, uint32_t eh0, uint32_t lh0, uint32_t ve0, uint32_t vl0) {
    const uint32_t cp = gps_trk::ssub16(gps_trk::ssub16(s0, p0), p0);
    const uint32_t cb = gps_trk::ssub16(gps_trk::ssub16(s0, b0), b0);
    acc_[0] = gps_trk::smlad(cp, c, acc_[0]);
    acc_[1] = gps_trk::smlsdx(cp, c, acc_[1]);
    if (!(b & 1)) {
        const uint32_t cd = gps_trk::ssub16(lh0, eh0);
        acc_[2] = gps_trk::smlad(cd, c, acc_[2]);
        acc_[3] = gps_trk::smlsdx(cd, c, acc_[3]);
    }
    acc_[4] = gps_trk::smlad(cb, c, acc_[4]);
    acc_[5] = gps_trk::smlsdx(cb, c, acc_[5]);
    if ((b & 3) == 3) {
        const uint32_t ce = gps_trk::ssub16(gps_trk::ssub16(s0, ve0), ve0), cl = gps_trk::ssub16(gps_trk::ssub16(s0, vl0), vl0);
        acc_[6] = gps_trk::smlad(ce, c, acc_[6]);
        acc_[7] = gps_trk::smlsdx(ce, c, acc_[7]);
        acc_[8] = gps_trk::smlad(cl, c, acc_[8]);
        acc_[9] = gps_trk::smlsdx(cl, c, acc_[9]);
        acc_[10] = gps_trk::smlad(cp, c, acc_[10]);
        acc_[11] = gps_trk::smlsdx(cp, c, acc_[11]);
    }
}

GAL_COLD inline void Channel::skip(uint64_t samples) {
    if (!in_use() || samples == 0) return;
    if (state_ == gps_trk::TRK_SEARCH) search_ms_ += static_cast<uint32_t>(samples / (gps_trk::kFs / 1000));
    phc_ += static_cast<uint32_t>(samples / gps_trk::kBlk) * wblk_;
    const uint64_t total = samples * static_cast<uint64_t>(delta_);
    uint64_t rep = 0;
    if (total > f_) rep = (total - f_ + 0xFFFFFFFFull) >> 32;
    f_ = static_cast<uint32_t>(static_cast<uint64_t>(f_) - total);
    const uint64_t mm = static_cast<uint64_t>(m_) + samples - rep;
    const uint64_t crossed = mm / kHalf;
    m_ = static_cast<uint32_t>(mm % kHalf);
    for (uint64_t k = 0; k < crossed; k++) { /* epochs passed: the secondary code keeps counting */
        epochs_++;
        put_symbol(0);
        cs_k_ = static_cast<uint8_t>(cs_k_ + 1 >= 25 ? 0 : cs_k_ + 1);
        if (state_ != gps_trk::TRK_SEARCH) {
            mode_ms_ += 4;
            if (state_ == gps_trk::TRK_TRACK) track_ms_ += 4;
        }
    }
    if (crossed) {
        for (auto& a : acc_) a = 0;
        q_n_ = 0;
        epoch_samples_ = 0;
        cs_hist_n_ = 0; /* the sign history has a hole */
        cs_cand_ = -1;
    }
    epoch_gap_ = true;
}

GAL_COLD inline void Channel::epoch_end() {
    const float k = 1.0f / 65536.0f;
    const bool valid = !epoch_gap_ && epoch_samples_ >= static_cast<uint32_t>(kHalf);
    const bool quarters = valid && q_n_ == 3;
    epoch_gap_ = false;
    epoch_samples_ = 0;
    float v[12];
    for (int i = 0; i < 12; i++) v[i] = acc_[i] * ((i & 1) ? -k : k);
    float qi[4], qq[4];
    if (quarters) {
        qi[0] = qacc_[0][0] * k, qq[0] = -qacc_[0][1] * k;
        for (int i = 1; i < 3; i++) qi[i] = (qacc_[i][0] - qacc_[i - 1][0]) * k, qq[i] = -(qacc_[i][1] - qacc_[i - 1][1]) * k;
        qi[3] = (acc_[0] - qacc_[2][0]) * k, qq[3] = -(acc_[1] - qacc_[2][1]) * k;
    }
    for (auto& a : acc_) a = 0;
    q_n_ = 0;
    epochs_++;
    track_epoch(v, valid, qi, qq, quarters);
    cs_k_ = static_cast<uint8_t>(cs_k_ + 1 >= 25 ? 0 : cs_k_ + 1);
}

/* the secondary code from the E1C prompt signs: found when a 25-chip window
 * matches one rotation (either polarity) and the next 25 chips confirm it */
GAL_COLD inline void Channel::secondary(float ic, bool valid) {
    const uint32_t bit = ic < 0 ? 1u : 0u;
    if (cs_sync_) {
        if (!valid) return;
        const uint32_t want = static_cast<uint32_t>((kCs25 >> (24 - cs_k_)) & 1u) ^ (cs_pol_ < 0 ? 1u : 0u);
        cs_chk_n_++;
        if (bit != want) {
            cs_chk_bad_++;
            if (cs_errs_ < 65535) cs_errs_++;
        }
        if (cs_chk_n_ >= 50) { /* more than 1 in 5 wrong over 200 ms: out of sync */
            if (cs_chk_bad_ > 10) cs_sync_ = false;
            cs_chk_n_ = cs_chk_bad_ = 0;
        }
        return;
    }
    if (!valid || !pll_lock_) {
        cs_hist_n_ = 0;
        cs_cand_ = -1;
        return;
    }
    cs_hist_ = ((cs_hist_ << 1) | bit) & kCsMask;
    if (cs_hist_n_ < 25) cs_hist_n_++;
    if (cs_hist_n_ < 25) return;
    /* the window's first chip is the epoch 24 back: rotation r means chip r there */
    int best = -1, pol = 1;
    for (int r = 0; r < 25; r++) {
        const int e = popcount32(cs_hist_ ^ cs25_rot(r));
        if (e <= 1) best = r, pol = 1;
        if (e >= 24) best = r, pol = -1;
    }
    if (best < 0) {
        cs_cand_ = -1;
        return;
    }
    /* the chip of the epoch just finished (the window's last) */
    const int cur = (best + 24) % 25;
    if (cs_cand_ >= 0 && cs_cand_ == cur && cs_cand_pol_ == pol) {
        cs_sync_ = ever_cs_ = true;
        cs_pol_ = static_cast<int8_t>(pol);
        cs_k_ = static_cast<uint8_t>(cur); /* advanced to the next chip after this epoch */
        cs_chk_n_ = cs_chk_bad_ = 0;
        if (cs_syncs_ < 255) cs_syncs_++;
        return;
    }
    /* one match: the same rotation must match again on the next 25 chips
     * (track_epoch keeps the candidate's chip in step with the epochs) */
    cs_cand_ = static_cast<int8_t>(cur);
    cs_cand_pol_ = static_cast<int8_t>(pol);
    cs_hist_n_ = 0;
}

GAL_COLD inline void Channel::track_epoch(const float* v, bool valid, const float* qi, const float* qq, bool quarters) {
    mode_ms_ += 4;
    if (state_ == gps_trk::TRK_TRACK) track_ms_ += 4;
    if (cs_cand_ >= 0 && !cs_sync_) cs_cand_ = static_cast<int8_t>((cs_cand_ + 1) % 25); /* the candidate's chip follows the epochs */
    float ic = v[0], qc = v[1], id = v[2], qd = v[3], ib = v[4], qb = v[5];
    if (!valid) {
        secondary(ic, false);
        have_prev_ = false;
        put_symbol(0);
        return;
    }
    /* the secondary code (and the polarity found with it) wiped off */
    if (cs_sync_) {
        const bool neg = (((kCs25 >> (24 - cs_k_)) & 1u) != 0) != (cs_pol_ < 0);
        if (neg) ic = -ic, qc = -qc, id = -id, qd = -qd, ib = -ib, qb = -qb;
    }
    if (on_epoch) on_epoch(on_epoch_ctx, prn_, ic, qc, ib, qb, cs_sync_);
    { /* the E1-B symbol: its prompt before the pilot's secondary code wipe (E1-B has
       * none), scaled by the pilot's amplitude; its sign is the data up to one
       * polarity for the whole lock (the decoder finds it on the page sync) */
        int8_t sv = 0;
        if (cs_sync_ && p2_avg_ > 0) {
            float x = v[4] * 40.0f / std::sqrt(p2_avg_);
            x = x > 127.0f ? 127.0f : (x < -127.0f ? -127.0f : x);
            sv = static_cast<int8_t>(x);
            if (!sv) sv = v[4] < 0 ? -1 : 1;
        }
        put_symbol(sv);
    }
    constexpr float T = 4e-3f;
    constexpr float kTwoPi = 6.2831853f;
    const float pll_err = (cs_sync_ ? atan_4q(qc, ic) : gps_trk::atan_2q(qc, ic)) / kTwoPi; /* cycles */
    /* FLL: in the pull-in first the two halves of this code period (2 ms
     * apart, +-250 Hz), then consecutive epochs (4 ms apart, +-62 Hz; each pair
     * sign-normalised on its dot product: the secondary code and data flip the
     * sign between epochs). The pull-in sums the products over kFllAccQ /
     * kFllAcc epochs and steps the frequency by part of the estimate (a weak
     * signal's single products are mostly noise); the FLL assist of the wide
     * PLL uses each epoch pair. */
    float fll_err = 0;
    const bool pullin = state_ == gps_trk::TRK_PULLIN && mode_ms_ <= static_cast<uint32_t>(kPullinEpochs) * 4u;
    const bool coarse = pullin && mode_ms_ <= static_cast<uint32_t>(kPullinQ) * 4u;
    float dot = 0, cross = 0;
    bool have_fll = false;
    /* the two halves of this code period (2 ms apart, no sign flip between them:
     * four-quadrant, +-250 Hz) */
    float hdot = 0, hcross = 0;
    if (quarters) {
        const float i1 = qi[0] + qi[1], q1 = qq[0] + qq[1], i2 = qi[2] + qi[3], q2 = qq[2] + qq[3];
        hdot = i1 * i2 + q1 * q2;
        hcross = i1 * q2 - q1 * i2;
    }
    if (coarse && quarters) {
        dot = hdot;
        cross = hcross;
        have_fll = true;
    } else if (!coarse && have_prev_) {
        dot = prev_i_ * v[0] + prev_q_ * v[1];
        cross = prev_i_ * v[1] - prev_q_ * v[0];
        if (dot < 0) dot = -dot, cross = -cross;
        fll_err = gps_trk::atan_2q(cross, dot) / (kTwoPi * T);
        have_fll = true;
    }
    /* code: Re{(Eh - Lh) P*} / |P|^2, Eh - Lh from half the blocks */
    const float p2 = ic * ic + qc * qc;
    p2_avg_ = p2_avg_ > 0 ? p2_avg_ + 0.05f * (p2 - p2_avg_) : p2;
    float ce = p2_avg_ > 0 ? kDllGain * (id * ic + qd * qc) / p2_avg_ : 0.0f;
    if (ce > 0.25f) ce = 0.25f;
    if (ce < -0.25f) ce = -0.25f;
    code_err_ = ce;
    /* carrier */
    prev_i_ = v[0], prev_q_ = v[1];
    have_prev_ = true;
    if (pullin) {
        if (have_fll) {
            fll_dot_ += dot;
            fll_cross_ += cross;
            if (++fll_n_ >= (coarse ? kFllAccQ : kFllAcc)) {
                if (coarse)
                    freq_int_ += 0.6f * atan_4q(fll_cross_, fll_dot_) / (kTwoPi * (2 * kQuarter / static_cast<float>(gps_trk::kFs)));
                else
                    freq_int_ += 0.5f * gps_trk::atan_2q(fll_cross_, fll_dot_) / (kTwoPi * T);
                fll_dot_ = fll_cross_ = 0;
                fll_n_ = 0;
            }
        }
        if (coarse && mode_ms_ + 4u > static_cast<uint32_t>(kPullinQ) * 4u) fll_dot_ = fll_cross_ = 0, fll_n_ = 0; /* the stages' sums apart */
        freq_ = freq_int_;
    } else {
        if (state_ == gps_trk::TRK_PULLIN) {
            state_ = gps_trk::TRK_TRACK;
            mode_ms_ = track_ms_ = 0;
        }
        const float bn = narrow_ ? p_.pll_bn : p_.pll_bn_wide;
        const float wn = bn / 0.53f;
        const float wf = narrow_ ? 0.0f : p_.fll_assist / 0.25f;
        freq_int_ += T * (wn * wn * pll_err + wf * fll_err);
        freq_ = freq_int_ + 1.414f * wn * pll_err;
        /* the 125 Hz false-lock watch: the quarter sums' rotation over 1 s (in
         * true lock ~0; at k x 125 Hz off it reads ~k x 125 Hz) */
        if (quarters) {
            fq_dot_ += hdot;
            fq_cross_ += hcross;
            if (++fq_n_ >= kFqWin) {
                const float fq = atan_4q(fq_cross_, fq_dot_) / (kTwoPi * (2 * kQuarter / static_cast<float>(gps_trk::kFs)));
                fq_dot_ = fq_cross_ = 0;
                fq_n_ = 0;
                if (fq > kFqThr || fq < -kFqThr) {
                    if (++fq_hits_ >= 2) {
                        freq_int_ += 125.0f * std::floor(fq / 125.0f + 0.5f);
                        freq_ = freq_int_;
                        fq_hits_ = 0;
                        if (fl_fixes_ < 255) fl_fixes_++;
                        narrow_ = pll_lock_ = cs_sync_ = false;
                        mode_ms_ = 0;
                        lock_good_ = lock_bad_ = 0;
                        pld_s_ = pld_num_ = pld_den_ = 0;
                        pld_n_ = 0;
                        have_prev_ = false;
                        cs_hist_n_ = 0;
                        cs_cand_ = -1;
                    }
                } else {
                    fq_hits_ = 0;
                }
            }
        }
    }
    code_corr_ = 4.0f * (narrow_ ? p_.dll_bn : p_.dll_bn_wide) * ce;
    set_nco();
    /* lock indicator (sign-free per epoch: before the secondary code too) */
    pld_num_ += ic * ic - qc * qc;
    pld_den_ += p2;
    if (++pld_n_ >= kPldWin) {
        const float pld = pld_den_ > 0 ? pld_num_ / pld_den_ : 0;
        pld_s_ += 0.3f * (pld - pld_s_);
        pld_num_ = pld_den_ = 0;
        pld_n_ = 0;
        const bool good = pld_s_ > 0.6f && state_ == gps_trk::TRK_TRACK;
        if (good) {
            lock_good_++;
            lock_bad_ = 0;
        } else {
            lock_bad_++;
            lock_good_ = 0;
        }
        pll_lock_ = pld_s_ > 0.5f && state_ == gps_trk::TRK_TRACK;
        if (!narrow_ && state_ == gps_trk::TRK_TRACK && mode_ms_ >= static_cast<uint32_t>(kWideEpochs) * 4u && lock_good_ >= 5)
            narrow_ = true;
        else if (narrow_ && lock_bad_ >= 8) { /* ~0.8 s of poor phase lock: widen again */
            narrow_ = false;
            mode_ms_ = 0;
        }
    }
    secondary(v[0], true);
    /* side-peak monitor */
    sp_e_ += v[6] * v[6] + v[7] * v[7];
    sp_l_ += v[8] * v[8] + v[9] * v[9];
    sp_p_ += v[10] * v[10] + v[11] * v[11];
    if (++sp_n_ >= kSideWin) {
        side_e_ = sp_p_ > 0 ? sp_e_ / sp_p_ : 0;
        side_l_ = sp_p_ > 0 ? sp_l_ / sp_p_ : 0;
        int8_t dir = 0;
        if (side_e_ > p_.side_thr && side_e_ > side_l_) dir = 1; /* the main peak is earlier: move the replica early */
        if (side_l_ > p_.side_thr && side_l_ > side_e_) dir = -1;
        side_flag_ = dir != 0;
        /* kSideWins windows in a row on the same side, with a signal strong enough
         * for the ratios to mean something (near the loss threshold noise pushes
         * the main peak's ~0.4-0.8 over 1 now and then) */
        if (!dir || cn0c_ < kSideMinCn0)
            sp_run_ = 0;
        else if (dir == sp_prev_)
            sp_run_++;
        else
            sp_run_ = 1;
        if (sp_run_ >= kSideWins && state_ == gps_trk::TRK_TRACK) {
            jump_ = static_cast<int8_t>(3 * dir);
            if (side_jumps_ < 65535) side_jumps_++;
            p2_avg_ = 0;
            dir = 0;
            sp_run_ = 0;
        }
        sp_prev_ = dir;
        sp_e_ = sp_l_ = sp_p_ = 0;
        sp_n_ = 0;
    }
    /* C/N0: M2M4 per component */
    m2c_ += p2;
    m4c_ += p2 * p2;
    const float pb = ib * ib + qb * qb;
    m2b_ += pb;
    m4b_ += pb * pb;
    if (++cn0_n_ >= kCn0Win) {
        auto est = [](float m2s, float m4s) {
            const float M2 = m2s / kCn0Win, M4 = m4s / kCn0Win;
            const float pd2 = 2.0f * M2 * M2 - M4;
            float e = 0;
            if (pd2 > 0) {
                const float pd = std::sqrt(pd2), pn = M2 - pd;
                if (pn > 0) e = 10.0f * std::log10(pd / pn / T);
            }
            return e < 0 ? 0.0f : e;
        };
        const float ec = est(m2c_, m4c_), eb = est(m2b_, m4b_);
        cn0c_ = cn0_wins_ == 0 ? ec : cn0c_ + 0.3f * (ec - cn0c_);
        cn0b_ = cn0_wins_ == 0 ? eb : cn0b_ + 0.3f * (eb - cn0b_);
        cn0_wins_++;
        m2c_ = m4c_ = m2b_ = m4b_ = 0;
        cn0_n_ = 0;
        if (ec < kLossCn0) {
            cn0_low_++;
        } else {
            cn0_low_ = 0;
            if (pll_lock_) good_freq_ = freq_int_;
        }
        if ((cn0_low_ >= 3 && cn0_wins_ >= 3) || (cn0_wins_ >= 5 && cn0c_ < kLossCn0 - 2.0f)) {
            lost();
            return;
        }
    }
    nolock_ms_ = pll_lock_ ? 0 : nolock_ms_ + 4;
    if (nolock_ms_ > kNoLockMs) { /* no phase lock for a long while */
        lost();
        return;
    }
}

GAL_COLD inline void Channel::lost() {
    if (++reacq_ == 0) reacq_ = 255;
    state_ = gps_trk::TRK_SEARCH;
    search_ms_ = 0;
    pll_lock_ = narrow_ = cs_sync_ = false;
    cn0c_ = cn0b_ = 0;
    /* the code keeps running at the last good Doppler for the search */
    freq_ = freq_int_ = good_freq_;
    code_corr_ = 0;
    set_nco();
}

}  // namespace gal_trk

#endif /*__GAL_TRK_H__*/
