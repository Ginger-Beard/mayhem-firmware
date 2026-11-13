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

/* Host tests of the FSK frame rules (common/sigdet_fsk.hpp) and receiver
 * (baseband/sigdet_fsk_rx.hpp) on synthetic frames: every line coding, check and
 * length rule, noise sweeps, two rules in one dwell, 100 kb/s GFSK, and the
 * negatives (wrong sync, wrong check, noise). All frame contents are made up. */

#include "doctest.h"
#include "sigdet_fsk_rx.hpp"

#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace {

using namespace sigdet;

/* A rule the way the app's parser builds it (sigdet_sig.cpp parse_fsk). */
struct RuleSpec {
    uint32_t rate = 9600;
    uint16_t bw_khz = 200;
    uint8_t coding = FSK_NRZ;
    bool lsb = false;
    uint8_t pre = 0x55;
    uint8_t pre_min = 2;
    uint32_t sync = 0x2DD4;
    uint8_t sync_bits = 16;
    uint8_t len_mode = FSK_LEN_FIELD;
    uint8_t len_off = 0, len_size = 1, len_counts = FSK_CNT_AFTER;
    int8_t len_adj = 0;
    uint8_t len_min = 1, len_max = 64;
    uint8_t check = FSK_CHK_CRC16;
    uint16_t seed = 0xFFFF, poly = 0x1021, xorout = 0;
    bool refin = false, refout = false, chk_le = false;
};

FskRule make_rule(const RuleSpec& s, uint8_t tag) {
    FskRule r{};
    uint8_t pre[1] = {s.pre};
    uint8_t sync[4];
    for (int i = 0; i < 4; i++) sync[i] = static_cast<uint8_t>((s.sync << (32 - s.sync_bits)) >> (24 - 8 * i));
    REQUIRE(fsk_pattern(pre, 1, s.pre_min, sync, s.sync_bits, s.coding, r.pattern, r.chips));
    r.max_err = static_cast<uint8_t>(r.chips / 12);
    r.coding = s.coding;
    r.flags = static_cast<uint8_t>((s.lsb ? FSK_F_LSB : 0) | (s.refin ? FSK_F_REFIN : 0) | (s.refout ? FSK_F_REFOUT : 0) | (s.chk_le ? FSK_F_CHK_LE : 0));
    r.len_mode = s.len_mode;
    r.len_off = s.len_off;
    r.len_size = s.len_size;
    r.len_counts = s.len_counts;
    r.len_adj = s.len_adj;
    r.len_min = s.len_min;
    r.len_max = s.len_max;
    r.check = s.check;
    r.cov_from = 0;
    r.cov_to = kFskBeforeCheck;
    r.chk_pos = kFskLast;
    r.seed = s.seed;
    r.poly = s.poly;
    r.xorout = s.xorout;
    r.tag = tag;
    return r;
}

FskDwell make_dwell(const std::vector<RuleSpec>& specs) {
    FskDwell d{};
    const uint32_t k = (1000u + specs[0].bw_khz / 2) / specs[0].bw_khz;
    d.decim = static_cast<uint8_t>(4 * (k < 1 ? 1 : (k > 62 ? 62 : k)));
    d.chip_hz = specs[0].rate * fsk_chips_per_bit(specs[0].coding);
    const uint32_t spc10 = kFskInputRate / d.decim * 10 / d.chip_hz;
    const uint32_t ma = (spc10 + 13) / 26;
    d.ma = static_cast<uint8_t>(ma < 1 ? 1 : (ma > 8 ? 8 : ma));
    for (size_t i = 0; i < specs.size(); i++) d.rules[d.nrules++] = make_rule(specs[i], static_cast<uint8_t>(i));
    return d;
}

/* A frame for the rule: length field (if any), body, check. */
std::vector<uint8_t> make_frame(const RuleSpec& s, std::vector<uint8_t> body) {
    const uint8_t cs = fsk_check_size(s.check);
    std::vector<uint8_t> f;
    if (s.len_mode == FSK_LEN_FIELD) {
        const size_t total = 1 + body.size() + cs;
        uint32_t L = s.len_counts == FSK_CNT_WHOLE ? total : (s.len_counts == FSK_CNT_AFTER ? total - 1 : total - 1 - cs);
        f.push_back(static_cast<uint8_t>(L - s.len_adj));
    }
    f.insert(f.end(), body.begin(), body.end());
    if (s.check == FSK_CHK_XOR8 || s.check == FSK_CHK_SUM8) {
        uint8_t x = static_cast<uint8_t>(s.seed);
        for (auto b : f) x = s.check == FSK_CHK_XOR8 ? static_cast<uint8_t>(x ^ b) : static_cast<uint8_t>(x + b);
        f.push_back(static_cast<uint8_t>(x ^ s.xorout));
    } else if (s.check == FSK_CHK_CRC8 || s.check == FSK_CHK_CRC16) {
        const uint16_t c = fsk_crc(f.data(), f.size(), s.check == FSK_CHK_CRC16 ? 16 : 8, s.poly, s.seed, s.refin, s.refout, s.xorout);
        if (s.check == FSK_CHK_CRC16) {
            f.push_back(static_cast<uint8_t>(s.chk_le ? c : c >> 8));
            f.push_back(static_cast<uint8_t>(s.chk_le ? c >> 8 : c));
        } else {
            f.push_back(static_cast<uint8_t>(c));
        }
    }
    return f;
}

struct Air {
    double dev = 20000, off = 0, amp = 40, sigma = 1.0;
    uint32_t seed = 1;
    bool gauss = false; /* GFSK-like: frequency smoothed (moving average) over half a bit */
    bool corrupt_sync = false;
    bool corrupt_check = false;
    double tail_ms = 0; /* carrier on after the frame (len=carrier frames end at the drop) */
};

/* 4 Msps int8 I/Q, the channel at +1 MHz: per frame silence, 8 preamble
 * bytes, the sync, the frame bits; silence at the end. */
std::vector<int8_t> modulate(const RuleSpec& s, const std::vector<std::vector<uint8_t>>& frames, const Air& a) {
    std::vector<int> chips; /* -1 silence, -2 carrier without data (a tone at +dev) */
    const uint8_t cpb = fsk_chips_per_bit(s.coding);
    const double chip_hz = s.rate * cpb;
    auto silence = [&](double ms) { for (int k = 0; k < static_cast<int>(ms * 1e-3 * chip_hz); k++) chips.push_back(-1); };
    for (const auto& fr : frames) {
        silence(20);
        std::vector<int> bits;
        for (int k = 0; k < 8; k++)
            for (int b = 7; b >= 0; b--) bits.push_back((s.pre >> b) & 1);
        uint32_t sync = s.sync ^ (a.corrupt_sync ? 0x0F0Fu : 0);
        for (int b = s.sync_bits - 1; b >= 0; b--) bits.push_back((sync >> b) & 1);
        std::vector<uint8_t> f = fr;
        if (a.corrupt_check) f.back() ^= 0x01;
        for (auto v : f)
            for (int i = 0; i < 8; i++) bits.push_back(s.lsb ? (v >> i) & 1 : (v >> (7 - i)) & 1);
        int level = 0;
        for (int b : bits) {
            if (s.coding == FSK_NRZ) {
                chips.push_back(b);
            } else if (s.coding == FSK_MAN) {
                chips.push_back(b);
                chips.push_back(!b);
            } else if (s.coding == FSK_MAN_INV) {
                chips.push_back(!b);
                chips.push_back(b);
            } else {
                const int c1 = b ? level : !level;
                chips.push_back(c1);
                chips.push_back(!c1);
                level = !c1;
            }
        }
        for (int k = 0; k < static_cast<int>(a.tail_ms * 1e-3 * chip_hz); k++) chips.push_back(-2);
    }
    silence(20);
    const double fs = 4e6;
    const size_t n = static_cast<size_t>(chips.size() / chip_hz * fs) / 2048 * 2048 + 2048;
    std::vector<int8_t> v(2 * n);
    std::mt19937 rng(a.seed);
    std::normal_distribution<double> g(0.0, a.sigma);
    const int win = a.gauss ? static_cast<int>(fs / s.rate / 2) : 1;
    std::vector<double> fq(n, 0.0);
    for (size_t k = 0; k < n; k++) {
        const size_t ci = static_cast<size_t>(k / fs * chip_hz);
        const int c = ci < chips.size() ? chips[ci] : -1;
        fq[k] = c == -1 ? 0.0 : (c == -2 ? a.dev : (c ? a.dev : -a.dev));
    }
    double acc = 0, ph = 0;
    for (size_t k = 0; k < n; k++) {
        acc += fq[k] - (k >= static_cast<size_t>(win) ? fq[k - win] : 0.0);
        const size_t ci = static_cast<size_t>(k / fs * chip_hz);
        const bool on = ci < chips.size() && chips[ci] != -1;
        ph += 2 * M_PI * (1e6 + a.off + acc / win) / fs;
        const double amp = on ? a.amp : 0;
        auto sat = [](double x) { return static_cast<int8_t>(x > 127 ? 127 : (x < -128 ? -128 : std::lround(x))); };
        v[2 * k] = sat(amp * std::cos(ph) + g(rng));
        v[2 * k + 1] = sat(amp * std::sin(ph) + g(rng));
    }
    return v;
}

struct Got {
    std::vector<FskFrame> f;
};
void collect(void* c, const FskFrame& f) {
    static_cast<Got*>(c)->f.push_back(f);
}

Got run(const std::vector<int8_t>& v, FskRx& rx) {
    Got g;
    for (size_t k = 0; k + 2048 <= v.size() / 2; k += 2048) rx.feed(v.data() + 2 * k, 2048, collect, &g);
    return g;
}

std::vector<std::vector<uint8_t>> some_frames(const RuleSpec& s, int n, size_t body = 10) {
    std::vector<std::vector<uint8_t>> out;
    for (int i = 0; i < n; i++) {
        std::vector<uint8_t> b;
        for (size_t k = 0; k < body; k++) b.push_back(static_cast<uint8_t>(0xA5 ^ (k * 37) ^ (i * 11)));
        out.push_back(make_frame(s, b));
    }
    return out;
}

}  // namespace

TEST_SUITE("sigdet_fsk") {
    TEST_CASE("CRC catalogue values (check string 123456789)") {
        const uint8_t m[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        CHECK(fsk_crc(m, 9, 16, 0x1021, 0xFFFF, false, false, 0) == 0x29B1);      /* CRC-16/CCITT-FALSE */
        CHECK(fsk_crc(m, 9, 16, 0x1021, 0x0000, true, true, 0) == 0x2189);        /* CRC-16/KERMIT */
        CHECK(fsk_crc(m, 9, 16, 0x8005, 0x0000, true, true, 0) == 0xBB3D);        /* CRC-16/ARC */
        CHECK(fsk_crc(m, 9, 16, 0x3D65, 0x0000, false, false, 0xFFFF) == 0xC2B7); /* CRC-16/EN-13757 (wM-Bus) */
        CHECK(fsk_crc(m, 9, 8, 0x07, 0x00, false, false, 0) == 0xF4);             /* CRC-8 */
        CHECK(fsk_crc(m, 9, 8, 0x31, 0x00, true, true, 0) == 0xA1);               /* CRC-8/MAXIM */
    }

    TEST_CASE("pattern: Z-Wave R1 preamble tail + SOF in Manchester is 48 chips") {
        const uint8_t pre[1] = {0x55}, sof[1] = {0xF0};
        uint64_t p;
        uint8_t n;
        REQUIRE(fsk_pattern(pre, 1, 10, sof, 8, FSK_MAN, p, n));
        CHECK(n == 48);
        CHECK((p & 0xFFFF) == 0xAA55); /* SOF F0: 1111 0000 -> 10101010 01010101 */
        /* a 32 bit sync in Manchester fills 64 chips: no preamble part */
        const uint8_t s4[4] = {0x12, 0x34, 0x56, 0x78};
        REQUIRE(fsk_pattern(pre, 1, 4, s4, 32, FSK_MAN, p, n));
        CHECK(n == 64);
        CHECK_FALSE(fsk_pattern(pre, 1, 4, s4, 0, FSK_MAN, p, n));
    }

    TEST_CASE("every coding x check x length rule: clean frames decoded as sent") {
        const uint8_t codings[] = {FSK_NRZ, FSK_MAN, FSK_MAN_INV, FSK_DMAN};
        const uint8_t checks[] = {FSK_CHK_XOR8, FSK_CHK_SUM8, FSK_CHK_CRC8, FSK_CHK_CRC16, FSK_CHK_NONE};
        const uint8_t lens[] = {FSK_LEN_FIXED, FSK_LEN_FIELD, FSK_LEN_CARRIER};
        int combos = 0;
        for (uint8_t code : codings)
            for (uint8_t chk : checks)
                for (uint8_t lm : lens) {
                    RuleSpec s;
                    s.coding = code;
                    s.check = chk;
                    s.len_mode = lm;
                    s.lsb = (combos % 3) == 1;
                    s.rate = code == FSK_NRZ ? 19200 : 9600;
                    if (chk == FSK_CHK_CRC8) s.poly = 0x07, s.seed = 0;
                    if (chk <= FSK_CHK_SUM8) s.seed = 0x5A;
                    const size_t body = 9;
                    if (lm == FSK_LEN_FIXED) s.len_min = s.len_max = static_cast<uint8_t>(body + fsk_check_size(chk));
                    if (lm == FSK_LEN_CARRIER) s.len_min = 2;
                    auto frames = some_frames(s, 3, body);
                    FskRx rx;
                    rx.configure(make_dwell({s}));
                    const Got g = run(modulate(s, frames, Air{}), rx);
                    INFO("coding " << int(code) << " check " << int(chk) << " len " << int(lm));
                    REQUIRE(g.f.size() == 3);
                    for (size_t i = 0; i < 3; i++) {
                        CHECK(g.f[i].len == frames[i].size());
                        CHECK(g.f[i].checked == (chk != FSK_CHK_NONE));
                        const size_t nb = std::min<size_t>(frames[i].size() - fsk_check_size(chk), kFskHdr);
                        CHECK(std::vector<uint8_t>(g.f[i].hdr, g.f[i].hdr + g.f[i].nhdr) == std::vector<uint8_t>(frames[i].begin(), frames[i].begin() + nb));
                        if (code != FSK_NRZ) CHECK(g.f[i].violations == 0);
                    }
                    combos++;
                }
        CHECK(combos == 60);
    }

    TEST_CASE("length field: counts, size 2 little endian, adjust") {
        RuleSpec s;
        s.len_counts = FSK_CNT_PAYLOAD;
        s.len_adj = -1;
        auto frames = some_frames(s, 2, 12);
        FskRx rx;
        rx.configure(make_dwell({s}));
        Got g = run(modulate(s, frames, Air{}), rx);
        REQUIRE(g.f.size() == 2);
        CHECK(g.f[0].len == frames[0].size());
        /* a CRC-16 sent little endian */
        s = RuleSpec{};
        s.chk_le = true;
        s.refin = s.refout = true;
        s.seed = 0;
        frames = some_frames(s, 2, 6);
        rx.configure(make_dwell({s}));
        g = run(modulate(s, frames, Air{}), rx);
        CHECK(g.f.size() == 2);
    }

    TEST_CASE("measurements: deviation, carrier offset and bit rate roughly as sent") {
        RuleSpec s;
        s.coding = FSK_MAN;
        s.rate = 9600;
        Air a;
        a.dev = 25000;
        a.off = -6000;
        auto frames = some_frames(s, 2, 30);
        FskRx rx;
        rx.configure(make_dwell({s}));
        const Got g = run(modulate(s, frames, a), rx);
        REQUIRE(g.f.size() == 2);
        MESSAGE("dev " << g.f[0].dev_hz << " Hz, off " << g.f[0].off_hz << " Hz, rate " << g.f[0].rate_bps << " b/s");
        CHECK(g.f[0].dev_hz == doctest::Approx(25000).epsilon(0.3));
        CHECK(g.f[0].off_hz == doctest::Approx(-6000).epsilon(0.3));
        CHECK(g.f[0].rate_bps == doctest::Approx(9600).epsilon(0.03));
    }

    TEST_CASE("two rules in one dwell: each frame goes to its own rule") {
        RuleSpec a, b;
        b.sync = 0xD391;
        b.check = FSK_CHK_XOR8;
        b.seed = 0;
        auto fa = some_frames(a, 2, 8);
        auto fb = some_frames(b, 2, 5);
        FskRx rx;
        rx.configure(make_dwell({a, b}));
        Got ga = run(modulate(a, fa, Air{}), rx);
        Got gb = run(modulate(b, fb, Air{}), rx);
        REQUIRE(ga.f.size() == 2);
        REQUIRE(gb.f.size() == 2);
        CHECK(ga.f[0].tag == 0);
        CHECK(gb.f[1].tag == 1);
    }

    TEST_CASE("100 kb/s GFSK NRZ (Z-Wave R3-like timing, made-up frame) in a 500 kHz dwell") {
        RuleSpec s;
        s.rate = 100000;
        s.bw_khz = 500;
        s.sync = 0xF0;
        s.sync_bits = 8;
        s.len_off = 7;
        s.len_min = 10;
        s.len_counts = FSK_CNT_WHOLE;
        s.seed = 0x1D0F;
        std::vector<std::vector<uint8_t>> frames;
        for (int i = 0; i < 4; i++) {
            std::vector<uint8_t> f = {0xC0, 0xFF, 0xEE, 0x42, 1, 0x41, static_cast<uint8_t>(i), 0, 5, 0x20, 0x01};
            f[7] = static_cast<uint8_t>(f.size() + 2);
            const uint16_t c = fsk_crc(f.data(), f.size(), 16, 0x1021, 0x1D0F, false, false, 0);
            f.push_back(static_cast<uint8_t>(c >> 8));
            f.push_back(static_cast<uint8_t>(c));
            frames.push_back(f);
        }
        Air a;
        a.dev = 29000;
        a.gauss = true;
        FskRx rx;
        const FskDwell d = make_dwell({s});
        CHECK(d.decim == 8);
        rx.configure(d);
        const Got g = run(modulate(s, frames, a), rx);
        MESSAGE("R3-like: " << g.f.size() << " / 4, syncs " << rx.stats().syncs);
        CHECK(g.f.size() == 4);
    }

    TEST_CASE("sensitivity: frames decoded per SNR (in the dwell's channel), documented in the note") {
        struct Case {
            const char* name;
            RuleSpec s;
        };
        std::vector<Case> cases(3);
        cases[0].name = "9.6k Manchester xor8";
        cases[0].s.coding = FSK_MAN;
        cases[0].s.check = FSK_CHK_XOR8;
        cases[0].s.seed = 0xFF;
        cases[1].name = "38.4k NRZ crc16";
        cases[1].s.rate = 38400;
        cases[2].name = "100k NRZ crc16 (500 kHz)";
        cases[2].s.rate = 100000;
        cases[2].s.bw_khz = 500;
        const double sigma = 6.0;
        for (auto& c : cases) {
            const FskDwell d = make_dwell({c.s});
            const double share = (4e6 / d.decim) / 4e6;
            auto frames = some_frames(c.s, 20, 12);
            std::string line;
            for (double snr : {2.0, 4.0, 6.0, 8.0, 12.0, 20.0}) {
                Air a;
                a.sigma = sigma;
                a.amp = std::sqrt(std::pow(10.0, snr / 10) * 2 * sigma * sigma * share);
                a.seed = static_cast<uint32_t>(snr * 10);
                a.off = 3000;
                a.gauss = c.s.rate >= 100000;
                a.dev = c.s.rate >= 100000 ? 29000 : 20000;
                FskRx rx;
                rx.configure(d);
                const size_t ok = run(modulate(c.s, frames, a), rx).f.size();
                line += " " + std::to_string(static_cast<int>(snr)) + "dB:" + std::to_string(ok) + "/20";
                if (snr >= 12.0) CHECK(ok >= (c.s.rate >= 100000 ? 18u : 19u));
                if (snr >= 20.0) CHECK(ok == 20);
            }
            const std::string msg = std::string(c.name) + ":" + line;
            MESSAGE(msg);
        }
    }

    TEST_CASE("negatives: wrong sync, wrong check, noise") {
        RuleSpec s;
        auto frames = some_frames(s, 5, 10);
        FskRx rx;
        rx.configure(make_dwell({s}));
        Air a;
        a.corrupt_sync = true;
        CHECK(run(modulate(s, frames, a), rx).f.empty());
        a = Air{};
        a.corrupt_check = true;
        rx.clear_stats();
        CHECK(run(modulate(s, frames, a), rx).f.empty());
        CHECK(rx.stats().bad_sum == 5);
        /* 10 s of noise through a checked and an unchecked rule */
        RuleSpec n1 = s;
        n1.check = FSK_CHK_NONE;
        n1.sync = 0x9E3B;
        rx.configure(make_dwell({s, n1}));
        std::vector<int8_t> v(2 * 4000000 / 2048 * 2048);
        std::mt19937 rng(3);
        std::normal_distribution<double> g(0.0, 6.0);
        size_t got = 0, unchecked = 0;
        for (int sec = 0; sec < 10; sec++) {
            for (auto& x : v) x = static_cast<int8_t>(std::lround(g(rng)));
            for (const auto& f : run(v, rx).f) {
                got++;
                unchecked += !f.checked;
            }
        }
        MESSAGE("noise 10 s: syncs " << rx.stats().syncs << ", frames " << got << " (unchecked " << unchecked << ")");
        CHECK(got - unchecked == 0);
        CHECK(unchecked <= 2); /* the app lists an unchecked rule only after 3 frames in 60 s */
    }
}
