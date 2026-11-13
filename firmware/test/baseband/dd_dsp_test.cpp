/*
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

/* Host tests of the drone detect GFSK (BLE advertising) demodulator in
 * baseband/dd_dsp.hpp, against a synthetic BLE 1M PHY modulator: GFSK BT 0.5,
 * h 0.5, 1 Msym/s, sampled at 4 Msps into int8 I/Q with the channel at +fs/4,
 * as the M4 sees it. */

#include "dd_dsp.hpp"
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.141592653589793238463;
constexpr size_t kSps = dd::kGfskSamplesPerSymbol;
constexpr size_t kFine = 16; /* modulator steps per output sample */
constexpr double kFs = dd::kGfskSampleRate;
constexpr double kChannelIf = kFs / 4; /* where the M4 finds the channel */
constexpr size_t kBuffer = 2048;       /* samples per DMA buffer on the M4 */

using cplx = std::complex<double>;

/* Small deterministic generator (xorshift64*), so the results don't depend on
 * the host's <random>. */
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed)
        : s{seed * 0x9E3779B97F4A7C15ull + 1} {}
    uint32_t operator()() {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return static_cast<uint32_t>((s * 0x2545F4914F6CDD1Dull) >> 32);
    }
    double uniform() { return ((*this)() + 0.5) / 4294967296.0; }
    double angle() { return 2 * kPi * uniform(); }
    /* Box-Muller, one value per call. */
    double normal(double sigma) {
        const double u1 = uniform(), u2 = uniform();
        return sigma * std::sqrt(-2 * std::log(u1)) * std::cos(2 * kPi * u2);
    }
};

/* A legacy ADV_NONCONN_IND carrying one Open Drone ID message:
 * AdvA (6) + 1E 16 FA FF 0D counter + 25 byte message = 37 byte payload. */
std::vector<uint8_t> odid_pdu(uint8_t counter, uint32_t seed) {
    Rng rng(seed);
    std::vector<uint8_t> pdu;
    pdu.push_back(0x40 | dd::ADV_NONCONN_IND); /* TxAdd = random */
    pdu.push_back(37);
    for (int i = 0; i < 6; i++) pdu.push_back(rng() & 0xFF);
    const uint8_t head[] = {0x1E, 0x16, 0xFA, 0xFF, 0x0D};
    pdu.insert(pdu.end(), head, head + 5);
    pdu.push_back(counter);
    pdu.push_back(0x12); /* Location message, version 2 */
    for (int i = 0; i < 24; i++) pdu.push_back(rng() & 0xFF);
    return pdu;
}

/* Air bits of a packet: preamble, AA, then the whitened PDU and CRC. */
std::vector<int> air_bits(const std::vector<uint8_t>& pdu, uint8_t channel) {
    std::vector<uint8_t> body = pdu;
    const uint32_t crc = dd::ble_crc24(pdu.data(), pdu.size());
    body.push_back(crc & 0xFF);
    body.push_back((crc >> 8) & 0xFF);
    body.push_back((crc >> 16) & 0xFF);
    dd::ble_whiten(channel, body.data(), body.size());

    std::vector<int> bits;
    for (int b = 0; b < 8; b++) bits.push_back((0xAA >> b) & 1);
    for (int b = 0; b < 32; b++) bits.push_back((dd::kBleAdvAccessAddress >> b) & 1);
    for (uint8_t byte : body)
        for (int b = 0; b < 8; b++) bits.push_back((byte >> b) & 1);
    return bits;
}

/* Gaussian filtered rectangular frequency pulse, BT 0.5, t in symbols. */
double gauss_pulse(double t) {
    const double c = kPi * 0.5 * std::sqrt(2.0 / std::log(2.0));
    return 0.5 * (std::erf(c * (t + 0.5)) - std::erf(c * (t - 0.5)));
}

/* gauss_pulse() at every modulator step over +-3 symbols. */
const std::vector<double>& pulse_table() {
    static std::vector<double> t;
    if (t.empty()) {
        const size_t per_symbol = kSps * kFine;
        for (size_t i = 0; i < 6 * per_symbol; i++) t.push_back(gauss_pulse((i + 0.5) / per_symbol - 3.0));
    }
    return t;
}

/* Unit amplitude GFSK at offset f_hz from 0 Hz, timing shifted by `frac`
 * sixteenths of a sample, `lead` symbols of plain carrier before and after. */
std::vector<cplx> gfsk(const std::vector<int>& bits, double f_hz, size_t frac, double phase0, size_t lead = 2) {
    const size_t per_symbol = kSps * kFine;
    const size_t symbols = bits.size() + 2 * lead;
    const double dev = 250000.0; /* h = 0.5 at 1 Msym/s */
    const auto& pulse = pulse_table();
    /* Frequency, in units of the deviation, at every step. */
    std::vector<double> freq(symbols * per_symbol, 0.0);
    for (size_t k = 0; k < bits.size(); k++) {
        const double a = bits[k] ? 1.0 : -1.0;
        /* The pulse table starts 3 symbols before the symbol's centre. */
        const long start = static_cast<long>((k + lead) * per_symbol) + static_cast<long>(per_symbol / 2) - static_cast<long>(3 * per_symbol);
        for (size_t i = 0; i < pulse.size(); i++) {
            const long at = start + static_cast<long>(i);
            if (at >= 0 && at < static_cast<long>(freq.size())) freq[at] += a * pulse[i];
        }
    }
    std::vector<cplx> out;
    out.reserve(symbols * kSps);
    double phase = phase0;
    for (size_t f = 0; f < freq.size(); f++) {
        phase += 2 * kPi * (dev * freq[f] + f_hz) / (kFs * kFine);
        if ((f % kFine) == frac) out.push_back(std::polar(1.0, phase));
    }
    return out;
}

struct Decoded {
    std::vector<uint8_t> pdu;
    int16_t rssi_cdb;
    int16_t cfo_khz;
    uint8_t channel;
};

void collect(void* context, const dd::BlePacket& p) {
    auto& v = *static_cast<std::vector<Decoded>*>(context);
    v.push_back({std::vector<uint8_t>(p.pdu, p.pdu + 2 + p.length), p.rssi_cdb, p.cfo_khz, p.channel});
}

std::vector<int8_t> quantise(const std::vector<cplx>& x) {
    std::vector<int8_t> iq(2 * x.size());
    for (size_t i = 0; i < x.size(); i++) {
        const double re = std::round(x[i].real()), im = std::round(x[i].imag());
        iq[2 * i] = static_cast<int8_t>(std::max(-128.0, std::min(127.0, re)));
        iq[2 * i + 1] = static_cast<int8_t>(std::max(-128.0, std::min(127.0, im)));
    }
    return iq;
}

/* Runs the demodulator over the stream in DMA sized pieces. */
std::vector<Decoded> run(dd::GfskDemod& demod, const std::vector<int8_t>& iq, size_t chunk = kBuffer) {
    std::vector<Decoded> out;
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at < n; at += chunk) {
        const size_t c = std::min(chunk, n - at);
        demod.execute(iq.data() + 2 * at, c, collect, &out);
    }
    return out;
}

struct Scenario {
    double esn0_db = 100;        /* noise; 100 = none */
    double cfo_hz = 0;           /* carrier offset of the wanted packets */
    size_t frac = 0;             /* timing offset, 1/16 sample */
    double amplitude = 30;       /* wanted signal, int8 units */
    double interferer_db = -100; /* adjacent channel GFSK, relative to wanted */
    double interferer_hz = -2e6; /* ... its offset from the wanted channel */
    uint8_t channel = 37;
};

/* `count` packets, `spacing` samples apart, through the scenario. Returns
 * how many decoded with the right content. */
size_t decode_rate(const Scenario& s, size_t count, uint32_t seed, size_t spacing = 3000, std::vector<Decoded>* decoded = nullptr) {
    Rng rng(seed);
    std::vector<std::vector<uint8_t>> sent;
    std::vector<cplx> x(count * spacing + 2000, cplx{});
    for (size_t p = 0; p < count; p++) {
        sent.push_back(odid_pdu(p & 0xFF, seed * 7919 + p));
        const auto wave = gfsk(air_bits(sent.back(), s.channel), kChannelIf + s.cfo_hz, s.frac, rng.angle());
        const size_t at = 500 + p * spacing;
        for (size_t i = 0; i < wave.size() && at + i < x.size(); i++) x[at + i] += s.amplitude * wave[i];
    }
    if (s.interferer_db > -90) {
        std::vector<int> bits(x.size() / kSps + 8);
        for (auto& b : bits) b = rng() & 1;
        const double a = s.amplitude * std::pow(10.0, s.interferer_db / 20);
        const auto wave = gfsk(bits, kChannelIf + s.interferer_hz, 0, 0.0, 0);
        for (size_t i = 0; i < x.size(); i++) x[i] += a * wave[i];
    }
    if (s.esn0_db < 99) {
        /* Es/N0 = A^2 T / N0; noise power per complex sample N0 fs = A^2 sps / (Es/N0). */
        const double sigma = s.amplitude * std::sqrt(kSps / std::pow(10.0, s.esn0_db / 10) / 2);
        for (auto& v : x) v += cplx(rng.normal(sigma), rng.normal(sigma));
    }
    dd::GfskDemod demod;
    demod.configure(s.channel);
    const auto got = run(demod, quantise(x));
    if (decoded) *decoded = got;
    size_t good = 0;
    std::vector<bool> seen(count, false);
    for (const auto& d : got) {
        for (size_t p = 0; p < count; p++) {
            if (!seen[p] && d.pdu == sent[p]) {
                seen[p] = true;
                good++;
                break;
            }
        }
    }
    return good;
}

/* The two tables of proc_btlerx.hpp, read from the source. */
std::vector<unsigned long> numbers_after(const std::string& text, const std::string& key, size_t count) {
    std::vector<unsigned long> out;
    size_t at = text.find(key);
    if (at == std::string::npos) return out;
    at = text.find('{', at);
    const char* p = text.c_str() + at;
    while (out.size() < count && *p) {
        if ((*p >= '0' && *p <= '9')) {
            char* end = nullptr;
            out.push_back(std::strtoul(p, &end, 0));
            p = end;
        } else {
            p++;
        }
    }
    return out;
}

std::string read_file(const char* path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace

TEST_CASE("dd whitening matches the proc_btlerx scramble table on all 40 channels") {
    const auto text = read_file(DD_BTLERX_HPP);
    REQUIRE(!text.empty());
    const auto table = numbers_after(text, "scramble_table[40][42]", 40 * 42);
    REQUIRE(table.size() == 40 * 42);
    for (uint8_t ch = 0; ch < 40; ch++) {
        uint8_t w[42] = {};
        dd::ble_whiten(ch, w, sizeof(w));
        for (size_t i = 0; i < 42; i++) {
            CAPTURE(ch);
            CAPTURE(i);
            CHECK(w[i] == table[ch * 42 + i]);
        }
    }
}

TEST_CASE("dd CRC24 matches the proc_btlerx table driven CRC") {
    const auto text = read_file(DD_BTLERX_HPP);
    const auto table = numbers_after(text, "crc_table[256]", 256);
    REQUIRE(table.size() == 256);
    /* proc_btlerx: init = crc_init_reorder(0x555555) = 0xAAAAAA, then
     * crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8). */
    Rng rng(1);
    for (int trial = 0; trial < 200; trial++) {
        std::vector<uint8_t> d(2 + (rng() % 38));
        for (auto& b : d) b = rng() & 0xFF;
        uint32_t crc = 0xAAAAAA;
        for (uint8_t b : d) crc = (table[(crc ^ b) & 0xFF] ^ (crc >> 8)) & 0xFFFFFF;
        CHECK(dd::ble_crc24(d.data(), d.size()) == crc);
    }
}

TEST_CASE("dd CRC24 bytes appended in air order leave a zero remainder") {
    /* ADV_NONCONN_IND, AdvA 11:22:33:44:55:66, flags 02 01 06. */
    const uint8_t pdu[] = {0x42, 0x09, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x02, 0x01, 0x06};
    const uint32_t crc = dd::ble_crc24(pdu, sizeof(pdu));
    uint8_t all[sizeof(pdu) + 3];
    memcpy(all, pdu, sizeof(pdu));
    all[sizeof(pdu)] = crc & 0xFF;
    all[sizeof(pdu) + 1] = (crc >> 8) & 0xFF;
    all[sizeof(pdu) + 2] = (crc >> 16) & 0xFF;
    uint32_t state = dd::reverse24(dd::kBleAdvCrcInit);
    for (uint8_t byte : all) {
        uint32_t cur = byte;
        for (int b = 0; b < 8; b++) {
            const uint32_t next = (state ^ cur) & 1;
            cur >>= 1;
            state >>= 1;
            if (next) state ^= 0xDA6000;
        }
    }
    CHECK(state == 0);
}

TEST_CASE("dd power_to_cdb is within 0.15 dB") {
    for (uint32_t p = 1; p < (1u << 30); p = p * 3 + 1) {
        const double exact = 1000.0 * std::log10(static_cast<double>(p));
        CHECK(std::fabs(dd::power_to_cdb(p) - exact) < 15.0);
    }
}

TEST_CASE("dd decodes a packet at every offset to a 2048 sample buffer boundary") {
    /* Spacing 2049: each packet starts one sample later relative to the DMA
     * buffers than the one before, so 2048 packets cover every alignment. */
    Scenario s;
    s.esn0_db = 30;
    const size_t count = kBuffer;
    const size_t good = decode_rate(s, count, 11, kBuffer + 1);
    CHECK(good == count);
}

TEST_CASE("dd state carries across odd sized execute() calls") {
    Scenario s;
    s.esn0_db = 30;
    std::vector<Decoded> ref;
    const size_t good = decode_rate(s, 64, 5, 2500, &ref);
    CHECK(good == 64);
    /* Same samples, chopped into random pieces of 1..700 samples. */
    Rng rng(9);
    std::vector<cplx> x(64 * 2500 + 2000, cplx{});
    Rng rng2(5);
    std::vector<std::vector<uint8_t>> sent;
    for (size_t p = 0; p < 64; p++) {
        sent.push_back(odid_pdu(p, 5 * 7919 + p));
        const auto wave = gfsk(air_bits(sent.back(), 37), kChannelIf, 0, rng2.angle());
        for (size_t i = 0; i < wave.size(); i++) x[500 + p * 2500 + i] += 30.0 * wave[i];
    }
    const auto iq = quantise(x);
    dd::GfskDemod demod;
    demod.configure(37);
    std::vector<Decoded> got;
    for (size_t at = 0; at < x.size();) {
        const size_t c = std::min<size_t>(1 + rng() % 700, x.size() - at);
        demod.execute(iq.data() + 2 * at, c, collect, &got);
        at += c;
    }
    REQUIRE(got.size() == 64);
    for (size_t p = 0; p < 64; p++) CHECK(got[p].pdu == sent[p]);
}

TEST_CASE("dd back to back packets") {
    /* A packet is 376 symbols = 1504 samples (+ 4 symbols of carrier from the
     * modulator): spacing 1520 leaves under 4 us between them. */
    Scenario s;
    s.esn0_db = 25;
    CHECK(decode_rate(s, 200, 21, 1520) == 200);
}

TEST_CASE("dd whitening of channels 37, 38 and 39") {
    for (uint8_t ch : {37, 38, 39}) {
        Scenario s;
        s.esn0_db = 25;
        s.channel = ch;
        std::vector<Decoded> got;
        CHECK(decode_rate(s, 100, 30 + ch, 3000, &got) == 100);
        for (const auto& d : got) CHECK(d.channel == ch);
    }
    /* And a packet whitened for 37 is not taken on 38. */
    Scenario s;
    s.esn0_db = 25;
    s.channel = 37;
    std::vector<cplx> x(4000, cplx{});
    const auto wave = gfsk(air_bits(odid_pdu(1, 2), 37), kChannelIf, 0, 0.0);
    for (size_t i = 0; i < wave.size(); i++) x[500 + i] = 30.0 * wave[i];
    dd::GfskDemod demod;
    demod.configure(38);
    CHECK(run(demod, quantise(x)).empty());
}

TEST_CASE("dd conjugate mode takes a mirrored spectrum") {
    std::vector<cplx> x(64 * 3000 + 2000, cplx{});
    Rng rng(17);
    std::vector<std::vector<uint8_t>> sent;
    for (size_t p = 0; p < 64; p++) {
        sent.push_back(odid_pdu(p, 300 + p));
        const auto wave = gfsk(air_bits(sent.back(), 37), kChannelIf, 0, rng.angle());
        for (size_t i = 0; i < wave.size(); i++) x[500 + p * 3000 + i] = 30.0 * std::conj(wave[i]);
    }
    const auto iq = quantise(x);
    /* (Without noise the normal mode takes it too: the mirror sits 2 MHz below
     * the channel, in the filter's skirt but not gone, and a 2 MHz offset is a
     * phase step of pi per sample, which the discriminator folds back.) */
    dd::GfskDemod demod;
    demod.configure(37);
    demod.set_conjugate(true);
    const auto got = run(demod, iq);
    REQUIRE(got.size() == 64);
    for (size_t p = 0; p < 64; p++) CHECK(got[p].pdu == sent[p]);
}

TEST_CASE("dd fractional timing offsets") {
    for (size_t frac = 0; frac < kFine; frac += 2) {
        Scenario s;
        s.esn0_db = 16;
        s.frac = frac;
        const size_t good = decode_rate(s, 200, 40 + frac);
        CAPTURE(frac);
        CHECK(good >= 190);
    }
}

TEST_CASE("dd RSSI and CFO estimates") {
    for (double cfo : {-150e3, -100e3, -50e3, 0.0, 50e3, 100e3, 150e3}) {
        Scenario s;
        s.esn0_db = 30;
        s.cfo_hz = cfo;
        std::vector<Decoded> got;
        CHECK(decode_rate(s, 20, 77, 3000, &got) == 20);
        double mean = 0;
        for (const auto& d : got) mean += d.cfo_khz;
        mean /= got.size();
        CAPTURE(cfo);
        CHECK(std::fabs(mean - cfo / 1000) < 15);
    }
    /* Amplitude 30 of 128: 20 log10(30 / 128) = -12.6 dBFS. */
    Scenario s;
    s.esn0_db = 40;
    std::vector<Decoded> got;
    decode_rate(s, 10, 3, 3000, &got);
    REQUIRE(!got.empty());
    CHECK(std::abs(got[0].rssi_cdb - (-1260)) < 100);
}

TEST_CASE("dd random noise for 10 s yields no CRC valid packet") {
    Rng rng(1234);
    dd::GfskDemod demod;
    demod.configure(37);
    std::vector<Decoded> got;
    std::vector<int8_t> iq(2 * kBuffer);
    const size_t buffers = static_cast<size_t>(10 * kFs) / kBuffer;
    for (size_t b = 0; b < buffers; b++) {
        for (auto& v : iq) v = static_cast<int8_t>(std::max(-128.0, std::min(127.0, std::round(rng.normal(20)))));
        demod.execute(iq.data(), kBuffer, collect, &got);
    }
    CHECK(got.empty());
    MESSAGE("10 s of noise: " << demod.stats().triggers << " soft checks, " << demod.stats().aa_hits << " syncs, "
                              << demod.stats().len_bad << " length rejects, " << demod.stats().crc_bad << " CRC rejects");
}

namespace {

/* M4 cycles of one execute() call from what it did, calibrated on the
 * Cortex-M4 code (arm-none-eabi-g++ -O3, the PDRN flags) with an instruction
 * level cycle model: filter + transition registers 17.6 cycles a sample
 * (rounded up), a soft check of all 9 candidates at most 3000 (strong signal,
 * no early exit; noise triggers cost about 1200), a demodulated bit 36 (45
 * with the per-call overhead), the end of a packet (CRC, copy, CFO division,
 * callback) at most 2500. */
uint32_t work_cycles(size_t samples, const dd::GfskDemod::Stats& before, const dd::GfskDemod::Stats& after) {
    const uint32_t ends = (after.ok - before.ok) + (after.crc_bad - before.crc_bad);
    return 18 * samples + 3000 * (after.triggers - before.triggers) + 45 * (after.bits - before.bits) + 2500 * ends;
}

/* Any PDU of the given type and payload length, CRC appended by air_bits(). */
std::vector<uint8_t> random_pdu(Rng& rng, uint8_t type, size_t len) {
    std::vector<uint8_t> pdu{static_cast<uint8_t>(0x40 | type), static_cast<uint8_t>(len)};
    for (size_t i = 0; i < len; i++) pdu.push_back(rng() & 0xFF);
    return pdu;
}

struct WorkReport {
    uint32_t max_cycles = 0;
    uint64_t total_cycles = 0;
    uint32_t max_triggers = 0;
    size_t calls = 0;
    size_t decoded = 0;
};

WorkReport run_measured(dd::GfskDemod& demod, const std::vector<int8_t>& iq) {
    WorkReport w;
    std::vector<Decoded> got;
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at + kBuffer <= n; at += kBuffer) {
        const auto before = demod.stats();
        demod.execute(iq.data() + 2 * at, kBuffer, collect, &got);
        const auto& after = demod.stats();
        const uint32_t c = work_cycles(kBuffer, before, after);
        w.max_cycles = std::max(w.max_cycles, c);
        w.total_cycles += c;
        w.max_triggers = std::max(w.max_triggers, after.triggers - before.triggers);
        w.calls++;
    }
    w.decoded = got.size();
    return w;
}

}  // namespace

TEST_CASE("dd work per 2048 sample buffer stays inside the 512 us budget with dense traffic") {
    /* 2 s at 4 Msps: adverts of random length (Poisson, 330/s, a fifth with a
     * SCAN_REQ and SCAN_RSP 150 us after), levels -30..-6 dBFS, +-50 kHz CFO,
     * so they collide and fail too, in noise that triggers the sync by itself. */
    Rng rng(4242);
    const size_t n = static_cast<size_t>(2 * kFs);
    std::vector<cplx> x(n + 4000, cplx{});
    size_t sent = 0;
    auto put = [&](size_t at, uint8_t type, size_t len, double amp, double cfo) {
        const auto wave = gfsk(air_bits(random_pdu(rng, type, len), 37), kChannelIf + cfo, rng() % kFine, rng.angle());
        for (size_t i = 0; i < wave.size() && at + i < x.size(); i++) x[at + i] += amp * wave[i];
        sent++;
        return wave.size();
    };
    for (double t = 0;;) {
        t += -std::log(rng.uniform()) / 330.0 * kFs;
        if (t >= n) break;
        size_t at = static_cast<size_t>(t);
        const double amp = 4.0 * std::pow(10.0, rng.uniform() * 1.2);
        const double cfo = (rng.uniform() - 0.5) * 100e3;
        size_t w = put(at, dd::ADV_IND, 6 + rng() % 32, amp, cfo);
        if (rng() % 5 == 0) {
            at += w + 600 - 16;
            w = put(at, dd::SCAN_REQ, 12, amp, cfo);
            at += w + 600 - 16;
            put(at, dd::SCAN_RSP, 6 + rng() % 32, amp, cfo);
        }
    }
    for (auto& v : x) v += cplx(rng.normal(6), rng.normal(6));
    dd::GfskDemod demod;
    demod.configure(37);
    const auto w = run_measured(demod, quantise(x));
    const auto& st = demod.stats();
    const double mean = static_cast<double>(w.total_cycles) / w.calls;
    MESSAGE(sent << " packets sent, " << w.decoded << " decoded; " << st.triggers << " soft checks, " << st.aa_hits << " syncs, "
                 << st.len_bad << " length / " << st.crc_bad << " CRC rejects; estimated M4 cycles per buffer: mean "
                 << static_cast<uint32_t>(mean) << " (" << 100.0 * mean / 102400 << "%), max " << w.max_cycles
                 << " (" << w.max_cycles / 200 << " us of 512)");
    CHECK(w.decoded > sent / 3);
    CHECK(st.capped == 0);           /* real traffic never meets the cap */
    CHECK(mean < 0.5 * 102400);      /* <= 50 % load */
    CHECK(w.max_cycles < 400 * 200); /* < 400 us per buffer */
}

TEST_CASE("dd sync storm: soft checks per call are capped, work stays bounded") {
    /* Back to back bursts of preamble + access address + a header with
     * length 0, 56 symbols each: every one passes the soft check and dies at
     * the length check, 9 of them per buffer, the most soft checks a signal
     * can cause. */
    Rng rng(99);
    const size_t n = 40 * kBuffer;
    std::vector<cplx> x(n + 4000, cplx{});
    uint8_t header[2] = {0x40 | dd::ADV_NONCONN_IND, 0};
    dd::ble_whiten(37, header, sizeof(header));
    std::vector<int> bits;
    for (int burst = 0; burst < 400; burst++) {
        for (int b = 0; b < 8; b++) bits.push_back((0xAA >> b) & 1);
        for (int b = 0; b < 32; b++) bits.push_back((dd::kBleAdvAccessAddress >> b) & 1);
        for (uint8_t byte : header)
            for (int b = 0; b < 8; b++) bits.push_back((byte >> b) & 1);
    }
    const auto wave = gfsk(bits, kChannelIf, 0, 0.0);
    for (size_t i = 0; i < wave.size() && i < x.size(); i++) x[i] += 40.0 * wave[i];
    for (auto& v : x) v += cplx(rng.normal(3), rng.normal(3));
    dd::GfskDemod demod;
    demod.configure(37);
    const auto w = run_measured(demod, quantise(x));
    MESSAGE("storm: " << demod.stats().triggers << " soft checks, " << demod.stats().capped << " capped, max "
                      << w.max_triggers << " per call, estimated max " << w.max_cycles << " cycles");
    CHECK(demod.stats().capped > 0);
    CHECK(w.max_triggers <= dd::GfskDemod::kMaxSyncsPerCall);
    CHECK(w.max_cycles < 400 * 200);
}

TEST_CASE("dd decode rate table (Es/N0, CFO, adjacent channel)") {
    const size_t n = 200;
    std::string table = "\nDecode rate, " + std::to_string(n) + " ODID adverts per cell (37 byte payload)\n";
    table += "Es/N0 dB |  CFO 0  | +-50k  | +-100k | +-150k | ACI 0dB | ACI+10 | ACI+20\n";
    for (int snr = 8; snr <= 20; snr += 2) {
        char line[160];
        double cells[7];
        int c = 0;
        for (double cfo : {0.0, 50e3, 100e3, 150e3}) {
            size_t good = 0, total = 0;
            for (double sign : {1.0, -1.0}) {
                if (cfo == 0 && sign < 0) continue;
                Scenario s;
                s.esn0_db = snr;
                s.cfo_hz = sign * cfo;
                good += decode_rate(s, n, 1000 + snr * 10 + c);
                total += n;
            }
            cells[c++] = 100.0 * good / total;
        }
        for (double aci : {0.0, 10.0, 20.0}) {
            Scenario s;
            s.esn0_db = snr;
            s.interferer_db = aci;
            s.amplitude = 11; /* wanted + 20 dB interferer still inside int8 */
            cells[c] = 100.0 * decode_rate(s, n, 2000 + snr * 10 + c) / n;
            c++;
        }
        snprintf(line, sizeof(line), "   %2d    | %6.1f%% | %5.1f%% | %5.1f%% | %5.1f%% | %6.1f%% | %5.1f%% | %5.1f%%\n",
                 snr, cells[0], cells[1], cells[2], cells[3], cells[4], cells[5], cells[6]);
        table += line;
        if (snr == 16) {
            CHECK(cells[0] >= 95);
            CHECK(cells[1] >= 90);
        }
        if (snr == 20) {
            CHECK(cells[2] >= 90);
            CHECK(cells[4] >= 90);
        }
    }
    MESSAGE(table);
}

namespace {
const uint8_t kAddrA[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
const uint8_t kAddrB[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x67};
}  // namespace

TEST_CASE("dd dedup suppresses the same key within 300 ms") {
    dd::RidDedup d;
    CHECK_FALSE(d.duplicate(kAddrA, 0, 5, 0x1234, 1000));
    CHECK(d.duplicate(kAddrA, 0, 5, 0x1234, 1000));
    CHECK(d.duplicate(kAddrA, 0, 5, 0x1234, 1150));
    CHECK(d.duplicate(kAddrA, 0, 5, 0x1234, 1299));
}

TEST_CASE("dd dedup forwards again 300 ms after the forward, not after the last duplicate") {
    dd::RidDedup d;
    CHECK_FALSE(d.duplicate(kAddrA, 1, 7, 99, 1000));
    CHECK(d.duplicate(kAddrA, 1, 7, 99, 1200)); /* suppressed: must not refresh */
    CHECK_FALSE(d.duplicate(kAddrA, 1, 7, 99, 1300));
    CHECK(d.duplicate(kAddrA, 1, 7, 99, 1599));
    CHECK_FALSE(d.duplicate(kAddrA, 1, 7, 99, 1600));
}

TEST_CASE("dd dedup keys on counter, type, address and payload sum") {
    dd::RidDedup d;
    CHECK_FALSE(d.duplicate(kAddrA, 0, 5, 100, 1000));
    CHECK_FALSE(d.duplicate(kAddrA, 0, 6, 100, 1001)); /* counter */
    CHECK_FALSE(d.duplicate(kAddrA, 1, 5, 100, 1002)); /* type */
    CHECK_FALSE(d.duplicate(kAddrB, 0, 5, 100, 1003)); /* address */
    CHECK_FALSE(d.duplicate(kAddrA, 0, 5, 101, 1004)); /* payload sum */
    CHECK(d.duplicate(kAddrA, 0, 5, 100, 1005));
    CHECK(d.duplicate(kAddrA, 0, 6, 100, 1005));
    CHECK(d.duplicate(kAddrA, 1, 5, 100, 1005));
    CHECK(d.duplicate(kAddrB, 0, 5, 100, 1005));
    CHECK(d.duplicate(kAddrA, 0, 5, 101, 1005));
}

TEST_CASE("dd dedup with more distinct keys than slots replaces the oldest") {
    dd::RidDedup d;
    const uint32_t n = dd::kDedupSlots + 8;
    for (uint32_t i = 0; i < n; i++) CHECK_FALSE(d.duplicate(kAddrA, 0, static_cast<uint8_t>(i), 1, 1000 + i));
    /* The most recent kDedupSlots are still known. */
    for (uint32_t i = n - dd::kDedupSlots; i < n; i++) CHECK(d.duplicate(kAddrA, 0, static_cast<uint8_t>(i), 1, 1100));
    /* The oldest were replaced and are forwarded again. */
    CHECK_FALSE(d.duplicate(kAddrA, 0, 0, 1, 1100));
}

TEST_CASE("dd dedup reset forgets everything") {
    dd::RidDedup d;
    CHECK_FALSE(d.duplicate(kAddrA, 0, 1, 1, 1000));
    d.reset();
    CHECK_FALSE(d.duplicate(kAddrA, 0, 1, 1, 1001));
}

TEST_CASE("dd dedup rate cap: 8 forwards per 100 ms window") {
    dd::RidDedup d;
    int pass = 0, refused = 0;
    for (uint32_t i = 0; i < 20; i++) (d.rate_ok(5000 + i) ? pass : refused)++;
    CHECK(pass == 8);
    CHECK(refused == 12);
    CHECK_FALSE(d.rate_ok(5099));
    /* A new window starts at >= 100 ms after the window start. */
    for (int i = 0; i < 8; i++) CHECK(d.rate_ok(5100 + i));
    CHECK_FALSE(d.rate_ok(5110));
}

TEST_CASE("dd dedup survives t_ms wrap-around") {
    dd::RidDedup d;
    const uint32_t t0 = 0xFFFFFFF0u;
    CHECK_FALSE(d.duplicate(kAddrA, 0, 1, 1, t0));
    CHECK(d.duplicate(kAddrA, 0, 1, 1, t0 + 100)); /* wrapped past 0 */
    CHECK(d.duplicate(kAddrA, 0, 1, 1, t0 + 299));
    CHECK_FALSE(d.duplicate(kAddrA, 0, 1, 1, t0 + 300));

    dd::RidDedup r;
    int pass = 0;
    for (uint32_t i = 0; i < 20; i++)
        if (r.rate_ok(t0 + i * 2)) pass++; /* 40 ms, crosses the wrap */
    CHECK(pass == 8);
    for (int i = 0; i < 8; i++) CHECK(r.rate_ok(t0 + 100 + i)); /* new window across the wrap */
    CHECK_FALSE(r.rate_ok(t0 + 110));
}
