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

/* Host tests of the drone detect pattern receiver (dd::PatternDemod) and the
 * DJI advert filter (dd_ble_vendor.hpp), synthetic and on real recordings.
 *
 * Real recordings stay outside the repository; each test that needs one
 * says "skipped" and passes when it is absent:
 *   DD_IQ_DIR   HackRF Pro .ci8 recordings with SigMF-style .json sidecars
 *               (the dronedetect-bench set: toy_SLU23T24T23_*, ambient_*,
 *               c5_ble4_*, c5silent_*)
 *   DD_DJI_DIR  sdrmm fixture remote_id_ble_dji_mini4_4m.sigmf-{meta,data}
 *               (RFUAV DJI Mini 4 Pro, BLE channel 38, 4 MS/s cf32)
 * A recording is brought to what the M4 sees in a GFSK dwell: shifted so the
 * channel sits at +1 MHz (the LO a quarter of 4 Msps below it), low-pass
 * filtered and resampled to 4 Msps, rounded to int8 at the recording's own
 * scale, then fed in 2048 sample buffers. */

#include "dd_dsp.hpp"
#include "dd_ble_vendor.hpp"
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.141592653589793238463;
constexpr double kFs = dd::kGfskSampleRate;
constexpr size_t kSps = dd::kGfskSamplesPerSymbol;
constexpr size_t kBuffer = 2048;

using cplx = std::complex<double>;
using cplxf = std::complex<float>;

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
    double normal(double sigma) {
        const double u1 = uniform(), u2 = uniform();
        return sigma * std::sqrt(-2 * std::log(u1)) * std::cos(2 * kPi * u2);
    }
};

/* The RC toy (FCC ID SLU23T24T23) as recorded: 3 bytes 0x55, then a constant
 * 148 bit body, MSB first, bit 1 = positive deviation. */
const uint8_t kToyBody[] = {0x27, 0x15, 0x1b, 0x97, 0x28, 0xd7, 0x20, 0x30, 0x48, 0x13,
                            0xdb, 0x49, 0xfc, 0xbb, 0x14, 0x6b, 0x93, 0x77, 0x80};
constexpr size_t kToyBodyBits = 148;
/* Sync of the firmware's built-in entry: last preamble byte + 4 body bytes. */
constexpr uint64_t kToySync40 = 0x5527151B97ull;

std::vector<int> toy_bits() {
    std::vector<int> b;
    for (int i = 0; i < 24; i++) b.push_back((0x55 >> (7 - (i & 7))) & 1);
    for (size_t i = 0; i < kToyBodyBits; i++) b.push_back((kToyBody[i / 8] >> (7 - (i & 7))) & 1);
    return b;
}

/* Body bits k.. as a right aligned n bit value (first bit in the MSB). */
uint64_t toy_body_value(size_t k, size_t n) {
    uint64_t v = 0;
    for (size_t i = 0; i < n; i++) v = (v << 1) | ((kToyBody[(k + i) / 8] >> (7 - ((k + i) & 7))) & 1);
    return v;
}

double gauss_pulse(double t) {
    const double c = kPi * 0.5 * std::sqrt(2.0 / std::log(2.0));
    return 0.5 * (std::erf(c * (t + 0.5)) - std::erf(c * (t - 0.5)));
}

/* GFSK BT 0.5 at 1 Mb/s, deviation dev_hz, centred on f_hz, timing shifted
 * by frac sixteenths of a sample, 4 symbols of carrier either side. */
std::vector<cplx> gfsk(const std::vector<int>& bits, double f_hz, double dev_hz, size_t frac = 0, double phase0 = 0.3) {
    constexpr size_t kFine = 16;
    const size_t per = kSps * kFine;
    const size_t lead = 4;
    const size_t symbols = bits.size() + 2 * lead;
    static std::vector<double> pulse;
    if (pulse.empty())
        for (size_t i = 0; i < 6 * per; i++) pulse.push_back(gauss_pulse((i + 0.5) / per - 3.0));
    std::vector<double> freq(symbols * per, 0.0);
    for (size_t k = 0; k < bits.size(); k++) {
        const double a = bits[k] ? 1.0 : -1.0;
        const long start = static_cast<long>((k + lead) * per) + static_cast<long>(per / 2) - static_cast<long>(3 * per);
        for (size_t i = 0; i < 6 * per; i++) {
            const long at = start + static_cast<long>(i);
            if (at >= 0 && at < static_cast<long>(freq.size())) freq[at] += a * pulse[i];
        }
    }
    std::vector<cplx> out;
    double phase = phase0;
    for (size_t f = 0; f < freq.size(); f++) {
        phase += 2 * kPi * (dev_hz * freq[f] + f_hz) / (kFs * kFine);
        if ((f % kFine) == frac) out.push_back(std::polar(1.0, phase));
    }
    return out;
}

std::vector<int8_t> quantise(const std::vector<cplx>& x) {
    std::vector<int8_t> iq(2 * x.size());
    for (size_t i = 0; i < x.size(); i++) {
        iq[2 * i] = static_cast<int8_t>(std::max(-128.0, std::min(127.0, std::round(x[i].real()))));
        iq[2 * i + 1] = static_cast<int8_t>(std::max(-128.0, std::min(127.0, std::round(x[i].imag()))));
    }
    return iq;
}

void collect_hit(void* ctx, const dd::PatternHit& h) {
    static_cast<std::vector<dd::PatternHit>*>(ctx)->push_back(h);
}

std::vector<dd::PatternHit> run_pattern(dd::PatternDemod& d, const std::vector<int8_t>& iq, size_t chunk = kBuffer) {
    std::vector<dd::PatternHit> out;
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at < n; at += chunk) d.execute(iq.data() + 2 * at, std::min(chunk, n - at), collect_hit, &out);
    /* Flush: a hit waits for the samples after it (merge window, payload). */
    std::vector<int8_t> tail(2 * 1024, 0);
    d.execute(tail.data(), 1024, collect_hit, &out);
    return out;
}

dd::PatternConfig toy_config(uint8_t max_errors = 2, uint8_t payload_bits = 0) {
    dd::PatternConfig c;
    c.sync = kToySync40;
    c.sync_bits = 40;
    c.max_errors = max_errors;
    c.payload_bits = payload_bits;
    return c;
}

/* One toy packet with its sync ending (last sync bit's centre) near sample
 * `end`, amplitude 40, in a buffer of `len` samples, optional noise. */
std::vector<cplx> toy_signal(size_t len, size_t at, double cfo = 0, size_t frac = 0, const std::vector<int>* bits = nullptr, double amp = 40) {
    std::vector<cplx> x(len, cplx{});
    const auto b = bits ? *bits : toy_bits();
    const auto w = gfsk(b, kFs / 4 + cfo, 250000, frac);
    for (size_t i = 0; i < w.size() && at + i < len; i++) x[at + i] += amp * w[i];
    return x;
}

/* Sample (in the stream) of the last bit of the 40 bit toy sync for a packet
 * placed at `at`: 4 lead symbols, 24 preamble bits, 32 body bits. */
size_t toy_sync_end(size_t at) {
    return at + (4 + 24 + 32 - 1) * kSps;
}

}  // namespace

/* ---------------------------------------------------------------- synthetic */

TEST_CASE("pattern: max errors follow the 16 bit pieces") {
    CHECK(dd::pattern_max_errors(8) == 0);
    CHECK(dd::pattern_max_errors(16) == 0);
    CHECK(dd::pattern_max_errors(17) == 1);
    CHECK(dd::pattern_max_errors(40) == 2);
    CHECK(dd::pattern_max_errors(48) == 2);
    CHECK(dd::pattern_max_errors(49) == 3);
    CHECK(dd::pattern_max_errors(64) == 3);
    dd::PatternDemod d;
    auto c = toy_config(9);
    CHECK(d.configure(c));
    CHECK(d.config().max_errors == 2);
    c.sync_bits = 4;
    CHECK_FALSE(d.configure(c));
    c.sync_bits = 40;
    c.rate = dd::PatternRate::R2M;
    CHECK_FALSE(d.configure(c));
    c.rate = dd::PatternRate::R1M;
    c.payload_bits = 65;
    CHECK_FALSE(d.configure(c));
}

TEST_CASE("pattern: one hit at every offset across a 2048 sample buffer boundary, payload intact") {
    /* The sync's end walks over two whole symbols either side of the
     * boundary in 1/16 sample steps every 4th offset. */
    for (int off = -12; off <= 12; off++) {
        for (size_t frac : {size_t{0}, size_t{5}, size_t{11}}) {
            const size_t end = kBuffer + off;
            const size_t at = end - (toy_sync_end(0));
            const auto iq = quantise(toy_signal(3 * kBuffer, at, 0, frac));
            dd::PatternDemod d;
            d.configure(toy_config(2, 64));
            const auto hits = run_pattern(d, iq);
            REQUIRE(hits.size() == 1);
            CHECK(hits[0].errors == 0);
            CHECK(hits[0].payload_bits == 64);
            CHECK(hits[0].payload == toy_body_value(32, 64));
            /* Within a sample of the last sync bit's centre (filter delay ~3). */
            const int32_t err = static_cast<int32_t>(hits[0].end_sample - end);
            CHECK(err >= -1);
            CHECK(err <= 6);
        }
    }
}

TEST_CASE("pattern: state carries across odd sized execute() calls") {
    const auto iq = quantise(toy_signal(3 * kBuffer, 1000));
    for (size_t chunk : {size_t{1}, size_t{3}, size_t{7}, size_t{333}, size_t{4097}}) {
        dd::PatternDemod d;
        d.configure(toy_config(2, 32));
        const auto hits = run_pattern(d, iq, chunk);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].payload == toy_body_value(32, 32));
    }
}

TEST_CASE("pattern: bit errors up to the tolerance, not beyond") {
    /* Flipped bits on air inside the sync (preamble byte or body), one bit
     * in each 16 bit piece at most so a clean piece remains up to 2. */
    const size_t sync_first = 24 - 8; /* index in toy_bits() of the sync's first bit */
    const size_t flips[3] = {sync_first + 3, sync_first + 21, sync_first + 37};
    for (size_t nflip = 0; nflip <= 3; nflip++) {
        auto bits = toy_bits();
        for (size_t k = 0; k < nflip; k++) bits[flips[k]] ^= 1;
        const auto iq = quantise(toy_signal(2 * kBuffer, 700, 0, 0, &bits));
        for (uint8_t tol = 0; tol <= 2; tol++) {
            dd::PatternDemod d;
            d.configure(toy_config(tol));
            const auto hits = run_pattern(d, iq);
            if (nflip <= tol) {
                REQUIRE(hits.size() == 1);
                CHECK(hits[0].errors == nflip);
            } else {
                CHECK(hits.empty());
            }
        }
    }
}

TEST_CASE("pattern: inverted polarity") {
    auto bits = toy_bits();
    for (auto& b : bits) b ^= 1;
    const auto iq = quantise(toy_signal(2 * kBuffer, 500, 0, 0, &bits));
    dd::PatternDemod d;
    d.configure(toy_config(2, 16));
    CHECK(run_pattern(d, iq).empty());
    auto c = toy_config(2, 16);
    c.invert = true;
    d.configure(c);
    const auto hits = run_pattern(d, iq);
    REQUIRE(hits.size() == 1);
    CHECK(hits[0].payload == toy_body_value(32, 16));
}

TEST_CASE("pattern: back to back packets, carrier offset, RSSI") {
    /* Three packets 300 us apart (the toy's own are 724 us), +-100 kHz CFO
     * and 20 dB Es/N0. */
    Rng rng(77);
    std::vector<cplx> x(12 * kBuffer, cplx{});
    const double amp = 30;
    for (int p = 0; p < 3; p++) {
        const double cfo = (p - 1) * 100e3;
        const auto w = gfsk(toy_bits(), kFs / 4 + cfo, 250000, p * 3);
        for (size_t i = 0; i < w.size(); i++) x[3000 + p * 1200 + i] += amp * w[i];
    }
    const double sigma = amp * std::sqrt(kSps / std::pow(10.0, 20 / 10.0) / 2);
    for (auto& v : x) v += cplx(rng.normal(sigma), rng.normal(sigma));
    dd::PatternDemod d;
    d.configure(toy_config(2));
    const auto hits = run_pattern(d, quantise(x));
    REQUIRE(hits.size() == 3);
    /* 30 int8 amplitude: 20 log10(30 / 128) = -12.6 dBFS. */
    for (const auto& h : hits) CHECK(std::abs(h.rssi_cdb - (-1260)) < 150);
}

TEST_CASE("pattern: 10 s of noise and of random GFSK give no hit") {
    Rng rng(4321);
    dd::PatternDemod d;
    d.configure(toy_config(2));
    std::vector<dd::PatternHit> got;
    std::vector<int8_t> iq(2 * kBuffer);
    const size_t buffers = static_cast<size_t>(10 * kFs) / kBuffer;
    for (size_t b = 0; b < buffers; b++) {
        for (auto& v : iq) v = static_cast<int8_t>(std::max(-128.0, std::min(127.0, std::round(rng.normal(20)))));
        d.execute(iq.data(), kBuffer, collect_hit, &got);
    }
    MESSAGE("10 s noise: " << d.stats().cands << " piece passes (" << d.stats().cands / 10 << "/s), "
                           << d.stats().matches << " matches");
    CHECK(got.empty());

    /* Random 1 Mb/s GFSK (other GFSK links): 2.5 s of continuous traffic. */
    std::vector<int> bits(2500000);
    for (auto& b : bits) b = rng() & 1;
    const auto w = gfsk(bits, kFs / 4, 250000);
    std::vector<cplx> x(w.size());
    for (size_t i = 0; i < w.size(); i++) x[i] = 40.0 * w[i] + cplx(rng.normal(2), rng.normal(2));
    dd::PatternDemod d2;
    d2.configure(toy_config(2));
    const auto hits = run_pattern(d2, quantise(x));
    MESSAGE("2.5 s random GFSK: " << d2.stats().cands << " piece passes, " << d2.stats().matches << " matches, " << hits.size()
                                  << " hits (expected about 2.5e6 x 4 x 821 / 2^40 = 0.007)");
    CHECK(hits.empty());
}

/* ---------------------------------------------------------------- DJI filter */

TEST_CASE("vendor: DJI by OUI (public only), name prefix, company id") {
    const uint8_t dji_pub[6] = {0xE4, 0x7A, 0x2C, 0xAB, 0x98, 0xF9};
    const uint8_t other[6] = {0xC0, 0x11, 0x22, 0x33, 0x44, 0x55};
    const uint8_t name_adv[] = {0x02, 0x01, 0x06, 0x13, 0x09, 'D', 'J', 'I', '-', 'M', 'I', 'N', 'I', '4', '-', 'P', 'r', 'o', '-', 'T', 'E', 'S', 'T'};
    const uint8_t cid_adv[] = {0x05, 0xFF, 0xAA, 0x08, 0x01, 0x02};
    const uint8_t plain_adv[] = {0x02, 0x01, 0x06, 0x05, 0x09, 'P', 'h', 'o', 'n'};
    const uint8_t* name;
    size_t nl;
    CHECK(dd::dji_ble_match(0x00, dji_pub, plain_adv, sizeof(plain_adv), &name, &nl) == dd::VENDOR_OUI);
    CHECK(nl == 4);
    CHECK(dd::dji_ble_match(0x40, dji_pub, plain_adv, sizeof(plain_adv), &name, &nl) == 0); /* random address */
    CHECK(dd::dji_ble_match(0x46, other, name_adv, sizeof(name_adv), &name, &nl) == dd::VENDOR_NAME);
    CHECK(nl == 18);
    CHECK(std::string(reinterpret_cast<const char*>(name), nl) == "DJI-MINI4-Pro-TEST");
    CHECK(dd::dji_ble_match(0x44, other, cid_adv, sizeof(cid_adv), &name, &nl) == dd::VENDOR_CID);
    CHECK(dd::dji_ble_match(0x03, dji_pub, name_adv, sizeof(name_adv), &name, &nl) == 0); /* SCAN_REQ: no AdvData */
    /* Malformed lengths stop the walk without reading past the end. */
    const uint8_t bad[] = {0x1E, 0x09, 'D', 'J'};
    CHECK(dd::dji_ble_match(0x42, other, bad, sizeof(bad), &name, &nl) == 0);
    CHECK(name == nullptr);
}

/* ---------------------------------------------------------------- recordings */

namespace {

std::string env_dir(const char* var) {
    const char* v = std::getenv(var);
    return v ? std::string(v) : std::string();
}

bool file_exists(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return f.good();
}

std::string slurp(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/* First number after "key": in a JSON text (enough for the sidecars). */
double json_number(const std::string& text, const std::string& key) {
    const size_t at = text.find("\"" + key + "\"");
    if (at == std::string::npos) return 0;
    const size_t colon = text.find(':', at + key.size() + 2); /* the keys have colons too */
    return std::strtod(text.c_str() + colon + 1, nullptr);
}

/* Windowed sinc low-pass (Blackman), taps at rate fs. */
std::vector<float> lowpass(size_t taps, double cutoff, double fs) {
    if (taps < 2) return std::vector<float>(1, 1.0f);
    std::vector<float> h(taps);
    double sum = 0;
    for (size_t i = 0; i < taps; i++) {
        const double t = i - (taps - 1) / 2.0;
        const double x = 2 * cutoff / fs * t;
        const double sinc = t == 0 ? 1.0 : std::sin(kPi * x) / (kPi * x);
        const double w = 0.42 - 0.5 * std::cos(2 * kPi * i / (taps - 1)) + 0.08 * std::cos(4 * kPi * i / (taps - 1));
        h[i] = static_cast<float>(sinc * w);
        sum += h[i];
    }
    for (auto& v : h) v = static_cast<float>(v / sum);
    return h;
}

/* A recording brought to the M4's view of a GFSK dwell on `channel_hz`:
 * int8 I/Q at 4 Msps with the channel at +1 MHz. in: complex samples at
 * fs_in centred on fc. up/down: rational resampling to 4 Msps. scale:
 * multiplies before rounding. */
std::vector<int8_t> to_dwell(const std::vector<cplxf>& in, double fs_in, double fc, double channel_hz, double scale = 1.0) {
    const double lo = channel_hz - kFs / 4;
    const double shift = fc - lo; /* the recording's 0 Hz lands here */
    /* Rational up / down with fs_in * up / down = 4 Msps. */
    size_t up = 1, down = 1;
    const double ratio = fs_in / kFs;
    for (size_t u = 1; u <= 8; u++) {
        const double d = ratio * u;
        if (std::abs(d - std::round(d)) < 1e-9) {
            up = u;
            down = static_cast<size_t>(std::round(d));
            break;
        }
    }
    const double fs_up = fs_in * up;
    const auto h = lowpass(up == 1 && down == 1 ? 1 : 127 * up, 1.8e6, fs_up);
    const size_t L = h.size();
    /* Mix first (at fs_in). */
    std::vector<cplxf> x(in.size());
    double ph = 0;
    const double w = 2 * kPi * shift / fs_in;
    for (size_t i = 0; i < in.size(); i++) {
        x[i] = in[i] * cplxf(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
        ph += w;
        if (ph > kPi) ph -= 2 * kPi;
        if (ph < -kPi) ph += 2 * kPi;
    }
    std::vector<int8_t> out;
    out.reserve(2 * (in.size() * up / down + 1));
    const float gain = static_cast<float>(up * scale);
    for (size_t t = 0;; t += down) {
        /* Output at upsampled time t: sum over inputs i with 0 <= t - i up < L. */
        if (t / up >= x.size()) break;
        cplxf acc{};
        const size_t i_hi = t / up;
        const size_t i_lo = t + 1 > L ? (t + 1 - L + up - 1) / up : 0;
        for (size_t i = i_lo; i <= i_hi; i++) acc += h[t - i * up] * x[i];
        acc *= gain;
        out.push_back(static_cast<int8_t>(std::max(-128.f, std::min(127.f, std::round(acc.real())))));
        out.push_back(static_cast<int8_t>(std::max(-128.f, std::min(127.f, std::round(acc.imag())))));
    }
    return out;
}

struct Recording {
    std::vector<cplxf> x;
    double fs = 0, fc = 0;
    bool ok = false;
};

Recording load_ci8(const std::string& dir, const std::string& stem) {
    Recording r;
    const std::string data = dir + "/" + stem + ".ci8", meta = dir + "/" + stem + ".json";
    if (dir.empty() || !file_exists(data) || !file_exists(meta)) return r;
    const std::string m = slurp(meta);
    r.fs = json_number(m, "core:sample_rate");
    r.fc = json_number(m, "core:frequency");
    if (r.fs < 1e6 || r.fc < 1e6) return r;
    const std::string raw = slurp(data);
    r.x.resize(raw.size() / 2);
    for (size_t i = 0; i < r.x.size(); i++) r.x[i] = cplxf(static_cast<int8_t>(raw[2 * i]), static_cast<int8_t>(raw[2 * i + 1]));
    r.ok = r.fs > 0 && r.fc > 0 && !r.x.empty();
    return r;
}

struct PatternRun {
    size_t hits = 0;
    double seconds = 0;
    size_t by_errors[4] = {};
    int rssi_min = 0, rssi_max = -20000;
    double rate() const { return seconds > 0 ? hits / seconds : 0; }
};

PatternRun pattern_on(const std::vector<int8_t>& iq, const dd::PatternConfig& c) {
    dd::PatternDemod d;
    d.configure(c);
    const auto hits = run_pattern(d, iq);
    PatternRun r;
    r.hits = hits.size();
    r.seconds = iq.size() / 2 / kFs;
    for (const auto& h : hits) {
        r.by_errors[std::min<size_t>(h.errors, 3)]++;
        r.rssi_min = std::min<int>(r.rssi_min, h.rssi_cdb);
        r.rssi_max = std::max<int>(r.rssi_max, h.rssi_cdb);
    }
    return r;
}

/* Syncs compared on the recordings: the 40 bit one of the firmware's
 * built-in entry and longer ones out of the constant body. */
struct SyncCase {
    const char* name;
    uint64_t sync;
    uint8_t bits;
    uint8_t max_errors;
};
const SyncCase kSyncs[] = {
    {"40b <=2 (built-in)", kToySync40, 40, 2},
    {"40b <=1", kToySync40, 40, 1},
    {"48b <=2", 0x5527151B9728ull, 48, 2},
    {"64b <=3", 0x5527151B9728D720ull, 64, 3},
};

void collect_packet(void* ctx, const dd::BlePacket& p) {
    static_cast<std::vector<dd::BlePacket>*>(ctx)->push_back(p);
}

std::vector<dd::BlePacket> run_ble(uint8_t channel, const std::vector<int8_t>& iq, dd::GfskDemod::Stats* stats = nullptr) {
    dd::GfskDemod d;
    d.configure(channel);
    std::vector<dd::BlePacket> out;
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at < n; at += kBuffer) d.execute(iq.data() + 2 * at, std::min(kBuffer, n - at), collect_packet, &out);
    if (stats) *stats = d.stats();
    return out;
}

/* The proc_drone_detect test for Open Drone ID: AD 0x16, UUID FFFA, app code 0x0D. */
bool is_odid(const dd::BlePacket& p) {
    const uint8_t t = p.pdu_type();
    if (t != dd::ADV_IND && t != dd::ADV_NONCONN_IND && t != dd::ADV_SCAN_IND && t != dd::SCAN_RSP) return false;
    const uint8_t* adv = p.pdu + 8;
    const size_t len = p.length - 6;
    for (size_t i = 0; i + 1 < len;) {
        const size_t l = adv[i];
        if (l == 0 || i + 1 + l > len) break;
        if (adv[i + 1] == 0x16 && l >= 4 && adv[i + 2] == 0xFA && adv[i + 3] == 0xFF && adv[i + 4] == 0x0D) return true;
        i += 1 + l;
    }
    return false;
}

std::string hex(const uint8_t* p, size_t n, const char* sep = "") {
    std::string s;
    char b[4];
    for (size_t i = 0; i < n; i++) {
        snprintf(b, sizeof(b), "%02X", p[i]);
        if (i) s += sep;
        s += b;
    }
    return s;
}

std::string addr_of(const dd::BlePacket& p) {
    uint8_t a[6];
    for (size_t i = 0; i < 6; i++) a[i] = p.pdu[7 - i];
    return hex(a, 6, ":");
}

const char* const kPduNames[16] = {"ADV_IND", "ADV_DIRECT_IND", "ADV_NONCONN_IND", "SCAN_REQ", "SCAN_RSP", "CONNECT_IND", "ADV_SCAN_IND", "?7",
                                   "?8", "?9", "?10", "?11", "?12", "?13", "?14", "?15"};

}  // namespace

TEST_CASE("recording: RC toy SLU23T24T23 found on both channels, never in the ambient files") {
    const std::string dir = env_dir("DD_IQ_DIR");
    struct File {
        const char* stem;
        double channel_mhz;
        bool toy;
    };
    const File files[] = {
        {"toy_SLU23T24T23_c5silent_2430MHz_20Msps", 2429.0, true},
        {"toy_SLU23T24T23_c5silent_2448MHz_20Msps", 2449.0, true},
        {"ambient_c5silent_2430MHz_20Msps", 2429.0, false},
        {"ambient_c5silent_2448MHz_20Msps", 2449.0, false},
    };
    std::string table = "\nPattern receiver on the recordings (hits/s; by bit errors 0/1/2/3; RSSI range dBFS at LNA 8 VGA 16)\n";
    bool any = false;
    for (const auto& f : files) {
        const auto rec = load_ci8(dir, f.stem);
        if (!rec.ok) continue;
        any = true;
        const auto iq = to_dwell(rec.x, rec.fs, rec.fc, f.channel_mhz * 1e6);
        for (const auto& s : kSyncs) {
            dd::PatternConfig c;
            c.sync = s.sync;
            c.sync_bits = s.bits;
            c.max_errors = s.max_errors;
            const auto r = pattern_on(iq, c);
            char line[200];
            snprintf(line, sizeof(line), "  %-42s %7.1f MHz  %-19s %6.1f/s  %zu/%zu/%zu/%zu  %.1f..%.1f\n", f.stem, f.channel_mhz, s.name, r.rate(),
                     r.by_errors[0], r.by_errors[1], r.by_errors[2], r.by_errors[3], r.rssi_min / 100.0, r.rssi_max / 100.0);
            table += line;
            if (s.sync == kToySync40 && s.max_errors == 2) {
                if (f.toy) {
                    CHECK(r.rate() > 60.0); /* toydet.py: 67.3/s per channel */
                    CHECK(r.rate() < 75.0);
                } else {
                    CHECK(r.hits == 0);
                }
            }
        }
    }
    if (!any) {
        MESSAGE("skipped: no toy / ambient recordings in DD_IQ_DIR");
        return;
    }
    MESSAGE(table);
}

TEST_CASE("recording: no pattern hit anywhere else in the 2.4 GHz recordings") {
    /* Every 2 MHz channel within +-8 MHz of the centre of each 2.4 GHz
     * ambient / toy / C5 recording (the toy's own two channels and their
     * immediate neighbours excluded): false hits at the built-in setting. */
    const std::string dir = env_dir("DD_IQ_DIR");
    const char* const stems[] = {
        "ambient_c5silent_2412MHz_20Msps",
        "ambient_c5silent_2430MHz_20Msps",
        "ambient_c5silent_2448MHz_20Msps",
        "ambient_c5silent_2466MHz_20Msps",
        "toy_SLU23T24T23_c5silent_2412MHz_20Msps",
        "toy_SLU23T24T23_c5silent_2466MHz_20Msps",
        "c5_hop154_4ms_ch11-26_2440MHz_20Msps",
        "c5_wifi_beacon_ch6_1Mbps_2437MHz_20Msps",
        "c5_ble5_coded_ch37in_2408MHz_20Msps",
    };
    size_t total_hits = 0;
    double channel_seconds = 0;
    std::string where;
    for (const char* stem : stems) {
        const auto rec = load_ci8(dir, stem);
        if (!rec.ok) continue;
        for (int off = -8; off <= 8; off += 2) {
            const double ch = std::round(rec.fc / 1e6) + off;
            if (std::abs(ch - 2429) <= 1 || std::abs(ch - 2449) <= 1) continue;
            const auto iq = to_dwell(rec.x, rec.fs, rec.fc, ch * 1e6);
            const auto r = pattern_on(iq, toy_config(2));
            channel_seconds += r.seconds;
            total_hits += r.hits;
            if (r.hits) where += std::string(" ") + stem + "@" + std::to_string(static_cast<int>(ch)) + ":" + std::to_string(r.hits);
        }
    }
    if (channel_seconds == 0) {
        MESSAGE("skipped: no recordings in DD_IQ_DIR");
        return;
    }
    MESSAGE("false hits at the built-in setting: " << total_hits << " in " << channel_seconds << " channel-seconds" << where);
    CHECK(total_hits == 0);
}

TEST_CASE("recording: BLE4 Remote ID from the C5 at the firmware's own tuning") {
    const std::string dir = env_dir("DD_IQ_DIR");
    const auto rec = load_ci8(dir, "c5_ble4_legacy_ch37_fwtuning_2401MHz_4Msps");
    if (!rec.ok) {
        MESSAGE("skipped: c5_ble4_legacy_ch37_fwtuning_2401MHz_4Msps not in DD_IQ_DIR");
        return;
    }
    REQUIRE(rec.fs == kFs);
    std::vector<int8_t> iq(2 * rec.x.size());
    for (size_t i = 0; i < rec.x.size(); i++) {
        iq[2 * i] = static_cast<int8_t>(rec.x[i].real());
        iq[2 * i + 1] = static_cast<int8_t>(rec.x[i].imag());
    }
    dd::GfskDemod::Stats st{};
    const auto pk = run_ble(37, iq, &st);
    /* The C5: the address of the Remote ID adverts. */
    std::vector<uint32_t> t_rid;
    std::string c5;
    size_t odid = 0, others = 0;
    std::vector<std::string> payloads;
    for (const auto& p : pk) {
        if (is_odid(p)) {
            odid++;
            if (c5.empty()) c5 = addr_of(p);
            if (addr_of(p) == c5) {
                t_rid.push_back(p.end_sample);
                payloads.push_back(hex(p.pdu + 8, p.length - 6));
            }
        } else {
            others++;
        }
    }
    std::sort(payloads.begin(), payloads.end());
    const size_t distinct = std::unique(payloads.begin(), payloads.end()) - payloads.begin();
    /* Sent: the C5 advertises every 30 ms + advDelay (0..10 ms) on 37, 38, 39;
     * gaps of more than 1.5 intervals are packets missed here. */
    size_t missed = 0;
    std::string gaps;
    for (size_t i = 1; i < t_rid.size(); i++) {
        const double gap_ms = (t_rid[i] - t_rid[i - 1]) / 4000.0;
        gaps += " " + std::to_string(static_cast<int>(std::round(gap_ms)));
        if (gap_ms > 52) missed += static_cast<size_t>(std::round(gap_ms / 35.0)) - 1;
    }
    MESSAGE("C5 gaps (ms):" << gaps);
    const double seconds = rec.x.size() / kFs;
    MESSAGE("BLE4 recording (" << seconds << " s, ch37): " << pk.size() << " CRC-valid adverts (" << odid << " Remote ID, " << others
                               << " others), " << st.aa_hits << " syncs, " << st.crc_bad << " CRC fails; C5 " << c5 << ": " << t_rid.size()
                               << " decoded, ~" << missed << " missed by the gap count (decoded / sent " << t_rid.size() << "/"
                               << t_rid.size() + missed << "), " << distinct << " distinct payloads ("
                               << distinct / seconds << "/s); nominal sent at 30 ms + ~5 ms: ~" << static_cast<int>(seconds / 0.035));
    /* Baseline of 2026-11-11: 47 decoded, 9 missed (16 %), 21 distinct. This
     * recording has a strong third-party emitter on ch37 (106 us bursts at
     * -9 dBFS, 24 dB over the C5, every 1 ms for ~0.5 s of the 2 s): a 376 us
     * packet overlapping one is lost whatever the receiver, which accounts
     * for a loss of that order. A regression shows as fewer decodes here. */
    CHECK(t_rid.size() >= 45); /* ~57 sent in 2 s */
    CHECK(missed <= 12);
    CHECK(distinct >= 19);
    CHECK(odid == t_rid.size()); /* nobody else sends Remote ID here */
}

TEST_CASE("recording: no Remote ID in the C5-silent BLE control and ambient recordings") {
    const std::string dir = env_dir("DD_IQ_DIR");
    struct File {
        const char* stem;
        double channel_mhz;
        uint8_t ble;
    };
    const File files[] = {
        {"c5silent_ble_control_2404MHz_10Msps", 2402, 37},
        {"c5silent_ble5_control_2408MHz_20Msps", 2402, 37},
        {"ambient_c5silent_2430MHz_20Msps", 2426, 38},
        {"toy_SLU23T24T23_c5silent_2430MHz_20Msps", 2426, 38},
    };
    bool any = false;
    for (const auto& f : files) {
        const auto rec = load_ci8(dir, f.stem);
        if (!rec.ok) continue;
        any = true;
        const auto iq = to_dwell(rec.x, rec.fs, rec.fc, f.channel_mhz * 1e6);
        dd::GfskDemod::Stats st{};
        const auto pk = run_ble(f.ble, iq, &st);
        size_t odid = 0, dji = 0;
        for (const auto& p : pk) {
            odid += is_odid(p);
            uint8_t a[6];
            for (size_t i = 0; i < 6; i++) a[i] = p.pdu[7 - i];
            const uint8_t* name;
            size_t nl;
            dji += dd::dji_ble_match(p.pdu[0], a, p.pdu + 8, p.length - 6, &name, &nl) != 0;
        }
        MESSAGE(f.stem << " ch" << int(f.ble) << ": " << pk.size() << " CRC-valid adverts from third parties, " << odid
                       << " Remote ID, " << dji << " DJI matches, " << st.crc_bad << " CRC fails");
        CHECK(odid == 0);
        CHECK(dji == 0);
    }
    if (!any) MESSAGE("skipped: no control recordings in DD_IQ_DIR");
}

TEST_CASE("recording: real DJI Mini 4 Pro BLE advert (RFUAV via sdrmm fixture)") {
    const std::string dir = env_dir("DD_DJI_DIR");
    const std::string stem = dir + "/remote_id_ble_dji_mini4_4m";
    if (dir.empty() || !file_exists(stem + ".sigmf-data") || !file_exists(stem + ".sigmf-meta")) {
        MESSAGE("skipped: DJI fixture not in DD_DJI_DIR");
        return;
    }
    const std::string meta = slurp(stem + ".sigmf-meta");
    const double fs = json_number(meta, "core:sample_rate"), fc = json_number(meta, "core:frequency");
    const std::string raw = slurp(stem + ".sigmf-data");
    std::vector<cplxf> x(raw.size() / 8);
    float peak = 0;
    for (size_t i = 0; i < x.size(); i++) {
        float re, im;
        memcpy(&re, raw.data() + 8 * i, 4);
        memcpy(&im, raw.data() + 8 * i + 4, 4);
        x[i] = cplxf(re, im);
        peak = std::max(peak, std::max(std::abs(re), std::abs(im)));
    }
    REQUIRE(peak > 0);
    REQUIRE(fs == kFs);
    std::string report;
    size_t dji_hits = 0;
    /* Peak at 3/4 of full scale, and 12 dB lower (an 8 bit receiver at a
     * lower gain). */
    for (double level : {96.0, 24.0}) {
        const auto iq = to_dwell(x, fs, fc, 2426e6, level / peak);
        const auto pk = run_ble(38, iq);
        report += "\n  level " + std::to_string(static_cast<int>(level)) + "/128 peak: " + std::to_string(pk.size()) + " CRC-valid adverts";
        for (const auto& p : pk) {
            uint8_t a[6];
            for (size_t i = 0; i < 6; i++) a[i] = p.pdu[7 - i];
            const uint8_t* name;
            size_t nl;
            const uint8_t why = dd::dji_ble_match(p.pdu[0], a, p.pdu + 8, p.length - 6, &name, &nl);
            char line[160];
            snprintf(line, sizeof(line), "\n    t=%.3f ms %-15s %s %s rssi %.1f dBFS cfo %d kHz", p.end_sample / 4000.0, kPduNames[p.pdu_type()],
                     addr_of(p).c_str(), (p.pdu[0] & 0x40) ? "random" : "public", p.rssi_cdb / 100.0, p.cfo_khz);
            report += line;
            if (why) {
                report += std::string(" DJI[") + ((why & dd::VENDOR_OUI) ? "OUI " : "") + ((why & dd::VENDOR_NAME) ? "NAME " : "") +
                          ((why & dd::VENDOR_CID) ? "CID" : "") + "]";
                if (level == 96.0) dji_hits++;
            }
            /* AD structures. */
            const uint8_t* adv = p.pdu + 8;
            const size_t len = p.length - 6;
            const uint8_t t = p.pdu_type();
            if (t == dd::ADV_IND || t == dd::ADV_NONCONN_IND || t == dd::ADV_SCAN_IND || t == dd::SCAN_RSP) {
                for (size_t i = 0; i + 1 < len;) {
                    const size_t l = adv[i];
                    if (l == 0 || i + 1 + l > len) break;
                    report += "\n        AD 0x" + hex(adv + i + 1, 1) + " [" + hex(adv + i + 2, l - 1, " ") + "]";
                    if (adv[i + 1] == 0x08 || adv[i + 1] == 0x09) report += " \"" + std::string(reinterpret_cast<const char*>(adv + i + 2), l - 1) + "\"";
                    i += 1 + l;
                }
            } else {
                report += " payload " + hex(p.pdu + 2, p.length);
            }
        }
    }
    MESSAGE("DJI Mini 4 Pro fixture (" << x.size() / fs * 1000 << " ms at " << fs / 1e6 << " MS/s, centre " << fc / 1e6 << " MHz):" << report);
    CHECK(dji_hits >= 1);
}
