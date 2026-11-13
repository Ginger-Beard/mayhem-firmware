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

/* Host tests of the DJI DroneID burst detector (baseband/sigdet_droneid.hpp,
 * PSDT MODE_DRONEID): the firmware header itself, fed 2048 sample buffers
 * the way the processor feeds it (the previous buffer read in place).
 *
 * Synthetic tests always run. The "recording:" tests read real recordings
 * that stay outside the repository and say "skipped" without them:
 *   SIGDET_DRONEID_DIR  the PC prototype's test set (mayhem-build/proto/droneid/
 *                   data): recorded real DJI bursts (O1 / O2 / O4) at
 *                   15.36 Msps as HackRF int8, DJI downlinks / uplinks /
 *                   enhanced Wi-Fi around them, Monte-Carlo streams;
 *   SIGDET_IQ_DIR       the dronedetect-bench set (household 2.4 GHz, test
 *                   transmitter, RC toy, 5.8 GHz OFDM), 20 Msps, taken
 *                   through a model of the HackRF Pro's 15.36 Msps path;
 *   SIGDET_SESSION_DIR  15.36 Msps captures at the three 2.4 GHz DroneID
 *                   centres with no drone present (fieldcap dji session).
 * With SIGDET_PROTO_DRONEID_HPP (the prototype's cpp/sigdet_droneid.hpp) defined,
 * every recording also goes through the prototype kernel and the two must
 * agree bit for bit (candidates, power, floor, CP coherence, ZC score, lags,
 * layout, decision).
 *
 * PRIVACY: the recordings hold third-party traffic. The detector only
 * measures energy, CP coherence and correlation with the DroneID reference;
 * the tests print counts, times and scores, nothing else. */

#include "sigdet_droneid.hpp"
#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifdef SIGDET_PROTO_DRONEID_HPP
#include SIGDET_PROTO_DRONEID_HPP
#ifdef SIGDET_PROTO_TABLES_HPP
#include SIGDET_PROTO_TABLES_HPP
#endif
#endif

namespace {

namespace did = sigdet::droneid;
using cplx = std::complex<double>;
constexpr double kPi = 3.141592653589793238463;

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

int8_t sat8(double v) {
    const double r = std::nearbyint(v);
    return static_cast<int8_t>(r > 127 ? 127 : (r < -128 ? -128 : r));
}

/* One detection or candidate as the processor reports it. */
struct Event {
    uint32_t start, end, pw_q4, floor_q4, snap_start;
    did::Result r;
};

/* The processor's use of the kernel: buffers in order, the previous one in
 * place, stage B on every candidate. busy_buffers > 0: the snapshot stays
 * frozen that many buffers after a candidate (stage B time in the event
 * thread), as on the M4. */
struct Run {
    std::vector<Event> events;
    uint32_t candidates = 0, busy_skips = 0;
};

Run run_firmware(const std::vector<int8_t>& iq, uint32_t busy_buffers = 0) {
    auto det = std::make_unique<did::Detector>();
    det->gate.reset();
    Run out;
    const size_t nbuf = iq.size() / (2 * did::kBuffer);
    /* A 4 transfer DMA ring, as on the M4: the previous buffer is read in place. */
    std::vector<int8_t> ring(4 * 2 * did::kBuffer);
    const int8_t* prev = nullptr;
    uint32_t release_in = 0;
    for (size_t b = 0; b < nbuf; b++) {
        int8_t* slot = ring.data() + (b % 4) * 2 * did::kBuffer;
        std::copy(iq.begin() + b * 2 * did::kBuffer, iq.begin() + (b + 1) * 2 * did::kBuffer, slot);
        if (release_in && --release_in == 0) det->gate.release();
        if (det->gate.feed(slot, prev)) {
            const auto& c = det->gate.candidate();
            const uint32_t a = det->gate.snapshot_start();
            Event e{c.start, c.end, c.mean_pw_q4, c.floor_q4, a, {}};
            e.r = det->stage_b.evaluate(det->gate.snapshot(), c.end - a, c.start - a, c.end - a);
            out.events.push_back(e);
            if (busy_buffers)
                release_in = busy_buffers;
            else
                det->gate.release();
        }
        prev = slot;
    }
    out.candidates = det->gate.candidates();
    out.busy_skips = det->gate.busy_skips();
    return out;
}

#ifdef SIGDET_PROTO_DRONEID_HPP
/* The prototype kernel exactly as its host replay (test_droneid.cpp) runs it
 * at 15.36 Msps with the recommended parameters. */
std::vector<Event> run_prototype(const std::vector<int8_t>& iq) {
    ddid::GateConfig gc;
    gc.fs = 15360000;
    gc.k = 16;
    gc.box2 = true;
    gc.on_q8 = 362;
    gc.off_q8 = 308;
    gc.dur_lo = static_cast<uint32_t>(std::ceil(535e-6 * gc.fs - 1e-9));
    gc.dur_hi = static_cast<uint32_t>(std::floor(720e-6 * gc.fs + 1e-9));
    gc.pre = static_cast<uint32_t>(40e-6 * gc.fs);
    ddid::StageBConfig bc;
    bc.fs = 15360000;
    bc.lag = 1024;
    bc.win = 64;
    bc.tnum = 1096;
    bc.tden = 1;
    bc.layout_samples = static_cast<uint32_t>(std::floor(620e-6 * bc.fs + 1e-9));
    bc.span = 12;
    auto gate = std::make_unique<ddid::Gate>();
    auto sb = std::make_unique<ddid::StageB>();
    gate->configure(gc);
    sb->configure(bc, ddid::kZc600_15M36, ddid::kZc600_15M36_len);
    std::vector<Event> out;
    const size_t nbuf = iq.size() / (2 * ddid::kBuffer);
    for (size_t b = 0; b < nbuf; b++) {
        if (gate->feed(iq.data() + b * 2 * ddid::kBuffer)) {
            const auto& c = gate->candidate();
            const uint32_t a = gate->snapshot_start();
            const ddid::Result pr = sb->evaluate(gate->snapshot(), c.end - a, c.start - a, c.end - a);
            Event e{c.start, c.end, c.mean_pw_q4, c.floor_q4, a, {}};
            e.r.cp = pr.cp;
            e.r.zc = pr.zc;
            e.r.zc_lag = pr.zc_lag;
            e.r.grid_m = pr.grid_m;
            e.r.uz = pr.uz;
            e.r.nsym = pr.nsym;
            e.r.det = pr.det;
            e.r.cfo_frac_hz = pr.cfo_frac_hz;
            out.push_back(e);
            gate->release();
        }
    }
    return out;
}

/* Bit for bit: same candidates and the same stage B output. The float scores
 * of both are the same expressions on the same integers; the host compiler
 * does not contract them. */
void check_same(const std::string& name, const std::vector<Event>& fw, const std::vector<Event>& pr) {
    INFO(name);
    REQUIRE(fw.size() == pr.size());
    for (size_t i = 0; i < fw.size(); i++) {
        const Event &a = fw[i], &b = pr[i];
        CHECK(a.start == b.start);
        CHECK(a.end == b.end);
        CHECK(a.pw_q4 == b.pw_q4);
        CHECK(a.floor_q4 == b.floor_q4);
        CHECK(a.snap_start == b.snap_start);
        CHECK(a.r.cp == b.r.cp);
        CHECK(a.r.zc == b.r.zc);
        CHECK(a.r.zc_lag == b.r.zc_lag);
        CHECK(a.r.grid_m == b.r.grid_m);
        CHECK(a.r.uz == b.r.uz);
        CHECK(a.r.nsym == b.r.nsym);
        CHECK(a.r.det == b.r.det);
        CHECK(a.r.cfo_frac_hz == b.r.cfo_frac_hz);
    }
}
#endif

size_t detections(const Run& r) {
    size_t n = 0;
    for (const auto& e : r.events) n += e.r.det;
    return n;
}

float max_zc_negative(const Run& r) {
    float m = 0;
    for (const auto& e : r.events)
        if (!e.r.det) m = std::max(m, e.r.zc);
    return m;
}

/* ------------------------------------------------------------ synthetic */

/* One OFDM-like burst at 15.36 Msps: nsym symbols of 1096 samples (CP 80
 * on the first, 72 on the others, cyclic), the ZC root 600 reference in
 * symbol 2 (8 symbols) or 3 (9), the others random band-limited noise
 * symbols (CP structure only). amp: rms per component. */
std::vector<cplx> burst(Rng& rng, int nsym, double amp, double cfo_hz = 0.0) {
    std::vector<cplx> out;
    const int zi = nsym == 9 ? 3 : 2;
    for (int s = 0; s < nsym; s++) {
        std::vector<cplx> u(1024);
        if (s == zi) {
            for (size_t t = 0; t < 1024; t++) u[t] = cplx(did::kZc600[2 * t], did::kZc600[2 * t + 1]) / 64.0;
        } else {
            /* white noise smoothed by a 3 tap filter: roughly the 9 MHz of a real symbol */
            std::vector<cplx> w(1026);
            for (auto& v : w) v = cplx(rng.normal(1), rng.normal(1));
            for (size_t t = 0; t < 1024; t++) u[t] = (w[t] + w[t + 1] + w[t + 2]) / std::sqrt(3.0);
        }
        const size_t cp = s == 0 ? 80 : 72;
        for (size_t t = 1024 - cp; t < 1024; t++) out.push_back(u[t]);
        for (size_t t = 0; t < 1024; t++) out.push_back(u[t]);
    }
    for (size_t n = 0; n < out.size(); n++) out[n] *= amp * std::polar(1.0, 2 * kPi * cfo_hz * n / did::kSampleRate);
    return out;
}

/* Noise (sigma per component, LSB) with bursts added at the given sample
 * positions, rounded to int8. */
std::vector<int8_t> stream(Rng& rng, size_t n, double sigma, const std::vector<std::pair<size_t, std::vector<cplx>>>& bursts) {
    std::vector<cplx> x(n);
    for (auto& v : x) v = cplx(rng.normal(sigma), rng.normal(sigma));
    for (const auto& b : bursts)
        for (size_t i = 0; i < b.second.size() && b.first + i < n; i++) x[b.first + i] += b.second[i];
    std::vector<int8_t> out(2 * n);
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = sat8(x[i].real());
        out[2 * i + 1] = sat8(x[i].imag());
    }
    return out;
}

}  // namespace

TEST_CASE("droneid: reference table: 1024 samples, peak 127, the energy stage B normalises with") {
    uint32_t e = 0;
    int peak = 0, nonzero_tail = 0;
    for (size_t i = 0; i < 2 * did::kRefLen; i++) {
        e += static_cast<uint32_t>(did::kZc600[i] * did::kZc600[i]);
        peak = std::max(peak, std::abs(static_cast<int>(did::kZc600[i])));
    }
    for (size_t i = 2 * did::kRefLen - 48; i < 2 * did::kRefLen; i++) nonzero_tail += did::kZc600[i] != 0;
    CHECK(peak == 127);
    CHECK(nonzero_tail > 40); /* the whole array was filled in, not zero padded */
    did::StageB sb;
    CHECK(sb.ref_energy() == e);
#if defined(SIGDET_PROTO_DRONEID_HPP) && defined(SIGDET_PROTO_TABLES_HPP)
    REQUIRE(ddid::kZc600_15M36_len == did::kRefLen);
    for (size_t i = 0; i < 2 * did::kRefLen; i++) CHECK(ddid::kZc600_15M36[i] == did::kZc600[i]);
#endif
}

TEST_CASE("droneid: SIMD helpers (host equivalents) equal plain integer arithmetic") {
    Rng rng(7);
    alignas(4) int8_t buf[2 * 1100];
    for (auto& v : buf) v = static_cast<int8_t>(rng() & 0xFF);
    /* box2 block power */
    for (size_t blk = 0; blk < 4; blk++) {
        const int8_t* p = buf + blk * 64;
        uint32_t want = 0;
        for (size_t m = 0; m < 16; m++) {
            const int32_t i = p[4 * m] + p[4 * m + 2], q = p[4 * m + 1] + p[4 * m + 3];
            want += static_cast<uint32_t>(i * i + q * q);
        }
        CHECK(did::block_power_box2(reinterpret_cast<const uint32_t*>(p)) == want);
    }
    /* correlation at even and odd sample offsets (2 and 4 byte alignment) */
    did::StageB sb;
    for (size_t off : {0u, 1u, 7u, 30u}) {
        const int8_t* x = buf + 2 * off;
        int32_t a = 0, b = 0;
        uint32_t e = 0;
        for (size_t t = 0; t < did::kRefLen; t++) {
            const int32_t xr = x[2 * t], xi = x[2 * t + 1], rr = did::kZc600[2 * t], ri = did::kZc600[2 * t + 1];
            a += xr * rr + xi * ri;
            b += xi * rr - xr * ri;
            e += static_cast<uint32_t>(xr * xr + xi * xi);
        }
        int32_t cr, ci;
        uint32_t ex;
        sb.correlate(x, cr, ci, ex);
        CHECK(cr == a);
        CHECK(ci == b);
        CHECK(ex == e);
    }
    uint32_t pw = 0;
    for (size_t t = 0; t < 64; t++) pw += static_cast<uint32_t>(buf[2 * t + 2] * buf[2 * t + 2] + buf[2 * t + 3] * buf[2 * t + 3]);
    CHECK(did::power_sum(buf + 2, 64) == pw);
}

TEST_CASE("droneid: synthetic 8 and 9 symbol bursts are detected, noise and wrong durations are not") {
    Rng rng(11);
    const size_t n = 40 * did::kBuffer;
    /* ~+6 dB in-band over a 2 LSB floor */
    auto b8 = burst(rng, 8, 4.0), b9 = burst(rng, 9, 4.0), b8c = burst(rng, 8, 4.0, 40e3);
    const auto iq = stream(rng, n, 2.0, {{10000, b8}, {30001, b9}, {52345, b8c}});
    const Run r = run_firmware(iq);
    REQUIRE(r.events.size() == 3);
    CHECK(detections(r) == 3);
    CHECK(r.events[0].r.nsym == 8);
    CHECK(r.events[1].r.nsym == 9);
    for (const auto& e : r.events) MESSAGE("zc " << e.r.zc << " cp " << e.r.cp << " lag " << e.r.zc_lag);

    /* noise only: no candidate at all */
    const auto quiet = stream(rng, n, 2.0, {});
    CHECK(run_firmware(quiet).candidates == 0);

    /* too short (5 symbols, 0.36 ms) and too long (12 symbols, 0.86 ms): no candidate */
    auto b5 = burst(rng, 8, 4.0);
    b5.resize(5 * 1096);
    auto b12 = burst(rng, 8, 4.0), more = burst(rng, 8, 4.0);
    b12.insert(b12.end(), more.begin(), more.begin() + 4 * 1096);
    const auto wrong = stream(rng, n, 2.0, {{10000, b5}, {40000, b12}});
    const Run w = run_firmware(wrong);
    CHECK(w.candidates == 0);
    CHECK(detections(w) == 0);

    /* the right duration without the ZC structure (all symbols random): a
     * candidate, rejected by stage B */
    auto fake = burst(rng, 8, 4.0);
    {
        Rng r2(99);
        for (size_t t = 80 + 2 * 1096; t < 80 + 2 * 1096 + 1024; t++) fake[t] = cplx(r2.normal(4.0), r2.normal(4.0));
    }
    const Run f = run_firmware(stream(rng, n, 2.0, {{20000, fake}}));
    CHECK(f.candidates == 1);
    CHECK(detections(f) == 0);
    if (!f.events.empty()) CHECK(f.events[0].r.zc < 0.03f);
}

TEST_CASE("droneid: a burst at every phase across a buffer boundary; a lost buffer drops a capture safely") {
    Rng rng(5);
    int found = 0, tried = 0;
    for (size_t at = 3 * did::kBuffer * 4; at < 3 * did::kBuffer * 4 + did::kBuffer; at += 97) {
        const auto iq = stream(rng, 32 * did::kBuffer, 2.0, {{at, burst(rng, 8, 4.0)}});
        const Run r = run_firmware(iq);
        tried++;
        found += detections(r) == 1 && r.events.size() == 1;
    }
    CHECK(found == tried);

    /* A discontinuity in the middle of a burst: its capture is dropped (no
     * detection of a burst with a hole), the next burst is detected. */
    const auto iq = stream(rng, 40 * did::kBuffer, 2.0, {{8 * did::kBuffer + 100, burst(rng, 8, 4.0)}, {24 * did::kBuffer, burst(rng, 8, 4.0)}});
    auto det = std::make_unique<did::Detector>();
    det->gate.reset();
    int dets = 0;
    const int8_t* prev = nullptr;
    for (size_t b = 0; b < 40; b++) {
        if (b == 10) {
            det->gate.discontinuity(); /* buffer 10 "lost" while the first burst is on */
            prev = nullptr;
            continue;
        }
        const int8_t* p = iq.data() + b * 2 * did::kBuffer;
        if (det->gate.feed(p, prev)) {
            const auto& c = det->gate.candidate();
            const uint32_t a = det->gate.snapshot_start();
            dets += det->stage_b.evaluate(det->gate.snapshot(), c.end - a, c.start - a, c.end - a).det;
            det->gate.release();
        }
        prev = p;
    }
    CHECK(dets == 1);
}

TEST_CASE("droneid: a frozen snapshot (stage B running) skips later candidates and counts them") {
    Rng rng(3);
    std::vector<std::pair<size_t, std::vector<cplx>>> bs;
    for (int i = 0; i < 4; i++) bs.push_back({8000 + i * 3 * did::kBuffer * 4, burst(rng, 8, 4.0)});
    const auto iq = stream(rng, 64 * did::kBuffer, 2.0, bs);
    const Run all = run_firmware(iq);
    CHECK(detections(all) == 4);
    /* frozen for 30 buffers (4 ms): every second burst (12 buffers apart) is missed */
    const Run busy = run_firmware(iq, 30);
    CHECK(busy.candidates == 4);
    CHECK(busy.busy_skips == busy.candidates - busy.events.size());
    CHECK(busy.events.size() < 4);
    CHECK(detections(busy) == busy.events.size());
}

/* ------------------------------------------------------------ recordings */

namespace {

std::string env_dir(const char* var) {
    const char* v = std::getenv(var);
    return v ? std::string(v) : std::string();
}

bool read_file(const std::string& p, std::string& out) {
    std::ifstream f(p, std::ios::binary);
    if (!f.good()) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

std::vector<int8_t> load_ci8(const std::string& path) {
    std::string raw;
    if (!read_file(path, raw)) return {};
    return std::vector<int8_t>(raw.begin(), raw.end());
}

/* "a": N, "b": M pairs of a Monte-Carlo sidecar's truth list. */
std::vector<std::pair<uint32_t, uint32_t>> truth_of(const std::string& json) {
    std::vector<std::pair<uint32_t, uint32_t>> t;
    size_t at = json.find("\"truth\"");
    while (at != std::string::npos) {
        const size_t a = json.find("\"a\":", at);
        if (a == std::string::npos) break;
        const size_t b = json.find("\"b\":", a);
        if (b == std::string::npos) break;
        t.push_back({static_cast<uint32_t>(std::strtoul(json.c_str() + a + 4, nullptr, 10)),
                     static_cast<uint32_t>(std::strtoul(json.c_str() + b + 4, nullptr, 10))});
        at = b;
    }
    return t;
}

/* scipy.signal.resample_poly(x, up, down) (Kaiser 5.0, half length 10 x
 * max(up, down)), complex. */
std::vector<cplx> resample_poly(const std::vector<cplx>& x, uint64_t up, uint64_t down) {
    auto i0 = [](double v) {
        double sum = 1, term = 1;
        for (int k = 1; k < 60; k++) {
            term *= (v / (2 * k)) * (v / (2 * k));
            sum += term;
            if (term < 1e-17 * sum) break;
        }
        return sum;
    };
    const uint64_t m = std::max(up, down), half = 10 * m, nt = 2 * half + 1;
    std::vector<double> h(nt);
    const double fc = 1.0 / m, alpha = (nt - 1) / 2.0;
    double sum = 0;
    for (size_t i = 0; i < nt; i++) {
        const double t = fc * (i - alpha);
        const double sinc = t == 0 ? 1.0 : std::sin(kPi * t) / (kPi * t);
        const double r = 2.0 * i / (nt - 1) - 1;
        h[i] = fc * sinc * i0(5.0 * std::sqrt(std::max(0.0, 1 - r * r))) / i0(5.0);
        sum += h[i];
    }
    for (auto& v : h) v = v / sum * static_cast<double>(up);
    const size_t n_out = (x.size() * up + down - 1) / down;
    std::vector<cplx> y(n_out);
    for (size_t k = 0; k < n_out; k++) {
        const long base = static_cast<long>(k * down + half);
        long lo = base - static_cast<long>(2 * half);
        lo = lo < 0 ? 0 : (lo + static_cast<long>(up) - 1) / static_cast<long>(up);
        long hi = base / static_cast<long>(up);
        if (hi >= static_cast<long>(x.size())) hi = static_cast<long>(x.size()) - 1;
        cplx acc{};
        for (long n = lo; n <= hi; n++) acc += x[n] * h[base - n * static_cast<long>(up)];
        y[k] = acc;
    }
    return y;
}

/* A 20 Msps recording as the HackRF Pro would deliver it at 15.36 Msps:
 * ADC at 30.72 Msps (resampled 192/125), then the gateware's half band
 * decimator hbfir1 (fpga/top/standard.py), rounded to int8. */
std::vector<int8_t> pro_15m36_from_20m(const std::vector<int8_t>& raw) {
    std::vector<cplx> x(raw.size() / 2);
    for (size_t i = 0; i < x.size(); i++) x[i] = cplx(raw[2 * i], raw[2 * i + 1]);
    const std::vector<cplx> y30 = resample_poly(x, 192, 125);
    static constexpr double hb1[23] = {-2, 0, 7, 0, -18, 0, 41, 0, -92, 0, 320, 512, 320, 0, -92, 0, 41, 0, -18, 0, 7, 0, -2};
    const size_t n15 = y30.size() / 2;
    std::vector<int8_t> out(2 * n15);
    for (size_t k = 0; k < n15; k++) {
        cplx acc{};
        for (size_t j = 0; j < 23; j++) {
            const long q = static_cast<long>(2 * k) + 11 - static_cast<long>(j);
            if (q >= 0 && static_cast<size_t>(q) < y30.size()) acc += hb1[j] / 1024.0 * y30[q];
        }
        out[2 * k] = sat8(acc.real());
        out[2 * k + 1] = sat8(acc.imag());
    }
    return out;
}

struct FileExpect {
    const char* stem;
    int cands; /* -1: not checked */
    int dets;
};

/* Expected at 15.36 Msps (the prototype's own results/compare.txt; the real
 * bursts in them were matched to ground truth there). */
const FileExpect kProtoFiles[] = {
    {"pos_frb_air3s_0245_ch+0.0MHz_15M36", 0, 0},      /* 0.75 ms file: no floor before the burst */
    {"pos_frb_air3s_0309_ch+0.1MHz_15M36", 1, 1},      /* Air 3S (O4) */
    {"pos_rub_mavicair2_ch-12.5MHz_15M36", 2, 1},      /* Mavic Air 2 (O2) */
    {"pos_rub_mavicair2_ch+2.0MHz_15M36", 0, 0},       /* too short at 15.36 */
    {"pos_rub_mini2_ch+9.6MHz_15M36", 10, 10},         /* Mini 2 (O2) */
    {"neg_tam_mavicpro_o1edge_at_2444.5_15M36", 1, 1}, /* Mavic Pro (O1): one real DroneID among video / uplink */
    {"rfuav_m4p_vbw10_at_2444.5_15M36", 215, 2},       /* Mini 4 Pro (O4) DroneID in its 10 MHz video + Wi-Fi */
    {"rfuav_m4p_vbw20_at_2444.5_15M36", 65, 1},        /* ... 20 MHz video */
    {"neg_tam_mavicmini_enhwifi_2412_15M36", 45, 0},   /* DJI enhanced Wi-Fi */
    {"neg_tam_mavicmini_enhwifi_at_2414.5_15M36", 45, 0},
    {"neg_tam_mavicpro_o1uplink_at_2414.5_15M36", 20, 0}, /* O1 uplink */
    {"neg_tam_mavicpro_o1video_2454_15M36", 1, 0},        /* O1 video */
    {"mc_awgn_15M36", 80, 80},                            /* 80 real bursts, -3..+15 dB, +-50 kHz, AWGN */
    {"mc_clutter_15M36", 39, 37},                         /* 40 real bursts injected into ambient 2430 MHz */
};

}  // namespace

TEST_CASE("recording: droneid prototype test set: real DJI bursts detected, DJI downlinks / enhanced Wi-Fi rejected") {
    const std::string dir = env_dir("SIGDET_DRONEID_DIR");
    std::string probe;
    if (dir.empty() || !read_file(dir + "/mc_awgn_15M36.json", probe)) {
        MESSAGE("skipped: SIGDET_DRONEID_DIR (proto/droneid/data) not available");
        return;
    }
    int total_dets = 0, real_dets = 0;
    float worst_negative = 0, weakest_positive = 1;
    for (const auto& fe : kProtoFiles) {
        const auto iq = load_ci8(dir + "/" + fe.stem + ".ci8");
        if (iq.empty()) {
            MESSAGE("missing " << std::string(fe.stem));
            continue;
        }
        const Run r = run_firmware(iq);
        const size_t d = detections(r);
        MESSAGE(std::string(fe.stem) << ": " << r.events.size() << " candidates, " << d << " detected, max ZC of the rejected " << max_zc_negative(r));
        CHECK_MESSAGE(static_cast<int>(r.events.size()) == fe.cands, fe.stem);
        CHECK_MESSAGE(static_cast<int>(d) == fe.dets, fe.stem);
        worst_negative = std::max(worst_negative, max_zc_negative(r));
        for (const auto& e : r.events)
            if (e.r.det) weakest_positive = std::min(weakest_positive, e.r.zc);
        total_dets += static_cast<int>(d);
        if (std::string(fe.stem).rfind("mc_", 0) != 0) real_dets += static_cast<int>(d);
        /* Monte-Carlo streams: every detection lies on an injected burst */
        std::string js;
        if (std::string(fe.stem).rfind("mc_", 0) == 0 && read_file(dir + "/" + fe.stem + ".json", js)) {
            const auto truth = truth_of(js);
            for (const auto& e : r.events) {
                if (!e.r.det) continue;
                bool hit = false;
                for (const auto& t : truth) hit = hit || (e.start < t.second && e.end > t.first);
                CHECK_MESSAGE(hit, fe.stem << " detection at " << e.start << " on no injected burst");
            }
        }
#ifdef SIGDET_PROTO_DRONEID_HPP
        check_same(fe.stem, r.events, run_prototype(iq));
#endif
    }
    MESSAGE("real bursts detected: " << real_dets << ", with Monte-Carlo " << total_dets << "; weakest detection ZC " << weakest_positive
                                     << ", largest rejected ZC " << worst_negative << " (threshold " << did::kZcMin << ")");
    CHECK(real_dets == 16);
    CHECK(worst_negative < 0.03f);
}

TEST_CASE("recording: droneid ambient at the three DroneID centres, 15.36 Msps, no drone: no detection") {
    const std::string dir = env_dir("SIGDET_SESSION_DIR");
    const char* const stems[] = {"02_droneid24_2414.5MHz", "03_droneid24_2429.5MHz", "04_droneid24_2444.5MHz"};
    int files = 0;
    for (const char* s : stems) {
        const auto iq = dir.empty() ? std::vector<int8_t>{} : load_ci8(dir + "/" + s + ".ci8");
        if (iq.empty()) continue;
        files++;
        const Run r = run_firmware(iq);
        const double secs = iq.size() / 2.0 / did::kSampleRate;
        MESSAGE(std::string(s) << ": " << secs << " s, " << r.events.size() << " candidates (" << r.events.size() / secs << "/s), "
                               << detections(r) << " detected, max ZC " << max_zc_negative(r));
        CHECK(detections(r) == 0);
        CHECK(max_zc_negative(r) < 0.03f);
        /* the snapshot busy for stage B (~5 ms on the M4) changes nothing here */
        CHECK(detections(run_firmware(iq, 40)) == 0);
#ifdef SIGDET_PROTO_DRONEID_HPP
        check_same(s, r.events, run_prototype(iq));
#endif
    }
    if (!files) MESSAGE("skipped: no 15.36 Msps DroneID-centre session in SIGDET_SESSION_DIR");
}

TEST_CASE("recording: droneid bench recordings through the Pro's 15.36 Msps path (household 2.4 GHz, C5, toy, 5.8 GHz OFDM): no detection") {
    const std::string dir = env_dir("SIGDET_IQ_DIR");
    const char* const stems[] = {
        "ambient_c5silent_2412MHz_20Msps", "ambient_c5silent_2430MHz_20Msps", "ambient_c5silent_2448MHz_20Msps",
        "ambient_c5silent_2466MHz_20Msps", "toy_SLU23T24T23_c5silent_2412MHz_20Msps", "toy_SLU23T24T23_c5silent_2430MHz_20Msps",
        "toy_SLU23T24T23_c5silent_2448MHz_20Msps", "toy_SLU23T24T23_c5silent_2466MHz_20Msps", "c5_wifi_beacon_ch6_1Mbps_2437MHz_20Msps",
        "c5silent_ch6_control_2437MHz_20Msps", "c5_wifi_beacon_ch6_6Mbps_2437MHz_20Msps", "c5_wifi_nan_ch6_1Mbps_2437MHz_20Msps",
        "c5_wifi_frenchbeacon_ch6_1Mbps_2437MHz_20Msps", "c5_wifi_djibeacon_ch6_1Mbps_2437MHz_20Msps", "c5_ble5_coded_2412MHz_20Msps",
        "c5_ble5_coded_2412MHz_20Msps_lowgain", "c5_ble5_coded_2440MHz_20Msps", "c5_ble5_coded_2440MHz_20Msps_lowgain",
        "c5_ble5_coded_ch37in_2408MHz_20Msps", "c5silent_ble5_control_2408MHz_20Msps", "c5_hop154_4ms_ch11-26_2440MHz_20Msps",
        "c5_wifi_beacon_ch149_6Mbps_5745MHz_20Msps", "c5_wifi_blast_ch149_duty100_5745MHz_20Msps", "c5_wifi_blast_ch149_duty50_5745MHz_20Msps"};
    int files = 0;
    double secs_total = 0;
    size_t cands_total = 0;
    float worst = 0;
    for (const char* s : stems) {
        const auto raw = dir.empty() ? std::vector<int8_t>{} : load_ci8(dir + "/" + s + ".ci8");
        if (raw.empty()) continue;
        files++;
        const auto iq = pro_15m36_from_20m(raw);
        const Run r = run_firmware(iq);
        const double secs = iq.size() / 2.0 / did::kSampleRate;
        secs_total += secs;
        cands_total += r.events.size();
        worst = std::max(worst, max_zc_negative(r));
        MESSAGE(std::string(s) << ": " << secs << " s, " << r.events.size() << " candidates, " << detections(r) << " detected, max ZC " << max_zc_negative(r));
        CHECK(detections(r) == 0);
#ifdef SIGDET_PROTO_DRONEID_HPP
        check_same(s, r.events, run_prototype(iq));
#endif
    }
    if (!files) {
        MESSAGE("skipped: no 20 Msps bench recordings in SIGDET_IQ_DIR");
        return;
    }
    MESSAGE(files << " files, " << secs_total << " s, " << cands_total << " candidates, 0 detections allowed; largest ZC " << worst);
    CHECK(worst < 0.03f);
}
