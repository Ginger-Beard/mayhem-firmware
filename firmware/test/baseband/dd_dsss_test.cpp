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

/* Host tests of the drone detect Wi-Fi Remote ID receiver: the 802.11b
 * 1 Mb/s kernel (baseband/dd_dsss.hpp), the frame classifier and privacy
 * gate (common/dd_wifi_rid.hpp) and the per-buffer work bound of the PDRN
 * DSSS mode, on a synthetic transmitter and on the real recordings.
 *
 * Real recordings stay outside the repository (DD_IQ_DIR, the
 * dronedetect-bench set); the "recording:" tests say "skipped" and pass
 * without them. They see what the M4 sees: each recording through the
 * PRALINE slice model (dd_dsss_testkit.hpp) at the firmware's tuning, slice
 * -2 and +2 MHz from the Wi-Fi channel centre, fed in 2048 sample buffers.
 *
 * PRIVACY: the recordings hold third-party Wi-Fi. Frames are looked at only
 * through dd_wifi_rid_find() (what the M4 forwards) and the test
 * transmitter's address check (is_ours); everything else is counted. */

#include "dd_dsss.hpp"
#include "dd_wifi_rid.hpp"
#include "dd_dsss_testkit.hpp"
#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace ddtest;

namespace {

using Demod = dd::DsssDemod<dd::kDsssFirmwareSearch>; /* the firmware's kernel */
using Rid = dd::DroneDetectWifiRid;
constexpr size_t kBuffer = 2048;

const uint8_t kOurMac[6] = {0x02, 0x52, 0x49, 0x44, 0x00, 0x01};
const uint8_t kOtherMac[6] = {0x0A, 0x11, 0x22, 0x33, 0x44, 0x55}; /* synthetic "third party" */

Demod& fresh(int slice_mhz, const dd::DsssTrigger& trig = dd::kDsssTriggerFirmware) {
    static Demod d; /* 17.8 KB: static, as on the M4 */
    d.configure(*dd::dsss_taps(slice_mhz), 2, false, true, true, 3, trig);
    d.clear_stats();
    return d;
}

/* What a test keeps of a frame: content of ours only (privacy gate). */
struct Got {
    size_t ours = 0, others = 0;
    std::vector<std::vector<uint8_t>> mpdus; /* ours */
    std::vector<int16_t> rssi;               /* ours */
    std::vector<uint32_t> end;               /* ours */
};

void collect(void* ctx, const dd::DsssFrame& f) {
    Got& g = *static_cast<Got*>(ctx);
    if (is_ours(f.mpdu, f.length)) {
        g.ours++;
        g.mpdus.emplace_back(f.mpdu, f.mpdu + f.length);
        g.rssi.push_back(f.rssi_cdb);
        g.end.push_back(f.end_sample);
    } else {
        g.others++;
    }
}

Got run_buffers(Demod& d, const std::vector<int8_t>& iq, size_t chunk = kBuffer) {
    Got g;
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at < n; at += chunk) d.execute(iq.data() + 2 * at, std::min(chunk, n - at), collect, &g);
    return g;
}

Got run_random_chunks(Demod& d, const std::vector<int8_t>& iq, uint64_t seed) {
    Got g;
    Rng rng(seed);
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at < n;) {
        const size_t c = std::min<size_t>(1 + rng() % 3000, n - at);
        d.execute(iq.data() + 2 * at, c, collect, &g);
        at += c;
    }
    return g;
}

std::vector<uint8_t> our_beacon(uint8_t counter, int32_t lat = 5000, int32_t lon = 5000) {
    return beacon(kOurMac, "RID-TEST-0001", odid_pack({odid_basic_id("TEST-RID-0001"), odid_location(lat, lon)}), counter);
}

/* A stream of frames at 4 Msps (complex, full scale 1.0), then noise of
 * sigma LSB per component, then int8. */
struct Stream {
    std::vector<cplx> y;
    explicit Stream(size_t samples)
        : y(samples) {}
    /* amp from Eb/N0 for noise sigma (LSB): P_chip = Eb/N0 * N0 * 1 MHz, N0 = 2 sigma^2 / 4 MHz (full scale 128). */
    static double amp_for(double ebn0_db, double sigma_lsb) {
        const double n0 = 2 * (sigma_lsb / 128) * (sigma_lsb / 128) / 4e6;
        return std::sqrt(std::pow(10.0, ebn0_db / 10) * n0 * 1e6);
    }
    size_t add(size_t at, const std::vector<cplx>& f) {
        for (size_t i = 0; i < f.size() && at + i < y.size(); i++) y[at + i] += f[i];
        return f.size();
    }
    std::vector<int8_t> quantise(double sigma_lsb, uint64_t seed) const {
        Rng rng(seed);
        std::vector<cplx> z(y);
        for (auto& v : z) v += cplx(rng.normal(sigma_lsb / 128), rng.normal(sigma_lsb / 128));
        return SliceModel::to_int8(z);
    }
};

}  // namespace

/* ---------------------------------------------------------------- basics */

TEST_CASE("dsss: CRC-32 of data + FCS leaves the residue; PLCP CRC-16 matches a bit-serial reference") {
    std::vector<uint8_t> m = {0x80, 0x00, 0x12, 0x34, 0x56};
    append_fcs(m);
    uint32_t c = 0xFFFFFFFFu;
    for (uint8_t b : m) c = dd::crc32_byte(c, b);
    CHECK(c == dd::kCrc32Residue);
    /* Reference: x^16 + x^12 + x^5 + 1 over 32 bits LSB first, preset ones, complemented, sent b15 first. */
    for (uint32_t h : {0x00000A00u | (1544u << 16), 0x0000140Au, 0xFFFFFFFFu, 0x12345678u}) {
        uint32_t reg = 0xFFFF;
        for (int i = 0; i < 32; i++) {
            const uint32_t in = (h >> i) & 1;
            const uint32_t msb = (reg >> 15) & 1;
            reg = (reg << 1) & 0xFFFF;
            if (in ^ msb) reg ^= 0x1021;
        }
        reg = ~reg & 0xFFFF;
        uint32_t air = 0;
        for (int i = 0; i < 16; i++) air |= ((reg >> (15 - i)) & 1) << i;
        CHECK(dd::plcp_crc16(h) == air);
    }
}

TEST_CASE("dsss: tap tables: +s is the conjugate of -s, 0 MHz is real, |h| <= 16") {
    for (int s = 1; s <= 2; s++)
        for (int f = 0; f < 2; f++)
            for (int k = 0; k < 4; k++) {
                CHECK(dd::dsss_taps(s)->h[f][k][0] == dd::dsss_taps(-s)->h[f][k][0]);
                CHECK(dd::dsss_taps(s)->h[f][k][1] == -dd::dsss_taps(-s)->h[f][k][1]);
            }
    for (int s = -2; s <= 2; s++)
        for (int f = 0; f < 2; f++)
            for (int k = 0; k < 4; k++) {
                CHECK(std::abs(dd::dsss_taps(s)->h[f][k][0]) <= 16);
                CHECK(std::abs(dd::dsss_taps(s)->h[f][k][1]) <= 16);
                if (s == 0) CHECK(dd::dsss_taps(s)->h[f][k][1] == 0);
            }
    CHECK(dd::dsss_taps(3) == nullptr);
    CHECK(dd::dsss_taps(-3) == nullptr);
}

/* ---------------------------------------------------------------- classifier */

TEST_CASE("dsss classifier: ODID vendor IE (ASTM and Parrot OUI), NAN service discovery, probe response") {
    const auto pack = odid_pack({odid_basic_id("TEST-RID-0001"), odid_location(5000, 5000)});
    Rid r{};
    SUBCASE("beacon, ASTM OUI") {
        const auto f = beacon(kOurMac, "X", pack, 7);
        REQUIRE(dd::dd_wifi_rid_find(f.data(), f.size(), false, r) == Rid::KIND_ODID_IE);
        CHECK(r.counter == 7);
        CHECK(r.frame_type == 0x80);
        CHECK(std::memcmp(r.addr, kOurMac, 6) == 0);
        CHECK(r.payload_len == 5 + pack.size());
        CHECK(r.payload[0] == 0xFA);
        CHECK(std::memcmp(r.payload + 5, pack.data(), pack.size()) == 0);
    }
    SUBCASE("probe response, Parrot OUI") {
        static constexpr uint8_t parrot[3] = {0x90, 0x3A, 0xE6};
        const auto f = beacon(kOurMac, "X", pack, 9, {}, 0x50, parrot);
        REQUIRE(dd::dd_wifi_rid_find(f.data(), f.size(), false, r) == Rid::KIND_ODID_IE);
        CHECK(r.frame_type == 0x50);
        CHECK(r.counter == 9);
    }
    SUBCASE("NAN") {
        const auto f = nan_sdf(kOurMac, pack, 3);
        REQUIRE(dd::dd_wifi_rid_find(f.data(), f.size(), false, r) == Rid::KIND_NAN);
        CHECK(r.frame_type == 0xD0);
        CHECK(r.counter == 3);
        CHECK(r.payload_len == 1 + pack.size());
        CHECK(r.payload[0] == 3);
        CHECK(std::memcmp(r.payload + 1, pack.data(), pack.size()) == 0);
    }
    SUBCASE("the ODID IE after other vendor IEs") {
        const std::vector<uint8_t> other = {0xDD, 0x07, 0x00, 0x50, 0xF2, 0x02, 0x01, 0x01, 0x00};
        const auto f = beacon(kOurMac, "X", pack, 1, other);
        CHECK(dd::dd_wifi_rid_find(f.data(), f.size(), false, r) == Rid::KIND_ODID_IE);
    }
}

TEST_CASE("dsss classifier: French / legacy DJI IEs only with the bench flag; ODID wins over them") {
    const std::vector<uint8_t> french = {0xDD, 0x08, 0x6A, 0x5C, 0x35, 0x01, 0x01, 0x01, 0x02, 0x03};
    const std::vector<uint8_t> dji = {0xDD, 0x06, 0x26, 0x37, 0x12, 0x58, 0x62, 0x13};
    Rid r{};
    const auto ff = beacon(kOurMac, "X", {}, 0, french);
    const auto fd = beacon(kOurMac, "X", {}, 0, dji);
    CHECK(dd::dd_wifi_rid_find(ff.data(), ff.size(), false, r) == 0);
    CHECK(dd::dd_wifi_rid_find(fd.data(), fd.size(), false, r) == 0);
    CHECK(dd::dd_wifi_rid_find(ff.data(), ff.size(), true, r) == Rid::KIND_FRENCH);
    CHECK(r.payload_len == 8);
    CHECK(dd::dd_wifi_rid_find(fd.data(), fd.size(), true, r) == Rid::KIND_DJI);
    const auto both = beacon(kOurMac, "X", odid_pack({odid_basic_id("A")}), 0, french);
    CHECK(dd::dd_wifi_rid_find(both.data(), both.size(), true, r) == Rid::KIND_ODID_IE);
}

TEST_CASE("dsss classifier (privacy): ordinary frames give nothing and leave the output untouched") {
    Rid r;
    std::memset(&r, 0xA5, sizeof(r));
    const Rid sentinel = r;
    std::vector<std::vector<uint8_t>> frames;
    frames.push_back(beacon(kOtherMac, "HomeNetwork", {}, 0));                                            /* plain beacon */
    frames.push_back(beacon(kOtherMac, "Cafe", {}, 0, {0xDD, 0x05, 0x00, 0x50, 0xF2, 0x04, 0x10}, 0x50)); /* WPS IE */
    {
        auto f = beacon(kOtherMac, "X", odid_pack({odid_basic_id("A")}), 0);
        f[0] = 0x08; /* a data frame carrying the same bytes */
        frames.push_back(f);
    }
    {
        auto f = nan_sdf(kOtherMac, odid_pack({odid_basic_id("A")}), 0);
        f[4] = 0xFF; /* not to the NAN cluster address */
        frames.push_back(f);
    }
    {
        auto f = nan_sdf(kOtherMac, odid_pack({odid_basic_id("A")}), 0);
        f[24 + 6 + 3] ^= 0x01; /* another service ID */
        frames.push_back(f);
    }
    {
        auto f = beacon(kOtherMac, "X", odid_pack({odid_basic_id("A")}), 0);
        f[f.size() - 4 - 25 - 3 - 2] = 0x0C; /* OUI type 0x0C, not 0x0D */
        frames.push_back(f);
    }
    for (const auto& f : frames) CHECK(dd::dd_wifi_rid_find(f.data(), f.size(), true, r) == 0);
    CHECK(std::memcmp(&r, &sentinel, sizeof(r)) == 0);
}

TEST_CASE("dsss classifier: truncated, malformed and random frames are bounds safe and forward nothing") {
    Rng rng(77);
    Rid r{};
    const auto good = beacon(kOurMac, "X", odid_pack({odid_basic_id("A"), odid_location(1, 2)}), 0);
    const auto nan = nan_sdf(kOurMac, odid_pack({odid_basic_id("A")}), 0);
    /* Every truncation: on the heap, exactly sized, so ASan sees an overread. */
    for (const auto* src : {&good, &nan})
        for (size_t n = 0; n <= src->size(); n++) {
            std::vector<uint8_t> f(src->begin(), src->begin() + n);
            dd::dd_wifi_rid_find(f.data(), f.size(), true, r);
        }
    size_t found = 0;
    for (int i = 0; i < 20000; i++) {
        std::vector<uint8_t> f(24 + rng() % 400);
        for (auto& b : f) b = static_cast<uint8_t>(rng());
        f[0] = (i & 1) ? 0x80 : 0xD0;
        if (i & 2) std::memcpy(f.data() + 4, "\x51\x6F\x9A\x01\x00\x00", 6);
        found += dd::dd_wifi_rid_find(f.data(), f.size(), true, r) != 0;
    }
    MESSAGE("random frames classified as Remote ID: " << found << " of 20000");
    CHECK(found == 0);
}

TEST_CASE("dsss: DroneDetectWifiRid wire size: payload last, 18 byte header") {
    Rid r{};
    r.payload_len = 133;
    CHECK(r.wire_size() == 18 + 133);
    CHECK(sizeof(Rid) >= 18 + 255);
}

/* ---------------------------------------------------------------- synthetic */

TEST_CASE("dsss: one beacon decodes at every offset across a 2048 sample buffer boundary, both slices") {
    for (int slice : {-2, 2}) {
        const auto mpdu = our_beacon(42);
        const auto f = frame_in_slice(mpdu, Stream::amp_for(30, 1.5), 0, slice, 0, 0.3);
        size_t ok = 0, tried = 0;
        for (size_t shift = 0; shift < f.size() + 64; shift += 397) {
            Stream s(kBuffer * 2 + f.size() + 1000);
            s.add(kBuffer * 2 - shift % (kBuffer * 2), f);
            Demod& d = fresh(slice);
            const Got g = run_buffers(d, s.quantise(1.5, shift));
            tried++;
            ok += g.ours == 1 && g.mpdus[0] == mpdu;
        }
        MESSAGE("slice " << slice << " MHz: " << ok << "/" << tried);
        CHECK(ok == tried);
    }
}

TEST_CASE("dsss: random chunk sizes give the same frames as 2048 sample buffers") {
    Rng rng(5);
    Stream s(4000000 / 4); /* 250 ms */
    std::vector<std::vector<uint8_t>> sent;
    size_t at = 1000;
    for (int i = 0; at < s.y.size() - 12000; i++) {
        const auto m = our_beacon(static_cast<uint8_t>(i));
        sent.push_back(m);
        at += s.add(at, frame_in_slice(m, Stream::amp_for(25, 1.5), (rng.uniform() - 0.5) * 80e3, -2, rng() % 6, rng.uniform() * 6.28)) +
              200 + rng() % 4000;
    }
    const auto iq = s.quantise(1.5, 9);
    Demod& d = fresh(-2);
    const Got a = run_buffers(d, iq);
    auto rel = [](const std::vector<uint32_t>& e) { /* the static receiver's sample count runs on between tests */
                                                    std::vector<uint32_t> r;
                                                    for (uint32_t v : e) r.push_back(v - e[0]);
                                                    return r;
    };
    for (uint64_t seed : {1, 2, 3}) {
        Demod& e = fresh(-2);
        const Got b = run_random_chunks(e, iq, seed);
        CHECK(b.mpdus == a.mpdus);
        CHECK(rel(b.end) == rel(a.end));
    }
    MESSAGE(sent.size() << " sent, " << a.ours << " decoded");
    CHECK(a.ours == sent.size());
}

TEST_CASE("dsss: carrier offset up to +-100 kHz, both slices, NAN and probe responses") {
    for (int slice : {-2, 2})
        for (double cfo : {-100e3, -50e3, 0.0, 50e3, 100e3}) {
            Stream s(60000);
            const auto b = our_beacon(1);
            const auto n = nan_sdf(kOurMac, odid_pack({odid_basic_id("TEST-RID-0001")}), 2);
            size_t at = s.add(500, frame_in_slice(b, Stream::amp_for(28, 1.5), cfo, slice, 3, 1.0)) + 1500;
            s.add(at, frame_in_slice(n, Stream::amp_for(28, 1.5), cfo, slice, 1, 2.0));
            Demod& d = fresh(slice);
            const Got g = run_buffers(d, s.quantise(1.5, 3));
            CAPTURE(slice);
            CAPTURE(cfo);
            CHECK(g.ours == 2);
        }
}

TEST_CASE("dsss: 4 s of noise: no FCS valid frame; the firmware trigger rule keeps Syncs rare") {
    const size_t n = 16000000;
    std::vector<int8_t> iq(2 * n);
    Rng rng(123);
    std::fesetround(FE_TONEAREST);
    for (auto& v : iq) v = static_cast<int8_t>(std::max(-128.0, std::min(127.0, std::nearbyint(rng.normal(1.5)))));
    Demod& d = fresh(-2);
    const Got g = run_buffers(d, iq);
    const auto fw = d.stats();
    Demod& p = fresh(-2, dd::DsssTrigger{});
    const Got gp = run_buffers(p, iq);
    const auto pr = p.stats();
    MESSAGE("noise 1.5 LSB rms, 4 s: firmware rule " << fw.triggers << " triggers (" << fw.sync_lost << " Sync timeouts), prototype rule "
                                                     << pr.triggers << " triggers; SFDs " << fw.sfd << " / " << pr.sfd << ", FCS valid " << fw.ok
                                                     << " / " << pr.ok);
    CHECK(g.ours + g.others == 0);
    CHECK(gp.ours + gp.others == 0);
    CHECK(fw.triggers * 5 <= pr.triggers + 5);
    CHECK(fw.triggers <= 4 * 10); /* <= 10 a second */
}

TEST_CASE("dsss: the input ring drops a backlog it can't hold, counts it and recovers") {
    Demod& d = fresh(-2);
    std::vector<int8_t> zeros(2 * kBuffer, 0);
    for (int i = 0; i < 3; i++) d.push(zeros.data(), kBuffer); /* 6144 waiting, nothing demodulated */
    CHECK(d.backlog() == 3 * kBuffer);
    CHECK(d.stats().overflows == 0);
    d.push(zeros.data(), kBuffer); /* 8192 > 8192 - 1024 kept for the CFO look back: the backlog goes */
    CHECK(d.stats().overflows == 1);
    CHECK(d.backlog() == kBuffer);
    CHECK(d.stats().dropped == 3 * kBuffer);
    /* And it still decodes. */
    Stream s(30000);
    const auto m = our_beacon(3);
    s.add(1000, frame_in_slice(m, Stream::amp_for(30, 1.5), 0, -2, 0, 0.0));
    Got g;
    const auto iq = s.quantise(1.5, 1);
    d.execute(iq.data(), iq.size() / 2, collect, &g);
    CHECK(g.ours == 1);
}

/* Search on 4 vs 2 sample phases: frames decoded against Eb/N0, every
 * sub-symbol timing (22 offsets of 1/22 us), both slices, random carrier
 * offset +-50 kHz and carrier phase. The firmware's choice rests on this
 * table and on the M4's measured cost of the 4 phase search. */
namespace {
template <int P>
size_t decoded_at(double ebn0, int slice, uint64_t seed, int trig_zeros) {
    static dd::DsssDemod<P> d;
    Rng rng(seed);
    const double sigma = 4.0;
    size_t ok = 0;
    for (size_t frac = 0; frac < 22; frac++) {
        d.configure(*dd::dsss_taps(slice), 2, false, true, true, trig_zeros, dd::kDsssTriggerFirmware);
        Stream s(9000);
        const double cfo = (static_cast<double>(rng() % 1001) - 500.0) * 100.0;
        const double ph = (rng() % 6283) / 1000.0;
        s.add(600, frame_in_slice(our_beacon(static_cast<uint8_t>(frac)), Stream::amp_for(ebn0, sigma), cfo, slice, frac, ph));
        Got g;
        const auto iq = s.quantise(sigma, seed * 131 + frac);
        d.execute(iq.data(), iq.size() / 2, collect, &g);
        ok += g.ours;
    }
    return ok;
}
}  // namespace

TEST_CASE("dsss search phases: sensitivity of the 4 and 2 phase search (synthetic, both slices, all timings)") {
    std::string t = "\nframes decoded of 88 (22 timings x 2 slices x 2 seeds) per Eb/N0: 4 phases | 3 (2 + 2 interpolated, firmware) | 2 phases\n";
    size_t sum4 = 0, sum3 = 0, sum2 = 0;
    for (double e : {12.0, 13.0, 14.0, 15.0, 16.0, 18.0, 20.0, 30.0}) {
        size_t a = 0, b = 0, c = 0;
        for (int slice : {-2, 2})
            for (uint64_t k = 0; k < 2; k++) {
                const uint64_t seed = 1000 + static_cast<uint64_t>(e * 10) + 7 * (slice > 0) + 101 * k;
                a += decoded_at<4>(e, slice, seed, 3);
                b += decoded_at<3>(e, slice, seed, 3);
                c += decoded_at<2>(e, slice, seed, 3);
            }
        sum4 += a;
        sum3 += b;
        sum2 += c;
        char line[96];
        std::snprintf(line, sizeof(line), "Eb/N0 %4.1f dB: %2zu | %2zu | %2zu\n", e, a, b, c);
        t += line;
    }
    MESSAGE(t);
    CHECK(sum3 >= sum2);
    /* Measured: mode 3 ~1 dB behind the 4 phase search at the threshold
     * (Eb/N0 14-16 dB), ~0.5 dB ahead of 2 phases; equal from ~20 dB. */
    CHECK(sum3 * 100 >= sum4 * 85);
}

/* ---------------------------------------------------------------- work bound */

namespace {

/* Cortex-M4 cycles (200 MHz, 50 a sample at 4 Msps) of the kernel by state,
 * MEASURED on the HackRF Pro (DWT, DDSTAT cyc_* fields, 2026-11-11; the
 * instruction-count model used before was 20-40 % low: 4 phase Search 100
 * modelled, 121 measured). A group is 4 samples (one symbol). Search by
 * mode: 4 phases 121, 3 (the firmware's) 107, 2 phases 83. */
constexpr uint32_t kCyclesSearchGroup = 107; /* mode 3 */
constexpr uint32_t kCyclesSearchGroup2 = 83; /* groups shed to 2 phases */
constexpr uint32_t kCyclesSyncGroup = 430;
constexpr uint32_t kCyclesLockGroup = 117;
constexpr uint32_t kCyclesCfoPass = 5300; /* a step of its own, <= 64 symbols */
constexpr uint32_t kCyclesStep = 150;        /* step()/run(): calls, history copy, state switch */
constexpr uint32_t kCyclesFrame = 600;       /* FCS valid frame: copy to the hand-off slot, wake the event thread */
constexpr uint32_t kCyclesBuffer = 2600;     /* execute(): 4 KB into the ring (128 x ldm/stm of 8 words, ~18 cycles each), bookkeeping */
/* proc_drone_detect.hpp: dsss_limit_cycles, dsss_step_bound_cycles. */
constexpr uint32_t kLimit = 350 * 200;
constexpr uint32_t kStepBound = 32 * 200;
constexpr uint32_t kBufferCycles = 512 * 200;

struct Work {
    uint32_t max_buffer = 0, max_step = 0, max_backlog = 0;
    uint64_t total = 0;
    size_t buffers = 0;
    Got got;
    double mean_pct() const { return 100.0 * total / (buffers * static_cast<double>(kBufferCycles)); }
};

/* proc_drone_detect's execute_dsss() with a cost model as its clock. */
Work run_emulated_costs(Demod& d, const std::vector<int8_t>& iq, uint32_t c_search, uint32_t c_sync, uint32_t c_lock, uint32_t c_cfo,
                        uint32_t c_shed = kCyclesSearchGroup2) {
    Work w;
    const size_t n = iq.size() / 2;
    for (size_t at = 0; at + kBuffer <= n; at += kBuffer) {
        uint32_t elapsed = kCyclesBuffer;
        d.push(iq.data() + 2 * at, kBuffer);
        w.max_backlog = std::max<uint32_t>(w.max_backlog, d.backlog());
        Demod::Stats last = d.stats();
        bool first = true;
        d.process(
            [&]() {
                const auto& s = d.stats();
                const uint32_t c = first ? 0
                                         : kCyclesStep + c_search * (s.g_search - last.g_search - (s.g_shed - last.g_shed)) + c_shed * (s.g_shed - last.g_shed) +
                                               c_sync * (s.g_sync - last.g_sync) +
                                               c_lock * (s.g_lock - last.g_lock) + kCyclesFrame * (s.ok - last.ok) +
                                               c_cfo * (s.cfo_passes - last.cfo_passes);
                first = false;
                last = s;
                w.max_step = std::max(w.max_step, c);
                elapsed += c;
                return elapsed;
            },
            kLimit, kStepBound, collect, &w.got);
        w.max_buffer = std::max(w.max_buffer, elapsed);
        w.total += elapsed;
        w.buffers++;
    }
    return w;
}

Work run_emulated(Demod& d, const std::vector<int8_t>& iq) {
    return run_emulated_costs(d, iq, kCyclesSearchGroup, kCyclesSyncGroup, kCyclesLockGroup, kCyclesCfoPass);
}

void report(const std::string& what, const Work& w, const Demod& d) {
    const auto& s = d.stats();
    MESSAGE(what << ": modelled M4 cycles per 2048 sample buffer: mean " << static_cast<uint32_t>(w.total / w.buffers) << " (" << w.mean_pct()
                 << " %), max " << w.max_buffer << " (" << w.max_buffer / 200 << " us of 512), longest step " << w.max_step << " ("
                 << w.max_step / 200 << " us); backlog max " << w.max_backlog << " samples, overflows " << s.overflows << "; triggers "
                 << s.triggers << ", SFDs " << s.sfd << ", Sync timeouts " << s.sync_lost << ", filtered " << s.filtered << ", FCS ok "
                 << s.ok << " (ours " << w.got.ours << ")");
}

}  // namespace

namespace {

/* 1 s of traffic: ODID beacons (ours, 40 %), long third-party beacons (30 %),
 * data frames (20 %, filtered at byte 0), preambles without SFD (10 %);
 * Eb/N0 8..30 dB, +-60 kHz; gaps of gap_min + 0..gap_span samples; noise
 * 1.5 LSB rms. */
struct Traffic {
    std::vector<int8_t> iq;
    size_t sent_ours = 0, frames = 0, busy = 0;
};

Traffic traffic(size_t gap_min, size_t gap_span, uint64_t seed) {
    Rng rng(seed);
    Stream s(4000000);
    Traffic t;
    size_t at = 2000;
    while (at < s.y.size() - 12000) {
        const uint32_t kind = rng() % 10;
        std::vector<uint8_t> m;
        bool sfd = true;
        if (kind < 4) {
            m = our_beacon(static_cast<uint8_t>(t.frames));
            t.sent_ours++;
        } else if (kind < 7) {
            std::vector<uint8_t> ies;
            for (size_t i = 0; i < 6; i++) {
                ies.push_back(0xDD);
                ies.push_back(30);
                for (int k = 0; k < 30; k++) ies.push_back(static_cast<uint8_t>(rng()));
            }
            m = beacon(kOtherMac, "SomeNetwork", {}, 0, ies);
        } else if (kind < 9) {
            m.assign(100 + rng() % 400, 0);
            for (auto& b : m) b = static_cast<uint8_t>(rng());
            m[0] = 0x08;
            append_fcs(m);
        } else {
            m.assign(20, 0x55);
            sfd = false;
        }
        const double eb = 8 + 22 * rng.uniform();
        const size_t len = s.add(at, frame_in_slice(m, Stream::amp_for(eb, 1.5), (rng.uniform() - 0.5) * 120e3, -2, rng() % 6, rng.uniform() * 6.28, sfd));
        t.busy += len;
        at += len + gap_min + rng() % (gap_span + 1);
        t.frames++;
    }
    t.iq = s.quantise(1.5, seed + 1);
    return t;
}

}  // namespace

/* With the costs measured on the M4, Lock (117 cycles a group) needs about
 * all of the per-buffer limit by itself: no airtime of 1 Mb/s frames near
 * 70 % can be followed without shedding the backlog. What the receiver
 * keeps up with, without loss, is printed by the sweep below; the test
 * after it holds the firmware to its busy-channel point. (On the bench the
 * household channel 6 carried ~25 1 Mb/s frames a second, mostly short.) */
TEST_CASE("dsss work: airtime the receiver follows without backlog loss (measured costs)") {
    std::string t = "\nairtime of 1 Mb/s frames vs overflows (4 s, 40 % of frames 193 byte beacons):\n";
    for (size_t span : {5000u, 10000u, 20000u, 40000u, 80000u}) {
        const Traffic tr = traffic(200, span, 4242);
        Demod& d = fresh(-2);
        const Work w = run_emulated(d, tr.iq);
        char line[160];
        std::snprintf(line, sizeof(line), "  %3zu %% airtime, %3zu frames: overflows %u, shed %4.1f %%, mean load %.0f %%, ours %zu/%zu\n",
                      100 * tr.busy / (tr.iq.size() / 2), tr.frames, d.stats().overflows,
                      100.0 * d.stats().g_shed / (d.stats().g_search ? d.stats().g_search : 1), w.mean_pct(), w.got.ours, tr.sent_ours);
        t += line;
        CHECK(w.max_step <= kStepBound);
        CHECK(w.max_buffer <= kLimit);
    }
    MESSAGE(t);
}

TEST_CASE("dsss work: per buffer bound, no backlog loss on a busy channel (~40 % airtime)") {
    const Traffic t = traffic(200, 20000, 4242);
    Demod& d = fresh(-2);
    const Work w = run_emulated(d, t.iq);
    report("busy (" + std::to_string(100 * t.busy / (t.iq.size() / 2)) + " % airtime, " + std::to_string(t.frames) + " frames)", w, d);
    CHECK(w.max_step <= kStepBound);
    CHECK(w.max_buffer <= kLimit);
    CHECK(d.stats().overflows == 0);
    CHECK(w.got.ours >= t.sent_ours * 6 / 10); /* the weakest ones (8-12 dB) are lost */
}

TEST_CASE("dsss work: heavy traffic (~70 % airtime): bounded per buffer; the backlog is shed, counted") {
    const Traffic t = traffic(200, 5000, 4242);
    Demod& d = fresh(-2);
    const Work w = run_emulated(d, t.iq);
    report("heavy (" + std::to_string(100 * t.busy / (t.iq.size() / 2)) + " % airtime, " + std::to_string(t.frames) + " frames)", w, d);
    CHECK(w.max_step <= kStepBound);
    CHECK(w.max_buffer <= kLimit);
    CHECK(w.got.ours >= t.sent_ours / 2);
}

TEST_CASE("dsss work: saturation (back to back frames, ~100 % airtime): still bounded per buffer; the backlog is shed, counted") {
    const Traffic t = traffic(200, 400, 4343);
    Demod& d = fresh(-2);
    const Work w = run_emulated(d, t.iq);
    report("saturated (" + std::to_string(100 * t.busy / (t.iq.size() / 2)) + " % airtime, " + std::to_string(t.frames) + " frames)", w, d);
    CHECK(w.max_step <= kStepBound);
    CHECK(w.max_buffer <= kLimit);
    CHECK(w.got.ours >= t.sent_ours / 2);
}

TEST_CASE("dsss work: Sync storm (preambles without SFD back to back): bounded per buffer, overflow counted, recovers") {
    Rng rng(9);
    Stream s(4000000 / 2 + 200000);
    size_t at = 1000;
    std::vector<uint8_t> junk(20, 0x55);
    while (at < 4000000 / 2) at += s.add(at, frame_in_slice(junk, Stream::amp_for(30, 1.5), 0, -2, rng() % 6, rng.uniform() * 6.28, false)) + 200;
    const auto m = our_beacon(1);
    s.add(4000000 / 2 + 50000, frame_in_slice(m, Stream::amp_for(30, 1.5), 0, -2, 0, 0.0));
    Demod& d = fresh(-2);
    const Work w = run_emulated(d, s.quantise(1.5, 5));
    report("storm", w, d);
    CHECK(w.max_step <= kStepBound);
    CHECK(w.max_buffer <= kLimit);
    CHECK(w.got.ours == 1); /* the frame after the storm */
}

TEST_CASE("dsss work: a tone 0.5 MHz off the slice centre (all ones after the descrambler) starts one Sync, not one per 128 symbols") {
    Stream s(4000000 / 2);
    for (size_t i = 0; i < s.y.size(); i++) s.y[i] = std::polar(0.05, 2 * kPi * 0.5e6 * i / 4e6);
    Demod& d = fresh(-2);
    const Work w = run_emulated(d, s.quantise(1.5, 2));
    report("tone", w, d);
    CHECK(d.stats().triggers <= 2);
    CHECK(d.stats().overflows == 0);
    CHECK(w.max_buffer <= kLimit);
    Demod& p = fresh(-2, dd::DsssTrigger{});
    run_emulated(p, s.quantise(1.5, 2));
    MESSAGE("tone, prototype trigger rule: " << p.stats().triggers << " triggers, " << p.stats().overflows << " overflows");
}

/* ---------------------------------------------------------------- recordings */

namespace {

struct FileCase {
    const char* stem;
    int expect_m2, expect_p2; /* frames of ours expected at slice -2 / +2 (-1: just report) */
    bool legacy;              /* French / DJI: forwarded only with the bench flag */
};

/* Expected from the prototype (kernel on its own slices, bit-exact with
 * this one): every C5 1 Mb/s file 10/10 at -2 MHz (9 on the 11 Msps file, 10
 * MHz analogue filter); at +2 MHz the NAN / French / DJI files carry a ~1 MHz
 * wide burst at +0.6..+1 MHz over nearly every frame (0-1/10 there). */
const FileCase kFiles[] = {
    {"c5_wifi_beacon_ch6_1Mbps_2437MHz_20Msps", 10, 10, false},
    {"c5_wifi_beacon_ch6_1Mbps_2437MHz_11Msps", 9, 10, false},
    {"c5_wifi_beacon_ch6_1Mbps_2437MHz_22Msps", 10, 10, false},
    {"c5_wifi_nan_ch6_1Mbps_2437MHz_20Msps", 10, -1, false},
    {"c5_wifi_frenchbeacon_ch6_1Mbps_2437MHz_20Msps", 10, -1, true},
    {"c5_wifi_djibeacon_ch6_1Mbps_2437MHz_20Msps", 10, -1, true},
    {"c5silent_ch6_control_2437MHz_20Msps", 0, 0, false},
    {"c5_wifi_beacon_ch6_6Mbps_2437MHz_20Msps", 0, 0, false},
};

/* proc_drone_detect's forwarding: classifier, de-duplication, rate cap. */
struct Forward {
    dd::WifiDedup dedup{};
    bool legacy = false;
    size_t odid = 0, legacy_n = 0, fwd = 0, fcs_ok = 0, ours = 0, others = 0, others_fwd = 0;
    std::vector<int16_t> rssi;
};

void forward_cb(void* ctx, const dd::DsssFrame& f) {
    Forward& w = *static_cast<Forward*>(ctx);
    w.fcs_ok++;
    const bool ours = is_ours(f.mpdu, f.length);
    (ours ? w.ours : w.others)++;
    if (ours) w.rssi.push_back(f.rssi_cdb);
    Rid r{};
    const uint8_t kind = dd::dd_wifi_rid_find(f.mpdu, f.length, w.legacy, r);
    if (!kind) return;
    (kind <= Rid::KIND_NAN ? w.odid : w.legacy_n)++;
    uint16_t sum = 0;
    for (size_t i = 0; i < r.payload_len; i++) sum += r.payload[i];
    const uint32_t t_ms = f.end_sample / 4000;
    if (w.dedup.duplicate(r.addr, kind, r.counter, sum, t_ms) || !w.dedup.rate_ok(t_ms)) return;
    w.fwd++;
    if (!ours) w.others_fwd++;
}

Forward forward_run(const std::vector<int8_t>& iq, int slice, bool legacy, bool random_chunks = false) {
    Demod& d = fresh(slice);
    Forward w;
    w.legacy = legacy;
    const size_t n = iq.size() / 2;
    Rng rng(31);
    for (size_t at = 0; at < n;) {
        const size_t c = std::min<size_t>(random_chunks ? 1 + rng() % 3000 : kBuffer, n - at);
        d.execute(iq.data() + 2 * at, c, forward_cb, &w);
        at += c;
    }
    return w;
}

}  // namespace

TEST_CASE("recording: dsss Wi-Fi Remote ID from the C5 at the firmware's tuning (slices -2 / +2 MHz)") {
    const std::string dir = env_dir("DD_IQ_DIR");
    std::string table =
        "\nWi-Fi DSSS receiver on the recordings (2 s each, 10 frames of ours expected; ch 6, firmware kernel and tuning)\n"
        "file                                           slice  ours FCS-ok  forwarded (ODID/legacy)  third-party FCS-ok/forwarded  RSSI dBFS\n";
    bool any = false;
    for (const auto& fc : kFiles) {
        const Recording r = load_recording(dir, fc.stem);
        if (!r.ok) continue;
        any = true;
        for (int slice : {-2, 2}) {
            const auto iq = dwell_view(r, 2437e6, slice);
            const Forward w = forward_run(iq, slice, fc.legacy);
            const Forward wr = forward_run(iq, slice, fc.legacy, true);
            const Forward wn = fc.legacy ? forward_run(iq, slice, false) : w;
            int rmin = 0, rmax = -20000;
            for (int16_t v : w.rssi) {
                rmin = std::min<int>(rmin, v);
                rmax = std::max<int>(rmax, v);
            }
            char line[260];
            std::snprintf(line, sizeof(line), "%-46s %+d     %2zu          %2zu (%zu/%zu)                  %2zu / %zu                        %s\n", fc.stem, slice,
                          w.ours, w.fwd, w.odid, w.legacy_n, w.others, w.others_fwd,
                          w.rssi.empty() ? "-" : (std::to_string((rmin - 50) / 100) + ".." + std::to_string((rmax - 50) / 100)).c_str());
            table += line;
            CAPTURE(fc.stem);
            CAPTURE(slice);
            const int expect = slice < 0 ? fc.expect_m2 : fc.expect_p2;
            if (expect > 0) CHECK(static_cast<int>(w.ours) >= expect);
            if (expect == 0) CHECK(w.ours == 0);
            CHECK(w.others_fwd == 0); /* nothing of third parties, ever */
            CHECK(wr.ours == w.ours); /* chunking does not matter */
            CHECK(wr.fwd == w.fwd);
            CHECK(w.fwd == w.ours);                                    /* every frame of ours carries Remote ID */
            if (fc.legacy) CHECK(wn.fwd == 0);                         /* legacy IEs: bench flag only */
            for (int16_t v : w.rssi) CHECK(std::abs(v + 3080) <= 200); /* C5 bursts: -30.8 dBFS over 20 MHz */
        }
    }
    if (!any) {
        MESSAGE("skipped: no C5 Wi-Fi recordings in DD_IQ_DIR");
        return;
    }
    MESSAGE(table);
}

/* Direct 4 Msps captures at exactly the firmware's tuning (HackRF mode,
 * hackrf_transfer -s 4e6 -f slice centre): what the real PRALINE front end
 * delivers, no slice model. The first direct check of the model the tap
 * tables come from. Each file is also run as the app's Debug "Inv" does in
 * DSSS mode (the taps of the other half on the same samples: what a
 * mirrored front end would need), which must decode less. (Conjugating the
 * samples as well would change nothing: taps(-s) = conj(taps(s)) and DBPSK
 * decisions survive a conjugate.) */
TEST_CASE("recording: dsss direct 4 Msps captures at the firmware's slice tuning (front end check, no model)") {
    const std::string dir = env_dir("DD_IQ_DIR");
    struct Direct {
        const char* stem;
        int expect; /* frames of ours at least (-1: just report) */
    };
    const Direct files[] = {
        /* 10 s each, the C5 sends 50 beacons (measured: 49 and 50). */
        {"c5_wifi_beacon_ch6_1Mbps_fwslice_m2_2435MHz_4Msps", 45},
        {"c5_wifi_beacon_ch6_1Mbps_fwslice_p2_2439MHz_4Msps", 45},
        {"c5_wifi_beacon_ch6_1Mbps_fwslice_m2_2435MHz_4Msps_b", 45}, /* re-capture ~25 min later (measured 47, 50) */
        {"c5_wifi_beacon_ch6_1Mbps_fwslice_p2_2439MHz_4Msps_b", 45},
        {"c5silent_ch6_control_fwslice_m2_2435MHz_4Msps", 0},
        {"c5silent_ch6_control_fwslice_p2_2439MHz_4Msps", 0},
    };
    std::string table =
        "\nDirect 4 Msps captures (firmware kernel, 2048 sample buffers): ours FCS-ok / forwarded / third-party FCS-ok, and with the other half's taps (Debug Inv)\n";
    bool any = false;
    size_t frames = 0, frames_inv = 0; /* all FCS valid frames, right taps / mirrored taps */
    for (const auto& fc : files) {
        const Recording r = load_recording(dir, fc.stem);
        if (!r.ok) continue;
        any = true;
        CHECK(std::abs(r.fs - 4e6) < 1e3);
        const int slice = static_cast<int>(std::lround((r.fc - 2437e6) / 1e6));
        CAPTURE(fc.stem);
        CAPTURE(slice);
        REQUIRE(dd::dsss_taps(slice) != nullptr);
        std::vector<int8_t> iq(2 * r.x.size());
        double p = 0;
        for (size_t i = 0; i < r.x.size(); i++) {
            iq[2 * i] = static_cast<int8_t>(std::lround(r.x[i].real() * 128));
            iq[2 * i + 1] = static_cast<int8_t>(std::lround(r.x[i].imag() * 128));
            p += std::norm(r.x[i]) * 128 * 128 / 2;
        }
        const Forward w = forward_run(iq, slice, false);
        const Forward wc = forward_run(iq, -slice, false); /* Debug "Inv" */
        Demod& d = fresh(slice); /* once more, for the kernel's counters */
        const Got g = run_buffers(d, iq);
        const auto s = d.stats();
        int rmin = 0, rmax = -20000;
        for (int16_t v : w.rssi) {
            rmin = std::min<int>(rmin, v);
            rmax = std::max<int>(rmax, v);
        }
        char line[400];
        std::snprintf(line, sizeof(line),
                      "%-50s slice %+d  %4.1f s  rms %.1f LSB  ours %3zu fwd %3zu 3rd %3zu | Inv: ours %3zu 3rd %3zu | trig %u sfd %u hdr_bad %u hdr_2m %u not_1m %u filt %u fcs_bad %u ok %u | RSSI %s dBFS\n",
                      fc.stem, slice, r.x.size() / 4e6, std::sqrt(p / r.x.size()), w.ours, w.fwd, w.others, wc.ours, wc.others,
                      s.triggers, s.sfd, s.hdr_bad, s.hdr_2m, s.not_1m, s.filtered, s.fcs_bad, s.ok,
                      w.rssi.empty() ? "-" : (std::to_string((rmin - 50) / 100) + ".." + std::to_string((rmax - 50) / 100)).c_str());
        table += line;
        (void)g;
        if (fc.expect > 0) CHECK(static_cast<int>(w.ours) >= fc.expect);
        frames += w.ours + w.others;
        frames_inv += wc.ours + wc.others;
        if (fc.expect == 0) CHECK(w.ours == 0);
        CHECK(w.others_fwd == 0);
        CHECK(w.fwd == w.ours);
    }
    if (!any) {
        MESSAGE("skipped: no direct 4 Msps captures in DD_IQ_DIR");
        return;
    }
    MESSAGE(table << "FCS valid frames, all files: " << frames << " with the slice's taps, " << frames_inv << " with the mirrored ones");
    /* The front end is not mirrored: the strong C5 frames decode either way
     * (the taps matter little at -12 dBFS), the weaker third-party frames
     * need the right ones (measured 413 vs 348 over the six files). */
    CHECK(frames > frames_inv + frames_inv / 20);
}

/* The direct captures through proc_drone_detect's real-time scheduling with
 * the cycle costs measured on the HackRF Pro (DDSTAT cyc_* with the search
 * mode 3 build): what the device should decode if its only difference from
 * the plain host run is the per-buffer cycle limit. */
TEST_CASE("recording: dsss direct captures under the device's measured cycle costs (real-time emulation)") {
    const std::string dir = env_dir("DD_IQ_DIR");
    std::string table = "\nReal-time emulation, measured costs (search 107 / sync 430 / lock 117 cycles a group, CFO 5100):\n";
    bool any = false;
    for (const char* stem : {"c5_wifi_beacon_ch6_1Mbps_fwslice_m2_2435MHz_4Msps", "c5_wifi_beacon_ch6_1Mbps_fwslice_p2_2439MHz_4Msps",
                             "c5_wifi_beacon_ch6_1Mbps_fwslice_m2_2435MHz_4Msps_b", "c5_wifi_beacon_ch6_1Mbps_fwslice_p2_2439MHz_4Msps_b"}) {
        const Recording r = load_recording(dir, stem);
        if (!r.ok) continue;
        any = true;
        const int slice = static_cast<int>(std::lround((r.fc - 2437e6) / 1e6));
        std::vector<int8_t> iq(2 * r.x.size());
        for (size_t i = 0; i < r.x.size(); i++) {
            iq[2 * i] = static_cast<int8_t>(std::lround(r.x[i].real() * 128));
            iq[2 * i + 1] = static_cast<int8_t>(std::lround(r.x[i].imag() * 128));
        }
        Demod& d = fresh(slice);
        const Work w = run_emulated_costs(d, iq, 107, 430, 117, 5300);
        const auto& s = d.stats();
        char line[300];
        std::snprintf(line, sizeof(line), "%-52s ours %3zu  overflows %u  shed %4.1f %%  mean %.0f %%  max buffer %u us  max step %u us  max backlog %u  trig %u sfd %u hdr_bad %u fcs_bad %u ok %u\n",
                      stem, w.got.ours, s.overflows, 100.0 * s.g_shed / (s.g_search ? s.g_search : 1), w.mean_pct(), w.max_buffer / 200, w.max_step / 200, w.max_backlog, s.triggers, s.sfd, s.hdr_bad, s.fcs_bad, s.ok);
        CHECK(w.got.ours >= 45);
        CHECK(w.max_buffer <= kLimit);
        CHECK(w.max_step <= kStepBound);
        table += line;
    }
    if (!any) {
        MESSAGE("skipped: no direct 4 Msps captures in DD_IQ_DIR");
        return;
    }
    MESSAGE(table);
}

TEST_CASE("recording: dsss privacy: nothing forwarded from the control, OFDM and ambient recordings (third-party 1 Mb/s frames decoded, counted only)") {
    const std::string dir = env_dir("DD_IQ_DIR");
    struct Amb {
        const char* stem;
        double channels[2];
    };
    /* Wi-Fi channels whose two slices lie inside each recording's 20 MHz. */
    const Amb amb[] = {
        {"c5silent_ch6_control_2437MHz_20Msps", {2437e6, 0}},
        {"c5_wifi_beacon_ch6_6Mbps_2437MHz_20Msps", {2437e6, 0}},
        {"ambient_c5silent_2412MHz_20Msps", {2412e6, 0}},
        {"ambient_c5silent_2430MHz_20Msps", {2427e6, 2432e6}},
        {"ambient_c5silent_2448MHz_20Msps", {2447e6, 2452e6}},
        {"ambient_c5silent_2466MHz_20Msps", {2462e6, 2467e6}},
    };
    size_t third_party = 0, forwarded = 0, views = 0;
    std::string table = "\nPrivacy: FCS valid third-party frames decoded / forwarded per dwell view (legacy flag on)\n";
    for (const auto& a : amb) {
        const Recording r = load_recording(dir, a.stem);
        if (!r.ok) continue;
        for (double ch : a.channels) {
            if (ch == 0) continue;
            for (int slice : {-2, 2}) {
                const Forward w = forward_run(dwell_view(r, ch, slice), slice, true);
                views++;
                third_party += w.others;
                forwarded += w.fwd;
                CHECK(w.ours == 0);
                CHECK(w.fwd == 0);
                char line[120];
                std::snprintf(line, sizeof(line), "%-40s %4.0f MHz %+d: %zu / %zu\n", a.stem, ch / 1e6, slice, w.others, w.fwd);
                table += line;
            }
        }
    }
    if (!views) {
        MESSAGE("skipped: no control / ambient recordings in DD_IQ_DIR");
        return;
    }
    MESSAGE(table << views << " views: " << third_party << " third-party frames decoded, " << forwarded << " forwarded");
    CHECK(third_party > 0); /* the gate saw real traffic */
    CHECK(forwarded == 0);
}
