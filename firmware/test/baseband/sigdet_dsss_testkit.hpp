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

/* Host test kit of the Wi-Fi Remote ID receiver (baseband/sigdet_dsss.hpp):
 *  - SliceModel: what the M4 sees in a DSSS dwell. The HackRF Pro (PRALINE)
 *    receive chain as the prototype (mayhem-build/proto/dsss/slicer.py)
 *    modelled it: shift the slice centre to 0 Hz, resample to 16 Msps with a
 *    Kaiser (beta 10) low-pass exactly as scipy's resample_poly designs it
 *    (stands in for the flat earlier FPGA stages and the recording's own
 *    filter), the FPGA's last two half-band decimators hbfir2 (11 taps) and
 *    hbfir1 (23 taps), round to int8 (half to even) at the recording's own
 *    scale. Streaming, so a 2 s recording needs no gigabytes.
 *  - Recordings (.ci8 + SigMF-style .json sidecar) from SIGDET_IQ_DIR.
 *  - Dot11Tx: a synthetic 802.11b 1 Mb/s long preamble transmitter (scrambler,
 *    DBPSK, 11 chip Barker) at 22 Msps, and MPDU builders (beacon with an
 *    Open Drone ID vendor IE, NAN service discovery frame) with FCS.
 *  - The privacy gate of the tests: frame content is looked at only when
 *    addr2 is the test transmitter's (02:52:49:44:{00,F0,D0}:xx); everything
 *    else is counted, never printed. */

#ifndef __SIGDET_DSSS_TESTKIT_HPP__
#define __SIGDET_DSSS_TESTKIT_HPP__

#include "sigdet_dsss.hpp"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace sigdet_test {

constexpr double kPi = 3.141592653589793238463;
using cplx = std::complex<double>;

/* Small deterministic generator (xorshift64*). */
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

/* ------------------------------------------------------------ slice model */

inline double bessel_i0(double x) {
    double sum = 1, term = 1;
    for (int k = 1; k < 60; k++) {
        term *= (x / (2 * k)) * (x / (2 * k));
        sum += term;
        if (term < 1e-17 * sum) break;
    }
    return sum;
}

inline uint64_t gcd64(uint64_t a, uint64_t b) {
    while (b) {
        const uint64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

class SliceModel {
   public:
    static constexpr double kOutRate = 4e6;

    /* fs_in: the input rate (an integer number of Hz); shift_hz: where the
     * slice centre lies in the input (input frequency that lands at 0 Hz). */
    SliceModel(double fs_in, double shift_hz)
        : fs_in_{fs_in}, shift_{shift_hz} {
        const uint64_t num = static_cast<uint64_t>(4 * kOutRate), den = static_cast<uint64_t>(std::llround(fs_in));
        const uint64_t g = gcd64(num, den);
        up_ = num / g;
        down_ = den / g;
        if (up_ != 1 || down_ != 1) {
            /* scipy.signal.resample_poly: firwin(2 half_len + 1, 1 / max, kaiser 10), x up. */
            const uint64_t m = std::max(up_, down_);
            half_ = 10 * m;
            const size_t n = 2 * half_ + 1;
            h_.resize(n);
            const double fc = 1.0 / m, alpha = (n - 1) / 2.0;
            double sum = 0;
            for (size_t i = 0; i < n; i++) {
                const double t = fc * (i - alpha);
                const double sinc = t == 0 ? 1.0 : std::sin(kPi * t) / (kPi * t);
                const double r = 2.0 * i / (n - 1) - 1;
                const double w = bessel_i0(10.0 * std::sqrt(std::max(0.0, 1 - r * r))) / bessel_i0(10.0);
                h_[i] = fc * sinc * w;
                sum += h_[i];
            }
            for (auto& v : h_) v = v / sum * static_cast<double>(up_);
        }
    }

    /* x: input samples (full scale 1.0) -> complex at 4 Msps (full scale 1.0, not rounded). */
    std::vector<cplx> run(const std::vector<cplx>& x) const {
        static constexpr double hb2[11] = {3 / 256.0, 0, -16 / 256.0, 0, 77 / 256.0, 128 / 256.0, 77 / 256.0, 0, -16 / 256.0, 0, 3 / 256.0};
        static constexpr double hb1[23] = {-2 / 1024.0, 0, 7 / 1024.0, 0, -18 / 1024.0, 0, 41 / 1024.0, 0, -92 / 1024.0, 0, 320 / 1024.0, 512 / 1024.0,
                                           320 / 1024.0, 0, -92 / 1024.0, 0, 41 / 1024.0, 0, -18 / 1024.0, 0, 7 / 1024.0, 0, -2 / 1024.0};
        const size_t n_in = x.size();
        std::vector<cplx> xs(n_in);
        const double w = -2 * kPi * shift_ / fs_in_;
        for (size_t i = 0; i < n_in; i++) {
            const double ph = std::fmod(w * static_cast<double>(i), 2 * kPi);
            xs[i] = x[i] * cplx(std::cos(ph), std::sin(ph));
        }
        const size_t n16 = h_.empty() ? n_in : static_cast<size_t>((n_in * up_ + down_ - 1) / down_);
        const size_t n8 = (n16 + 1) / 2, n4 = (n8 + 1) / 2;
        std::vector<cplx> y8(n8), y4(n4);
        /* Stage 1 + hbfir2 ('same' convolution, every 2nd): y8[i] = sum hb2[j] y16[2 i + 5 - j]. */
        cplx hist16[11] = {};
        size_t at16 = 0;
        auto put16 = [&](size_t m, cplx v) {
            hist16[m % 11] = v;
            const long i2 = static_cast<long>(m) - 5;
            if (i2 >= 0 && (i2 & 1) == 0 && static_cast<size_t>(i2 / 2) < n8) {
                cplx acc{};
                for (size_t j = 0; j < 11; j++)
                    if (m >= j) acc += hb2[j] * hist16[(m - j) % 11];
                y8[i2 / 2] = acc;
            }
        };
        for (size_t m = 0; m < n16; m++, at16++) put16(m, y16(xs, m));
        for (size_t m = n16; m < n16 + 6; m++) put16(m, cplx{});
        /* hbfir1: y4[k] = sum hb1[j] y8[2 k + 11 - j]. */
        for (size_t k = 0; k < n4; k++) {
            cplx acc{};
            for (size_t j = 0; j < 23; j++) {
                const long q = static_cast<long>(2 * k) + 11 - static_cast<long>(j);
                if (q >= 0 && static_cast<size_t>(q) < n8) acc += hb1[j] * y8[q];
            }
            y4[k] = acc;
        }
        (void)at16;
        return y4;
    }

    /* Rounded to int8 (numpy's round: half to even), saturating; scale
     * multiplies first (full scale 1.0 -> 128). */
    static std::vector<int8_t> to_int8(const std::vector<cplx>& y, double scale = 128.0) {
        std::vector<int8_t> out(2 * y.size());
        std::fesetround(FE_TONEAREST);
        auto q = [](double v) { return static_cast<int8_t>(std::max(-128.0, std::min(127.0, std::nearbyint(v)))); };
        for (size_t i = 0; i < y.size(); i++) {
            out[2 * i] = q(y[i].real() * scale);
            out[2 * i + 1] = q(y[i].imag() * scale);
        }
        return out;
    }

   private:
    /* y16[m] = sum_n x[n] h[m down + half - n up] (resample_poly's alignment). */
    cplx y16(const std::vector<cplx>& x, size_t m) const {
        if (h_.empty()) return m < x.size() ? x[m] : cplx{};
        const long base = static_cast<long>(m * down_ + half_);
        long n_lo = (base - static_cast<long>(2 * half_) + static_cast<long>(up_) - 1) / static_cast<long>(up_);
        if (base - static_cast<long>(2 * half_) < 0) n_lo = 0;
        long n_hi = base / static_cast<long>(up_);
        if (n_hi >= static_cast<long>(x.size())) n_hi = static_cast<long>(x.size()) - 1;
        cplx acc{};
        for (long n = n_lo; n <= n_hi; n++) acc += x[n] * h_[base - n * static_cast<long>(up_)];
        return acc;
    }

    double fs_in_, shift_;
    uint64_t up_{1}, down_{1}, half_{0};
    std::vector<double> h_;
};

/* ------------------------------------------------------------ recordings */

inline std::string env_dir(const char* var) {
    const char* v = std::getenv(var);
    return v ? std::string(v) : std::string();
}

inline bool file_exists(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return f.good();
}

inline std::string slurp(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/* First number after "key": in a JSON text (enough for the sidecars). */
inline double json_number(const std::string& text, const std::string& key) {
    const size_t at = text.find("\"" + key + "\"");
    if (at == std::string::npos) return 0;
    const size_t colon = text.find(':', at + key.size() + 2);
    return std::strtod(text.c_str() + colon + 1, nullptr);
}

struct Recording {
    std::vector<cplx> x; /* full scale 1.0 */
    double fs = 0, fc = 0;
    bool ok = false;
};

/* stem.ci8 + stem.json from dir; fs is the sidecar's measured rate when it
 * has one (the "22 Msps" file runs at 21.8). */
inline Recording load_recording(const std::string& dir, const std::string& stem) {
    Recording r;
    const std::string data = dir + "/" + stem + ".ci8", meta = dir + "/" + stem + ".json";
    if (dir.empty() || !file_exists(data) || !file_exists(meta)) return r;
    const std::string m = slurp(meta);
    r.fs = json_number(m, "measured_rate_estimate");
    if (r.fs < 1e6) r.fs = json_number(m, "core:sample_rate");
    r.fc = json_number(m, "core:frequency");
    if (r.fs < 1e6 || r.fc < 1e6) return r;
    const std::string raw = slurp(data);
    r.x.resize(raw.size() / 2);
    for (size_t i = 0; i < r.x.size(); i++)
        r.x[i] = cplx(static_cast<int8_t>(raw[2 * i]) / 128.0, static_cast<int8_t>(raw[2 * i + 1]) / 128.0);
    r.ok = !r.x.empty();
    return r;
}

/* The M4's int8 view of a DSSS dwell on Wi-Fi channel centre channel_hz,
 * slice offset slice_mhz, with the firmware's tuning (sigdet_dsss.hpp): the
 * slice centre channel + slice lands at 0 Hz. */
inline std::vector<int8_t> dwell_view(const Recording& r, double channel_hz, int slice_mhz) {
    const double centre = channel_hz + slice_mhz * 1e6;
    return SliceModel::to_int8(SliceModel(r.fs, centre - r.fc).run(r.x));
}

/* ------------------------------------------------------------ privacy gate */

/* addr2 (transmitter) of the C5 test transmitter: 02:52:49:44:{00,F0,D0}:xx. */
inline bool is_ours(const uint8_t* mpdu, size_t len) {
    if (len < 16) return false;
    const uint8_t* a2 = mpdu + 10;
    return a2[0] == 0x02 && a2[1] == 0x52 && a2[2] == 0x49 && a2[3] == 0x44 && (a2[4] == 0x00 || a2[4] == 0xF0 || a2[4] == 0xD0);
}

/* ------------------------------------------------------------ transmitter */

inline uint32_t crc32_bytes(const uint8_t* p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) c = sigdet::crc32_byte(c, p[i]);
    return ~c;
}

inline void append_fcs(std::vector<uint8_t>& mpdu) {
    const uint32_t c = crc32_bytes(mpdu.data(), mpdu.size());
    for (int i = 0; i < 4; i++) mpdu.push_back(static_cast<uint8_t>(c >> (8 * i)));
}

/* A 25 byte ODID Basic ID message (serial number, multirotor). */
inline std::vector<uint8_t> odid_basic_id(const char* id) {
    std::vector<uint8_t> m(25, 0);
    m[0] = 0x02; /* type 0, protocol version 2 */
    m[1] = 0x12; /* ID type 1 (serial), UA type 2 (multirotor) */
    for (size_t i = 0; i < 20 && id[i]; i++) m[2 + i] = static_cast<uint8_t>(id[i]);
    return m;
}

/* A 25 byte ODID Location message (airborne) at lat/lon (1e-7 degrees). */
inline std::vector<uint8_t> odid_location(int32_t lat, int32_t lon) {
    std::vector<uint8_t> m(25, 0);
    m[0] = 0x12; /* type 1, version 2 */
    m[1] = 0x20; /* status airborne */
    m[3] = 32;   /* 8 m/s */
    for (int i = 0; i < 4; i++) {
        m[5 + i] = static_cast<uint8_t>(lat >> (8 * i));
        m[9 + i] = static_cast<uint8_t>(lon >> (8 * i));
    }
    m[13] = m[15] = m[17] = 0x10; /* altitudes / height raw 0x0810: 32 m */
    m[14] = m[16] = m[18] = 0x08;
    return m;
}

/* Message pack header + messages. */
inline std::vector<uint8_t> odid_pack(const std::vector<std::vector<uint8_t>>& msgs) {
    std::vector<uint8_t> p = {0xF2, 25, static_cast<uint8_t>(msgs.size())};
    for (const auto& m : msgs) p.insert(p.end(), m.begin(), m.end());
    return p;
}

inline void put_header(std::vector<uint8_t>& f, uint8_t fc, const uint8_t* da, const uint8_t* sa) {
    f = {fc, 0, 0, 0};
    f.insert(f.end(), da, da + 6);
    f.insert(f.end(), sa, sa + 6);
    f.insert(f.end(), sa, sa + 6); /* BSSID */
    f.push_back(0x10);
    f.push_back(0x00);
}

/* Beacon (fc 0x80, or 0x50 probe response) with SSID and, if pack is not
 * empty, the ASTM vendor IE (oui) FA:0B:BC 0D counter pack; extra_ies before it. */
inline std::vector<uint8_t> beacon(const uint8_t* sa, const char* ssid, const std::vector<uint8_t>& pack, uint8_t counter, const std::vector<uint8_t>& extra_ies = {}, uint8_t fc = 0x80, const uint8_t* oui = nullptr) {
    static constexpr uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static constexpr uint8_t astm[3] = {0xFA, 0x0B, 0xBC};
    std::vector<uint8_t> f;
    put_header(f, fc, bcast, sa);
    for (int i = 0; i < 8; i++) f.push_back(0); /* timestamp */
    f.push_back(0x64);                          /* interval 100 TU */
    f.push_back(0x00);
    f.push_back(0x21); /* capability */
    f.push_back(0x04);
    const size_t sl = std::string(ssid).size();
    f.push_back(0x00);
    f.push_back(static_cast<uint8_t>(sl));
    f.insert(f.end(), ssid, ssid + sl);
    f.insert(f.end(), {0x01, 0x01, 0x82}); /* supported rates: 1 Mb/s */
    f.insert(f.end(), extra_ies.begin(), extra_ies.end());
    if (!pack.empty()) {
        const uint8_t* o = oui ? oui : astm;
        f.push_back(0xDD);
        f.push_back(static_cast<uint8_t>(5 + pack.size()));
        f.insert(f.end(), o, o + 3);
        f.push_back(0x0D);
        f.push_back(counter);
        f.insert(f.end(), pack.begin(), pack.end());
    }
    append_fcs(f);
    return f;
}

/* NAN service discovery frame with the ODID service (opendroneid layout:
 * service descriptor + extension attributes). */
inline std::vector<uint8_t> nan_sdf(const uint8_t* sa, const std::vector<uint8_t>& pack, uint8_t counter) {
    static constexpr uint8_t cluster[6] = {0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00};
    std::vector<uint8_t> f;
    put_header(f, 0xD0, cluster, sa);
    f.insert(f.end(), {0x04, 0x09, 0x50, 0x6F, 0x9A, 0x13});
    const size_t si = 1 + pack.size();
    const size_t len = 6 + 3 + 1 + si;
    f.insert(f.end(), {0x03, static_cast<uint8_t>(len), static_cast<uint8_t>(len >> 8), 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09, 0x01, 0x00, 0x10,
                       static_cast<uint8_t>(si), counter});
    f.insert(f.end(), pack.begin(), pack.end());
    f.insert(f.end(), {0x0E, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00});
    append_fcs(f);
    return f;
}

/* 802.11b 1 Mb/s long preamble PPDU at 22 Msps (2 samples a chip),
 * amplitude 1. sfd: false sends SYNC without SFD (a frame that never locks). */
inline std::vector<cplx> dot11b_waveform(const std::vector<uint8_t>& mpdu, double phase0 = 0.0, bool sfd = true) {
    std::vector<int> bits;
    for (int i = 0; i < 128; i++) bits.push_back(1);
    const uint32_t sfd_bits = sfd ? 0xF3A0u : 0xFFFFu;
    for (int i = 0; i < 16; i++) bits.push_back((sfd_bits >> i) & 1);
    const uint32_t len_us = static_cast<uint32_t>(mpdu.size() * 8);
    const uint32_t hdr = sigdet::kPlcpSignal1M | (0x04u << 8) | (len_us << 16);
    const uint32_t crc = sigdet::plcp_crc16(hdr);
    for (int i = 0; i < 32; i++) bits.push_back((hdr >> i) & 1);
    for (int i = 0; i < 16; i++) bits.push_back((crc >> i) & 1);
    for (uint8_t b : mpdu)
        for (int i = 0; i < 8; i++) bits.push_back((b >> i) & 1);
    /* Scrambler z^-7 + z^-4 + 1 (self-synchronising), long preamble seed. */
    uint32_t sr = 0x6C;
    static constexpr int barker[11] = {1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1};
    std::vector<cplx> w;
    w.reserve(bits.size() * 22);
    double ph = phase0;
    for (int b : bits) {
        const int s = b ^ ((sr >> 3) & 1) ^ ((sr >> 6) & 1);
        sr = ((sr << 1) | static_cast<uint32_t>(s)) & 0x7F;
        if (s) ph += kPi;
        const cplx sym = std::polar(1.0, ph);
        for (int c = 0; c < 11; c++) {
            w.push_back(sym * static_cast<double>(barker[c]));
            w.push_back(sym * static_cast<double>(barker[c]));
        }
    }
    return w;
}

/* One frame through the slice model: at 4 Msps, complex, full scale 1.0.
 * amp: chip amplitude; cfo_hz; offset_mhz: slice centre - channel centre;
 * frac: 0..5 samples of 22 Msps in front (sub-sample timing at 4 Msps). */
inline std::vector<cplx> frame_in_slice(const std::vector<uint8_t>& mpdu, double amp, double cfo_hz, int offset_mhz, size_t frac, double phase0, bool sfd = true) {
    std::vector<cplx> w(frac, cplx{});
    const auto f = dot11b_waveform(mpdu, phase0, sfd);
    w.insert(w.end(), f.begin(), f.end());
    for (int i = 0; i < 200; i++) w.push_back(cplx{}); /* filter tail */
    for (size_t i = 0; i < w.size(); i++) w[i] *= amp * std::polar(1.0, 2 * kPi * cfo_hz * i / 22e6);
    return SliceModel(22e6, offset_mhz * 1e6).run(w);
}

}  // namespace sigdet_test

#endif /*__SIGDET_DSSS_TESTKIT_HPP__*/
