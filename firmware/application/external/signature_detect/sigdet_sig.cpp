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

#include "sigdet_sig.hpp"

#include <cstring>

namespace ui::external_app::signature_detect {

namespace {

constexpr const char* kKindNames[SK_COUNT] = {"other", "tracker", "light", "beacon", "phone", "audio", "sensor",
                                              "home", "mesh", "meter", "radio", "noise", "drone"};
constexpr const char kKindTags[SK_COUNT][5] = {"OTHR", "TRKR", "LITE", "BCN", "PHON", "AUD", "SENS",
                                               "HOME", "MESH", "METR", "RAD", "NOIS", "DRON"};
constexpr const char* kValidNames[3] = {"doc", "standin", "real"};

bool is_space(char c) {
    return c == ' ' || c == '\t';
}

void trim(const char*& b, const char*& e) {
    while (b < e && is_space(*b)) b++;
    while (e > b && is_space(e[-1])) e--;
}

bool equals(const char* b, const char* e, const char* key) {
    const size_t n = strlen(key);
    return static_cast<size_t>(e - b) == n && memcmp(b, key, n) == 0;
}

bool parse_uint(const char* b, const char* e, uint32_t max, uint32_t& out) {
    if (b == e || e - b > 7) return false;
    uint32_t v = 0;
    for (; b < e; b++) {
        if (*b < '0' || *b > '9') return false;
        v = v * 10 + (*b - '0');
    }
    if (v > max) return false;
    out = v;
    return true;
}

/* "lo-hi" (each 0..max). */
bool parse_range(const char* b, const char* e, uint32_t max, uint32_t& lo, uint32_t& hi) {
    const char* d = b;
    while (d < e && *d != '-') d++;
    if (d == e) return false;
    const char* lb = b;
    const char* le = d;
    const char* hb = d + 1;
    const char* he = e;
    trim(lb, le);
    trim(hb, he);
    return parse_uint(lb, le, max, lo) && parse_uint(hb, he, max, hi) && lo <= hi;
}

/* MHz with up to 3 decimals -> kHz, 1..7250 MHz. */
bool parse_mhz(const char* b, const char* e, uint32_t& khz) {
    const char* dot = b;
    while (dot < e && *dot != '.') dot++;
    uint32_t whole = 0, frac = 0;
    if (!parse_uint(b, dot, 7250, whole)) return false;
    if (dot < e) {
        const char* f = dot + 1;
        if (f == e || e - f > 3) return false;
        uint32_t scale = 100;
        for (; f < e; f++, scale /= 10) {
            if (*f < '0' || *f > '9') return false;
            frac += (*f - '0') * scale;
        }
    }
    khz = whole * 1000 + frac;
    return khz >= 1000 && khz <= 7250000;
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Exactly 4 hex digits. */
bool parse_hex16(const char* b, const char* e, uint16_t& out) {
    if (e - b != 4) return false;
    uint32_t v = 0;
    for (; b < e; b++) {
        const int d = hex_digit(*b);
        if (d < 0) return false;
        v = (v << 4) | static_cast<uint32_t>(d);
    }
    out = static_cast<uint16_t>(v);
    return true;
}

/* Byte pattern: hex pairs, '?' a wildcard nibble, optional &MASK (hex). */
bool parse_pattern(const char* b, const char* e, SigBytes& p) {
    p = SigBytes{};
    const char* amp = b;
    while (amp < e && *amp != '&') amp++;
    const size_t digits = amp - b;
    if (digits == 0 || (digits & 1) || digits / 2 > kSigPatBytes) return false;
    p.n = static_cast<uint8_t>(digits / 2);
    for (size_t i = 0; i < digits; i++) {
        const char c = b[i];
        uint8_t v = 0, m = 0xF;
        if (c == '?') {
            m = 0;
        } else {
            const int d = hex_digit(c);
            if (d < 0) return false;
            v = static_cast<uint8_t>(d);
        }
        const size_t k = i / 2;
        const int sh = (i & 1) ? 0 : 4;
        p.v[k] |= static_cast<uint8_t>(v << sh);
        p.m[k] |= static_cast<uint8_t>(m << sh);
    }
    if (amp < e) {
        const char* mb = amp + 1;
        if (static_cast<size_t>(e - mb) != digits) return false;
        for (size_t k = 0; k < p.n; k++) {
            const int hi = hex_digit(mb[2 * k]), lo = hex_digit(mb[2 * k + 1]);
            if (hi < 0 || lo < 0) return false;
            p.m[k] &= static_cast<uint8_t>((hi << 4) | lo);
        }
    }
    for (size_t k = 0; k < p.n; k++) p.v[k] &= p.m[k];
    return true;
}

bool copy_label(const char* b, const char* e, char* out, size_t cap) {
    const size_t n = e - b;
    if (n == 0 || n >= cap) return false;
    for (size_t i = 0; i < n; i++) {
        const char c = b[i];
        out[i] = (c >= 0x20 && c < 0x7F && c != '"') ? c : '_';
    }
    out[n] = 0;
    return true;
}

bool parse_kind(const char* b, const char* e, uint8_t& kind) {
    for (uint8_t k = 0; k < SK_COUNT; k++)
        if (equals(b, e, kKindNames[k])) {
            kind = k;
            return true;
        }
    return false;
}

bool parse_valid(const char* b, const char* e, uint8_t& valid) {
    for (uint8_t k = 0; k < 3; k++)
        if (equals(b, e, kValidNames[k])) {
            valid = k;
            return true;
        }
    return false;
}

/* The built-in band of this name ("2.4", "915", ...), BAND_COUNT if none. */
uint8_t builtin_band(const char* b, const char* e) {
    for (uint8_t i = 0; i < BAND_FIXED; i++)
        if (equals(b, e, kBands[i].name)) return i;
    return BAND_COUNT;
}

/* Fields of a "key=value,..." remainder, one at a time. */
struct Fields {
    const char* p;
    const char* end;
    const char *kb, *ke, *vb, *ve;
    bool has_eq;

    bool next() {
        while (p < end) {
            const char* f = p;
            while (p < end && *p != ',') p++;
            const char* fe = p;
            if (p < end) p++;
            trim(f, fe);
            if (f == fe) continue;
            const char* eq = f;
            while (eq < fe && *eq != '=') eq++;
            has_eq = eq < fe;
            kb = f;
            ke = eq;
            vb = has_eq ? eq + 1 : fe;
            ve = fe;
            trim(kb, ke);
            trim(vb, ve);
            return true;
        }
        return false;
    }
    bool key(const char* k) const { return equals(kb, ke, k); }
};

bool pattern_hit(const SigBytes& p, const uint8_t* d, size_t n) {
    if (n < p.n) return false;
    for (size_t i = 0; i < p.n; i++)
        if ((d[i] & p.m[i]) != p.v[i]) return false;
    return true;
}

}  // namespace

const char* sig_kind_name(uint8_t kind) {
    return kKindNames[kind < SK_COUNT ? kind : 0];
}

const char* sig_kind_tag(uint8_t kind) {
    return kKindTags[kind < SK_COUNT ? kind : 0];
}

const char* sig_valid_name(uint8_t valid) {
    return kValidNames[valid < 3 ? valid : 0];
}

/* ---------------------------------------------------------------- parser */

void SigParser::begin(SigClass& cls, PatternTable* pat) {
    char name[kSigNameLen];
    memcpy(name, cls.name, sizeof(name));
    cls = SigClass{};
    memcpy(cls.name, name, sizeof(name));
    cls_ = &cls;
    pat_ = pat;
    if (pat_) pat_->begin_file(false);
    len_ = 0;
    long_ = false;
}

void SigParser::reject(uint16_t line, const char* why) {
    SigReport& r = cls_->report;
    if (r.bad < 255) r.bad++;
    if (!r.bad_line || line < r.bad_line) {
        r.bad_line = line;
        const size_t n = strlen(why);
        const size_t m = n < sizeof(r.why) - 1 ? n : sizeof(r.why) - 1;
        memcpy(r.why, why, m);
        r.why[m] = 0;
    }
}

const char* SigParser::parse_line(const char* line, size_t len) {
    SigClass& c = *cls_;
    const char* p = line;
    const char* end = line + len;
    const char* tb = p;
    while (p < end && *p != ',') p++;
    const char* te = p;
    trim(tb, te);
    if (p < end) p++;
    Fields f{p, end, nullptr, nullptr, nullptr, nullptr, false};

    if (equals(tb, te, "BLE")) {
        if (c.nble >= kSigMaxBle) return "over 20 BLE rules";
        BleRule r{};
        r.valid = SV_DOC;
        bool label = false;
        uint32_t v = 0;
        while (f.next()) {
            if (!f.has_eq) return "no '=' in a field";
            if (f.key("label")) {
                if (!copy_label(f.vb, f.ve, r.label, sizeof(r.label))) return "label length";
                label = true;
            } else if (f.key("kind")) {
                if (!parse_kind(f.vb, f.ve, r.kind)) return "unknown kind";
            } else if (f.key("valid")) {
                if (!parse_valid(f.vb, f.ve, r.valid)) return "valid: real/standin/doc";
            } else if (f.key("cid")) {
                if (!parse_hex16(f.vb, f.ve, r.cid)) return "cid: 4 hex digits";
                r.flags |= BR_CID | BR_MFR;
            } else if (f.key("uuid") || f.key("sdata")) {
                if (!parse_hex16(f.vb, f.ve, r.uuid)) return "uuid: 4 hex digits";
                r.flags |= f.key("uuid") ? BR_UUID : BR_SDATA;
            } else if (f.key("mfr") || f.key("sd")) {
                if (r.data.n || (r.flags & BR_NAME)) return "one of mfr/sd/name";
                if (!parse_pattern(f.vb, f.ve, r.data)) return "bad pattern";
                if (f.key("mfr")) r.flags |= BR_MFR;
            } else if (f.key("name")) {
                if (r.data.n) return "one of mfr/sd/name";
                const size_t n = f.ve - f.vb;
                if (n == 0 || n > kSigPatBytes) return "name: 1..8 chars";
                r.data.n = static_cast<uint8_t>(n);
                for (size_t i = 0; i < n; i++) {
                    r.data.v[i] = static_cast<uint8_t>(f.vb[i]);
                    r.data.m[i] = 0xFF;
                }
                r.flags |= BR_NAME;
            } else if (f.key("not")) {
                const char* q = f.vb;
                while (q < f.ve) {
                    const char* qb = q;
                    while (q < f.ve && *q != '/') q++;
                    const char* qe = q;
                    if (q < f.ve) q++;
                    trim(qb, qe);
                    if (r.nnot >= 2) return "not: at most 2";
                    if (!parse_pattern(qb, qe, r.nots[r.nnot])) return "bad not pattern";
                    r.nnot++;
                }
            } else if (f.key("minlen")) {
                if (!parse_uint(f.vb, f.ve, 31, v)) return "minlen not 0..31";
                r.minlen = static_cast<uint8_t>(v);
            } else if (f.key("addr")) {
                if (equals(f.vb, f.ve, "public"))
                    r.flags |= BR_PUBLIC;
                else if (equals(f.vb, f.ve, "random"))
                    r.flags |= BR_RANDOM;
                else
                    return "addr: public/random";
            } else if (f.key("beep")) {
                if (!parse_uint(f.vb, f.ve, 1, v)) return "beep not 0/1";
                if (v) r.flags |= BR_BEEP;
            } else {
                return "unknown key";
            }
        }
        if (!label) return "no label";
        if (!(r.flags & (BR_CID | BR_UUID | BR_SDATA | BR_NAME))) return "no cid/uuid/sdata/name";
        if (r.data.n && !(r.flags & (BR_MFR | BR_NAME | BR_SDATA))) return "sd needs sdata";
        if ((r.flags & BR_SDATA) && (r.flags & BR_MFR) && r.data.n) return "mfr with sdata";
        c.ble[c.nble++] = r;
        c.rx |= RX_BLE;
        c.report.ble++;
        return nullptr;
    }

    if (equals(tb, te, "NB")) {
        if (c.nnb >= kSigMaxNb) return "over 10 NB rules";
        NbRule r{};
        r.valid = SV_DOC;
        r.min_db = 8;
        r.duty_hi = 100;
        r.band = BAND_COUNT;
        bool label = false, freq = false, bw = false;
        uint32_t v = 0, lo = 0, hi = 0;
        while (f.next()) {
            if (!f.has_eq) return "no '=' in a field";
            if (f.key("label")) {
                if (!copy_label(f.vb, f.ve, r.label, sizeof(r.label))) return "label length";
                label = true;
            } else if (f.key("kind")) {
                if (!parse_kind(f.vb, f.ve, r.kind)) return "unknown kind";
            } else if (f.key("valid")) {
                if (!parse_valid(f.vb, f.ve, r.valid)) return "valid: real/standin/doc";
            } else if (f.key("f")) {
                if (!parse_mhz(f.vb, f.ve, r.f_khz)) return "bad f";
                freq = true;
            } else if (f.key("bw")) {
                if (!parse_uint(f.vb, f.ve, 20000, v) || v == 0) return "bw not 1..20000";
                r.bw_khz = static_cast<uint16_t>(v);
                bw = true;
            } else if (f.key("db")) {
                if (!parse_uint(f.vb, f.ve, 60, v) || v < 3) return "db not 3..60";
                r.min_db = static_cast<uint8_t>(v);
            } else if (f.key("duty")) {
                if (!parse_range(f.vb, f.ve, 100, lo, hi)) return "duty: lo-hi %";
                r.duty_lo = static_cast<uint8_t>(lo);
                r.duty_hi = static_cast<uint8_t>(hi);
            } else if (f.key("burst")) {
                if (!parse_range(f.vb, f.ve, 60000, lo, hi) || !hi) return "burst: lo-hi ms";
                r.burst_lo = static_cast<uint16_t>(lo);
                r.burst_hi = static_cast<uint16_t>(hi);
            } else if (f.key("period")) {
                if (!parse_range(f.vb, f.ve, 60000, lo, hi) || !hi) return "period: lo-hi ms";
                r.period_lo = static_cast<uint16_t>(lo);
                r.period_hi = static_cast<uint16_t>(hi);
            } else if (f.key("band")) {
                r.band = builtin_band(f.vb, f.ve);
                if (r.band == BAND_COUNT) {
                    for (size_t i = 0; i < c.nbands; i++)
                        if (equals(f.vb, f.ve, c.bands[i].name)) r.band = static_cast<uint8_t>(BAND_X0 + i);
                }
                if (r.band == BAND_COUNT) return "band: unknown (BAND first)";
            } else {
                return "unknown key";
            }
        }
        if (!label) return "no label";
        if (!freq) return "no f";
        if (!bw) return "no bw";
        r.line = c.report.lines;
        c.nb[c.nnb++] = r;
        return nullptr;
    }

    if (equals(tb, te, "BAND")) {
        SigBand b{0, 0, 320, 25, {}};
        const char *nb = nullptr, *ne = nullptr;
        bool custom = false;
        uint32_t v = 0;
        while (f.next()) {
            if (!f.has_eq) return "no '=' in a field";
            if (f.key("name")) {
                nb = f.vb;
                ne = f.ve;
            } else if (f.key("lo") || f.key("hi")) {
                if (!parse_uint(f.vb, f.ve, 7250, v) || v < 1) return "lo/hi: MHz 1..7250";
                (f.key("lo") ? b.lo_mhz : b.hi_mhz) = static_cast<uint16_t>(v);
                custom = true;
            } else if (f.key("points")) {
                if (!parse_uint(f.vb, f.ve, RfEngine::kMaxPoints, v) || v < 64) return "points not 64..320";
                b.points = static_cast<uint16_t>(v);
            } else if (f.key("dwell")) {
                if (!parse_uint(f.vb, f.ve, 500, v) || v < 50) return "dwell not 50..500";
                b.dwell_cs = static_cast<uint8_t>(v / 10);
            } else {
                return "unknown key";
            }
        }
        if (!nb) return "no name";
        if (!custom) {
            const uint8_t id = builtin_band(nb, ne);
            if (id == BAND_COUNT) return "unknown band";
            c.band_bits |= static_cast<uint16_t>(1u << id);
            c.report.bands++;
            return nullptr;
        }
        if (builtin_band(nb, ne) != BAND_COUNT) return "custom: own name";
        if (ne - nb > 3) return "name: 1..3 chars";
        if (!b.lo_mhz || b.hi_mhz <= b.lo_mhz || b.hi_mhz - b.lo_mhz > 100) return "span 1..100 MHz";
        if (c.nbands >= kSigMaxBands) return "over 2 custom bands";
        memcpy(b.name, nb, ne - nb);
        b.name[ne - nb] = 0;
        c.bands[c.nbands] = b;
        c.band_bits |= static_cast<uint16_t>(1u << (BAND_X0 + c.nbands));
        c.nbands++;
        c.report.bands++;
        return nullptr;
    }

    if (equals(tb, te, "PAT")) {
        if (!pat_) return "no pattern table";
        /* The PATTERNS.TXT line without the valid= field. */
        char buf[kSigLineMax + 1];
        size_t n = 0;
        while (f.next()) {
            if (f.key("valid")) continue;
            const size_t k = f.ve - f.kb;
            if (n + k + 1 >= sizeof(buf)) return "line too long";
            if (n) buf[n++] = ',';
            memcpy(buf + n, f.kb, k);
            n += k;
        }
        PatternEntry e;
        const char* why = PatternTable::parse_line(buf, n, e);
        if (why) return why;
        if (!pat_->add(e)) return "over 16 PAT rules";
        c.rx |= RX_PAT;
        c.report.pat++;
        return nullptr;
    }

    if (equals(tb, te, "ZW")) {
        if (c.rx & RX_ZW) return "one ZW rule a class";
        ZwRule r{};
        r.f_khz = 908420;
        r.kind = SK_HOME;
        r.valid = SV_DOC;
        bool label = false;
        while (f.next()) {
            if (!f.has_eq) return "no '=' in a field";
            if (f.key("label")) {
                if (!copy_label(f.vb, f.ve, r.label, sizeof(r.label))) return "label length";
                label = true;
            } else if (f.key("kind")) {
                if (!parse_kind(f.vb, f.ve, r.kind)) return "unknown kind";
            } else if (f.key("valid")) {
                if (!parse_valid(f.vb, f.ve, r.valid)) return "valid: real/standin/doc";
            } else if (f.key("f")) {
                if (!parse_mhz(f.vb, f.ve, r.f_khz)) return "bad f";
            } else if (f.key("rate")) {
                if (!equals(f.vb, f.ve, "9k6")) return "rate: 9k6 only";
            } else {
                return "unknown key";
            }
        }
        if (!label) return "no label";
        c.zw = r;
        c.rx |= RX_ZW;
        c.report.zw++;
        return nullptr;
    }

    if (equals(tb, te, "RX")) {
        while (f.next()) {
            if (f.has_eq) return "RX: words only";
            if (f.key("ble"))
                c.rx |= RX_BLE;
            else if (f.key("wifi"))
                c.rx |= RX_WIFI;
            else if (f.key("droneid"))
                c.rx |= RX_DID;
            else if (f.key("rid"))
                c.rx |= RX_RID;
            else if (f.key("dji"))
                c.rx |= RX_DJI;
            else if (f.key("rfcls"))
                c.rx |= RX_RFCLS;
            else
                return "RX: ble/wifi/droneid/rid/dji/rfcls";
        }
        return nullptr;
    }

    return "unknown rule type";
}

void SigParser::end_line() {
    SigReport& r = cls_->report;
    const bool long_line = long_;
    size_t n = len_;
    len_ = 0;
    long_ = false;
    if (r.truncated) return;
    if (r.lines >= kSigMaxLines) {
        r.truncated = true;
        return;
    }
    r.lines++;
    if (long_line) {
        reject(r.lines, "line too long");
        return;
    }
    if (n && line_[n - 1] == '\r') n--;
    const char* b = line_;
    const char* e = line_ + n;
    trim(b, e);
    if (b == e || *b == '#') return;
    const char* why = parse_line(b, e - b);
    if (why) reject(r.lines, why);
}

void SigParser::feed(const char* data, size_t n) {
    for (size_t i = 0; i < n; i++) {
        const char c = data[i];
        if (c == '\n')
            end_line();
        else if (len_ < kSigLineMax)
            line_[len_++] = c;
        else
            long_ = true;
    }
}

/* NB rules get their band (the first swept band that holds f, else the
 * first built-in one, which the class then sweeps); a rule no band holds is
 * dropped. */
void SigParser::finish() {
    if (len_ || long_) end_line();
    SigClass& c = *cls_;
    size_t keep = 0;
    for (size_t i = 0; i < c.nnb; i++) {
        NbRule r = c.nb[i];
        auto holds = [&](uint8_t b) {
            const uint32_t lo = b >= BAND_X0 ? c.bands[b - BAND_X0].lo_mhz : kBands[b].lo_mhz;
            const uint32_t hi = b >= BAND_X0 ? c.bands[b - BAND_X0].hi_mhz : kBands[b].hi_mhz;
            return r.f_khz >= lo * 1000 && r.f_khz <= hi * 1000;
        };
        if (r.band == BAND_COUNT) {
            for (uint8_t b = 0; b < BAND_X0 + c.nbands && r.band == BAND_COUNT; b++) {
                if (b >= BAND_FIXED && b < BAND_X0) continue;
                if ((c.band_bits & (1u << b)) && holds(b)) r.band = b;
            }
            for (uint8_t b = 0; b < BAND_FIXED && r.band == BAND_COUNT; b++)
                if (holds(b)) r.band = b;
        }
        if (r.band == BAND_COUNT || !holds(r.band)) {
            reject(r.line, "no band holds f");
            continue;
        }
        c.band_bits |= static_cast<uint16_t>(1u << r.band);
        c.nb[keep++] = r;
        c.report.nb++;
    }
    c.nnb = static_cast<uint8_t>(keep);
}

/* ---------------------------------------------------------------- matching */

void adv_parse(const uint8_t* adv, size_t len, AdvInfo& a) {
    a = AdvInfo{};
    size_t i = 0;
    while (i + 1 < len) {
        const size_t l = adv[i];
        if (l == 0 || i + 1 + l > len) break;
        const uint8_t t = adv[i + 1];
        const uint8_t* v = adv + i + 2;
        const size_t vl = l - 1;
        if (t == 0xFF && !a.mfr && vl >= 2) {
            a.mfr = v;
            a.mfr_len = static_cast<uint8_t>(vl);
        } else if (t == 0x02 || t == 0x03) {
            for (size_t k = 0; k + 1 < vl && a.nuuid < AdvInfo::kMaxUuid; k += 2) a.uuid[a.nuuid++] = static_cast<uint16_t>(v[k] | (v[k + 1] << 8));
        } else if (t == 0x16 && vl >= 2 && a.nsd < AdvInfo::kMaxSd) {
            a.sd_uuid[a.nsd] = static_cast<uint16_t>(v[0] | (v[1] << 8));
            a.sd[a.nsd] = v + 2;
            a.sd_len[a.nsd] = static_cast<uint8_t>(vl - 2);
            a.nsd++;
        } else if ((t == 0x08 || t == 0x09) && !a.name) {
            a.name = v;
            a.name_len = static_cast<uint8_t>(vl);
        }
        i += 1 + l;
    }
}

namespace {

bool field_ok(const BleRule& r, const uint8_t* d, size_t n) {
    if (n < r.minlen) return false;
    if (r.data.n && !pattern_hit(r.data, d, n)) return false;
    for (size_t k = 0; k < r.nnot; k++)
        if (pattern_hit(r.nots[k], d, n)) return false;
    return true;
}

bool rule_hit(const BleRule& r, uint8_t pdu, const AdvInfo& a) {
    const bool random = (pdu & 0x40) != 0;
    if ((r.flags & BR_PUBLIC) && random) return false;
    if ((r.flags & BR_RANDOM) && !random) return false;
    if (r.flags & BR_NAME) {
        if (!a.name || !pattern_hit(r.data, a.name, a.name_len)) return false;
    }
    if (r.flags & BR_MFR) {
        if (!a.mfr) return false;
        if ((r.flags & BR_CID) && static_cast<uint16_t>(a.mfr[0] | (a.mfr[1] << 8)) != r.cid) return false;
        if (!field_ok(r, a.mfr + 2, a.mfr_len - 2u)) return false;
    }
    if (r.flags & BR_SDATA) {
        bool any = false;
        for (size_t k = 0; k < a.nsd && !any; k++)
            any = a.sd_uuid[k] == r.uuid && field_ok(r, a.sd[k], a.sd_len[k]);
        if (!any) return false;
    }
    if (r.flags & BR_UUID) {
        bool any = false;
        for (size_t k = 0; k < a.nuuid && !any; k++) any = a.uuid[k] == r.uuid;
        for (size_t k = 0; k < a.nsd && !any; k++) any = a.sd_uuid[k] == r.uuid;
        if (!any) return false;
    }
    return true;
}

}  // namespace

int sig_ble_match(const SigClass& cls, uint8_t pdu_header, const AdvInfo& a) {
    for (size_t i = 0; i < cls.nble; i++)
        if (rule_hit(cls.ble[i], pdu_header, a)) return static_cast<int>(i);
    return -1;
}

/* ---------------------------------------------------------------- NB rules */

void nb_begin(const SigClass& cls, NbState* st, uint8_t band, const RfEngine& rf) {
    for (size_t i = 0; i < cls.nnb; i++) {
        NbState& s = st[i];
        s = NbState{};
        const NbRule& r = cls.nb[i];
        if (r.band != band || !rf.points()) continue;
        const uint32_t half = r.bw_khz / 2u;
        s.x0 = static_cast<uint16_t>(rf.point_of(r.f_khz > half ? r.f_khz - half : 0));
        s.x1 = static_cast<uint16_t>(rf.point_of(r.f_khz + half));
        s.max_cdb = -32768;
        /* The floor must be known from an earlier dwell. */
        s.active = rf.floor_valid(band);
    }
}

void nb_look(const SigClass& cls, NbState* st, const int16_t* cdb, const RfEngine& rf) {
    for (size_t i = 0; i < cls.nnb; i++) {
        NbState& s = st[i];
        if (!s.active) continue;
        int32_t best = -32768;
        for (size_t x = s.x0; x <= s.x1; x++) {
            const int32_t rel = cdb[x] - rf.floor_at(x);
            if (rel > best) best = rel;
        }
        if (best > s.max_cdb) s.max_cdb = static_cast<int16_t>(best > 32767 ? 32767 : best);
        if (s.n >= NbState::kLooks) continue;
        if (best >= cls.nb[i].min_db * 100) {
            s.bits[s.n >> 3] |= static_cast<uint8_t>(1u << (s.n & 7));
            s.on++;
        }
        s.n++;
    }
}

NbResult nb_eval(const NbRule& r, const NbState& s, uint16_t dwell_ms) {
    NbResult res{};
    res.max_cdb = s.max_cdb;
    if (!s.active || !s.n) return res;
    res.duty = static_cast<uint8_t>(s.on * 100u / s.n);
    /* Runs of looks with energy: whole ones (not touching the dwell's ends)
     * give the burst length, run starts the period. */
    uint32_t whole = 0, whole_len = 0, starts = 0, first = 0, last = 0;
    size_t run = 0;
    for (size_t k = 0; k <= s.n; k++) {
        const bool on = k < s.n && ((s.bits[k >> 3] >> (k & 7)) & 1);
        if (on) {
            if (!run) {
                if (k > 0) {
                    if (!starts) first = k;
                    last = k;
                    starts++;
                }
            }
            run++;
        } else if (run) {
            if (k < s.n && k - run > 0) {
                whole++;
                whole_len += run;
            }
            if (res.runs < 255) res.runs++;
            run = 0;
        }
    }
    const uint32_t look_us = dwell_ms * 1000u / s.n;
    if (whole) res.burst_ms = static_cast<uint16_t>(whole_len * look_us / whole / 1000u);
    if (starts >= 2) res.period_ms = static_cast<uint16_t>((last - first) * look_us / (starts - 1) / 1000u);
    /* Two looks at least: one noise peak over db= is not a signal. */
    if (res.max_cdb < r.min_db * 100 || s.on < 2) return res;
    if (res.duty < r.duty_lo || res.duty > r.duty_hi) return res;
    if (r.burst_hi && (!whole || res.burst_ms < r.burst_lo || res.burst_ms > r.burst_hi)) return res;
    if (r.period_hi && (starts < 2 || res.period_ms < r.period_lo || res.period_ms > r.period_hi)) return res;
    res.match = true;
    return res;
}

} /* namespace ui::external_app::signature_detect */
