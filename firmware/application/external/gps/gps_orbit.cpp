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

#include "gps_orbit.hpp"

#include <cmath>
#include <cstring>

namespace ui::external_app::gps {

namespace {

constexpr double kWgsA = 6378137.0;
constexpr double kWgsF = 1.0 / 298.257223563;
constexpr double kWgsE2 = kWgsF * (2.0 - kWgsF);
constexpr double kDeg = kPi / 180.0;

/* Kepler's equation; returns E. */
double eccentric_anomaly(double m, double e) {
    double ea = m;
    for (int k = 0; k < 10; k++) {
        const double d = (ea - e * std::sin(ea) - m) / (1.0 - e * std::cos(ea));
        ea -= d;
        if (std::fabs(d) < 1e-13) break;
    }
    return ea;
}

/* Time difference t - ref folded to +-half a week (IS-GPS-200 20.3.3.4.3). */
double fold_week(double dt) {
    if (dt > kWeekS / 2) dt -= kWeekS;
    if (dt < -kWeekS / 2) dt += kWeekS;
    return dt;
}

bool starts_with(const char* s, const char* p) {
    return std::strncmp(s, p, std::strlen(p)) == 0;
}

/* Value after the ':' of a YUMA line. */
double yuma_value(const char* s) {
    const char* c = std::strchr(s, ':');
    if (!c) return 0;
    return parse_double(c + 1, std::strlen(c + 1));
}

int field_int(const char* s, size_t from, size_t n) {
    const size_t len = std::strlen(s);
    if (from >= len) return 0;
    if (from + n > len) n = len - from;
    return static_cast<int>(parse_double(s + from, n));
}

}  // namespace

double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

int32_t days_from_civil(int32_t y, int32_t m, int32_t d) {
    y -= m <= 2;
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const int32_t yoe = y - era * 400;
    const int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

void civil_from_days(int32_t z, int32_t& y, int32_t& m, int32_t& d) {
    z += 719468;
    const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
    const int32_t doe = z - era * 146097;
    const int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const int32_t mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = yoe + era * 400 + (m <= 2);
}

double parse_double(const char* s, size_t n) {
    size_t i = 0;
    while (i < n && (s[i] == ' ' || s[i] == '\t')) i++;
    bool neg = false;
    if (i < n && (s[i] == '-' || s[i] == '+')) neg = s[i++] == '-';
    int64_t mant = 0;
    int digits = 0, exp10 = 0;
    bool dot = false;
    for (; i < n; i++) {
        const char c = s[i];
        if (c >= '0' && c <= '9') {
            if (digits < 18) {
                mant = mant * 10 + (c - '0');
                digits += mant != 0;
                if (dot) exp10--;
            } else if (!dot) {
                exp10++;
            }
        } else if (c == '.' && !dot) {
            dot = true;
        } else {
            break;
        }
    }
    if (i < n && (s[i] == 'E' || s[i] == 'e' || s[i] == 'D' || s[i] == 'd')) {
        i++;
        bool eneg = false;
        if (i < n && (s[i] == '-' || s[i] == '+')) eneg = s[i++] == '-';
        int e = 0;
        while (i < n && s[i] >= '0' && s[i] <= '9') e = e * 10 + (s[i++] - '0');
        exp10 += eneg ? -e : e;
    }
    double v = static_cast<double>(mant);
    /* scale by 10^exp10 with exact powers where possible */
    double p = 1.0;
    int a = exp10 < 0 ? -exp10 : exp10;
    double b = 10.0;
    while (a) {
        if (a & 1) p *= b;
        b *= b;
        a >>= 1;
    }
    v = exp10 < 0 ? v / p : v * p;
    return neg ? -v : v;
}

AlmRec to_alm(const Orbit& o) {
    AlmRec a;
    a.sqrt_a = o.sqrt_a;
    a.toe = o.toe;
    a.e = static_cast<float>(o.e);
    a.i0 = static_cast<float>(o.i0);
    a.omega0 = static_cast<float>(o.omega0);
    a.omega = static_cast<float>(o.omega);
    a.m0 = static_cast<float>(o.m0);
    a.omega_dot = o.omega_dot;
    a.af0 = o.af0;
    a.af1 = o.af1;
    a.prn = o.prn;
    a.health = o.health;
    a.valid = o.valid;
    return a;
}

Orbit from_alm(const AlmRec& a) {
    Orbit o;
    o.sqrt_a = a.sqrt_a;
    o.toe = o.toc = a.toe;
    o.e = a.e;
    o.i0 = a.i0;
    o.omega0 = a.omega0;
    o.omega = a.omega;
    o.m0 = a.m0;
    o.omega_dot = a.omega_dot;
    o.af0 = a.af0;
    o.af1 = a.af1;
    o.prn = a.prn;
    o.health = a.health;
    o.ephemeris = 0;
    o.valid = a.valid;
    return o;
}

Vec3 sat_position(const Orbit& o, double t, double* clock_s) {
    const double a = o.sqrt_a * o.sqrt_a;
    const double n = std::sqrt((o.ephemeris == 2 ? kMuGal : kMu) / (a * a * a)) + o.dn;
    /* toe is continuous GPS time: fold only an ephemeris (valid for hours; its
     * toc/toe weeks may differ at a week boundary). An almanac is used for
     * weeks or months: folding to +-3.5 days would put the satellite ~14 deg
     * off along its orbit for every whole week of age. */
    const double tk = o.ephemeris ? fold_week(t - o.toe) : t - o.toe;
    const double m = o.m0 + n * tk;
    const double ea = eccentric_anomaly(m, o.e);
    const double sin_e = std::sin(ea), cos_e = std::cos(ea);
    const double nu = std::atan2(std::sqrt(1.0 - o.e * o.e) * sin_e, cos_e - o.e);
    const double phi = nu + o.omega;
    double u = phi, r = a * (1.0 - o.e * cos_e), i = o.i0 + o.idot * tk;
    if (o.ephemeris) {
        const double s2 = std::sin(2 * phi), c2 = std::cos(2 * phi);
        u += o.cus * s2 + o.cuc * c2;
        r += o.crs * s2 + o.crc * c2;
        i += o.cis * s2 + o.cic * c2;
    }
    const double xp = r * std::cos(u), yp = r * std::sin(u);
    /* omega0 is the longitude at the start of the week of toe */
    const double toe_sow = std::fmod(o.toe, kWeekS);
    const double om = o.omega0 + (o.omega_dot - kOmegaE) * tk - kOmegaE * toe_sow;
    const double co = std::cos(om), so = std::sin(om), ci = std::cos(i), si = std::sin(i);
    if (clock_s) {
        const double dt = o.ephemeris ? fold_week(t - o.toc) : t - o.toc;
        *clock_s = o.af0 + o.af1 * dt + o.af2 * dt * dt + kFRel * o.e * o.sqrt_a * sin_e - (o.ephemeris ? o.tgd : 0.0f);
    }
    return {xp * co - yp * ci * so, xp * so + yp * ci * co, yp * si};
}

Vec3 sat_velocity(const Orbit& o, double t) {
    const Vec3 a = sat_position(o, t + 0.05);
    const Vec3 b = sat_position(o, t - 0.05);
    return {(a.x - b.x) * 10.0, (a.y - b.y) * 10.0, (a.z - b.z) * 10.0};
}

Vec3 lla_to_ecef(double lat_deg, double lon_deg, double h) {
    const double la = lat_deg * kDeg, lo = lon_deg * kDeg;
    const double sl = std::sin(la), cl = std::cos(la);
    const double nn = kWgsA / std::sqrt(1.0 - kWgsE2 * sl * sl);
    return {(nn + h) * cl * std::cos(lo), (nn + h) * cl * std::sin(lo), (nn * (1.0 - kWgsE2) + h) * sl};
}

void ecef_to_lla(const Vec3& x, double& lat_deg, double& lon_deg, double& h) {
    const double lon = std::atan2(x.y, x.x);
    const double p = std::sqrt(x.x * x.x + x.y * x.y);
    double lat = std::atan2(x.z, p * (1.0 - kWgsE2));
    h = 0;
    for (int k = 0; k < 6; k++) {
        const double sl = std::sin(lat);
        const double nn = kWgsA / std::sqrt(1.0 - kWgsE2 * sl * sl);
        h = p / std::cos(lat) - nn;
        lat = std::atan2(x.z, p * (1.0 - kWgsE2 * nn / (nn + h)));
    }
    lat_deg = lat / kDeg;
    lon_deg = lon / kDeg;
}

Observer make_observer(const Vec3& pos) {
    double lat, lon, h;
    ecef_to_lla(pos, lat, lon, h);
    const double la = lat * kDeg, lo = lon * kDeg;
    Observer o;
    o.pos = pos;
    o.sla = std::sin(la);
    o.cla = std::cos(la);
    o.slo = std::sin(lo);
    o.clo = std::cos(lo);
    return o;
}

void az_el(const Observer& o, const Vec3& sat, double& az_deg, double& el_deg) {
    const double dx = sat.x - o.pos.x, dy = sat.y - o.pos.y, dz = sat.z - o.pos.z;
    const double e = -o.slo * dx + o.clo * dy;
    const double nn = -o.sla * o.clo * dx - o.sla * o.slo * dy + o.cla * dz;
    const double u = o.cla * o.clo * dx + o.cla * o.slo * dy + o.sla * dz;
    az_deg = std::atan2(e, nn) / kDeg;
    if (az_deg < 0) az_deg += 360.0;
    el_deg = std::atan2(u, std::sqrt(e * e + nn * nn)) / kDeg;
}

void az_el(const Vec3& rx, const Vec3& sat, double& az_deg, double& el_deg) {
    az_el(make_observer(rx), sat, az_deg, el_deg);
}

Vec3 earth_rotate(const Vec3& x, double dt) {
    const double a = kOmegaE * dt;
    const double c = std::cos(a), s = std::sin(a);
    return {c * x.x + s * x.y, -s * x.x + c * x.y, x.z};
}

double geometric_range(const Orbit& o, const Vec3& rx, double t_rx, double* t_tx, Vec3* sat) {
    double tau = 0.075, rho = 0;
    Vec3 s{};
    for (int k = 0; k < 4; k++) {
        s = earth_rotate(sat_position(o, t_rx - tau), tau);
        const Vec3 d{s.x - rx.x, s.y - rx.y, s.z - rx.z};
        rho = norm(d);
        tau = rho / kC;
    }
    if (t_tx) *t_tx = t_rx - tau;
    if (sat) *sat = s;
    return rho;
}

bool YumaParser::line(const char* s, Orbit& out) {
    while (*s == ' ' || *s == '\t') s++;
    if (starts_with(s, "ID")) {
        cur_ = Orbit{};
        seen_ = 1;
        cur_.prn = static_cast<uint8_t>(yuma_value(s));
    } else if (!seen_) {
        return false;
    } else if (starts_with(s, "Health")) {
        cur_.health = static_cast<uint8_t>(yuma_value(s));
    } else if (starts_with(s, "Eccentricity")) {
        cur_.e = yuma_value(s);
    } else if (starts_with(s, "Time of Applicability")) {
        toa_ = yuma_value(s);
    } else if (starts_with(s, "Orbital Inclination")) {
        cur_.i0 = yuma_value(s);
    } else if (starts_with(s, "Rate of Right Ascen")) {
        cur_.omega_dot = static_cast<float>(yuma_value(s));
    } else if (starts_with(s, "SQRT(A)")) {
        cur_.sqrt_a = yuma_value(s);
    } else if (starts_with(s, "Right Ascen at Week")) {
        cur_.omega0 = yuma_value(s);
    } else if (starts_with(s, "Argument of Perigee")) {
        cur_.omega = yuma_value(s);
    } else if (starts_with(s, "Mean Anom")) {
        cur_.m0 = yuma_value(s);
    } else if (starts_with(s, "Af0")) {
        cur_.af0 = static_cast<float>(yuma_value(s));
    } else if (starts_with(s, "Af1")) {
        cur_.af1 = static_cast<float>(yuma_value(s));
    } else if (starts_with(s, "week")) {
        int32_t wk = static_cast<int32_t>(yuma_value(s));
        if (wk < 1024 && week_hint_ > 0) { /* 10-bit week: the full week nearest the hint */
            const int32_t base = week_hint_ - (week_hint_ % 1024);
            wk += base;
            if (wk - week_hint_ > 512) wk -= 1024;
            if (week_hint_ - wk > 512) wk += 1024;
        }
        cur_.toe = wk * kWeekS + toa_;
        cur_.toc = cur_.toe;
        cur_.ephemeris = 0;
        cur_.valid = cur_.prn >= 1 && cur_.prn <= 32 && cur_.sqrt_a > 4000.0;
        seen_ = 0;
        out = cur_;
        return out.valid;
    }
    return false;
}

void RinexNavParser::finish(Orbit& out) {
    Orbit& o = cur_;
    o.af0 = static_cast<float>(v_[0]);
    o.af1 = static_cast<float>(v_[1]);
    o.af2 = static_cast<float>(v_[2]);
    o.crs = static_cast<float>(v_[4]);
    o.dn = static_cast<float>(v_[5]);
    o.m0 = v_[6];
    o.cuc = static_cast<float>(v_[7]);
    o.e = v_[8];
    o.cus = static_cast<float>(v_[9]);
    o.sqrt_a = v_[10];
    o.cic = static_cast<float>(v_[12]);
    o.omega0 = v_[13];
    o.cis = static_cast<float>(v_[14]);
    o.i0 = v_[15];
    o.crc = static_cast<float>(v_[16]);
    o.omega = v_[17];
    o.omega_dot = static_cast<float>(v_[18]);
    o.idot = static_cast<float>(v_[19]);
    const double week = v_[21];
    o.toe = week * kWeekS + v_[11];
    o.health = static_cast<uint8_t>(v_[24] != 0.0);
    o.tgd = static_cast<float>(v_[25]);
    o.ephemeris = 1;
    /* toc came from the epoch (calendar GPS time); keep toe's week frame */
    o.valid = o.prn >= 1 && o.prn <= 32 && o.sqrt_a > 4000.0 && week > 0;
    out = o;
}

bool RinexNavParser::line(const char* s, Orbit& out) {
    if (!header_done_) {
        if (std::strstr(s, "RINEX VERSION")) version_ = static_cast<int>(parse_double(s, 9));
        if (std::strstr(s, "END OF HEADER")) header_done_ = true;
        return false;
    }
    if (skip_ > 0) {
        skip_--;
        return false;
    }
    const size_t len = std::strlen(s);
    if (row_ < 0) {
        if (len < 22) return false;
        size_t vcol;
        int y, mo, d, h, mi;
        double sec;
        if (version_ >= 3) {
            const char sys = s[0];
            if (sys != 'G') {
                /* GLONASS and SBAS records have 4 lines, the others 8 */
                skip_ = (sys == 'R' || sys == 'S') ? 3 : 7;
                return false;
            }
            cur_ = Orbit{};
            cur_.prn = static_cast<uint8_t>(field_int(s, 1, 2));
            y = field_int(s, 4, 4);
            mo = field_int(s, 9, 2);
            d = field_int(s, 12, 2);
            h = field_int(s, 15, 2);
            mi = field_int(s, 18, 2);
            sec = parse_double(s + 21, 2);
            vcol = 23;
        } else {
            cur_ = Orbit{};
            cur_.prn = static_cast<uint8_t>(field_int(s, 0, 2));
            y = field_int(s, 3, 2);
            y += y < 80 ? 2000 : 1900;
            mo = field_int(s, 6, 2);
            d = field_int(s, 9, 2);
            h = field_int(s, 12, 2);
            mi = field_int(s, 15, 2);
            sec = parse_double(s + 17, 5);
            vcol = 22;
        }
        const double unix_s = days_from_civil(y, mo, d) * 86400.0 + h * 3600.0 + mi * 60.0 + sec;
        cur_.toc = unix_s - kUnixToGps; /* epochs are GPS time: no leap seconds */
        for (int k = 0; k < 3; k++)
            v_[k] = vcol + 19 * k < len ? parse_double(s + vcol + 19 * k, 19) : 0.0;
        row_ = 1;
        return false;
    }
    const size_t col = version_ >= 3 ? 4 : 3;
    for (int k = 0; k < 4; k++) {
        const size_t c = col + 19 * k;
        v_[3 + (row_ - 1) * 4 + k] = c < len ? parse_double(s + c, len - c < 19 ? len - c : 19) : 0.0;
    }
    if (++row_ == 8) {
        row_ = -1;
        finish(out);
        return out.valid;
    }
    return false;
}

void keep_best(Orbit* table, const Orbit& o, double t) {
    if (!o.valid || o.prn < 1 || o.prn > 32) return;
    Orbit& slot = table[o.prn - 1];
    if (!slot.valid) {
        slot = o;
        return;
    }
    if (slot.health != 0 && o.health == 0) {
        slot = o;
        return;
    }
    if (o.health != 0 && slot.health == 0) return;
    if (std::fabs(o.toe - t) < std::fabs(slot.toe - t)) slot = o;
}

}  // namespace ui::external_app::gps
