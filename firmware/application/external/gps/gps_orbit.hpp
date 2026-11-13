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

#ifndef __GPS_ORBIT_H__
#define __GPS_ORBIT_H__

/* GPS orbits from an almanac (YUMA text) or a broadcast ephemeris (RINEX 2/3
 * navigation file), geometry helpers (ECEF, geodetic, azimuth/elevation) and
 * GPS time. The orbit equations are the user algorithm of IS-GPS-200 (table
 * 20-IV), written from the specification. All doubles: on the M0 they are soft
 * floats, used a few hundred times per screen update at most.
 *
 * No heap, no file access: the parsers take one text line at a time (the
 * caller reads the SD card), so they can be host tested. */

#include <cstddef>
#include <cstdint>

namespace ui::external_app::gps {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMu = 3.986005e14;         /* WGS84 GM for GPS, m^3/s^2 */
constexpr double kOmegaE = 7.2921151467e-5; /* earth rotation, rad/s */
constexpr double kC = 299792458.0;          /* m/s */
constexpr double kFRel = -4.442807633e-10;  /* relativistic clock term, s/m^0.5 */
constexpr double kWeekS = 604800.0;
constexpr int32_t kUnixToGps = 315964800; /* 1980-01-06 - 1970-01-01, s */
constexpr int32_t kLeapSeconds = 18;      /* GPS - UTC since 2017-01-01 */

struct Vec3 {
    double x, y, z;
};

/* One satellite's Kepler elements. An almanac fills the first block only;
 * an ephemeris adds the harmonic corrections and the clock polynomial. */
struct Orbit {
    double sqrt_a{0}; /* m^0.5 */
    double e{0};
    double i0{0};       /* rad */
    double omega0{0};   /* longitude of the ascending node at weekly epoch, rad */
    double omega{0};    /* argument of perigee, rad */
    double m0{0};       /* mean anomaly at toe, rad */
    double toe{0};      /* reference time, continuous GPS seconds (week * 604800 + sow) */
    double toc{0};      /* clock reference time, continuous GPS seconds */
    float omega_dot{0}; /* rad/s */
    float idot{0};      /* rad/s */
    float dn{0};        /* mean motion correction, rad/s */
    float cuc{0}, cus{0}, crc{0}, crs{0}, cic{0}, cis{0};
    float af0{0}, af1{0}, af2{0}, tgd{0};
    uint8_t prn{0};
    uint8_t health{0};    /* 0 = healthy */
    uint8_t ephemeris{0}; /* 1: from a broadcast ephemeris (corrections valid) */
    uint8_t valid{0};
};

/* An almanac record kept beside the orbit table (56 bytes instead of 120), so
 * a satellite whose ephemeris has gone stale falls back to its almanac.
 * sqrt_a and toe stay double (a float sqrt_a would add ~3 km of along-track
 * error after 30 days); the angles as float are within ~6 m. */
struct AlmRec {
    double sqrt_a{0};
    double toe{0}; /* time of applicability, continuous GPS s */
    float e{0}, i0{0}, omega0{0}, omega{0}, m0{0}, omega_dot{0}, af0{0}, af1{0};
    uint8_t prn{0};
    uint8_t health{0};
    uint8_t valid{0};
};
AlmRec to_alm(const Orbit& o);
Orbit from_alm(const AlmRec& a);

/* Satellite position (ECEF at time t, metres) and clock offset (s, incl.
 * the relativistic term; minus TGD for ephemerides). t: continuous GPS s. */
Vec3 sat_position(const Orbit& o, double t, double* clock_s = nullptr);
Vec3 sat_velocity(const Orbit& o, double t);

/* Geometry. */
Vec3 lla_to_ecef(double lat_deg, double lon_deg, double h_m);
void ecef_to_lla(const Vec3& x, double& lat_deg, double& lon_deg, double& h_m);
/* Azimuth (0 = north, clockwise) and elevation, degrees. */
void az_el(const Vec3& rx, const Vec3& sat, double& az_deg, double& el_deg);
/* An observer with its local east/north/up basis precomputed: az/el without
 * the geodetic iteration per call (the pass search calls it thousands of
 * times; on the M0 that iteration was most of the cost). */
struct Observer {
    Vec3 pos{};
    double sla{0}, cla{1}, slo{0}, clo{1};
};
Observer make_observer(const Vec3& pos);
void az_el(const Observer& o, const Vec3& sat, double& az_deg, double& el_deg);
/* Rotate an ECEF position by the earth's rotation during dt seconds (Sagnac). */
Vec3 earth_rotate(const Vec3& x, double dt);
/* Light-time solution: range (m), transmit time and the satellite position
 * in the receive-time frame. */
double geometric_range(const Orbit& o, const Vec3& rx, double t_rx, double* t_tx = nullptr, Vec3* sat = nullptr);
double norm(const Vec3& v);

/* Time: UTC (unix seconds) <-> continuous GPS seconds. */
inline double unix_to_gps(double unix_s) {
    return unix_s - kUnixToGps + kLeapSeconds;
}
inline double gps_to_unix(double gps_s) {
    return gps_s + kUnixToGps - kLeapSeconds;
}
/* Days since 1970-01-01 for a civil date (proleptic Gregorian). */
int32_t days_from_civil(int32_t y, int32_t m, int32_t d);
void civil_from_days(int32_t days, int32_t& y, int32_t& m, int32_t& d);

/* Fixed-format number parsing (RINEX "D" exponents, leading blanks). */
double parse_double(const char* s, size_t n);

/* YUMA almanac: feed every line; a satellite is complete at its "week" line
 * (the last field of a record). week_hint: the current full GPS week, used to
 * resolve the 10-bit week of older files. Returns true when `out` was filled. */
class YumaParser {
   public:
    explicit YumaParser(int32_t week_hint)
        : week_hint_{week_hint} {}
    void reset(int32_t week_hint) {
        week_hint_ = week_hint;
        seen_ = 0;
    }
    bool line(const char* s, Orbit& out);

   private:
    Orbit cur_{};
    int32_t week_hint_;
    double toa_{0};
    uint32_t seen_{0};
};

/* RINEX navigation file, versions 2.x and 3.x, GPS records only (other
 * systems' records are skipped). Feed every line; returns true when a GPS
 * record (8 lines) is complete in `out`. */
class RinexNavParser {
   public:
    void reset() {
        version_ = 0;
        row_ = -1;
        skip_ = 0;
        header_done_ = false;
    }
    bool line(const char* s, Orbit& out);
    int version() const { return version_; }

   private:
    double v_[32]{};
    Orbit cur_{};
    int version_{0};
    int8_t row_{-1}; /* -1: header / skipping, 0..7 inside a GPS record */
    int8_t skip_{0}; /* lines left of a non-GPS record */
    bool header_done_{false};
    void finish(Orbit& out);
};

/* Keep the best record per PRN: healthy, toe closest to `t` (continuous GPS
 * seconds). table has 32 slots indexed by PRN - 1. */
void keep_best(Orbit* table, const Orbit& o, double t);

}  // namespace ui::external_app::gps

#endif /*__GPS_ORBIT_H__*/
