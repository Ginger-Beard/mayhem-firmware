/*
 * Near-earth SGP4 (see gps_sgp4.hpp for the origin and the MIT notice of the
 * code this follows: Vallado et al. 2006, via python-sgp4 by Brandon Rhodes).
 */

#include "gps_sgp4.hpp"

#include <cmath>
#include <cstring>

namespace ui::external_app::gps {

namespace {

/* WGS-72 constants (as the reference test vectors use). */
constexpr double kMuKm = 398600.8;
constexpr double kRe = 6378.135; /* km */
constexpr double kJ2 = 0.001082616;
constexpr double kJ3 = -0.00000253881;
constexpr double kJ4 = -0.00000165597;
constexpr double kJ3oJ2 = kJ3 / kJ2;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kX2o3 = 2.0 / 3.0;

double xke() {
    return 60.0 / std::sqrt(kRe * kRe * kRe / kMuKm);
}

/* Fixed-column field of a TLE line. */
double tle_field(const char* l, size_t from, size_t n) {
    return parse_double(l + from, n);
}

/* "12345-5" style (implied decimal point, exponent): 0.12345e-5 */
double tle_exp_field(const char* l, size_t from) {
    char m[8];
    size_t k = 0;
    const char sign = l[from];
    for (size_t i = from + 1; i < from + 6; i++) m[k++] = l[i];
    m[k] = 0;
    const double mant = parse_double(m, k);
    const double e = parse_double(l + from + 6, 2);
    double v = mant * 1e-5 * std::pow(10.0, e);
    return sign == '-' ? -v : v;
}

bool checksum_ok(const char* l) {
    int sum = 0;
    for (int i = 0; i < 68; i++) {
        const char c = l[i];
        if (c >= '0' && c <= '9') sum += c - '0';
        if (c == '-') sum += 1;
    }
    return l[68] >= '0' && l[68] <= '9' && (sum % 10) == (l[68] - '0');
}

double fmod2p(double x) {
    double r = std::fmod(x, kTwoPi);
    if (r < 0) r += kTwoPi;
    return r;
}

}  // namespace

double gmst(double jd_ut1) {
    const double tut1 = (jd_ut1 - 2451545.0) / 36525.0;
    double temp = -6.2e-6 * tut1 * tut1 * tut1 + 0.093104 * tut1 * tut1 +
                  (876600.0 * 3600 + 8640184.812866) * tut1 + 67310.54841;
    temp = std::fmod(temp * kDeg2Rad / 240.0, kTwoPi);
    if (temp < 0.0) temp += kTwoPi;
    return temp;
}

Vec3 teme_to_ecef(const double r[3], double jd) {
    const double g = gmst(jd);
    const double c = std::cos(g), s = std::sin(g);
    return {(c * r[0] + s * r[1]) * 1000.0, (-s * r[0] + c * r[1]) * 1000.0, r[2] * 1000.0};
}

bool parse_tle(const char* l1, const char* l2, Tle& t) {
    t = Tle{};
    if (std::strlen(l1) < 69 || std::strlen(l2) < 69) return false;
    if (l1[0] != '1' || l2[0] != '2') return false;
    if (!checksum_ok(l1) || !checksum_ok(l2)) return false;
    t.catnr = static_cast<uint32_t>(tle_field(l1, 2, 5));
    int yy = static_cast<int>(tle_field(l1, 18, 2));
    const int year = yy < 57 ? 2000 + yy : 1900 + yy;
    const double days = tle_field(l1, 20, 12); /* day of year, 1.0 = Jan 1 0h */
    t.jd_epoch = unix_to_jd(days_from_civil(year, 1, 1) * 86400.0) + days - 1.0;
    const double xpdotp = 1440.0 / kTwoPi;
    double ndot = tle_field(l1, 33, 10);
    t.nddot = tle_exp_field(l1, 44);
    t.bstar = tle_exp_field(l1, 53);
    t.inclo = tle_field(l2, 8, 8) * kDeg2Rad;
    t.nodeo = tle_field(l2, 17, 8) * kDeg2Rad;
    char ecc[9] = "0.";
    for (int i = 0; i < 7; i++) ecc[2 + i] = l2[26 + i] == ' ' ? '0' : l2[26 + i];
    t.ecco = parse_double(ecc, 9);
    t.argpo = tle_field(l2, 34, 8) * kDeg2Rad;
    t.mo = tle_field(l2, 43, 8) * kDeg2Rad;
    t.no_kozai = tle_field(l2, 52, 11) / xpdotp;
    t.ndot = ndot / (xpdotp * 1440.0);
    t.nddot = t.nddot / (xpdotp * 1440.0 * 1440.0);
    t.valid = t.no_kozai > 0;
    return t.valid;
}

bool sgp4_init(const Tle& tle, Sgp4& s) {
    std::memset(&s, 0, sizeof(s));
    const double k = xke();
    const double temp4 = 1.5e-12;
    s.bstar = tle.bstar;
    s.ecco = tle.ecco;
    s.argpo = tle.argpo;
    s.inclo = tle.inclo;
    s.mo = tle.mo;
    s.nodeo = tle.nodeo;
    s.jd_epoch = tle.jd_epoch;

    const double ss = 78.0 / kRe + 1.0;
    const double qzms2ttemp = (120.0 - 78.0) / kRe;
    const double qzms2t = qzms2ttemp * qzms2ttemp * qzms2ttemp * qzms2ttemp;

    /* initl */
    const double eccsq = s.ecco * s.ecco;
    const double omeosq = 1.0 - eccsq;
    const double rteosq = std::sqrt(omeosq);
    const double cosio = std::cos(s.inclo);
    const double cosio2 = cosio * cosio;
    const double ak = std::pow(k / tle.no_kozai, kX2o3);
    const double d1 = 0.75 * kJ2 * (3.0 * cosio2 - 1.0) / (rteosq * omeosq);
    double del = d1 / (ak * ak);
    const double adel = ak * (1.0 - del * del - del * (1.0 / 3.0 + 134.0 * del * del / 81.0));
    del = d1 / (adel * adel);
    s.no_unkozai = tle.no_kozai / (1.0 + del);
    const double ao = std::pow(k / s.no_unkozai, kX2o3);
    const double sinio = std::sin(s.inclo);
    const double po = ao * omeosq;
    const double con42 = 1.0 - 5.0 * cosio2;
    s.con41 = -con42 - cosio2 - cosio2;
    const double posq = po * po;
    const double rp = ao * (1.0 - s.ecco);

    if (kTwoPi / s.no_unkozai >= 225.0) {
        s.error = 7; /* deep space: SDP4 not included */
        return false;
    }
    if (omeosq >= 0.0 || s.no_unkozai >= 0.0) {
        s.isimp = rp < 220.0 / kRe + 1.0;
        double sfour = ss, qzms24 = qzms2t;
        const double perige = (rp - 1.0) * kRe;
        if (perige < 156.0) {
            sfour = perige - 78.0;
            if (perige < 98.0) sfour = 20.0;
            const double q = (120.0 - sfour) / kRe;
            qzms24 = q * q * q * q;
            sfour = sfour / kRe + 1.0;
        }
        const double pinvsq = 1.0 / posq;
        const double tsi = 1.0 / (ao - sfour);
        s.eta = ao * s.ecco * tsi;
        const double etasq = s.eta * s.eta;
        const double eeta = s.ecco * s.eta;
        const double psisq = std::fabs(1.0 - etasq);
        const double coef = qzms24 * std::pow(tsi, 4.0);
        const double coef1 = coef / std::pow(psisq, 3.5);
        const double cc2 = coef1 * s.no_unkozai *
                           (ao * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) +
                            0.375 * kJ2 * tsi / psisq * s.con41 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
        s.cc1 = s.bstar * cc2;
        double cc3 = 0.0;
        if (s.ecco > 1.0e-4) cc3 = -2.0 * coef * tsi * kJ3oJ2 * s.no_unkozai * sinio / s.ecco;
        s.x1mth2 = 1.0 - cosio2;
        s.cc4 = 2.0 * s.no_unkozai * coef1 * ao * omeosq *
                (s.eta * (2.0 + 0.5 * etasq) + s.ecco * (0.5 + 2.0 * etasq) -
                 kJ2 * tsi / (ao * psisq) *
                     (-3.0 * s.con41 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta)) +
                      0.75 * s.x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * std::cos(2.0 * s.argpo)));
        s.cc5 = 2.0 * coef1 * ao * omeosq * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);
        const double cosio4 = cosio2 * cosio2;
        const double temp1 = 1.5 * kJ2 * pinvsq * s.no_unkozai;
        const double temp2 = 0.5 * temp1 * kJ2 * pinvsq;
        const double temp3 = -0.46875 * kJ4 * pinvsq * pinvsq * s.no_unkozai;
        s.mdot = s.no_unkozai + 0.5 * temp1 * rteosq * s.con41 +
                 0.0625 * temp2 * rteosq * (13.0 - 78.0 * cosio2 + 137.0 * cosio4);
        s.argpdot = -0.5 * temp1 * con42 + 0.0625 * temp2 * (7.0 - 114.0 * cosio2 + 395.0 * cosio4) +
                    temp3 * (3.0 - 36.0 * cosio2 + 49.0 * cosio4);
        const double xhdot1 = -temp1 * cosio;
        s.nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * cosio2) + 2.0 * temp3 * (3.0 - 7.0 * cosio2)) * cosio;
        s.omgcof = s.bstar * cc3 * std::cos(s.argpo);
        s.xmcof = 0.0;
        if (s.ecco > 1.0e-4) s.xmcof = -kX2o3 * coef * s.bstar / eeta;
        s.nodecf = 3.5 * omeosq * xhdot1 * s.cc1;
        s.t2cof = 1.5 * s.cc1;
        if (std::fabs(cosio + 1.0) > 1.5e-12)
            s.xlcof = -0.25 * kJ3oJ2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
        else
            s.xlcof = -0.25 * kJ3oJ2 * sinio * (3.0 + 5.0 * cosio) / temp4;
        s.aycof = -0.5 * kJ3oJ2 * sinio;
        const double delmotemp = 1.0 + s.eta * std::cos(s.mo);
        s.delmo = delmotemp * delmotemp * delmotemp;
        s.sinmao = std::sin(s.mo);
        s.x7thm1 = 7.0 * cosio2 - 1.0;
        if (s.isimp != 1) {
            const double cc1sq = s.cc1 * s.cc1;
            s.d2 = 4.0 * ao * tsi * cc1sq;
            const double temp = s.d2 * tsi * s.cc1 / 3.0;
            s.d3 = (17.0 * ao + sfour) * temp;
            s.d4 = 0.5 * temp * ao * tsi * (221.0 * ao + 31.0 * sfour) * s.cc1;
            s.t3cof = s.d2 + 2.0 * cc1sq;
            s.t4cof = 0.25 * (3.0 * s.d3 + s.cc1 * (12.0 * s.d2 + 10.0 * cc1sq));
            s.t5cof = 0.2 * (3.0 * s.d4 + 12.0 * s.cc1 * s.d3 + 6.0 * s.d2 * s.d2 + 15.0 * cc1sq * (2.0 * s.d2 + cc1sq));
        }
    }
    double r[3], v[3];
    return sgp4_propagate(s, 0.0, r, v);
}

bool sgp4_propagate(Sgp4& s, double t, double r[3], double v[3]) {
    const double k = xke();
    const double temp4 = 1.5e-12;
    const double vkmpersec = kRe * k / 60.0;
    s.error = s.error == 7 ? 7 : 0;
    if (s.error) return false;

    const double xmdf = s.mo + s.mdot * t;
    const double argpdf = s.argpo + s.argpdot * t;
    const double nodedf = s.nodeo + s.nodedot * t;
    double argpm = argpdf, mm = xmdf;
    const double t2 = t * t;
    double nodem = nodedf + s.nodecf * t2;
    double tempa = 1.0 - s.cc1 * t;
    double tempe = s.bstar * s.cc4 * t;
    double templ = s.t2cof * t2;
    if (s.isimp != 1) {
        const double delomg = s.omgcof * t;
        const double delmtemp = 1.0 + s.eta * std::cos(xmdf);
        const double delm = s.xmcof * (delmtemp * delmtemp * delmtemp - s.delmo);
        const double temp = delomg + delm;
        mm = xmdf + temp;
        argpm = argpdf - temp;
        const double t3 = t2 * t, t4 = t3 * t;
        tempa = tempa - s.d2 * t2 - s.d3 * t3 - s.d4 * t4;
        tempe = tempe + s.bstar * s.cc5 * (std::sin(mm) - s.sinmao);
        templ = templ + s.t3cof * t3 + t4 * (s.t4cof + t * s.t5cof);
    }
    double nm = s.no_unkozai;
    double em = s.ecco;
    const double inclm = s.inclo;
    if (nm <= 0.0) {
        s.error = 2;
        return false;
    }
    const double am = std::pow(k / nm, kX2o3) * tempa * tempa;
    nm = k / std::pow(am, 1.5);
    em = em - tempe;
    if (em >= 1.0 || em < -0.001) {
        s.error = 1;
        return false;
    }
    if (em < 1.0e-6) em = 1.0e-6;
    mm = mm + s.no_unkozai * templ;
    double xlm = mm + argpm + nodem;
    nodem = nodem >= 0.0 ? std::fmod(nodem, kTwoPi) : -std::fmod(-nodem, kTwoPi);
    argpm = fmod2p(argpm);
    xlm = fmod2p(xlm);
    mm = fmod2p(xlm - argpm - nodem);

    const double sinip = std::sin(inclm), cosip = std::cos(inclm);
    const double ep = em, argpp = argpm, nodep = nodem, mp = mm;

    const double axnl = ep * std::cos(argpp);
    double temp = 1.0 / (am * (1.0 - ep * ep));
    const double aynl = ep * std::sin(argpp) + temp * s.aycof;
    const double xl = mp + argpp + nodep + temp * s.xlcof * axnl;
    const double u = fmod2p(xl - nodep);
    double eo1 = u, tem5 = 9999.9, sineo1 = 0, coseo1 = 0;
    for (int ktr = 1; std::fabs(tem5) >= 1.0e-12 && ktr <= 10; ktr++) {
        sineo1 = std::sin(eo1);
        coseo1 = std::cos(eo1);
        tem5 = 1.0 - coseo1 * axnl - sineo1 * aynl;
        tem5 = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
        if (std::fabs(tem5) >= 0.95) tem5 = tem5 > 0.0 ? 0.95 : -0.95;
        eo1 = eo1 + tem5;
    }
    const double ecose = axnl * coseo1 + aynl * sineo1;
    const double esine = axnl * sineo1 - aynl * coseo1;
    const double el2 = axnl * axnl + aynl * aynl;
    const double pl = am * (1.0 - el2);
    if (pl < 0.0) {
        s.error = 4;
        return false;
    }
    const double rl = am * (1.0 - ecose);
    const double rdotl = std::sqrt(am) * esine / rl;
    const double rvdotl = std::sqrt(pl) / rl;
    const double betal = std::sqrt(1.0 - el2);
    temp = esine / (1.0 + betal);
    const double sinu = am / rl * (sineo1 - aynl - axnl * temp);
    const double cosu = am / rl * (coseo1 - axnl + aynl * temp);
    double su = std::atan2(sinu, cosu);
    const double sin2u = (cosu + cosu) * sinu;
    const double cos2u = 1.0 - 2.0 * sinu * sinu;
    temp = 1.0 / pl;
    const double temp1 = 0.5 * kJ2 * temp;
    const double temp2 = temp1 * temp;
    const double mrt = rl * (1.0 - 1.5 * temp2 * betal * s.con41) + 0.5 * temp1 * s.x1mth2 * cos2u;
    su = su - 0.25 * temp2 * s.x7thm1 * sin2u;
    const double xnode = nodep + 1.5 * temp2 * cosip * sin2u;
    const double xinc = inclm + 1.5 * temp2 * cosip * sinip * cos2u;
    const double mvt = rdotl - nm * temp1 * s.x1mth2 * sin2u / k;
    const double rvdot = rvdotl + nm * temp1 * (s.x1mth2 * cos2u + 1.5 * s.con41) / k;
    const double sinsu = std::sin(su), cossu = std::cos(su);
    const double snod = std::sin(xnode), cnod = std::cos(xnode);
    const double sini = std::sin(xinc), cosi = std::cos(xinc);
    const double xmx = -snod * cosi, xmy = cnod * cosi;
    const double ux = xmx * sinsu + cnod * cossu;
    const double uy = xmy * sinsu + snod * cossu;
    const double uz = sini * sinsu;
    const double vx = xmx * cossu - cnod * sinsu;
    const double vy = xmy * cossu - snod * sinsu;
    const double vz = sini * cossu;
    const double mr = mrt * kRe;
    r[0] = mr * ux;
    r[1] = mr * uy;
    r[2] = mr * uz;
    v[0] = (mvt * ux + rvdot * vx) * vkmpersec;
    v[1] = (mvt * uy + rvdot * vy) * vkmpersec;
    v[2] = (mvt * uz + rvdot * vz) * vkmpersec;
    (void)temp4;
    if (mrt < 1.0) {
        s.error = 6;
        return false;
    }
    return true;
}

}  // namespace ui::external_app::gps
