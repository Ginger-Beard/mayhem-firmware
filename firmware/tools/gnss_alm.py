#!/usr/bin/env python3
#
# Copyright (C) 2026 PortaPack Mayhem contributors
#
# This file is part of PortaPack.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

"""gnss_alm.py: Galileo / BeiDou / GLONASS / SBAS prediction almanac for the GPS app (/GPS/GNSSALM.TXT).

Usage: gnss_alm.py BRDC_MIXED.rnx [more.rnx ...] > GNSSALM.TXT
  BRDC_MIXED.rnx: a public broadcast navigation file, RINEX 3, mixed (e.g. the BKG daily BRDC file,
  https://igs.bkg.bund.de/root_ftp/IGS/BRDC/<yyyy>/<doy>/BRDC00WRD_S_<yyyy><doy>0000_01D_MN.rnx.gz).

For each Galileo (E), BeiDou (C), GLONASS (R) and SBAS (S) satellite it takes the earliest and the latest
record in the files, computes the satellite's position and velocity at both (the system's own broadcast
model: Galileo / BeiDou Keplerian with harmonics, BeiDou GEO with its -5 deg rotation, GLONASS and SBAS
state vectors), converts both states to orbital elements in an inertial frame, and fits the mean motion
and the node rate from the two (which removes most of the short-period J2 error of a single osculating
set). The elements at the latest record are written in the app's almanac model (the GPS almanac
equations of IS-GPS-200 with the GPS constants), one line per satellite:

  sys prn health sqrtA e i0 Omega0 OmegaDot omega M0 week toa [k]

sys E/C/R/S; health 0 = usable (else the system's flag); sqrtA m^0.5; angles rad (Omega0: longitude of
the node at the start of GPS week `week`), OmegaDot rad/s; toa seconds of that GPS week; k: GLONASS
frequency channel. Accuracy: --check (prediction error against later broadcast records), and the GPS
app's README (sdcard/GPS/README.TXT). Predictions only: positions for the sky plot and the satellites up, not for a fix.
"""

import math
import sys

MU = {'E': 3.986004418e14, 'C': 3.986004418e14}
OMEGA_E = {'E': 7.2921151467e-5, 'C': 7.292115e-5}
MU_GPS = 3.986005e14           # the app's constants (gps_orbit.cpp)
OMEGA_E_GPS = 7.2921151467e-5
WEEK = 604800.0
GPS_EPOCH_JD = 2444244.5       # 1980-01-06
LEAP = 18                      # GPS - UTC, s (valid since 2017)
BDT_GPS = 14.0                 # GPS - BDT, s
J2_GLO, AE_GLO, MU_GLO, OMEGA_E_GLO = 1.0826257e-3, 6378136.0, 3.9860044e14, 7.292115e-5


def gps_seconds(y, mo, d, h, mi, s):
    """continuous GPS seconds of a civil date given in GPS time"""
    a = (14 - mo) // 12
    yy = y + 4800 - a
    mm = mo + 12 * a - 3
    jdn = d + (153 * mm + 2) // 5 + 365 * yy + yy // 4 - yy // 100 + yy // 400 - 32045
    return (jdn - 0.5 - GPS_EPOCH_JD) * 86400.0 + h * 3600 + mi * 60 + s


def fnum(s):
    s = s.strip().replace('D', 'e').replace('d', 'e')
    return float(s) if s else 0.0


def read_records(paths):
    recs = []
    for p in paths:
        lines = open(p, errors='replace').read().splitlines()
        i = 0
        while i < len(lines) and 'END OF HEADER' not in lines[i]:
            i += 1
        i += 1
        while i < len(lines):
            ln = lines[i]
            if len(ln) < 23 or ln[0] not in 'GERCSJI' or not ln[1:3].strip().isdigit():
                i += 1
                continue
            sysc, prn = ln[0], int(ln[1:3])
            n_lines = 4 if sysc in 'RS' else 8
            block = lines[i:i + n_lines]
            i += n_lines
            if sysc not in 'ECRS' or len(block) < n_lines:
                continue
            y, mo, d, h, mi, s = int(ln[4:8]), int(ln[9:11]), int(ln[12:14]), int(ln[15:17]), int(ln[18:20]), int(ln[21:23])
            v = [fnum(ln[23 + 19 * k:42 + 19 * k]) for k in range(3)]
            for b in block[1:]:
                v += [fnum(b[4 + 19 * k:23 + 19 * k]) for k in range(4)]
            recs.append((sysc, prn, (y, mo, d, h, mi, s), v))
    return recs


def rot3(v, a):
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2])


def rot1(v, a):
    c, s = math.cos(a), math.sin(a)
    return (v[0], c * v[1] + s * v[2], -s * v[1] + c * v[2])


def kepler_pos(sysc, prn, v, t_sow):
    """Galileo / BeiDou broadcast ephemeris: ECEF position at t (seconds of the system's week)"""
    mu, we = MU[sysc], OMEGA_E[sysc]
    crs, dn, m0 = v[4], v[5], v[6]
    cuc, e, cus, sqa = v[7], v[8], v[9], v[10]
    toe, cic, om0, cis = v[11], v[12], v[13], v[14]
    i0, crc, w, omd = v[15], v[16], v[17], v[18]
    idot = v[19]
    a = sqa * sqa
    tk = t_sow - toe
    if tk > WEEK / 2:
        tk -= WEEK
    if tk < -WEEK / 2:
        tk += WEEK
    n = math.sqrt(mu / a ** 3) + dn
    m = m0 + n * tk
    ea = m
    for _ in range(20):
        ea = m + e * math.sin(ea)
    nu = math.atan2(math.sqrt(1 - e * e) * math.sin(ea), math.cos(ea) - e)
    phi = nu + w
    u = phi + cus * math.sin(2 * phi) + cuc * math.cos(2 * phi)
    r = a * (1 - e * math.cos(ea)) + crs * math.sin(2 * phi) + crc * math.cos(2 * phi)
    i = i0 + idot * tk + cis * math.sin(2 * phi) + cic * math.cos(2 * phi)
    xp, yp = r * math.cos(u), r * math.sin(u)
    geo = sysc == 'C' and (prn <= 5 or prn >= 59)
    if not geo:
        om = om0 + (omd - we) * tk - we * toe
        return (xp * math.cos(om) - yp * math.cos(i) * math.sin(om),
                xp * math.sin(om) + yp * math.cos(i) * math.cos(om), yp * math.sin(i))
    # BeiDou GEO (BDS ICD): inertial-ish frame, then Rz(we tk) Rx(-5 deg)
    om = om0 + omd * tk - we * toe
    xg = xp * math.cos(om) - yp * math.cos(i) * math.sin(om)
    yg = xp * math.sin(om) + yp * math.cos(i) * math.cos(om)
    zg = yp * math.sin(i)
    p = rot1((xg, yg, zg), math.radians(-5.0))
    return rot3(p, we * tk)


def glo_deriv(st, acc):
    x, y, z, vx, vy, vz = st
    r = math.sqrt(x * x + y * y + z * z)
    mu, j2, ae, w = MU_GLO, J2_GLO, AE_GLO, OMEGA_E_GLO
    c = 1.5 * j2 * mu * ae * ae / r ** 5
    zz = 5 * z * z / (r * r)
    ax = -mu * x / r ** 3 - c * x * (1 - zz) + w * w * x + 2 * w * vy + acc[0]
    ay = -mu * y / r ** 3 - c * y * (1 - zz) + w * w * y - 2 * w * vx + acc[1]
    az = -mu * z / r ** 3 - c * z * (3 - zz) + acc[2]
    return (vx, vy, vz, ax, ay, az)


def glo_propagate(st, acc, dt):
    """GLONASS ICD: RK4 in the rotating PZ-90 frame (J2, centrifugal, Coriolis, lunisolar acc)"""
    h = 30.0 if dt >= 0 else -30.0
    t = 0.0
    while abs(dt - t) > 1e-9:
        step = h if abs(dt - t) > abs(h) else dt - t
        k1 = glo_deriv(st, acc)
        k2 = glo_deriv([s + step / 2 * k for s, k in zip(st, k1)], acc)
        k3 = glo_deriv([s + step / 2 * k for s, k in zip(st, k2)], acc)
        k4 = glo_deriv([s + step * k for s, k in zip(st, k3)], acc)
        st = [s + step / 6 * (a + 2 * b + 2 * c + d) for s, a, b, c, d in zip(st, k1, k2, k3, k4)]
        t += step
    return st


def state(sysc, prn, ep, v):
    """(continuous GPS seconds, ECEF position m, ECEF velocity m/s, health, k) of a record"""
    if sysc in 'EC':
        if sysc == 'E':
            t = gps_seconds(*ep)
            week = int(v[21])
            sow = v[11]
            t_sys_sow = sow
            t_gps = week * WEEK + sow
            health = int(v[24]) if len(v) > 24 else 0
        else:
            week = int(v[21])          # BDT week
            sow = v[11]                # toe, BDT seconds
            t_gps = (week + 1356) * WEEK + sow + BDT_GPS
            t_sys_sow = sow
            health = int(v[24]) if len(v) > 24 else 0  # SatH1
        p0 = kepler_pos(sysc, prn, v, t_sys_sow - 0.5)
        p1 = kepler_pos(sysc, prn, v, t_sys_sow + 0.5)
        pos = kepler_pos(sysc, prn, v, t_sys_sow)
        vel = tuple(b - a for a, b in zip(p0, p1))
        return t_gps, pos, vel, health, None
    t = gps_seconds(*ep)
    if sysc == 'R':
        t += LEAP  # GLONASS epochs are UTC
        sc = 1000.0
        pos = (v[3] * sc, v[7] * sc, v[11] * sc)
        vel = (v[4] * sc, v[8] * sc, v[12] * sc)
        acc = (v[5] * sc, v[9] * sc, v[13] * sc)
        return t, pos, vel, int(v[6]), int(v[10]), acc
    r = math.sqrt(v[3] ** 2 + v[7] ** 2 + v[11] ** 2)
    sc = 1000.0 if r < 1e6 else 1.0  # SBAS: km per RINEX, some converters write m
    pos = (v[3] * sc, v[7] * sc, v[11] * sc)
    vel = (v[4] * sc, v[8] * sc, v[12] * sc)
    return t, pos, vel, int(v[6]), None


def to_inertial(pos, vel, theta):
    """ECEF -> an inertial frame that coincides with ECEF when theta = 0 (rotated by -theta)"""
    w = OMEGA_E_GPS
    vi = (vel[0] - w * pos[1], vel[1] + w * pos[0], vel[2])
    c, s = math.cos(theta), math.sin(theta)
    rp = (c * pos[0] - s * pos[1], s * pos[0] + c * pos[1], pos[2])
    rv = (c * vi[0] - s * vi[1], s * vi[0] + c * vi[1], vi[2])
    return rp, rv


def elements(r, v, mu):
    rr = math.sqrt(sum(x * x for x in r))
    vv2 = sum(x * x for x in v)
    h = (r[1] * v[2] - r[2] * v[1], r[2] * v[0] - r[0] * v[2], r[0] * v[1] - r[1] * v[0])
    hn = math.sqrt(sum(x * x for x in h))
    i = math.acos(max(-1.0, min(1.0, h[2] / hn)))
    node = (-h[1], h[0], 0.0)
    nn = math.hypot(node[0], node[1])
    a = 1.0 / (2.0 / rr - vv2 / mu)
    rv = sum(x * y for x, y in zip(r, v))
    ev = tuple(((vv2 - mu / rr) * r[k] - rv * v[k]) / mu for k in range(3))
    e = math.sqrt(sum(x * x for x in ev))
    if nn < 1e-9 * hn:  # equatorial: node along x
        Om = 0.0
        node_u = (1.0, 0.0, 0.0)
    else:
        Om = math.atan2(node[1], node[0])
        node_u = (node[0] / nn, node[1] / nn, 0.0)
    # argument of latitude of the position, and of the perigee
    hx = (h[0] / hn, h[1] / hn, h[2] / hn)
    m_u = (hx[1] * node_u[2] - hx[2] * node_u[1], hx[2] * node_u[0] - hx[0] * node_u[2], hx[0] * node_u[1] - hx[1] * node_u[0])
    u = math.atan2(sum(a_ * b for a_, b in zip(r, m_u)), sum(a_ * b for a_, b in zip(r, node_u)))
    if e > 1e-7:
        w = math.atan2(sum(a_ * b for a_, b in zip(ev, m_u)), sum(a_ * b for a_, b in zip(ev, node_u)))
        nu = u - w
        ea = math.atan2(math.sqrt(1 - e * e) * math.sin(nu), e + math.cos(nu))
        M = ea - e * math.sin(ea)
    else:
        w, M = 0.0, u
    return a, e, i, Om, w, M, u


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def alm_position(f, t):
    """the app's almanac model (gps_orbit.cpp sat_position, ephemeris = 0, dn = 0) for a parsed line"""
    sqa, e, i0, om0, omd, w, m0, week, toa = f
    a = sqa * sqa
    n = math.sqrt(MU_GPS / a ** 3)
    toe = week * WEEK + toa
    tk = t - toe
    m = m0 + n * tk
    ea = m
    for _ in range(30):
        ea = m + e * math.sin(ea)
    nu = math.atan2(math.sqrt(1 - e * e) * math.sin(ea), math.cos(ea) - e)
    u = nu + w
    r = a * (1 - e * math.cos(ea))
    xp, yp = r * math.cos(u), r * math.sin(u)
    om = om0 + (omd - OMEGA_E_GPS) * tk - OMEGA_E_GPS * (toe % WEEK)
    return (xp * math.cos(om) - yp * math.cos(i0) * math.sin(om), xp * math.sin(om) + yp * math.cos(i0) * math.cos(om),
            yp * math.sin(i0))


def check(alm_path, rnx_paths):
    """prediction error of an almanac file against later broadcast records (their exact positions)"""
    alm = {}
    for ln in open(alm_path):
        if ln.startswith('#') or not ln.strip():
            continue
        p = ln.split()
        alm[(p[0], int(p[1]))] = [float(x) for x in p[3:10]] + [int(p[10]), float(p[11])]
    stats = {}
    for sysc, prn, ep, v in read_records(rnx_paths):
        if (sysc, prn) not in alm:
            continue
        st = state(sysc, prn, ep, v)
        if not all(math.isfinite(x) for x in st[1]) or math.sqrt(sum(x * x for x in st[1])) < 2e7:
            continue
        f = alm[(sysc, prn)]
        t = st[0]
        age_h = (t - (f[7] * WEEK + f[8])) / 3600.0
        q = alm_position(f, t)
        d = math.sqrt(sum((a_ - b_) ** 2 for a_, b_ in zip(q, st[1])))
        stats.setdefault(sysc, []).append((age_h, d, prn))
    for sysc in 'ECRS':
        if sysc not in stats:
            continue
        lst = sorted(stats[sysc])
        for lo, hi in ((-30, 0), (0, 12), (12, 24), (24, 48), (48, 1e9)):
            ds = sorted(d for a_, d, _ in lst if lo <= a_ < hi)
            if not ds:
                continue
            worst = max((d, p) for a_, d, p in lst if lo <= a_ < hi)
            print('%s age %5.0f..%-5.0f h: %4d records, error median %7.1f km, p95 %7.1f km, max %7.1f km (%s%02d); '
                  'as seen from the ground ~%.2f deg median' % (sysc, lo, min(hi, 999), len(ds), ds[len(ds) // 2] / 1e3,
                                                                ds[int(0.95 * (len(ds) - 1))] / 1e3, worst[0] / 1e3, sysc,
                                                                worst[1], math.degrees(ds[len(ds) // 2] / 2.2e7)))


def positions(alm_path, hours):
    """positions of every satellite of an almanac file at toa + each of `hours` (the app's model)"""
    for ln in open(alm_path):
        if ln.startswith('#') or not ln.strip():
            continue
        p = ln.split()
        f = [float(x) for x in p[3:10]] + [int(p[10]), float(p[11])]
        for h in hours:
            t = f[7] * WEEK + f[8] + h * 3600.0
            q = alm_position(f, t)
            print('%s %s %.3f %.3f %.3f %.3f' % (p[0], p[1], t, q[0], q[1], q[2]))


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--check':
        return check(sys.argv[2], sys.argv[3:])
    if len(sys.argv) > 2 and sys.argv[1] == '--positions':
        return positions(sys.argv[2], [float(h) for h in sys.argv[3:]] or [0.0, 24.0, 240.0, 720.0])
    paths = [p for p in sys.argv[1:] if not p.startswith('-')]
    if not paths:
        sys.exit(__doc__)
    recs = read_records(paths)
    by_sat = {}
    for sysc, prn, ep, v in recs:
        st = state(sysc, prn, ep, v)
        if st is None or not all(math.isfinite(x) for x in st[1]):
            continue
        r = math.sqrt(sum(x * x for x in st[1]))
        if r < 2.0e7:  # not a GNSS orbit (bad record)
            continue
        by_sat.setdefault((sysc, prn), []).append((st, v))
    out = []
    for (sysc, prn), lst in sorted(by_sat.items(), key=lambda kv: ('ECRS'.index(kv[0][0]), kv[0][1])):
        lst.sort(key=lambda x: x[0][0])
        healthy = [x for x in lst if x[0][3] == 0] or lst
        s1, s2 = healthy[0][0], healthy[-1][0]
        t1, t2 = s1[0], s2[0]
        if sysc == 'R':  # move the earlier state to its own epoch's exact position (records are exact there)
            pass
        theta = lambda t: OMEGA_E_GPS * (t - t2)
        r2, v2 = to_inertial(s2[1], s2[2], theta(t2))
        a2, e2, i2, Om2, w2, M2, u2 = elements(r2, v2, MU_GPS)
        n_osc = math.sqrt(MU_GPS / a2 ** 3)
        n_eff, omd = n_osc, 0.0
        if t2 - t1 > 3 * 3600:
            r1, v1 = to_inertial(s1[1], s1[2], theta(t1))
            a1, e1, i1, Om1, w1, M1, u1 = elements(r1, v1, MU_GPS)
            dt = t2 - t1
            omd = wrap(Om2 - Om1) / dt
            if i2 < math.radians(2.0):  # (near-)equatorial: node undefined, fit the longitude of u + Om
                omd = 0.0
                du = (u2 + Om2) - (u1 + Om1)
            else:
                du = u2 - u1
            k = round((n_osc * dt - du) / (2 * math.pi))
            n_eff = (du + 2 * math.pi * k) / dt
            if abs(n_eff / n_osc - 1) > 0.01:
                n_eff = n_osc
        a = (MU_GPS / n_eff ** 2) ** (1.0 / 3.0)
        week = int(t2 // WEEK)
        toa = t2 - week * WEEK
        Om0 = wrap(Om2 + OMEGA_E_GPS * toa)  # the app: Om_k = Om0 + (Omd - wE) tk - wE toa
        # SBAS: the RINEX health bits are about ranging; the GEOs broadcast their messages regardless
        health = 0 if sysc == 'S' else s2[3]
        # the fitted elements must reproduce the earlier record: if not (records of different quality,
        # a manoeuvre, or two satellites under one number), mark the satellite unusable
        q = alm_position([math.sqrt(a), e2, i2, Om0, omd, w2, wrap(M2), week, toa], t1)
        if math.sqrt(sum((x - y) ** 2 for x, y in zip(q, s1[1]))) > 200e3:
            health = health or 1
        line = '%s %d %d %.6f %.9f %.9f %.9f %.6e %.9f %.9f %d %.3f' % (sysc, prn, health, math.sqrt(a), e2, i2, Om0, omd,
                                                                     w2, wrap(M2), week, toa)
        if sysc == 'R' and s2[4] is not None:
            line += ' %d' % s2[4]
        out.append(line)
    print('# GNSSALM.TXT: Galileo (E), BeiDou (C), GLONASS (R), SBAS (S) prediction almanac for the GPS app')
    print('# from %s (public broadcast navigation, RINEX 3 mixed) by gnss_alm.py' % ' '.join(p.split('/')[-1] for p in paths))
    print('# sys prn health sqrtA e i0 Omega0 OmegaDot omega M0 week toa [k]   (GPS almanac model and constants)')
    for line in out:
        print(line)


if __name__ == '__main__':
    main()
