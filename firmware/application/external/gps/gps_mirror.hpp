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

#ifndef __GPS_MIRROR_H__
#define __GPS_MIRROR_H__

/* A mirrored spectrum (the samples' Q negated: seen once on the device, cause
 * unknown) told from one snapshot, for Autonomous, which takes no second
 * snapshot: each detection's measured Doppler against the one predicted from
 * the orbits and the saved position. Read directly, measured - predicted is
 * the receiver clock's frequency term, the same for every satellite; read
 * mirrored, (-measured) - predicted is. Two or more agreeing under the mirror
 * reading (or one plus a 2nd-snapshot "mirrored" rejection) while no two agree
 * under the direct reading: mirrored. Header only, host tested. */

#include <cmath>
#include <cstdint>

namespace ui::external_app::gps {

enum MirrorVerdict : uint8_t {
    MIRROR_UNKNOWN = 0, /* too few detections, or both readings fit (similar Dopplers) */
    MIRROR_DIRECT = 1,
    MIRROR_YES = 2,
};

struct MirrorResult {
    MirrorVerdict verdict;
    uint8_t direct;   /* detections agreeing under the direct reading (largest group) */
    uint8_t mirrored; /* ... under the mirror reading (+ 2nd-snapshot "mirrored" rejections) */
    float clk_hz;     /* the clock term of the reading chosen */
};

/* largest group of residuals within tol of one another's centre; *centre its mean */
inline int mirror_group(const float* r, int n, float tol, float* centre) {
    int best = 0;
    float c = 0;
    for (int i = 0; i < n; i++) {
        int k = 0;
        float s = 0;
        for (int j = 0; j < n; j++)
            if (std::fabs(r[j] - r[i]) <= tol) {
                k++;
                s += r[j];
            }
        if (k > best) {
            best = k;
            c = s / k;
        }
    }
    *centre = c;
    return best;
}

/* meas / pred: Doppler (Hz) of n detections, measured and predicted (geometric,
 * no clock term); n_reason4: 2nd-snapshot rejections as mirrored (Snapshot /
 * Heard only); tol: Hz the clock term may differ between satellites (receiver
 * motion: ~150 Hz at 25 m/s, prediction from a rough position and time);
 * max_clk: Hz the clock term can reach (TCXO). */
inline MirrorResult mirror_classify(const float* meas, const float* pred, int n, int n_reason4, float tol = 300.0f, float max_clk = 5000.0f) {
    MirrorResult res{MIRROR_UNKNOWN, 0, 0, 0};
    if (n > 16) n = 16;
    float rd[16], rm[16];
    for (int i = 0; i < n; i++) {
        rd[i] = meas[i] - pred[i];
        rm[i] = -meas[i] - pred[i];
    }
    float cd = 0, cm = 0;
    int kd = mirror_group(rd, n, tol, &cd), km = mirror_group(rm, n, tol, &cm);
    if (std::fabs(cd) > max_clk) kd = n > 0 ? 1 : 0; /* an implausible clock term: no agreement */
    if (std::fabs(cm) > max_clk) km = n > 0 ? 1 : 0;
    res.direct = static_cast<uint8_t>(kd);
    res.mirrored = static_cast<uint8_t>(km + n_reason4);
    if (kd >= 2 && kd > km + n_reason4) {
        res.verdict = MIRROR_DIRECT;
        res.clk_hz = cd;
    } else if (kd <= 1 && km + n_reason4 >= 2) {
        res.verdict = MIRROR_YES;
        res.clk_hz = cm;
    }
    return res;
}

}  // namespace ui::external_app::gps

#endif /*__GPS_MIRROR_H__*/
