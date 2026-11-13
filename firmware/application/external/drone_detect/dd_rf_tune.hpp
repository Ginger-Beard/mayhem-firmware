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

/* Every tunable number of the RF (energy sweep) detector in one place. All
 * of them are first guesses from the protocol facts, to be tuned against the
 * DDSWP dumps of real emitters. Levels in centi-dB (cdB), frequencies in kHz,
 * shares in percent, durations in milliseconds (the display frame rate is not
 * 60 Hz on every LCD: the H4M's runs at ~76 Hz, so frame counts were 21 %
 * short). */

#ifndef __DD_RF_TUNE_HPP__
#define __DD_RF_TUNE_HPP__

#include <cstdint>

/* Full feature set: on in every build where the app is built in (tier.txt
 * is 2, so FLASH_TIER >= 2: the HackRF Pro). Elsewhere the app is external
 * (32 KiB section) and, off PRALINE, only a "Requires HackRF Pro" stub; a
 * PRALINE build with a smaller flash tier would get the reduced set that fits
 * 32 KiB: without the SD copy of the sweep dump, the 1.2 and 5.3 GHz bands,
 * the 2.4 GHz hopper / toy / oven-hit classifiers, the RF lines of the detail
 * view, the demo drones, the raw frame console, "Inv" and the low confidence
 * RF beep. Host tests set it with -DDD_FULL=0/1. */
#ifndef DD_FULL
#if defined(PRALINE) && defined(FLASH_TIER) && FLASH_TIER >= 2
#define DD_FULL 1
#else
#define DD_FULL 0
#endif
#endif

namespace ui::external_app::drone_detect {

/* How a classifier rule or table entry was validated; the labels must not
 * claim more. A GRADE_DOC rule caps its verdict at low confidence and marks
 * its label with '?' (the detail view says what '?' means). */
enum RfGrade : uint8_t {
    GRADE_PRIOR,    // rule from before the grades, output as it was (not verified on its real emitter either)
    GRADE_DOC,      // from documents (FCC filings, open-source code); simulation only
    GRADE_STANDIN,  // tested with a bench stand-in emitter
    GRADE_REAL,     // verified on the real emitter
};

} /* namespace ui::external_app::drone_detect */

namespace ui::external_app::drone_detect::tune {

/* ---- scheduler (ui_drone_detect.cpp) ---- */
constexpr uint16_t kBleMinMs = 300;       // shortest BLE dwell: one Location message per ~1 s per drone
constexpr uint16_t kBleOnlyMs = 1200;     // BLE dwell per channel without sweeps (RID only)
constexpr uint8_t kBleRatio = 1;          // BLE dwell = ratio x the next sweep dwell: >= 50 % airtime
constexpr uint8_t kBleRatioRid = 3;       // ... while Remote ID was heard: >= 75 % airtime
constexpr uint8_t kRidBoostSeconds = 15;  // how long a Remote ID frame keeps the boost
/* Wi-Fi Remote ID dwells (PDRN DSSS mode, baseband/dd_dsss.hpp): 802.11b
 * 1 Mb/s on a 4 MHz slice of the channel, the same 4 Msps as BLE (a retune,
 * no rate switch). The BLE dwell before it grows by the ratio, so BLE keeps
 * its >= 50 / 75 % share. */
constexpr uint16_t kWifiDwellMs = 300;   // catches a 5 Hz beacon in every dwell (~1.5 frames), a 1 Hz one in 30 %
constexpr uint8_t kWifiEveryBle = 2;     // a Wi-Fi dwell after every 2nd BLE dwell (~15 % airtime with sweeps)
constexpr uint8_t kWifiEveryBleRid = 1;  // ... after every one while on-board Wi-Fi hears Remote ID (kRidBoostSeconds)
constexpr int8_t kWifiSliceMhz = 2;      // slice centres at -2 / +2 MHz from the channel centre, in turn per visit:
                                         // a narrowband interferer in one half (seen on the bench) spares the other
struct WifiChannel {
    uint8_t number;
    uint16_t mhz;
};
/* 2.4 GHz only: 802.11b DSSS does not exist at 5 GHz (OFDM PHYs only), so
 * channel 149 needs the 20 Msps OFDM receiver sketched in proto/dsss
 * README step 4c, not this mode. Channel 14 (DSSS only, Japan) left out. */
constexpr WifiChannel kWifiChannels[] = {{6, 2437}, {1, 2412}, {11, 2462}};
constexpr uint8_t kWifiChannelMask = 0x01;     // entries scanned: channel 6 (where Remote ID beacons usually are)
constexpr uint8_t kWifiChannelMaskAll = 0x07;  // debug key w: channels 6, 1 and 11

/* DJI DroneID dwells (PDRN MODE_DRONEID, baseband/dd_droneid.hpp): 15.36 Msps
 * (AFE 30.72 Msps, FPGA half band / 2), the LO on one 2.4 GHz DroneID centre,
 * presence only. The v3 hop schedule (7 centres, 640 ms publications) puts
 * one publication on each 2.4 GHz centre every 2.56-4.48 s, so a 4.5 s dwell
 * always holds a whole burst: P(detect) = the per-burst Pd (prototype: 0.9 at
 * about -3 dB in-band SNR in a clean channel, 0.65-0.9 in a busy household
 * channel). Time based: a dwell when kDidGapMs have passed since the last one
 * ended, outside the BLE ratio, the cycle resumed after it (rf_test/
 * sched_model.py: default cycle 4.90 -> 5.50 s, +12.4 %; 11 % airtime). */
constexpr uint32_t kDidSampleRate = 15360000;
constexpr uint16_t kDidDwellMs = 4500;
constexpr uint16_t kDidGapMs = 36000;                                // one DroneID dwell every ~40.5 s (< kExpireSeconds: a drone stays listed)
constexpr uint16_t kDidFirstMs = 6000;                               // the first one this long after the start
constexpr uint32_t kDidChannelsKhz[] = {2414500, 2429500, 2444500};  // in turn
/* Sweep rate (measured on the HackRF Pro, slices per second, 20 Msps, FFT
 * 256): paced avg 4 settle 100 ~540; unpaced avg 4 ~640; paced avg 1 ~720-870;
 * unpaced avg 1 settle 0 ~1100-1500. The M4's FFTs bound it (4 per slice at
 * avg 4), then the frame pacing. */
constexpr uint8_t kSweepAvgPeak = 4;  // peak detector bands (2.4, 915/868): ~3.5 bins a point, noise peaks need the average
constexpr uint8_t kSweepAvgMean = 1;  // average detector bands (5.8, 5.3, 1.2): ~12 bins a point already average the noise
/* Unpaced: with the M4's FFTs the limit, the replies come ~1 ms apart and the
 * M0 queue never builds up (peak 58 bytes measured); UI latency unchanged. A
 * band inside one tune (868) is paced anyway (its capture repeats every DMA
 * buffer). */
constexpr bool kFramePaced = false;
constexpr uint16_t kSweepSettleUs = 0;  // PRALINE: no slice edge artefacts at 0 (5.8 GHz floor ripple <= 1 dB either way)
/* Complete sweeps thrown away at the start of each dwell: the first slice
 * after the switch from 4 Msps (BLE) to 20 Msps is captured while the radio
 * still settles, ~10 dB low in 11 % of looks at 915 MHz (always the first
 * look of a dwell; 7 % with 100 us settle). It made a false 13 MHz "FPV?"
 * hump at 909 MHz out of the floor. */
constexpr uint8_t kDiscardLooks = 1;
constexpr uint8_t kStallReset = 2;        // sweep dwells in a row without a slice: reapply the rates
constexpr uint16_t kDumpMinGapMs = 1000;  // single band dump lines at most once a second (USB queue, M0 time)

/* ---- per point accumulation (one dwell) ---- */
constexpr uint8_t kMaxLooks = 250;       // complete sweeps per dwell at most: uint8 counters, uint16 sums
constexpr int16_t kOccPeakCdb = 600;     // occupied / hit: above floor (peak detector, ~3.5 bins/point)
constexpr int16_t kOccAvgCdb = 400;      // ... (average detector: 12 bins/point, noise std < 1 dB)
constexpr int16_t kSumOffsetCdb = 2000;  // per look level stored as (rel + 20 dB) in 0.5 dB, 0..255

/* ---- noise floor (per band, smoothed across cycles) ---- */
constexpr uint8_t kFloorSegs = 8;              // floor pieces per band, interpolated: follows gain slopes
constexpr uint8_t kFloorPctl = 25;             // floor = lower quartile of the point means: signals may fill 3/4
constexpr uint8_t kFloorShift = 2;             // floor follows 1/4 of a rise per dwell (a burst cannot lift it)
constexpr int16_t kFloorFastDownCdb = 300;     // ... but drops at once when 3 dB lower (gain change, first fix)
constexpr int16_t kFloorMaxAboveMedCdb = 800;  // a piece filled by one wide signal: capped near the others

/* ---- wide segments (video, Wi-Fi) ---- */
constexpr uint8_t kSegMinOccPct = 15;     // a point belongs to a segment when occupied in >= 15 % of looks
constexpr uint8_t kSegGapPts = 1;         // gaps of 1 point inside a segment are bridged
constexpr uint16_t kWideMinKhz = 3500;    // narrower runs are left to the hit statistics
constexpr uint8_t kMaxSegs = 8;           // segments kept per dwell
constexpr int16_t kSpanFloorCdb = 150;    // -3/-10 dB spans stop 1.5 dB above floor
constexpr int16_t kShapeMarginCdb = 300;  // ... and the "-10 dB" one at most down to 3 dB over floor (low SNR)
constexpr uint8_t kCentralPct = 60;       // flatness and duty over the central 60 % of the -10 dB span

/* ---- analog FPV video (FM, continuous, peaked) ---- */
constexpr uint8_t kAnalogMinDuty = 90;       // carrier never pauses
constexpr uint16_t kAnalogMinW10Khz = 4000;  // FM video: 6..18 MHz, some margin
constexpr uint16_t kAnalogMaxW10Khz = 22000;
constexpr int16_t kAnalogMinFlatCdb = 300;  // peak 3 dB over the median of its centre: peaked hump
constexpr uint8_t kAnalogMaxShapePct = 70;  // or -3 dB width <= 70 % of the -10 dB width (Gaussian: 55 %)
constexpr uint16_t kAnalogTolKhz = 3000;    // centre within 3 MHz of a band table channel
constexpr int16_t kHighSnrCdb = 1000;       // >= 10 dB over floor: high confidence

/* ---- digital video / Wi-Fi (OFDM flat top) ---- */
constexpr int16_t kDigMaxFlatCdb = 250;   // flat top: peak within 2.5 dB of the centre median
constexpr uint8_t kDigMinShapePct = 75;   // ... and steep skirts: -3 dB width >= 75 % of -10 dB width
constexpr uint8_t kDigMinDuty = 80;       // video downlink: sustained
constexpr uint8_t kWifiMaxDuty = 50;      // Wi-Fi traffic below this on the channel grid: suppressed
constexpr uint16_t kDigTolKhz = 2000;     // centre within 2 MHz of a digital FPV table channel
constexpr uint16_t kWifi24TolKhz = 1000;  // 2.4 GHz Wi-Fi centres 2412 + 5k MHz
constexpr uint16_t kWifi5TolKhz = 2000;   // 5 GHz Wi-Fi centres on a 10 MHz grid (20/40/80 MHz); ~2 points
/* -3 dB width classes (occupied ~0.9 x nominal) of 5, 10, 20, 40, 60/80 MHz.
 * The 5 MHz class is GRADE_DOC (DJI's 2.4 GHz 5 MHz mode, Skydio and
 * Antigravity 5 MHz OFDM, from their FCC filings): before it such a signal
 * was analog FPV when it read peaked (the 5 GHz bands' ~1 MHz points) and
 * nothing otherwise. */
constexpr uint8_t kWidthClasses = 5;
constexpr uint32_t kWidthLoKhz[kWidthClasses] = {3500, 7000, 14000, 28000, 48000};
constexpr uint32_t kWidthHiKhz[kWidthClasses] = {6500, 11500, 22500, 44000, 84000};
constexpr uint8_t kWidthMhz[kWidthClasses] = {5, 10, 20, 40, 60};
constexpr uint8_t kWidthGrade[kWidthClasses] = {GRADE_DOC, GRADE_PRIOR, GRADE_PRIOR, GRADE_PRIOR, GRADE_PRIOR};
constexpr uint16_t kW5MaxW10Khz = 8000;  // 5 MHz class: -10 dB width at most 8 MHz (wider: an FM hump's skirt) ...
constexpr uint8_t kW5MinShapePct = 60;   // ... and w3/w10 >= 60 % (a Gaussian FM hump reads ~55 %: stays analog)
/* Channel-grid tests on a centre (DJI's x.5 MHz grid) only where a point is
 * at most this wide: 2.4 GHz has 276 kHz points; the 5 GHz bands' ~970 kHz
 * points cannot resolve a +-250 kHz test, so their grid entries stay inert
 * (and a signal wider than kGridMaxPiecePct of a floor piece is never
 * tested: dd_rf.cpp video_grid(); at 2.4 GHz that leaves the 5 MHz class). */
constexpr uint16_t kGridMaxPtKhz = 400;
constexpr uint8_t kGridMaxPiecePct = 75;

/* ---- microwave oven (2.4 GHz, mains gated, wandering) ---- */
constexpr uint8_t kOvenMinPct = 20;  // looks with a 3..8 MHz hit / segment duty: 25..75 % +- margin
constexpr uint8_t kOvenMaxPct = 80;
constexpr uint32_t kOvenLoKhz = 2425000;  // magnetrons sit at 2.43..2.48 GHz
constexpr uint32_t kOvenHiKhz = 2485000;
constexpr uint16_t kOvenMinSpreadKhz = 10000;  // the peak wanders over >= 10 MHz in a dwell

/* ---- narrow hits (hoppers) ---- */
constexpr uint16_t kHitThinKhz = 250;   // narrower: meters, LoRaWAN 125 kHz: counted, not a hop
constexpr uint8_t kHitMinPts = 2;       // single point runs are noise peaks (or thin), never hops
constexpr uint16_t kHitNormKhz = 3200;  // 0.25..3.2 MHz: ELRS, FHSS, BT, toys, 802.15.4
constexpr uint16_t kHitWideKhz = 8000;  // 3.2..8 MHz: oven / pieces of wide bursts
constexpr uint8_t kFixedPct = 50;       // a point hit in >= 50 % of looks is a carrier or spur, not a hop
constexpr uint16_t kBleExclKhz = 1200;  // BLE advertising 2402/2426/2480 +- 1 MHz (+ part of a point)
constexpr uint8_t kMinHits = 6;         // hits per dwell before any hopper verdict

/* ---- 868/915 MHz control links ---- */
constexpr uint8_t kThinRejectPct = 60;    // mostly < 250 kHz hits: LoRaWAN 125k / meters
constexpr uint8_t kLoraDlRejectPct = 70;  // mostly in the US LoRaWAN downlink block 923.3..927.5
constexpr uint32_t kLoraDlLoKhz = 923000;
constexpr uint32_t kLoraDlHiKhz = 928000;
constexpr uint16_t kGridTolKhz = 150;    // ELRS channel grid tolerance (points are ~80 kHz)
constexpr uint8_t kElrsMinDistinct = 6;  // ELRS 915 uses all 40 channels: many in 250 ms
constexpr uint16_t kElrsMinSpanKhz = 6000;
constexpr uint8_t kElrsMinGridPct = 60;
constexpr uint8_t kElrsHighDistinct = 12;  // high confidence
constexpr uint8_t kElrsHighGridPct = 80;
constexpr uint8_t kElrsEuMinDistinct = 4;  // EU868: only 13 channels over 6.3 MHz
constexpr uint16_t kElrsEuMinSpanKhz = 2500;
constexpr uint8_t kFhssMinDistinct = 8;  // off grid: generic FHSS (SiK, unnamed), low confidence
/* A band's further hop grids (dd_rf.cpp kHopGrids: Crossfire, FrSky R9,
 * ELRS 2.4) name a hopper only when their on-grid share beats every other
 * grid of the band by this much: ELRS 915 lands ~60 % on R9's 0.5 MHz grid
 * (every 5th ELRS channel is on it, two more within 100 kHz). */
constexpr uint8_t kGridMarginPct = 20;
constexpr uint8_t kNamedGridPct = 70;     // ... and hold at least this share of the hits themselves
constexpr uint16_t kHopGridTolKhz = 100;  // Crossfire (x.25 / x.75) and R9 (x.0 / x.5) are 250 kHz apart
constexpr uint16_t kElrs24TolKhz = 150;   // ELRS 2.4 (x.4) against the toys' x.0; points are 276 kHz there
/* ELRS 2.4 hops over 80 channels (60-75 distinct per 30-look dwell in
 * simulation); household clutter named it from 9 distinct in the corpus. */
constexpr uint8_t kElrs24MinDistinct = 16;
/* DJI video centre grid test (x.5 MHz): wider would take Wi-Fi's integer
 * centres read a quarter MHz off. */
constexpr uint16_t kVideoGridTolKhz = 250;

/* ---- 2.4 GHz hoppers / toys (LOW confidence: BT Classic looks the same) ---- */
constexpr uint8_t kHop24MinDistinct = 7;  // BT / ELRS 2.4 / FrSky: many channels
constexpr uint8_t kToyMaxDistinct = 6;    // toy protocols: 3..16, usually 4 data channels
constexpr uint8_t kToyMinGridPct = 60;    // toys tune 2400 + n MHz
constexpr uint16_t kToyGridTolKhz = 300;

/* ---- persistence ---- */
constexpr uint8_t kConfirmDwells = 2;     // dwells in a row with the verdict: confirmed
constexpr uint8_t kConfirmDwellsLow = 3;  // ... low confidence classes
/* Duty classes (analog, digital, DV/WF) judge duty over all the looks of the
 * track's dwells, not one short dwell: confirmed only with this many looks
 * and the class's duty over them. Wide bands get ~10-20 looks a dwell. */
constexpr uint16_t kConfirmLooks = 24;
constexpr uint8_t kDutySlackPct = 20;  // per dwell: analog >= 70 %, digital >= 60 % (the track: 90 / 80 %)
constexpr uint8_t kDropMisses = 3;     // dwells of the band without it: dropped
constexpr uint16_t kMatchKhz = 3000;   // same emitter: centre within 3 MHz (or a quarter of its width)

} /* namespace ui::external_app::drone_detect::tune */

#endif /*__DD_RF_TUNE_HPP__*/
