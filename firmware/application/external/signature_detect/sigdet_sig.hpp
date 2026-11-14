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

/* Signature classes of the Signature Detect app: one text file per class in
 * /SIGDET/SIGS/<NAME>.TXT (the way Looking Glass reads the .TXT files of /LOOKINGGLASS),
 * picked with the wheel on the main view. "Drones" is built in (no file, the
 * detectors as they always were). Integer only, fixed arrays, no UI or ChibiOS
 * dependency (host testable).
 *
 * File format: one rule per line, a type word then comma separated key=value
 * fields (FREQMAN style, as /SIGDET/PATTERNS.TXT); '#' starts a comment
 * line; lines up to kSigLineMax characters. Every rule may carry
 * valid=real|standin|doc (how it was validated; default doc), shown with the
 * detection and logged.
 *
 *   BLE,label=Apple Find My,kind=tracker,cid=004C,mfr=12,valid=real
 *     label=   up to 19 characters (required)
 *     kind=    tracker light beacon phone audio sensor home mesh meter radio
 *              noise drone other (default other)
 *     cid=     company ID of the manufacturer data, hex as the Bluetooth SIG
 *              lists it (004C = Apple)
 *     uuid=    16 bit service UUID, hex: in a UUID list or as service data
 *     sdata=   16 bit service data UUID, hex (service data only)
 *     mfr=     pattern on the manufacturer data after the company ID
 *     sd=      pattern on the service data after the UUID (needs sdata=)
 *     name=    prefix of the local name (ASCII, up to 8 characters)
 *     not=     up to two patterns ('/' separated) that exclude the advert,
 *              on the same field as mfr= / sd=
 *     minlen=  that field holds at least this many bytes
 *     addr=    public or random (TxAdd)
 *     beep=    1: beep when found (with "Beep on drone")
 *   A pattern is up to 8 bytes in hex, '?' a wildcard nibble, optionally
 *   &MASK in hex (40&FE matches 40 and 41). At least one of cid, uuid,
 *   sdata, name. The first matching rule of the file wins.
 *
 *   NB,label=Meshtastic slot,kind=mesh,f=925.375,bw=250,db=8,valid=real
 *     Narrowband energy at a frequency, from the energy sweep: shown as low
 *     confidence ('?'), it says energy at this frequency with this timing,
 *     nothing about the protocol.
 *     f=       centre, MHz, up to 3 decimals (required)
 *     bw=      expected width, kHz (required)
 *     db=      level over the noise floor needed, dB (default 8)
 *     duty=    lo-hi, % of the dwell's looks with energy (default 0-100)
 *     burst=   lo-hi, ms of one burst (needs whole bursts inside the dwell)
 *     period=  lo-hi, ms from one burst start to the next (two in a dwell)
 *     band=    the band to measure it in (default: the first that holds f)
 *
 *   BAND,name=915                built-in band: 2.4 5.8 5.3 1.2 915 868 5.1
 *   BAND,name=LMR,lo=450,hi=470,points=320,dwell=300
 *     custom band (at most kSigMaxBands): MHz, 64..320 points, 50..500 ms
 *
 *   PAT,name=...,class=...,ch=...,sync=...   a pattern scan entry, the
 *     /SIGDET/PATTERNS.TXT format (sigdet_pattern.hpp), up to kPatFileEntries
 *
 *   ZW,label=Z-Wave,kind=home,rate=9k6,f=908.42,valid=real
 *     Z-Wave frame receiver: the built-in FSK rule kZwFskFields (R1, 9.6 kb/s
 *     Manchester, XOR checksum) on f (default 908.42 MHz, US) with the Z-Wave
 *     header checks and network table (post=zwave). One per class; rate 9k6
 *     only. A network is listed after 2 valid frames of its home ID.
 *
 *   FSK,label=...,f=...,rate=...,sync=...,len=...[,...]
 *     Narrowband FSK frame rule (PSDT MODE_FSK, common/sigdet_fsk.hpp): the
 *     grammar is in /SIGDET/README.TXT ("FSK RULES") and parse_fsk(). Up to
 *     kSigMaxFsk FSK + ZW rules a class; rules with the same f, bw and chip
 *     rate share a dwell (up to sigdet::kFskRulesPerDwell).
 *
 *   SCAN,lo=1,hi=6000,res=81.5,looks=4,fft=0
 *     an interference scan range (sigdet_scan.hpp; up to kSigMaxScan): MHz,
 *     point spacing kHz (320 points a window), looks per window 1..8, FFT
 *     size 0 (auto) / 256..2048. A class with SCAN lines sweeps its scan
 *     ranges instead of bands (its BAND and NB lines are not swept).
 *   RX,ble,wifi,droneid,rid,dji,rfcls   receivers and built-in drone
 *     detectors a class asks for: ble (BLE dwells without BLE rules), wifi
 *     (Wi-Fi Remote ID dwells), droneid (DJI DroneID dwells), rid (Remote ID,
 *     BLE and Wi-Fi, as detections), dji (DJI BLE presence), rfcls (the
 *     sweep's drone classifiers). BLE dwells run with BLE rules (or ble),
 *     pattern dwells with PAT rules, sweeps with BAND or NB rules. Without
 *     rid / dji / rfcls those detectors list, log and beep nothing (Remote ID
 *     and DJI adverts then reach the survey as ordinary advertisers).
 */

#ifndef __SIGDET_SIG_HPP__
#define __SIGDET_SIG_HPP__

#include <cstddef>
#include <cstdint>

#include "sigdet_fsk.hpp"
#include "sigdet_pattern.hpp"
#include "sigdet_rf.hpp"
#include "sigdet_scan.hpp"

namespace ui::external_app::signature_detect {

constexpr size_t kSigMaxBle = 20;
constexpr size_t kSigMaxNb = 10;
constexpr size_t kSigMaxBands = 2; /* BAND_X0, BAND_X1 */
constexpr size_t kSigLabelLen = 20;
constexpr size_t kSigPatBytes = 8;
constexpr size_t kSigLineMax = 200;
constexpr size_t kSigMaxLines = 300;
constexpr size_t kSigNameLen = 16; /* class name (file stem) incl. the terminator */
constexpr size_t kSigMaxFsk = 4;   /* FSK + ZW rules, and so dwells, a class */
constexpr size_t kSigMaxScan = 2;  /* SCAN ranges (interference scan) */

enum SigKind : uint8_t {
    SK_OTHER,
    SK_TRACKER,
    SK_LIGHT,
    SK_BEACON,
    SK_PHONE,
    SK_AUDIO,
    SK_SENSOR,
    SK_HOME,
    SK_MESH,
    SK_METER,
    SK_RADIO,
    SK_NOISE,
    SK_DRONE,
    SK_COUNT
};
const char* sig_kind_name(uint8_t kind); /* "tracker" ... */
const char* sig_kind_tag(uint8_t kind);  /* four letters for the list: "TRKR" ... */

enum SigValid : uint8_t {
    SV_DOC,      // from documents only
    SV_STANDIN,  // tested with a bench stand-in
    SV_REAL,     // seen in test recordings / on the air
};
const char* sig_valid_name(uint8_t valid);

struct SigBytes {
    uint8_t n;
    uint8_t v[kSigPatBytes];
    uint8_t m[kSigPatBytes];
};

/* BleRule::flags */
enum : uint8_t {
    BR_CID = 0x01,
    BR_UUID = 0x02,
    BR_SDATA = 0x04,
    BR_NAME = 0x08,
    BR_PUBLIC = 0x10,
    BR_RANDOM = 0x20,
    BR_BEEP = 0x40,
    BR_MFR = 0x80, /* data applies to manufacturer data (with or without cid) */
};

struct BleRule {
    uint16_t cid;
    uint16_t uuid; /* BR_UUID or BR_SDATA */
    uint8_t flags;
    uint8_t minlen;
    uint8_t kind;
    uint8_t valid;
    uint8_t nnot;
    SigBytes data; /* mfr data after the company ID, service data after the UUID, or the name prefix */
    SigBytes nots[2];
    char label[kSigLabelLen];
};

struct NbRule {
    uint32_t f_khz;
    uint16_t bw_khz;
    uint8_t band;
    uint8_t min_db;
    uint8_t duty_lo, duty_hi;
    uint16_t burst_lo, burst_hi;   /* ms, hi 0 = any */
    uint16_t period_lo, period_hi; /* ms, hi 0 = any */
    uint16_t line;                 /* file line (a rule rejected at the end) */
    uint8_t kind;
    uint8_t valid;
    char label[kSigLabelLen];
};

struct SigBand {
    uint16_t lo_mhz, hi_mhz, points;
    uint8_t dwell_cs;
    char name[4];
};

/* SigClass::rx */
enum : uint8_t {
    RX_BLE = 0x01,
    RX_WIFI = 0x02,
    RX_DID = 0x04,
    RX_PAT = 0x08,
    RX_ZW = 0x10,
    RX_RID = 0x20,   /* Remote ID (BLE and Wi-Fi) as detections: RX,rid */
    RX_DJI = 0x40,   /* DJI BLE adverts (presence): RX,dji */
    RX_RFCLS = 0x80, /* the sweep's drone classifiers (FPV video, ELRS / FHSS, toys, ...): RX,rfcls */
};

struct ZwRule {
    uint32_t f_khz;
    uint8_t kind;
    uint8_t valid;
    char label[kSigLabelLen];
};

/* FskRuleDef::post */
enum : uint8_t {
    FSK_POST_NONE,
    FSK_POST_ZWAVE, /* Z-Wave header checks + network table (sigdet_zwnet), the ZW rule */
};

/* One FSK rule as the app keeps it; the M4's part (sigdet::FskRule) is in its
 * dwell's plan: fskg[group].dwell.rules[slot]. */
struct FskRuleDef {
    uint32_t f_khz;
    uint32_t rate;    /* bits/s */
    uint32_t chip_hz; /* rate x chips per bit */
    uint16_t bw_khz;
    uint16_t dev_lo, dev_hi; /* kHz, 0-0 = not given */
    uint16_t line;
    uint8_t decim, ma;
    uint8_t kind, valid;
    uint8_t min_frames; /* frames before it is listed */
    uint8_t post;       /* FSK_POST_* */
    uint8_t group;      /* its dwell, SigClass::fskg */
    uint8_t slot;       /* ... and its place in it */
    uint8_t check;      /* sigdet::FskCheck */
    char label[kSigLabelLen];
};

/* One FSK dwell: a frequency, a front end and up to 4 rules. */
struct FskGroup {
    uint32_t f_khz;
    uint16_t bw_khz;
    sigdet::FskDwell dwell;
};

struct SigReport {
    uint16_t lines;
    uint8_t ble, nb, bands, pat, zw, fsk, scan; /* rules loaded */
    uint8_t bad;                                /* lines rejected */
    uint16_t bad_line;                          /* first rejected line (1 based), 0 if none */
    bool truncated;
    char why[20];
};

/* One class as loaded (lives in the M4 RAM PSDT lends the app). */
struct SigClass {
    char name[kSigNameLen];
    uint8_t nble, nnb, nbands;
    uint8_t rx;
    uint16_t band_bits; /* bit b: RfBand b swept */
    BleRule ble[kSigMaxBle];
    NbRule nb[kSigMaxNb];
    SigBand bands[kSigMaxBands];
    ZwRule zw;
    uint8_t nfsk, nfskg;
    int8_t zw_rule; /* the ZW rule's index in fsk[], -1 none */
    FskRuleDef fsk[kSigMaxFsk];
    FskGroup fskg[kSigMaxFsk];
    uint8_t nscan;
    ScanRange scan[kSigMaxScan]; /* a class with SCAN lines sweeps these ranges, not its bands */
    SigReport report;
};

/* The built-in Z-Wave R1 rule (the ZW line), as FSK fields. */
extern const char* const kZwFskFields;

/* Reads a class file in pieces of any size: begin(), feed() ..., finish().
 * PAT lines go into the pattern table (begin_file without the built-in toy
 * entry). Bounded: kSigMaxLines lines, the per type limits. */
class SigParser {
   public:
    void begin(SigClass& cls, PatternTable* pat);
    void feed(const char* data, size_t n);
    void finish();

    /* One line (no line end) into the class; nullptr if fine, else why
     * (exposed for tests). */
    const char* parse_line(const char* line, size_t len);

   private:
    SigClass* cls_{nullptr};
    PatternTable* pat_{nullptr};
    char line_[kSigLineMax + 1]{};
    size_t len_{0};
    bool long_{false};
    FskRuleDef fsk_def_{}; /* an FSK / ZW line being parsed (off the stack) */
    sigdet::FskRule fsk_m4_{};

    void end_line();
    void reject(uint16_t line, const char* why);
};

/* What one advert holds (AD structures walked once). */
struct AdvInfo {
    static constexpr size_t kMaxUuid = 4;
    static constexpr size_t kMaxSd = 3;
    const uint8_t* mfr; /* manufacturer data incl. the company ID */
    uint8_t mfr_len;
    uint8_t nuuid, nsd;
    uint16_t uuid[kMaxUuid];
    uint16_t sd_uuid[kMaxSd];
    const uint8_t* sd[kMaxSd]; /* service data after the UUID */
    uint8_t sd_len[kMaxSd];
    const uint8_t* name;
    uint8_t name_len;
};
void adv_parse(const uint8_t* adv, size_t len, AdvInfo& out);

/* Index of the first BLE rule of the class matching the advert, -1 if none. */
int sig_ble_match(const SigClass& cls, uint8_t pdu_header, const AdvInfo& a);

/* Per NB rule state of a sweep dwell: which looks had energy. */
struct NbState {
    static constexpr size_t kLooks = 256;
    uint16_t x0, x1; /* points of the rule's width, 0 = not in this dwell's band */
    uint16_t n, on;  /* looks, looks with energy */
    int16_t max_cdb; /* strongest point over floor seen */
    uint8_t bits[kLooks / 8];
    bool active;
};

struct NbResult {
    bool match;
    uint8_t duty; /* % */
    uint8_t runs;
    uint16_t burst_ms;  /* mean, 0 = no whole burst */
    uint16_t period_ms; /* mean start to start, 0 = fewer than two starts */
    int16_t max_cdb;
};

/* Before the first look of a dwell of `band` (the engine has begun it). */
void nb_begin(const SigClass& cls, NbState* st, uint8_t band, const RfEngine& rf);
/* One complete look (centi-dBFS points), after RfEngine::add_look(). */
void nb_look(const SigClass& cls, NbState* st, const int16_t* cdb, const RfEngine& rf);
/* After the dwell: does rule i hold (looks over dwell_ms)? */
NbResult nb_eval(const NbRule& r, const NbState& st, uint16_t dwell_ms);

} /* namespace ui::external_app::signature_detect */

#endif /*__SIGDET_SIG_HPP__*/
