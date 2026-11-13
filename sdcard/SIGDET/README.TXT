SIGNATURE DETECT - SD CARD FILES
================================
(The app was called Drone Detect before 2026-11-12; its folder was /DRONEDET and
its logs DRONEDET_*.CSV. For one release it still READS classes and PATTERNS.TXT
from /DRONEDET when /SIGDET/SIGS is missing, and never writes there. Settings
move from SETTINGS/rx_drone_detect.ini to rx_sig_detect.ini on first start.
The Receive menu and the title bar show the short form "Sig Detect": the title
bar fits 14 characters at most.)

/SIGDET/PATTERNS.TXT   extra pattern scan entries for the built-in Drones class
/SIGDET/SIGS/*.TXT     signature classes, one file per class (the "Sig" wheel)
/LOGS/SIGDET_<date><time>.CSV   the log, a new file each time the app starts
                         with Log ticked (and a new part, _1, _2 ..., past 16 MB
                         or after a write error)
/SETTINGS/position.ini   the position shared by all apps (Setup Load / Save)


SIGNATURE CLASSES (/SIGDET/SIGS)
----------------------------------
The "Sig" field on the main view (turn the wheel) picks the class:
"Drones" is built in (every drone detector, as set in Setup; works without an
SD card), then one entry per .TXT file in /SIGDET/SIGS, named after the file
(up to 15 characters, up to 12 files; DRONES.TXT only documents the built-in
class and is not listed). The choice is saved by name.

A class file decides which receivers run: BLE dwells when it has BLE rules,
pattern dwells with PAT rules, energy sweeps of the bands its BAND and NB rules
need, Wi-Fi Remote ID / DJI DroneID dwells only with an RX line. Setup's
receiver and band boxes apply to the Drones class only. Remote ID is still
decoded in every BLE dwell.

One rule per line: a type word, then comma separated key=value fields. '#'
starts a comment line. Lines up to 200 characters, 300 lines. A bad line is
skipped; the status line shows "SIG ln<N>?" (yellow) and the Debug view why.
Every rule may carry valid=real|standin|doc: how it was validated (real: seen
in our own recordings; standin: tested with a bench stand-in; doc: from
documents only). It is shown in the detail view and logged.

BLE  Bluetooth LE advert rule (up to 20 per class). The first matching rule wins.
     label=   name shown and logged, up to 19 characters (the list shows 16)
     kind=    tracker light beacon phone audio sensor home mesh meter radio
              noise drone other
     cid=     manufacturer data company ID, 4 hex digits (004C = Apple)
     uuid=    16 bit service UUID (in a UUID list or as service data), 4 hex
     sdata=   16 bit service data UUID, 4 hex
     mfr=     pattern on the manufacturer data after the company ID
     sd=      pattern on the service data after the UUID (with sdata=)
     name=    prefix of the advertised name, 1-8 characters
     not=     one or two patterns, '/' separated, that exclude the advert
              (same field as mfr= / sd=)
     minlen=  that field has at least this many bytes
     addr=    public | random
     beep=    1 = beep when found (with "Beep on drone" in Setup)
     Pattern: up to 8 bytes as hex, '?' = any nibble, optional &MASK
     (sd=40&FE matches 40 and 41). At least one of cid / uuid / sdata / name.
     Example: BLE,label=Samsung SmartTag,kind=tracker,sdata=FD5A,valid=real

NB   narrowband energy rule (up to 10 per class), from the energy sweep.
     Always LOW confidence (shown with '?'): "energy at this frequency with
     this timing", nothing about the protocol.
     label=   up to 19 characters (the list shows 15 plus the '?')
     f=       centre, MHz, up to 3 decimals (required)
     bw=      expected width, kHz (required)
     db=      dB over the noise floor needed (default 8), in 2 looks at least
     duty=    lo-hi, % of the dwell's looks with energy
     burst=   lo-hi, ms of one whole burst inside the dwell
     period=  lo-hi, ms from one burst start to the next (2 starts in a dwell)
     band=    band to measure in (default: the class's bands, then the first
              built-in band that holds f)
     Resolution: points are 81 kHz apart at 915 MHz, 276 kHz at 2.4 GHz,
     ~970 kHz at 5 GHz, (hi-lo)/(points-1) in a custom band; one look of the
     band takes a few ms (915: ~1-4 ms, 2.4 GHz: ~5-10 ms), so burst and
     period are measured to about one look and only within one dwell (250-
     300 ms). Narrower than a point, or closer than two points, cannot be
     told apart.
     Example: NB,label=Mesh 925.375,kind=mesh,f=925.375,bw=250,db=10,valid=real

BAND sweep band. Built-in: BAND,name=915 (2.4 5.8 5.3 1.2 915 868 5.1).
     Custom (at most 2 per class): BAND,name=LMR,lo=450,hi=470,points=320,dwell=300
     name 1-3 characters, lo/hi MHz (span 1-100), points 64-320 (default 320),
     dwell 50-500 ms (default 250). Keep custom spans at 14 MHz or more: those
     sweep at 20 Msps like the built-in bands (narrower spans use lower rates,
     untested on the device). No drone classifiers run in custom bands.

PAT  pattern scan entry, the PATTERNS.TXT format after "PAT," (up to 16).

ZW   Z-Wave frame receiver (one per class): ZW,label=Z-Wave,kind=home,rate=9k6,f=908.42,valid=real
     Listens on f (default 908.42 MHz, US) for checksum-valid R1 frames (9.6 kb/s
     FSK, Manchester). A network is listed as "ZW <home ID>" after 2 valid frames
     of it (a single corrupted frame can pass the 8 bit checksum). Alone in a
     class it listens all the time; with other receivers it gets a 1.5 s dwell per
     cycle. rate: 9k6 only (40 kb/s and 100 kb/s are not decoded yet). Header
     fields are logged, never the payload.

RX   receivers and built-in drone detectors a class asks for:
       RX,ble,wifi,droneid,rid,dji,rfcls
     ble     BLE dwells without BLE rules
     wifi    Wi-Fi Remote ID dwells (channel 6, 1 Mb/s)
     droneid DJI DroneID dwells (every ~40 s)
     rid     Remote ID (BLE and Wi-Fi) as detections
     dji     DJI BLE adverts (presence) as detections
     rfcls   the sweep's drone classifiers (analog / digital video, ELRS,
             Crossfire, R9, FHSS, 2.4 GHz hoppers / toys) as detections
     Without rid / dji / rfcls those detectors list, log and beep nothing:
     Remote ID and DJI adverts then reach the survey as ordinary advertisers
     (BLE_SEEN with sd=FFFA for Remote ID). A class change drops every entry of
     the old class from the list (with its *_DROP row).

WHAT EACH SHIPPED CLASS RUNS
  Drones (built in)  everything, as set in Setup: Remote ID (BLE, Wi-Fi), DJI
                     BLE presence, DroneID, pattern scan (+ PATTERNS.TXT),
                     sweeps with the drone classifiers
  TRACKERS           BLE dwells, the tracker BLE rules
  HOME_IOT           BLE dwells, the light / beacon BLE rules, 2.4 GHz and 915
                     MHz sweeps (NB rules: Zigbee, Z-Wave energy), Z-Wave frames
  ISM_900            915 MHz sweep (NB rules), Z-Wave frames
  ZWAVE              Z-Wave frames only (100 % on 908.42 MHz)
  LAND_MOBILE        two custom sweep bands, NB rules
  INTERFERENCE       two custom sweep bands, NB rules
  None of the file classes has rid, dji or rfcls: no drone detections there.

LIST COLUMNS
  ID (name), Src (receiver letters; signature rows: the label continues here),
  dB, then Dist / Kind: "Dist" in the Drones class (distance of a Remote ID
  drone; pattern rows: hits/s, RF rows: confidence L/M/H), "Kind" in a class
  without Remote ID, "D/K" in a class with both. Kind tags:
    TRKR tracker  LITE light  BCN beacon  PHON phone / PC  AUD audio
    SENS sensor   HOME home device  MESH mesh radio (LoRa, Meshtastic)
    METR meter    RAD land-mobile radio  NOIS interference  DRON drone
    OTHR other
  NB (energy) hits carry a '?' in their label: low confidence.


OWN POSITION AND THE SHARED POSITION FILE
-----------------------------------------
Own position (distance to a drone, lat/lon in the log): a live GPS fix (ESP32
add-on GPS or the USB shell "gotgps") while it is fresh (30 s), else the
manual position from Setup. The main status shows "ManPos" when the manual
one is used; Setup shows where it came from and how old it is ("Map 2h").

Setup, "Manual pos:" row:
  On map   pick the manual position on the map (source Map, time now)
  Clear    forget it
  Load     copy the shared position from /SETTINGS/position.ini into the
           manual one (keeps its source and time); a POS_LOAD log row
  Save     write /SETTINGS/position.ini: the GPS fix while there is one
           (source GPS, time now, altitude, satellites), else the manual
           position with the source and time it came with; a POS_SAVE row
The SD card is used only when Load or Save is pressed.

/SETTINGS/position.ini is meant for every Mayhem app (a future GPS receiver
app writes it with source 2). Same format as the apps' .ini files, one
key=value per line, all integers:
  lat_e7   latitude, 1e-7 degree (WGS84), e.g. 508704920 = 50.8704920
  lon_e7   longitude, 1e-7 degree, e.g. -15622980 = -1.5622980
  alt_m    altitude, metres (0 = unknown)
  source   1 GPS (external module / shell), 2 HackRF as GPS receiver,
           3 typed in by hand, 4 picked on a map
  time     when the position was found, FAT date-time: bits 31-25 year-1980,
           24-21 month, 20-16 day, 15-11 hour, 10-5 minute, 4-0 seconds/2
           (0 = unknown); e.g. 2026-11-12 17:30:00 = 1567394752
  acc_m    estimated horizontal error, metres (0 = unknown)
  sats     satellites used (0 = unknown / not a GPS fix)
A file with source 0, latitude outside +-90, longitude outside +-180 or 0/0
is ignored ("No saved position on SD"). It can be written on a PC too.


LOG FILE (format 2)
-------------------
Tick "Log" on the main view. "Survey" (next to Sig) adds rows for what the app
received but could NOT classify, to write signatures from later; ticking it
ticks Log. Rows reach the card within 2 s (synced every 2 s), so a pulled
battery loses at most the last 2 s. A write error shows "LOG! <why>" in red on
the status line; the same file is reopened after 2, 4, 8, 16 s, then every
30 s (a REOPEN row), and after 3 failures in a row a new part is started. Rows
that could not be written meanwhile are counted (lost= in HB and STOP). The status line shows LOG
(detections) or SRV (survey) while the file is open.

Every row has the same 14 columns (plain CSV; no field contains a comma or a
quote). The first line holds the column names:

  uptime_ms,time,event,src,id,label,kind,freq_khz,bw_khz,level,unit,lat,lon,detail

uptime_ms  ms since the PortaPack started (monotonic; the RTC may jump)
time       RTC date and time, YYYY-MM-DD HH:MM:SS
event      see below
src        receiver letters: B on-board BLE, H on-board Wi-Fi, R energy sweep,
           P pattern scan, O DJI DroneID detector, W / E ESP32 add-on, D demo
id         UAS ID or address, RF track (band:uid), NB rule (NB<n>), band ...
label      name of what was detected (or the class, in START / CLASS / HB rows)
kind       remote_id, an RF class tag (AV DV DW CL H24 TOY), a signature kind
           (tracker, light ...), ble, energy, droneid, dji_ble, a pattern class
freq_khz   centre frequency, kHz (sweep, pattern, DroneID, NB rules)
bw_khz     width, kHz
level      number in the unit of the next column
unit       dBFS (on-board receivers, uncalibrated), dBm (ESP32), dB>floor (sweep)
lat, lon   degrees: the drone's position (RID rows); our own position (GPS or
           the manual one) in SIG, BLE_*, NRG_* and HB rows when known
detail     key=value pairs separated by ';' (keys below; empty ones left out)

Events
  REOPEN   the log reopened after an error (same detail as START)
  START    the log opened: detail format, fw (version), build (date), log
           (detections|survey), class, part, lna, vga, amp
  CLASS    the signature class changed (same detail as START)
  STOP     the log closed (app exit or Log unticked): rows, lost
  HB, HB2  heartbeat every 60 s, two rows (each under 512 bytes; HB2 starts at
           wifi_frames). detail: up_s, cyc_ms (cycle time), cpu (M4 %),
           ble_aa / ble_ok / ble_crc (BLE syncs, CRC good, CRC bad), rid_ok,
           rid_fwd, other_fwd / other_dup / other_lost (other adverts),
           dji_ok, drops, drops_ble, qf (M4->M0 queue full), qf_rid, zw_syncs / zw_ok / zw_bad (Z-Wave preambles, valid frames, bad or lost), wifi_frames,
           wifi_fcs_ok, wifi_fcs_bad, wifi_beacon / _probe / _action, wifi_rid,
           pat_hits, pat_rx, did_cands, did_dets, slices (sweep replies),
           stalls, partial, i2c_faults, i2c_recov, clk_err, entities (list),
           heap_core / heap_free (M0 bytes), rows / lost / log_err / bytes
           (this log), srv_drop (survey rows over the rate limit), srv_ble /
           srv_nrg (survey table entries), usb_drop, lna, vga, amp, gps
           (fix|nofix|none), survey, class_ok, class_bad_line, m0_stack_free_w (the
           UI thread's stack never used since boot, in 4-byte words, as sysinfo). Counters run
           from the app start.
  RID_NEW / RID_LOC / RID_DROP   Remote ID: level, unit, drone lat/lon; detail
           mac, id_type, ua_type, status, alt_geo_m, height_m, speed_ms,
           heading, vspeed_ms, op_lat, op_lon, op_alt_m, op_loc, operator_id,
           self_id
  RF_NEW / RF_CONF / RF_DROP     energy sweep track (candidate, confirmed,
           gone): label, class tag, freq, width, dB>floor; detail band, conf
           (L M H), duty (%), flat_db
  PAT_NEW / PAT_DROP   pattern scan: name, class, channel, dBFS; detail class,
           name, freq_khz, rssi, hits_per_s
  DJI_NEW / DJI_DROP   DJI Bluetooth advert (presence): address, name, match
  DID_NEW / DID_DROP   DJI DroneID bursts (presence): freq, snr_cdb, zc_milli,
           nsym, bursts
  SIG_NEW / SIG_DROP   a signature class rule (a class change drops the old
           class's detections at once, so SIG_DROP names the class they came
           from): id (address or NB<n>), label,
           kind, freq/bw (NB), level; detail class, rule (index in the file's
           BLE or NB rules), valid, frames, seen_s, max (DROP); NB: conf=low,
           duty, runs, burst_ms, period_ms
  POS_LOAD / POS_SAVE   Setup Load / Save of /SETTINGS/position.ini: label
           the source (GPS, HackRF GPS, Manual, Map), kind position, lat/lon
           the position; detail source (number), alt_m, age_s (-1 unknown),
           acc_m, sats, file
  ZW_NEW / ZW_FRM / ZW_SUM   Z-Wave (src Z): ZW_NEW once per network and
           session (when it is listed), ZW_FRM for every valid frame of a listed
           network, ZW_SUM when it has been quiet for 60 s. id = home ID (hex;
           your own network's identifier: keep these logs private), label,
           kind, freq_khz 908420, bw 200, level dBFS (the frame's mean power),
           own position; detail src, dst (node IDs), type (singlecast,
           multicast, ack, routed), routed, ackreq, seq, len, cc (command class
           byte, when the frame has one), rep / hop (routed frames: repeaters,
           hop), nodes, frames, node_ids
  TRK_START / TRK / TRK_STOP   the Track view (fox hunt), TRK once a second:
           src M (meter at a frequency) or B (BLE target); id frequency kHz,
           address or rule<n>; label the target; kind track; freq_khz /
           bw_khz (meter); level = strongest dBFS of that second; lat/lon own
           position; detail peak_hold, min, max (dBFS), lna, vga, amp, clip
           (1 = near full scale in the last 2 s), meter: mean, floor,
           pulse_ms (median pulse start to start), width_ms, reports; BLE:
           pkt_s
  Survey rows (Survey ticked; at most 30 at once, then 1 a second):
  BLE_SEEN an advertiser no detector or rule took, first heard (once per
           address per session: the last 64 addresses reported are remembered,
           so one evicted and heard again gets no second BLE_SEEN; a class
           change clears the survey tables, so it starts again): id address,
           level = strongest dBFS; detail pdu (the PDU types heard, '+'
           separated: ADV_IND, ADV_NONCONN_IND, ADV_SCAN_IND, SCAN_RSP; a
           scan response is the same entry and fills in its data), addr_type (public|random), cid (company
           ID, hex) and mfr (first <= 8 bytes after it, hex), uuid (16 bit
           service UUIDs, '/' separated), sd (service data UUID), len (AdvData
           bytes), name (1 = it has a name; the name is not logged), n (frames
           received), n_new (since its last row), rssi_min, rssi_max,
           gap_min_fwd_ms (shortest gap between frames forwarded: the M4
           passes an unchanged advert at most once a second, so values under
           1000 only appear for changing payloads; it is not the advertising
           interval), seen_s, ch (37+38+39 heard on); sd_data = the first <= 8
           bytes of the service data after its UUID (hex)
  BLE_SUM  the same, every 5 min while still heard, when its table slot is
           needed (32 slots by address, no time expiry: the one heard longest
           ago goes), and at the end
  NRG_NEW  energy in a sweep dwell that no confident detector took (no label or
           a low confidence one; mean >= 4 dB, or max hold >= 10 dB in 2 looks
           or more), once seen in 2 dwells (once per band and 1 MHz cell per
           session, last 64 remembered): id band, label the
           class tag of a wide segment over it (e.g. WFI = Wi-Fi traffic),
           freq (strongest point), bw (width of the run of points), level max
           hold dB>floor; detail band (custom bands of a class also lo_mhz,
           hi_mhz, class), mean_db, duty (% of looks, max of the dwells),
           dwells, dwells_new, seen_s, conf, pt_khz (point spacing)
  NRG_SUM  the same every 5 min while seen again (16 slots), and at the end

Size: detections only, a few kB an hour without drones (HB rows ~0.6 kB a
minute: ~40 kB/h). Survey: the first build measured 1.15 MB/h in testing (1304
rows in 17 min, most of them repeated first-seen rows); with one BLE_SEEN per
address and one NRG_NEW per cell per session it should be ~0.1-0.2 MB/h at
home. The rate limit caps survey rows at ~3,600 an hour (~0.8 MB/h, a crowd),
so an 8 hour night is at most ~7 MB.


CAPTURES (IQ for the PC tool)
-----------------------------
Start a capture from a detection's detail view (Cap), from the Track view (Cap)
or from the main view's Target button (a typed frequency, or one of the survey's
last unknowns: NRG energy events and unmatched BLE advertisers, newest first;
Survey must be on to collect them). Signature Detect closes and its capture view
opens with the stock capture baseband: the frequency field, gains, the stock
bandwidth (= sample rate) options, the stock RecordView (record button, file
name, time left on the card) and the stock waterfall. "Detect" returns to Drone
Detect (a new log part is not started: a new session file).

  Rate:   preset from the target (2.5 x its width, at least 500k; BLE 2000k;
          Z-Wave 500k), any stock option can be chosen. C16 below 1500k, C8
          from 1500k on (as the stock Capture app). Bytes per second: C16 = 4 x
          rate, C8 = 2 x rate (500k: 2 MB/s; 1000k: 4 MB/s; 2000k C8: 4 MB/s;
          4000k C8: 8 MB/s). The status line shows kB/s, the size of one
          capture and the free space (red when the card cannot hold it or one
          capture is over 500 MB). Above 2.5 MHz the M4 may drop samples (stock
          note); a fast card is needed above 1 MHz C16 / 2.25 MHz C8.
  Dur:    5 s, 10 s, 30 s (stops by itself) or man (stop with the button).
  device OFF / device ON:  the state of the device being captured. Record the
          baseline with OFF and the device active with ON: the two files of a
          pair share a pair number.
  Lbl:    a short label (letters, digits, - and _), Ant: an antenna note.

Files: /SIGDET/CAPTURES/<YYYYMMDD>_<HHMMSS>_<label>/ (made at the first
recording of the view; the label at that time), per capture:
  P<nn>_OFF.C16 (or .C8)  IQ, unchanged stock format: interleaved I, Q; C16
                          little endian int16, C8 int8
  P<nn>_OFF.TXT           the stock metadata: center_frequency=<Hz> and
                          sample_rate=<Hz>, plus latitude=, longitude=,
                          satinuse= when a GPS position was known
  P<nn>_OFF.SIG           the sidecar below
  (P<nn>_ON.* the same for the ON capture; a name already taken gets _2, _3)
Pairs: a capture joins the current pair unless that pair already has its state;
then pair nn+1 starts (OFF, ON -> P01_OFF, P01_ON; OFF -> P02_OFF ...).

The .SIG sidecar, format 1: key=value lines (CRLF), in this order:
  format=1
  file=P01_OFF.C16          data file (same folder)
  metadata=P01_OFF.TXT      stock metadata file
  center_hz=908420000       tuned centre frequency, Hz
  sample_rate=500000        Hz (the stock bandwidth option)
  sample_format=cs16        cs16 or cs8
  bb_filter_hz=1750000      baseband (anti-alias) filter of the capture
  lna=32 / vga=30 / amp=0   gains, dB / dB / 0-1 (three lines)
  time=2026-11-12T20:15:00  RTC at the start
  uptime_ms=1234567         PortaPack uptime at the start
  duration_ms=10000         from the data size (bytes / bytes per second)
  bytes=20000000            data file size
  dropped_pct=0             samples dropped by the stock capture thread, %
                            (0..99, the highest it showed while recording)
  state=off                 off or on
  pair=P01                  shared by the OFF and ON files of a pair
  label=door_remote
  target_kind=zwave         manual, track, rf, nb, ble, energy, zwave, pattern,
                            droneid, remote_id, dji_ble, sig
  target_label=ZW 1A2B3C4D  the detection's name (local file: addresses and home
                            IDs may appear here)
  target_extra=home=1A2B3C4D;...   key=value;... about the target: bw_khz,
                            band, bw_seen_khz, db, class, conf, class_file,
                            rule_idx, rule, cid, mfr, uuid, sd, pdu_hdr, addr,
                            channel, home, and target_hz=<Hz> (the target's
                            own frequency) when the capture was tuned off it
  survey_log=LOGS/SIGDET_20261112201000.CSV   the session log, empty if off
  survey_uptime_ms=1200000  uptime of the row / detection it came from, 0 none
  firmware=<version>
  build=<build date and time>
  antenna=<note>
Log rows (session CSV, when Log was on): CAP_START and CAP_STOP, src C, id =
P01_OFF, label, kind capture, freq_khz, bw_khz = rate / 1000; detail state,
pair, folder, file, rate, lna, vga, amp, target, and on CAP_STOP duration_ms,
bytes. (The capture view appends them to the file Signature Detect was writing.)
CAP_START is written when Record is pressed, before the recording starts (the
recording begins up to a second later); CAP_STOP after the files are saved.
CAP_FAIL (no CAP_STOP) means the recording could not start. Nothing is written
to the card from the screen while a recording runs.
