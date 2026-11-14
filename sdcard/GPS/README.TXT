GPS APP - SD CARD FILES (receive only)
======================================

The GPS app (Receive menu, HackRF Pro) reads these files from /GPS/. They
come with the firmware's SD card files: nothing has to be downloaded to use
the app.

  /GPS/ALMANAC.TXT   GPS almanac, YUMA format     bundled. Sky plot, search,
                                                  and the COARSE fix
  /GPS/TLE.TXT       two-line element sets (TLE)  bundled. Passes (ISS etc.)
  /GPS/SATFREQ.TXT   satellite downlinks          bundled. Follow (listen,
                                                  meter, Doppler corrected)
  /GPS/EPHEM.RNX     GPS broadcast ephemeris,     OPTIONAL, not bundled: a
                     RINEX 2 or 3 navigation file  same-day file makes the
                                                  fix PRECISE (~20 m)
  /GPS/EPH_AIR.TXT   ephemerides decoded off the  WRITTEN BY THE APP
                     air (Autonomous method)      (Autonomous); read at the
                                                  next start, valid ~4 h

Press "Load" on the Passes tab after copying new files (they are also read
when the app opens).

Baseband images (/APPS)
-----------------------
The receiver's signal processing comes from two files in /APPS, next to the
.ppma apps, built with the firmware (they are in the APPS folder of the
firmware package, portapack-mayhem_OCI.ppfw.tar):
  /APPS/GPS_L1A.BIN   L1 acquisition: the snapshot search (Heard, Fix)
  /APPS/GPS_L1T.BIN   L1 tracking: the Autonomous method's tracking loops
Like the .ppma apps they must come from the same firmware build: copy the
whole APPS folder after every firmware update. The app loads the search
image when it opens and swaps to the tracking image (and back) by itself;
the swap takes a fraction of a second. If a file is missing, damaged or from
another build, the Fix and Heard tabs say so in red ("GPS_L1A.BIN:
missing", "... other firmware version", "... damaged") and searching is
off; the Passes tab still works. Copy the files and press Get fix (or
Search) again.

The bundled almanac
-------------------
ALMANAC.TXT is the US Coast Guard NAVCEN almanac downloaded 2026-11-13: GPS
week 2445, time of applicability 61440 s = 2026-11-15 17:04 UTC. The app
shows its age in days (Passes tab: "Alm 31 3d"; Fix tab: "almanac 3 d old").
How good a coarse fix is, by almanac age (measured with this app's own code
against a day of broadcast ephemerides, 2020 and 2026, 7 satellites heard):
  under 2 days   about 0.4 km typical, 0.7 km for 9 fixes in 10
  1 week         about 1.5 km typical, 3 km
  2 weeks        about 3 km typical, 10 km
  1 month        about 10 km typical, 30 km; many fixes refused
  over 30 days   refused ("Almanac is 45 d old, max 30")
So this file gives fixes until about 2026-12-15 (the Fix tab shows "until
15 Dec"), good to a km or two until about 2026-11-25: REPLACE IT ABOUT ONCE A
MONTH to keep the coarse fix working. The sky plot, the search and the
pointing guide only need the almanac to within a few degrees: an almanac
months old still works there.
To refresh it (same folder, same file name), download on a computer:
  https://www.navcen.uscg.gov/sites/default/files/gps/almanac/current_yuma.alm
  (or https://celestrak.org/GPS/almanac/Yuma/<year>/ for older weeks)
and save it as /GPS/ALMANAC.TXT. Older files with a 10-bit week number are
resolved to the week nearest the PortaPack's date.

Better accuracy (optional): EPHEM.RNX
-------------------------------------
A broadcast ephemeris holds exact orbits, each valid for about 4 hours; a
daily file holds every 2-hour update of the day, so a file for TODAY (UTC)
covers the whole day and the Fix tab then says "Precise". An older file is
ignored ("EPHEM.RNX is 2d00 old (max 4h)") and the almanac is used. Sources:
  BKG (no login): https://igs.bkg.bund.de/root_ftp/IGS/BRDC/<year>/<day of year>/
     BRDC00WRD_R_<year><doy>0000_01D_MN.rnx.gz  (multi-GNSS; GPS records used)
  NASA CDDIS (Earthdata login): https://cddis.nasa.gov/archive/gnss/data/daily/
     <year>/brdc/brdc<doy>0.<yy>n.gz  (GPS only, RINEX 2)
Unzip (gzip) and save as /GPS/EPHEM.RNX. A PC helper does the BKG download:
mayhem-build/tools/gps_fetch_ephemeris.sh (prints the URL it uses). Only GPS
records are read; a multi-GNSS file (about 10 MB) takes a while to read.
RINEX 4 is not supported.

TLE.TXT (two-line elements)
---------------------------
Bundled: the 10 satellites of SATFREQ.TXT, from CelesTrak's GP data
downloaded 2026-11-13 16:59 UTC (one query per catalogue number: 25544 ISS,
27607 SO-50, 61781 AO-123, 44909 RS-44, 24278 FO-29, 7530 AO-7, 39444 AO-73,
68446 SO-127, 57166 Meteor-M2-3, 59051 Meteor-M2-4; epochs 2026-11-12/13).
Name line, then lines 1 and 2, repeated. Up to 12 sets are used: the first 12
in the file, except that satellites listed in SATFREQ.TXT take the place of
ones that are not, so a whole group file can be copied as TLE.TXT and the
satellites you can Follow are kept. Near-earth orbits only (period under 225
minutes).
HOW FAST TLEs GO STALE: for low satellites (ISS, the amateur satellites,
Meteor) days to two weeks. The ISS also changes orbit with each reboost, after
which an older set is minutes off at once. Pass times drift by seconds per
day at first, then faster; the Doppler correction suffers in proportion (see
"Follow"). The satellite page shows the age yellow after 7 days and red "too
old to point by" after 14; the Passes tab says "TLE 20 d old: refresh (max
14)". Refresh about weekly before you go out, by copying (no login):
  https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=TLE
  (all amateur satellites; the SATFREQ ones are kept, see above) and, for the
  two Meteors, https://celestrak.org/NORAD/elements/gp.php?GROUP=weather&FORMAT=TLE
  Simplest: one set per satellite, pasted one after another into one file:
  https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE  (ISS)
  (replace 25544 with each NORAD number of SATFREQ.TXT)
Save as /GPS/TLE.TXT (a group file holds over 12 sets: copy the amateur
group, then append the two Meteor sets at the end; the table's satellites
are found wherever they are in the file). CelesTrak asks for one download per
data update (a few hours).

Antenna power
-------------
GPS needs an ACTIVE L1 antenna (one with its own amplifier) on the HackRF's
antenna port, powered by the port (3.3 V, up to 50 mA). The app never
changes the antenna-power setting, not on entry, not on exit: you do, with
Mayhem's antenna-power icon in the status bar (or Settings > Radio). Like the
other receive apps, the GPS app keeps the receiver on while it is open, so
your setting is applied to the port (an LNA's LED lights) the whole time.
Top right shows what is applied ("Ant: ON" / "Ant: off"), as does Mayhem's
status-bar icon. Never turn it on with a passive antenna fitted (some are a
DC short). Use the antenna outdoors with open sky.

Position and time
-----------------
Passes, the sky plot and the fix need your rough position and UTC:
- Position: /SETTINGS/position.ini, shared by all apps. Set it with "Pos" on
  the Passes tab (degrees / minutes / seconds, saved as "Manual"; your town
  is close enough), with Signature Detect's Setup "Save", or with the GPS
  fix itself ("Save pos").
- Time: the PortaPack clock (Settings > Date/Time) and the "Zone" field on
  the Passes tab (hours and :00 / :30 / :45, saved). Zone = your local time
  zone, the offset of the clock from UTC: -5 :00 for US Eastern, 1 :00 for
  CET, 5 :30 for India. Mayhem has no system time zone, so the app keeps its
  own. With Mayhem's daylight-saving setting ON (Settings > Date/Time), enter
  the STANDARD offset (-5 for Eastern all year): the clock hardware keeps
  standard time and the app uses it. With that setting OFF, the clock shows
  whatever you set: enter the offset in force (-4 in US summer, -5 in winter).
  The tab shows "Local" (the clock) and "UTC" (what the app computes with);
  with Zone 0 it warns "Zone 0: clock must be UTC!". Check the UTC line
  against a phone. The fix needs UTC within about 30 s.

Autonomous fix (Fix tab, method "Autonomous")
---------------------------------------------
No files needed: the receiver tracks the satellites and decodes their orbits
(ephemerides) from the signal. Get fix: search ("Searching n/m"), a COARSE fix
from the almanac at once if one is on the card, then "Upgrading: n of 4
ephemerides" while it tracks (each satellite needs about 30 s of clean signal),
then "PRECISE" (a few metres; "Err est." is an estimate). Keep it outdoors with
open sky and the active antenna powered. Stop works at any time.
While it tracks, the app takes GPS time from the satellites (the clock and Zone
are not needed): from then on passes, the sky and the search use GPS time, so a
wrong clock cannot block a fix or the predictions. The Fix tab shows it with
the clock's error ("clock +0.4s" = the PortaPack clock is 0.4 s ahead).
"Set clock" (Fix tab, top right) writes the PortaPack clock from GPS time with
your Zone applied, only when you press it (the app never changes the clock by
itself); without GPS time yet it says so and changes nothing. Every decoded ephemeris is appended to /GPS/EPH_AIR.TXT (three
lines each; the file restarts when it passes 48 KB); within about 4 hours the
next start is precise at once. The app never writes EPHEM.RNX: a file you copy
there is read as before, and per satellite the record closest to now is used.
Autonomous skips the second confirmation snapshot: each detection goes
straight to a tracking channel (Heard shows it yellow with "?" until then),
and the loops verify it: a channel that is not phase locked at 28 dB-Hz for a
second (or bit-synced) within 10 s is freed ("not verified: freed") and
given to the next detection. Corrections: every fix uses a troposphere model
(standard atmosphere at the fix's height, "T" after HDOP on the Fix tab) and,
once known, the broadcast ionosphere model ("T+I"): its coefficients come
from the satellites (subframe 4 page 18, every 12.5 minutes) and are kept in
EPH_AIR.TXT for 7 days; until then no ionosphere correction is applied.
Satellites below 5 degrees are left out while 5 or more stay above. Once there is a fix, satellites rising above
10 degrees get an aided search (code phase and Doppler predicted from the fix
and the orbits) in a free channel or one extra search slot ("rising:
searching" on Heard). The Heard tab shows the tracked satellites:
C/N0, L (phase lock), B (bits), T (time decoded), E (orbit decoded); yellow
rows are not verified yet ("verifying").

Diagnostics log (Fix tab, "Log" box; a development aid)
-------------------------------------------------------
With "Log" ticked (saved; off by default) the app writes /LOGS/GPS_<yyyymmdd>_
<hhmmss>.CSV (clock time at app start), a new file each time the app opens.
Rows are queued in RAM and written a little at a time; the file is flushed
every 3 s, so pulling the battery loses at most a few seconds. A card error
shows "LOG ERROR: ..." in red on the Fix tab's bottom row; the app keeps
running and tries again every 10 s. Expect roughly 1-2 KB per second while
tracking. The once-a-second SEC row also goes to the USB serial shell after
the shell command "asyncmsg enable".

Every row starts with ms (since power-on), utc (unix seconds: from the
satellites once the app has GPS time, else clock minus Zone) and a type. The
file starts with a header line and one "#TYPE,..." line per row type. Types:
  START   fw build, LNA, VGA, amp, antenna power as read (0/1), Zone minutes,
          saved position (valid, source, age s, accuracy m, lat/lon 1e-7 deg,
          alt m), almanac count and age (days x10), ephemerides, TLEs, clock
          (yyyymmddhhmmss as shown), clock UTC (unix s), free heap, lent RAM
          bytes, Fix method (0 snapshot, 1 autonomous), tab, file name
  SEC     every second: job (0 none, 1 Heard, 2 Fix, 3 Autonomous), done,
          tracking, search (1 assisted, 2 full, 3 full after assisted), baseband
          state (0 idle, 1 capturing, 2 searching, 3 refining, 4 done,
          5 tracking), level (0 ok, 1 too low, 2 clipping), confirming, PRNs
          done / requested, progress %, input level (LSB x10), clipping
          (per mille), M4 cycles per search step, steps, search number,
          snapshot A->B interval ms, solving, solve kind (0 snapshot, 1 PVT),
          solve pending, fix ok, fix mode (1 precise, 2 coarse), fix tried,
          heard, heard with an orbit, error estimate m, time to precise fix s,
          time to first fix s, tracked, with TOW, with ephemeris, GPS time
          known, GPS minus clock ms, free heap, log rows, rows dropped (RAM
          full), log errors, ephemerides cached, cache write errors,
          ephemerides held, almanac entries, antenna power as read
  M4      every second while tracking: M4 cycles per 1 ms buffer (mean, max;
          200000 = 100 %), buffers handled, buffers missed, channels in use,
          hand-overs, age of the search result at the hand-over (ms),
          subframes published, measurement sets published, input level
          while tracking (AC rms LSB x10, DC I and Q x10, clipping per mille)
  CH      every second per channel: channel, PRN, state (1 pull-in, 2 track,
          3 lost: searching, 4 given up), loop (0 FLL, 1 PLL wide, 2 PLL narrow,
          3 search), flags (L phase lock, B bit sync, T subframe time,
          V verified, E verified since the hand-over, R released: never
          verified, channel freed, A aided search for a newly risen
          satellite), C/N0 x10, phase lock indicator x100 (cos 2 phi; ~100 =
          locked), Doppler Hz x10, code phase chips x64, code discriminator
          chips x1000 (code lock: near 0), ms in track, bits, parity-good
          words, parity failures, subframes, last TOW s, losses of lock,
          search sweeps since the last loss, 500 Hz false locks corrected
  SR      each search result (after every search): search number, search
          kind, snapshot ms, PRN, searched, detected, seen once only, refined,
          peak/mean x100, threshold x100, C/N0 x10, Doppler Hz, code phase
          chips x1000, verdict (accepted / detected (unconfirmed: the
          tracker verifies; Autonomous) / rejected: below threshold /
          rejected: 2nd snapshot did not find it / Doppler moved / code phase
          off the prediction) and, when rejected that way, what the 2nd
          snapshot found (peak/mean x100, Doppler Hz, code phase x1000)
  HO      hand-over to tracking: count, age of the search result ms, PRNs
  CLK     GPS time: event (tow: from the first decoded time, pvt: from a fix,
          logged once and again when the offset moves by 0.1 s or more; set:
          the RTC written by "Set clock"), GPS - RTC ms, both taken at the same
          instant to ~10 ms (the RTC's second from its 1 Hz tick; set: the
          step written),
          RTC UTC s, GPS UTC s (set: the value written), Zone minutes
  ADD     aided search for a newly risen satellite (every 5 s at most, when
          a channel or the extra search slot is free): PRN, elevation deg,
          predicted code phase chips x1000, predicted Doppler Hz (with the
          clock term), the clock term Hz, windows of 15 half chips either side,
          age of the fix used ms, orbit used (eph / alm), aided starts so far
  REL     a channel freed because its detection never verified (Autonomous):
          PRN, ms from its start, releases so far, spare detections started
          in freed channels so far
  SF      each parity-good subframe: PRN, channel, subframe 1-5, page,
          TOW of its start s, the ten 24-bit data words (hex)
  EPH     decoded / cached events (PRN, subframes held 1+2+4, not yet saved,
          in the orbit table, now - toe s) and every 5 s the list held
          ("Gnn:subframes:unsaved:in table")
  MEAS    each measurement set (1/s): number, sample (x2048), missed, count,
          per channel prn:flags:transmit ms of week:fraction (hex, 2^-32 ms):
          Doppler Hz:C/N0 x10
  SOLVE   a solve starts: kind (snapshot_coarse / snapshot_precise /
          pvt_coarse / pvt_precise), satellites, PRNs (with code phase for a
          snapshot), time (s of day for a snapshot: when its samples were
          taken, receive-time guess ms for a PVT), prior lat / lon 1e-7 deg,
          residual limit m, clock limit s, almanac age days x10; snapshot: how
          long ago its samples were taken, ms (the search's duration)
  FIX     its result: kind, ok, why (ok / not tried: ... / rejected: ...),
          satellites used, dropped by RAIM, residual m x10, HDOP x10, VDOP x10,
          clock m, time correction ms (snapshot) or TCXO ppb (PVT), lat / lon
          1e-7 deg, alt m, speed cm/s; PVT also: satellites below the 5 deg
          mask left out, mean troposphere and ionosphere corrections m x10,
          ionosphere source (0 none, 1 decoded off the air, 2 cached)
  IONO    the ionosphere source in use changed: none / air / cache, age of
          the cached set h, alpha0-3 x1e9, beta0-3
  SCREEN  the Fix tab's rows (joined by |) whenever they change, with the
          tab shown; HEARD the Heard tab's rows likewise
  ERR     card, ring or solver problems (what, detail)
  LOG     the log file reopened because the Log box was ticked again (event,
          errors so far); after a card error the reopen is an ERR row
  IMG     a baseband image loaded from /APPS: image (GPS_L1A / GPS_L1T),
          result (ok / missing / read error / not an image / wrong image /
          other firmware version / too large / damaged), bytes, load time ms
          (card read and start), loads so far
The log holds your position: keep it to yourself.

Heard tab (what the receiver hears)
-----------------------------------
"Search" records a snapshot (Quick 10 ms or Deep 20 ms) and searches it for
GPS satellites. With the almanac, a position and the time it searches only
the satellites that should be up, around their expected Doppler (faster);
without them all 32 (slow; the list says why). A satellite is reported as
heard only when a SECOND snapshot, taken after the first search, finds it
again at the same Doppler with its code phase moved as that Doppler predicts
("Confirming ..."). One-off peaks are listed grey as "G13? once, not
confirmed" and not counted. The list shows each confirmed satellite with a
C/N0 bar (dB-Hz; 38 and up is good) and its Doppler in kHz; the top line
says "Heard 3 of 6 for a fix" until there are enough.
The second line is the measured input level ("Level x.x LSB", rms per
component, DC removed). Target 4-30 LSB: below 1.5 LSB the ADC is starved
and the app does not search ("Signal too low"); below 4 the list ends with
"Level 2.1: target 4-30" and the gains in use (gains saved by an older
version or another app carry over: raise them). "CLIP" means lower them.
Starting gains: LNA 40, VGA 46, AMP off.

Fix tab (your position)
-----------------------
"Get fix" records 20 ms, searches the satellites that should be up, confirms
them and solves the position. 6 or more satellites are needed (the sixth
checks the other five). Two kinds of result:
- COARSE (default, from the bundled almanac): "COARSE fix from the almanac",
  "almanac 3 d, est. +/-600 m" (about 2 fixes in 3 are within the estimate).
  Before a fix the tab shows the almanac's age and "until 15 Dec; refresh
  monthly": after that date the coarse fix is refused until ALMANAC.TXT is
  replaced (see "The bundled almanac").
- PRECISE (with a same-day /GPS/EPHEM.RNX): "Err est. ~15 m".
The solver runs in the background: "Solving..." can take a few seconds and
the screen and touch keep working ("Stop" cancels). "Save pos" writes
/SETTINGS/position.ini (source "HackRF GPS", with the estimated error as
acc_m: e.g. 600 for a coarse fix), which Signature Detect's Setup "Load" and
the Passes tab use. When something is missing the tab says what, why and
what to do (no position, no almanac, almanac too old, too few satellites).
"Autonomous" (decoding the orbits from the satellites, no files) is not
available yet; the tab says so.

Passes tab
----------
Top: your position, the clock (Local), UTC, Zone, what was loaded ("Alm 31
3d Eph 0 TLE 5": counts, almanac age). Sky plot ("GPS now"): GPS satellites
above the horizon (green, PRN numbers) and TLE satellites (orange). Right:
GPS satellites up (PRN, elevation, azimuth). Bottom: the next pass of each
TLE satellite in the next 24 h, soonest first: "in 1h05 47d 9m" = rises in
1 h 05 min, highest 47 degrees, visible 9 min ("UP" = above the horizon now;
"search 35%" while it is being computed). Four rows show at a time; a "+" at
the end of the last row means more below: the wheel (sky plot selected)
scrolls through all of them. The sky is recomputed every few seconds, one
satellite per display frame, so touch stays responsive.

Selecting a satellite, pointing guide
-------------------------------------
On Passes and Heard, touch the sky plot (a yellow frame shows it has the
wheel) and turn the wheel: it steps through the satellites listed (TLE pass
rows, then the GPS satellites up; on Heard the heard ones). The selected one
is highlighted in the list, ringed on the plot, and its pass is drawn as a
cyan arc. Select (or a second tap on the same row) opens its page:
- where to point now in big letters ("NW 310 deg", "24 deg up") and in
  30 s, and a countdown ("rises in 5m12", "peak in 2m05", "sets in 7m30");
- the pass arc with the satellite ringed where it is now;
- a plain summary in local time: "Rises 14:02 NW 310", "Peak 14:07 42 up,
  SW", "Sets 14:12 SE 135" (and the rise time in UTC);
- details: GPS satellites: health, orbit source and age, Doppler at L1,
  heard or not (C/N0, how long ago), range; TLE satellites: TLE age, range,
  closing / receding speed, highest elevation and duration.
Point the antenna to the compass bearing (true north; a phone compass shows
magnetic north, which differs by a few degrees in most places) and tilt it up
by the elevation. TLEs of low satellites go stale in days to weeks: after
7 days the age shows yellow, after 14 days red "too old to point by".
For a satellite listed in SATFREQ.TXT the page also shows its downlinks
("437.800 NFM UHF") and a "Follow" button. Back returns to the tab.

Follow (listen to a satellite and point by ear)
-----------------------------------------------
"Follow" on the satellite page closes the GPS part of the app and opens the
Follow screen on the stock audio receiver (NFM, AM / USB / LSB, or WFM, as
SATFREQ.TXT says). "GPS" (bottom right) goes back to the GPS app, which reads
the files again and selects the satellite; the back arrow in the title bar
goes to the menu. Receive only; the antenna-power setting is never changed
(top right shows it, red when ON).
- Line 2: the satellite and which downlink (touch it to switch, e.g. ISS
  repeater / voice / APRS).
- Line 3: the frequency as received now, corrected for Doppler 4 times a
  second ("437.80823 Dop +8.23k": the satellite comes towards you, so you
  hear it higher; at the peak the shift passes through 0, as it moves away it
  goes negative). The tuner is set once; the shift is followed inside the
  audio baseband (the stock "sliding" frequency shift of the NFM and AM
  receivers), so there are no retuning clicks. WFM has no such shift and is
  retuned in 1 kHz steps.
- Vol (volume), Sq (squelch in dB of the meter: silent below it; at -120 the
  squelch is off), Snd: Voice (the receiver's audio) or Tone (beeps: silent
  below Sq, a slow low beep just above it, faster and higher as the signal
  rises, a steady high tone 24 dB above Sq). Tone uses the NFM receiver
  whatever the mode, so it also works for CW beacons and digital signals.
- The pointing readout as on the satellite page ("NW 310 deg", "24 deg up",
  "sets in 4m12, max 47"), so you need not switch screens while holding the
  antenna.
- The meter: level in dB (relative to full scale, gain dependent: compare
  only at the same gains), peak hold (falls 10 dB/s; white mark on the bar),
  min and max since Reset (yellow marks), Sq (orange mark), and a 40 s history
  strip (one column per 0.2 s).
- Hint "Fading high up: rotate antenna": shown for 15 s (then not again for
  90 s) when the level falls smoothly by 6 dB or more over 3 s or more while
  the satellite is at least 15 deg up and the signal is still above the noise.
  It is a guess: satellites tumble and their antennas are linear, so the
  polarisation turns; rotating a linear panel by 90 deg often brings it back.
  A transmitter that just stops (repeater idle, ISS crew not talking) drops
  in one step and gives no hint.
- "TLE 3 d, Doppler err ~0.3 kHz": the estimated tuning error, from a clock
  error of 5 s (the PortaPack clock is set by hand and drifts), the TLE age
  (about 1 km + 1.5 km per day along the orbit) and a 5 km position error, at
  the present Doppler rate. Measured on an ISS pass (max 13 deg) with this
  app's code: 5 s of clock error gives up to 280 Hz at 437.8 MHz (93 Hz at
  145.8), 10 km of position error up to 70 Hz; at the peak of a high pass the
  Doppler changes up to ~180 Hz per second at 437.8 MHz, so there 5 s of clock
  error is ~0.9 kHz. NFM voice tolerates about 1-2 kHz; SSB and CW are
  touchier (retune by ear with the wheel in the stock Audio app if needed).
  The radio's own reference error adds to this (1 ppm = 438 Hz at 437.8 MHz;
  Settings > Radio has the frequency correction). Set the clock against a
  phone before a pass.
- Gains: LNA / VGA / AMP as in every receive app, saved for Follow
  (rx_gpsfol.ini). Start: LNA 40, VGA 32, AMP off; with a passive panel and
  no LNA, try AMP 1 if the level does not rise when you point at the
  satellite.
How to use it: before the pass, open the satellite, press Follow, set Sq
just above the level you see with the antenna pointed at empty sky, choose
Tone. As the countdown reaches "rises", point at the bearing and elevation
shown and sweep slowly until the beeps get faster and higher; then switch
Snd to Voice to listen.

Downlinks (SATFREQ.TXT)
-----------------------
Format: see the comments at the top of the file. Each line: NORAD number,
name, downlink (MHz with a point, or Hz), mode, bandwidth kHz (or "-"),
note. Edit it with any text editor; a line the app cannot read is skipped
and the satellite page says "SATFREQ line 7 bad". Below 300 MHz (VHF: 137 and
145 MHz) a 300 MHz-6 GHz panel does not receive: those entries are kept for a
VHF antenna and marked VHF on screen.
Bundled entries, checked 2026-11-13 (AMSAT reports are from
https://status.amsat.org/, page of 2026-11-12/13 with reports Nov 8-13):
- ISS 437.800 NFM, FM cross-band repeater (uplink 145.990 PL 67): active, ARISS
  station status of 2026-11-04 ("voice repeater mode",
  https://www.ariss.org/current-status-of-iss-stations.HTML) and AMSAT reports
  heard 2026-11-13. Silent unless someone uses it; pre-empted by crew events.
- ISS 145.800 NFM, voice / SSTV: event-based only. SSTV reported heard this
  week (AMSAT); the ARISS status page lists no current event. The frequency is
  the long-standing ARISS downlink, not restated on that page (uncertain).
- ISS 437.825 NFM, APRS packets: active per ARISS (2026-11-04: "APRS is
  currently active on 437.825 MHz") and AMSAT (heard 2026-11-13). The old
  145.825 MHz VHF digipeater was NOT heard this week: not listed.
- SO-50 436.795 NFM: active, AMSAT reports heard 2026-11-13; frequency from the
  AMSAT FM satellite summary (page dated 2026-10-31,
  https://www.amsat.org/fm-satellite-frequency-summary/).
- AO-123 (ASRTU-1) 435.400 NFM: active, AMSAT reports 2026-11-13; frequency
  from the AMSAT FM summary and SatNOGS (https://db.satnogs.org/satellite/61781).
- RS-44 beacon 435.605 CW and passband 435.610-435.670 USB (centre 435.640):
  active, AMSAT reports 2026-11-13; AMSAT linear summary (2026-10-31,
  https://www.amsat.org/linear-satellite-frequency-summary/). The beacon is the
  best pointing target: steady.
- FO-29 beacon 435.795 CW and passband 435.800-435.900 USB: active by schedule
  and in sunlight (AMSAT, 2026-10-31; reports 2026-11-13).
- AO-7 mode B passband 145.925-145.975 USB (centre 145.950): reported this week,
  sun only, alternates with mode A (29 MHz, outside the HackRF's range) on a
  24 h timer. VHF.
- AO-73 (FUNcube-1) 145.935 BPSK telemetry beacon: telemetry reports this week
  (AMSAT). VHF.
- SO-127 (HADES-SA) 436.875 FSK / SSDV data: a few AMSAT reports this week
  (newest 2026-11-13); launched 2026-03-30; SatNOGS
  (https://db.satnogs.org/satellite/68446). UNCERTAIN: how often it transmits.
- Meteor-M2-3 and M2-4, LRPT 137.900 MHz (M2-4 backup 137.100): digital
  (OQPSK, 72 ksym/s per a 2026-09-23 decode report and sigidwiki of
  2026-10-06; SatNOGS / WMO OSCAR also list 80k: uncertain). M2-3's antenna is
  reported not fully deployed (weak late in passes). Followed and metered
  only, not decoded here. VHF: below a 300 MHz panel's range.
Not listed (checked 2026-11-13): NOAA-15, -18 and -19 APT - all three were
decommissioned in 2025 (NOAA-18 2025-06-06, NOAA-19 2025-08-13, NOAA-15
2025-08-19; NOAA NESDIS, https://www.nesdis.noaa.gov/news/legacy-orbit-noaa-decommissions-the-poes-satellite-constellation);
AO-91 (battery: mostly "not heard" this week); AO-27, PO-101, JO-97 (no
fresh reports, "dying", or conflicting); IO-86 (equatorial: only within
about 30 deg of the equator); SO-121, SO-124, SO-125 (re-entered).
Satellites fail and change: check https://status.amsat.org/ before a pass.
To refresh the table, edit it by hand from the AMSAT pages above; it is not
downloaded. Keep TLE.TXT holding a set for every NORAD number in it.
