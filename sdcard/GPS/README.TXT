GPS APP - SD CARD FILES (receive only)
======================================

The GPS app (Receive menu, HackRF Pro) reads these files from /GPS/. They
come with the firmware's SD card files: nothing has to be downloaded to use
the app.

  /GPS/ALMANAC.TXT   GPS almanac, YUMA format     bundled. Sky plot, search,
                                                  and the COARSE fix
  /GPS/TLE.TXT       two-line element sets (TLE)  bundled. Passes (ISS etc.)
  /GPS/EPHEM.RNX     GPS broadcast ephemeris,     OPTIONAL, not bundled: a
                     RINEX 2 or 3 navigation file  same-day file makes the
                                                  fix PRECISE (~20 m)

Press "Load" on the Passes tab after copying new files (they are also read
when the app opens).

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
So this file gives fixes until about 2026-12-15, good to a km or two until
about 2026-11-25. The sky plot and the search only need the almanac to
within a few degrees: an almanac months old still works there.
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
Bundled: CelesTrak's "stations" group downloaded 2026-11-13 (ISS and other
crewed stations). Name line, then lines 1 and 2, repeated; up to 5 sets are
used (the first 5). Near-earth orbits only (period under 225 minutes).
TLEs age: after about two weeks pass times can be minutes off; the Passes tab
then says "TLE 20 d old: refresh (max 14)". Source (no login; CelesTrak
allows one download per data update):
  https://celestrak.org/NORAD/elements/gp.php?GROUP=stations&FORMAT=TLE
  https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE  (ISS)
Save as /GPS/TLE.TXT.

Antenna power
-------------
GPS needs an ACTIVE L1 antenna (one with its own amplifier) on the HackRF's
antenna port, powered by the port (3.3 V, up to 50 mA). The app never
changes the antenna-power setting, not on entry, not on exit: you do, with
Mayhem's antenna-power icon in the status bar (or Settings > Radio). Like the
other receive apps, the GPS app keeps the receiver on while it is open, so
your setting is applied to the port (an LNA's LED lights) the whole time.
Top right shows what is applied ("Ant: ON" / "Ant: off") and a red line
while it is on. Never turn it on with a passive antenna fitted (some are a
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
TLE satellite in the next 24 h: "in 1h05 47d 9m" = rises in 1 h 05 min,
highest 47 degrees, visible 9 min ("UP" = above the horizon now; "search 35%"
while it is being computed). The sky is recomputed every few seconds, one
satellite per display frame, so touch stays responsive.
