GPS APP - SD CARD FILES (receive only)
======================================

The GPS app (Receive menu, HackRF Pro) reads these files from /GPS/. The
PortaPack has no network: download them on a computer and copy them to the
card. All are plain text; the app keeps only what it needs in memory.

  /GPS/ALMANAC.TXT   GPS almanac, YUMA format     (Passes tab; rough positions)
  /GPS/EPHEM.RNX     GPS broadcast ephemeris,     (Passes tab; needed by the
                     RINEX 2 or 3 navigation file  snapshot fix of the Fix tab)
  /GPS/TLE.TXT       two-line element sets (TLE)  (Passes tab: ISS and others)

Press "Load" on the Passes tab after copying new files (they are also read
when the app opens).

Position and time
-----------------
Passes and the sky plot need your position and UTC:
- Position: /SETTINGS/position.ini, shared by all apps. Set it with "Pos" on
  the Passes tab (degrees / minutes / seconds, saved as "Manual"), with
  Signature Detect's Setup "Save", or (later) with the GPS fix itself.
- Time: the PortaPack clock (Settings > Date/Time). If the clock shows local
  time, set "Off" on the Passes tab to the clock minus UTC in minutes (for
  example -300 for US Eastern Standard Time, -240 in summer, 60 for CET).
  The snapshot fix needs the clock within about 30 s of the real time.

ALMANAC.TXT (YUMA)
------------------
Valid for weeks (positions to a few km): good for passes and the sky plot,
NOT accurate enough for a position fix. Sources (no login):
  https://www.navcen.uscg.gov/sites/default/files/gps/almanac/current_yuma.alm
  https://celestrak.org/GPS/almanac/Yuma/<year>/  (almanac.yuma.weekWWWW.TOA.txt)
Save the file as /GPS/ALMANAC.TXT. Older files with a 10-bit week number
are resolved to the week nearest the PortaPack's date.

EPHEM.RNX (RINEX navigation, broadcast ephemeris)
-------------------------------------------------
Exact orbits, each valid for about 4 hours around its reference time; a
daily file holds every 2-hour update of the day, so a file for TODAY (UTC)
covers the whole day. Sources:
  BKG (no login): https://igs.bkg.bund.de/root_ftp/IGS/BRDC/<year>/<day of year>/
     BRDC00WRD_R_<year><doy>0000_01D_MN.rnx.gz  (multi-GNSS; GPS records used)
  NASA CDDIS (Earthdata login): https://cddis.nasa.gov/archive/gnss/data/daily/
     <year>/brdc/brdc<doy>0.<yy>n.gz  (GPS only, RINEX 2)
Unzip (gzip) and save as /GPS/EPHEM.RNX. Only GPS records are read; a
multi-GNSS file takes longer to read (about 1 s per MB). RINEX 4 is not
supported.

TLE.TXT (two-line elements)
---------------------------
Name line, then lines 1 and 2, repeated; up to 5 satellites are used (the
first 5 in the file). Near-earth orbits only (period under 225 minutes, e.g.
the ISS, weather and amateur satellites); deep-space sets are skipped.
Source (no login; CelesTrak allows one download per data update):
  https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE  (ISS)
  https://celestrak.org/NORAD/elements/gp.php?GROUP=stations&FORMAT=TLE
TLEs age: refresh them every few days for minute-accurate pass times.

Heard tab (what the receiver hears)
-----------------------------------
"Search" records a snapshot (Quick 10 ms or Deep 20 ms) and searches it for
GPS satellites. With orbit files, a position and the time it searches only the
satellites that should be up, around their expected Doppler (faster); without
them all 32 (Quick: about 1 minute, Deep: about 2 minutes, estimated). The list
shows each satellite heard with a C/N0 bar (dB-Hz; 38 and up is good) and its
Doppler in kHz. The second line shows the noise level: aim for 6-30 LSB with
no "clip" (lower the gains if "gain-" shows, raise them if "gain+" shows), and
the M4 time of one search step. The sky plot shows satellites predicted above
the horizon (grey) and those heard (green).

Fix tab (your position)
-----------------------
"Snapshot+eph file": needs /GPS/EPHEM.RNX for today, a rough position (within
about 100 km: the saved position, e.g. your town) and the clock within about
30 s. "Get fix" records 20 ms, searches the satellites that should be up and
solves the position (about 10-20 s, estimated). It shows latitude, longitude,
a rough altitude, the estimated error, satellites used / heard and the age of
the ephemeris. "Save pos" writes /SETTINGS/position.ini (source "HackRF GPS"),
which Signature Detect's Setup "Load" and the Passes tab use. 6 or more
satellites are needed (the extra one checks the result).
"Autonomous" (decoding the orbits from the signal, no file) is not available
yet: selecting it shows "NOT AVAILABLE".

Passes tab
----------
Top: your position, UTC, what was loaded. Sky plot: GPS satellites above the
horizon from the almanac/ephemeris (green, PRN numbers) and TLE satellites
(orange). Right: GPS satellites up (PRN, elevation, azimuth). Bottom: the next pass of each
TLE satellite in the next 24 h: "in 1h05 47d 9m" = rises in 1 h 05 min,
highest 47 degrees, visible 9 min ("UP" = above the horizon now).
