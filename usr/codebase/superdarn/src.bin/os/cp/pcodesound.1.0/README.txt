Radar Control Program Name:
==========================
pcodesound

Control Program ID (CPID):
=========================
990 / 991

Parameters:
==========
nbeams: 16+
intt: 6 s / 3 s
scan: 2 min / 1 min
ngates: 225+
frang: 180 km
rsep: 15 km

Description:
===========
pcodesound is a variant on the pcodescan and normalsound radar control
programs that performs a 1- or 2-min scan in a sequential manner.
In the remaining time until the end of the minute it performs
scans through a set of up to 12 frequencies and through all
beams [even/odd]. Note that unlike previous versions of normalsound,
this information is not used to adjust the radar operating
frequency in real-time.

The control program requires a radar-specific sounding file called
"sounder.dat.[rad]", where "[rad]" should be replaced by
the three-letter radar station string. By default, the control
program will look for this file in the SD_SITE_PATH directory.
This file should contain the following values (one per line):

Number of sounder frequencies (maximum of 12)
The sounder frequencies [kHz]

If this file does not exist, default values are used. This
is not a good idea, as the program may try to sound at
forbidden frequencies.

The sounding data are written to *.snd files in the SD_SND_PATH
directory. If this environment variable is not set, the control
program will attempt to write the sounding data to the "/data/ros/snd"
directory. If this directory does not exist, no sounding data will
be written. The 2-hr sounding files contain a reduced set of
radar operating parameters and fitted values (e.g., velocity,
power, spectral width, phi0) in dmap-format.

Source:
======
E.G. Thomas (20250123)
