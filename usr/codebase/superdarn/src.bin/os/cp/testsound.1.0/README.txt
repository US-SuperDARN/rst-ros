Radar Control Program Name:
==========================
testsound

Control Program ID (CPID):
=========================
1100 / 1101

Parameters:
==========
nbeams: 16+
intt: 6 s / 3 s
scan: 2 min / 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
testsound is a variant on the normalsound radar control
program that performs a 1- or 2-min scan in a sequential manner.
In the remaining time until the end of the minute it performs
scans through a set of up to 12 frequencies and through all
beams [even/odd]. Note that unlike normalsound, this control
program also writes the sounding output to a separate RAWACF-
format file. The idea is to validate the quality of the end-of-scan
sounding mode data processed using on-site FITACF software vs
the output when post-procesing with fitting algorithms available
in the Radar Software Toolkit (RST). The raw IQ samples can
also be written to a separate IQDAT-format file using an
optional command line flag (-iqdat).

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
E.G. Thomas (20241001)
