Radar Control Program Name:
==========================
normalsound_usrp

Control Program ID (CPID):
=========================
1108 / 1109

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
normalsound_usrp is a variant on the normalsound radar control
program that steps through a set of up to 12 frequencies and all
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

Source:
======
E.G. Thomas (20250311)
