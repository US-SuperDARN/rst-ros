Radar Control Program Name:
==========================
testscan

Control Program ID (CPID):
=========================
131

Parameters:
==========
nbeams: 20+
intt: 3 s
scan: 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
testscan is a variant on the normalscan radar control program
that performs a 1-min scan in a non-standard manner. The beam
sequence is slightly adjusted such that the scan no longer starts
on the overlapping meridional beams of a dual-site MSI-style
radar to try debugging the noise problem on those beams.

Source:
======
E.G. Thomas (20221123)
