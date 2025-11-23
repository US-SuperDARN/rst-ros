Radar Control Program Name:
==========================
widetest

Control Program ID (CPID):
=========================
1700 / 1701

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
widetest is a variant on the normalscan radar control program
that performs alternating 1- or 2-min scans using different
transmission beam patterns. During the first scan, all requested
beams are transmitted and received using the standard narrow
beamforming approach. During the second scan, all requested
beams are transmitted using a single wide beam and received
using the standard narrow beamforming approach. Data records
corresponding to the wide transmission beam are indicated
by a bmazm value equal to zero in the radar parameter block.

Source:
======
E.G. Thomas (20251003)
