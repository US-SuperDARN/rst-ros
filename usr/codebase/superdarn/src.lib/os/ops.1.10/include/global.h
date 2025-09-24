/* global.h
   ========
   Author: R.J.Barnes
*/

/*
 LICENSE AND DISCLAIMER

 Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory

 This file is part of the RST Radar Operating System (RST-ROS).

 RST-ROS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/


#ifndef _GLOBAL_H
#define _GLOBAL_H

extern struct RadarParm *prm;
extern struct IQ *iq;
extern struct RawData *raw;
extern struct FitData *fit;
extern struct FitBlock *fblk;
extern struct RadarSite *site;
extern struct RadarTdiff *tdiff;
extern struct FreqTable *ftable;
extern struct RShellTable rstable;
extern struct RadarNetwork *network;
extern struct Radar *radar;

extern struct TSGbuf *tsgbuf;
extern struct TSGprm tsgprm;

extern struct RMsgBlock msg;

extern unsigned char command[128];

extern int debug;


extern int rnum;
extern int cnum;

extern int tsgid;
extern int stid;
extern int yr;
extern int mo;
extern int dy;
extern int hr;
extern int mt;
extern int sc;
extern int us;
extern int txpow;
extern int nave;
extern int atten;
extern int lagfr;
extern int smsep;
extern int ercod;
extern int agcstat;
extern int lopwrstat;
extern int nbaud;
extern float noise;
extern int rxrise;
extern int intsc;
extern int intus;
extern int txpl;
extern int mpinc;
extern int mppul;
extern int mplgs;
extern int mplgexs;
extern int ifmode;
extern int invert;
extern int nrang;
extern int frang;
extern int rsep;
extern int bmnum;
extern int xcf;
extern int tfreq;
extern int scan;
extern int mxpwr;
extern int lvmax;
extern int cp;
extern int channel;

extern int *pcode;

extern int noisefac;
extern int noiselim;

extern float noisestat[4];
extern char combf[128];

extern int rxchn;
extern int smpnum;
extern int skpnum;
extern struct timespec seqtval[MAXNAVE];
extern int seqatten[MAXNAVE];
extern float seqnoise[MAXNAVE];
extern int seqoff[MAXNAVE];
extern int seqsze[MAXNAVE];

extern struct {
  int num;
  unsigned int *start;
  unsigned int *length;
} seqbadtr[MAXNAVE];

extern int iqbufsize;
extern int16 *samples;

extern unsigned int *badtr;

extern int shmemfd;
extern char sharedmemory[256];

extern float pwr0[MAX_RANGE];
extern float acfd[MAX_RANGE*LAG_SIZE*2];
extern float xcfd[MAX_RANGE*LAG_SIZE*2];

extern int backward;
extern int sbm;
extern int ebm;

extern int xcnt;

extern int day;
extern int night;

extern int dfrq;
extern int nfrq;

extern int dmpinc;
extern int nmpinc;

extern int dfrang;
extern int nfrang;

extern int stfrq;
extern int frqrng;

/* lists for parameters across a scan, need to send to usrp_server for swings to work.. */
extern int32_t scan_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
extern int32_t scan_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
extern int32_t scan_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
extern int32_t nBeams_per_scan;
extern int current_beam, iBeam;

/* time sync of integration periods/ beams */
extern int sync_scan;
extern int *scan_times;  /* scan times in ms */

#endif
