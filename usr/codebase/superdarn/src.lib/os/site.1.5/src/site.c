/* site.c
   ======
   Author R.J.Barnes
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


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include "rtypes.h"
#include "limit.h"
#include "tsg.h"
#include "maketsg.h"
#include "acf.h"
#include "tcpipmsg.h"
#include "rosmsg.h"
#include "shmem.h"
#include "global.h"
#include "site.h"

int num_transmitters;
/*struct timeval tock;*/
struct ControlPRM rprm;
struct RosData rdata;
struct DataPRM dprm;
struct TRTimes badtrdat;
struct TXStatus txstatus;
struct SiteLibrary sitelib;
int cancel_count=0;
int dmatch=0;

struct TCPIPMsgHost ros={"127.0.0.1",45000,-1};
struct TCPIPMsgHost errlog={"127.0.0.1",44100,-1};
struct TCPIPMsgHost shell={"127.0.0.1",44101,-1};
struct TCPIPMsgHost task[4]={
  {"127.0.0.1",1,-1}, /* iqwrite */
  {"127.0.0.1",2,-1}, /* rawacfwrite */
  {"127.0.0.1",3,-1}, /* fitacfwrite */
  {"127.0.0.1",4,-1}  /* rtserver */
};
int baseport=44100;


int SiteStart(char *host,char *ststr) {
  rdata.main=NULL;
  rdata.back=NULL;
  badtrdat.start_usec=NULL;
  badtrdat.duration_usec=NULL;
  return (sitelib.start)(host,ststr);
}

int SiteSetupRadar() {
  return (sitelib.setupradar)();
}

int SiteStartScan(int32_t periods_per_scan, int32_t *scan_beam_list,
                  int32_t *clrfreq_fstart_list, int32_t *clrfreq_bandwidth_list,
                  int32_t fixFreq, int32_t sync_scan, int32_t *beam_times,
                  int32_t scn_sc, int32_t scn_us, int32_t int_sc, int32_t int_us,
                  int32_t start_period) {
  return (sitelib.startscan)(periods_per_scan, scan_beam_list, clrfreq_fstart_list,
                             clrfreq_bandwidth_list, fixFreq, sync_scan, beam_times,
                             scn_sc, scn_us, int_sc, int_us, start_period);
}

int SiteStartIntt(int sec,int usec) {
  return (sitelib.startintt)(sec,usec);
}

int SiteFCLR(int stfreq,int edfreq) {
  return (sitelib.fclr)(stfreq,edfreq);
}

int SiteTimeSeq(int *ptab) {
  return (sitelib.tmseq)(ptab);
}

int SiteIntegrate(int (*lags)[2]) {
  return (sitelib.integrate)(lags);
}

int SiteEndScan(int bsc,int bus, unsigned sleepus) {
  return (sitelib.endscan)(bsc,bus, sleepus);
}

void SiteExit(int signo) {
  (sitelib.exit)(signo);
}

