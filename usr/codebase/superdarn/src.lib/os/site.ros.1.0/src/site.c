/* site.c
   ======
   Author R.J.Barnes
*/

/*
 $License$
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <libconfig.h>
#include "rtypes.h"
#include "limit.h"
#include "tsg.h"
#include "maketsg.h"
#include "acf.h"
#include "acfex.h"
#include "tcpipmsg.h"
#include "rosmsg.h"
#include "shmem.h"
#include "errlog.h"
#include "global.h"
#include "site.h"
#include "siteglobal.h"

#define REAL_BUF_OFFSET 0
#define IMAG_BUF_OFFSET 1
#define USEC 1000000.0
int exit_flag=0;

struct timeval tock;

config_t cfg;
char station[10];


void SiteRosExit(int signum) {

  struct ROSMsg msg;

  switch(signum) {
    case 2:
      cancel_count++;
      exit_flag = signum;
      if (debug) printf("SiteRosExit: Sig 2: %d\n", exit_flag);
      if (cancel_count < 3) break;
    case 0:
      if (debug) printf("SiteRosExit: Sig 0: %d\n",exit_flag);
      if (exit_flag != 0) {
        msg.type = QUIT;
        TCPIPMsgSend(ros.sock, &msg, sizeof(struct ROSMsg));
        TCPIPMsgRecv(ros.sock, &msg, sizeof(struct ROSMsg));
        if (debug) {
          fprintf(stderr,"QUIT:type=%c\n",msg.type);
          fprintf(stderr,"QUIT:status=%d\n",msg.status);
        }
        close(ros.sock);
        config_destroy(&cfg);
        if (samples != NULL)
          ShMemFree((unsigned char *)samples,sharedmemory,iqbufsize,1,shmemfd);
        exit(errno);
      }
      break;
    default:
      printf("SiteRosExit: Sig %d: %d\n",signum,exit_flag);
      if (exit_flag == 0) exit_flag = signum;
      if (exit_flag != 0) {
        msg.type = QUIT;
        TCPIPMsgSend(ros.sock, &msg, sizeof(struct ROSMsg));
        TCPIPMsgRecv(ros.sock, &msg, sizeof(struct ROSMsg));
        if (debug) {
          fprintf(stderr,"QUIT:type=%c\n",msg.type);
          fprintf(stderr,"QUIT:status=%d\n",msg.status);
        }
        close(ros.sock);
        config_destroy(&cfg);
        if (samples != NULL)
          ShMemFree((unsigned char *)samples,sharedmemory,iqbufsize,1,shmemfd);
        exit(errno);
      }
      break;
  }
}


int SiteRosStart(char *host,char *ststr) {

  int n,ltemp,retval;
  const char *str;
  char *dfststr="tst";

  char *config_dir=NULL;
  char config_filepath[256];

  signal(SIGPIPE, SiteRosExit);
  signal(SIGINT,  SiteRosExit);
  signal(SIGUSR1, SiteRosExit);
  exit_flag = 0;
  cancel_count = 0;

  ros.sock = 0;

  config_dir = getenv("SD_SITE_PATH");
  if (config_dir == NULL) {
    fprintf(stderr,"Environment variable 'SD_SITE_PATH' not defined\nSiteRosStart aborting\n");
    return -1;
  }

  if (ststr==NULL) ststr=dfststr;

  if (cnum == 0) {
    sprintf(config_filepath,"%s/site.%s/%s.cfg",config_dir,ststr,ststr);
  } else {
    sprintf(config_filepath,"%s/site.%s/%s.%c.cfg",config_dir,ststr,ststr,cnum+96);
  }

  config_init(&cfg);
  retval = config_read_file(&cfg,config_filepath);
  if (retval == CONFIG_FALSE) {
    fprintf(stderr,"Site Cfg Read Error:: %d - %s\nSiteRosStart aborting\n",
            config_error_line(&cfg), config_error_text(&cfg));
    return -1;
  }

  /* Get the station name */
  if (config_lookup_string(&cfg, "station", &str)) {
    strcpy(station,str);
  } else {
    strcpy(station,ststr);
    fprintf(stderr,"Site Cfg Warning:: 'station' setting undefined in site cfg file, using: %s\n",station);
  }

  /* Get the radar beam scan direction */
  if (config_lookup_int(&cfg, "backward", &ltemp)) {
    backward = ltemp;
  } else {
    backward = 0;
    fprintf(stderr,"Site Cfg Warning:: 'backward' setting undefined in site cfg file, using: %d\n",backward);
  }

  /* Get the XCF (ie elevation angle) collection option */
  if (config_lookup_int(&cfg, "xcf", &ltemp)) {
    xcnt = ltemp;
  } else {
    xcnt = 1;
    fprintf(stderr,"Site Cfg Warning:: 'xcf' setting undefined in site cfg file, using: %d\n",xcnt);
  }

  /* Get the starting beam number */
  if (config_lookup_int(&cfg, "sbm", &ltemp)) {
    sbm = ltemp;
  } else {
    sbm = 0;
    fprintf(stderr,"Site Cfg Warning:: 'sbm' setting undefined in site cfg file, using: %d\n",sbm);
  }

  /* Get the ending beam number */
  if (config_lookup_int(&cfg, "ebm", &ltemp)) {
    ebm = ltemp;
  } else {
    ebm = 15;
    fprintf(stderr,"Site Cfg Warning:: 'ebm' setting undefined in site cfg file, using: %d\n",ebm);
  }

  /* Get the radar number to register with the ROS server */
  if (config_lookup_int(&cfg, "rnum", &ltemp)) {
    rnum = ltemp;
  } else {
    rnum = 0;
    fprintf(stderr,"Site Cfg Warning:: 'rnum' setting undefined in site cfg file, using: %d\n",rnum);
  }

  /* Get the channel number to register with the ROS server */
  if (config_lookup_int(&cfg, "cnum", &ltemp)) {
    cnum = ltemp;
  } else {
    if (cnum == 0) cnum = 1;
    fprintf(stderr,"Site Cfg Warning:: 'cnum' setting undefined in site cfg file, using: %d\n",cnum);
  }

  /* Get the Universal time to start using the day frequency (dfrq) */
  if (config_lookup_int(&cfg, "day", &ltemp)) {
    day = ltemp;
  } else {
    day = 15;
    fprintf(stderr,"Site Cfg Warning:: 'day' setting undefined in site cfg file, using: %d\n",day);
  }

  /* Get the Universal time to start using the night frequency (nfrq) */
  if (config_lookup_int(&cfg, "night", &ltemp)) {
    night = ltemp;
  } else {
    night = 3;
    fprintf(stderr,"Site Cfg Warning:: 'night' setting undefined in site cfg file, using: %d\n",night);
  }

  /* Get the day frequency */
  if (config_lookup_int(&cfg, "dfrq", &ltemp)) {
    dfrq = ltemp;
  } else {
    dfrq = 14700;
    fprintf(stderr,"Site Cfg Warning:: 'dfrq' setting undefined in site cfg file, using: %d\n",dfrq);
  }

  /* Get the night frequency */
  if (config_lookup_int(&cfg, "nfrq", &ltemp)) {
    nfrq = ltemp;
  } else {
    nfrq = 10400;
    fprintf(stderr,"Site Cfg Warning:: 'nfrq' setting undefined in site cfg file, using: %d\n",nfrq);
  }

  /* Get the number of range gates */
  if (config_lookup_int(&cfg, "nrang", &ltemp)) {
    nrang = ltemp;
  } else {
    nrang = 75;
    fprintf(stderr,"Site Cfg Warning:: 'nrang' setting undefined in site cfg file, using: %d\n",nrang);
  }

  /* Get whether you need to correct for inverted phase between main and interf rf signal
   *   invert=0         No inversion necessary
   *   invert=non-zero  Inversion necessary */
  if (config_lookup_int(&cfg, "invert", &ltemp)) {
    invert = ltemp;
  } else {
    invert = 1;
    fprintf(stderr,"Site Cfg Warning:: 'invert' setting undefined in site cfg file, using: %d\n",invert);
  }

  /* Get the number of rx channels (typically 1) */
  if (config_lookup_int(&cfg, "rxchn", &ltemp)) {
    rxchn = ltemp;
  } else {
    rxchn = 1;
    fprintf(stderr,"Site Cfg Warning:: 'rxchn' setting undefined in site cfg file, using: %d\n",rxchn);
  }

  /* Get the match filter value */
  if (config_lookup_int(&cfg, "match_filter", &ltemp)) {
    dmatch = ltemp;
  } else {
    dmatch = 1;
    fprintf(stderr,"Site Cfg Warning:: 'match_filter' setting undefined in site cfg file, using: %d\n",dmatch);
  }

  /* Get the sample separation value */
  if (config_lookup_int(&cfg, "smsep", &ltemp)) {
    smsep = ltemp;
  } else {
    smsep = 300;
    fprintf(stderr,"Site Cfg Warning:: 'smsep' setting undefined in site cfg file, using: %d\n",smsep);
  }

  /* Get the ROS server address */
  if (host !=NULL) {
    strcpy(ros.host,host);
  } else if (config_lookup_string(&cfg, "ros.host", &str)) {
    strcpy(ros.host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'ros.host' setting undefined in site cfg file, using: %s\n",ros.host);
  }

  /* Get the ROS server port */
  if (config_lookup_int(&cfg, "ros.port", &ltemp)) {
    ros.port = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'ros.port' setting undefined in site cfg file, using: %d\n",ros.port);
  }

  /* Get the errlog server address */
  if (config_lookup_string(&cfg, "errlog.host", &str)) {
    strcpy(errlog.host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'errlog.host' setting undefined in site cfg file, using: %s\n",errlog.host);
  }

  /* Get the errlog server port */
  if (config_lookup_int(&cfg, "errlog.port", &ltemp)) {
    errlog.port = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'errlog.port' setting undefined in site cfg file, using: %d\n",errlog.port);
  }

  /* Get the shell server address */
  if (config_lookup_string(&cfg, "shell.host", &str)) {
    strcpy(shell.host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'shell.host' setting undefined in site cfg file, using: %s\n",shell.host);
  }

  /* Get the shell server port */
  if (config_lookup_int(&cfg, "shell.port", &ltemp)) {
    shell.port = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'shell.port' setting undefined in site cfg file, using: %d\n",shell.port);
  }

  /* Get the task server address */
  if (config_lookup_string(&cfg, "task.host", &str)) {
    for (n=0;n<4;n++) strcpy(task[n].host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'task.host' setting undefined in site cfg file, using: %s\n",task[0].host);
  }

  /* Get the task server baseport */
  if (config_lookup_int(&cfg, "task.baseport", &ltemp)) {
    baseport = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'task.baseport' setting undefined in site cfg file, using: %d\n",baseport);
  }

  return 0;
}


int SiteRosSetupRadar() {

  int32 temp32,data_length;
  struct ROSMsg smsg,rmsg;

  char logtxt[1024]="";

  if (debug) ErrLog(errlog.sock,"SiteRosSetupRadar","Entering SiteRosSetupRadar");

  if ((ros.sock=TCPIPMsgOpen(ros.host,ros.port)) == -1) return -1;

  if (debug) ErrLog(errlog.sock,"SiteRosSetupRadar","Sending SET_RADAR_CHAN");
  smsg.type = SET_RADAR_CHAN;
  TCPIPMsgSend(ros.sock, &smsg,sizeof(struct ROSMsg));
  temp32 = stid;
  TCPIPMsgSend(ros.sock, &temp32, sizeof(int32));
  data_length = strlen(station)+1;
  TCPIPMsgSend(ros.sock, &data_length, sizeof(int32));
  TCPIPMsgSend(ros.sock, &station, data_length*sizeof(char));
  temp32 = rnum;
  TCPIPMsgSend(ros.sock, &temp32, sizeof(int32));
  temp32 = cnum;
  TCPIPMsgSend(ros.sock, &temp32, sizeof(int32));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (rmsg.status < 0) {
    ErrLog(errlog.sock,"SiteRosSetupRadar","Requested radar channel unavailable. Sleeping 1 second and exiting");
    sleep(1);
    SiteRosExit(-1);
  }
  if (debug) {
    sprintf(logtxt,"SET_RADAR_CHAN:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
    sprintf(logtxt,"SET_RADAR_CHAN:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosSetupRadar","Sending QUERY_INI_SETTINGS");
  smsg.type = QUERY_INI_SETTINGS;
  TCPIPMsgSend(ros.sock, &smsg,  sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &txpow, sizeof(int));
  TCPIPMsgRecv(ros.sock, &atten, sizeof(int));
  TCPIPMsgRecv(ros.sock, &rmsg,  sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"QUERY_INI_SETTINGS:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
    sprintf(logtxt,"QUERY_INI_SETTINGS:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosSetupRadar","Sending GET_PARAMETERS");
  smsg.type = GET_PARAMETERS;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rprm, sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"GET_PARAMETERS:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
    sprintf(logtxt,"GET_PARAMETERS:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
  }

  sprintf(sharedmemory,"IQBuff_%s_%d_%d",station,rnum,cnum);

  samples=(int16 *)
    ShMemAlloc(sharedmemory,iqbufsize,O_RDWR | O_CREAT,1,&shmemfd);

  if (samples == NULL) {
    sprintf(logtxt,"IQBuffer %s is Null",sharedmemory);
    ErrLog(errlog.sock,"SiteRosSetupRadar",logtxt);
    SiteRosExit(-1);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosSetupRadar","Leaving SiteRosSetupRadar");

  return 0;
}


int SiteRosStartScan(int32_t periods_per_scan, int32_t *scan_beam_list,
                     int32_t *clrfreq_fstart_list, int32_t *clrfreq_bandwidth_list,
                     int32_t fixFreq, int32_t sync_scan, int32_t *beam_times,
                     int32_t scn_sc, int32_t scn_us, int32_t int_sc, int32_t int_us,
                     int32_t start_period) {

  struct ROSMsg smsg,rmsg;

  if (debug) ErrLog(errlog.sock,"SiteRosStartScan","Entering SiteRosStartScan");

  if (debug) ErrLog(errlog.sock,"SiteRosStartScan","Sending SET_ACTIVE");
  smsg.type = SET_ACTIVE;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));

  TCPIPMsgSend(ros.sock, &periods_per_scan, sizeof(int32_t));    /* number of periods */

  TCPIPMsgSend(ros.sock, &fixFreq,  sizeof(int32_t));    /* fixed frequency or -1 for clear_frequency search */
  TCPIPMsgSend(ros.sock, &clrfreq_fstart_list[0], periods_per_scan * sizeof(int32_t));    /* start frequency of clrfreq */
  TCPIPMsgSend(ros.sock, &clrfreq_bandwidth_list[0], periods_per_scan * sizeof(int32_t));    /* bandwidth of clrfreq in hertz */
  TCPIPMsgSend(ros.sock, &scan_beam_list[0], periods_per_scan * sizeof(int32_t));    /* start frequency of clrfreq */

  TCPIPMsgSend(ros.sock, &sync_scan, sizeof(int32_t));    /* if the periods/beams should start at fixed times*/
  TCPIPMsgSend(ros.sock, &scn_sc, sizeof(int32_t));    /* scan and inetegration times */
  TCPIPMsgSend(ros.sock, &scn_us, sizeof(int32_t));
  TCPIPMsgSend(ros.sock, &int_sc, sizeof(int32_t));
  TCPIPMsgSend(ros.sock, &int_us, sizeof(int32_t));

  TCPIPMsgSend(ros.sock, &start_period, sizeof(int32_t));
  if (sync_scan == 1) {
    TCPIPMsgSend(ros.sock, &beam_times[0], periods_per_scan * sizeof(int32_t));
  }

  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));

  if (debug) ErrLog(errlog.sock,"SiteRosStartScan","Leaving SiteRosStartScan");

  return rmsg.status;
}


int SiteRosStartIntt(int sec,int usec) {

  struct ROSMsg smsg,rmsg;
  int total_samples=0;
  double secs;

  char logtxt[1024]="";

  if (debug) ErrLog(errlog.sock,"SiteRosStartIntt","Entering SiteRosStartIntt");

  SiteRosExit(0);

  total_samples = tsgprm.samples + tsgprm.smdelay;

  if (debug) ErrLog(errlog.sock,"SiteRosStartIntt","Sending PING");
  smsg.type = PING;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"PING:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"PING:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosStartIntt","Sending GET_PARAMETERS");
  smsg.type = GET_PARAMETERS;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rprm, sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"GET_PARAMETERS:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"GET_PARAMETERS:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
  }

  rprm.tbeam = bmnum;
  rprm.tfreq = 12000;
  rprm.rbeam = bmnum;
  rprm.rfreq = 12000;
  rprm.trise = 5000;
  rprm.baseband_samplerate = ((double)nbaud/(double)txpl)*1E6;
  rprm.filter_bandwidth    = rprm.baseband_samplerate;
  rprm.match_filter        = dmatch;
  rprm.number_of_samples   = total_samples + nbaud + 10;
  rprm.priority            = cnum;
  rprm.buffer_index        = 0;
  strncpy(rprm.name,station,10);

  if (debug) ErrLog(errlog.sock,"SiteRosStartIntt","Sending SET_PARAMETERS");
  smsg.type = SET_PARAMETERS;
  TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock,&rprm,sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"SET_PARAMETERS:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:tbeam=%d",rprm.tbeam);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:tfreq=%d",rprm.tfreq);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:rbeam=%d",rprm.rbeam);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:rfreq=%d",rprm.rfreq);
    ErrLog(errlog.sock,"SiteRosStartIntt",logtxt);
  }

  gettimeofday(&tock,NULL);
  secs = sec + (double)usec/1E6;
  tock.tv_sec  += floor(secs);
  tock.tv_usec += (secs-floor(secs))*1E6;

  if (debug) ErrLog(errlog.sock,"SiteRosStartIntt","Leaving SiteRosStartIntt");

  return 0;
}


int SiteRosFCLR(int stfreq,int edfreq) {

  int32 tfreq;
  struct ROSMsg smsg,rmsg;
  struct CLRFreqPRM fprm;

  char logtxt[1024]="";

  if (debug) ErrLog(errlog.sock,"SiteRosFCLR","Entering SiteRosFCLR");

  SiteRosExit(0);

  fprm.start = stfreq;
  fprm.end   = edfreq;
  fprm.nave  = 20;
  fprm.filter_bandwidth = 250;

  if (debug) ErrLog(errlog.sock,"SiteRosFCLR","Sending REQUEST_CLEAR_FREQ_SEARCH");
  smsg.type = REQUEST_CLEAR_FREQ_SEARCH;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock, &fprm, sizeof(struct CLRFreqPRM));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"REQUEST_CLEAR_FREQ_SEARCH:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosFCLR",logtxt);
    sprintf(logtxt,"REQUEST_CLEAR_FREQ_SEARCH:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosFCLR",logtxt);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosFCLR","Sending REQUEST_ASSIGNED_FREQ");
  smsg.type = REQUEST_ASSIGNED_FREQ;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock,&tfreq, sizeof(int32));
  TCPIPMsgRecv(ros.sock,&noise, sizeof(float));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"REQUEST_ASSIGNED_FREQ:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosFCLR",logtxt);
    sprintf(logtxt,"REQUEST_ASSIGNED_FREQ:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosFCLR",logtxt);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosFCLR","Leaving SiteRosFCLR");

  return tfreq;
}


int SiteRosTimeSeq(int *ptab) {

  int i;
  int flag,index=0;
  struct ROSMsg smsg,rmsg;
  struct SeqPRM tprm;

  char logtxt[1024]="";

  if (debug) ErrLog(errlog.sock,"SiteRosTimeSeq","Entering SiteRosTimeSeq");

  SiteRosExit(0);

  if (tsgbuf != NULL) TSGFree(tsgbuf);
  if (tsgprm.pat != NULL) free(tsgprm.pat);
  memset(&tsgprm,0,sizeof(struct TSGprm));

  tsgprm.nrang   = nrang;
  tsgprm.frang   = frang;
  tsgprm.rtoxmin = 0;
  tsgprm.stdelay = 18+2;
  tsgprm.gort    = 1;
  tsgprm.rsep    = rsep;
  tsgprm.smsep   = smsep;
  tsgprm.txpl    = txpl;
  tsgprm.mpinc   = mpinc;
  tsgprm.mppul   = mppul;
  tsgprm.mlag    = 0;
  tsgprm.nbaud   = nbaud;
  tsgprm.code    = pcode;
  tsgprm.pat     = malloc(sizeof(int)*tsgprm.mppul);
  for (i=0; i<tsgprm.mppul; i++) tsgprm.pat[i] = ptab[i];

  tsgbuf = TSGMake(&tsgprm,&flag);

  if (tsgbuf==NULL) {
    if (flag != 0) return flag;
    else return -1;
  }

  lagfr = tsgprm.lagfr;
  smsep = tsgprm.smsep;
  txpl  = tsgprm.txpl;

  tprm.index   = index;
/*  memcpy(&tprm.buf,tsgbuf,sizeof(struct TSGbuf));*/
  tprm.len     = tsgbuf->len;
  tprm.step    = CLOCK_PERIOD;
  tprm.samples = tsgprm.samples;
  tprm.smdelay = tsgprm.smdelay;

  if (debug) ErrLog(errlog.sock,"SiteRosTimeSeq","Sending REGISTER_SEQ");
  smsg.type = REGISTER_SEQ;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock, &tprm, sizeof(struct SeqPRM));
  TCPIPMsgSend(ros.sock, tsgbuf->rep, sizeof(unsigned char)*tprm.len);
  TCPIPMsgSend(ros.sock, tsgbuf->code, sizeof(unsigned char)*tprm.len);

  if (debug) {
    sprintf(logtxt,"REGISTER_SEQ:intsc=%d",intsc);
    ErrLog(errlog.sock,"SiteRosTimeSeq",logtxt);
    sprintf(logtxt,"REGISTER_SEQ:intus=%d",intus);
    ErrLog(errlog.sock,"SiteRosTimeSeq",logtxt);
  }

  TCPIPMsgSend(ros.sock, &intsc, sizeof(int));
  TCPIPMsgSend(ros.sock, &intus, sizeof(int));

  TCPIPMsgSend(ros.sock, &nrang, sizeof(int));
  TCPIPMsgSend(ros.sock, &mpinc, sizeof(int));
  TCPIPMsgSend(ros.sock, &smsep, sizeof(int));
  TCPIPMsgSend(ros.sock, &lagfr, sizeof(int));
  TCPIPMsgSend(ros.sock, &mppul, sizeof(int));
  for (i=0; i<mppul; i++)
    TCPIPMsgSend(ros.sock, &tsgprm.pat[i], sizeof(int));

  TCPIPMsgSend(ros.sock, &nbaud, sizeof(int));
  for (i=0; i<nbaud; i++)
    TCPIPMsgSend(ros.sock, &pcode[i], sizeof(int));

  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"REGISTER_SEQ:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosTimeSeq",logtxt);
    sprintf(logtxt,"REGISTER_SEQ:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosTimeSeq",logtxt);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosTimeSeq","Leaving SiteRosTimeSeq");

  if (rmsg.status != 0) return -2;

  return index;
}


int SiteRosIntegrate(int (*lags)[2]) {

  int *lagtable[2]={NULL,NULL};
  int lagsum[LAG_SIZE];

  int badrng=0;
  int i,j;
  int roff=REAL_BUF_OFFSET;
  int ioff=IMAG_BUF_OFFSET;
  int rngoff=2;

  struct ROSMsg smsg,rmsg;

  char logtxt[1024]="";

  int iqoff=0; /* Sequence offset in bytes for current sequence relative to *
                           start of samples buffer */
  int iqsze=0; /* Total number of bytes so far recorded into samples buffer*/

  int nave=0;

  int atstp=0;
  int thr=0,lmt=0;
  int aflg=0,abflg=0;
  void *dest=NULL; /*AJ*/
  int total_samples=0; /*AJ*/
  int usecs;
  short I,Q;

  uint32_t number_of_sequences_in_integration_period;

  /* phase code declarations */
  int n,nsamp, *code, Iout, Qout;
  uint32 uI32,uQ32;
  //uint32 *maddr, *baddr;

  if (debug) ErrLog(errlog.sock,"SiteRosIntegrate","Entering SiteRosIntegrate");

  SiteRosExit(0);

  if (nrang >= MAX_RANGE) return -1;
  for (j=0; j<LAG_SIZE; j++) lagsum[j] = 0;

  if (mplgexs == 0) {
    lagtable[0] = malloc(sizeof(int)*(mplgs+1));
    if (lagtable[0] == NULL) {
      ErrLog(errlog.sock,"SiteRosIntegrate","Lagtable-0 is Null");
      SiteRosExit(-1);
    }
    lagtable[1] = malloc(sizeof(int)*(mplgs+1));
    if (lagtable[1] == NULL) {
      ErrLog(errlog.sock,"SiteRosIntegrate","Lagtable-1 is Null");
      SiteRosExit(-1);
    }
    for (i=0; i<=mplgs; i++) {
      lagtable[0][i] = lags[i][0];
      lagtable[1][i] = lags[i][1];
    }
  } else {
    lagtable[0] = malloc(sizeof(int)*(mplgexs+1));
    if (lagtable[0] == NULL) {
      ErrLog(errlog.sock,"SiteRosIntegrate","Lagtable-0 is Null");
      SiteRosExit(-1);
    }
    lagtable[1] = malloc(sizeof(int)*(mplgexs+1));
    if (lagtable[1] == NULL) {
      ErrLog(errlog.sock,"SiteRosIntegrate","Lagtable-1 is Null");
      SiteRosExit(-1);
    }

    for (i=0; i<=mplgexs; i++) {
      lagtable[0][i] = lags[i][0];
      lagtable[1][i] = lags[i][1];
      j = abs(lags[i][0]-lags[i][1]);
      lagsum[j]++;
    }
  }

  total_samples = tsgprm.samples + tsgprm.smdelay;
  skpnum = tsgprm.smdelay;  /* skpnum != 0  returns 1, which is used as the  *
                                            dflg argument in ACFCalculate to *
                                            enable smdelay usage in offset   *
                                            calculations */

  badrng = ACFBadLagZero(&tsgprm,mplgs,lagtable);

  for (i=0; i<MAX_RANGE; i++) {
    pwr0[i] = 0;
    for (j=0; j<LAG_SIZE*2; j++) {
      acfd[i*LAG_SIZE*2+j] = 0;
      xcfd[i*LAG_SIZE*2+j] = 0;
    }
  }

  /* Start of SiteIntegration loop was located here */
  SiteRosExit(0);

  seqatten[nave] = 0;
  seqnoise[nave] = 0;
  seqbadtr[nave].num = 0;

  rprm.tbeam = bmnum;
  rprm.tfreq = tfreq;
  rprm.rbeam = bmnum;
  rprm.rfreq = tfreq;
  rprm.trise = 5000;
  rprm.baseband_samplerate = ((double)nbaud/(double)txpl)*1E6;
  rprm.filter_bandwidth    = rprm.baseband_samplerate;
  rprm.match_filter        = dmatch;
  rprm.number_of_samples   = total_samples + nbaud + 10;
  rprm.priority            = cnum;
  rprm.buffer_index        = 0;
  strncpy(rprm.name,station,10);

  usecs = (int)(rprm.number_of_samples/rprm.baseband_samplerate*1E6);

  if (debug) ErrLog(errlog.sock,"SiteRosIntegrate","Sending SET_PARAMETERS");
  smsg.type = SET_PARAMETERS;
  TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock,&rprm,sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"SET_PARAMETERS:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:tbeam=%d",rprm.tbeam);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:tfreq=%d",rprm.tfreq);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:rbeam=%d",rprm.rbeam);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"SET_PARAMETERS:rfreq=%d",rprm.rfreq);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
  }
  if (rmsg.status < 0) {
    ErrLog(errlog.sock,"SiteRosIntegrate","SET_PARAMETERS failed. Sleeping 1 second and exiting");
    sleep(1);
    SiteRosExit(-1);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosIntegrate","Sending SET_READY_FLAG");
  smsg.type = SET_READY_FLAG;
  TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"SET_READY_FLAG:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"SET_READY_FLAG:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
  }

  usleep(usecs);

  if (debug) ErrLog(errlog.sock,"SiteRosIntegrate","Sending GET_DATA");
  smsg.type = GET_DATA;
  if (rdata.main != NULL) free(rdata.main);
  if (rdata.back != NULL) free(rdata.back);
  rdata.main = NULL;
  rdata.back = NULL;
  TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
  if (debug) ErrLog(errlog.sock,"SiteRosIntegrate","GET_DATA: recv dprm");

  TCPIPMsgRecv(ros.sock,&dprm,sizeof(struct DataPRM));
  if (rdata.main) free(rdata.main);
  if (rdata.back) free(rdata.back);
  if (debug) {
    sprintf(logtxt,"GET_DATA: samples %d status %d",dprm.samples,dprm.status);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
  }
  smpnum = dprm.samples;

  TCPIPMsgRecv(ros.sock,&number_of_sequences_in_integration_period, sizeof(uint32_t));

  if (dprm.status == 0) {
    //if (debug)
    //  fprintf(stderr,"%s GET_DATA: rdata.main: uint32: %ld array: %ld\n",
    //          station,sizeof(uint32),sizeof(uint32)*dprm.samples);

    rdata.main = malloc(sizeof(uint32)*dprm.samples);
    rdata.back = malloc(sizeof(uint32)*dprm.samples);

    if (badtrdat.start_usec    != NULL) free(badtrdat.start_usec);
    if (badtrdat.duration_usec != NULL) free(badtrdat.duration_usec);
    badtrdat.start_usec = NULL;
    badtrdat.duration_usec = NULL;

    if (debug) {
      sprintf(logtxt,"GET_DATA: trtimes length %d",badtrdat.length);
      ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    }

    TCPIPMsgRecv(ros.sock, &badtrdat.length, sizeof(badtrdat.length));
    //if (debug)
    //  fprintf(stderr,"%s GET_DATA: badtrdat.start_usec: uint32: %ld array: "
    //          "%ld\n",station,sizeof(uint32),sizeof(uint32)*badtrdat.length);
    badtrdat.start_usec    = malloc(sizeof(uint32)*badtrdat.length);
    badtrdat.duration_usec = malloc(sizeof(uint32)*badtrdat.length);
    //if (debug) fprintf(stderr,"%s GET_DATA: start_usec\n",station);

    TCPIPMsgRecv(ros.sock, badtrdat.start_usec, sizeof(uint32)*badtrdat.length);
    //if (debug) fprintf(stderr,"%s GET_DATA: duration_usec\n",station);

    TCPIPMsgRecv(ros.sock, badtrdat.duration_usec, sizeof(uint32)*badtrdat.length);
    TCPIPMsgRecv(ros.sock, &num_transmitters, sizeof(int));
    TCPIPMsgRecv(ros.sock, &txstatus.AGC, sizeof(int)*num_transmitters);
    TCPIPMsgRecv(ros.sock, &txstatus.LOWPWR, sizeof(int)*num_transmitters);

    /* receive trigger time of integration period */
    TCPIPMsgRecv(ros.sock, &yr, sizeof(int));
    TCPIPMsgRecv(ros.sock, &mo, sizeof(int));
    TCPIPMsgRecv(ros.sock, &dy, sizeof(int));
    TCPIPMsgRecv(ros.sock, &hr, sizeof(int));
    TCPIPMsgRecv(ros.sock, &mt, sizeof(int));
    TCPIPMsgRecv(ros.sock, &sc, sizeof(int));
    TCPIPMsgRecv(ros.sock, &us, sizeof(int));
  }

  TCPIPMsgRecv(ros.sock, &rprm, sizeof(struct ControlPRM));

  if (debug) {
    sprintf(logtxt,"Number of samples: dprm.samples:%d tsgprm.samples:%d "
                   "total_samples:%d",dprm.samples,tsgprm.samples,total_samples);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"nave=%d dprm.status=%d rprm.tbeam=%d rprm.tfreq=%d",
                   nave,dprm.status,rprm.tbeam,rprm.tfreq);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
  }

  /* loop for receiving data from each pulse sequence */
  for (nave=0; nave<number_of_sequences_in_integration_period; nave++) {

    TCPIPMsgRecv(ros.sock, &dprm.event_secs, sizeof(uint32_t));
    TCPIPMsgRecv(ros.sock, &dprm.event_usecs, sizeof(uint32_t));

    /* recieve samples from main and back array */
    //if (debug)
    //  fprintf(stderr,"%s GET_DATA: recv main, expecting %d samples, %ld bytes\n",
    //          station,dprm.samples,(sizeof(uint32_t)*dprm.samples));
    TCPIPMsgRecv(ros.sock, rdata.main, sizeof(uint32)*dprm.samples);
    //if (debug)
    //  fprintf(stderr,"%s GET_DATA: recv back\n",station);
    TCPIPMsgRecv(ros.sock, rdata.back, sizeof(uint32)*dprm.samples);

    /* instead of receving a full dprm for every pulse sequence,
       now just update the fields which change between pulse sequences within an integration period
       that would be uint32_t dprm.event_secs, and uint32_t dprm.event_usecs
       so, receive these from the usrp_server, use them to generate a new tstruct and update dprm
     */

    if (nave == 0) {
      bmnum = rprm.tbeam;
      tfreq = rprm.tfreq;
    }

    if (dprm.status == 0) {
      nsamp = (int)dprm.samples;

      /* invert interf phase here if necessary */
      if (invert !=0) {
        for (n=0; n<nsamp; n++) {
          Q = (short)((rdata.main[n] & 0xffff0000) >> 16);
          I = (short)(rdata.main[n] & 0x0000ffff);
          Q = -Q;
          I = -I;
          uQ32 = ((uint32) Q) << 16;
          uI32 = ((uint32) I) & 0xFFFF;
          (rdata.main)[n] = uQ32|uI32;
        }
      }

      /* decode phase coding here */
      if (nbaud > 1) {
        code = pcode;
        for (n=0; n<(nsamp-nbaud); n++) {
          /* Process the main array sample: n */
          Iout = 0;
          Qout = 0;
          for (i=0; i<nbaud; i++) {
            Q = ((rdata.main)[n+i] & 0xffff0000) >> 16;
            I = (rdata.main)[n+i] & 0x0000ffff;
            Iout += (int)I*(int)code[i];
            Qout += (int)Q*(int)code[i];
          }
          Iout /= nbaud;
          Qout /= nbaud;
          I = (short)Iout;
          Q = (short)Qout;
          uQ32 = ((uint32) Q) << 16;
          uI32 = ((uint32) I) & 0xFFFF;
          (rdata.main)[n] = uQ32|uI32;

          /* Process the interf array sample: n */
          Iout = 0;
          Qout = 0;
          for (i=0; i<nbaud; i++) {
            Q = ((rdata.back)[n+i] & 0xffff0000) >> 16;
            I = (rdata.back)[n+i] & 0x0000ffff;
            Iout += (int)I*(int)code[i];
            Qout += (int)Q*(int)code[i];
          }
          Iout /= nbaud;
          Qout /= nbaud;
          I = (short)Iout;
          Q = (short)Qout;
          uQ32 = ((uint32) Q) << 16;
          uI32 = ((uint32) I) & 0xFFFF;
          (rdata.back)[n] = uQ32|uI32;
        }
      }
      /*
        if (dprm.samples < total_samples) {
          fprintf(stderr,"Not enough samples from the ROS in SiteIntegrate\n");
          fflush(stderr);
        }
       */

      /* copy samples here */
      seqoff[nave] = iqsze/2;          /* Sequence offset in 16bit units */
      seqsze[nave] = dprm.samples*2*2; /* Sequence length in 16bit units */

      seqtval[nave].tv_sec  = dprm.event_secs;
      seqtval[nave].tv_nsec = dprm.event_usecs*1000;

      if (seqbadtr[nave].start != NULL)  free(seqbadtr[nave].start);
      if (seqbadtr[nave].length != NULL) free(seqbadtr[nave].length);
      seqbadtr[nave].start  = NULL;
      seqbadtr[nave].length = NULL;

      seqbadtr[nave].num    = badtrdat.length;
      seqbadtr[nave].start  = malloc(sizeof(uint32)*badtrdat.length);
      seqbadtr[nave].length = malloc(sizeof(uint32)*badtrdat.length);

      memcpy(seqbadtr[nave].start,badtrdat.start_usec,
             sizeof(uint32)*badtrdat.length);
      memcpy(seqbadtr[nave].length,badtrdat.duration_usec,
             sizeof(uint32)*badtrdat.length);

      /* AJ new way, does work for some reason */
      /* samples is natively an int16 pointer */
      /* rdata.main is natively an uint32 pointer */
      /* rdata.back is natively an uint32 pointer */
      /* dprm.samples*8 represents number of bytes for main and back samples */

      dest = (void *)(samples);  /* look iqoff bytes into samples area */
      dest += iqoff;
      if ((iqoff+dprm.samples*2*sizeof(uint32)) < iqbufsize) {
        memmove(dest, rdata.main, dprm.samples*sizeof(uint32));
        /* skip ahead number of samples * 32 bit per sample to account for
           rdata.main */
        dest += dprm.samples*sizeof(uint32);
        memmove(dest, rdata.back, dprm.samples*sizeof(uint32));
      } else {
        fprintf(stderr,"IQ Buffer overrun in SiteIntegrate\n");
        fflush(stderr);
      }
      /* Total of number bytes so far copied into samples array */
      iqsze += dprm.samples*sizeof(uint32)*2;
      //if (debug) {
      //  fprintf(stderr,"%s seq %d :: ioff: %8d\n",station,nave,iqoff);
      //  fprintf(stderr,"%s seq %d :: rdata.main 16bit :\n",station,nave);
      //  fprintf(stderr," [  n  ] :: [  Im  ] [  Qm  ] :: [ Ii ] [ Qi ]\n");
      //  nsamp = (int)dprm.samples;
      //  maddr = (uint32 *)rdata.main;
      //  baddr = (uint32 *)rdata.back;
      //  for (n=0; n<(nsamp); n++) {
      //    Q = (maddr[n] & 0xffff0000) >> 16;
      //    I = maddr[n] & 0x0000ffff;
      //    fprintf(stderr," %7d :: 0x%8x : %7d %7d " ,n,(uint32)maddr[n],(int)I,(int)Q);
      //    Q = ((rdata.back)[n] & 0xffff0000) >> 16;
      //    I = ((uint32)((rdata.back)[n])) & 0x0000ffff;
      //    fprintf(stderr," :: 0x%8x : %7d %7d\n" ,(uint32)baddr[n],(int)I,(int)Q);
      //  }
      //  dest = (void *)(samples);
      //  dest += iqoff;
      //  fprintf(stderr,"%s seq %d :: rdata.back 16bit 30: %8d %8d\n",station,nave,
      //          ((int16 *)rdata.back)[60],((int16 *)rdata.back)[61]);
      //  dest += dprm.samples*sizeof(uint32);
      //  fprintf(stderr,"%s seq %d :: samples    16bit 30: %8d %8d\n",station,nave,
      //          ((int16 *)dest)[60],((int16 *)dest)[61]);
      //  fprintf(stderr,"%s seq %d :: iqsze: %8d\n",station,nave,iqsze);
      //}

      /* calculate ACF */
      if (mplgexs == 0) {
        dest = (void *)(samples);
        dest += iqoff;
        rngoff = 2*rxchn;
        //if (debug) fprintf(stderr,"%s seq %d :: rngoff %d rxchn %d\n"
        //                          "%s seq %d :: ACFSumPower\n",
        //                   station,nave,rngoff,rxchn,station,nave);
        aflg = ACFSumPower(&tsgprm,mplgs,lagtable,pwr0,(int16 *)dest,
                           rngoff,skpnum!=0,roff,ioff,badrng,noise,mxpwr,
                           seqatten[nave]*atstp,thr,lmt,&abflg);
        //if (debug) fprintf(stderr,"%s seq %d :: rngoff %d rxchn %d\n"
        //                          "%s seq %d :: ACFCalculate acf\n",
        //                   station,nave,rngoff,rxchn,station,nave);
        ACFCalculate(&tsgprm,(int16 *)dest,rngoff,skpnum!=0,roff,ioff,mplgs,
                     lagtable,acfd,ACF_PART,2*dprm.samples,badrng,
                     seqatten[nave]*atstp,NULL);
        if (xcf == 1) {
          //if (debug) fprintf(stderr,"%s seq %d :: rngoff %d rxchn %d\n"
          //                          "%s seq %d :: ACFCalculate xcf\n",
          //                   station,nave,rngoff,rxchn,station,nave);
          ACFCalculate(&tsgprm,(int16 *)dest,rngoff,skpnum!=0,roff,ioff,mplgs,
                       lagtable,xcfd,XCF_PART,2*dprm.samples,badrng,
                       seqatten[nave]*atstp,NULL);
        }
        if ((nave > 0) && (seqatten[nave] != seqatten[nave-1])) {
          //if (debug) fprintf(stderr,"%s seq %d :: rngoff %d rxchn %d\n"
          //                          "%s seq %d :: ACFNormalize\n",
          //                   station,nave,rngoff,rxchn,station,nave);
          ACFNormalize(pwr0,acfd,xcfd,tsgprm.nrang,mplgs,atstp);
        }
        //if (debug) fprintf(stderr,"%s seq %d :: rngoff %d rxchn %d\n",
        //                   station,nave,rngoff,rxchn);
      }
      //if (debug) fprintf(stderr,"Sending ACK for seq %d\n", nave);

      TCPIPMsgSend(ros.sock, &nave, sizeof(int32_t));

      iqoff = iqsze;  /* set the offset bytes for the next sequence */
    }
  } /* end of while loop */

  /* recv GET_DATA command status */
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"GET_DATA:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
    sprintf(logtxt,"GET_DATA:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosIntegrate",logtxt);
  }

  /* Now divide by nave to get the average pwr0 and acfd values for the
     integration period */

  if (mplgexs == 0) {
    if (nave > 0) {
      ACFAverage(pwr0,acfd,xcfd,nave,tsgprm.nrang,mplgs);
/*
      for (range=0; range < nrang;range++) {
        pwr0[range] = (double)pwr0[range]/(double)nave;

        for (lag=0;lag < mplgs; lag++) {
          acfd[range*(2*mplgs)+2*lag] = (double) acfd[range*(2*mplgs)+2*lag]/
                                      (double) nave;
          acfd[range*(2*mplgs)+2*lag+1] = (double) acfd[range*(2*mplgs)+2*lag+1]/
                                      (double) nave;
        }
      }
*/
    }
  } else if (nave > 0) {
    /* ACFEX calculation */
    ACFexCalculate(&tsgprm,(int16 *)samples,nave*smpnum,nave,smpnum,
                   roff,ioff,mplgs,mplgexs,lagtable,lagsum,pwr0,acfd,&noise);
  }
  free(lagtable[0]);
  free(lagtable[1]);
  //if (debug) {
  //  fprintf(stderr,"%s SiteIntegrate: iqsize in bytes: %ld in 16bit samples:"
  //                 "  %ld in 32bit samples: %ld\n",
  //                 station,(long int)iqsze,(long int)iqsze/2,(long int)iqsze/4);
  //  fprintf(stderr,"%s SiteIntegrate: end: nave: %d\n",station,nave);
  //}
  SiteRosExit(0);

  if (debug) ErrLog(errlog.sock,"SiteRosIntegrate","Leaving SiteRosIntegrate");

  return nave;
}


int SiteRosEndScan(int bsc,int bus, unsigned sleepus) {

  struct ROSMsg smsg,rmsg;
  struct timeval tock;
  struct timeval tick;
  double bnd;
  double tme;
  int count=0;
  useconds_t sleep_left;

  char logtxt[1024]="";

  if (debug) ErrLog(errlog.sock,"SiteRosEndScan","Entering SiteRosEndScan");

  SiteRosExit(0);

  bnd = bsc + bus/USEC;

  if (debug) ErrLog(errlog.sock,"SiteRosEndScan","Sending SET_INACTIVE");
  smsg.type = SET_INACTIVE;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    sprintf(logtxt,"SET_INACTIVE:type=%c",rmsg.type);
    ErrLog(errlog.sock,"SiteRosEndScan",logtxt);
    sprintf(logtxt,"SET_INACTIVE:status=%d",rmsg.status);
    ErrLog(errlog.sock,"SiteRosEndScan",logtxt);
  }

  if (gettimeofday(&tock,NULL)==-1) return -1;

  tme = tock.tv_sec + tock.tv_usec/USEC;
  tme = bnd*floor(1.0+tme/bnd);
  tock.tv_sec  = tme;
  tock.tv_usec = (tme-floor(tme))*USEC;

  gettimeofday(&tick,NULL);
  smsg.type = PING;
  while (1) {
    if (tick.tv_sec > tock.tv_sec) break;
    if ((tick.tv_sec == tock.tv_sec) && (tick.tv_usec >= (tock.tv_usec-2000))) break;

    if (count >= 100) {
      TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
      TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
      count = 0;
    }

    //if (debug) {
    //  fprintf(stderr,"PING:type=%c\n",rmsg.type);
    //  fprintf(stderr,"PING:status=%d\n",rmsg.status);
    //  fprintf(stderr,"PING:count=%d\n",count);
    //  fflush(stderr);
    //}
    count++;
    SiteRosExit(0);

    sleep_left = (tock.tv_sec-tick.tv_sec)*USEC + (tock.tv_usec-tick.tv_usec) - 2000;
    if (sleep_left < sleepus) {
      usleep(sleep_left);
      break;
    } else {
      usleep(sleepus);
    }

    SiteRosExit(0);
    gettimeofday(&tick,NULL);
  }

  if (debug) ErrLog(errlog.sock,"SiteRosEndScan","Leaving SiteRosEndScan");

  return 0;
}

