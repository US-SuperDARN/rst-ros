/* rbspcan.c
 ============
 Author: S.G.Shepherd

 Based on themisscan.1.6 this is a 3 camping beam mode to work out the
  azimuthal wave number of ULF waves in support of the RBSP mission. The
  basic idea is that a mini 3 beam mode runs inside the full scan.

 This is not as flexible at the themisscan mode since the number of
  permutation with 3 camping beams is a lot.

 Details:
   - integration period is fixed to 3s for some reason that is not quite clear
     to me. Perhaps to synchronize data for analysis?

   - do NOT repeat any of the camping beams.
   - can change to 1-min scan but then limited to 11 beam extent for scan.

   - can add buffer at end of scan (see bufsc and bufus) which reduces number
     of beams sampled (with no buffer you have 40 beams in a 2-min scan)

   - MUST use -stid xxx with this control program or seg faults

 Updates:
   20121206 - changed cpid to 200, 201 for fast version
   20170501 - added PFISR flag to select beams on cvw that overlap PFISR

 */

/*
 license stuff should go here...
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <zlib.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "dmap.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iq.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"
#include "errlog.h"
#include "freq.h"
#include "tcpipmsg.h"
#include "rmsg.h"
#include "rmsgsnd.h"
#include "radarshell.h"
#include "build.h"
#include "global.h"
#include "reopen.h"
#include "sequence.h"
#include "setup.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";
void *tmpbuf;
size_t tmpsze;
char progid[80]={"rbspscan 2025/07/22"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: rbspscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024]="";
  char tempLog[40];

  int nowait=0;
  /* these are set for a standard 2-min scan, any changes here will affect
     the number of beams sampled, etc.
   */
  int scnsc=120;
  int scnus=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int cnt=0;
  int i,n;
  unsigned char fast=0;
  unsigned char pfisr=0;
  unsigned char tromso=0;               /* ICE beams over Tromso heater */
  unsigned char risr=0;                 /* ICW beams over RISR-N & -C   */
  unsigned char lyr=0;                  /* ICE beams over Longyearbyen  */
  unsigned char discretion=0;
  int status=0;
  int setintt=0;  /* flag to override auto-calc of integration time */

  int bufsc=3;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */

  /* Flag and variables for beam synchronizing */
  int bm_sync = 0;
  int bmsc    = 3;
  int bmus    = 0;

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int startup=1;
  int fixfrq=0;
  int clrscan=0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                     */
  int nbm=40;         /* number of integration periods per scan; SGS 2-min  */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;
  int cbm[3];         /* array to hold camping beams; only for display      */

  /*
    beam sequences for 24-beam MSI radars; camping beams are n-1,n,n+3 where
      n = 2 for east radars and n = 21 for west radars. Note that fhw will
      use the same sequence with beam-=2 to account for the fact that it is
      a 22-beam radar (fhe remains unchanged.)
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  /*          21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 */
  int bmse[40] =
             { 1, 0, 2, 3, 5, 4, 1, 6, 2, 7, 5, 8, 1, 9, 2,10, 5,11, 1,12,
               2,13, 5,14, 1,15, 2,16, 5,17, 1,18, 2,19, 5,20, 1,21, 2, 5};
  int bmsw[40] =
             {22,23,21,20,18,19,22,17,21,16,18,15,22,14,21,13,18,12,22,11,
              21,10,18, 9,22, 8,21, 7,18, 6,22, 5,21, 4,18, 3,22, 2,21,18};
  /* ICE camping beams over Tromso and Skiboten: 20, 21, 22 */
  int eiscatbms[40] =
             {20, 0,21, 1,22, 2,20, 3,21, 4,22, 5,20, 6,21, 7,22, 8,20, 9,
              21,10,22,11,20,12,21,13,22,14,20,15,21,16,22,17,20,18,21,19};
  /*
   * alt that 'camps' on 19,20,21
  int eiscatbms[40] =
             {19, 0,20, 1,21, 2,19, 3,20, 4,21, 5,19, 6,20, 7,21, 8,19, 9,
              20,10,21,11,19,12,20,13,21,14,19,15,20,16,21,17,19,18,20,21};
   */
  /* ICE camping beams over Longyearbyen: 11, 12, 13 */
  int lyrbms[40] =
             {11, 0,12, 1,13, 2,11, 3,12, 4,13, 5,11, 6,12, 7,13, 8,11, 9,
              12,10,13,14,11,15,12,16,13,17,11,18,12,19,13,20,11,21,12,13};
  /* ICW camping beams over RISR: 17, 16, 15 */
  int rbisr[40] =
             {17,23,16,22,15,21,17,20,16,19,15,18,17,14,16,13,15,12,17,11,
              16,10,15, 9,17, 8,16, 7,15, 6,17, 5,16, 4,15, 3,17, 2,16,15};
  /* cvw camping beams over PFISR: 12, 10, 8 */
  int pbisr[40] =
             {12,23,10,22, 8,21,12,20,10,19, 8,18,12,17,10,16, 8,15,12,14,
              10,13, 8,11,12, 9,10, 7, 8, 6,12, 5,10, 4, 8, 3,12, 2,10, 8};

  /* use odd/even beams to generate a 1-min scan at lower spatial resolution */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  /*          21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 */
  int alte[40] = { 1, 7, 2, 9, 5,11, 1,13, 2,15, 5,17, 1,19, 2,21, 5, 0, 1, 4,
               2, 6, 5, 8, 1,10, 2,12, 5,14, 1,16, 2,18, 5,20, 1, 3, 2, 5};
  int altw[40] = {22,16,21,14,18,12,22,10,21, 8,18, 6,22, 4,21, 2,18,23,22,19,
              21,17,18,15,22,13,21,11,18, 9,22, 7,21, 5,18, 3,22,20,21,18};

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  /* standard radar defaults */
  cp     = 200;         /* rbsbscan cpid SGS need to pick something */
  intsc  = 3;           /* integration period; not sure how critical this is */
  intus  = 0;           /*  but can be changed here */
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  rsep   = 45;
  txpl   = 300;         /* note: recomputed below */
  nbaud  = 1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */
  OptionAdd(&opt,"di",    'x',&discretion);
  OptionAdd(&opt,"fast",  'x',&fast);
  OptionAdd(&opt,"pfisr", 'x',&pfisr);
  OptionAdd(&opt,"tromso",'x',&tromso);
  OptionAdd(&opt,"risr",  'x',&risr);
  OptionAdd(&opt,"lyr",   'x',&lyr);
  OptionAdd(&opt,"frang", 'i',&frang);
  OptionAdd(&opt,"rsep",  'i',&rsep);
  OptionAdd(&opt,"dt",    'i',&day);
  OptionAdd(&opt,"nt",    'i',&night);
  OptionAdd(&opt,"df",    'i',&dfrq);
  OptionAdd(&opt,"nf",    'i',&nfrq);
  OptionAdd(&opt,"xcf",   'x',&xcnt);
  OptionAdd(&opt,"nrang", 'i',&nrang);
  OptionAdd(&opt,"ep",    'i',&errlog.port);
  OptionAdd(&opt,"sp",    'i',&shell.port);
  OptionAdd(&opt,"bp",    'i',&baseport);
  OptionAdd(&opt,"stid",  't',&ststr);
  OptionAdd(&opt,"nowait",'x',&nowait);
  OptionAdd(&opt,"clrscan",'x',&clrscan);
  OptionAdd(&opt,"clrskip",'i',&clrskip);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt,"bm_sync",'x',&bm_sync);   /* flag to enable beam sync   */
  OptionAdd(&opt,"bmsc",  'i',&bmsc);       /* beam sync period, sec      */
  OptionAdd(&opt,"bmus",  'i',&bmus);       /* beam sync period, microsec */
  OptionAdd(&opt,"intsc", 'i',&intsc);
  OptionAdd(&opt,"intus", 'i',&intus);
  OptionAdd(&opt,"setintt",'x',&setintt);
  OptionAdd(&opt,"c",     'i',&cnum);
  OptionAdd(&opt,"ros",   't',&roshost);    /* Set the roshost IP address */
  OptionAdd(&opt,"debug", 'x',&debug);
  OptionAdd(&opt,"-help", 'x',&hlp);        /* just dump some parameters */
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);
  OptionAdd(&opt,"baud",  'i',&nbaud);
  OptionAdd(&opt,"tau",   'i',&mpinc);

  /* Process all of the command line options
      Important: need to do this here because we need stid and ststr */
  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }

  if (version==1) {
    OptionVersion(stdout);
    exit(0);
  }

  if (ststr==NULL) ststr=dfststr;

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    cbm[0] = 1;
    cbm[1] = 2;
    cbm[2] = 5;
    if (fast)
      bms = alte;       /* odd/even beam sequence */
    else
      bms = bmse;       /* standard 2-min sequence */
    if (tromso) {	/* ICE camp over Tromso and Skiboten */
      bms = eiscatbms;
      cbm[0] = 20;
      cbm[1] = 21;
      cbm[2] = 22;
    } else if (lyr) {	/* ICE camping beams over Longyearbyen */
      bms = lyrbms;
      cbm[0] = 11;
      cbm[1] = 12;
      cbm[2] = 13;
    }
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0) || (strcmp(ststr,"bks") == 0)) {
    cbm[0] = 22;
    cbm[1] = 21;
    cbm[2] = 18;
    if (fast)
      bms = altw;       /* odd/even beam sequence */
    else
      bms = bmsw;       /* standard 2-min sequence */
    if (pfisr) {        /* cvw camping beams over PFSIR */
      bms = pbisr;
      cbm[0] = 12;
      cbm[1] = 10;
      cbm[2] =  8;
    } else if (risr) {	/* ICW camping beams over RISR */
      bms = rbisr;
      cbm[0] = 17;
      cbm[1] = 16;
      cbm[2] = 15;
    }
  } else if (strcmp(ststr,"fhw") == 0) {
    cbm[0] = 20;
    cbm[1] = 19;
    cbm[2] = 16;
    if (fast)
      bms = altw;       /* odd/even beam sequence */
    else
      bms = bmsw;       /* standard 2-min sequence */
    for (i=0; i<nbm; i++)
      bms[i] -= 2;      /* decrement beams by 2 for 22 beam radar */
  } else {
    if (hlp) usage();
    else     printf("Error: Not intended for station %s\n", ststr);
    return (-1);
  }

  if (hlp) {
    usage();

/*  printf("  start beam: %2d\n", sbm); */
/*  printf("  end   beam: %2d\n", ebm); */
    printf("\n");
    printf("sqnc  bmno\n");
    for (i=0; i<nbm; i++) {
      printf(" %2d   %2d", i, bms[i]);
      for (n=0; n<3; n++)
        if (bms[i] == cbm[n]) printf("   CAMP");
      printf("\n");
    }

    return (-1);
  }

  nBeams_per_scan = nbm;

  if (bm_sync) {
    sync_scan = 1;
    scan_times = malloc(nBeams_per_scan*sizeof(int));
  }

  for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = bms[iBeam];
    if (bm_sync) scan_times[iBeam] = iBeam * (bmsc*1000 + bmus/1000); /* in ms*/
  }

  /* recomputing the integration period for each beam */
  /* Note that I have added a buffer here to account for things at the end
     of the scan. Traditionally this has been set to 3s, but I cannot find
     any justification of the need for it. -SGS */
  if ((nowait==0) && (setintt==0)) {
    total_scan_usecs = scnsc*1e6 + scnus - (bufsc*1e6 + bufus);
/*  total_scan_usecs = (scnsc-3)*1E6+scnus; */
    total_integration_usecs = total_scan_usecs/nBeams_per_scan;
    intsc = total_integration_usecs/1e6;
    intus = total_integration_usecs -(intsc*1e6);
  }

  /* Configure phasecoded operation if nbaud > 1 */
  pcode=(int *)malloc((size_t)sizeof(int)*seq->mppul*nbaud);
  OpsBuildPcode(nbaud,seq->mppul,pcode);

  txpl=(nbaud*rsep*20)/3;

  /* Attempt to adjust mpinc to be a multiple of 10 and a multiple of txpl */
  if ((mpinc % txpl) || (mpinc % 10))  {
    ErrLog(errlog.sock,progname,"Error: mpinc not multiple of txpl... checking to see if it can be adjusted");
    sprintf(logtxt,"Initial: mpinc: %d  txpl: %d  nbaud: %d  rsep: %d", mpinc, txpl, nbaud, rsep);
    ErrLog(errlog.sock,progname,logtxt);

    if ((txpl % 10) == 0) {
      ErrLog(errlog.sock,progname, "Attempting to adjust mpinc to be correct");
      if (mpinc < txpl) mpinc = txpl;
      int minus_remain = mpinc % txpl;
      int plus_remain = txpl - (mpinc % txpl);
      if (plus_remain > minus_remain) mpinc = mpinc - minus_remain;
      else                            mpinc = mpinc + plus_remain;
      if (mpinc == 0) mpinc = mpinc + plus_remain;
    }
  }

  /* Check mpinc and if still invalid, exit with error */
  if ((mpinc % txpl) || (mpinc % 10) || (mpinc==0))  {
    sprintf(logtxt,"Error: mpinc: %d  txpl: %d  nbaud: %d  rsep: %d", mpinc, txpl, nbaud, rsep);
    ErrLog(errlog.sock,progname,logtxt);
    SiteExit(0);
  }

  /* end of main Dartmouth mods */
  /* not sure if -nrang commandline option works */

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  OpsStart(ststr);

  status=SiteBuild(libstr);

  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  /* IMPORTANT: sbm and ebm are reset by this function */
  status = SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  /* Reprocess the command line to restore desired parameters */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;       /* this almost certainly got reset */

  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  /* dump beams to log file */
  sprintf(progname,"rbspscan");
  for (i=0; i<nbm; i++){
    sprintf(tempLog, "%3d", bms[i]);
    strcat(logtxt, tempLog);
  }
  ErrLog(errlog.sock,progname,logtxt);

  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l"
                  " frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &frqrng,&xcnt);

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  status=SiteSetupRadar();

  fprintf(stderr,"Status: %d\n",status);

  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  /* Initialize timing variables */
  elapsed_secs=0;
  gettimeofday(&t1,NULL);
  gettimeofday(&t0,NULL);

  if (fast) cp += 1;        /* SGS: correct cpid? */
  if (discretion) cp = -cp;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
  }

  OpsFitACFStart();

  tsgid=SiteTimeSeq(seq->ptab);    /* get the timing sequence */

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  /* Synchronize start of first scan to minute boundary */
  if (nowait==0) {
    ErrLog(errlog.sock,progname,"Synchronizing to scan boundary.");
    SiteEndScan(scnsc,scnus,100000);
  }

  do {

    /* reset clearfreq parameters, in case daytime changed */
    for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
      scan_clrfreq_fstart_list[iBeam] = (int32_t) (OpsDayNight() == 1 ? dfrq : nfrq);
      scan_clrfreq_bandwidth_list[iBeam] = frqrng;
    }

    /* set iBeam for scan loop */
    iBeam = 0;

    /* send scan data to usrp_sever */
    if (SiteStartScan(nBeams_per_scan, scan_beam_number_list, scan_clrfreq_fstart_list,
                      scan_clrfreq_bandwidth_list, fixfrq, sync_scan, scan_times,
                      scnsc, scnus, intsc, intus, iBeam) !=0) {
      ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");
      sleep(1);
      continue;
    }

    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    if (OpsReOpen(1,59,59) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan=1;

    ErrLog(errlog.sock,progname,"Starting scan.");

    if (clrscan) startup=1;
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;


    do {

      bmnum = scan_beam_number_list[iBeam];

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      stfrq = scan_clrfreq_fstart_list[iBeam];
      if (fixfrq > 0) {
        stfrq=fixfrq;
        tfreq=fixfrq;
        noise=0;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",bmnum,
              intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      SiteStartIntt(intsc,intus);
      gettimeofday(&t1,NULL);
      elapsed_secs=t1.tv_sec-t0.tv_sec;
      if (elapsed_secs<0) elapsed_secs=0;
      if ((elapsed_secs >= clrskip) || (startup==1)) {
          startup = 0;
          ErrLog(errlog.sock,progname,"Doing clear frequency search.");
          sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
          ErrLog(errlog.sock,progname, logtxt);

          if (fixfrq<=0) {
              tfreq=SiteFCLR(stfrq,stfrq+frqrng);
          }
          t0.tv_sec  = t1.tv_sec;
          t0.tv_usec = t1.tv_usec;
      }
      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      nave=SiteIntegrate(seq->lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error: %d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,seq->ptab,seq->lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);

      FitACF(prm,raw,fblk,fit,site,tdiff,-999);
      FitSetAlgorithm(fit,"fitacf2");

      msg.num=0;
      msg.tsize=0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *) badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *) sharedmemory,
                 IQS_TYPE,0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan = 0;

      iBeam++;
      if (iBeam >= nBeams_per_scan) break;

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    if (nowait==0) SiteEndScan(scnsc,scnus,50000);
  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
  printf("\nrbspscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("  -stid char: radar string (required)\n");
  printf("    -di     : indicates running during discretionary time\n");
  printf("  -fast     : run beam sequence that can do full scans in 1-min\n");
  printf(" -frang int : delay to first range (km) [180]\n");
  printf("  -rsep int : range separation (km) [45]\n");
  printf("    -dt int : hour when day freq. is used\n");
  printf("    -nt int : hour when night freq. is used\n");
  printf("    -df int : daytime frequency (kHz)\n");
  printf("    -nf int : nighttime frequency (kHz)\n");
  printf("   -xcf     : set for computing XCFs\n");
  printf(" -nrang int : number of range gates\n");
  printf("    -ep int : error log port\n");
  printf("    -sp int : shell port\n");
  printf("    -bp int : base port\n");
  printf("-nowait     : Do not wait for minute scan boundary\n");
  printf("-clrscan    : Force clear frequency search at start of scan\n");
  printf("-clrskip int: Minimum number of seconds to skip between clear frequency search\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("-bm_sync    : set to enable beam syncing.\n");
  printf("  -bmsc int : beam syncing interval seconds.\n");
  printf("  -bmus int : beam syncing interval microseconds.\n");
  printf("-setintt    : set to enable integration period override.\n");
  printf(" -intsc int : integration period seconds.\n");
  printf(" -intus int : integration period microseconds.\n");
  printf("  -baud int : baud to use for Barker phase coded sequence (1,2,3,4,5,7,11,13) [1]\n");
  printf("   -tau int : lag spacing in usecs [1500]\n");
  printf("     -c int : channel number for multi-channel radars.\n");
  printf("   -ros char: change the roshost IP address\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

