/* interleavesound.c
   =================
   Author: E.G.Thomas

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
#include "snddata.h"
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
#include "snd.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

#define RT_TASK 3

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";
void *tmpbuf;
size_t tmpsze;
char progid[80]={"interleavesound 2025/12/15"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: interleavesound --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024]="";
  char tempLog[40];

  int scnsc=60;
  int scnus=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int cnt=0;
  int i,n;
  unsigned char discretion=0;
  int status=0;

  int def_nrang=0;
  int def_rsep=0;
  int def_txpl=0;

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int startup=1;
  int fixfrq=0;
  int clrscan=0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                     */
  int nintgs=20;      /* number of integration periods per scan; SGS 1-min  */

  unsigned char iq_flg=0;
  unsigned char raw_flg=0;

  int32_t snd_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_bc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_fc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_nBeams_per_scan = 0;
  int snd_iBeam;

  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  /*
    beam sequences for 24-beam MSI radars but only using 20 most meridional
      beams;
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  /*          21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 */
  int bmse[20] =
             { 0, 4, 8,12,16, 2, 6,10,14,18, 1, 5, 9,13,17, 3, 7,11,15,19};
  int bmsw[20] =
             {23,19,15,11, 7,21,17,13, 9, 5,22,18,14,10, 6,20,16,12, 8, 4};

  /*
    beam sequences for 16-beam radars (forward and backward)
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 */
  int bmsf[16] =
             { 0, 4, 8,12, 2, 6,10,14, 1, 5, 9,13, 3, 7,11,15};
  int bmsb[16] =
             {15,11, 7, 3,13, 9, 5, 1,14,10, 6, 2,12, 8, 4, 0};

  /* ---------------- Variables for sounding --------------- */
  int *snd_bms;
  int snd_bmse[]={0,2,4,6,8,10,12,14,16,18};   /* beam sequences for 24-beam MSI radars using only */
  int snd_bmsw[]={22,20,18,16,14,12,10,8,6,4}; /*  the 20 most meridional beams */
  int snd_freq_cnt=0, snd_bm_cnt=0;
  int snd_bms_tot=10, odd_beams=0;
  int snd_freq;
  int snd_frqrng=100;
  int snd_nrang=75;
  int snd_rsep=45;
  int snd_txpl=300;
  int snd_sc=12;
  int fast_intt_sc=2;
  int fast_intt_us=400000;
  int snd_intt_sc=1;
  int snd_intt_us=500000;
  float time_needed=3.0;
  /* ------------------------------------------------------- */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  /* standard radar defaults */
  cp     = 197;
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  rsep   = 45;
  txpl   = 300;     /* note: recomputed below */
  nbaud  = 1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */
  OptionAdd(&opt,"di",    'x',&discretion);
  OptionAdd(&opt,"wide",  'x',&wide_tx);
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
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt,"frqrng",'i',&frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt,"sfrqrng",'i',&snd_frqrng); /* sounding FCLR window [kHz] */
  OptionAdd(&opt,"iqdat", 'x',&iq_flg);     /* store IQ samples */
  OptionAdd(&opt,"rawacf",'x',&raw_flg);    /* store rawacf data */
  OptionAdd(&opt,"c",     'i',&cnum);
  OptionAdd(&opt,"ros",   't',&roshost);    /* Set the roshost IP address */
  OptionAdd(&opt,"debug", 'x',&debug);
  OptionAdd(&opt,"-help", 'x',&hlp);        /* just dump some parameters */
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

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

  if (hlp) {
    usage();
    return (-1);
  }

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    bms = bmse;
    snd_bms = snd_bmse;
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0) || (strcmp(ststr,"bks") == 0)) {
    bms = bmsw;
    snd_bms = snd_bmsw;
  } else if (strcmp(ststr,"fhw") == 0) {
    bms = bmsw;
    snd_bms = snd_bmsw;
    for (i=0; i<nintgs; i++)
      bms[i] -= 2;
    for (i=0; i<snd_bms_tot; i++)
      snd_bms[i] -= 2;
  } else {
    if ((strcmp(ststr,"kap") == 0) || (strcmp(ststr,"ksr") == 0)) {
      bms = bmsb;
    } else {
      bms = bmsf;
    }
    nintgs = 16;
    fast_intt_sc = 3;
    fast_intt_us = 0;
    snd_bms = snd_bmse;
    snd_bms_tot = 8;
    snd_intt_sc = 2;
    snd_intt_us = 0;
  }

  nBeams_per_scan = nintgs;

  sync_scan = 0;

  for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = bms[iBeam];
  }

  snd_nBeams_per_scan = (snd_sc-time_needed)/(snd_intt_sc + snd_intt_us*1e-6);

  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-snd_sc)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  fast_intt_sc = total_integration_usecs/1E6;
  fast_intt_us = total_integration_usecs - (fast_intt_sc*1e6);

  intsc = fast_intt_sc;
  intus = fast_intt_us;

  /* Configure phasecoded operation if nbaud > 1 */
  pcode=(int *)malloc((size_t)sizeof(int)*seq->mppul*nbaud);
  OpsBuildPcode(nbaud,seq->mppul,pcode);

  OpsStart(ststr);

  /* Load the sounding frequencies */
  OpsLoadSndFreqs(ststr);

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
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  /* dump beams to log file */
  sprintf(progname,"interleavesound (fast)");
  for (i=0; i<nintgs; i++){
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

  if (discretion) cp = -cp;

  txpl=(nbaud*rsep*20)/3;     /* computing TX pulse length */

  def_nrang = nrang;
  def_rsep = rsep;
  def_txpl = txpl;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock, strlen((char *)command), command);
  }

  OpsFitACFStart();
  OpsSndStart();

  OpsFindSndSkip(ststr,snd_bms,snd_bms_tot,&snd_bm_cnt,&odd_beams);

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  if ((def_nrang == snd_nrang) && (def_rsep == snd_rsep)) {
    tsgid=SiteTimeSeq(seq->ptab);  /* get the timing sequence */
  }

  /* Synchronize start of first scan to minute boundary */
  ErrLog(errlog.sock,progname,"Synchronizing to scan boundary.");
  SiteEndScan(scnsc,scnus,100000);

  do {

    ErrLog(errlog.sock,progname,"Starting scan.");

    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
      tsgid=SiteTimeSeq(seq->ptab);  /* get the timing sequence */
    }

    /* reset clearfreq paramaters, in case daytime changed */
    for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
      scan_clrfreq_fstart_list[iBeam] = (int32_t) (OpsDayNight() == 1 ? dfrq : nfrq);
      scan_clrfreq_bandwidth_list[iBeam] = frqrng;
    }

    /* Set iBeam for scan loop  */
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

    scan = 1;

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

    SiteEndScan(1,0,100000);

    ErrLog(errlog.sock,progname,"Starting SND scan.");

    /* In here comes the sounder code */
    /* set the "sounder mode" scan variable */
    scan = -2;

    /* set the xcf variable to do cross-correlations (AOA) */
    if (xcnt > 1) xcf = 1;

    /* set the sounding mode integration time, number of ranges,
     * and range separation */
    intsc = snd_intt_sc;
    intus = snd_intt_us;
    nrang = snd_nrang;
    rsep = snd_rsep;
    txpl = snd_txpl;

    for (snd_iBeam=0; snd_iBeam < snd_nBeams_per_scan; snd_iBeam++) {
      snd_beam_number_list[snd_iBeam] = snd_bms[snd_bm_cnt] + odd_beams;
      snd_clrfreq_fstart_list[snd_iBeam] = snd_freqs[snd_freq_cnt];
      snd_clrfreq_bandwidth_list[snd_iBeam] = snd_frqrng;
      snd_bc[snd_iBeam] = snd_bm_cnt;
      snd_fc[snd_iBeam] = snd_freq_cnt;

      snd_freq_cnt++;
      if (snd_freq_cnt >= snd_freqs_tot) {
        /* reset the freq counter and increment the beam counter */
        snd_freq_cnt = 0;
        snd_bm_cnt++;
        if (snd_bm_cnt >= snd_bms_tot) {
          snd_bm_cnt = 0;
          odd_beams = !odd_beams;
        }
      }
    }

    /* Print out details of sounding beams */
    fprintf(stderr, "Sounding sequence details: \n");
    for (snd_iBeam=0; snd_iBeam < snd_nBeams_per_scan; snd_iBeam++) {
      fprintf(stderr, "  sequence %2d: beam: %2d freq: %5d, \n",snd_iBeam,
              snd_beam_number_list[snd_iBeam], snd_clrfreq_fstart_list[snd_iBeam] );
    }

    snd_iBeam = 0;

    /* send sounding scan data to usrp_sever */
    if (SiteStartScan(snd_nBeams_per_scan, snd_beam_number_list, snd_clrfreq_fstart_list,
                      snd_clrfreq_bandwidth_list, 0, sync_scan, scan_times, snd_sc, 0,
                      snd_intt_sc, snd_intt_us, snd_iBeam) !=0) {
      ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");
      sleep(1);
      continue;
    }

    /* make a new timing sequence for the sounding */
    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
      tsgid = SiteTimeSeq(seq->ptab);
    }

    /* we have time until the end of the minute to do sounding */
    /* minus a safety factor given in time_needed */
    do {

      /* set the beam */
      bmnum = snd_beam_number_list[snd_iBeam];

      /* snd_freq will be an array of frequencies to step through */
      snd_freq = snd_clrfreq_fstart_list[snd_iBeam];

      /* the scanning code is here */
      sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
      ErrLog(errlog.sock,progname,"Starting SND Integration.");
      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock, progname, "Doing SND clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", snd_freq, snd_frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq = SiteFCLR(snd_freq, snd_freq + snd_frqrng);

      sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock, progname, logtxt);

      nave = SiteIntegrate(seq->lags);
      if (nave < 0) {
        sprintf(logtxt, "SND integration error: %d", nave);
        ErrLog(errlog.sock,progname, logtxt);
        continue;
      }
      sprintf(logtxt,"Number of SND sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,seq->ptab,seq->lags);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit,site,tdiff,-999);

      msg.num = 0;
      msg.tsize = 0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      RMsgSndSend(task[RT_TASK].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if ( (msg.data[n].type == PRM_TYPE) ||
             (msg.data[n].type == FIT_TYPE) )  free(msg.ptr[n]);
      }

      sprintf(logtxt, "SBC: %d  SFC: %d", snd_bc[snd_iBeam], snd_fc[snd_iBeam]);
      ErrLog(errlog.sock, progname, logtxt);

      /* set the scan variable for the sounding mode data file only */
      if ((bmnum == snd_bms[0]) && (snd_freq == snd_freqs[0])) {
        prm->scan = 1;
      } else {
        prm->scan = 0;
      }

      OpsBuildSnd(snd,prm,fit);

      /* save the sounding mode data */
      OpsWriteSnd(errlog.sock, progname, snd, ststr);

      if (iq_flg) {
        OpsBuildIQ(iq,&badtr);
        OpsWriteSndIQ(errlog.sock, progname, prm, iq, badtr, ststr);
      }

      if (raw_flg) OpsWriteSndRaw(errlog.sock, progname, prm, raw, ststr);

      ErrLog(errlog.sock, progname, "Polling SND for exit.");

      snd_iBeam++;
      if (snd_iBeam >= snd_nBeams_per_scan) break;

    } while (1);

    /* now wait for the next interleavescan */
    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    /* make sure we didn't miss any beams/frequencies */
    if (snd_iBeam < snd_nBeams_per_scan-1) {
      snd_bm_cnt = snd_bc[snd_iBeam-1];
      snd_freq_cnt = snd_fc[snd_iBeam-1];
    }

    intsc = fast_intt_sc;
    intus = fast_intt_us;
    nrang = def_nrang;
    rsep = def_rsep;
    txpl = def_txpl;

    SiteEndScan(scnsc,scnus,100000);

  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\ninterleavesound [command-line options]\n\n");
    printf("command-line options:\n");
    printf("   -stid char: radar string (required)\n");
    printf("     -di     : indicates running during discretionary time\n");
    printf("   -wide     : use a wide transmission beam\n");
    printf("  -frang int : delay to first range (km) [180]\n");
    printf("   -rsep int : range separation (km) [45]\n");
    printf("     -dt int : hour when day freq. is used\n");
    printf("     -nt int : hour when night freq. is used\n");
    printf("     -df int : daytime frequency (kHz)\n");
    printf("     -nf int : nighttime frequency (kHz)\n");
    printf("    -xcf     : set for computing XCFs\n");
    printf("  -nrang int : number of range gates\n");
    printf("     -ep int : error log port\n");
    printf("     -sp int : shell port\n");
    printf("     -bp int : base port\n");
    printf(" -fixfrq int : transmit on fixed frequency (kHz)\n");
    printf(" -frqrng int : set the clear frequency search window (kHz)\n");
    printf("-sfrqrng int : set the sounding FCLR search window (kHz)\n");
    printf("  -iqdat     : set for storing snd IQ samples\n");
    printf(" -rawacf     : set for writing snd rawacf files\n");
    printf("      -c int : channel number for multi-channel radars.\n");
    printf("    -ros char: change the roshost IP address\n");
    printf("  --help     : print this message and quit.\n");
    printf("\n");
}

