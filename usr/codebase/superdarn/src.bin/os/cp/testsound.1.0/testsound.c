/* testsound.c
   ===========
   Author: E.G.Thomas

*/

/*
 license stuff should go here...
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <zlib.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "dmap.h"
#include "shmem.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iq.h"
#include "iqwrite.h"
#include "rawdata.h"
#include "rawwrite.h"
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
#include "rosmsg.h"
#include "tsg.h"

void write_raw_snd_record(char *progname, struct RadarParm *prm,
                          struct RawData *raw);
void write_iq_snd_record(char *progname, struct RadarParm *prm,
                         struct IQ *iq, unsigned int *badtr);

#define RT_TASK 3


char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"testsound 2025/10/24"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: testsound --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

  char logtxt[1024];

  int scnsc=120;    /* total scan period in seconds */
  int scnus=0;
  int cnt=0;

  unsigned char fast=1;
  unsigned char slow=0;
  unsigned char discretion=0;

  int i,n;
  int status=0;

  int total_scan_usecs=0;
  int total_integration_usecs=0;
  int def_intt_sc=0;
  int def_intt_us=0;
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

  int32_t snd_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_bc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_fc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_nBeams_per_scan = 0;
  int snd_iBeam;

  unsigned char iq_flg=0;
  unsigned char no_raw=0;

  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;


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
  int snd_sc=-1;
  int snd_intt_sc=1;
  int snd_intt_us=500000;
  float time_needed=3.0;
  /* ------------------------------------------------------- */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp     = 1100;        /* CPID */
  intsc  = 7;           /* integration period; recomputed below ... */
  intus  = 0;
  mppul  = seq->mppul;  /* number of pulses */
  mplgs  = seq->mplgs;  /* number of lags */
  mpinc  = seq->mpinc;  /* multi-pulse increment [us] */
  rsep   = 45;          /* same for the range separation */
  txpl   = 300;         /* pulse length [us]; gets redefined below... */
  nbaud  = 1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "di",     'x', &discretion);
  OptionAdd(&opt, "frang",  'i', &frang);
  OptionAdd(&opt, "rsep",   'i', &rsep);
  OptionAdd(&opt, "nrang",  'i', &nrang);
  OptionAdd(&opt, "dt",     'i', &day);
  OptionAdd(&opt, "nt",     'i', &night);
  OptionAdd(&opt, "df",     'i', &dfrq);
  OptionAdd(&opt, "nf",     'i', &nfrq);
  OptionAdd(&opt, "xcf",    'x', &xcnt);
  OptionAdd(&opt, "ep",     'i', &errlog.port);
  OptionAdd(&opt, "sp",     'i', &shell.port);
  OptionAdd(&opt, "bp",     'i', &baseport);
  OptionAdd(&opt, "stid",   't', &ststr);
  OptionAdd(&opt, "fast",   'x', &fast);
  OptionAdd(&opt, "slow",   'x', &slow);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt, "frqrng", 'i', &frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt, "sfrqrng",'i', &snd_frqrng); /* sounding FCLR window [kHz] */
  OptionAdd(&opt, "sndsc",  'i', &snd_sc);     /* sounding duration per scan [sec] */
  OptionAdd(&opt, "iqdat",  'x', &iq_flg);     /* store IQ samples */
  OptionAdd(&opt, "noraw",  'x', &no_raw);     /* do not store rawacf data */
  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);    /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);        /* just dump some parameters */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt, "-version",'x',&version);

  /* process the commandline; need this for setting errlog port */
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

  backward = (sbm > ebm) ? 1 : 0;   /* allow for non-standard scan dir. */

  if (hlp) {
    usage();

    printf("  start beam: %2d\n", sbm);
    printf("  end   beam: %2d\n", ebm);
    if (backward) printf("  backward\n");
    else printf("  forward\n");

    return (-1);
  }

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    snd_bms = snd_bmse;
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0) || (strcmp(ststr,"bks") == 0)) {
    snd_bms = snd_bmsw;
  } else if (strcmp(ststr,"fhw") == 0) {
    snd_bms = snd_bmsw;
    for (i=0; i<snd_bms_tot; i++)
      snd_bms[i] -= 2;
  } else {
    snd_bms = snd_bmse;
    snd_bms_tot = 8;
    snd_intt_sc = 2;
    snd_intt_us = 0;
  }

  OpsStart(ststr);

  /* Load the sounding frequencies */
  OpsLoadSndFreqs(ststr);

  status=SiteBuild(libstr);
  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  status = SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  /* reprocess the commandline since some things are reset by SiteStart */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  if (slow) fast = 0;

  if (fast) sprintf(progname,"testsound (fast)");
  else sprintf(progname,"testsound");

  printf("Station ID: %s  %d\n",ststr,stid);
  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l"
                  " frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &frqrng,&xcnt);

  nBeams_per_scan = abs(ebm-sbm)+1;
  current_beam = sbm;

  if (fast) {
    cp    = 1101;
    scnsc = 60;
    scnus = 0;
    if (snd_sc < 0) snd_sc = 12;
  } else {
    scnsc = 120;
    scnus = 0;
    if (snd_sc < 0) snd_sc = 20;
  }

  sync_scan = 0;

  for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = current_beam;
    current_beam += backward ? -1:1;
  }

  snd_nBeams_per_scan = (snd_sc-time_needed)/(snd_intt_sc + snd_intt_us*1e-6);

  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-snd_sc)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  def_intt_sc = total_integration_usecs/1E6;
  def_intt_us = total_integration_usecs - (def_intt_sc*1e6);

  intsc = def_intt_sc;
  intus = def_intt_us;

  /* Configure phasecoded operation if nbaud > 1 */
  pcode=(int *)malloc((size_t)sizeof(int)*seq->mppul*nbaud);
  OpsBuildPcode(nbaud,seq->mppul,pcode);

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  status=SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  printf("Initial Setup Complete: Station ID: %s  %d\n",ststr,stid);

  /* Initialize timing variables */
  elapsed_secs=0;
  gettimeofday(&t1,NULL);
  gettimeofday(&t0,NULL);

  if (discretion) cp = -cp;

  txpl=(nbaud*rsep*20)/3;

  def_nrang = nrang;
  def_rsep = rsep;
  def_txpl = txpl;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Preparing OpsSndStart Station ID: %s  %d\n",ststr,stid);
  OpsSndStart();

  OpsFindSndSkip(ststr,snd_bms,snd_bms_tot,&snd_bm_cnt,&odd_beams);

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  if ((def_nrang == snd_nrang) && (def_rsep == snd_rsep)) {
    printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
    tsgid=SiteTimeSeq(seq->ptab);
  }

  /* Synchronize start of first scan to minute boundary */
  ErrLog(errlog.sock,progname,"Synchronizing to scan boundary.");
  SiteEndScan(scnsc,scnus,5000);

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {

    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
      printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
      tsgid=SiteTimeSeq(seq->ptab);
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

    scan = 1;   /* scan flag */

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
      if (fixfrq>0) {
        stfrq=fixfrq;
        tfreq=fixfrq;
        noise=0;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",
                     bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);

      /* clear frequency search business */
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
      if (nave < 0) {
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
      RMsgSndAdd(&msg,tmpsze,tmpbuf, PRM_TYPE,0);

      tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *) badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *)sharedmemory,
                 IQS_TYPE,0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0; n<msg.num; n++) {
        if ( (msg.data[n].type == PRM_TYPE) ||
             (msg.data[n].type == IQ_TYPE)  ||
             (msg.data[n].type == RAW_TYPE) ||
             (msg.data[n].type == FIT_TYPE) )  free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan = 0;

      iBeam++;
      if (iBeam >= nBeams_per_scan) break;

    } while (1);

    SiteEndScan(1,0,5000);

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

      if (!no_raw) {
        write_raw_snd_record(progname, prm, raw);
      }

      if (iq_flg) {
        OpsBuildIQ(iq,&badtr);
        write_iq_snd_record(progname, prm, iq, badtr);
      }

      ErrLog(errlog.sock, progname, "Polling SND for exit.");

      snd_iBeam++;
      if (snd_iBeam >= snd_nBeams_per_scan) break;

    } while (1);

    /* now wait for the next normalscan */
    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    /* make sure we didn't miss any beams/frequencies */
    if (snd_iBeam < snd_nBeams_per_scan-1) {
      snd_bm_cnt = snd_bc[snd_iBeam-1];
      snd_freq_cnt = snd_fc[snd_iBeam-1];
    }

    intsc = def_intt_sc;
    intus = def_intt_us;
    nrang = def_nrang;
    rsep = def_rsep;
    txpl = def_txpl;

    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\ntestsound [command-line options]\n\n");
    printf("command-line options:\n");
    printf("  -stid char: radar string (required)\n");
    printf("    -di     : indicates running during discretionary time\n");
    printf(" -frang int : delay to first range (km) [180]\n");
    printf("  -rsep int : range separation (km) [45]\n");
    printf(" -nrang int : number of range gates\n");
    printf("    -dt int : hour when day freq. is used\n");
    printf("    -nt int : hour when night freq. is used\n");
    printf("    -df int : daytime frequency (kHz)\n");
    printf("    -nf int : nighttime frequency (kHz)\n");
    printf("   -xcf     : set for computing XCFs\n");
    printf("    -sb int : starting beam\n");
    printf("    -eb int : ending beam\n");
    printf("    -ep int : error log port\n");
    printf("    -sp int : shell port\n");
    printf("    -bp int : base port\n");
    printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
    printf("-frqrng int : set the clear frequency search window (kHz)\n");
    printf("-sfrqrng int: set the sounding FCLR search window (kHz)\n");
    printf(" -sndsc int : set the sounding duration per scan (sec)\n");
    printf(" -iqdat     : set for storing snd IQ samples\n");
    printf(" -noraw     : do not write snd rawacf files\n");
    printf("     -c int : channel number for multi-channel radars.\n");
    printf("   -ros char: change the roshost IP address\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}


/********************** function write_raw_snd_record() ************************/
/* changed the output to rawacf format */

void write_raw_snd_record(char *progname, struct RadarParm *prm, struct RawData *raw) {

  char data_path[60], data_filename[40], filename[110];

  char *snd_dir;
  FILE *out;

  char logtxt[1024]="";
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd.rawacf", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Rawacf Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding rawacf file: %s",filename);
    ErrLog(errlog.sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = RawFwrite(out, prm, raw);
  if (status == -1) {
    ErrLog(errlog.sock,progname,"Error writing sounding rawacf record.");
  } else {
    ErrLog(errlog.sock,progname,"Sounding rawacf record successfully written.");
  }

  fclose(out);
}


/********************** function write_iq_snd_record() ************************/
/* changed the output to iqdat format */

void write_iq_snd_record(char *progname, struct RadarParm *prm, struct IQ *iq,
                         unsigned int *badtr) {

  char data_path[60], data_filename[40], filename[110];

  char *snd_dir;
  FILE *out;

  int fd,iqbufsize;
  unsigned char *p;

  char logtxt[1024]="";
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd.iqdat", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Iqdat Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding iqdat file: %s",filename);
    ErrLog(errlog.sock,progname,logtxt);
    return;
  }

  iqbufsize = ShMemSizeName(sharedmemory);
  p = ShMemAlloc(sharedmemory,iqbufsize,O_RDWR,0,&fd);

  /* write the sounding record */
  status = IQFwrite(out, prm, iq, badtr, (int16 *)p);
  if (status == -1) {
    ErrLog(errlog.sock,progname,"Error writing sounding iqdat record.");
  } else {
    ErrLog(errlog.sock,progname,"Sounding iqdat record successfully written.");
  }
  ShMemFree(p,sharedmemory,iqbufsize,0,fd);

  fclose(out);
}
