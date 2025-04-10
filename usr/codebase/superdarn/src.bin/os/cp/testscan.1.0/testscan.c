/* testscan.c
 ============
 Author: E.G.Thomas
         S.G.Shepherd

 A mode which slightly changes the beam sequence such that the scan no
   longer starts on the overlapping meridional beams, to try to debug
   the noise problem on those beams

 Updates:
   20160531 - first version
   20221123 - adapted from interleavescan to use beam arrays

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
char progid[80]={"testscan 2025/04/10"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: testscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024]="";
  char tempLog[40];

  int nowait=0;
  int scnsc=0;
  int scnus=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int cnt=0;
  int i,n;
  unsigned char fast=0;
  unsigned char discretion=0;
  int status=0;
  int setintt=0;  /* flag to override auto-calc of integration time */

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int startup=1;
  int fixfrq=0;
  int clrscan=0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                      */
  int intgt[24];      /* start times of each integration period              */
  int nintgs=20;      /* number of integration periods per scan;             */
  int bufsc=3;        /* a buffer at the end of scan; historically this has  */
  int bufus=0;        /*   been set to 3.0s to account for what???           */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  /* Flag and variables for beam synchronizing */
  int bm_sync = 0;
  int bmsc    = 6;
  int bmus    = 0;

  /*
    beam sequences for 24-beam MSI radars but only using 20 most meridional
      beams;
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  int bmse[20] =
             { 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19, 0};
  int bmsw[20] =
             {22,21,20,19,18,17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4,23};

  int bms_bks[24] =
             {22,21,20,19,18,17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 23};

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp     = 130;         /* not using 150 normalscan to keep Simon happy */
  intsc  = 7;           /* integration period; not sure how critical this is */
  intus  = 0;           /*  but can be changed here */
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  rsep   = 45;
  txpl   = 300;         /* note: recomputed below */
  nbaud  = 1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */
  OptionAdd(&opt,"di",    'x',&discretion);
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
  OptionAdd(&opt,"fast",  'x',&fast);
  OptionAdd(&opt,"nowait",'x',&nowait);
  OptionAdd(&opt,"clrscan",'x',&clrscan);
  OptionAdd(&opt,"clrskip",'i',&clrskip);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */

  OptionAdd(&opt, "bm_sync",'x', &bm_sync);  /* flag to enable beam sync    */
  OptionAdd(&opt, "bmsc",   'i', &bmsc);     /* beam sync period, sec       */
  OptionAdd(&opt, "bmus",   'i', &bmus);     /* beam sync period, microsec  */
  OptionAdd(&opt, "intsc",  'i', &intsc);
  OptionAdd(&opt, "intus",  'i', &intus);
  OptionAdd(&opt, "setintt",'x', &setintt);
  OptionAdd(&opt, "scnsc",  'i', &scnsc);     /* set the scan time */
  OptionAdd(&opt, "scnus",  'i', &scnus);

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

  /* start time of each integration period */
  for (i=0; i<nintgs; i++)
    intgt[i] = i*(intsc + intus*1e-6);

  if (ststr==NULL) ststr=dfststr;

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    bms = bmse;
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0)) {
    bms = bmsw;
  } else if (strcmp(ststr,"bks") == 0) {
    bms = bms_bks;
    nintgs = 24;
  } else if (strcmp(ststr,"fhw") == 0) {
    bms = bmsw;
    for (i=0; i<nintgs; i++)
      bms[i] -= 2;
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
    printf("sqnc  stme  bmno\n");
    for (i=0; i<nintgs; i++) {
      printf(" %2d   %3d    %2d", i, intgt[i], bms[i]);
      printf("\n");
    }

    return (-1);
  }

  nBeams_per_scan = nintgs;

  if (scnsc + scnus == 0) {  /* not set with command-line */
    if (fast) {     /* simple operation: 2-min scan or 1-min with fast set */
     cp    = 131;
     scnsc = 60;
     scnus = 0;
    } else {
     scnsc = 120;
     scnus = 0;
    }
  }

  if (bm_sync) {
    sync_scan = 1;
    scan_times = malloc(nBeams_per_scan*sizeof(int));
  }

  for (iBeam = 0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = bms[iBeam];
    if (bm_sync) scan_times[iBeam] = iBeam * (bmsc*1000 + bmus/1000); /* in ms */
  }

  /* Automatically calculate the integration times */
  if ((nowait==0) && (setintt==0)) {
    total_scan_usecs = scnsc*1e6 + scnus - (bufsc*1e6 + bufus);
/*  total_scan_usecs = (scnsc-3)*1E6+scnus; */
    total_integration_usecs = total_scan_usecs/nBeams_per_scan;
    intsc = total_integration_usecs/1E6;
    intus = total_integration_usecs - (intsc*1E6);
  }

  /* Configure phasecoded operation if nbaud > 1 */
  pcode=(int *)malloc((size_t)sizeof(int)*seq->mppul*nbaud);
  OpsBuildPcode(nbaud,seq->mppul,pcode);

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
  if (fast) sprintf(progname,"testscan (fast)");
  else      sprintf(progname,"testscan");

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

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
  }

  OpsFitACFStart();

  tsgid=SiteTimeSeq(seq->ptab);  /* get the timing sequence */

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  do {

    /* reset clearfreq parameters, in case daytime changed */
    for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
      scan_clrfreq_fstart_list[iBeam] = (int32_t) (OpsDayNight() == 1 ? dfrq : nfrq);
      scan_clrfreq_bandwidth_list[iBeam] = frqrng;
    }

    /* set iBeam for scan loop */
    if (nowait == 0) {
      iBeam = OpsFindSkip(scnsc,scnus,intsc,intus,nBeams_per_scan);
    } else {
      iBeam = 0;
    }

    /* send scan data to usrp_sever */
    if (SiteStartScan(nBeams_per_scan, scan_beam_number_list, scan_clrfreq_fstart_list,
                      scan_clrfreq_bandwidth_list, fixfrq, sync_scan, scan_times,
                      scnsc, scnus, intsc, intus, iBeam) !=0) {
      ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");
      sleep(1);
      continue;
    }

    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    if (OpsReOpen(2,0,0) !=0) {
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
    if (nowait==0) SiteEndScan(scnsc,scnus,5000);
  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
  printf("\ntestscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("  -stid char: radar string (required)\n");
  printf("    -di     : indicates running during discretionary time\n");
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
  printf("     -c int : channel number for multi-channel radars.\n");
  printf("   -ros char: change the roshost IP address.\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

