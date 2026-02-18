/* themisscan.c
 ============
 Author: J.Spaleta
 With Modifications by M. McClorey and SGS

 Basic Parameters:
   Scan from start_beam to end_beam in sequence with a camping beam sampled
   on alternate integrations, i.e., every other beam.

 Details:
   - integration period is fixed to 3s to coincide with THEMIS ground-based
     observations. Can be changed, but currently is an integer number of sec.

   - do NOT repeat camping beam multiple times.
     E.g.,  ... 3 6 4 6 5 6 7 6 8 6 9 ...

   - can change to 1-min scan but then limited to 11 beam extent for scan.

   - any extra beams at end of scan are set to camping beam.
     E.g. for -sb 0 -eb 3:  0 6 1 6 2 6 3 6 6 6 6 ...

   - can add buffer at end of scan (see bufsc and bufus) which reduces number
     of beams sampled (with no buffer you have 40 beams in a 2-min scan)

   - MUST use -stid xxx with this control program or seg faults

 v1.6 - SGS IMPORTANT change: start beam is the actual starting beam of the
            radar. For a westward scanning radar the start beam is greater
            than the end beam reflecting the fact that these radars typically
            scan in a counter-clockwise direction.
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
char progid[80]={"themisscan 2026/02/18"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: themisscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024]="";
  char tempLog[40];

  int nowait=0;
  int scnsc=120;
  int scnus=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int cnt=0;
  int i,n;
  unsigned char discretion=0;
  int status=0;
  int cbm=7;        /* default camping beam */

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int startup=1;
  int fixfrq=0;
  int clrscan=0;

  /* new variables for dynamically creating beam sequences */
  int *intgt;
  int *bms;       /* scanning beams */
  int nintgs;     /* number of integration periods per scan */
  int tbm;
  int odd;
  int bufsc=0;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  /* standard radar defaults */
  cp     = 3300;        /* themisscan cpid */
  intsc  = 3;           /* integration period; fundamental to THEMIS */
  intus  = 0;           /*  but can be changed here */
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  rsep   = 45;
  txpl   = 300;         /* note: recomputed below */
  nbaud  = 1;

/* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt,"di",    'x',&discretion);
  OptionAdd(&opt,"wide",  'x',&wide_tx);
  OptionAdd(&opt,"rfrate",'i',&rfrate);
  OptionAdd(&opt,"frang", 'i',&frang);
  OptionAdd(&opt,"rsep",  'i',&rsep);
  OptionAdd(&opt,"dt",    'i',&day);
  OptionAdd(&opt,"nt",    'i',&night);
  OptionAdd(&opt,"df",    'i',&dfrq);
  OptionAdd(&opt,"nf",    'i',&nfrq);
  OptionAdd(&opt,"xcf",   'x',&xcnt);
  OptionAdd(&opt,"nrang", 'i',&nrang);
  OptionAdd(&opt,"sb",    'i',&sbm);
  OptionAdd(&opt,"eb",    'i',&ebm);
  OptionAdd(&opt,"ep",    'i',&errlog.port);
  OptionAdd(&opt,"sp",    'i',&shell.port);
  OptionAdd(&opt,"bp",    'i',&baseport);
  OptionAdd(&opt,"stid",  't',&ststr);
  OptionAdd(&opt,"nowait",'x',&nowait);
  OptionAdd(&opt,"clrscan",'x',&clrscan);
  OptionAdd(&opt,"clrskip",'i',&clrskip);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt,"camp",  'i',&cbm);        /* camping beam number */
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

  /* number of integration periods possible in scan time */
  /* modified to allow no buffer at end of scan. change bufsc to 3s for
     standard 3s delay at end of scan */
  nintgs = (int)floor( (scnsc+scnus*1e-6 - (bufsc+bufus*1e-6))/
                        (intsc + intus*1e-6) );

  /* arrays for integration start times and beam sequences */
  intgt = (int *)malloc(nintgs*sizeof(int));
  bms   = (int *)malloc(nintgs*sizeof(int));

  /* start time of each integration period */
  for (i=0; i<nintgs; i++)
    intgt[i] = i*(intsc + intus*1e-6);

  /* Create a list of the beams that will be sequenced through, alternating
   * between the camping beam and each other beam in turn */

  odd = (sbm == cbm);       /* odd/even camp beams */
  backward = (sbm > ebm) ? 1 : 0;   /* allow for non-standard scan dir. */

  if (backward) {
    tbm = sbm;
    if (sbm == cbm) tbm--;

    for (i=0; i<nintgs; i++){
      if ((i%2 == odd) && (tbm >= ebm)) {
        /* exclude camp bm from regular beams */
        if ((tbm == cbm) && (sbm != cbm)) tbm--;
        if (tbm >= ebm) bms[i] = tbm;
        else bms[i] = cbm;
        tbm--;
      } else {
        bms[i] = cbm;
      }
    }
  } else {  /* forward */
    tbm = sbm;
    if (sbm == cbm) tbm++;

    for (i=0; i<nintgs; i++){
      if ((i%2 == odd) && (tbm <= ebm)) {
        /* exclude camp bm from regular beams */
        if ((tbm == cbm) && (sbm != cbm)) tbm++;
        if (tbm <= ebm) bms[i] = tbm;
        else bms[i] = cbm;
        tbm++;
      } else {
        bms[i] = cbm;
      }
    }
  }

  if (hlp) {
    usage();

    printf("  start beam: %2d\n", sbm);
    printf("  end   beam: %2d\n", ebm);
    printf("  camp  beam: %2d\n", cbm);
    printf("\n");
    printf("sqnc  stme  bmno\n");
    for (i=0; i<nintgs; i++) {
      printf(" %2d   %3d    %2d", i, intgt[i], bms[i]);
      if (bms[i] == cbm) printf("   CAMP");
      printf("\n");
    }

    free(intgt);
    free(bms);

    return (-1);
  }

  nBeams_per_scan = nintgs;

  sync_scan = 1;
  scan_times = malloc(nBeams_per_scan*sizeof(int));
  for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = bms[iBeam];
    scan_times[iBeam] = iBeam * (intsc*1000 + intus/1000); /* in ms */
  }

  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-3)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  intsc = total_integration_usecs/1E6;
  intus = total_integration_usecs - (intsc*1E6);

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
  sprintf(progname,"themisscan");
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

  tsgid=SiteTimeSeq(seq->ptab);      /* get the timing sequence */

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  /* Synchronize start of first scan to minute boundary */
  if (nowait==0) {
    ErrLog(errlog.sock,progname,"Synchronizing to scan boundary.");
    SiteEndScan(scnsc,scnus,100000);
  }

  do {

    ErrLog(errlog.sock,progname,"Starting scan.");

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
    if (nowait==0) SiteEndScan(scnsc,scnus,100000);
  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  free(bms);
  free(intgt);

  return 0;
}


void usage(void)
{
  printf("\nthemisscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("  -stid char: radar string (required)\n");
  printf("    -di     : indicates running during discretionary time\n");
  printf("  -wide     : use a wide transmission beam\n");
  printf("-rfrate int : set the USRP RF sampling rate (MHz) [5]\n");
  printf(" -frang int : delay to first range (km) [180]\n");
  printf("  -rsep int : range separation (km) [45]\n");
  printf("    -dt int : hour when day freq. is used\n");
  printf("    -nt int : hour when night freq. is used\n");
  printf("    -df int : daytime frequency (kHz)\n");
  printf("    -nf int : nighttime frequency (kHz)\n");
  printf("   -xcf     : set for computing XCFs\n");
  printf(" -nrang int : number of range gates\n");
  printf("    -sb int : starting beam\n");
  printf("    -eb int : ending beam\n");
  printf("  -camp int : camping beam [7]\n");
  printf("    -ep int : error log port\n");
  printf("    -sp int : shell port\n");
  printf("    -bp int : base port\n");
  printf("-nowait     : Do not wait for minute scan boundary\n");
  printf("-clrscan    : Force clear frequency search at start of scan\n");
  printf("-clrskip int: Minimum number of seconds to skip between clear frequency search\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("     -c int : channel number for multi-channel radars.\n");
  printf("   -ros char: change the roshost IP address\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

