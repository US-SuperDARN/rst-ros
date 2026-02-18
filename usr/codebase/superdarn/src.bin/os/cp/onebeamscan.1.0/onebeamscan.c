/* onebeamscan.c
   ============
   Author: R.J.Barnes
   with mods from SGS

 Basic Parameters:
   Scan from start_beam to end_beam in sequence.

 Details:
   - 2-min scan is historical, setting the -fast option is standard for 1-min

 v1.7 - SGS
   - added fixed frequency capability (basically over-ride whatever the
      clear frequency search returns.) CAREFUL you don't transmit on a
      forbidden frequency...

*/

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
#include "rosmsg.h"
#include "tsg.h"

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"onebeamscan 2026/02/18"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: onebeamscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024];

  int nowait=0;

  int scnsc=120;      /* total scan period in seconds */
  int scnus=0;
  int cnt=0;

  unsigned char fast=0;
  unsigned char discretion=0;
  int cpid=0;
  int setintt=0;  /* flag to override auto-calc of integration time */

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int startup=1;
  int fixfrq=0;
  int clrscan=0;

  int n;
  int status=0;

  int obm = 7;
  int nbm = 16;    /* default number of "beams" */
  int total_scan_usecs = 0;
  int total_integration_usecs = 0;

  int bufsc=3;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what??? Sounding?  */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  /* Flag and variables for beam synchronizing */
  int bm_sync = 0;
  int bmsc    = 6;
  int bmus    = 0;

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp     = 1240;        /* CPID */
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
  OptionAdd(&opt, "wide",   'x', &wide_tx);
  OptionAdd(&opt, "rfrate", 'i', &rfrate);
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
  OptionAdd(&opt, "nowait", 'x', &nowait);
  OptionAdd(&opt, "clrscan",'x', &clrscan);
  OptionAdd(&opt, "clrskip",'i', &clrskip);
  OptionAdd(&opt, "ob",     'i', &obm); /* one beam: i.e., THE beam */
  OptionAdd(&opt, "nb",     'i', &nbm); /* number of beams per "scan"; default is 16 */
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);   /* fix the transmit frequency */
  OptionAdd(&opt, "cpid",   'i', &cpid);     /* allow user to specify CPID */
  OptionAdd(&opt, "bm_sync",'x', &bm_sync);  /* flag to enable beam sync    */
  OptionAdd(&opt, "bmsc",   'i', &bmsc);     /* beam sync period, sec       */
  OptionAdd(&opt, "bmus",   'i', &bmus);     /* beam sync period, microsec  */
  OptionAdd(&opt, "intsc",  'i', &intsc);
  OptionAdd(&opt, "intus",  'i', &intus);
  OptionAdd(&opt, "setintt",'x', &setintt);
  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);  /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);      /* just dump some parameters */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x',&version);

  OptionAdd(&opt, "baud",   'i', &nbaud);
  OptionAdd(&opt, "tau",    'i', &mpinc);

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

  if (hlp) {
    usage();

    printf("  one beam: %2d\n", obm);

    return (-1);
  }

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  OpsStart(ststr);

  status = SiteBuild(libstr);
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
  arg = OptionProcess(1, argc, argv, &opt, NULL);

  if (fast) sprintf(progname,"onebeamscan (fast)");
  else sprintf(progname,"onebeamscan");

  printf("Station ID: %s  %d\n", ststr, stid);
  strncpy(combf, progid, 80);

  if ((errlog.sock = TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }
  if ((shell.sock = TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0; n<tnum; n++) task[n].port+=baseport;

  OpsSetupCommand(argc, argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l"
                           " frqrng l xcnt l",
                           &sbm,&ebm, &dfrq,&nfrq,
                           &frqrng,&xcnt);

  if (fast) {
    scnsc = 60;
    scnus = 0;
  } else {
    scnsc = 120;
    scnus = 0;
  }
  if (cpid) cp = cpid;  /* user is setting the CPID;
                           discretionary flips sign below */

  nBeams_per_scan = nbm;

  if (bm_sync) {
    sync_scan = 1;
    scan_times = malloc(nBeams_per_scan*sizeof(int));
  }

  for (iBeam=0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = obm;
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

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  status = SiteSetupRadar();
  if (status != 0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  printf("Initial Setup Complete: Station ID: %s  %d\n", ststr, stid);

  /* Initialize timing variables */
  elapsed_secs=0;
  gettimeofday(&t1,NULL);
  gettimeofday(&t0,NULL);

  if (discretion) cp = -cp;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0; n<tnum; n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
  tsgid = SiteTimeSeq(seq->ptab);

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  /* Synchronize start of first scan to minute boundary */
  if (nowait==0) {
    ErrLog(errlog.sock,progname,"Synchronizing to scan boundary.");
    SiteEndScan(scnsc,scnus,100000);
  }

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
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
        RMsgSndOpen(task[n].sock,strlen( (char *)command),command);
      }
    }

    scan = 1; /* scan flag */

    if (clrscan) startup=1;

    /* mechanism for collecting XCFs only on every xcnt scans? */
    if (xcnt > 0) {
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

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",
                      bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
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

      nave = SiteIntegrate(seq->lags);
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

  return 0;
}

void usage(void)
{
  printf("\nonebeamscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("  -stid char: radar string (required)\n");
  printf("    -di     : indicates running during discretionary time\n");
  printf("   -wide    : use a wide transmission beam\n");
  printf("  -fast     : 1-min scan (2-min default)\n");
  printf("-rfrate int : set the USRP RF sampling rate (MHz) [5]\n");
  printf(" -frang int : delay to first range (km) [180]\n");
  printf("  -rsep int : range separation (km) [45]\n");
  printf(" -nrang int : number of range gates\n");
  printf("    -dt int : hour when day freq. is used\n");
  printf("    -nt int : hour when night freq. is used\n");
  printf("    -df int : daytime frequency (kHz)\n");
  printf("    -nf int : nighttime frequency (kHz)\n");
  printf("   -xcf     : set for computing XCFs\n");
  printf("    -ep int : error log port\n");
  printf("    -sp int : shell port\n");
  printf("    -bp int : base port\n");
  printf("  -cpid int : set to override control program id\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("-clrscan    : Force clear frequency search at start of scan\n");
  printf("-clrskip int: Minimum number of seconds to skip between clear frequency search\n");
  printf("-nowait     : do not wait at end of scan boundary.\n");
  printf("    -ob int : THE one beam [7]\n");
  printf("    -nb int : number of beams per scan [16]\n");
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

