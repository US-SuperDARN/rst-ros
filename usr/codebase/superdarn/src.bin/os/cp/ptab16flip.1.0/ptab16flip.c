/* ptab16flip.c
   ============
   Author: J.Spaleta
   Modified: E.G.Thomas

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

char progid[80]={"ptab16flip 2023/09/18"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: ptab16flip --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

  char logtxt[1024];

  int nowait=0;

  int scnsc=60;    /* total scan period in seconds */
  int scnus=0;
  int cnt=0;

  unsigned char discretion=0;

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int startup=1;
  int fixfrq=0;
  int clrscan=0;

  int n;
  int status=0;

  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int bufsc=3;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */

  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  int flipflop=0;

  struct sequence *seq_short;
  struct sequence *seq_long;

  seq_short=OpsSequenceMake();
  OpsBuild8pulse(seq_short);

  seq_long=OpsSequenceMake();
  OpsBuild16pulse(seq_long);

  cp     = 9916;    /* CPID */
  intsc  = 3;       /* integration period; recomputed below ... */
  intus  = 0;
  txpl   = 100;     /* pulse length [us]; gets redefined below... */
  nbaud  = 1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "di",     'x', &discretion);
  OptionAdd(&opt, "dt",     'i', &day);
  OptionAdd(&opt, "nt",     'i', &night);
  OptionAdd(&opt, "df",     'i', &dfrq);
  OptionAdd(&opt, "nf",     'i', &nfrq);
  OptionAdd(&opt, "xcf",    'x', &xcnt);
  OptionAdd(&opt, "ep",     'i', &errlog.port);
  OptionAdd(&opt, "sp",     'i', &shell.port);
  OptionAdd(&opt, "bp",     'i', &baseport);
  OptionAdd(&opt, "stid",   't', &ststr);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "nowait", 'x', &nowait);
  OptionAdd(&opt, "clrscan",'x', &clrscan);
  OptionAdd(&opt, "clrskip",'i', &clrskip);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt, "frqrng", 'i', &frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);    /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);        /* just dump some parameters */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x', &version);

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

  OpsStart(ststr);

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

  /* non-standard nrang and rsep for this mode */
  nrang  = 300;     /* the number of ranges gets set in SiteXXXStart() */
  rsep   = 15;      /* same for the range separation */

  /* reprocess the commandline since some things are reset by SiteStart */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  sprintf(progname,"ptab16flip");

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

  for (iBeam =0; iBeam < nBeams_per_scan; iBeam++) {
    scan_beam_number_list[iBeam] = current_beam;
    current_beam += backward ? -1:1;
  }

  /* Automatically calculate the integration times */
  /* Note that I have added a buffer here to account for things at the end
     of the scan. Traditionally this has been set to 3s, but I cannot find
     any justification of the need for it. -SGS */
  total_scan_usecs = scnsc*1e6 + scnus - (bufsc*1e6 + bufus);
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  intsc = total_integration_usecs/1e6;
  intus = total_integration_usecs - (intsc*1e6);

  pcode=(int *)malloc((size_t)sizeof(int)*seq_long->mppul*nbaud);
  OpsBuildPcode(nbaud,seq_long->mppul,pcode);

  OpsSetupIQBuf(intsc,intus,seq_long->mppul,seq_long->mpinc,nbaud);

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

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
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
    printf("Entering Site Start Scan Station ID: %s  %d\n",ststr,stid);
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
      if (fixfrq > 0) {
        stfrq=fixfrq;
        tfreq=fixfrq;
        noise=0;
      }

      if (flipflop==0) {
        mpinc=seq_short->mpinc;
        mplgs=seq_short->mplgs;
        mppul=seq_short->mppul;
        tsgid=SiteTimeSeq(seq_short->ptab);
      } else {
        mpinc=seq_long->mpinc;
        mplgs=seq_long->mplgs;
        mppul=seq_long->mppul;
        tsgid=SiteTimeSeq(seq_long->ptab);
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus mppul:%d (%02d:%02d:%02d:%06d)",
                     bmnum,intsc,intus,mppul,hr,mt,sc,us);
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

      if (flipflop==0) {
        nave=SiteIntegrate(seq_short->lags);
      } else {
        nave=SiteIntegrate(seq_long->lags);
      }
      if (nave < 0) {
        sprintf(logtxt,"Integration error: %d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      if (flipflop==0) {
        OpsBuildPrm(prm,seq_short->ptab,seq_short->lags);
      } else {
        OpsBuildPrm(prm,seq_long->ptab,seq_long->lags);
      }
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

    if (flipflop==0) {
      flipflop++;
    } else {
      flipflop=0;
    }

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    if (nowait==0) SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\nptab16flip [command-line options]\n\n");
    printf("command-line options:\n");
    printf("  -stid char: radar string (required)\n");
    printf("    -di     : indicates running during discretionary time\n");
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
    printf("-nowait     : do not wait at end of scan boundary.\n");
    printf("-clrscan    : Force clear frequency search at start of scan\n");
    printf("-clrskip int: Minimum number of seconds to skip between clear frequency search\n");
    printf("-frqrng int : set the clear frequency search window (kHz)\n");
    printf("     -c int : channel number for multi-channel radars.\n");
    printf("   -ros char: change the roshost IP address\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

