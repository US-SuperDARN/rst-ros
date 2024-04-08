/* everyotherbeam.c
 ==================
 Author: S.G.Shepherd (but obviously stolen from some other control program)

 A mode which alternates between a camping beam and beams that skip every
  other beam. The intent is to have a camping (high time resolution) beam
  but also maintain 1-min scans and have adequate spatial coverage. Mike
  argues that the lower frequency (should see ground scatter from mid-latitudes
  during day) widens the beam and makes skipping every other beam not as
  detrimental as it would be at higher frequencies.

 To run during eclipse of 2017.

 The mode here is specific to 20 beams for cve (0-20) and cvw (23-3).

 Eliminating the 'fast' option since we want this to be a 1-minute scan.

 MUST use -stid xxx with this control program or seg faults

 Updates:
   20170718 - first version

 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
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

char *ststr = NULL;
char *dfststr = "tst";
char *libstr = "ros";
void *tmpbuf;
size_t tmpsze;
char progid[80] = {"everyotherbeam"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum = 4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: everyotherbeam --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024] = "";
  char tempLog[40];

  int scannowait = 0;
  /* these are set for a standard 1-min scan, any changes here will affect
     the number of beams sampled, etc.
   */
  int scnsc = 60;
  int scnus =  0;

  int skip;
/*  int skipsc= 3;    serves same purpose as globals intsc and intus */
/*  int skipus= 0;*/
  int cnt   =  0;
  int i,n;
  unsigned char discretion = 0;
  int status = 0;
  int fixfrq = 0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                     */
  int intgt[20];      /* start times of each integration period             */
  int nintgs=20;      /* number of integration periods per scan; SGS 1-min  */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  /*
    beam sequences for 24-beam MSI radars but only using 20 most meridional
      beams; also hard-coding the camping beam for each radar for this mode.
      cve: 0 to 20 with camp on 10
      cvw: 23 to 3 with camp on 11

    note the absence of sampling the camping beam 3-consecutive scans
   */
  /* count         1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  int bmse[20] = { 0,10, 2,10, 4,10, 6,10, 8,10,12,10,14,10,16,10,18,10,20,10};
  int bmsw[20] = {23,11,21,11,19,11,17,11,15,11,13,11, 9,11, 7,11, 5,11, 3,11};

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  /* standard radar defaults */
  cp     = 1231;        /* new CPID */
  intsc  = 3;           /* integration period; not sure how critical this is */
  intus  = 0;           /*  but can be changed here */
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  rsep   = 45;
  txpl   = 300;         /* note: recomputed below */

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
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);   /* fix the transmit frequency */
  OptionAdd(&opt,"ros",   't',&roshost);  /* Set the roshost IP address */
  OptionAdd(&opt,"c",     'i',&cnum);
  OptionAdd(&opt,"debug", 'x',&debug);
  OptionAdd(&opt,"-help", 'x',&hlp);      /* just dump some parameters */
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

  /* Process all of the command line options
      Important: need to do this here because we need stid and ststr */
  arg = OptionProcess(1,argc,argv,&opt,rst_opterr);

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
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"fhe") == 0)) {
    bms = bmse;   /* 1-min sequence */
  } else if (strcmp(ststr,"cvw") == 0) {
    bms = bmsw;   /* 1-min sequence */
  } else if (strcmp(ststr,"fhw") == 0) {
    bms = bmsw;   /* 1-min sequence */
    for (i=0; i<nintgs/2; i++) {
      bms[i*2] -= 2;
      if (bms[i*2] == 11) bms[i*2] -= 2;
    }
  } else {
    if (hlp) usage();
    else     printf("Error: Not intended for station %s\n", ststr);
    return (-1);
  }

  if (hlp) {
    usage();

    printf("\n");
    printf("sqnc  stme  bmno\n");
    for (i=0; i<nintgs; i++) {
      printf(" %2d   %3d    %2d", i, intgt[i], bms[i]);
      printf("\n");
    }

    return (-1);
  }

  /* end of main Dartmouth mods */
  /* not sure if -nrang commandline option works */

  if (ststr == NULL) ststr = dfststr;

  channel = cnum;

  OpsStart(ststr);

  if ((status = SiteBuild(libstr)) == -1) {
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
  arg = OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  strncpy(combf,progid,80);

  if ((errlog.sock = TCPIPMsgOpen(errlog.host,errlog.port)) == -1)
    fprintf(stderr,"Error connecting to error log.\n");

  if ((shell.sock = TCPIPMsgOpen(shell.host,shell.port)) == -1)
    fprintf(stderr,"Error connecting to shell.\n");

  for (n=0; n<tnum; n++) task[n].port += baseport;

  /* dump beams to log file */
  sprintf(progname,"everyotherbeam");
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

  status = SiteSetupRadar();

  fprintf(stderr,"Status: %d\n",status);
  
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  if (discretion) cp = -cp;

  txpl = (rsep*20)/3;   /* computing TX pulse length */

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0; n<tnum; n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  OpsFitACFStart();

  tsgid = SiteTimeSeq(seq->ptab);  /* get the timing sequence */

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

  do {

    if (SiteStartScan() !=0) continue;

    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    if (OpsReOpen(2,0,0) != 0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen((char *)command),command);
      }
    }

    scan = 1;

    ErrLog(errlog.sock,progname,"Starting scan.");

    if (xcnt > 0) {
      cnt++;
      if (cnt == xcnt) {
        xcf = 1;
        cnt = 0;
      } else xcf = 0;
    } else xcf = 0;

    skip = OpsFindSkip(scnsc,scnus,intsc,intus,nintgs);

    bmnum = bms[skip];    /* no longer need forward and backward arrays... */

    do {

      /* Synchronize to the desired start time */
      
      /* This will only work, if the total time through the do loop is < 3s */
      /* If this is not the case, decrease the Integration time */
      /* MAX < or <=  3s ? */
      /* once again, don't like this... */

      {
        int t_now;
        int t_dly;
        TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);
        t_now = ( (mt*60 + sc)*1000 + us/1000 ) % (scnsc*1000 + scnus/1000);
        t_dly = intgt[skip]*1000 - t_now;
        if (t_dly > 0) usleep(t_dly);
      }

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight() == 1) {
        stfrq = dfrq;
      } else {
        stfrq = nfrq;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",
              bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq = SiteFCLR(stfrq,stfrq+frqrng);

      if (fixfrq > 0) tfreq = fixfrq;

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

      msg.num = 0;
      msg.tsize = 0;

      tmpbuf = RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf = IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
            (unsigned char *)badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *)sharedmemory,
            IQS_TYPE,0);

      tmpbuf = RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf = FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      for (n=0; n<tnum; n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0; n<msg.num; n++) {
        if (msg.data[n].type == PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type == IQ_TYPE)  free(msg.ptr[n]);
        if (msg.data[n].type == RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type == FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan = 0;
      if (skip == (nintgs-1)) break;
      skip++;
      bmnum = bms[skip];

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    if (scannowait == 0) SiteEndScan(scnsc,scnus,5000);
  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return (0);
}

void usage(void)
{
    printf("\neveryotherbeam [command-line options]\n\n");
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
    printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
    printf("     -c int : channel number for multi-channel radars.\n");
    printf("   -ros char: change the roshost IP address\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

