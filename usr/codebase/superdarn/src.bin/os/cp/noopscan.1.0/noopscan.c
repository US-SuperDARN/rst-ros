/* noopscan.c
   ==========
   Author: R.J.Barnes & J.Spaleta

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

char progid[80]={"noopscan 2025/05/27"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: noopscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  int nowait=0;
 
  int scnsc=0;      /* total scan period in seconds */
  int scnus=0;

  unsigned char fast=0;
  unsigned char discretion=0;
  int cpid=0;
  int rxonly=0;
  int setintt=0;  /* flag to override auto-calc of integration time */

  /* Variables for controlling clear frequency search */
  int clrskip=-1;
  int fixfrq=0;
  int clrscan=0;

  int n;
  int status=0;
  int i=0;

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

  cp     = 150;         /* CPID */
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
  OptionAdd(&opt, "nowait", 'x', &nowait);
  OptionAdd(&opt, "clrscan",'x', &clrscan);
  OptionAdd(&opt, "clrskip",'i', &clrskip);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);   /* fix the transmit frequency  */
  OptionAdd(&opt, "cpid",   'i', &cpid);     /* allow user to specify CPID, *
                                                e.g., RX-only               */
  OptionAdd(&opt, "rxonly", 'x', &rxonly);   /* RX-only mode                */
  OptionAdd(&opt, "bm_sync",'x', &bm_sync);  /* flag to enable beam sync    */
  OptionAdd(&opt, "bmsc",   'i', &bmsc);     /* beam sync period, sec       */
  OptionAdd(&opt, "bmus",   'i', &bmus);     /* beam sync period, microsec  */
  OptionAdd(&opt, "intsc",  'i', &intsc);
  OptionAdd(&opt, "intus",  'i', &intus);
  OptionAdd(&opt, "setintt",'x', &setintt);
  OptionAdd(&opt, "scnsc",  'i', &scnsc);     /* set the scan time */
  OptionAdd(&opt, "scnus",  'i', &scnus);

  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);  /* Set the roshost IP address  */
  OptionAdd(&opt, "debug",  'x', &debug);

  OptionAdd(&opt, "-help",  'x', &hlp);      /* just dump some parameters   */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x',&version);

  /* process the commandline; need this for setting errlog port */
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
  arg = OptionProcess(1,argc,argv,&opt,NULL);

  if (fast) sprintf(progname,"noopscan (fast)");
  else sprintf(progname,"noopscan");

  printf("Station ID: %s  %d\n",ststr,stid);
  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1)
    fprintf(stderr,"Error connecting to error log.\n");

  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1)
    fprintf(stderr,"Error connecting to shell.\n");

  for (n=0; n<tnum; n++) task[n].port += baseport;

  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l"
                  " frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &frqrng,&xcnt);

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0; n<tnum; n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  ErrLog(errlog.sock,progname,"Entering NoOp loop.");
  do {

    sleep(1);
    i++;
    if (i > 100) i=0;

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}

void usage(void)
{
  printf("\nnoopscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("  -stid char: radar string (required)\n");
  printf("    -di     : indicates running during discretionary time\n");
  printf("  -fast     : 1-min scan (2-min default)\n");
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
  printf("  -cpid int : set to override control program id\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("-nowait     : do not wait at end of scan boundary.\n");
  printf("-clrscan    : Force clear frequency search at start of scan\n");
  printf("-clrskip int: Minimum number of seconds to skip between clear frequency search\n");
  printf("-rxonly     : bistatic RX only mode.\n");
  printf("-bm_sync    : set to enable beam syncing.\n");
  printf("  -bmsc int : beam syncing interval seconds.\n");
  printf("  -bmus int : beam syncing interval microseconds.\n");
  printf("-setintt    : set to enable integration period override.\n");
  printf(" -intsc int : integration period seconds.\n");
  printf(" -intus int : integration period microseconds.\n");
  printf("     -c int : channel number for multi-channel radars.\n");
  printf("   -ros char: change the roshost IP address\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

