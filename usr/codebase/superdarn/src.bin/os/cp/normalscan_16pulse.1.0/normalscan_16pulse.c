/* normalscan_16pulse.c
   ====================
   Author: M.Balaji & J.Spaleta
   Modified: E.G.Thomas

*/

/*
 license stuff should go here...
*/

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

char progid[80]={"normalscan_16pulse 2022/09/01"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: normalscan_16pulse --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

  int *bcode;
  int bcode1[1]={1};
  int bcode2[2]={1,-1};
  int bcode3[3]={1,1,-1};
  int bcode4[4]={1,1,-1,1};
  int bcode5[5]={1,1,1,-1,1};
  int bcode7[7]={1,1,1,-1,-1,1,-1};
  int bcode11[11]={1,1,1,-1,-1,-1,1,-1,-1,1,-1};
  int bcode13[13]={1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

  int ptab[16] = {0,4,19,42,78,127,191,270,364,474,600,745,905,1083,1280,1495};
  int lags[LAG_SIZE][2] = {
    {1495,1495},   /*  0 */
    {0,4},         /*  1 */
    {4,19},        /*  2 */
    {0,19},        /*  3 */
    {19,42},       /*  4 */
    {42,78},       /*  5 */
    {4,42},        /*  6 */
    {0,42},        /*  7 */
    {78,127},      /*  8 */
    {19,78},       /*  9 */
    {127,191},     /*  10 */
    {4,78},        /*  11 */
    {0,78},        /*  12 */
    {191,270},     /*  13 */
    {42,127},      /*  14 */
    {270,364},     /*  15 */
    {19,127},      /*  16 */
    {364,474},     /*  17 */
    {78,191},      /*  18 */
    {4,127},       /*  19 */
    {474,600},     /*  20 */
    {0,127},       /*  21 */
    {127,270},     /*  22 */
    {600,745},     /*  23 */
    {42,191},      /*  24 */
    {745,905},     /*  25 */
    {191,364},     /*  26 */
    {19,191},      /*  27 */
    {905,1083},    /*  28 */
    {4,191},       /*  29 */
    {0,191},       /*  30 */
    {78,270},      /*  31 */
    {1083,1280},   /*  32 */
    {270,474},     /*  33 */
    {1280,1495},   /*  34 */
    {42,270},      /*  35 */
    {127,364},     /*  36 */
    {364,600},     /*  37 */
    {19,270},      /*  38 */
    {4,270},       /*  39 */
    {0,270},       /*  40 */
    {474,745},     /*  41 */
    {191,474},     /*  42 */
    {78,364},      /*  43 */
    {600,905},     /*  44 */
    {42,364},      /*  45 */
    {270,600},     /*  46 */
    {745,1083},    /*  47 */
    {19,364},      /*  48 */
    {127,474},     /*  49 */
    {4,364},       /*  50 */
    {0,364},       /*  51 */
    {905,1280},    /*  52 */
    {364,745},     /*  53 */
    {78,474},      /*  54 */
    {191,600},     /*  55 */
    {1083,1495},   /*  56 */
    {474,905},     /*  57 */
    {42,474},      /*  58 */
    {19,474},      /*  59 */
    {4,474},       /*  60 */
    {127,600},     /*  61 */
    {0,474},       /*  62 */
    {270,745},     /*  63 */
    {600,1083},    /*  64 */
    {78,600},      /*  65 */
    {745,1280},    /*  66 */
    {364,905},     /*  67 */
    {191,745},     /*  68 */
    {42,600},      /*  69 */
    {19,600},      /*  70 */
    {905,1495},    /*  71 */
    {4,600},       /*  72 */
    {0,600},       /*  73 */
    {474,1083},    /*  74 */
    {127,745},     /*  75 */
    {270,905},     /*  76 */
    {78,745},      /*  77 */
    {600,1280},    /*  78 */
    {42,745},      /*  79 */
    {191,905},     /*  80 */
    {364,1083},    /*  81 */
    {19,745},      /*  82 */
    {4,745},       /*  83 */
    {0,745},       /*  84 */
    {745,1495},    /*  85 */
    {127,905},     /*  86 */
    {474,1280},    /*  87 */
    {270,1083},    /*  88 */
    {78,905},      /*  89 */
    {42,905},      /*  90 */
    {19,905},      /*  91 */
    {191,1083},    /*  92 */
    {600,1495},    /*  93 */
    {4,905},       /*  94 */
    {0,905},       /*  95 */
    {364,1280},    /*  96 */
    {127,1083},    /*  97 */
    {78,1083},     /*  98 */
    {270,1280},    /*  99 */
    {474,1495},    /*  100 */
    {42,1083},     /*  101 */
    {19,1083},     /*  102 */
    {4,1083},      /*  103 */
    {0,1083},      /*  104 */
    {191,1280},    /*  105 */
    {364,1495},    /*  106 */
    {127,1280},    /*  107 */
    {78,1280},     /*  108 */
    {270,1495},    /*  109 */
    {42,1280},     /*  110 */
    {19,1280},     /*  111 */
    {4,1280},      /*  112 */
    {0,1280},      /*  113 */
    {191,1495},    /*  114 */
    {127,1495},    /*  115 */
    {78,1495},     /*  116 */
    {42,1495},     /*  117 */
    {19,1495},     /*  118 */
    {4,1495},      /*  119 */
    {0,1495},      /*  120 */
    {1495,1495}};  /*  121 */

  char logtxt[1024];

  int scnsc=120;    /* total scan period in seconds */
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char fast=0;
  unsigned char discretion=0;
  int fixfrq=0;

  int n,i;
  int status=0;

  int beams=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int bufsc=3;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */
  unsigned char hlp=0;
  unsigned char option=0;

  if (debug) {
    printf("Size of int %lu\n",sizeof(int));
    printf("Size of long %lu\n",sizeof(long));
    printf("Size of long long %lu\n",sizeof(long long));
    printf("Size of struct TRTimes %lu\n",sizeof(struct TRTimes));
    printf("Size of struct SeqPRM %lu\n",sizeof(struct SeqPRM));
    printf("Size of struct RosData %lu\n",sizeof(struct RosData));
    printf("Size of struct DataPRM %lu\n",sizeof(struct DataPRM));
    printf("Size of Struct ControlPRM  %lu\n",sizeof(struct ControlPRM));
    printf("Size of Struct RadarPRM  %lu\n",sizeof(struct RadarPRM));
    printf("Size of Struct ROSMsg  %lu\n",sizeof(struct ROSMsg));
    printf("Size of Struct CLRFreq  %lu\n",sizeof(struct CLRFreqPRM));
    printf("Size of Struct TSGprm  %lu\n",sizeof(struct TSGprm));
    printf("Size of Struct SiteSettings  %lu\n",sizeof(struct SiteSettings));
  }


  cp     = 9100;    /* CPID */
  intsc  = 6;       /* integration period; recomputed below ... */
  intus  = 0;
  mppul  = 16;      /* number of pulses; tied to array above ... */
  mplgs  = 121;     /* same here for the number of lags */
  mpinc  = 100;     /* multi-pulse increment [us] */
  nrang  = 300;     /* the number of ranges gets set in SiteXXXStart() */
  rsep   = 15;      /* same for the range separation */
  txpl   = 100;     /* pulse length [us]; gets redefined below... */
  nbaud  = 1;

  dmpinc = nmpinc = mpinc;  /* set day and night to the same,
                               but could change */

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
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "baud",   'i', &nbaud);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt, "frqrng", 'i', &frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt, "ros",    't', &roshost);    /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);        /* just dump some parameters */
  OptionAdd(&opt, "-option",'x', &option);

  /* process the commandline; need this for setting errlog port */
  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
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

  switch (nbaud) {
    case  1: bcode = bcode1;  break;
    case  2: bcode = bcode2;  break;
    case  3: bcode = bcode3;  break;
    case  4: bcode = bcode4;  break;
    case  5: bcode = bcode5;  break;
    case  7: bcode = bcode7;  break;
    case 11: bcode = bcode11; break;
    case 13: bcode = bcode13; break;
    default: bcode = bcode1;
  }

  pcode = (int *)malloc((size_t)sizeof(int)*mppul*nbaud);
  for (i=0; i<mppul; i++) {
    for (n=0; n<nbaud; n++) {
      pcode[i*nbaud+n] = bcode[n];
    }
  }

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

  /* reprocess the commandline since some things are reset by SiteStart */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

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

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l"
                  " dmpinc l nmpinc l frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &dfrang,&nfrang, &dmpinc,&nmpinc, &frqrng,&xcnt);

  status=SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  printf("Initial Setup Complete: Station ID: %s  %d\n",ststr,stid);

  beams=abs(ebm-sbm)+1;
  if (fast) {
    cp    = 9101;
    scnsc = 60;
    scnus = 0;
  } else {
    scnsc = 120;
    scnus = 0;
  }

  /* Automatically calculate the integration times */
  /* Note that I have added a buffer here to account for things at the end
     of the scan. Traditionally this has been set to 3s, but I cannot find
     any justification of the need for it. -SGS */
  total_scan_usecs = scnsc*1e6 + scnus - (bufsc*1e6 + bufus);
  total_integration_usecs = total_scan_usecs/beams;
  intsc = total_integration_usecs/1e6;
  intus = total_integration_usecs - (intsc*1e6);

  if (discretion) cp = -cp;

  txpl = (nbaud*rsep*20)/3;

  if (fast) sprintf(progname,"normalscan_16pulse (fast)");
  else sprintf(progname,"normalscan_16pulse");

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
  tsgid=SiteTimeSeq(ptab);

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {

    printf("Entering Site Start Scan Station ID: %s  %d\n",ststr,stid);
    if (SiteStartScan() !=0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan = 1;   /* scan flag */

    ErrLog(errlog.sock,progname,"Starting scan.");
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    skip=OpsFindSkip(scnsc,scnus,intsc,intus,0);

    if (backward) {
      bmnum=sbm-skip;
      if (bmnum<ebm) bmnum=sbm;
    } else {
      bmnum=sbm+skip;
      if (bmnum>ebm) bmnum=sbm;
    }

    do {

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
        frang=nfrang;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",
                     bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);

      /* clear frequency search business */
      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq = fixfrq;

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      nave=SiteIntegrate(lags);
      if (nave < 0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,ptab,lags);
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

      RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *)progname,
                 NME_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0; n<msg.num; n++) {
        if ( (msg.data[n].type == PRM_TYPE) ||
             (msg.data[n].type == IQ_TYPE)  ||
             (msg.data[n].type == RAW_TYPE) ||
             (msg.data[n].type == FIT_TYPE) )  free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan = 0;
      if (bmnum == ebm) break;

      if (backward) bmnum--;
      else bmnum++;

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  if (pcode != NULL) free(pcode);

  return 0;
}


void usage(void)
{
    printf("\nnormalscan_16pulse [command-line options]\n\n");
    printf("command-line options:\n");
    printf("    -di     : indicates running during discretionary time\n");
    printf(" -frang int : delay to first range (km) [180]\n");
    printf("  -rsep int : range separation (km) [15]\n");
    printf(" -nrang int : number of range gates [225]\n");
    printf("    -dt int : hour when day freq. is used [site.c]\n");
    printf("    -nt int : hour when night freq. is used [site.c]\n");
    printf("    -df int : daytime frequency (kHz) [site.c]\n");
    printf("    -nf int : nighttime frequency (kHz) [site.c]\n");
    printf("   -xcf     : set for computing XCFs [global.c]\n");
    printf("    -sb int : starting beam [site.c]\n");
    printf("    -eb int : ending beam [site.c]\n");
    printf("    -ep int : error log port (must be set here for dual radars)\n");
    printf("    -sp int : shell port (must be set here for dual radars)\n");
    printf("    -bp int : base port (must be set here for dual radars)\n");
    printf(" -stid char : radar string (must be set here for dual radars)\n");
    printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
    printf("-frqrng int : set the clear frequency search window (kHz)\n");
    printf("  -ros char : change the roshost IP address\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

