/* campsound.c
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
#include "rosmsg.h"
#include "tsg.h"

#define RT_TASK 3


char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"campsound 2025/04/24"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: campsound --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

  char logtxt[1024];

  int scnsc=60;    /* total scan period in seconds */
  int scnus=0;
  int cnt=0;

  unsigned char discretion=0;
  int fixfrq=0;

  int n;
  int status=0;

  int total_scan_usecs=0;
  int total_integration_usecs=0;

  unsigned char iq_flg=0;
  unsigned char raw_flg=0;

  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  unsigned char ext_flg=0;

  int *freqs;
  int *bms;

  /* ---------------- Variables for sounding --------------- */
  int cve_freqs[] = {9500,10500,11500,12500,13500,14500,15500,16400,17400};
  int cvw_freqs[] = {9700,10700,11700,12700,13700,14700,15700,16700,17600};
  int ice_freqs[] = {9500,10500,11500,12400,13500,14500,15500,16470,17400};
  int icw_freqs[] = {9600,10600,11600,12600,13600,14600,15600,16570,17500};
  int nfreqs=9;

  int cve_bms_4[]={ 0, 1, 2, 3};
  int cvw_bms_4[]={13,12,11,10};
  int cve_bms_12[]={ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11};
  int cvw_bms_12[]={23,22,21,20,15,14,13,12,11,10, 9, 8};

  int ice_bms_4[]={10,11,12,13};
  int icw_bms_4[]={20,19,18,17};
  int ice_bms_12[]={ 6, 7, 8, 9,10,11,12,13,14,15,16,17};
  int icw_bms_12[]={23,22,21,20,19,18,17,16,15,14,13,12};

  int snd_freq_cnt=0, snd_bm_cnt=0;
  int nbeams=4;
  int snd_frqrng=100;
  /* ------------------------------------------------------- */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp     = 1105;        /* CPID */
  intsc  = 1;           /* integration period; recomputed below ... */
  intus  = 600000;
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
  OptionAdd(&opt, "xcf",    'x', &xcnt);
  OptionAdd(&opt, "ep",     'i', &errlog.port);
  OptionAdd(&opt, "sp",     'i', &shell.port);
  OptionAdd(&opt, "bp",     'i', &baseport);
  OptionAdd(&opt, "stid",   't', &ststr);
  OptionAdd(&opt, "12beam", 'x', &ext_flg);    /* use 12 beams instead of 4 */
  OptionAdd(&opt, "frqrng", 'i', &snd_frqrng); /* fix the FCLR window [kHz] */
  OptionAdd(&opt, "iqdat",  'x', &iq_flg);     /* store IQ samples */
  OptionAdd(&opt, "rawacf", 'x', &raw_flg);    /* store rawacf data */
  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);    /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);        /* just dump some parameters */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x',&version);

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
    return (-1);
  }

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  if (ext_flg) {
    cp = 1106;
    nbeams = 12;
    scnsc = 180;
  }

  /* Point to the beams here */
  if (strcmp(ststr,"cve") == 0) {
    freqs = cve_freqs;
    if (ext_flg) bms = cve_bms_12;
    else         bms = cve_bms_4;
  } else if (strcmp(ststr,"cvw") == 0) {
    freqs = cvw_freqs;
    if (ext_flg) bms = cvw_bms_12;
    else         bms = cvw_bms_4;
  } else if (strcmp(ststr,"ice") == 0) {
    freqs = ice_freqs;
    if (ext_flg) bms = ice_bms_12;
    else         bms = ice_bms_4;
  } else if (strcmp(ststr,"icw") == 0) {
    freqs = icw_freqs;
    if (ext_flg) bms = icw_bms_12;
    else         bms = icw_bms_4;
  } else {
    printf("Error: Not intended for station %s\n", ststr);
    return (-1);
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

  sprintf(progname,"campsound");

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

  sync_scan = 0;
  nBeams_per_scan = nbeams*nfreqs;

  for (snd_bm_cnt = 0; snd_bm_cnt < nbeams; snd_bm_cnt++) {
    for (snd_freq_cnt = 0; snd_freq_cnt < nfreqs; snd_freq_cnt++) {
      scan_beam_number_list[iBeam] = bms[snd_bm_cnt];
      scan_clrfreq_fstart_list[iBeam] = (int32_t) (freqs[snd_freq_cnt]);
      scan_clrfreq_bandwidth_list[iBeam] = snd_frqrng;
      iBeam++;
    }
  }

  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-3)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  intsc = total_integration_usecs/1E6;
  intus = total_integration_usecs - (intsc*1e6);

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

  printf("Preparing OpsSndStart Station ID: %s %d\n",ststr,stid);
  OpsSndStart();

  printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
  tsgid=SiteTimeSeq(seq->ptab);

  /* Synchronize start of first scan to minute boundary */
  SiteEndScan(scnsc,scnus,5000);

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {

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

    do {

      /* set the beam */
      bmnum = scan_beam_number_list[iBeam];

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      stfrq = scan_clrfreq_fstart_list[iBeam];

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",
                     bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);

      /* clear frequency search business */
      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, snd_frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq=SiteFCLR(stfrq,stfrq+snd_frqrng);

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

      OpsBuildSnd(snd,prm,fit);

      /* save the sounding mode data */
      OpsWriteSnd(errlog.sock, progname, snd, ststr);

      if (iq_flg) {
        OpsBuildIQ(iq,&badtr);
        OpsWriteSndIQ(errlog.sock, progname, prm, iq, badtr, ststr);
      }

      if (raw_flg) OpsWriteSndRaw(errlog.sock, progname, prm, raw, ststr);

      RadarShell(shell.sock,&rstable);

      scan = 0;

      iBeam++;
      if (iBeam >= nBeams_per_scan) break;

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\ncampsound [command-line options]\n\n");
    printf("command-line options:\n");
    printf("  -stid char: radar string (required)\n");
    printf("    -di     : indicates running during discretionary time\n");
    printf(" -frang int : delay to first range (km) [180]\n");
    printf("  -rsep int : range separation (km) [45]\n");
    printf(" -nrang int : number of range gates\n");
    printf("   -xcf     : set for computing XCFs\n");
    printf("    -ep int : error log port\n");
    printf("    -sp int : shell port\n");
    printf("    -bp int : base port\n");
    printf("-12beam     : use 12 beams instead of 4\n");
    printf("-frqrng int : set the clear frequency search window (kHz)\n");
    printf(" -iqdat     : set for storing snd IQ samples\n");
    printf("-rawacf     : set for writing snd rawacf files\n");
    printf("     -c int : channel number for multi-channel radars.\n");
    printf("   -ros char: change the roshost IP address\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

