/* snd.c
   =====
   Author: E.G.Thomas
*/


#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <zlib.h>
#include "rtypes.h"
#include "limit.h"
#include "rtime.h"
#include "dmap.h"
#include "shmem.h"
#include "rprm.h"
#include "freq.h"
#include "iq.h"
#include "iqwrite.h"
#include "rawdata.h"
#include "rawwrite.h"
#include "fitdata.h"
#include "snddata.h"
#include "fitsnd.h"
#include "sndwrite.h"
#include "sndread.h"
#include "errlog.h"
#include "global.h"
#include "snd.h"


int snd_freqs_tot;
int snd_freqs[MAX_SND_FREQS];

struct SndData *snd;


void OpsLoadSndFreqs(char *ststr) {
  char snd_filename[100];
  FILE *snd_dat;
  char *path;
  int stat;

  int tmp_freqs[MAX_SND_FREQS]={9500, 10500, 11500, 12500, 13500, 14500, 15500, 16500, 0, 0, 0, 0 };

  int snd_freq_cnt=0;
  int cnt=0;

  /* load the sounder frequencies from file in site directory if present */
  path = getenv("SD_SITE_PATH");
  if (path == NULL) {
    fprintf(stderr,"Environment variable 'SD_SITE_PATH' not defined.\n");
  }

  sprintf(snd_filename,"%s/site.%s/sounder_%dmhz.dat.%s", path, ststr, rfrate, ststr);
  fprintf(stderr,"Checking Sounder File: %s\n",snd_filename);
  snd_dat = fopen(snd_filename, "r");
  if (snd_dat != NULL) {
    stat = fscanf(snd_dat, "%d", &snd_freqs_tot);
    if (stat != 1) {
      fprintf(stderr,"Error reading number of sounder frequencies\n");
      snd_freqs_tot = 8;
      fclose(snd_dat);
    } else {
      if (snd_freqs_tot > MAX_SND_FREQS) snd_freqs_tot = MAX_SND_FREQS;
      for (snd_freq_cnt=0; snd_freq_cnt < snd_freqs_tot; snd_freq_cnt++)
        stat = fscanf(snd_dat, "%d", &tmp_freqs[snd_freq_cnt]);
      fclose(snd_dat);
      fprintf(stderr,"Sounder File: %s read\n",snd_filename);
    }
  } else {
    fprintf(stderr,"Sounder File: %s not found\n",snd_filename);
    snd_freqs_tot = 8;
  }

  /* Check whether sounder frequencies are valid */
  for (snd_freq_cnt=0; snd_freq_cnt < snd_freqs_tot; snd_freq_cnt++) {
    if (FreqTest(ftable,tmp_freqs[snd_freq_cnt]) == 0) {
      snd_freqs[cnt] = tmp_freqs[snd_freq_cnt];
      cnt++;
    } else {
      fprintf(stderr,"Bad sounder frequency: %d\n",tmp_freqs[snd_freq_cnt]);
    }
  }
  snd_freqs_tot = cnt;
}


int OpsSndStart() {
  snd=SndMake();
  fprintf(stderr,"Leaving OpsSndStart\n");
  fflush(stderr);
  return 0;
}


void OpsFindSndSkip(char *ststr,int *snd_bms,int snd_bms_tot,int *snd_bm_cnt,int *odd_beams) {

  char data_path[60], data_filename[40], filename[105];

  char *snd_dir;
  FILE *fp=NULL;

  struct SndData *tmp;
  int sbc = 0;
  int odd = 0;

  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL) {
    return;
  } else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", yr, mo, dy, (hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  fp = fopen(filename,"r");
  if (fp == NULL) {
    return;
  }

  tmp=SndMake();

  /* read the snd records */
  while (SndFread(fp,tmp) !=-1);

  /* find the most recently used snd beam
   * and skip to the next */
  for (sbc=0; sbc<snd_bms_tot; sbc++) {
    if      (tmp->bmnum == snd_bms[sbc]) break;
    else if (tmp->bmnum == snd_bms[sbc]+1) {
      odd = 1;
      break;
    }
  }
  sbc++;

  /* check for the end of a beam loop */
  if (sbc >= snd_bms_tot) {
    sbc = 0;
    odd = !odd;
  }

  *snd_bm_cnt = sbc;
  *odd_beams = odd;

  SndFree(tmp);
}


void OpsBuildSnd(struct SndData *snd, struct RadarParm *prm, struct FitData *fit) {
  SndSetOriginTime(snd,prm->origin.time);
  SndSetOriginCommand(snd,(char *)command);
  SndSetCombf(snd,combf);
  FitToSnd(snd,prm,fit,prm->scan);
}


void OpsWriteSnd(int sock, char *progname, struct SndData *snd, char *ststr) {

  char data_path[60], data_filename[40], filename[105];

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
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy,
                                                 (prm->time.hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding file: %s",filename);
    ErrLog(sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = SndFwrite(out, snd);
  if (status == -1) {
    ErrLog(sock,progname,"Error writing sounding record.");
  } else {
    ErrLog(sock,progname,"Sounding record successfully written.");
  }

  fclose(out);
}


void OpsWriteSndRaw(int sock, char *progname, struct RadarParm *prm,
                    struct RawData *raw, char *ststr) {

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
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy,
                                                 (prm->time.hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd.rawacf", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Rawacf Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding rawacf file: %s",filename);
    ErrLog(sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = RawFwrite(out, prm, raw);
  if (status == -1) {
    ErrLog(sock,progname,"Error writing sounding rawacf record.");
  } else {
    ErrLog(sock,progname,"Sounding rawacf record successfully written.");
  }

  fclose(out);
}


void OpsWriteSndIQ(int sock, char *progname, struct RadarParm *prm,
                   struct IQ *iq, unsigned int *badtr, char *ststr) {

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
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy,
                                                 (prm->time.hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd.iqdat", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Iqdat Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding iqdat file: %s",filename);
    ErrLog(sock,progname,logtxt);
    return;
  }

  iqbufsize = ShMemSizeName(sharedmemory);
  p = ShMemAlloc(sharedmemory,iqbufsize,O_RDWR,0,&fd);

  /* write the sounding record */
  status = IQFwrite(out, prm, iq, badtr, (int16 *)p);
  if (status == -1) {
    ErrLog(sock,progname,"Error writing sounding iqdat record.");
  } else {
    ErrLog(sock,progname,"Sounding iqdat record successfully written.");
  }
  ShMemFree(p,sharedmemory,iqbufsize,0,fd);

  fclose(out);
}
