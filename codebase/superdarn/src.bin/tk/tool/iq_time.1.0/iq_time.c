/* iq_time.c
   =========
   Author: E.G.Thomas
*/

/*
  Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
This file is part of the Radar Software Toolkit (RST).

RST is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

Modifications:
*/ 

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <zlib.h>
#include "rtypes.h"
#include "dmap.h"
#include "rtime.h"
#include "option.h"
#include "rprm.h"
#include "iq.h"
#include "version.h"

#include "iqread.h"
#include "iqwrite.h"
#include "iqindex.h"
#include "iqseek.h"

#include "errstr.h"
#include "hlpstr.h"

struct RadarParm *prm=NULL;
struct IQ *iq=NULL;
unsigned int *badtr=NULL;
int16 *samples=NULL;

struct OptionData opt;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: iq_fix --help\n");
  return(-1);
}
 
int main (int argc,char *argv[]) {

  int arg=0;

  unsigned char vb=0;
  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  FILE *fp=NULL;

  char str[128];

  int n=0;
  int yr,mo,dy,hr,mt;
  double tval,sec;

  prm=RadarParmMake();
  iq=IQMake();

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);
  OptionAdd(&opt,"vb",'x',&vb);

  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (help==1) {
    OptionPrintInfo(stdout,hlpstr);
    exit(0);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }

  if (version==1) {
    OptionVersion(stdout);
    exit(0);
  }

  if (arg==argc) fp=stdin;
  else fp=fopen(argv[arg],"r");

  if (fp==NULL) {
    fprintf(stderr,"File not found.\n");
    exit(-1);
  }


  while (IQFread(fp,prm,iq,&badtr,&samples) !=-1) {

    if (vb==1) {
      fprintf(stderr,"%04d-%02d-%02d  %02d:%02d:%02d  beam=%02d  nave=%d\n",prm->time.yr,prm->time.mo,
              prm->time.dy,prm->time.hr,prm->time.mt,prm->time.sc,prm->bmnum,prm->nave);
    }

    sprintf(str,"%04d-%02d-%02d  %02d:%02d:%02d.%06d",
            prm->time.yr,prm->time.mo,prm->time.dy,
            prm->time.hr,prm->time.mt,prm->time.sc,
            prm->time.us);

    /* Identify wide TX beam data with negative beam number */
    if (prm->bmazm == 0) {
      fprintf(stdout,"%s  %d -%02d  %05d  %02d",str,prm->scan,prm->bmnum,prm->tfreq,iq->seqnum);
    } else {
      fprintf(stdout,"%s  %d  %02d  %05d  %02d",str,prm->scan,prm->bmnum,prm->tfreq,iq->seqnum);
    }

    for (n=0; n < iq->seqnum; n++) {
      tval = iq->tval[n].tv_sec + (1.0*iq->tval[n].tv_nsec)/1.0e9;
      TimeEpochToYMDHMS(tval,&yr,&mo,&dy,&hr,&mt,&sec);
      fprintf(stdout,"  %02d:%02d:%09.6f",hr,mt,sec);
    }
    fprintf(stdout,"\n");
  }

  if (fp !=stdin) fclose(fp);

  return 0;

}
