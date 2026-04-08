/* iq_fix.c
   ========
   Author: E.G.Thomas

   Reprocesses an iqdat file by removing the first sequence of the first beam
     in a scan. For some MSI radars this sequence was shifted in time relative
     to the others, introducing noise in the first beam of a scan.
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

20250115 SGS implemented a simple algorithm that compares the power (squared)
             of the first sample of a sequence to the mean power (squared) of
             the entire sequence. If the power in the first sample is less than
             the mean, the sequence is considered to be shifted and is excluded
             from the record.
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

struct OptionData opt;    /* why does this have to be global?! */

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: iq_fix --help\n");
  return(-1);
}
 
int main (int argc,char *argv[]) {

  struct RadarParm *prm;
  struct IQ *iq, *oiq;
  unsigned int *badtr=NULL;
  unsigned int *obadtr=NULL;
  int16 *samples, *osamples;

  unsigned char vb, help, option, version;
  int arg, chnnum;

  FILE *fp = NULL;

  time_t ctime;
  int k, n, m, baselen;
  char command[128];
  char tmstr[40], modstr[40];

  int c, off, off1, off2, fac;
  int *badseq;
  int nseq, nbadtr;
  double pwr0, mnpwr;

  vb = help = option = version = 0;
  arg = chnnum = 0;
  badseq = NULL;
  samples = osamples = NULL;

  /* create some structures */
  prm = RadarParmMake();
  iq  = IQMake();
  oiq = IQMake();

  /* command line option processing */
  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);
  OptionAdd(&opt,"vb",'x',&vb);
  OptionAdd(&opt,"chnnum",'i',&chnnum);

  arg = OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg == -1) exit(-1);
  if (help == 1) {
    OptionPrintInfo(stdout,hlpstr);
    exit(0);
  }
  if (option == 1) {
    OptionDump(stdout,&opt);
    exit(0);
  }
  if (version == 1) {
    OptionVersion(stdout);
    exit(0);
  }

  if (arg == argc) fp = stdin;
  else             fp = fopen(argv[arg],"r");

  if (fp == NULL) {
    fprintf(stderr,"File not found.\n");
    exit(-1);
  }

  n = command[0] = 0;
  for (c=0; c<argc; c++) {
    n += strlen(argv[c])+1;
    if (n > 127) break;
    if (c != 0) strcat(command," ");
    strcat(command, argv[c]);
  }
  baselen = strlen(command);

  /* read through the iqdat file and process each record */
  while (IQFread(fp, prm, iq, &badtr, &samples) != -1) {

    /* set -vb for logfile output */
    if (vb) fprintf(stderr,"%04d%02d%02d %02d%02d%02d %d %02d %02d",
                    prm->time.yr, prm->time.mo, prm->time.dy,
                    prm->time.hr, prm->time.mt, prm->time.sc,
                    prm->cp, prm->bmnum, iq->seqnum);

    prm->origin.code = 1;
    ctime = time((time_t)0);
    strcpy(tmstr, asctime(gmtime(&ctime)));
    tmstr[24] = 0;
    RadarParmSetOriginTime(prm, tmstr);

    /* logic array to indicate which sequence(s) are shifted (bad) */
    badseq = (int *)realloc(badseq, iq->seqnum*sizeof(int));

    nseq = nbadtr = 0;
    /* look at each sequence and determine whether it is shifted (bad) or not */
    for (n=0; n<iq->seqnum; n++) {

      off1 = n*4*iq->smpnum;
      /* compute mean power in main samples for each sequence (not using
         interferometer samples, but maybe could?) */
      mnpwr = 0;
      for (m=0; m<iq->smpnum; m++)
        mnpwr += samples[off1+2*m]*samples[off1+2*m] +
                 samples[off1+1+2*m]*samples[off1+1+2*m];
      // no sqrt, do we need it?
      pwr0 = samples[off1]*samples[off1] + samples[off1+1]*samples[off1+1];
      pwr0 *= pwr0; // square of power

      /* comparing the mean power over the entire sequence to the power at
         the start of the sequence, which should be a pulse */
      if (pwr0 < mnpwr) {
        //printf("%02d BAD\n", n);    /* write to an output file for stats? */
        badseq[n] = 1;
        if (vb) fprintf(stderr," %02d", n);
      } else {
        badseq[n] = 0;
        nbadtr += iq->badtr[n];   /* should just be 8*nseq */
        nseq++;
      }
    }
    if (vb) fprintf(stderr,"\n");

    /* add original and modified number of sequences to origin string */
    sprintf(modstr, " %d %d", iq->seqnum, nseq);
    strcpy(command+baselen, modstr);
    RadarParmSetOriginCommand(prm, command);

//-----------------------------------------------------------------------------
/* debugging
        printf("iq->seqnum = %d\n", iq->seqnum);
        printf("iq->smpnum = %d\n", iq->smpnum);
        printf("iq->skpnum = %d\n", iq->skpnum);
        printf("iq->tbadtr = %d\n", iq->tbadtr);
      printf("offset   size     badtr\n");
      for (n=0; n<iq->seqnum; n++) {
        printf("%5d %4d %1d", iq->offset[n], iq->size[n], iq->badtr[n]);
        for (m=0; m<2*iq->badtr[n]; m++)
          printf(" %u", badtr[n*2*iq->badtr[n]+m]);
        printf("\n");
      }

      return (-1);

    printf("\n");
    for (k=0; k<iq->seqnum; k++) printf("  %2d %d\n", k, badseq[k]);
    printf("\n");
*/
//-----------------------------------------------------------------------------

    if (nseq == iq->seqnum) {   /* no bad sequences detected */

      if (chnnum > 0) iq->chnnum = chnnum;

      // FIX: fix the badtr array (*5) for these beams as well ...
      if (badtr[0] < 50) {  /* early IQ data files assumed that
                               samples were pulse width so values
                               are incorrect */

        off1 = 0;
        fac = 5;
        for (n=0; n<iq->seqnum; n++) {
          for (k=0; k<2*iq->badtr[n]; k++) badtr[off1+k] *= fac;
          off1 += 2*iq->badtr[n];
        }
      }

      IQFwrite(stdout,prm,iq,badtr,samples);

    } else {                    /* there are bad sequences */
      /* create a new structure with only the good sequences and write it */
      prm->nave = nseq;
      oiq->revision.major = iq->revision.major;
      oiq->revision.minor = iq->revision.minor;
      if (chnnum > 0) oiq->chnnum = chnnum;
      else            oiq->chnnum = iq->chnnum;
      oiq->smpnum = iq->smpnum;
      oiq->skpnum = iq->skpnum;
      oiq->seqnum = nseq;
      oiq->tbadtr = nbadtr;   /* total number of pulses */

      // FIX: Not sure how this works with channels...

      //printf(" nseq = %d\n", nseq);

      if (nseq > 0) {
        oiq->tval   = (struct timespec *)realloc(oiq->tval,
                                                  nseq*sizeof(struct timespec));
        oiq->atten  = (int *)realloc(oiq->atten,   nseq*sizeof(int));
        oiq->offset = (int *)realloc(oiq->offset,  nseq*sizeof(int));
        oiq->size   = (int *)realloc(oiq->size,    nseq*sizeof(int));
        oiq->badtr  = (int *)realloc(oiq->badtr,   nseq*sizeof(int));
        oiq->noise  = (float *)realloc(oiq->noise, nseq*sizeof(float));

        osamples = (int16 *)realloc(osamples, nseq*4*iq->smpnum*sizeof(int16));
        obadtr   = (unsigned int *)realloc(obadtr,
                                                nbadtr*2*sizeof(unsigned int));
/*
printf(" nbadtr = %d\n", nbadtr);
printf(" %d elements\n", nseq*4);
printf(" %d elements\n", nbadtr*2);
*/

        /* badtr array has the start times in us and the lengths */

        off = off1 = off2 = 0;
        fac = (badtr[0] < 50) ? 5 : 1;  /* early IQ data files assumed that
                                           samples were pulse width so values
                                           are incorrect */
        for (m=n=0; n<iq->seqnum; n++) {
          if (badseq[n] == 0) {
            /* I,Q s for main array, I,Q s for interferometer array */
            for (k=0; k<4*iq->smpnum; k++)
              osamples[m*4*iq->smpnum+k] = samples[n*4*iq->smpnum+k];

            oiq->tval[m]   = iq->tval[m];
            oiq->atten[m]  = iq->atten[n];
            oiq->noise[m]  = iq->noise[n];
            oiq->offset[m] = off; //m*4*iq->smpnum;  // Check this...
            oiq->badtr[m]  = iq->badtr[n];
            oiq->size[m]   = 4*iq->smpnum;
            off += oiq->size[m];
            for (k=0; k<2*iq->badtr[n]; k++) obadtr[off1+k] = fac*badtr[off2+k];

            off1 += 2*iq->badtr[n]; /* index into obadtr */
            m++;  /* good sequence counter */
          }
          off2 += 2*iq->badtr[n];   /* index into badtr */
        }
        IQFwrite(stdout,prm,oiq,obadtr,osamples);
      } else {
        /* FIX: what to do if the only sequences are all bad? */
        /* do these all have to be free'd and set to NULL? */
        //oiq->tval   = 
        //oiq->atten  = (int *)realloc(oiq->atten,   nseq*sizeof(int));
        //oiq->offset = (int *)realloc(oiq->offset,  nseq*sizeof(int));
        //oiq->size   = (int *)realloc(oiq->size,    nseq*sizeof(int));
        //oiq->badtr  = (int *)realloc(oiq->badtr,   nseq*sizeof(int));
        //oiq->noise  = (float *)realloc(oiq->noise, nseq*sizeof(float));
        IQFwrite(stdout,prm,oiq,NULL,NULL);
        //fprintf(stderr, "Should NOT be here!!\n");
      }
    } /* end bad sequence(s) detected */
  }   /* end reading through file */

  if (fp != stdin) fclose(fp);

  IQFree(iq);
  IQFree(oiq);

  /* arrays in structure get freed but what about samples?! */
  if (samples != NULL) free(samples);
  if (osamples != NULL) free(osamples);
  if (badtr != NULL) free(badtr);
  if (obadtr != NULL) free(obadtr);

  free(badseq);

  return (0);
}

