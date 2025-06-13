/* radar.c
   =======
   Author: R.J.Barnes
*/

/*
 LICENSE AND DISCLAIMER
 
 Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
 This file is part of the RST Radar Operating System (RST-ROS).
 
 RST-ROS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>. 
 
*/


#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "rtime.h"
#include "option.h"
#include "tsg.h"
#include "maketsg.h"

#include "tcpipmsg.h"
#include "rosmsg.h"
#include "freq.h"

#include "errstr.h"
#include "hlpstr.h"

#define DEF_PORT 45000

struct OptionData opt;
unsigned char vb;
time_t t;


int operate(pid_t parent,int sock) {

  struct ROSMsg smsg,rmsg;
  struct ControlPRM rprm;
  struct CLRFreqPRM fprm;
  struct SeqPRM tprm;
  struct TSGprm tsgprm;
  struct TSGbuf *tsgbuf=NULL;
  struct RosData rdata;
  struct DataPRM dprm;
  struct TRTimes badtrdat;

  FILE *fp;
  char *envstr=NULL;
  char freq_filepath[100];
  struct FreqTable *ftable=NULL;

  int stid,tfreq,dfreq,rnum,cnum,s,i;
  float noise=0.5;

  int txpow=20;
  int atten=0;

  int32 temp_int32,data_length;
  char ststr[10];
  char entry_name[80];
  char entry_type,return_type;

  uint32 uI32,uQ32;

  int num_transmitters=16;

  int AGC[16],LOPWR[16];

  int usecs;
  int yr,mo,dy,hr,mt,sc,us;
  int intsc,intus,nrang,mpinc,smsep,lagfr,mppul,nbaud;
  int *pcode=NULL;

  uint32_t nave,tmp;
  int32_t periods_per_scan,fixFreq,sync_scan;
  int32_t scn_sc,scn_us,int_sc,int_us,start_period;
  int32_t sbeam_list[100];
  int32_t clrfrq_fstart[100];
  int32_t clrfrq_bw[100];
  int32_t beam_times[100];

  memset(&rprm,0,sizeof(struct ControlPRM));
  memset(&fprm,0,sizeof(struct CLRFreqPRM));
  memset(&tprm,0,sizeof(struct SeqPRM));
  memset(&tsgprm,0,sizeof(struct TSGprm));
  memset(&dprm,0,sizeof(struct DataPRM));
  memset(&rdata,0,sizeof(struct RosData));
  memset(&badtrdat,0,sizeof(struct TRTimes));

  while(1) {
    s=TCPIPMsgRecv(sock,&smsg,sizeof(struct ROSMsg));
    if (s !=sizeof(struct ROSMsg)) break;

    rmsg.type=smsg.type;
    rmsg.status=1;

    switch (smsg.type) {
      case SET_RADAR_CHAN:
        if (vb) fprintf(stderr,"SET_RADAR_CHAN\n");
        TCPIPMsgRecv(sock, &stid, sizeof(int));
        TCPIPMsgRecv(sock, &data_length, sizeof(int32));
        TCPIPMsgRecv(sock, &ststr, data_length*sizeof(char));
        TCPIPMsgRecv(sock, &rnum, sizeof(int));
        TCPIPMsgRecv(sock, &cnum, sizeof(int));

        envstr=getenv("SD_SITE_PATH");
        if ((ststr !=NULL) && (envstr !=NULL)) {
          sprintf(freq_filepath,"%s/site.%s/restrict.dat.%s",envstr,ststr,ststr);
          fp=fopen(freq_filepath,"r");
          if (fp !=NULL) {
            ftable=FreqLoadTable(fp);
            fclose(fp);
          }
        }
        break;

      case QUERY_INI_SETTINGS:
        if (vb) fprintf(stderr,"QUERY_INI_SETTINGS\n");
        TCPIPMsgSend(sock, &txpow, sizeof(int));
        TCPIPMsgSend(sock, &atten, sizeof(int));
        break;

      case GET_PARAMETERS:
        if (vb) fprintf(stderr,"GET_PARAMETERS\n");
        TCPIPMsgSend(sock, &rprm, sizeof(struct ControlPRM));
        break;

      case SET_ACTIVE:
        if (vb) fprintf(stderr,"SET_ACTIVE\n");
        TCPIPMsgRecv(sock, &periods_per_scan, sizeof(int32_t)); /* number of periods */

        TCPIPMsgRecv(sock, &fixFreq, sizeof(int32_t)); /* fixed frequency or -1 for clear frequency search */
        TCPIPMsgRecv(sock, &clrfrq_fstart, periods_per_scan*sizeof(int32_t)); /* start frequency of clrfreq */
        TCPIPMsgRecv(sock, &clrfrq_bw, periods_per_scan*sizeof(int32_t)); /* bandwidth of clrfreq in Hz */
        TCPIPMsgRecv(sock, &sbeam_list, periods_per_scan*sizeof(int32_t)); /* scan beam list */

        TCPIPMsgRecv(sock, &sync_scan, sizeof(int32_t)); /* if the periods/beams should start at fixed times */
        TCPIPMsgRecv(sock, &scn_sc, sizeof(int32_t)); /* scan and integration times */
        TCPIPMsgRecv(sock, &scn_us, sizeof(int32_t));
        TCPIPMsgRecv(sock, &int_sc, sizeof(int32_t));
        TCPIPMsgRecv(sock, &int_us, sizeof(int32_t));

        TCPIPMsgRecv(sock, &start_period, sizeof(int32_t));
        if (sync_scan == 1) {
          TCPIPMsgRecv(sock, &beam_times, periods_per_scan*sizeof(int32_t));
        }

        rmsg.status=0;
        break;

      case SET_PARAMETERS:
        if (vb) fprintf(stderr,"SET_PARAMETERS\n");
        TCPIPMsgRecv(sock, &rprm, sizeof(struct ControlPRM));
        break;

      case REQUEST_CLEAR_FREQ_SEARCH:
        if (vb) fprintf(stderr,"REQUEST_CLEAR_FREQ_SEARCH\n");
        TCPIPMsgRecv(sock, &fprm, sizeof(struct CLRFreqPRM));
        dfreq=fprm.end-fprm.start;
        if (dfreq > 0) {
          tfreq=fprm.start + (rand() % dfreq);
          if (ftable !=NULL) {
            s=0;
            while ((FreqTest(ftable,tfreq) == 1) && (s < 10)) {
              tfreq=fprm.start + (rand() % dfreq);
              s++;
            }
          }
        } else {
          tfreq=fprm.start;
        }
        if (ftable !=NULL) {
          if (FreqTest(ftable,tfreq) == 1) {
            tfreq=0;
            noise=1e10;
          } else {
            noise=0.5;
          }
        }
        break;

      case REQUEST_ASSIGNED_FREQ:
        if (vb) fprintf(stderr,"REQUEST_ASSIGNED_FREQ\n");
        TCPIPMsgSend(sock, &tfreq, sizeof(int));
        TCPIPMsgSend(sock, &noise, sizeof(float));
        break;

      case REGISTER_SEQ:
        if (vb) fprintf(stderr,"REGISTER_SEQ\n");
        TCPIPMsgRecv(sock, &tprm, sizeof(struct SeqPRM));
        if (tsgbuf !=NULL) TSGFree(tsgbuf);
        tsgbuf=malloc(sizeof(struct TSGbuf));
        tsgbuf->len=tprm.len;
        tsgbuf->rep=malloc(sizeof(unsigned char)*tsgbuf->len);
        tsgbuf->code=malloc(sizeof(unsigned char)*tsgbuf->len);
        if (vb) fprintf(stderr,"%d\n",tprm.len);
        TCPIPMsgRecv(sock, tsgbuf->rep, sizeof(unsigned char)*tsgbuf->len);
        TCPIPMsgRecv(sock, tsgbuf->code, sizeof(unsigned char)*tsgbuf->len);

        TCPIPMsgRecv(sock, &intsc, sizeof(int));
        TCPIPMsgRecv(sock, &intus, sizeof(int));
        TCPIPMsgRecv(sock, &nrang, sizeof(int));
        TCPIPMsgRecv(sock, &mpinc, sizeof(int));
        TCPIPMsgRecv(sock, &smsep, sizeof(int));
        TCPIPMsgRecv(sock, &lagfr, sizeof(int));
        TCPIPMsgRecv(sock, &mppul, sizeof(int));

        if (tsgprm.pat !=NULL) free(tsgprm.pat);
        tsgprm.pat = malloc(sizeof(int)*mppul);
        for (i=0; i<mppul; i++)
          TCPIPMsgRecv(sock, &tsgprm.pat[i], sizeof(int));

        TCPIPMsgRecv(sock, &nbaud, sizeof(int));
        if (pcode !=NULL) free(pcode);
        pcode = (int *)malloc((size_t)sizeof(int)*mppul*nbaud);
        for (i=0; i<nbaud; i++)
          TCPIPMsgRecv(sock, &pcode[i], sizeof(int));

        rmsg.status=0;
        break;

      case PING:
        if (vb) fprintf(stderr,"PING\n");
        break;

      case SET_READY_FLAG:
        if (vb) fprintf(stderr,"SET_READY_FLAG\n");
        break;

      case GET_DATA:
        if (vb) fprintf(stderr,"GET_DATA\n");

        if (rdata.main !=NULL) free(rdata.main);
        if (rdata.back !=NULL) free(rdata.back);

        dprm.samples=rprm.number_of_samples;
        if (vb) fprintf(stderr,"Number of samples %d\n",dprm.samples);
        dprm.status=0;

        rdata.main=malloc(4*dprm.samples);
        rdata.back=malloc(4*dprm.samples);

        if (badtrdat.start_usec !=NULL) free(badtrdat.start_usec);
        if (badtrdat.duration_usec !=NULL) free(badtrdat.duration_usec);
        badtrdat.length = mppul;
        badtrdat.start_usec = malloc(sizeof(unsigned int)*badtrdat.length);
        badtrdat.duration_usec = malloc(sizeof(unsigned int)*badtrdat.length);

        usecs = (int)dprm.samples/rprm.baseband_samplerate*1e6;
        nave = (int)((int_sc*1e6+int_us)/usecs);

        TCPIPMsgSend(sock, &dprm, sizeof(struct DataPRM));
        TCPIPMsgSend(sock, &nave, sizeof(uint32_t));

        if (dprm.status == 0) {
          for (i=0; i<mppul; i++) {
            badtrdat.start_usec[i] = 230 + tsgprm.pat[i]*smsep*nbaud*5;
            badtrdat.duration_usec[i] = smsep*nbaud;
          }

          TCPIPMsgSend(sock, &badtrdat.length, sizeof(badtrdat.length));
          TCPIPMsgSend(sock, badtrdat.start_usec,
                       sizeof(unsigned int)*badtrdat.length);
          TCPIPMsgSend(sock, badtrdat.duration_usec,
                       sizeof(unsigned int)*badtrdat.length);

          TCPIPMsgSend(sock, &num_transmitters, sizeof(int));
          TCPIPMsgSend(sock, AGC, sizeof(int)*num_transmitters);
          TCPIPMsgSend(sock, LOPWR, sizeof(int)*num_transmitters);

          TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

          TCPIPMsgSend(sock, &yr, sizeof(int));
          TCPIPMsgSend(sock, &mo, sizeof(int));
          TCPIPMsgSend(sock, &dy, sizeof(int));
          TCPIPMsgSend(sock, &hr, sizeof(int));
          TCPIPMsgSend(sock, &mt, sizeof(int));
          TCPIPMsgSend(sock, &sc, sizeof(int));
          TCPIPMsgSend(sock, &us, sizeof(int));
        }

        TCPIPMsgSend(sock, &rprm, sizeof(struct ControlPRM));

        for (i=0; i<dprm.samples; i++) {
          uI32 = ((uint32) i) & 0xFFFF;
          uQ32 = ((uint32) -i) << 16;
          rdata.main[i] = uQ32 | uI32;
          rdata.back[i] = uQ32 | uI32;
        }

        for (i=0; i<nave; i++) {
          usleep((int)(usecs*0.5));

          TCPIPMsgSend(sock, &dprm.event_secs, sizeof(uint32_t));
          TCPIPMsgSend(sock, &dprm.event_usecs, sizeof(uint32_t));

          TCPIPMsgSend(sock, rdata.main, 4*dprm.samples);
          TCPIPMsgSend(sock, rdata.back, 4*dprm.samples);
          TCPIPMsgRecv(sock, &tmp, sizeof(uint32_t));
        }
        break;

      case QUIT:
        if (vb) fprintf(stderr,"QUIT\n");
        break;

      default:
        break;
    }

    if (vb) fprintf(stderr,"-return-\n");

    s=TCPIPMsgSend(sock,&rmsg,sizeof(struct ROSMsg));

    if (s !=sizeof(struct ROSMsg)) break;
  }

  if (vb) fprintf(stderr,"EXIT\n");
  return 0;
}


int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: radar --help\n");
  return(-1);
}


int main(int argc,char *argv[]) {

  int port=DEF_PORT,arg=0;
  int sock;
  int sc_reuseaddr=1,temp;

  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  socklen_t length;
  socklen_t clength;

  struct sockaddr_in server;
  struct sockaddr_in client;

  fd_set ready;

  struct hostent *gethostbyname();
  pid_t root;

  int msgsock=0;

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);
  OptionAdd(&opt,"vb",'x',&vb);
  OptionAdd(&opt,"lp",'i',&port);

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

  signal(SIGCHLD,SIG_IGN);
  signal(SIGPIPE,SIG_IGN);

  root=getpid();

  sock=socket(AF_INET,SOCK_STREAM,0); /* create our listening socket */
  if (sock<0) {
    perror("opening stream socket");
    exit(1);
  }

  /* set socket options */
  temp=setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&sc_reuseaddr,
                  sizeof(sc_reuseaddr));

  if (temp != 0) {
    perror("error setting socket options");
    exit(-1);
  }

  /* name and bind socket to an address and port number */

  server.sin_family=AF_INET;
  server.sin_addr.s_addr=INADDR_ANY;
  if (port !=0) server.sin_port=htons(port);
  else server.sin_port=0;

  if (bind(sock,(struct sockaddr *) &server,sizeof(server))) {
    perror("binding stream socket");
    exit(1);
  }

  /* Find out assigned port number and print it out */

  length=sizeof(server);
  if (getsockname(sock,(struct sockaddr *) &server,&length)) {
    perror("getting socket name");
    exit(1);
  }

  listen(sock,5); /* mark our socket willing to accept connections */

  /* seed the random number generator for the clear frequency search */
  srand((unsigned) time(&t));

  do {

    /* block until someone wants to attach to us */

    FD_ZERO(&ready);
    FD_SET(sock,&ready);
    if (select(sock+1,&ready,0,0,NULL) < 0) {
      perror("while testing for connections");
      continue;
    }

    /* Accept the connection from the client */

    fprintf(stdout,"Accepting a new connection...\n");
    clength=sizeof(client);
    msgsock=accept(sock,(struct sockaddr *) &client,&clength);

    if (msgsock==-1) {
      perror("accept");
      continue;
    }

    if (fork() == 0) {
      close(sock);
      operate(root,msgsock);
      exit(0);
    }

    close(msgsock);
  } while(1);

  return 0;

}
