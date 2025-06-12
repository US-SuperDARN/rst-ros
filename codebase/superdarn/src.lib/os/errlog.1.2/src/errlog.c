/* errlog.c
   ========
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

#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "tcpipmsg.h"
#include "errlog.h"

char str[128];

char *ErrLogStrTime() {
  struct tm *gmt;
  struct timespec err_tm;
  int stat;

  stat = clock_gettime(CLOCK_REALTIME, &err_tm);

  gmt = gmtime(&err_tm.tv_sec);
  sprintf(str,"%02d-%02d-%02d %02d:%02d:%02d.%03d",1900+gmt->tm_year,gmt->tm_mon+1,gmt->tm_mday,gmt->tm_hour,gmt->tm_min,gmt->tm_sec,(int)(err_tm.tv_nsec/1e6));

  return str;
}

//char *ErrLogStrTime() {
//  char *str;
//  time_t clock;
//  struct tm *gmt;
//
//  time(&clock);
//  gmt = gmtime(&clock); 
//  str = asctime(gmt);
//  str[strlen(str)-1] = 0; /* get rid of new line */
//  return str;
//}

int ErrLog(int sock,char *name,char *buffer) {

  int msg,pid,s;
  size_t nlen,blen;

  nlen=strlen(name)+1;
  blen=strlen(buffer)+1;

  msg=ERROR_MSG;

  s=TCPIPMsgSend(sock,&msg,sizeof(int));

  if (s !=sizeof(int)) {
    fprintf(stderr,"WARNING: Error not logged\n");
    fprintf(stderr,"%s : %d : %s : %s\n",ErrLogStrTime(),getpid(),name,buffer);
    return -1;
  }

  pid=getpid();
  s=TCPIPMsgSend(sock,&pid,sizeof(int));

  s=TCPIPMsgSend(sock,&nlen,sizeof(size_t));
  s=TCPIPMsgSend(sock,&blen,sizeof(size_t));

  s=TCPIPMsgSend(sock,name,nlen);
  s=TCPIPMsgSend(sock,buffer,blen);

  s=TCPIPMsgRecv(sock,&msg,sizeof(int));

  fprintf(stderr,"%s : %d : %s : %s\n",ErrLogStrTime(),getpid(),name,buffer);

  if ((s !=sizeof(int))  || (msg !=ERROR_OK)) {
    fprintf(stderr,"WARNING: Error not logged\n");
    return -1;
  }
  return 0;
}

