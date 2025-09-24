/* sync.c
   ======
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

#include <math.h>
#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>
#include "rtime.h"
#include "rtypes.h"
#include "dmap.h"
#include "limit.h"
#include "rprm.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "radar.h"
#include "global.h"

#define USEC 1000000.0

int OpsDayNight() {
  if (day < night) {
    if ((day <= hr) && (hr < night)) return 1;
    else return 0;
  } else {
    if ((night <= hr) && (hr < day)) return 0;
  }
  return 1;
}


/*int OpsFindSkip(int bsc,int bus)*/
int OpsFindSkip(int scnsc, int scnus, int synsc, int synus, int nbm)
{
  unsigned tv,bv,iv;
  int skip;

  /* option to pass in number of beams for camping mode */
  if (nbm <= 0) nbm = abs(ebm-sbm) + 1;

  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  /* all times converted to microseconds */
  iv   = synsc*1000000 + synus;  /* sync time for one beam */
  bv   = scnsc*1000000 + scnus;  /* time for a full scan */
  tv   = (mt*60 + sc)*1000000 + us + iv/2 - 100000; /* time from start of hour
                                                       add half a beam & some
                                                       rando buffer for FCLR? */
  skip = (tv % bv)/iv;
  if (skip > nbm-1 || skip < 0) skip = 0;

  return skip;
}

