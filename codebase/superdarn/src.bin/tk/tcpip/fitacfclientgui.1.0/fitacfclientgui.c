/* fitacfclientgui.c
   =================
   Author: E.G.Thomas
 
Copyright (C) 2022  Evan G. Thomas

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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>
#include <ncurses.h>
#include <signal.h>
#include "rtypes.h"
#include "option.h"
#include "dmap.h"
#include "rprm.h"
#include "fitdata.h"
#include "connex.h"
#include "fitcnx.h"

#include "errstr.h"
#include "hlpstr.h"



#define MAX_BEAMS 24
#define MAX_RANGE 300

struct DataBuffer {
  int beam[MAX_BEAMS];
  int qflg[MAX_BEAMS][MAX_RANGE];
  int gsct[MAX_BEAMS][MAX_RANGE];
  float vel[MAX_BEAMS][MAX_RANGE];
  float pow[MAX_BEAMS][MAX_RANGE];
  float wid[MAX_BEAMS][MAX_RANGE];
  float elv[MAX_BEAMS][MAX_RANGE];
} buffer;

struct OptionData opt;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: fitacfclientgui --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {
  int i,j,arg;
  int nrng=75;
  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  unsigned char colorflg=1;
  unsigned char rngflg=0;
  unsigned char gflg=0;
  unsigned char menu=1;
  double nlevels=5;
  double smin=0;
  double smax=0;
  int val=0;
  int start=0;

  unsigned char pwrflg=0;
  double pmin=0;
  double pmax=30;

  unsigned char velflg=0;
  double vmin=-500;
  double vmax=500;

  unsigned char widflg=0;
  double wmin=0;
  double wmax=250;

  unsigned char elvflg=0;
  double emin=0;
  double emax=40;

  int sock;
  int remote_port=0;
  char host[256];
  int flag,status;
  struct RadarParm *prm;
  struct FitData *fit;

  int c=0;

  prm=RadarParmMake();
  fit=FitMake();

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

  OptionAdd(&opt,"nrange",'i',&nrng);
  OptionAdd(&opt,"r",'x',&rngflg);

  OptionAdd(&opt,"gs",'x',&gflg);
  OptionAdd(&opt,"p",'x',&pwrflg);
  OptionAdd(&opt,"pmin",'d',&pmin);
  OptionAdd(&opt,"pmax",'d',&pmax);
  OptionAdd(&opt,"v",'x',&velflg);
  OptionAdd(&opt,"vmin",'d',&vmin);
  OptionAdd(&opt,"vmax",'d',&vmax);
  OptionAdd(&opt,"w",'x',&widflg);
  OptionAdd(&opt,"wmin",'d',&wmin);
  OptionAdd(&opt,"wmax",'d',&wmax);
  OptionAdd(&opt,"e",'x',&elvflg);
  OptionAdd(&opt,"emin",'d',&emin);
  OptionAdd(&opt,"emax",'d',&emax);

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

  if (argc-arg<2) {
    OptionPrintInfo(stdout,errstr);
    exit(-1);
  }

  strcpy(host,argv[argc-2]);
  remote_port=atoi(argv[argc-1]);

  sock=ConnexOpen(host,remote_port,NULL); 

  if (sock<0) {
    fprintf(stderr,"Could not connect to host.\n");
    exit(-1);
  }

  /* Initialize new screen */
  initscr();

  signal(SIGWINCH, NULL);

  /* Make getch a non-blocking call */
  nodelay(stdscr,TRUE);

  /* Disable input line buffering and don't echo */
  cbreak();
  noecho();

  /* Enable the keypad */
  keypad(stdscr, TRUE);

  /* Hide the cursor */
  curs_set(0);

  /* Check for color support */
  if (has_colors() == FALSE) colorflg = 0;

  /* Initialize colors */
  if (colorflg) {
    start_color();
    init_pair(1, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(2, COLOR_BLUE, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_CYAN, COLOR_BLACK);
    init_pair(5, COLOR_YELLOW, COLOR_BLACK);
    init_pair(6, COLOR_RED, COLOR_BLACK);

    init_pair(7, COLOR_MAGENTA, COLOR_MAGENTA);
    init_pair(8, COLOR_BLUE, COLOR_BLUE);
    init_pair(9, COLOR_GREEN, COLOR_GREEN);
    init_pair(10, COLOR_CYAN, COLOR_CYAN);
    init_pair(11, COLOR_YELLOW, COLOR_YELLOW);
    init_pair(12, COLOR_RED, COLOR_RED);

    init_pair(13, COLOR_WHITE, COLOR_BLACK);
    init_pair(14, COLOR_WHITE, COLOR_WHITE);

    if ((!pwrflg) && (!velflg) && (!widflg) && (!elvflg)) pwrflg=1;

    if (pwrflg) {
      smin=pmin;
      smax=pmax;
    } else if (velflg) {
      smin=vmin;
      smax=vmax;
    } else if (widflg) {
      smin=wmin;
      smax=wmax;
    } else if (elvflg) {
      smin=emin;
      smax=emax;
    }
  }

  if (nrng > MAX_RANGE) nrng = MAX_RANGE;

  do {

    /* Check for key press to exit */
    c = getch();
    if (colorflg) {
      if (c == 'p') {
        pwrflg=1;
        velflg=0;
        widflg=0;
        elvflg=0;
        smin=pmin;
        smax=pmax;
      } else if (c == 'v') {
        pwrflg=0;
        velflg=1;
        widflg=0;
        elvflg=0;
        smin=vmin;
        smax=vmax;
      } else if (c == 'w') {
        pwrflg=0;
        velflg=0;
        widflg=1;
        elvflg=0;
        smin=wmin;
        smax=wmax;
      } else if (c == 'e') {
        pwrflg=0;
        velflg=0;
        widflg=0;
        elvflg=1;
        smin=emin;
        smax=emax;
      } else if (c == 'g') {
        gflg = !gflg;
      } else if (c == 'r') {
        rngflg = !rngflg;
      } else if (c == 'n') {
        menu = !menu;
      } else if (c == KEY_UP) {
        if (pwrflg || elvflg ) smax += 5;
        else if (widflg) smax += 50;
        else if (velflg) {
          smax += 100;
          smin -= 100;
        }
      } else if (c == KEY_DOWN) {
        if (pwrflg || elvflg ) {
          smax -= 5;
          if (smax < 5) smax=5;
        } else if (widflg) {
          smax -= 50;
          if (smax < 50) smax=50;
        } else if (velflg) {
          smax -= 100;
          smin += 100;
          if (smax < 100) {
            smax=100;
            smin=-100;
          }
        }
      } else if (c == KEY_RIGHT) {
        if (pwrflg) {
          pwrflg=0;
          velflg=1;
          smin=vmin;
          smax=vmax;
        } else if (velflg) {
          velflg=0;
          widflg=1;
          smin=wmin;
          smax=wmax;
        } else if (widflg) {
          widflg=0;
          elvflg=1;
          smin=emin;
          smax=emax;
        } else if (elvflg) {
          elvflg=0;
          pwrflg=1;
          smin=pmin;
          smax=pmax;
        }
      } else if (c == KEY_LEFT) {
        if (pwrflg) {
          pwrflg=0;
          elvflg=1;
          smin=emin;
          smax=emax;
        } else if (velflg) {
          velflg=0;
          pwrflg=1;
          smin=pmin;
          smax=pmax;
        } else if (widflg) {
          widflg=0;
          velflg=1;
          smin=vmin;
          smax=vmax;
        } else if (elvflg) {
          elvflg=0;
          widflg=1;
          smin=wmin;
          smax=wmax;
        }
      } else if (c != ERR) break;
    } else if (c != ERR) break;

    status=FitCnxRead(1,&sock,prm,fit,&flag,NULL);

    if (status==-1) break;

    if (flag !=-1) {

      /* Store data from most recent beam in buffer */
      buffer.beam[prm->bmnum]=1;
      for (i=0; i<nrng; i++) {
        if (i >= prm->nrang) break;
        buffer.qflg[prm->bmnum][i]=fit->rng[i].qflg;
        if (fit->rng[i].qflg == 1) {
          buffer.gsct[prm->bmnum][i]=fit->rng[i].gsct;
          buffer.pow[prm->bmnum][i]=fit->rng[i].p_l;
          buffer.vel[prm->bmnum][i]=fit->rng[i].v;
          buffer.wid[prm->bmnum][i]=fit->rng[i].w_l;
          if (prm->xcf !=0 && fit->elv !=NULL) buffer.elv[prm->bmnum][i]=fit->elv[i].normal;
        }
      }

      /* Print date/time and radar operating parameters */
      move(0, 0);
      clrtoeol();
      printw("%04d-%02d-%02d %02d:%02d:%02d\n",
             prm->time.yr,prm->time.mo,prm->time.dy,
             prm->time.hr,prm->time.mt,prm->time.sc);
      clrtoeol();
      printw("stid  = %3d  cpid  = %d  channel = %d\n", prm->stid,prm->cp,prm->channel);
      clrtoeol();
      printw("bmnum = %3d  bmazm = %.2f  xcf = %d\n", prm->bmnum,prm->bmazm,prm->xcf);
      clrtoeol();
      printw("intt  = %3.1f  nave  = %3d  tfreq = %d\n",
             prm->intt.sc+prm->intt.us/1.0e6,prm->nave,prm->tfreq);
      clrtoeol();
      printw("frang = %3d  nrang = %3d\n", prm->frang,prm->nrang);
      clrtoeol();
      printw("rsep  = %3d  noise.search = %g\n", prm->rsep,prm->noise.search);
      clrtoeol();
      printw("scan  = %3d  noise.sky    = %g\n", prm->scan,fit->noise.skynoise);
      clrtoeol();
      printw("mppul = %3d  mpinc = %d\n", prm->mppul,prm->mpinc);

      clrtoeol();
      printw("origin.code = ");
      if ((prm->origin.time !=NULL) && (prm->origin.command !=NULL)) printw("%d\n", prm->origin.code);
      else printw("\n");

      clrtoeol();
      printw("origin.time = ");
      if (prm->origin.time != NULL) printw("%s\n",prm->origin.time);
      else printw("\n");

      clrtoeol();
      printw("origin.command = ");
      if (prm->origin.command !=NULL) printw("%s\n\n",prm->origin.command);
      else printw("\n\n");

      /* Draw a menu explaining the keyboard controls */
      if (colorflg) {
        move(0, 50);
        addch(ACS_ULCORNER);
        for (i=0;i<3;i++) addch(ACS_HLINE);
        if (menu) {
          printw("Keyboard Controls (1/2)");
          for (i=0;i<3;i++) addch(ACS_HLINE);
          addch(ACS_URCORNER);
          move(1, 50); addch(ACS_VLINE);
          printw(" p : power          n : next");
          move(1, 80); addch(ACS_VLINE);
          move(2, 50); addch(ACS_VLINE);
          printw(" v : velocity           page");
          move(2, 80); addch(ACS_VLINE);
          move(3, 50); addch(ACS_VLINE);
          printw(" w : spectral width");
          move(3, 80); addch(ACS_VLINE);
          move(4, 50); addch(ACS_VLINE);
          printw(" e : elevation angle");
          move(4, 80); addch(ACS_VLINE);
          move(5, 50); addch(ACS_VLINE);
          printw(" g : GS flag (velocity only)");
        } else {
          printw("Keyboard Controls (2/2)");
          for (i=0;i<3;i++) addch(ACS_HLINE);
          addch(ACS_URCORNER);
          move(1, 50); addch(ACS_VLINE);
          printw(" Arrow Keys:        n : prev");
          move(1, 80); addch(ACS_VLINE);
          move(2, 50); addch(ACS_VLINE);
          move(2, 52); addch(ACS_RARROW);
          printw(" : change param       page");
          move(2, 80); addch(ACS_VLINE);
          move(3, 50); addch(ACS_VLINE);
          move(3, 52); addch(ACS_LARROW);
          printw(" : change param");
          move(3, 80); addch(ACS_VLINE);
          move(4, 50); addch(ACS_VLINE);
          move(4, 52); addch(ACS_UARROW);
          printw(" : increase scale r : show");
          move(4, 80); addch(ACS_VLINE);
          move(5, 50); addch(ACS_VLINE);
          move(5, 52); addch(ACS_DARROW);
          if (rngflg) printw(" : decrease scale     gate");
          else        printw(" : decrease scale     rng");
        }
        move(5, 80); addch(ACS_VLINE);
        move(6, 50); addch(ACS_VLINE);
        move(6, 80); addch(ACS_VLINE);
        move(7, 50); addch(ACS_VLINE);
        printw(" Press any other key to quit ");
        addch(ACS_VLINE);
        move(8, 50); addch(ACS_LLCORNER);
        for (i=0;i<29;i++) addch(ACS_HLINE);
        addch(ACS_LRCORNER);
      } else {
        move(0, 53);
        printw("* Press any key to quit *");
      }

      /* Draw range gate labels */
      move(12, 0);
      if (rngflg) {
        printw("B\\R %d",prm->frang);
        for (i=1; i*10<nrng; i++) {
          printw("%10d",prm->frang+prm->rsep*i*10);
        }
      } else {
        printw("B\\G 0         ");
        for (i=1; i*10<nrng; i++) {
          if (i*10 < 100) printw("%d        ",i*10);
          else            printw("%d       ",i*10);
        }
      }
      printw("\n");

      /* Draw each range gate for each beam */
      for (j=0; j<MAX_BEAMS; j++) {
        if (buffer.beam[j] == 0) continue;
        move(j+13, 0);
        clrtoeol();
        if ((j==prm->bmnum) && colorflg) attron(COLOR_PAIR(6));
        printw("%02d: ",j);
        if ((j==prm->bmnum) && colorflg) attroff(COLOR_PAIR(6));
        for (i=0; i<nrng; i++) {
          if (buffer.qflg[j][i] == 1) {
            if (colorflg) {
              if (pwrflg)      val = (int)((buffer.pow[j][i]-smin)/(smax-smin)*nlevels)+1;
              else if (velflg) val = (int)((buffer.vel[j][i]-smin)/(smax-smin)*nlevels)+1;
              else if (widflg) val = (int)((buffer.wid[j][i]-smin)/(smax-smin)*nlevels)+1;
              else if (elvflg) val = (int)((buffer.elv[j][i]-smin)/(smax-smin)*nlevels)+1;

              if (val < 1) val=1;
              if (val > nlevels+1) val=nlevels+1;
              if (gflg && velflg && buffer.gsct[j][i]) attron(COLOR_PAIR(13));
              else                                     attron(COLOR_PAIR(val));
            }

            if (buffer.gsct[j][i] != 0) printw("g");
            else                        printw("i");

            if (colorflg) {
              if (gflg && velflg && buffer.gsct[j][i]) attroff(COLOR_PAIR(13));
              else                                     attroff(COLOR_PAIR(val));
            }
          } else {
            printw("-");
          }
        }
        printw("\n");
      }

      /* Draw a color bar */
      if (colorflg) {
        move(11, nrng+4);
        if (pwrflg)      printw("Pow [dB]");
        else if (velflg) printw("Vel [m/s]");
        else if (widflg) printw("Wid [m/s]");
        else if (elvflg) printw("Elv [deg]");
        start=12;
        for (j=12;j>6;j--) {
          attron(COLOR_PAIR(j));
          for (i=start;i<start+2;i++) {
            move(i, nrng+5);
            printw(" ");
          }
          attroff(COLOR_PAIR(j));
          move(i-1, nrng+7);
          clrtoeol();
          printw("%d",(int)((j-7)*(smax-smin)/nlevels+smin));
          start=start+2;
        }

        if (velflg && gflg) {
          attron(COLOR_PAIR(14));
          move(25, nrng+5);
          printw(" ");
          attroff(COLOR_PAIR(14));
          printw(" GS");
        } else {
          move(25, nrng+5);
          clrtoeol();
        }
      }

      /* Send output to terminal */
      refresh();

    }

  } while(1);

  /* Exit and restore terminal settings */
  endwin();

  RadarParmFree(prm);
  FitFree(fit);

  return 0;
}
