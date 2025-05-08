/* sndclientgui.c
   =================
   Author: E.G.Thomas
 
Copyright (C) 2023  Evan G. Thomas

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
#include "rtime.h"
#include "rprm.h"
#include "fitdata.h"
#include "connex.h"
#include "fitcnx.h"

#include "errstr.h"
#include "hlpstr.h"


#define MAX_BEAMS 24
#define MAX_RANGE 300

#define SND_BEAMS 24
#define SND_RANGE 100
#define SND_FREQS 22

struct FitBuffer {
  int beam[MAX_BEAMS];
  int qflg[MAX_BEAMS][MAX_RANGE];
  int gsct[MAX_BEAMS][MAX_RANGE];
  float vel[MAX_BEAMS][MAX_RANGE];
  float pow[MAX_BEAMS][MAX_RANGE];
  float wid[MAX_BEAMS][MAX_RANGE];
  float elv[MAX_BEAMS][MAX_RANGE];
};

struct SndBuffer {
  int beam[SND_BEAMS][SND_FREQS];
  int qflg[SND_BEAMS][SND_RANGE][SND_FREQS];
  int gsct[SND_BEAMS][SND_RANGE][SND_FREQS];
  float vel[SND_BEAMS][SND_RANGE][SND_FREQS];
  float pow[SND_BEAMS][SND_RANGE][SND_FREQS];
  float wid[SND_BEAMS][SND_RANGE][SND_FREQS];
  float elv[SND_BEAMS][SND_RANGE][SND_FREQS];
  double time[SND_BEAMS][SND_FREQS];
};

struct PlotOptions {
  int nrng;
  unsigned char colorflg;
  unsigned char rngflg;
  unsigned char gflg;
  unsigned char menu;
  double nlevels;
  double smin;
  double smax;
  unsigned char pwrflg;
  double pmin;
  double pmax;
  unsigned char velflg;
  double vmin;
  double vmax;
  unsigned char widflg;
  double wmin;
  double wmax;
  unsigned char elvflg;
  double emin;
  double emax;
  int max_beam;
  int sndflg;
  int b;
  int f;
  int snd;
  int hold;
  int force;
};

struct OptionData opt;


void init_plot(struct PlotOptions *plot);
int check_key(int c, struct PlotOptions *plot);
void read_fit_data(struct RadarParm *prm, struct FitData *fit, struct FitBuffer *fbuf, struct PlotOptions *plot);
void read_snd_data(struct RadarParm *prm, struct FitData *fit, struct SndBuffer *sbuf, struct PlotOptions *plot);
void print_radar_param(struct RadarParm *prm, struct FitData *fit);
void draw_menu(struct PlotOptions *plot);
void draw_snd_data(struct RadarParm *prm, struct SndBuffer *sbuf, struct PlotOptions *plot);
void draw_fit_data(struct RadarParm *prm, struct FitBuffer *fbuf, struct PlotOptions *plot);
void draw_colorbar(struct PlotOptions *plot);


int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: sndclientgui --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {
  int arg;
  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  struct FitBuffer fbuf;
  struct SndBuffer sbuf;
  struct PlotOptions plot;

  int sock;
  int remote_port=0;
  char host[256];
  int flag,status;
  struct RadarParm *prm;
  struct FitData *fit;

  int c=0;
  int ret=0;

  memset(&fbuf,0,sizeof(struct FitBuffer));
  memset(&sbuf,0,sizeof(struct SndBuffer));

  prm=RadarParmMake();
  fit=FitMake();

  init_plot(&plot);

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

  OptionAdd(&opt,"nrange",'i',&plot.nrng);
  OptionAdd(&opt,"r",'x',&plot.rngflg);

  OptionAdd(&opt,"gs",'x',&plot.gflg);
  OptionAdd(&opt,"p",'x',&plot.pwrflg);
  OptionAdd(&opt,"pmin",'d',&plot.pmin);
  OptionAdd(&opt,"pmax",'d',&plot.pmax);
  OptionAdd(&opt,"v",'x',&plot.velflg);
  OptionAdd(&opt,"vmin",'d',&plot.vmin);
  OptionAdd(&opt,"vmax",'d',&plot.vmax);
  OptionAdd(&opt,"w",'x',&plot.widflg);
  OptionAdd(&opt,"wmin",'d',&plot.wmin);
  OptionAdd(&opt,"wmax",'d',&plot.wmax);
  OptionAdd(&opt,"e",'x',&plot.elvflg);
  OptionAdd(&opt,"emin",'d',&plot.emin);
  OptionAdd(&opt,"emax",'d',&plot.emax);

  OptionAdd(&opt,"snd",'x',&plot.sndflg);
  OptionAdd(&opt,"force",'x',&plot.force);

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

  if (has_colors() == FALSE) plot.colorflg = 0;

  /* Initialize colors */
  if (plot.colorflg) {
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

    if ((!plot.pwrflg) && (!plot.velflg) && (!plot.widflg) && (!plot.elvflg)) plot.pwrflg=1;

    if (plot.pwrflg) {
      plot.smin = plot.pmin;
      plot.smax = plot.pmax;
    } else if (plot.velflg) {
      plot.smin = plot.vmin;
      plot.smax = plot.vmax;
    } else if (plot.widflg) {
      plot.smin = plot.wmin;
      plot.smax = plot.wmax;
    } else if (plot.elvflg) {
      plot.smin = plot.emin;
      plot.smax = plot.emax;
    }
  }

  if (plot.nrng > MAX_RANGE) plot.nrng = MAX_RANGE;


  do {

    /* Read real-time data from radar */
    status=FitCnxRead(1,&sock,prm,fit,&flag,NULL);

    if (status==-1) break;

    /* Check for key press to change options or exit */
    c = getch();
    ret = check_key(c,&plot);
    if (ret == -1) break;

    if (flag !=-1) {

      /* Store data from most recent beam in buffer */
      if (plot.force) {
        read_snd_data(prm,fit,&sbuf,&plot);
        read_fit_data(prm,fit,&fbuf,&plot);
      } else {
        if (prm->scan == -2) read_snd_data(prm,fit,&sbuf,&plot);
        else                 read_fit_data(prm,fit,&fbuf,&plot);
      }

      /* Print date/time and radar operating parameters */
      print_radar_param(prm,fit);

      /* Draw a menu explaining the keyboard controls */
      draw_menu(&plot);

      /* Draw fit or SND data vs range gate */
      if (plot.sndflg) draw_snd_data(prm,&sbuf,&plot);
      else             draw_fit_data(prm,&fbuf,&plot);

      /* Draw a color bar */
      if (plot.colorflg) draw_colorbar(&plot);

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


/* Initialize plotting options */
void init_plot(struct PlotOptions *plot) {

  plot->nrng = 75;
  plot->colorflg = 1;
  plot->rngflg = 0;
  plot->gflg = 0;
  plot->menu = 1;

  plot->nlevels = 5;
  plot->smin = 0;
  plot->smax = 0;

  plot->pwrflg = 0;
  plot->pmin = 0;
  plot->pmax = 30;

  plot->velflg = 0;
  plot->vmin = -500;
  plot->vmax = 500;

  plot->widflg = 0;
  plot->wmin = 0;
  plot->wmax = 250;

  plot->elvflg = 0;
  plot->emin = 0;
  plot->emax = 40;

  plot->max_beam = -100;

  plot->sndflg = 0;
  plot->b = 0;
  plot->f = 0;
  plot->snd = 0;
  plot->hold = 0;
  plot->force = 0;
}


/* Check for key press to change options or exit */
int check_key(int c, struct PlotOptions *plot) {

  if (plot->colorflg) {
    if (c == 'p') {
      plot->pwrflg = 1;
      plot->velflg = 0;
      plot->widflg = 0;
      plot->elvflg = 0;
      plot->smin = plot->pmin;
      plot->smax = plot->pmax;
    } else if (c == 'v') {
      plot->pwrflg = 0;
      plot->velflg = 1;
      plot->widflg = 0;
      plot->elvflg = 0;
      plot->smin = plot->vmin;
      plot->smax = plot->vmax;
    } else if (c == 'w') {
      plot->pwrflg = 0;
      plot->velflg = 0;
      plot->widflg = 1;
      plot->elvflg = 0;
      plot->smin = plot->wmin;
      plot->smax = plot->wmax;
    } else if (c == 'e') {
      plot->pwrflg = 0;
      plot->velflg = 0;
      plot->widflg = 0;
      plot->elvflg = 1;
      plot->smin = plot->emin;
      plot->smax = plot->emax;
    } else if (c == 'g') {
      plot->gflg = !plot->gflg;
    } else if (c == 'r') {
      plot->rngflg = !plot->rngflg;
    } else if (c == 's') {
      plot->sndflg = !plot->sndflg;
      plot->hold = 0;
    } else if (c == 'h') {
      plot->hold = !plot->hold;
    } else if (c == 'n') {
      plot->menu = !plot->menu;
    } else if (c == KEY_UP) {
      if (plot->sndflg) {
        plot->b += 1;
        if (plot->b > SND_BEAMS-1) {
          plot->b = 0;
        }
        plot->hold = 1;
      } else {
        if (plot->pwrflg || plot->elvflg ) plot->smax += 5;
        else if (plot->widflg) plot->smax += 50;
        else if (plot->velflg) {
          plot->smax += 100;
          plot->smin -= 100;
        }
      }
    } else if (c == KEY_DOWN) {
      if (plot->sndflg) {
        plot->b -= 1;
        if (plot->b < 0) {
          plot->b = SND_BEAMS-1;
        }
        plot->hold = 1;
      } else {
        if (plot->pwrflg || plot->elvflg ) {
          plot->smax -= 5;
          if (plot->smax < 5) plot->smax=5;
        } else if (plot->widflg) {
          plot->smax -= 50;
          if (plot->smax < 50) plot->smax=50;
        } else if (plot->velflg) {
          plot->smax -= 100;
          plot->smin += 100;
          if (plot->smax < 100) {
            plot->smax=100;
            plot->smin=-100;
          }
        }
      }
    } else if (c == KEY_RIGHT) {
      if (plot->pwrflg) {
        plot->pwrflg = 0;
        plot->velflg = 1;
        plot->smin = plot->vmin;
        plot->smax = plot->vmax;
      } else if (plot->velflg) {
        plot->velflg = 0;
        plot->widflg = 1;
        plot->smin = plot->wmin;
        plot->smax = plot->wmax;
      } else if (plot->widflg) {
        plot->widflg = 0;
        plot->elvflg = 1;
        plot->smin = plot->emin;
        plot->smax = plot->emax;
      } else if (plot->elvflg) {
        plot->elvflg = 0;
        plot->pwrflg = 1;
        plot->smin = plot->pmin;
        plot->smax = plot->pmax;
      }
    } else if (c == KEY_LEFT) {
      if (plot->pwrflg) {
        plot->pwrflg = 0;
        plot->elvflg = 1;
        plot->smin = plot->emin;
        plot->smax = plot->emax;
      } else if (plot->velflg) {
        plot->velflg = 0;
        plot->pwrflg = 1;
        plot->smin = plot->pmin;
        plot->smax = plot->pmax;
      } else if (plot->widflg) {
        plot->widflg = 0;
        plot->velflg = 1;
        plot->smin = plot->vmin;
        plot->smax = plot->vmax;
      } else if (plot->elvflg) {
        plot->elvflg = 0;
        plot->widflg = 1;
        plot->smin = plot->wmin;
        plot->smax = plot->wmax;
      }
    } else if (c != ERR) return -1;
  } else if (c != ERR) return -1;

  return 0;
}


/* Store data from most recent beam in buffer */
void read_fit_data(struct RadarParm *prm, struct FitData *fit,
                   struct FitBuffer *fbuf, struct PlotOptions *plot) {

  int i;

  plot->snd = 0;

  fbuf->beam[prm->bmnum] = 1;

  if (prm->bmnum > plot->max_beam) plot->max_beam = prm->bmnum;

  for (i=0; i<plot->nrng; i++) {
    if (i >= prm->nrang) break;
    fbuf->qflg[prm->bmnum][i] = fit->rng[i].qflg;
    if (fit->rng[i].qflg == 1) {
      fbuf->gsct[prm->bmnum][i] = fit->rng[i].gsct;
      fbuf->pow[prm->bmnum][i] = fit->rng[i].p_l;
      fbuf->vel[prm->bmnum][i] = fit->rng[i].v;
      fbuf->wid[prm->bmnum][i] = fit->rng[i].w_l;
      if (prm->xcf !=0 && fit->elv !=NULL) fbuf->elv[prm->bmnum][i] = fit->elv[i].normal;
    }
  }
}


/* Store data from most recent SND beam in buffer */
void read_snd_data(struct RadarParm *prm, struct FitData *fit,
                   struct SndBuffer *sbuf, struct PlotOptions *plot) {

  int i;

  plot->snd = 1;

  if (!plot->hold) plot->b = prm->bmnum;
  plot->f = (int)((prm->tfreq-8000)/500);

  sbuf->beam[prm->bmnum][plot->f] = 1;

  for (i=0; i<SND_RANGE; i++) {
    if (i >= prm->nrang) break;
    sbuf->qflg[prm->bmnum][i][plot->f] = fit->rng[i].qflg;
    if (fit->rng[i].qflg == 1) {
      sbuf->gsct[prm->bmnum][i][plot->f] = fit->rng[i].gsct;
      sbuf->pow[prm->bmnum][i][plot->f] = fit->rng[i].p_l;
      sbuf->vel[prm->bmnum][i][plot->f] = fit->rng[i].v;
      sbuf->wid[prm->bmnum][i][plot->f] = fit->rng[i].w_l;
      if (prm->xcf !=0 && fit->elv !=NULL) sbuf->elv[prm->bmnum][i][plot->f] = fit->elv[i].normal;
    }
    sbuf->time[prm->bmnum][plot->f] = TimeYMDHMSToEpoch(prm->time.yr,prm->time.mo,prm->time.dy,
                                                        prm->time.hr,prm->time.mt,
                                                        prm->time.sc+prm->time.us/1.0e6);
  }

}


/* Print date/time and radar operating parameters */
void print_radar_param(struct RadarParm *prm, struct FitData *fit) {

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
}


/* Draw a menu explaining the keyboard controls */
void draw_menu(struct PlotOptions *plot) {

  int i;

  if (plot->colorflg) {
    move(0, 50);
    addch(ACS_ULCORNER);
    for (i=0;i<3;i++) addch(ACS_HLINE);
    if (plot->menu) {
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
      if (plot->sndflg) printw(" : increase beam  s : SND");
      else              printw(" : increase scale s : SND");
      move(4, 80); addch(ACS_VLINE);
      move(5, 50); addch(ACS_VLINE);
      move(5, 52); addch(ACS_DARROW);
      if (plot->sndflg) printw(" : decrease beam      mode");
      else              printw(" : decrease scale     mode");
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
}


/* Draw fit data versus range gate and beam */
void draw_fit_data(struct RadarParm *prm, struct FitBuffer *fbuf, struct PlotOptions *plot) {

  int i,j;
  int val=0;

  /* Draw range gate labels */
  move(12, 0);
  if (plot->rngflg) {
    printw("B\\R %d",prm->frang);
    for (i=1; i*10<plot->nrng; i++) {
      printw("%10d",prm->frang+prm->rsep*i*10);
    }
  } else {
    printw("B\\G 0         ");
    for (i=1; i*10<plot->nrng; i++) {
      if (i*10 < 100) printw("%d        ",i*10);
      else            printw("%d       ",i*10);
    }
  }
  printw("\n");

  /* Draw each range gate for each beam */
  for (j=0; j<MAX_BEAMS; j++) {
    move(j+13, 0);
    clrtoeol();

    if (fbuf->beam[j] == 0) continue;

    if ((j==prm->bmnum) && (!plot->snd || plot->force) && plot->colorflg) attron(COLOR_PAIR(6));
    printw("%02d: ",j);
    if ((j==prm->bmnum) && (!plot->snd || plot->force) && plot->colorflg) attroff(COLOR_PAIR(6));

    for (i=0; i<plot->nrng; i++) {
      if (fbuf->qflg[j][i] == 1) {
        if (plot->colorflg) {
          if (plot->pwrflg)      val = (int)((fbuf->pow[j][i]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;
          else if (plot->velflg) val = (int)((fbuf->vel[j][i]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;
          else if (plot->widflg) val = (int)((fbuf->wid[j][i]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;
          else if (plot->elvflg) val = (int)((fbuf->elv[j][i]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;

          if (val < 1) val=1;
          if (val > plot->nlevels+1) val=plot->nlevels+1;
          if (plot->gflg && plot->velflg && fbuf->gsct[j][i]) attron(COLOR_PAIR(13));
          else                                                attron(COLOR_PAIR(val));
        }

        if (fbuf->gsct[j][i] != 0) printw("g");
        else                       printw("i");

        if (plot->colorflg) {
          if (plot->gflg && plot->velflg && fbuf->gsct[j][i]) attroff(COLOR_PAIR(13));
          else                                                attroff(COLOR_PAIR(val));
        }
      } else {
        printw("-");
      }
    }
    printw("\n");
  }
}


/* Draw SND data versus range gate and frequency */
void draw_snd_data(struct RadarParm *prm, struct SndBuffer *sbuf, struct PlotOptions *plot) {

  int i,j;

  int yr,mo,dy,hr,mt;
  double sc;
  double ftime=0.;

  int val=0;

  /* Draw range gate labels */
  move(12, 0);
  if (plot->rngflg) {
    printw("F\\R %d",prm->frang);
    for (i=1; i*10<plot->nrng; i++) {
      printw("%10d",prm->frang+prm->rsep*i*10);
    }
  } else {
    printw("F\\G 0         ");
    for (i=1; i*10<plot->nrng; i++) {
      if (i*10 < 100) printw("%d        ",i*10);
      else            printw("%d       ",i*10);
    }
  }
  printw("\n");

  /* Draw range gate for each frequency */
  for (j=0; j<SND_FREQS; j++) {
    move(j+13, 0);
    clrtoeol();

    if ((j==plot->f) && (prm->bmnum==plot->b) && (plot->snd || plot->force) && plot->colorflg) attron(COLOR_PAIR(6));
    printw("%3d ",j*5+80);
    if ((j==plot->f) && (prm->bmnum==plot->b) && (plot->snd || plot->force) && plot->colorflg) attroff(COLOR_PAIR(6));

    if (sbuf->beam[plot->b][j] == 0) continue;

    for (i=0; i<plot->nrng; i++) {
      if (sbuf->qflg[plot->b][i][j] == 1) {
        if (plot->colorflg) {
          if (plot->pwrflg)      val = (int)((sbuf->pow[plot->b][i][j]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;
          else if (plot->velflg) val = (int)((sbuf->vel[plot->b][i][j]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;
          else if (plot->widflg) val = (int)((sbuf->wid[plot->b][i][j]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;
          else if (plot->elvflg) val = (int)((sbuf->elv[plot->b][i][j]-plot->smin)/(plot->smax-plot->smin)*plot->nlevels)+1;

          if (val < 1) val=1;
          if (val > plot->nlevels+1) val=plot->nlevels+1;
          if (plot->gflg && plot->velflg && sbuf->gsct[plot->b][i][j]) attron(COLOR_PAIR(13));
          else                                                         attron(COLOR_PAIR(val));
        }

        if (sbuf->gsct[plot->b][i][j] != 0) printw("g");
        else                                printw("i");

        if (plot->colorflg) {
          if (plot->gflg && plot->velflg && sbuf->gsct[plot->b][i][j]) attroff(COLOR_PAIR(13));
          else                                                         attroff(COLOR_PAIR(val));
        }
      } else {
        printw("-");
      }
    }
    printw("\n");
  }

  /* Clear higher beams if necessary */
  if (plot->max_beam >= SND_FREQS) {
    move(SND_FREQS+13, 0);
    clrtoeol();
    move(SND_FREQS+14, 0);
    clrtoeol();
  }

  for (j=0; j<SND_FREQS; j++) {
    if (sbuf->time[plot->b][j] > ftime) {
      ftime = sbuf->time[plot->b][j];
    }
  }
  TimeEpochToYMDHMS(ftime,&yr,&mo,&dy,&hr,&mt,&sc);
  move(11, 0);
  printw("SND Beam: %02d   SND Time:", plot->b);
  if (ftime > 0) printw(" %04d-%02d-%02d %02d:%02d:%02d", yr,mo,dy,hr,mt,(int)sc);

}


/* Draw a color bar */
void draw_colorbar(struct PlotOptions *plot) {

  int i,j;
  int start=12;
  int rng=plot->nrng;

  move(11, rng+4);
  if (plot->pwrflg)      printw("Pow [dB]");
  else if (plot->velflg) printw("Vel [m/s]");
  else if (plot->widflg) printw("Wid [m/s]");
  else if (plot->elvflg) printw("Elv [deg]");

  for (j=12; j>6; j--) {
    attron(COLOR_PAIR(j));
    for (i=start; i<start+2; i++) {
      move(i, rng+5);
      printw(" ");
    }
    attroff(COLOR_PAIR(j));
    move(i-1, rng+7);
    clrtoeol();
    printw("%d",(int)((j-7)*(plot->smax-plot->smin)/plot->nlevels+plot->smin));
    start=start+2;
  }

  if (plot->velflg && plot->gflg) {
    attron(COLOR_PAIR(14));
    move(25, rng+5);
    printw(" ");
    attroff(COLOR_PAIR(14));
    printw(" GS");
  } else {
    move(25, rng+5);
    clrtoeol();
  }
}
