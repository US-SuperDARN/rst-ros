/* sequence.c
   ==========
   Author: E.G.Thomas
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <zlib.h>
#include "rtypes.h"
#include "limit.h"
#include "global.h"
#include "sequence.h"


struct sequence *OpsSequenceMake() {

  struct sequence *ptr=NULL;
  ptr=malloc(sizeof(struct sequence));
  if (ptr==NULL) return NULL;
  memset(ptr,0,sizeof(struct sequence));
  ptr->ptab=NULL;
  ptr->lags=NULL;
  return ptr;
}


int OpsBuild7pulse(struct sequence *ptr) {

  void *tmp=NULL;

  int ptab[7] = {0,9,12,20,22,26,27};

  int lags[LAG_SIZE][2] = {
    { 0, 0},    /*  0 */
    {26,27},    /*  1 */
    {20,22},    /*  2 */
    { 9,12},    /*  3 */
    {22,26},    /*  4 */
    {22,27},    /*  5 */
    {20,26},    /*  6 */
    {20,27},    /*  7 */
    {12,20},    /*  8 */
    { 0, 9},    /*  9 */
    {12,22},    /* 10 */
    { 9,20},    /* 11 */
    { 0,12},    /* 12 */
    { 9,22},    /* 13 */
    {12,26},    /* 14 */
    {12,27},    /* 15 */

    { 9,26},    /* 17 */
    { 9,27},    /* 18 */
    {27,27}};   /* alternate lag-0 */

  ptr->mppul = 7;
  ptr->mplgs = 18;
  ptr->mpinc = 2400;
  ptr->mplgexs = 0;

  if (ptr->ptab==NULL) tmp = malloc(sizeof(ptab));
  else tmp = realloc(ptr->ptab,sizeof(ptab));
  if (tmp==NULL) return -1;
  memcpy(tmp,ptab,sizeof(ptab));
  ptr->ptab = tmp;

  if (ptr->lags==NULL) tmp = malloc(sizeof(lags));
  else tmp = realloc(ptr->lags,sizeof(lags));
  if (tmp==NULL) return -1;
  memcpy(tmp,lags,sizeof(lags));
  ptr->lags = tmp;

  return 0;
}


int OpsBuild8pulse(struct sequence *ptr) {

  void *tmp=NULL;

  int ptab[8] = {0,14,22,24,27,31,42,43};

  int lags[LAG_SIZE][2] = {
    { 0, 0},    /*  0 */
    {42,43},    /*  1 */
    {22,24},    /*  2 */
    {24,27},    /*  3 */
    {27,31},    /*  4 */
    {22,27},    /*  5 */

    {24,31},    /*  7 */
    {14,22},    /*  8 */
    {22,31},    /*  9 */
    {14,24},    /* 10 */
    {31,42},    /* 11 */
    {31,43},    /* 12 */
    {14,27},    /* 13 */
    { 0,14},    /* 14 */
    {27,42},    /* 15 */
    {27,43},    /* 16 */
    {14,31},    /* 17 */
    {24,42},    /* 18 */
    {24,43},    /* 19 */
    {22,42},    /* 20 */
    {22,43},    /* 21 */
    { 0,22},    /* 22 */

    { 0,24},    /* 24 */

    {43,43}};   /* alternate lag-0  */

  ptr->mppul = 8;
  ptr->mplgs = 23;
  ptr->mpinc = 1500;
  ptr->mplgexs = 0;

  if (ptr->ptab==NULL) tmp = malloc(sizeof(ptab));
  else tmp = realloc(ptr->ptab,sizeof(ptab));
  if (tmp==NULL) return -1;
  memcpy(tmp,ptab,sizeof(ptab));
  ptr->ptab = tmp;

  if (ptr->lags==NULL) tmp = malloc(sizeof(lags));
  else tmp = realloc(ptr->lags,sizeof(lags));
  if (tmp==NULL) return -1;
  memcpy(tmp,lags,sizeof(lags));
  ptr->lags = tmp;

  return 0;
}


int OpsBuild16pulse(struct sequence *ptr) {

  void *tmp=NULL;

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

  ptr->mppul = 16;
  ptr->mplgs = 121;
  ptr->mpinc = 100;
  ptr->mplgexs = 0;

  if (ptr->ptab==NULL) tmp = malloc(sizeof(ptab));
  else tmp = realloc(ptr->ptab,sizeof(ptab));
  if (tmp==NULL) return -1;
  memcpy(tmp,ptab,sizeof(ptab));
  ptr->ptab = tmp;

  if (ptr->lags==NULL) tmp = malloc(sizeof(lags));
  else tmp = realloc(ptr->lags,sizeof(lags));
  if (tmp==NULL) return -1;
  memcpy(tmp,lags,sizeof(lags));
  ptr->lags = tmp;

  return 0;
}


int OpsBuildTauscan(struct sequence *ptr) {

  int i=0;
  void *tmp=NULL;

  int ptab[13] = {0,15,16,23,27,29,32,47,50,52,56,63,64};

  int lags[LAG_SIZE][2] = {
    { 0, 0},              /* 0 */
    {15,16},    /* 1 */
    {63,64},              /* 1 */
    {27,29},    /* 2 */
    {50,52},              /* 2 */
    {29,32},    /* 3 */
    {47,50},              /* 3 */
    {23,27},    /* 4 */
    {52,56},              /* 4 */
    {27,32},    /* 5 */
    {47,52},              /* 5 */
    {23,29},    /* 6 */
    {50,56},              /* 6 */
    {16,23},    /* 7 */
    {56,63},              /* 7 */
    {15,23},    /* 8 */
    {56,64},              /* 8 */
    {23,32},    /* 9 */
    {47,56},              /* 9 */
    /* missing 10 */
    {16,27},    /* 11 */
    {52,63},              /* 11 */
    {15,27},    /* 12 */
    {52,64},              /* 12 */
    {16,29},    /* 13 */
    {50,63},              /* 13 */
    {15,29},    /* 14 */
    {50,64},              /* 14 */
    {32,47},    /* 15 */
    { 0,15},              /* 15 */
    {16,32},    /* 16 */
    {47,63},              /* 16 */
    {15,32},    /* 17 */
    {47,64},              /* 17 */
    {64,64}     /* alternate lag zero */
  };

  ptr->mppul = 13;
  ptr->mplgs = 18;
  ptr->mpinc = 2400;
  ptr->mplgexs = 0;

  for (i=1; i<LAG_SIZE; i++) {
    if ((lags[i][0]==64) && (lags[i][1]==64)) break;
    ptr->mplgexs++;
  }
  ptr->mplgexs++;

  if (ptr->ptab==NULL) tmp = malloc(sizeof(ptab));
  else tmp = realloc(ptr->ptab,sizeof(ptab));
  if (tmp==NULL) return -1;
  memcpy(tmp,ptab,sizeof(ptab));
  ptr->ptab = tmp;

  if (ptr->lags==NULL) tmp = malloc(sizeof(lags));
  else tmp = realloc(ptr->lags,sizeof(lags));
  if (tmp==NULL) return -1;
  memcpy(tmp,lags,sizeof(lags));
  ptr->lags = tmp;

  return 0;
}


int OpsBuildTauscan11(struct sequence *ptr) {

  int i=0;
  void *tmp=NULL;

  int ptab[11] = {0,10,13,14,19,21,31,33,38,39,42};

  int lags[LAG_SIZE][2] = {
    { 0, 0},              /* 0 */
    {13,14},    /* 1 */
    {38,39},              /* 1 */
    {19,21},    /* 2 */
    {31,33},              /* 2 */
    {10,13},    /* 3 */
    {39,42},              /* 3 */
    {10,14},    /* 4 */
    {38,42},              /* 4 */
    {14,19},    /* 5 */
    {33,38},              /* 5 */
    {13,19},    /* 6 */
    {33,39},              /* 6 */
    {16,23},    /* 7 */
    {31,38},              /* 7 */
    {15,23},    /* 8 */
    {31,39},              /* 8 */
    {10,19},    /* 9 */
    {33,42},              /* 9 */
    { 0,10},    /* 10 */
    {21,31},              /* 10 */
    {10,21},    /* 11 */
    {31,42},              /* 11 */
    {42,42}     /* alternate lag zero */
  };

  ptr->mppul = 11;
  ptr->mplgs = 12;
  ptr->mpinc = 3000;
  ptr->mplgexs = 0;

  for (i=1; i<LAG_SIZE; i++) {
    if ((lags[i][0]==42) && (lags[i][1]==42)) break;
    ptr->mplgexs++;
  }
  ptr->mplgexs++;

  if (ptr->ptab==NULL) tmp = malloc(sizeof(ptab));
  else tmp = realloc(ptr->ptab,sizeof(ptab));
  if (tmp==NULL) return -1;
  memcpy(tmp,ptab,sizeof(ptab));
  ptr->ptab = tmp;

  if (ptr->lags==NULL) tmp = malloc(sizeof(lags));
  else tmp = realloc(ptr->lags,sizeof(lags));
  if (tmp==NULL) return -1;
  memcpy(tmp,lags,sizeof(lags));
  ptr->lags = tmp;

  return 0;
}


void OpsBuildPcode(int nbaud, int mppul, int *pcode) {

  int i,n;

  int *bcode=NULL;
  int bcode1[1]={1};
  int bcode2[2]={1,-1};
  int bcode3[3]={1,1,-1};
  int bcode4[4]={1,1,-1,1};
  int bcode5[5]={1,1,1,-1,1};
  int bcode7[7]={1,1,1,-1,-1,1,-1};
  int bcode11[11]={1,1,1,-1,-1,-1,1,-1,-1,1,-1};
  int bcode13[13]={1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

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

  for (i=0; i<mppul; i++) {
    for (n=0; n<nbaud; n++) {
      pcode[i*nbaud+n] = bcode[n];
    }
  }

}

