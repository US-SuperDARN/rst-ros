/* site.h
   ======
   Author: R.J.Barnes
*/


#ifndef _SITE_H
#define _SITE_H

struct SiteLibrary {
  int (*start)(char *,char *);
  int (*setupradar)();
  int (*startscan)();
  int (*startintt)(int,int);
  int (*fclr)(int,int);
  int (*tmseq)(int *);
  int (*integrate)(int (*lags)[2]);
  int (*wideintegrate)(int (*lags)[2],float *,float *,float *);
  int (*endscan)(int,int, unsigned);
  void (*exit)(int);
};

int SiteStart(char *host,char *ststr);
int SiteSetupRadar();
int SiteStartScan(int32_t periods_per_scan, int32_t *scan_beam_list,
                  int32_t *clrfreq_fstart_list, int32_t *clrfreq_bandwidth_list,
                  int32_t fixFreq, int32_t sync_scan, int32_t *beam_times,
                  int32_t scn_sc, int32_t scn_us, int32_t int_sc, int32_t int_us,
                  int32_t start_period);
int SiteStartIntt(int intsc,int intus);
int SiteFCLR(int stfreq,int edfreq);
int SiteTimeSeq(int *ptab);
int SiteIntegrate(int (*lags)[2]);
int SiteWideIntegrate(int (*lags)[2],float *pwr0_arr,float *acfd_arr,float *xcfd_arr);
int SiteEndScan(int bndsc,int bndus, unsigned sleepus);
void SiteExit(int signo);


#endif

