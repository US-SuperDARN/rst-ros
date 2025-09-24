/* site.ros.h
   ==========
   Author: R.J.Barnes
*/


#ifndef _SITEROS_H
#define _SITEROS_H

int SiteRosStart(char *host,char *ststr);
int SiteRosSetupRadar();
int SiteRosStartScan(int32_t periods_per_scan, int32_t *scan_beam_list,
                     int32_t *clrfreq_fstart_list, int32_t *clrfreq_bandwidth_list,
                     int32_t fixFreq, int32_t sync_scan, int32_t *beam_times,
                     int32_t scn_sc, int32_t scn_us, int32_t int_sc, int32_t int_us,
                     int32_t start_period);
int SiteRosStartIntt(int intsc,int intus);
int SiteRosFCLR(int stfreq,int edfreq);
int SiteRosTimeSeq(int *ptab);
int SiteRosIntegrate(int (*lags)[2]);
int SiteRosWideIntegrate(int (*lags)[2], float *pwr0_arr, float *acfd_arr, float *xcfd_arr);
int SiteRosEndScan(int bsc,int bus, unsigned sleepus);
void SiteRosExit(int signum);

#endif

