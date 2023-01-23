/* siteglobal.h
   ============ 
   Author: R.J.Barnes
*/
   

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
 
 
 
*/

#ifndef _SITEGLOBAL_H
#define _SITEGLOBAL_H

extern int num_transmitters;
/*extern struct timeval tock;*/
extern struct ControlPRM rprm;
extern struct RosData rdata;
extern struct DataPRM dprm;
extern struct TRTimes badtrdat;
extern struct TXStatus txstatus;
extern struct SiteLibrary sitelib;
extern int cancel_count;
extern int dmatch;

extern struct TCPIPMsgHost ros;
extern struct TCPIPMsgHost errlog;
extern struct TCPIPMsgHost shell;
extern struct TCPIPMsgHost task[4];
extern int baseport;

#endif
