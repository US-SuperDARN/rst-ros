/* setup.h
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

#ifndef _SETUP_H
#define _SETUP_H

#define START_STRING "PROGRAM START->"

int OpsSetupCommand(int argc,char *argv[]);
int OpsStart(char *ststr);
int OpsFitACFStart();
void OpsLogStart(int sock,char *name,int argc,char *argv[]);
void OpsSetupTask(int tnum,struct TCPIPMsgHost *task,int sock,char *name);
void OpsSetupShell();
void OpsSetupIQBuf(int intsc,int intus,int mppul,int mpinc,int nbaud);

#endif
