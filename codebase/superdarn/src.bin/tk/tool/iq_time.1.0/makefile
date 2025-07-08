# Makefile for iq_time
# ====================
# Author: E.G.Thomas
#
include $(MAKECFG).$(SYSTEM)

INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn
OBJS = iq_time.o
SRC=hlpstr.h errstr.h iq_time.c
DSTPATH = $(BINPATH)
OUTPUT = iq_time
LIBS=-liqdata.1 -lopt.1 -ldmap.1 -lradar.1 -lrtime.1 -lrcnv.1 
SLIB=-lm -lz
 

include $(MAKEBIN).$(SYSTEM)
