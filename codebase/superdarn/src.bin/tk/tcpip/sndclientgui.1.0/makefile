# Makefile for sndclientgui
# ============================
# Author: E.G.Thomas
# by E.G.Thomas
#
#
include $(MAKECFG).$(SYSTEM)
INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn
OBJS = sndclientgui.o
SRC=hlpstr.h errstr.h sndclientgui.c

LIBS=-lfitcnx.1 -lfit.1 -lcfit.1 -lrscan.1 -lradar.1 -lcnx.1 \
     -ldmap.1 -lopt.1 -lrtime.1 -lbinplot.1 -lrcnv.1 -lncurses
ifneq ($(OSTYPE),darwin)
LIBS += -ltinfo
endif
SLIB=-lm -lz
DSTPATH = $(BINPATH)
OUTPUT = sndclientgui

include $(MAKEBIN).$(SYSTEM)
