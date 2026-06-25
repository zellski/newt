# This Makefile is basically stolen from the HQP/Omuses package.

.SUFFIXES: .c .C .o .so

# Dependency locations, Tcl linkage and include paths. Override any of these on
# the make command line. Defaults are selected per platform (macOS/Apple clang
# vs Linux/GNU).

UNAME := $(shell uname)
ifeq ($(UNAME),Darwin)
  ADOLC_INC ?= /usr/local/include/adolc
  ADOLC_LIB ?= /usr/local/lib64
  HQP_INC   ?= /usr/local/include/hqp
  HQP_LIB   ?= /usr/local/lib
  TCL_INC   ?= /opt/homebrew/opt/tcl-tk@8/include/tcl-tk
  MACH_OBJS ?= -framework Tcl
else
  ADOLC_INC ?= /opt/newtdeps/include
  ADOLC_LIB ?= /opt/newtdeps/lib64
  HQP_INC   ?= /opt/newtdeps/include/hqp
  HQP_LIB   ?= /opt/newtdeps/lib
  TCL_INC   ?= /usr/include/tcl8.6
  MACH_OBJS ?= -ltcl8.6 -ldl -lm
endif

CXX = g++
CC = gcc

DEPFLAGS = -MMD -MP
INCFLAGS = -I../include -I../include/vendor -I${HQP_INC} -I${ADOLC_INC} -I${TCL_INC}
# See ../src/Makefile for why the standard headers are force-included first.
CFLAGS = -g -O3 -fPIC -DOMU_WITH_ADOLC=1 -include ../stdprelude.h ${DEPFLAGS} ${INCFLAGS}

LDFLAGS = -L../src -Wl,-rpath,../src -Wl,-rpath,${HQP_LIB} -Wl,-rpath,${ADOLC_LIB} -L${HQP_LIB} -L${ADOLC_LIB} -ladolc -lhqp -lomu

#
# machine independent part
#

SRCS = 	Odc_Init.C \
	Newt_Cmds.C \
	Scenario.C

OBJS = $(SRCS:.C=.o) Odc_Main.o

RESULT = odc

odc:	always $(OBJS)
	$(CXX) -o $(RESULT) $(OBJS) \
	$(LDFLAGS) -lomu -lhqp -lnewt $(MACH_OBJS)

always:
	cd ../src; $(MAKE)

.C.o:
	$(CXX) -c $(CFLAGS) $<

.c.o:
	$(CC) -c $(CFLAGS) $<

clean:
	rm -f $(RESULT) *.o *~

DEPS = $(SRCS:.C=.d) Odc_Main.d
-include ${DEPS}
