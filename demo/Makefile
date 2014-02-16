# This Makefile is basically stolen from the HQP/Omuses package.

.SUFFIXES: .c .C .o .so

# objects to link with to create an executable
MACH_OBJS = -ltcl8.4 -ldl -lm

# Change this to point to include directories for HQP, Omuses, AdolC, etc

ADOLC_BASE = /usr/local
HQP_BASE = /usr/local

# Change these according to your system!

CXX = g++
CC = gcc

CFLAGS = -g -DOMU_WITH_ADOLC=1 -I../include -I${HQP_BASE}/include/hqp -I${ADOLC_BASE}/include/adolc
LDFLAGS = -L../src -Wl,-R../src -L${HQP_BASE}/lib

#
# machine independent part
#

SRCS = 	Odc_Init.C \
	Needle.C \
	Luxo.C \
	Human.C \
#	Snake.C \
#	Trivial.C \

OBJS = $(SRCS:.C=.o) Odc_Main.o

RESULT = odc

odc:	always $(OBJS) Makefile
	$(CXX) -o $(RESULT) $(LDFLAGS) $(OBJS) \
	-lomu -lhqp -lnewt $(MACH_OBJS)

always:
	cd ../src; make

depend:
	$(CXX) -MM -MG -MP -I../include $(SRCS) > .depend

.C.o:
	$(CXX) -c $(CFLAGS) $<

.c.o:
	$(CC) -c $(CFLAGS) $<

clean:
	rm -f $(RESULT) *.o *~

include .depend
# DO NOT DELETE
