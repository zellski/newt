# This Makefile is basically stolen from the HQP/Omuses package.

.SUFFIXES: .c .C .o .so

# objects to link with to create an executable
#MACH_OBJS = -ltcl8.4 -ldl -lm
MACH_OBJS = -framework Tcl

# Change this to point to include directories for HQP, Omuses, AdolC, etc

ADOLC_INC = /usr/local/include/adolc
ADOLC_LIB = /usr/local/lib64
HQP_INC = /usr/local/include/hqp
HQP_LIB = /usr/local/lib

# Change these according to your system!

CXX = g++
CC = gcc

DEPFLAGS = -MMD -MP
INCFLAGS = -I../include -I${HQP_INC} -I${ADOLC_INC}
CFLAGS = -g -W\#warnings -O3 -fPIC -DOMU_WITH_ADOLC=1 ${DEPFLAGS} ${INCFLAGS}

LDFLAGS = -L../src -Wl,-rpath,../src -Wl,-rpath,${HQP_LIB} -Wl,-rpath,${ADOLC_LIB} -L${HQP_LIB} -L${ADOLC_LIB} -ladolc -lhqp -lomu

#
# machine independent part
#

SRCS = 	Odc_Init.C \
	Needle.C \
	Luxo.C \
	Human.C \
	Snake.C \
	Trivial.C

OBJS = $(SRCS:.C=.o) Odc_Main.o

RESULT = odc

odc:	always $(OBJS)
	$(CXX) -o $(RESULT) $(OBJS) \
	$(LDFLAGS) -lomu -lhqp -lnewt $(MACH_OBJS)

always:
	cd ../src; make

.C.o:
	$(CXX) -c $(CFLAGS) $<

.c.o:
	$(CC) -c $(CFLAGS) $<

clean:
	rm -f $(RESULT) *.o *~

DEPS = $(SRCS:.C=.d) Odc_Main.d
-include ${DEPS}
