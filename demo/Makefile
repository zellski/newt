# This Makefile is basically stolen from the HQP/Omuses package.

.SUFFIXES: .c .C .o .so

# objects to link with to create an executable
MACH_OBJS = -ltcl8.0 -ldl -lm

# Change these according to where Omuses is installed.

OMU_INCDIR = /usr/local/include/omuses
OMU_LIBDIR = /usr/local/lib

# Change these according to your system!

CXX = g++
CC = gcc

CFLAGS = -g -O2 -I../include -I${OMU_INCDIR}
LDFLAGS = -L../src -L${OMU_LIBDIR}

#
# machine independent part
#

SRCS = 	Odc_Init.C \
	Luxo.C \
	Snake.C \
	Needle.C \

OBJS = $(SRCS:.C=.o) Odc_Main.o

RESULT = odc

all: $(OBJS) Makefile
	$(CXX) -o $(RESULT) $(LDFLAGS) $(OBJS) \
	-lomu -lhqp -lnewt $(MACH_OBJS)

.C.o:
	$(CXX) -c $(CFLAGS) $<

.c.o:
	$(CC) -c $(CFLAGS) $<

clean:
	rm -f $(RESULT) *.o *~

$(OBJS): ../include/World.h ../include/Creature.h \
 ../include/AnchorPoint.h ../include/Primitives.h \
 ../include/RigidBody.h ../include/BodyPoint.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/Link.h \
 ../include/Muscle.h ../include/Constant.h ../include/Fun.h \
 ../include/Impulse.h ../include/Hat.h \
 ../include/Hermite.h ../include/PWL.h ../include/PWC.h
