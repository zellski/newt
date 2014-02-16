# This Makefile is basically stolen from the HQP/Omuses package.

.SUFFIXES: .c .C .o .so

# Change this to point to include directories for HQP, Omuses, AdolC, etc

NEWT_INCLUDES = -I/usr/local/include/hqp -I/usr/local/include/hqp/adolc

# Change these according to your system!

CXX = g++
CC = gcc

LD = gcc -shared -o

CFLAGS = -g -O3 -march=pentium4 -fPIC -I../include ${NEWT_INCLUDES}

# machine independent part

SRCS = World.C \
	Impulse.C \
	Force.C \
	Stage.C \
	Muscle.C \
	Constraints.C \
	Primitives.C \
	DOF.C \
	Creature.C \
	PWL.C \
	Hat.C \
	Link.C \
	Hermite.C \
	Hermlet.C \
	RigidBody.C \
	AnchorPoint.C \
	SimpsonIntegrator.C \
	GaussianIntegrator.C \
	RIBVisualizer.C
#	VRMLVisualizer.C

OBJS = $(SRCS:.C=.o)

libnewt.so: $(OBJS) Makefile
	$(LD) libnewt.so $(OBJS)

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
