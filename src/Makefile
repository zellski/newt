# This Makefile is basically stolen from the HQP/Omuses package.

.SUFFIXES: .c .C .o .so

# Change this according to where Omuses is installed.

OMU_INCDIR = /usr/local/include/omuses

# Change these according to your system!

CXX = g++
CC = gcc
LD = gcc -shared -o

CFLAGS = -g -O2 -fPIC -I../include -I${OMU_INCDIR}

#
# machine independent part
#

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
	RigidBody.C \
	AnchorPoint.C \
	Visualizer.C

OBJS = $(SRCS:.C=.o)

RESULT = libnewt.so

lib: $(OBJS) Makefile
	$(LD) $(RESULT) $(OBJS)

.C.o:
	$(CXX) -c $(CFLAGS) $<

.c.o:
	$(CC) -c $(CFLAGS) $<

clean:
	rm -f $(RESULT) *.o *~


AnchorPoint.o: AnchorPoint.C ../include/Stage.h \
 ../include/AnchorPoint.h ../include/BodyPoint.h \
 ../include/Constraints.h
Constraints.o: Constraints.C ../include/Stage.h ../include/DOF.h \
 ../include/Constraints.h ../include/AnchorPoint.h \
 ../include/RigidBody.h
Creature.o: Creature.C ../include/Creature.h ../include/AnchorPoint.h \
 ../include/World.h ../include/DOF.h ../include/Constraints.h \
 ../include/Stage.h ../include/RigidBody.h ../include/BodyPoint.h
DOF.o: DOF.C ../include/DOF.h ../include/Constraints.h \
 ../include/Stage.h ../include/World.h ../include/Fun.h
Force.o: Force.C ../include/Fun.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/Force.h \
 ../include/AnchorPoint.h
Hat.o: Hat.C ../include/Hat.h ../include/Fun.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/World.h
Hermite.o: Hermite.C ../include/Hermite.h ../include/Fun.h \
 ../include/DOF.h ../include/Constraints.h ../include/Stage.h \
 ../include/World.h
Impulse.o: Impulse.C ../include/Impulse.h ../include/AnchorPoint.h \
 ../include/Stage.h
Link.o: Link.C ../include/Link.h ../include/Constraints.h \
 ../include/Creature.h ../include/AnchorPoint.h ../include/Impulse.h \
 ../include/World.h ../include/Stage.h ../include/DOF.h \
 ../include/Fun.h
Muscle.o: Muscle.C ../include/Fun.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/Muscle.h
PWC.o: PWC.C ../include/PWC.h ../include/Fun.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/World.h
PWL.o: PWL.C ../include/PWL.h ../include/Fun.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/World.h
Primitives.o: Primitives.C ../include/Primitives.h \
 ../include/RigidBody.h ../include/BodyPoint.h \
 ../include/AnchorPoint.h ../include/DOF.h ../include/Constraints.h \
 ../include/Stage.h ../include/World.h
RigidBody.o: RigidBody.C ../include/RigidBody.h ../include/BodyPoint.h \
 ../include/AnchorPoint.h ../include/Impulse.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/World.h
Stage.o: Stage.C ../include/Stage.h ../include/World.h \
 ../include/Constraints.h ../include/Creature.h \
 ../include/AnchorPoint.h ../include/Impulse.h ../include/Force.h \
 ../include/Muscle.h ../include/DOF.h ../include/Fun.h
Visualizer.o: Visualizer.C ../include/Visualizer.h ../include/DOF.h \
 ../include/Constraints.h ../include/Stage.h ../include/World.h \
 ../include/Creature.h ../include/AnchorPoint.h ../include/BodyPoint.h \
 ../include/Primitives.h ../include/RigidBody.h
World.o: World.C ../include/World.h ../include/Stage.h \
 ../include/DOF.h ../include/Constraints.h ../include/Creature.h \
 ../include/AnchorPoint.h ../include/Fun.h ../include/Visualizer.h
