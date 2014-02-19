.SUFFIXES: .c .C .o .so

# Change these according to your system!

ADOLC_INC = /usr/local/include/adolc
ADOLC_LIB = /usr/local/lib64
HQP_INC = /usr/local/include/hqp
HQP_LIB = /usr/local/lib

CXX = g++
CC = gcc

LD = ${CXX} -shared -o

DEPFLAGS = -MMD -MP
INCFLAGS = -I../include -I${HQP_INC} -I${ADOLC_INC}
CFLAGS = -g -O3 -fPIC -DOMU_WITH_ADOLC=1 ${DEPFLAGS} ${INCFLAGS}

LDFLAGS = -L${HQP_LIB} -L${ADOLC_LIB} -ladolc -lhqp -lomu -Wl,-install_name,@rpath/libnewt.so -W,-rpath,../src

# machine independent part

SRCS = AVec.C \
	World.C \
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

OBJS=$(SRCS:.C=.o)

libnewt.so: $(OBJS)
	$(LD) libnewt.so $(OBJS) ${LDFLAGS}

.C.o:
	$(CXX) -c $(CFLAGS) $<

clean:
	rm -f libnewt.so *.o *~

DEPS=$(SRCS:.C=.d)
-include $(DEPS)
