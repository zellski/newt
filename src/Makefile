.SUFFIXES: .c .C .o .so

# Dependency locations and platform link conventions. Override any of these on
# the make command line, e.g. make HQP_INC=$HOME/repos/local/include/hqp ...
# Defaults are selected per platform (macOS/Apple clang vs Linux/GNU).

UNAME := $(shell uname)
ifeq ($(UNAME),Darwin)
  ADOLC_INC ?= /usr/local/include/adolc
  ADOLC_LIB ?= /usr/local/lib64
  HQP_INC   ?= /usr/local/include/hqp
  HQP_LIB   ?= /usr/local/lib
  TCL_INC   ?= /opt/homebrew/opt/tcl-tk@8/include/tcl-tk
  SONAME     = -Wl,-install_name,@rpath/libnewt.so
else
  ADOLC_INC ?= /opt/newtdeps/include
  ADOLC_LIB ?= /opt/newtdeps/lib64
  HQP_INC   ?= /opt/newtdeps/include/hqp
  HQP_LIB   ?= /opt/newtdeps/lib
  TCL_INC   ?= /usr/include/tcl8.6
  SONAME     = -Wl,-soname,libnewt.so
endif

CXX = g++
CC = gcc

LD = ${CXX} -shared -o

DEPFLAGS = -MMD -MP
INCFLAGS = -I../include -I../include/vendor -I${HQP_INC} -I${ADOLC_INC} -I${TCL_INC}
# -include ../stdprelude.h pulls in the libstdc++ headers before HQP's
# Meschach.h, whose unparenthesised min/max macros otherwise corrupt them on
# GNU libstdc++ (it is a harmless no-op under Apple libc++).
CFLAGS = -g -O3 -fPIC -DOMU_WITH_ADOLC=1 -include ../stdprelude.h ${DEPFLAGS} ${INCFLAGS}

LDFLAGS = -L${HQP_LIB} -L${ADOLC_LIB} -ladolc -lhqp -lomu -lsqlite3 ${SONAME} -Wl,-rpath,../src

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
	SpecParser.C \
	SpecBuilder.C \
	Census.C \
	Residuals.C \
	Recorder.C \
	Sweep.C
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
