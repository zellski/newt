# Standalone unit tests for the math underpinnings of NewT. These build
# against the stub adouble in ./stub instead of HQP/ADOL-C, so they can
# run anywhere. The stub directory must precede ../include so that the
# stub adoublev.h and Stage.h shadow the real ones.

CXX = g++
CXXFLAGS = -g -O2 -Wall -Istub -I../include

TESTS = test_avec test_quadrature test_dynamics test_specparser

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_avec: test_avec.C ../src/AVec.C ../include/AVec.h stub/adoublev.h
	$(CXX) $(CXXFLAGS) -o $@ test_avec.C ../src/AVec.C

test_quadrature: test_quadrature.C ../src/SimpsonIntegrator.C \
		../src/GaussianIntegrator.C stub/adoublev.h stub/Stage.h
	$(CXX) $(CXXFLAGS) -o $@ test_quadrature.C \
		../src/SimpsonIntegrator.C ../src/GaussianIntegrator.C

DYN_SRCS = ../src/AVec.C ../src/AnchorPoint.C ../src/RigidBody.C \
		../src/Force.C ../src/Impulse.C

test_dynamics: test_dynamics.C $(DYN_SRCS) stub/adoublev.h stub/Stage.h \
		stub/World.h stub/Omu_Variables.h
	$(CXX) $(CXXFLAGS) -o $@ test_dynamics.C $(DYN_SRCS)

# the spec parser is solver-free std C++; no stubs involved
test_specparser: test_specparser.C ../src/SpecParser.C \
		../include/newt/Spec.h ../include/newt/SpecParser.h
	$(CXX) -g -O2 -Wall -I../include -I../include/vendor -o $@ \
		test_specparser.C ../src/SpecParser.C

clean:
	rm -f $(TESTS) *.o *~

.PHONY: test clean
