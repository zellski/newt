# Standalone unit tests for the math underpinnings of NewT. These build
# against the stub adouble in ./stub instead of HQP/ADOL-C, so they can
# run anywhere. The stub directory must precede ../include so that the
# stub adoublev.h and Stage.h shadow the real ones.

CXX = g++
CXXFLAGS = -g -O2 -Wall -Istub -I../include

TESTS = test_avec test_quadrature

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_avec: test_avec.C ../src/AVec.C ../include/AVec.h stub/adoublev.h
	$(CXX) $(CXXFLAGS) -o $@ test_avec.C ../src/AVec.C

test_quadrature: test_quadrature.C ../src/SimpsonIntegrator.C \
		../src/GaussianIntegrator.C stub/adoublev.h stub/Stage.h
	$(CXX) $(CXXFLAGS) -o $@ test_quadrature.C \
		../src/SimpsonIntegrator.C ../src/GaussianIntegrator.C

clean:
	rm -f $(TESTS) *.o *~

.PHONY: test clean
