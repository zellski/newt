# pragma once

# include "Integrator.h"

class SimpsonIntegrator : public Integrator {
    int n;
    double *weights;
    double *points;

  public:
    SimpsonIntegrator(int nn);
    virtual void integrate(
       Stage *S, const int slice, const adoublev &x,
       adoublev &c, adouble &f0);
};
