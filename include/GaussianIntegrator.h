# pragma once

# include "Integrator.h"

class GaussianIntegrator : public Integrator {
    int n;
    double *weights;
    double *points;

  public:
    GaussianIntegrator::GaussianIntegrator(int nn);
    virtual void GaussianIntegrator::integrate(Stage *S,
                                              const int slice,
                                              const adoublev &x,
                                              adoublev &c,
                                              adouble &f0);
};
