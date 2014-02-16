# ifndef SIMPSON_INTEGRATOR_H
# define SIMPSON_INTEGRATOR_H

# include "Integrator.h"

class SimpsonIntegrator : public Integrator {
    int n;
    double *weights;
    double *points;

  public:
    SimpsonIntegrator::SimpsonIntegrator(int nn);
    virtual void SimpsonIntegrator::integrate(Stage *S,
                                              const int slice,
                                              const adoublev &x,
                                              adoublev &c,
                                              adouble &f0);
};

# endif