# pragma once

# include "adolc.h"

class Stage;
class adoublev;
class adouble;

class Integrator {
  public:
    virtual void integrate(
       Stage *S, const int slice, const adoublev &x,
       adoublev &c, adouble &f0) = 0;
};
