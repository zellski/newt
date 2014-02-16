# ifndef INTEGRATOR_H
# define INTEGRATOR_H

class Stage;
class adoublev;
class adouble;

class Integrator {
  public:
    virtual void Integrator::integrate(Stage *S,
                                       const int slice,
                                       const adoublev &x,
                                       adoublev &c,
                                       adouble &f0) = 0;
};

# endif
