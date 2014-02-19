# pragma once

# include <vector>
# include <Omu_Variables.h>
# include "adolc.h"

class Integrator;
class World;
class Muscle;
class Constraint;
class Impulse;
class Fun;
class Force;

class Stage {
public:
   World *const W;

   vector<Muscle *> Muscles;
   vector<Force *> Forces;
   vector<Fun *> Funs;
   vector<Impulse *> Impulses;
   vector<Constraint *> Cons;

   Integrator *integrator;

   const double Min, Max, Start;

   double speedContribution, torqueContribution;

   const int N;

   const int sIx;
   const int tIx;

   adouble T;
   adouble h;

   Stage(World *w, Integrator *i, int n, double t);
   Stage(World *w, Integrator *i, int n, double min, double max, double start);

   int Register(Muscle *M);
   int Register(Force *F);
   int Register(Fun *F);
   int Register(Impulse *I);
   int Register(Constraint *C);

   int claimVars(int n);
   int claimCons(int n);

   void Update(const adoublev &x, adoublev &c, adouble &f0);
   void SnapShot(const adoublev &x, int slice, double t);
   void FEMEquations(const adoublev &x, adoublev &c, adouble &f0);
   void FEMPoint(const int slice, const double t, const double weight,
                 const adoublev &x, adoublev &c, adouble &f0);

   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
};
