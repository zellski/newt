# pragma once

# include <vector>
# include <Omu_Variables.h>
# include "adolc.h"

class DOF;
class Constraint;
class Muscle;
class Stage;
class Creature;

class World {
public:
   vector<Creature *> Creatures;
   vector<DOF *> DOFs;
   vector<Stage *> Stages;

   int xIx, cIx;	// allocation indices into Omu_VariableVec x and c

   const double G;

   const bool ImplicitMuscles;

   World(double g, bool implicit = false);

   void Update(const adoublev &x, adoublev &c, adouble &f0);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);

   int claimVars(int n);
   int claimCons(int n);

   int Register(Creature *C);
   int Register(DOF *D);
   int Register(Stage *S);
};
