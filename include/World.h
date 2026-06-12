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
   // the most recently constructed World; lets the recording layer find
   // the problem without threading it through the Omu_Program plumbing
   static World *Active;

   vector<Creature *> Creatures;
   vector<DOF *> DOFs;
   vector<Stage *> Stages;

   int xIx, cIx;        // allocation indices into Omu_VariableVec x and c

   const double G;

   World(double g);

   void Update(const adoublev &x, adoublev &c, adouble &f0);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);

   int claimVars(int n);
   int claimCons(int n);

   int Register(Creature *C);
   int Register(DOF *D);
   int Register(Stage *S);
};
