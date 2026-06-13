# pragma once

# include <string>
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

   // one human-readable label per claimed row, parallel to x and c;
   // legacy unlabeled claims read "?"
   std::vector<std::string> xLabels, cLabels;

   // the constraint bounds as set by Initialize, snapshotted for
   // residual reporting (Omu owns the live vectors)
   std::vector<double> cMins, cMaxs;

   const double G;

   World(double g);

   void Update(const adoublev &x, adoublev &c, adouble &f0);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);

   int claimVars(int n, const std::string &label = "?");
   int claimCons(int n, const std::string &label = "?");

   // refine a single row of a block claim after the fact
   void LabelVar(int ix, const std::string &label);
   void LabelCon(int ix, const std::string &label);

   int Register(Creature *C);
   int Register(DOF *D);
   int Register(Stage *S);
};
