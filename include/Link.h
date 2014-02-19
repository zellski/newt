# pragma once

# include <map>

# include <Omu_Variables.h>

# include "adolc.h"
# include "Constraints.h"

class Stage;

class Link: public Constraint {
private:
   Stage *const A;
   Stage *const B;
   int cIx;
public:
   Link(Stage *const a, Stage *const b);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {}
   void Evaluate(const adoublev &x, adoublev &c);
};
