# pragma once

# include <Omu_Program.h>

class World;

// the generic program: builds its World from the YAML scenario whose
// path arrived via the newt_scenario Tcl command
class Scenario: public Omu_Program {
private:
   World *W;
public:
   void setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c);
   void update(int kk,
               const adoublev &x, const adoublev &u,
               adoublev &f, adouble &f0, adoublev &c);
   const char *name() { return "Scenario"; }
};
