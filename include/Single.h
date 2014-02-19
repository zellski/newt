# pragma once

# include "adolc.h"
# include "Fun.h"

class Single: public Fun {
private:
   const int xIx;
public:
   Single(Stage *const s, DOF *const d) :
      Fun(s, d),
      xIx(S->claimVars(1))
   {}
   Single(Stage *const s) :
      Fun(s),
      xIx(S->claimVars(1))
   {}

   void SnapShot(const adoublev &x, int slice, double t) {
      Val = x[xIx];
      Dot = Bis = 0;
   }
   bool isConstant() { return true; }
};
