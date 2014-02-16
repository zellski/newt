# include <adouble.h>
# include "PWC.h"
# include "World.h"
# include "DOF.h"
# include "Stage.h"

PWC::PWC(Stage *const s) :
   Fun(s),
   xIx(S->claimVars(S->N))
{}

adouble PWC::Val(const adoublev &x, int slice, double t) const {
   return x[xIx+slice] + 0.0;
}
