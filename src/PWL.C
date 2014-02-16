# include <adouble.h>
# include "PWL.h"
# include "World.h"
# include "DOF.h"
# include "Stage.h"

PWL::PWL(Stage *const s) :
   Fun(s),
   xIx(S->claimVars(2*S->N))
{}

void PWL::SnapShot(const adoublev &x, int slice, double t) {
   Val = (1-t)*x[xIx+2*slice] + t*x[xIx+2*slice+1] + 0.0;
   Dot = Bis = 0;
}
