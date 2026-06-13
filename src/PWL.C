# include "Stage.h"
# include "adolc.h"
# include "PWL.h"
# include "World.h"
# include "DOF.h"

PWL::PWL(Stage *const s, const std::string &label) :
   Fun(s),
   xIx(S->claimVars(2*S->N, label))
{}

void PWL::SnapShot(const adoublev &x, int slice, double t) {
   Val = (1-t)*x[xIx+2*slice] + t*x[xIx+2*slice+1] + 0.0;
   Dot = Bis = 0;
}
