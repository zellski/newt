# include "Stage.h"
# include "Fun.h"
# include "Muscle.h"
# include "DOF.h"

Muscle::Muscle(Stage *const s, DOF *const d, Fun *const f, double weight) :
   S(s), D(d), F(f),
   Weight(weight)
{
   assert(F);
   S->Register(this);
}

void Muscle::SnapShot(const adoublev &x, int slice, double t) {
   F->SnapShot(x, slice, t);
   MVal = F->Val;
   D->QVal += MVal;
}
