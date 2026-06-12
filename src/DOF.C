# include "Stage.h"
# include "DOF.h"
# include "World.h"
# include "Fun.h"

DOF::DOF(World *const w, const char *const S) :
   W(w),
   Name(S),
   DOFReps()
{
   assert(!!W);
   W->Register(this);
}

void DOF::SnapShot(const adoublev &x, int ival, int slice, double t) {
   Fun *F = Rep(ival);
   F->SnapShot(x, slice, t);
   qVal = F->Val;
   qDot = F->Dot;
   QVal = JVal = 0;
}

void DOF::Register(Fun *const f) {
   cerr << "Registering function for DOF " << Name << "...\n";
   DOFReps[f->S->sIx] = f;
}
