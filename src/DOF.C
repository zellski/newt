# include "DOF.h"
# include "World.h"
# include "Fun.h"
# include "Stage.h"

DOF::DOF(World *const w, const char *const S) :
   W(w),
   Name(S),
   DOFReps()
{
   assert(!!W);
   W->Register(this);
}

void DOF::SnapShot(const adoublev &x, int ival, int slice, double t) {
   DOFReps[ival]->SnapShot(x, slice, t);
   qVal = DOFReps[ival]->Val;
   qDot = DOFReps[ival]->Dot;
   qBis = DOFReps[ival]->Bis;
//   cerr << "SnapShot: [ " << qVal << ", " << qDot << ", " << qBis << " ]\n";
   QVal = JVal = 0;
}

void DOF::Register(Fun *const f) {
   cerr << "Registering function for DOF " << Name << "...\n";
   DOFReps[f->S->sIx] = f;
}
