# include "Stage.h"
# include "Fun.h"
# include "Force.h"
# include "AnchorPoint.h"

Force::Force(Stage *s, AnchorPoint *p, double sx, double sy, double mag) :
   S(s),
   P(p),
   F(0),
   vx(sx), vy(sy),
   FVec(sx*mag, sy*mag)
{
   S->Register(this);
}

Force::Force(Stage *s, AnchorPoint *p, double sx, double sy, Fun *const f) :
   S(s),
   P(p),
   F(f),
   vx(sx), vy(sy)
{
   S->Register(this);
}

void Force::SnapShot(const adoublev &x, int slice, double t) {
   if (F) {
      F->SnapShot(x, slice, t);
      FVal = F->Val;
      FVec.set(vx*FVal, vy*FVal);
      P->TotF += FVec;
//      cerr << "Pushing AnchorPoint " << P->Name << " in direction ("
//	   << vx << ", " << vy << ") magnitude " << v << "..\n";
   }
}
