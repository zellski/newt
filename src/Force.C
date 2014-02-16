# include "Fun.h"
# include "Force.h"
# include "AnchorPoint.h"
# include "Stage.h"

Force::Force(Stage *s, AnchorPoint *p, double sx, double sy, double mag) :
   S(s),
   P(p),
   F(0),
   vx(sx), vy(sy),
   FVec(2)
{
   FVec[0] = sx*mag; FVec[1] = sy*mag;
   S->Register(this);
}

Force::Force(Stage *s, AnchorPoint *p, double sx, double sy, Fun *const f) :
   S(s),
   P(p),
   F(f),
   vx(sx), vy(sy),
   FVec(2)
{
   S->Register(this);
}

void Force::SnapShot(const adoublev &x, int slice, double t) {
   if (F) {
      F->SnapShot(x, slice, t);
      FVal = F->Val;
      FVec[0] = vx*FVal; FVec[1] = vy*FVal;
      P->TotF += FVec;
//      cerr << "Pushing AnchorPoint " << P->Name << " in direction ("
//	   << vx << ", " << vy << ") magnitude " << v << "..\n";
   }
}
