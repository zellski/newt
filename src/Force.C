# include "Stage.h"
# include "Fun.h"
# include "Force.h"
# include "AnchorPoint.h"

Force::Force(Stage *s, AnchorPoint *p, double sx, double sy, double mag) :
   S(s),
   P(p),
   vx(sx), vy(sy),
   F(0),
   FVec(sx*mag, sy*mag)
{
   S->Register(this);
}

Force::Force(Stage *s, AnchorPoint *p, double sx, double sy, Fun *const f) :
   S(s),
   P(p),
   vx(sx), vy(sy),
   F(f)
{
   S->Register(this);
}

void Force::SnapShot(const adoublev &x, int slice, double t) {
   if (F) {
      F->SnapShot(x, slice, t);
      FVal = F->Val;
      FVec.set(vx*FVal, vy*FVal);
   }
   // constant-magnitude forces keep the FVec computed at construction
   P->TotF += FVec;
}
