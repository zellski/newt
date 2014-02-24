# include "Stage.h"
# include "Impulse.h"
# include "AnchorPoint.h"

Impulse::Impulse(Stage *const s, AnchorPoint *p,
                 double sx, double sy, double mag) :
   S(s),
   P(p),
   vx(sx), vy(sy),
   JVec(sx*mag, sy*mag),
   xIx(-1)
{
   S->Register(this);
}

Impulse::Impulse(Stage *const s, AnchorPoint *p, double sx, double sy) :
   S(s),
   P(p),
   vx(sx), vy(sy),
   xIx(s->claimVars(1))
{
   S->Register(this);
   cerr << "Allocated xIx " << xIx << " for ("        << vx << ", "<< vy << ") impulse\n";
}

void Impulse::SnapShot(const adoublev &x) {
   if (xIx >= 0) {
      JVec.set(x[xIx]*vx, x[xIx]*vy);
      P->TotJ += JVec;
      cerr << "Impulsing AnchorPoint " << P->Name << " in direction ("
           << vx << ", " << vy << ") magnitude " << x[xIx] << "..\n";
   }
}
