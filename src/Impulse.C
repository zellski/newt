# include "Stage.h"
# include "Impulse.h"
# include "AnchorPoint.h"

Impulse::Impulse(Stage *const s, AnchorPoint *p,
                 double sx, double sy, double mag) :
   xIx(-1),
   S(s),
   P(p),
   vx(sx), vy(sy),
   JVec(sx*mag, sy*mag)
{
   S->Register(this);
}

Impulse::Impulse(Stage *const s, AnchorPoint *p, double sx, double sy,
                 const std::string &label) :
   xIx(s->claimVars(1, label)),
   S(s),
   P(p),
   vx(sx), vy(sy)
{
   S->Register(this);
   cerr << "Allocated xIx " << xIx << " for ("        << vx << ", "<< vy << ") impulse\n";
}

void Impulse::SnapShot(const adoublev &x) {
   if (xIx >= 0) {
      JVec.set(x[xIx]*vx, x[xIx]*vy);
   }
   // fixed-magnitude impulses keep the JVec computed at construction
   P->TotJ += JVec;
}
