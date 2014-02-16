# include "Impulse.h"
# include "AnchorPoint.h"
# include "Stage.h"

Impulse::Impulse(Stage *const s, AnchorPoint *p,
		 double sx, double sy, double mag) :
   S(s),
   P(p),
   vx(sx), vy(sy),
   JVec(2),
   xIx(-1)
{
   S->Register(this);
   JVec[0] = sx*mag; JVec[1] = sy*mag;
}

Impulse::Impulse(Stage *const s, AnchorPoint *p, double sx, double sy) :
   S(s),
   P(p),
   vx(sx), vy(sy),
   JVec(2),
   xIx(s->claimVars(1))
{
   S->Register(this);
   cerr << "Allocated xIx " << xIx << " for ("
	<< vx << ", "<< vy << ") impulse\n";
}

void Impulse::SnapShot(const adoublev &x) {
   if (xIx >= 0) {
      JVec[0] = x[xIx] * vx;
      JVec[1] = x[xIx] * vy;
      P->TotJ += JVec;
      cerr << "Impulsing AnchorPoint " << P->Name << " in direction ("
	   << vx << ", " << vy << ") magnitude " << x[xIx] << "..\n";
   }
}
