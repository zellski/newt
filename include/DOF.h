# pragma once

# include <map>

# include "adolc.h"
# include "Constraints.h"
# include "Stage.h"

class World;
class Fun;

class DOF {
private:
   World *const W;

public:
   map<int, Fun *, less<int> > DOFReps;
   const char *const Name;

   adouble qVal, qDot, qBis;                // we fill these in during SnapShot
   adouble qMomentum, qCurvature;        // this is the SnapShot end product
   adouble QVal, JVal;                        // force + impulse

   DOF(World *w, const char *S);

   void Register(Fun *const f);

   void SnapShot(const adoublev &x, int ival, int slice, double t);
};
