# pragma once

# include <assert.h>
# include <stdlib.h>
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

   adouble qVal, qDot;                  // we fill these in during SnapShot
   adouble qMomentum, qCurvature;       // this is the SnapShot end product
   adouble QVal, JVal;                  // force + impulse

   DOF(World *w, const char *S);

   void Register(Fun *const f);

   // checked lookup; a DOF must have a representation on every stage,
   // and the map's operator[] would silently insert a null instead.
   // checked unconditionally — an assert would compile out under NDEBUG
   // and leave us dereferencing an end iterator
   Fun *Rep(int ival) const {
      map<int, Fun *, less<int> >::const_iterator p = DOFReps.find(ival);
      if (p == DOFReps.end() || !(*p).second) {
         cerr << "DOF " << Name << " has no representation on stage "
              << ival << "!\n";
         abort();
      }
      return (*p).second;
   }

   void SnapShot(const adoublev &x, int ival, int slice, double t);
};
