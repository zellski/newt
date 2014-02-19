# pragma once

# include "adolc.h"

class Stage;
class DOF;
class World;
class Fun;

class Muscle {
public:
   Stage *const S;
   DOF *const D;
   Fun *const F;
   double Weight;
   adouble MVal;

   Muscle(Stage *const s, DOF *const d, Fun *const f, double Weight = 1);

   void SnapShot(const adoublev &x, int slice, double t);
};
