# pragma once

# include "AVec.h"

class Stage;
class AnchorPoint;
class Fun;

class Force {
   Stage *const S;
   AnchorPoint *P;
   const double vx, vy;
   Fun *const F;
public:
   adouble FVal;
   AVec FVec;

   Force(Stage *s, AnchorPoint *p, double sx, double sy, double mag);
   Force(Stage *s, AnchorPoint *p, double sx, double sy, Fun *const f);
   void SnapShot(const adoublev &x, int slice, double t);
};
