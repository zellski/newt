# pragma once

# include "AVec.h"

class Stage;
class AnchorPoint;

class Impulse {
   int xIx;
public:
   Stage *const S;
   AnchorPoint *P;
   const double vx, vy;
   AVec JVec;

   Impulse(Stage *const s, AnchorPoint *p, double sx, double sy, double mag);
   Impulse(Stage *const s, AnchorPoint *p, double sx, double sy);

   void SnapShot(const adoublev &x);
};
