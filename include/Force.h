# ifndef FORCE_H
# define FORCE_H

# include <adouble.h>

class Stage;
class AnchorPoint;
class Fun;

class Force {
   Stage *const S;
   AnchorPoint *P;
   const double vx, vy;
   Fun *const F;
public:
   adouble  FVal;
   adoublev FVec;

   Force(Stage *s, AnchorPoint *p, double sx, double sy, double mag);
   Force(Stage *s, AnchorPoint *p, double sx, double sy, Fun *const f);
   void SnapShot(const adoublev &x, int slice, double t);
};

# endif
