# ifndef IMPULSE_H
# define IMPULSE_H

# include <adouble.h>

class Stage;
class AnchorPoint;

class Impulse {
   int xIx;
public:
   Stage *const S;
   AnchorPoint *P;
   const double vx, vy;
   adoublev JVec;

   Impulse(Stage *const s, AnchorPoint *p, double sx, double sy, double mag);
   Impulse(Stage *const s, AnchorPoint *p, double sx, double sy);

   void SnapShot(const adoublev &x);
};
# endif
