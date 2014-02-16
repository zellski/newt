# ifndef CONSTANT_H
# define CONSTANT_H

# include <adouble.h>
# include "Fun.h"

class Constant: public Fun {
public:
   Constant(Stage *const s, DOF *const d, double v) : Fun(s,d) {
      Val = v;
   }
   Constant(Stage *const s, double v) : Fun(s) {
      Val = v;
   }

   bool isConstant() { return true; }

   void SnapShot(const adoublev &x, int slice, double t) {}
   void IntegrateFEM(adoublev &c, adouble lhs, adouble rhs,
		     int slice, double t, double weight) const {};
};

#endif
