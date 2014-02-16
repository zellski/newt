# ifndef HAT_H
# define HAT_H

# include <Omu_Variables.h>
# include <adouble.h>
# include "Fun.h"

/*
**	Hats! Piecewise linear.
*/

class Stage;

class Hat: public Fun {
protected:
   const int xIx;
   const int eIx;
   double sVal(int width, int depth, double t) const;
   double sDot(int width, int depth, double t) const;
public:
   Hat(Stage *const s);
   Hat(Stage *const s, DOF *const D, double from=0, double to=0,
       double min=-Inf, double max=Inf);

   void SnapShot(const adoublev &x, int slice, double t);

   bool isConstant() { return false; }

   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);

   void IntegrateFEM(adoublev &c, adouble lhs, adouble rhs,
		     int slice, double t, double weight) const;
};

#endif
