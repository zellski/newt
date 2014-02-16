# ifndef BSPLINE_H
# define BSPLINE_H

# include <Omu_Variables.h>
# include <adouble.h>
# include "Fun.h"

/*
**	This is a basis of cubic piecewise polynomials, of width and
**	depth two; largely uncoupled and with good control over value
**	and first-derivate interpolation; "BSpline Cubics"
*/

class Stage;

class BSpline: public Fun, public Constraint {
private:
   static const double ScaleCoef[2][2][4];
protected:
   const int xIx;
   const int cIx;
   const int eIx;
   double sVal(int width, int depth, double t) const;
   double sDot(int width, int depth, double t) const;
   double sBis(int width, int depth, double t) const;
public:
   BSpline(Stage *const s, double from=0, double to=0);
   BSpline(Stage *const s, double from, double to, double min, double max);
   BSpline(Stage *const s, DOF *const D, double from=0, double to=0);
   BSpline(Stage *const s, DOF *const D, double from, double to,
	   double min, double max);

   bool isConstant() { return false; }

   void SnapShot(const adoublev &x, int slice, double t);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
   void Evaluate(const adoublev &x, adoublev &c);

   void IntegrateFEM(adoublev &c, adouble lhs, adouble rhs,
		     int slice, double t, double weight) const;
};

#endif
