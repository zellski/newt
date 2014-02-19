# pragma once

# include <Omu_Variables.h>
# include "adolc.h"
# include "Fun.h"

/*
**	This is a basis of cubic piecewise polynomials, of width and
**	depth two; largely uncoupled and with good control over value
**	and first-derivate interpolation; "Hermite Cubics"
*/

class Stage;

class Hermlet: public Fun, public Constraint {
private:
   static const double ScaleCoef[2][2][4];
protected:
   const int xIx;
   const int cIx;
   const int eIx;
   double s1Val(double t) const;
   double s1Dot(double t) const;
   double s1Bis(double t) const;
   double s2Val(double t) const;
   double s2Dot(double t) const;
   double s2Bis(double t) const;
   double w1Val(double t) const;
   double w1Dot(double t) const;
   double w1Bis(double t) const;
   double w2Val(double t) const;
   double w2Dot(double t) const;
   double w2Bis(double t) const;
public:
   Hermlet(Stage *const s, double from=0, double to=0);
   Hermlet(Stage *const s, double from, double to, double min, double max);
   Hermlet(Stage *const s, DOF *const D, double from=0, double to=0);
   Hermlet(Stage *const s, DOF *const D, double from, double to,
	   double min, double max);

   bool isConstant() { return false; }

   void SnapShot(const adoublev &x, int slice, double t);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
   void Evaluate(const adoublev &x, adoublev &c);

   void IntegrateFEM(adoublev &c, adouble lhs, adouble rhs,
		     int slice, double t, double weight) const;

   void testN() {
      if (log2(S->N) != nearbyint(log2(S->N))) {
	 fprintf(stderr, "S->N must be some 2^k, but log2(S->N) = %f.\n",
		 log2(S->N));
	 assert(false);
      }
   }
};
