# pragma once

# include <Omu_Variables.h>
# include "adolc.h"
# include "Fun.h"

/*
**        This is a basis of cubic piecewise polynomials, of width and
**        depth two; largely uncoupled and with good control over value
**        and first-derivate interpolation; "Hermite Cubics"
*/

class Stage;

class Hermite: public Fun, public Constraint {
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
   Hermite(Stage *const s, double from=0, double to=0);
   Hermite(Stage *const s, double from, double to, double min, double max);
   Hermite(Stage *const s, DOF *const D, double from=0, double to=0);
   Hermite(Stage *const s, DOF *const D, double from, double to,
           double min, double max);

   bool isConstant() { return false; }

   void SnapShot(const adoublev &x, int slice, double t);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
   void Evaluate(const adoublev &x, adoublev &c);

   void IntegrateFEM(adoublev &c, adouble lhs, adouble rhs,
                     int slice, double t, double weight) const;
};
