# ifndef FUN_H
# define FUN_H

# include <adouble.h>
# include "DOF.h"

class Stage;

class Fun {
public:
   Stage *const S;
   const double From, To, Min, Max;
   adouble Val, Dot, Bis;

   Fun(Stage *const s, DOF *const D, double from=0, double to=0,
       double min=-Inf, double max=Inf) :
      S(s),
      From(from), To(to),
      Min(min), Max(max),
      Val(0), Dot(0), Bis(0)
   {
      D->Register(this);
   }

   Fun(Stage *const s, double from=0, double to=0,
       double min=-Inf, double max=Inf) :
      S(s),
      From(from), To(to),
      Min(min), Max(max),
      Val(0), Dot(0), Bis(0)
   {
      S->Register(this);
   }

   virtual bool isConstant() = 0;

   virtual void SnapShot(const adoublev &x, int slice, double t) = 0;
   virtual void Initialize(Omu_Vector &, Omu_Vector &) {}
   virtual void IntegrateFEM(adoublev &c, adouble lhs, adouble rhs,
			     int slice, double t, double weight) const {}

};

# endif
