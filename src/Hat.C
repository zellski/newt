# include <adouble.h>
# include "Hat.h"
# include "World.h"
# include "DOF.h"
# include "Stage.h"

Hat::Hat(Stage *const s) :
   Fun(s),
   xIx(S->claimVars(S->N+1)),
   eIx(-1)
{}

Hat::Hat(Stage *const s, DOF *const d, double from, double to,
	 double min, double max) :
   Fun(s,d,from,to,min,max),
   xIx(S->claimVars(S->N+1)),
   eIx(S->claimCons(S->N-1))
{}

double Hat::sVal(int width, int depth, double t) const {
   if (width == 0) {
      return t;
   }
   if (width == 1) {
      return 1-t;
   }
   assert(0);
}

double Hat::sDot(int width, int depth, double t) const {
   if (width == 0) {
      return 1;
   }
   if (width == 1) {
      return -1;
   }
   assert(0);
}


void Hat::SnapShot(const adoublev &x, int slice, double t) {
   Val = (x[xIx+slice+1]*sVal(0, 0, t) + x[xIx+slice]*sVal(1, 0, t));
   Dot = (x[xIx+slice+1]*sDot(0, 0, t) + x[xIx+slice]*sDot(1, 0, t)) / S->h;
   Bis = 0;
}


void Hat::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   for (int i = 0; i <= S->N; i ++) {
      x.min[xIx + i] = Min;
      x.max[xIx + i] = Max;
      x.initial[xIx + i] = From + i*(To-From)/S->N;
   }
}

void Hat::IntegrateFEM(adoublev &c, adouble qM, adouble qC,
		       int slice, double t, double weight) const {
   assert(eIx >= 0);

   qC *= S->h;

   if (slice != 0) {
      // skip left-most hat function
//      cerr << "<" << (eIx+slice-1) << ">";
      c[eIx+slice-1] += weight * (sDot(1, 0, t)*qM + sVal(1, 0, t)*qC);
   }
   if (slice != S->N-1) {
//      cerr << "[" << (eIx+slice) << "]";
      // and right-most too
      c[eIx+slice] += weight * (sDot(0, 0, t)*qM + sVal(0, 0, t)*qC);
   }
}
