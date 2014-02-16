# include <adouble.h>
# include "BSpline.h"
# include "World.h"
# include "DOF.h"
# include "Stage.h"

BSpline::BSpline(Stage *const s, double from, double to) :
   Fun(s,from,to),
   xIx(S->claimVars(S->N+3)),
   cIx(-1),
   eIx(-1)
{}

BSpline::BSpline(Stage *const s, DOF *const d, double from, double to) :
   Fun(s,d,from,to),
   xIx(S->claimVars(S->N+3)),
   cIx(-1),
   eIx(S->claimCons(S->N))
{}

const double Hermite::ScaleCoef[4][4] = {
   {  1/6,    0,     0,   0   },
   {  1/2,   -1,     0,   2/3 }
   { -1/2,  1/2,   1/2,   1/6 },
   { -1/6,  1/2,  -1/2,   1/6 }
};

double Hermite::sVal(int width, double t) const {
   double t2 = t*t;
   double t3 = t2*t;

   return (ScaleCoef[width][0] * t3 +
	   ScaleCoef[width][1] * t2 +
	   ScaleCoef[width][2] * t +
	   ScaleCoef[width][3]);
}

double BSpline::sDot(int width, double t) const {
   double t2 = t*t;

   return (3 * ScaleCoef[width][0] * t2 +
	   2 * ScaleCoef[width][1] * t +
	   1 * ScaleCoef[width][2]);
}

double BSpline::sBis(int width, double t) const {
   return (6 * ScaleCoef[width][0] * t +
	   2 * ScaleCoef[width][1]);
}

void BSpline::SnapShot(const adoublev &x, int slice, double t) {
   Val = Dot = Bis = 0;

   for (int width = 0; width < 4; width ++) {
	 Val += x[xIx + (3 + slice - width)] * sVal(width, t);
	 Dot += x[xIx + (3 + slice - width)] * sDot(width, t);
	 Bis += x[xIx + (3 + slice - width)] * sBis(width, t);
   }
   Dot /= S->h;
   Bis /= (S->h * S->h);
}

void BSpline::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   for (int i = 0; i <= S->N+3; i ++) {
      x.min[xIx+i*2+0] = Min;
      x.max[xIx+i*2+0] = Max;
      x.initial[xIx+i] = From + i*(To-From)/(S->N+3);
   }
}

void BSpline::Evaluate(const adoublev &x, adoublev &c) {
//   if (cIx >= 0) {
//      for (int i = 0; i < S->N; i ++) {
//      cerr << "Evaluating indices [" << (cIx+i*P+1) << " - " << (cIx+i*P+P) << "] as {";
//	 SnapShot(x, i, 0.3); c[cIx+i*2] = Val;
//	 SnapShot(x, i, 0.7); c[cIx+i*2+1] = Val;
//      }
//   }
}

void BSpline::IntegrateFEM(adoublev &c, adouble qM, adouble qC,
			   int slice, double t, double weight) const {
   assert(eIx >= 0);

   int ix = eIx + slice;

   qC *= S->h;

   /* basis function 1: not used for N, N-1, N-2 */
   if (slice < N-2) {
      
	    

   if (slice != 0) {
      // skip left-most value-interpolating trial function
      c[ix ++] += weight * (sDot(1, 0, t)*qM + sVal(1, 0, t)*qC);
   } else {
      ix ++;
   }
   c[ix ++] += weight * (sDot(1, 1, t)*qM + sVal(1, 1, t)*qC);
   if (slice != S->N-1) {
      // and right-most too
      c[ix ++] += weight * (sDot(0, 0, t)*qM + sVal(0, 0, t)*qC);
   }
   c[ix ++] += weight * (sDot(0, 1, t)*qM + sVal(0, 1, t)*qC);
}
