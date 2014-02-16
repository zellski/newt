# include <adouble.h>
# include "Hermite.h"
# include "World.h"
# include "DOF.h"
# include "Stage.h"

Hermite::Hermite(Stage *const s, double from, double to) :
   Fun(s,from,to),
   xIx(S->claimVars(S->N*2+2)),
   cIx(-1),
   eIx(-1)
{}

Hermite::Hermite(Stage *const s, double from, double to,
		 double min, double max) :
   Fun(s,from,to,min,max),
   xIx(S->claimVars(S->N*2+2)),
   cIx(S->claimCons(S->N*2)),
   eIx(-1)
{
   S->Register((Constraint *) this);
}

Hermite::Hermite(Stage *const s, DOF *const d, double from, double to) :
   Fun(s,d,from,to),
   xIx(S->claimVars(S->N*2+2)),
   cIx(-1),
   eIx(S->claimCons(S->N*2))
{}

Hermite::Hermite(Stage *const s, DOF *const d, double from, double to,
		 double min, double max) :
   Fun(s,d,from,to,min,max),
   xIx(S->claimVars(S->N*2+2)),
   cIx(S->claimCons(S->N*2)),
   eIx(S->claimCons(S->N*2))
{
   S->Register((Constraint *) this);
}


const double Hermite::ScaleCoef[2][2][4] = {
   {
      { -2,   3,   0,   0 },
      {  2,  -3,   0,   1 }
   }, {
      { 1,  -1,   0,   0 },
      { 1,  -2,   1,   0 }
   }
};

double Hermite::sVal(int width, int depth, double t) const {
   double t2 = t*t;
   double t3 = t2*t;

   return (ScaleCoef[depth][width][0] * t3 +
	   ScaleCoef[depth][width][1] * t2 +
	   ScaleCoef[depth][width][2] * t +
	   ScaleCoef[depth][width][3]);
}

double Hermite::sDot(int width, int depth, double t) const {
   double t2 = t*t;

   return (3 * ScaleCoef[depth][width][0] * t2 +
	   2 * ScaleCoef[depth][width][1] * t +
	   1 * ScaleCoef[depth][width][2]);
}

double Hermite::sBis(int width, int depth, double t) const {
   return (6 * ScaleCoef[depth][width][0] * t +
	   2 * ScaleCoef[depth][width][1]);
}

void Hermite::SnapShot(const adoublev &x, int slice, double t) {
   Val = Dot = Bis = 0;

   for (int width = 0; width < 2; width ++) {
      for (int depth = 0; depth < 2; depth ++) {
	 Val += x[xIx + 2*(slice+1-width) + depth] * sVal(width, depth, t);
	 Dot += x[xIx + 2*(slice+1-width) + depth] * sDot(width, depth, t);
	 Bis += x[xIx + 2*(slice+1-width) + depth] * sBis(width, depth, t);
      }
   }
   Dot /= S->h;
   Bis /= (S->h * S->h);
}

void Hermite::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   if (cIx >= 0) {
      cerr << "Constraining indices [" << cIx << " - " << cIx+4*S->N << "] to lie between [" << Min << " - " << Max << "]\n";
      for (int i = 0; i < S->N; i ++) {
	 c.min[cIx+i*2] = Min; c.max[cIx+i*2] = Max;
	 c.min[cIx+i*2+1] = Min; c.max[cIx+i*2+1] = Max;
      }
   }
   cerr << "Constraints [" << Min << ", " << Max << "] -> [" << -(Max-Min)/value(S->h) << ", " << (Max-Min)/value(S->h) << "]\n";
   for (int i = 0; i <= S->N; i ++) {
      x.min[xIx+i*2+0] = Min;
      x.max[xIx+i*2+0] = Max;
      x.initial[xIx+i*2+0] = From + i*(To-From)/S->N;
   }
}

void Hermite::Evaluate(const adoublev &x, adoublev &c) {
   if (cIx >= 0) {
      for (int i = 0; i < S->N; i ++) {
//      cerr << "Evaluating indices [" << (cIx+i*P+1) << " - " << (cIx+i*P+P) << "] as {";
	 SnapShot(x, i, 0.3); c[cIx+i*2] = Val;
	 SnapShot(x, i, 0.7); c[cIx+i*2+1] = Val;
      }
   }
}

void Hermite::IntegrateFEM(adoublev &c, adouble qM, adouble qC,
			   int slice, double t, double weight) const {
   assert(eIx >= 0);

   int ix = eIx + 2*slice - 1;

   qC *= S->h;

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
