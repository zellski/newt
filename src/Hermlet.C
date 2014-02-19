# include "Stage.h"
# include "adolc.h"
# include "Hermlet.h"
# include "World.h"
# include "DOF.h"

Hermlet::Hermlet(Stage *const s, double from, double to) :
   Fun(s,from,to),
   xIx(S->claimVars(S->N*2+2)),
   cIx(-1),
   eIx(-1)
{
   testN();
}

Hermlet::Hermlet(Stage *const s, double from, double to,
		 double min, double max) :
   Fun(s,from,to,min,max),
   xIx(S->claimVars(S->N*2+2)),
   cIx(S->claimCons(S->N*2)),
   eIx(-1)
{
   testN();
   S->Register((Constraint *) this);
}

Hermlet::Hermlet(Stage *const s, DOF *const d, double from, double to) :
   Fun(s,d,from,to),
   xIx(S->claimVars(S->N*2+2)),
   cIx(-1),
   eIx(S->W->ImplicitMuscles ? -1 : S->claimCons(S->N*2))
{
   testN();
}

Hermlet::Hermlet(Stage *const s, DOF *const d, double from, double to,
		 double min, double max) :
   Fun(s,d,from,to,min,max),
   xIx(S->claimVars(S->N*2+2)),
   cIx(S->claimCons(S->N*2)),
   eIx(S->W->ImplicitMuscles ? -1 : S->claimCons(S->N*2))
{
   testN();
   S->Register((Constraint *) this);
}

double Hermlet::s1Val(double t) const {
   if (t < 0) {
      if (t < -1) return 0;
      return square(t+1)*(1-2*t);
   }
   if (t > 1) return 0;
   return square(1-t)*(t*2+1);
}

double Hermlet::s1Dot(double t) const {
   if (t < 0) {
      if (t < -1) return 0;
      return -6*t*(t+1);
   }
   if (t > 1) return 0;
   return 6*t*(t-1);
}

double Hermlet::s1Bis(double t) const {
   if (t < 0) {
      if (t < -1) return 0;
      return -12*t - 6;
   }
   if (t > 1) return 0;
   return 12*t - 6;
}

double Hermlet::s2Val(double t) const {
   if (t < 0) {
      if (t < -1) return 0;
      return t*square(t+1);
   }
   if (t > 1) return 0;
   return t*square(t-1);
}

double Hermlet::s2Dot(double t) const {
   if (t < 0) {
      if (t < -1) return 0;
      return 3*t*t+4*t+1;
   }
   if (t > 1) return 0;
   return 3*t*t - 4*t + 1;
}

double Hermlet::s2Bis(double t) const {
   if (t < 0) {
      if (t < -1) return 0;
      return 6*t + 4;
   }
   if (t > 1) return 0;
   return 6*t - 4;
}


double Hermlet::w1Val(double t) const {
   assert(t >= -1.0 && t <= 1.0);
   return 1 *
      (-2*s1Val(2*t + 1) + 4*s1Val(2*t) - 2*s1Val(2*t - 1)
       - 21*s2Val(2*t + 1) + 21*s2Val(2*t - 1));
}

double Hermlet::w1Dot(double t) const {
   assert(t >= -1.0 && t <= 1.0);
   return 2 *
      (-2*s1Dot(2*t + 1) + 4*s1Dot(2*t) - 2*s1Dot(2*t - 1)
       - 21*s2Dot(2*t + 1) + 21*s2Dot(2*t - 1));
}
double Hermlet::w1Bis(double t) const {
   assert(t >= -1.0 && t <= 1.0);
   return 4 *
      (-2*s1Bis(2*t + 1) + 4*s1Bis(2*t) - 2*s1Bis(2*t - 1)
       - 21*s2Bis(2*t + 1) + 21*s2Bis(2*t - 1));
}

double Hermlet::w2Val(double t) const {
   assert(t >= -1.0 && t <= 1.0);
   return 1 *
      (1*s1Val(2*t + 1) - 1*s1Val(2*t - 1)
       + 9*s2Val(2*t + 1) + 12*s2Val(2*t) + 9*s2Val(2*t - 1));
}
double Hermlet::w2Dot(double t) const {
   assert(t >= -1.0 && t <= 1.0);
   return 2 *
      (1*s1Dot(2*t + 1) - 1*s1Dot(2*t - 1)
       + 9*s2Dot(2*t + 1) + 12*s2Dot(2*t) + 9*s2Dot(2*t - 1));
}
double Hermlet::w2Bis(double t) const {
   assert(t >= -1.0 && t <= 1.0);
   return 4 *
      (1*s1Bis(2*t + 1) - 1*s1Bis(2*t - 1)
       + 9*s2Bis(2*t + 1) + 12*s2Bis(2*t) + 9*s2Bis(2*t - 1));
}

# define fprintf // fprintf

void Hermlet::SnapShot(const adoublev &x, int slice, double t) {
   Val = 0;
   Dot = 0;
   Bis = 0;

   /*
   ** There are two fundamental intervals to be covered,
   ** and we use three each of the two scaling functions,
   ** for a total of six.
   */

   int n = 2; int scale = S->N/2;
   /* derive which local segment we're in */
   int i = slice / scale;
   /* and what local t to use in that segment */
   double tttt = ((slice % scale) + t) / scale;

   int six = xIx + i*2;

   fprintf(stderr, "At slice/t (%d, %f) i/tttt = (%d, %f), six = %d", slice, t, i, tttt, six-xIx);

   /* first the right half of the relevant s1 */
   Val += x[six] * s1Val(tttt);
   Dot += x[six] * s1Dot(tttt);
   Bis += x[six] * s1Bis(tttt);
   fprintf(stderr, "[%d]", six-xIx);
   six ++;

   /* then the right half of the s2 */
   Val += x[six] * s2Val(tttt);
   Dot += x[six] * s2Dot(tttt);
   Bis += x[six] * s2Bis(tttt);
   fprintf(stderr, "[%d]", six-xIx);
   six ++;

   /* then left half of s1 */
   Val += x[six] * s1Val(tttt-1);
   Dot += x[six] * s1Dot(tttt-1);
   Bis += x[six] * s1Bis(tttt-1);
   fprintf(stderr, "[%d]", six-xIx);
   six ++;

   /* and finally left half of w2 */
   Val += x[six] * s2Val(tttt-1);
   Dot += x[six] * s2Dot(tttt-1);
   Bis += x[six] * s2Bis(tttt-1);
   fprintf(stderr, "[%d]\n", six-xIx);

   Dot /= S->T/n;
   Bis /= (S->T/n * S->T/n);

      /*
      ** Next, we go into the wavelet layer, where generally at a
      ** depth k we've split (0, 1) into 2^k segments. We have w1
      ** wavelets from 1 to (2^k)-1 and w2's from 0 to 2^k, for
      ** a total of (2^k)-1+(2^k)+1 = 2*2^k functions.
      */

      int depth = 1;
      int lix = xIx + 6;
      double waveScale = 1.0;

      while (n != S->N) {
	 /* derive which local segment we're in */
	 int i = slice / scale;
	 /* and finally what local t to use in that segment */
	 double tttt = ((slice % scale) + t) / scale;

	 int six = lix + i*2 - 1;

	 fprintf(stderr, "At slice/t (%d, %f) depth %d, i/tttt = (%d, %f), six = %d", slice, t, depth, i, tttt, six-xIx);

	 adouble lVal = 0, lDot = 0, lBis = 0;

	 /* first the right half of the relevant w1 */
	 if (i != 0) {
	    /* unless we're at the extreme left */
	    lVal += x[six] * w1Val(tttt);
	    lDot += x[six] * w1Dot(tttt);
	    lBis += x[six] * w1Bis(tttt);
	    fprintf(stderr, "[%d]", six-xIx);
	 } else {
	    fprintf(stderr, "{%d}", six-xIx);
	 }
	 six ++;

	 /* then the right half of the w2 */

	 lVal += x[six] * w2Val(tttt);
	 lDot += x[six] * w2Dot(tttt);
	 lBis += x[six] * w2Bis(tttt);
	 fprintf(stderr, "[%d]", six-xIx);
	 six ++;

	 /* then left half of w1 */
	 if (i != n-1) {
	    lVal += x[six] * w1Val(tttt-1);
	    lDot += x[six] * w1Dot(tttt-1);
	    lBis += x[six] * w1Bis(tttt-1);
	    fprintf(stderr, "[%d]", six-xIx);
	    six ++;
	 } else {
	    fprintf(stderr, "{%d}", six-xIx);
	 }

	 /* and finally left half of w2 */
	 lVal += x[six] * w2Val(tttt-1);
	 lDot += x[six] * w2Dot(tttt-1);
	 lBis += x[six] * w2Bis(tttt-1);

	 fprintf(stderr, "[%d]\n", six-xIx);

	 waveScale *= sqrt(2.0);

	 Val += lVal / waveScale;
	 Dot += lDot / waveScale / (S->T/n);
	 Bis += lBis / waveScale / (S->T/n * S->T/n);

	 depth ++;
	 n <<= 1;
	 scale >>= 1;
	 lix += n;
   }
}

void Hermlet::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   if (cIx >= 0) {
      cerr << "Constraining Hermlet indices [" << cIx << " - " << cIx+4*S->N << "] to lie between [" << Min << " - " << Max << "]\n";
      for (int i = 0; i < S->N; i ++) {
	 c.min[cIx+i*2+0] = Min; c.max[cIx+i*2+0] = Max;
	 c.min[cIx+i*2+1] = Min; c.max[cIx+i*2+1] = Max;
      }
   }
   cerr << "Hermlet Constraints [" << Min << ", " << Max << "] -> [" << -(Max-Min)/S->h.value() << ", " << (Max-Min)/S->h.value() << "]\n";
//   for (int i = 0; i <= S->N; i ++) {
//      x.min[xIx+i*2+0] = Min;
//      x.max[xIx+i*2+0] = Max;
//      x.initial[xIx+i*2+0] = From + i*(To-From)/S->N;
//   }
   x.initial[xIx] = From;
   x.initial[xIx+2] = (From + To) / 2;
   x.initial[xIx+4] = To;
}

void Hermlet::Evaluate(const adoublev &x, adoublev &c) {
   if (cIx >= 0) {
      for (int i = 0; i < S->N; i ++) {
//      cerr << "Evaluating indices [" << (cIx+i*P+1) << " - " << (cIx+i*P+P) << "] as {";
	 SnapShot(x, i, 0.2); c[cIx+i*2+0] = Val;
	 SnapShot(x, i, 0.7); c[cIx+i*2+1] = Val;
      }
   }
}

void Hermlet::IntegrateFEM(adoublev &c, adouble qM, adouble qC,
			   int slice, double t, double weight) const {
   if (eIx < 0) {
      return;
   }

   int ix = eIx + 2*slice - 1;

   qC *= S->h;

   if (slice != 0) {
      // skip left-most value-interpolating trial function
      c[ix ++] += weight * (s1Dot(t)*qM + s1Val(t)*qC);
   } else {
      ix ++;
   }
   c[ix ++] += weight * (s2Dot(t)*qM + s2Val(t)*qC);
   if (slice != S->N-1) {
      // and right-most too
      c[ix ++] += weight * (s1Dot(t-1)*qM + s1Val(t-1)*qC);
   }
   c[ix ++] += weight * (s2Dot(t-1)*qM + s2Val(t-1)*qC);
}
