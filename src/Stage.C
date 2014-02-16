# include "Stage.h"
# include "World.h"
# include "Constraints.h"
# include "Creature.h"
# include "Impulse.h"
# include "Force.h"
# include "Muscle.h"
# include "DOF.h"
# include "Fun.h"

Stage::Stage(World *w, int n, double t) :
   W(w),
   Muscles(),
   Forces(),
   Impulses(),
   Cons(),
   Min(-1), Max(-1), Start(-1), // not used
   N(n),
   T(t),
   h(t/n),
   sIx(w->Register(this)),
   tIx(-1)
{
   assert(!!W);
   assert(N > 0);
   assert(T > 0);
};

Stage::Stage(World *w, int n, double min, double max, double start) :
   W(w),
   Muscles(),
   Forces(),
   Impulses(),
   Cons(),
   Min(min), Max(max), Start(start),
   N(n),
   T(start),
   h(start/n),
   sIx(w->Register(this)),
   tIx(w->claimVars(1))
{
   assert(!!W);
   assert(N > 0);
   assert(T > 0);
   cerr << "Stage " << sIx << " claiming time ix " << tIx << "\n";
}

int Stage::claimVars(int n) {
   return W->claimVars(n);
}

int Stage::claimCons(int n) {
   return W->claimCons(n);
}


int Stage::Register(Constraint *C) {
   assert(!!C);
   Cons.push_back(C);
   return Cons.size() - 1;
}

int Stage::Register(Muscle *M) {
   assert(!!M);
   Muscles.push_back(M);
   return Muscles.size() - 1;
}

int Stage::Register(Force *F) {
   assert(!!F);
   Forces.push_back(F);
   return Forces.size() - 1;
}

int Stage::Register(Fun *F) {
   assert(!!F);
   Funs.push_back(F);
   return Funs.size() - 1;
}

int Stage::Register(Impulse *I) {
   assert(!!I);
   Impulses.push_back(I);
   return Impulses.size() - 1;
}

void Stage::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   cerr << "For stage " << sIx << ":\n";
   cerr << " * Initializing DOFReps.\n";
   if (tIx >= 0) {
      x.min[tIx] = Min;
      x.max[tIx] = Max;
      x.initial[tIx] = Start;
   }
   for (vector<DOF *> :: const_iterator p = W->DOFs.begin();
	p != W->DOFs.end(); p++) {
      assert(!!(*p)->DOFReps[sIx]);
      (*p)->DOFReps[sIx]->Initialize(x, c);
   }
   cerr << " * Initializing Constraints\n";
   for (vector<Constraint *> :: const_iterator p = Cons.begin();
	p != Cons.end(); p++) {
      (*p)->Initialize(x, c);
   }
   cerr << " * Done!\n";
}

void Stage::Update(const adoublev &x, adoublev &c, adouble &f0) {
   if (tIx >= 0) {
      T = x[tIx];
      h = T/N;
//      cerr << "Stage " << sIx << ": T == " << T;
//      if (T <= Min || T >= Max) {
//	 cerr << " (bumping)";
//      }
//      cerr << "\n";

   }
   // now integrate the FEMequation constraints
//   cerr << " - Integrating FEM Equations: ";
   FEMEquations(x, c, f0);
//   cerr << " done.\n";

   // then sweep through all constraints slightly redundantly (for now)
//   cerr << " - Calling Evaluate in all constraints: ";
   for (vector<Constraint *> :: const_iterator p = Cons.begin();
	p != Cons.end(); p++) {
      (*p)->Evaluate(x, c);
//      cerr << ".";
   }
//   cerr << " done.\n";
}

void Stage::SnapShot(const adoublev &x, int slice, double t) {
   for (vector<Creature *>::const_iterator p = W->Creatures.begin();
	p != W->Creatures.end(); p++) {
      (*p)->CleanSweep();
   }
   for (vector<DOF *> :: const_iterator p = W->DOFs.begin();
	p != W->DOFs.end(); p++) {
      (*p)->SnapShot(x, sIx, slice, t);
   }
   for (vector<Fun *>::const_iterator p = Funs.begin();
	p != Funs.end(); p++) {
      (*p)->SnapShot(x, slice, t);
   }
   for (vector<Muscle *>::const_iterator p = Muscles.begin();
	p != Muscles.end(); p++) {
      (*p)->SnapShot(x, slice, t);
   }
   for (vector<Force *>::const_iterator p = Forces.begin();
	p != Forces.end(); p++) {
      (*p)->SnapShot(x, slice, t);
   }
   for (vector<Creature *>::const_iterator p = W->Creatures.begin();
	p != W->Creatures.end(); p++) {
      (*p)->BuildSweep(x);
   }
}   

//static const int SAMPLES = 3;
//static const double weights[] = { 1, 4, 2 };
//static const int SAMPLES = 7;
//static const double weights[] = { 1, 4, 2, 4, 2, 4, 1 };
static const int SAMPLES = 11;
static const double weights[] = { 1, 4, 2, 4, 2, 4, 2, 4, 2, 4, 1 };

void Stage::FEMEquations(const adoublev &x, adoublev &c, adouble &f0) {
   for (int slice = 0; slice < N; slice ++) {
//      cerr << "(* " << slice << " *)";
      /* slice is an interval of time, over which we wish to integrate */
      for (int sample = 0; sample < SAMPLES; sample ++) {
	 const double t = (double) sample/(SAMPLES-1);
	 const double weight = weights[sample]/SAMPLES/3;

	 SnapShot(x, slice, t);
	 for (vector<DOF *> :: const_iterator p = W->DOFs.begin();
	      p != W->DOFs.end(); p++) {
	    adouble qC = (*p)->qCurvature + (*p)->QVal;
	    adouble qM = (*p)->qMomentum;

	    (*p)->DOFReps[sIx]->IntegrateFEM(c, qM, qC, slice, t, weight);
	 }
	 for (vector<Muscle *>::const_iterator m = Muscles.begin();
	      m != Muscles.end(); m ++) {
	    adouble foo = (*m)->MVal;
	    foo = (*m)->Weight*foo*foo;
	    f0 += weight * h * foo;
	 }
      }
   }
}
