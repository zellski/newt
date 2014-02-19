# include "Stage.h"
# include "World.h"
# include "Constraints.h"
# include "Creature.h"
# include "Impulse.h"
# include "Force.h"
# include "Muscle.h"
# include "DOF.h"
# include "Fun.h"
# include "Integrator.h"

Stage::Stage(World *w, Integrator *i, int n, double t) :
   W(w),
   Muscles(),
   Forces(),
   Impulses(),
   Cons(),
   integrator(i),
   Min(-1), Max(-1), Start(-1), // not used
   N(n),
   T(t),
   h(t/n),
   speedContribution(0.0),
   torqueContribution(0.0),
   sIx(w->Register(this)),
   tIx(-1)
{
   assert(!!W);
   assert(N > 0);
   assert(T > 0);
};

Stage::Stage(World *w, Integrator *i,
             int n, double min, double max, double start) :
   W(w),
   Muscles(),
   Forces(),
   Impulses(),
   Cons(),
   Min(min), Max(max), Start(start),
   integrator(i),
   N(n),
   T(start),
   h(start/n),
   speedContribution(0.0),
   torqueContribution(0.0),
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
      cerr << "Stage " << sIx << ": T == " << T.value();
      if (T.value() <= Min || T.value() >= Max) {
	 cerr << " (bumping)";
      }
      cerr << "\n";

   }
   speedContribution = torqueContribution = 0.0;

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
   cerr << "Stage [" << sIx << "]: speed/torque = " << (speedContribution/torqueContribution) << "\n";
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

void Stage::FEMEquations(const adoublev &x, adoublev &c, adouble &f0) {
   for (int slice = 0; slice < N; slice ++) {
       integrator->integrate(this, slice, x, c, f0);
   }
}

void Stage::FEMPoint(const int slice, const double t, const double weight,
                     const adoublev &x, adoublev &c, adouble &f0) {
    SnapShot(x, slice, t);

    for (vector<DOF *> :: const_iterator p = W->DOFs.begin();
         p != W->DOFs.end(); p++) {
        adouble qC = (*p)->qCurvature;
        if (!W->ImplicitMuscles) {
            qC += (*p)->QVal;
        } else {
            adouble foo = (*p)->QVal;
            f0 += weight * h * foo * foo;
        }
        adouble qM = (*p)->qMomentum;

        adouble foo = (*p)->qDot;
        foo = foo * foo;
        f0 += 5 * weight * h * foo;

        speedContribution += 5 * weight * h.value() * foo.value();

        (*p)->DOFReps[sIx]->IntegrateFEM(c, qM, qC, slice, t, weight);
    }
    for (vector<Muscle *>::const_iterator m = Muscles.begin();
         m != Muscles.end(); m ++) {
        adouble foo = (*m)->MVal;
        foo = (*m)->Weight*foo*foo;
        f0 += weight * h * foo;
        torqueContribution += weight * h.value() * foo.value();
    }
}

