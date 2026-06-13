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

Stage::Stage(World *w, const char *name, Integrator *i, int n, double t) :
   W(w),
   Name(name),
   Muscles(),
   Forces(),
   Funs(),
   Impulses(),
   Cons(),
   integrator(i),
   Min(-1), Max(-1), Start(-1), // not used
   speedContribution(0.0),
   torqueContribution(0.0),
   N(n),
   sIx(w->Register(this)),
   tIx(-1),
   T(t),
   h(t/n)
{
   assert(!!W);
   assert(N > 0);
   assert(T > 0);
};

Stage::Stage(World *w, const char *name, Integrator *i,
             int n, double min, double max, double start) :
   W(w),
   Name(name),
   Muscles(),
   Forces(),
   Funs(),
   Impulses(),
   Cons(),
   integrator(i),
   Min(min), Max(max), Start(start),
   speedContribution(0.0),
   torqueContribution(0.0),
   N(n),
   sIx(w->Register(this)),
   tIx(w->claimVars(1, std::string(name) + ": duration")),
   T(start),
   h(start/n)
{
   assert(!!W);
   assert(N > 0);
   assert(T > 0);
   cerr << "Stage " << sIx << " claiming time ix " << tIx << "\n";
}

int Stage::claimVars(int n, const std::string &label) {
   return W->claimVars(n, label);
}

int Stage::claimCons(int n, const std::string &label) {
   return W->claimCons(n, label);
}

std::string ClaimLabel(Stage *s, DOF *d, const char *what) {
   return std::string(s->Name) + ": " + d->Name + " " + what;
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
      (*p)->Rep(sIx)->Initialize(x, c);
   }
   // stage-registered Funs (muscles, forces) are disjoint from the
   // DOFReps above; they need their bounds/initials set up too
   for (vector<Fun *> :: const_iterator p = Funs.begin();
        p != Funs.end(); p++) {
      (*p)->Initialize(x, c);
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
//   cerr << "Stage [" << sIx << "]: speed/torque = " << (speedContribution/torqueContribution) << "\n";
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
        qC += (*p)->QVal;
        adouble qM = (*p)->qMomentum;

# if 0
        adouble qDot = (*p)->qDot;
        adouble qDotContrib = qDot * qDot;
        f0 += 5 * weight * h * qDotContrib; // completely arbitrary; this is highly experimental

        speedContribution += 5 * weight * h.value() * qDotContrib.value();
# endif

        (*p)->Rep(sIx)->IntegrateFEM(c, qM, qC, slice, t, weight);
    }
    for (vector<Muscle *>::const_iterator m = Muscles.begin();
         m != Muscles.end(); m ++) {
        adouble MVal = (*m)->MVal;
        adouble MContrib = (*m)->Weight*MVal*MVal;
        f0 += weight * h * MContrib;
        torqueContribution += weight * h.value() * MContrib.value();
    }
}

