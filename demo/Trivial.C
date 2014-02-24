/*
**	This is an N-limbed pendulum, stretching out.
*/

# include <math.h>
# include "Stage.h"
# include "Trivial.h"
# include "World.h"
# include "Creature.h"
# include "Primitives.h"
# include "BodyPoint.h"
# include "Link.h"
# include "DOF.h"
# include "Muscle.h"
# include "Constant.h"
# include "Force.h"
# include "Impulse.h"
# include "Fun.h"
# include "Hermite.h"
# include "Hermlet.h"
# include "Hat.h"
# include "PWL.h"
# include "GaussianIntegrator.h"

# define PI	M_PI

void Trivial::setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c) {
   DOF *alpha;
   ThinRod *rod;

   W = new World(0.0, false);

   DOF *X = new DOF(W, "X");
   DOF *Y = new DOF(W, "Y");

   Creature *C = new Creature(W, "Anchor", X, Y);

   alpha = new DOF(W, "LinkAngle");
   rod = new ThinRod(W, "Rod", alpha, 1.0, 0.04);
   rod->MakePoint("Top", -0.5, 0.0, 0.0);
   rod->MakePoint("Bottom", 0.5, 0.0, 0.0);

   W->Register(C);
   C->Attach(rod->GetPoint("Top"));

   Integrator *i = new GaussianIntegrator(5);
   Stage *S1 = new Stage(W, i, 32, 2);

   new Constant(S1, X, 0);
   new Constant(S1, Y, 0);

   if (!W->ImplicitMuscles) {
      new Muscle(S1, alpha, new PWL(S1));
   }
   new Hermlet(S1, alpha, -PI/2, PI/2);
   new ValConstraint(S1, 0, 0, alpha->qVal, -PI/2);
   new ValConstraint(S1, 0, 0, alpha->qDot, 0);
   new ValConstraint(S1, S1->N-1, 1, alpha->qVal, PI/2);
   new ValConstraint(S1, S1->N-1, 1, alpha->qDot, 0);
   W->Initialize(x, c);
}

void Trivial::update(int kk, 
		    const adoublev &x, const adoublev &u,
		    adoublev &f, adouble &f0, adoublev &c) {
   W->Update(x, c, f0);
}


// propagate the class to the command interface
IF_CLASS_DEFINE("Trivial", Trivial, Omu_Program);

