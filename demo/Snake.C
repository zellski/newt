/*
**	This is an N-limbed pendulum, stretching out.
*/

# include <math.h>
# include "Stage.h"
# include "Snake.h"
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

# define LINKS 8
# define PI	M_PI

void Snake::setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c) {
   DOF *Angles[LINKS];
   ThinRod *Rods[LINKS];

   W = new World(-9.81, false);

   DOF *X = new DOF(W, "X");
   DOF *Y = new DOF(W, "Y");

   Creature *C = new Creature(W, "Anchor", X, Y);

   for (int i = 0; i < LINKS; i ++) {
      double len = 2.0 / LINKS;

      Angles[i] = new DOF(W, "LinkAngle");
      Rods[i] = new ThinRod(W, "LinkRod", Angles[i], len, 0.04);

      Rods[i]->MakePoint("Top", -len/2, 0.0, 0.0);
      Rods[i]->MakePoint("Bot",  len/2, 0.0, 0.0);
   }

   W->Register(C);
   C->Attach(Rods[0]->GetPoint("Top"));
   for (int i = 1; i < LINKS; i ++) {
      Rods[i-1]->GetPoint("Bot")->Attach(Rods[i]->GetPoint("Top"));
   }

   Integrator *i = new GaussianIntegrator(5);
   Stage *S1 = new Stage(W, i, 16, 1);

   new Constant(S1, X, 0);
   new Constant(S1, Y, 0);

   if (!W->ImplicitMuscles) {
      for (int i = 0; i < LINKS; i ++) {
         new Muscle(S1, Angles[i], new PWL(S1), 1/(i+1));
      }
   }
   new Hat(S1, Angles[0], -PI/2, PI/2, -PI, PI);
   for (int i = 1; i < LINKS; i ++) {
      new Hat(S1, Angles[i], 0, 0, -PI/4, PI/4);
   }
   new ValConstraint(S1, 0, 0, Angles[0]->qVal, -M_PI/2);
   new ValConstraint(S1, 0, 0, Angles[0]->qDot, 0);
   new ValConstraint(S1, S1->N-1, 1, Angles[0]->qVal, PI/2);
   new ValConstraint(S1, S1->N-1, 1, Angles[0]->qDot, 0);
   for (int i = 1; i < LINKS; i ++) {
      new ValConstraint(S1, 0, 0, Angles[i]->qVal, 0);
      new ValConstraint(S1, 0, 0, Angles[i]->qDot, 0);
      new ValConstraint(S1, S1->N-1, 1, Angles[i]->qVal, 0);
      new ValConstraint(S1, S1->N-1, 1, Angles[i]->qDot, 0);
   }
   W->Initialize(x, c);
}

void Snake::update(int kk, 
		    const adoublev &x, const adoublev &u,
		    adoublev &f, adouble &f0, adoublev &c) {
   W->Update(x, c, f0);
}


// propagate the class to the command interface
IF_CLASS_DEFINE("Snake", Snake, Omu_Program);

