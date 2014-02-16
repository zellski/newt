/*
**	This is a thin rod falling onto a friction-less surface.
*/

# include <math.h>
# include "Needle.h"
# include "World.h"
# include "Constraints.h"
# include "Creature.h"
# include "Primitives.h"
# include "BodyPoint.h"
# include "Link.h"
# include "DOF.h"
# include "Muscle.h"
# include "Constant.h"
# include "Hermite.h"
# include "Stage.h"
# include "Impulse.h"

void Needle::setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c) {
   W = new World(-9.81);

   DOF *X = new DOF(W, "X");
   DOF *Y = new DOF(W, "Y");
   DOF *Alpha = new DOF(W, "Alpha");

   Creature *C = new Creature(W, "Anchor", X, Y);

   ThinRod *P1 = new ThinRod(W, "P1", Alpha, 1, 0.01);
   BodyPoint *P1Top = P1->MakePoint("Top", -0.5, 0.0, 0.0);
   BodyPoint *P1Bot = P1->MakePoint("Bot",  0.5, 0.0, 0.0);

   W->Register(C);
   C->Attach(P1Top);

   // now establish interval representations

//   Stage *I1 = new Stage(W, 4, 1);
   Stage *I1 = new Stage(W, 4, 0.1, 2, 0.5);
   new Hermite(I1, X, 0, 0);
   new Hermite(I1, Y, 1, 0);
   new Hermite(I1, Alpha, 8*M_PI/17, 8*M_PI/17);

//   Stage *I2 = new Stage(W, 4, 1);
   Stage *I2 = new Stage(W, 4, 0.1, 2, 0.3);
   new Hermite(I2, X, 0, 0);
   new Hermite(I2, Y, 0, -1);
   new Hermite(I2, Alpha, 8*M_PI/17, 0);

   Stage *I3 = new Stage(W, 3, 0.1);
   new Hermite(I3, X, -1, -1);
   new Constant(I3, Y, 0);
   new Constant(I3, Alpha, 0);

     new Impulse(I1, P1Top, 0, 1);
     new Link(I1, I2);

     new Impulse(I2, P1Bot, 0, 1);
     new Link(I2, I3);

     new ValConstraint(I1, 0, 0, Alpha->qVal, 8*M_PI/17);
     new ValConstraint(I1, 0, 0, Alpha->qDot, 0);

     new ValConstraint(I1, 0, 0, Y->qVal, 1);
     new ValConstraint(I1, 0, 0, Y->qDot, 0);

     new ValConstraint(I1, 0, 0, X->qVal, 0);
     new ValConstraint(I1, 0, 0, X->qDot, 0);

//     new ValConstraint(I2, 0, 0, Alpha->qDot, 0.1);

   W->Initialize(x, c);
}

void Needle::update(int kk, 
		    const adoublev &x, const adoublev &u,
		    adoublev &f, adouble &f0, adoublev &c) {
   W->Update(x, c, f0);
}


// propagate the class to the command interface
IF_CLASS_DEFINE("Needle", Needle, Omu_Program);
