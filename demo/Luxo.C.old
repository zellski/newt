/*
**	This is a model of Luxo, the hopping desk lamp.
*/

# include <math.h>
# include "Luxo.h"
# include "World.h"
# include "Creature.h"
# include "Primitives.h"
# include "BodyPoint.h"
# include "DOF.h"
# include "Link.h"
# include "Muscle.h"
# include "Constant.h"
# include "Impulse.h"
# include "Single.h"
# include "Hat.h"
# include "Hermite.h"
# include "Stage.h"

# define PI	M_PI

# define BETA_REST	 3*PI/11
# define GAMMA_REST	-6*PI/11
# define DELTA_REST	-1*PI/11

void Luxo::setup(int k, Omu_Vector &x, Omu_Vector &u, Omu_Vector &c) {
   W = new World(-9.81);

   // Luxo has six degrees of freedom
   DOF *X = new DOF(W, "X");
   DOF *Y = new DOF(W, "Y");
   DOF *Alpha = new DOF(W, "Alpha");
   DOF *Beta  = new DOF(W, "Beta");
   DOF *Gamma = new DOF(W, "Gamma");
   DOF *Delta = new DOF(W, "Delta");

   // He is built from a Cylinder, two Rods, and a Sphere
   RigidBody *Base = new Disk(W, "Base", Alpha, 0.3, 0.04, 0.2);
   RigidBody *L1   = new ThinRod(W, "L1", Beta, 0.3, 0.06);
   RigidBody *L2   = new ThinRod(W, "L2", Gamma, 0.3, 0.06);
   RigidBody *Head = new Sphere(W, "Head", Delta, 0.1, 0.1);

   Creature *C = new Creature(W, "Creature", X, Y);

   BodyPoint *BaseMid = Base->MakePoint("BaseMid", 0, 0, 0);
   BodyPoint *BaseBot = Base->MakePoint("BaseBot", -0.01, 0, 0);
   BodyPoint *BaseTop = Base->MakePoint("BaseTop", 0.15, 0, 0); // outside
   BodyPoint *BaseLeft = Base->MakePoint("BaseLeft", 0, -0.3, 0);
   BodyPoint *BaseRight = Base->MakePoint("BaseRight", 0, 0.3, 0);
   BodyPoint *L1Bot = L1->MakePoint("L1Bot", -.15, 0.0, 0.0);
   BodyPoint *L1Top = L1->MakePoint("L1Top", .15, 0.0, 0.0);
   BodyPoint *L2Bot = L2->MakePoint("L2Bot", -.15, 0.0, 0.0);
   BodyPoint *L2Top = L2->MakePoint("L2Top", .15, 0.0, 0.0);
   BodyPoint *HeadBot = Head->MakePoint("HeadBot", -0.15, 0, 0); // outside

   // Register the Creature,
   W->Register(C);
   // attach the base to its root,
   C->Attach(BaseBot);
   // then the rest of the limbs
   BaseTop->Attach(L1Bot);
   L1Top->Attach(L2Bot);
   L2Top->Attach(HeadBot);

   // The first stage is quite short, between .05 and .2 seconds,
   Stage *S1 = new Stage(W, 3, .05, .2, .1);

   new Muscle(S1, Beta,  new Hermite(S1), 3);
   new Muscle(S1, Gamma, new Hermite(S1), 1);
   new Muscle(S1, Delta, new Hermite(S1), 3);

   // and Luxo is nailed to the floor so he can gather leap-momentum
   new Constant(S1, X, -1);
   new Constant(S1, Y, 0);
   new Constant(S1, Alpha, PI/2);

   new Hermite(S1, Beta, BETA_REST, PI/11, -2*PI/11, 4*PI/11);
   new Hermite(S1, Gamma, GAMMA_REST, -3*PI/11, -7*PI/11, 0);
   new Hermite(S1, Delta, DELTA_REST, DELTA_REST, -2*PI/11, 0);

   new ValConstraint(S1, 0, 0, Beta->qVal, BETA_REST);
   new ValConstraint(S1, 0, 0, Beta->qDot, 0);
   new ValConstraint(S1, 0, 0, Gamma->qVal, GAMMA_REST);
   new ValConstraint(S1, 0, 0, Gamma->qDot, 0);
   new ValConstraint(S1, 0, 0, Delta->qVal, DELTA_REST);
   new ValConstraint(S1, 0, 0, Delta->qDot, 0);

   // In the second stage, Luxo flies -- this can take longer,
   Stage *S2 = new Stage(W, 5, .1, 1, .5);

   new Muscle(S2, Beta, new Hermite(S2), 3);
   new Muscle(S2, Gamma, new Hermite(S2), 1);
   new Muscle(S2, Delta, new Hermite(S2), 1);

   // and the base position is now variable.
   new Hermite(S2, X, -1, 1);
   new Hermite(S2, Y, .3, .3);

   new Hermite(S2, Alpha, PI/2, PI/2, PI/3, 2*PI/3);
   new Hermite(S2, Beta, PI/11, PI/11, -1*PI/11, 4*PI/11);
   new Hermite(S2, Gamma, -3*PI/11, -3*PI/11, -6*PI/11, 0);
   new Hermite(S2, Delta, DELTA_REST, DELTA_REST, -2*PI/11, 0);

   // When Luxo lands, apply impact vectors of unknown magnitude!
   new Impulse(S2, BaseRight, 0, 1);
   new Impulse(S2, BaseLeft, 0, 1);
   new Impulse(S2, BaseMid, 1, 0);

   // In stage three, Luxo has landed, and must now stabilize
   Stage *S3 = new Stage(W, 3, .05, .2, .1);

   new Muscle(S3, Beta, new Hermite(S3), 3);
   new Muscle(S3, Gamma, new Hermite(S3), 1);
   new Muscle(S3, Delta, new Hermite(S3), 1);

   // with the nail once again nailed down, elsewhere.
   new Constant(S3, X, 1);
   new Constant(S3, Y, 0);

   new Constant(S3, Alpha, PI/2);

   new Hermite(S3, Beta, PI/11, BETA_REST, -2*PI/11, 4*PI/11);
   new Hermite(S3, Gamma, -3*PI/11, GAMMA_REST, -6*PI/11, 0);
   new Hermite(S3, Delta, DELTA_REST, DELTA_REST, -2*PI/11, 0);

   int slice = S3->N-1;

   new ValConstraint(S3, slice, 1, Beta->qVal, BETA_REST);
   new ValConstraint(S3, slice, 1, Beta->qDot, 0);
   new ValConstraint(S3, slice, 1, Gamma->qVal, GAMMA_REST);
   new ValConstraint(S3, slice, 1, Gamma->qDot, 0);
   new ValConstraint(S3, slice, 1, Delta->qVal, DELTA_REST);
   new ValConstraint(S3, slice, 1, Delta->qDot, 0);

   new Link(S1, S2);
   new Link(S2, S3);

   W->Initialize(x, c);
}


void Luxo::update(int kk, 
		  const adoublev &x, const adoublev &u,
		  adoublev &f, adouble &f0, adoublev &c) {
   W->Update(x, c, f0);
}


// propagate the class to the command interface
IF_CLASS_DEFINE("Luxo", Luxo, Omu_Program);
