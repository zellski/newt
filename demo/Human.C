/*
**        This is a model of Human, the hopping desk lamp.
*/

# include <math.h>
# include "Stage.h"
# include "Human.h"
# include "World.h"
# include "Creature.h"
# include "Primitives.h"
# include "BodyPoint.h"
# include "DOF.h"
# include "Link.h"
# include "Muscle.h"
# include "Force.h"
# include "Constant.h"
# include "Impulse.h"
# include "Single.h"
# include "Hat.h"
# include "PWC.h"
# include "PWL.h"
# include "Hermlet.h"
# include "GaussianIntegrator.h"

# define PI        M_PI

void Human::setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c) {
    W = new World(-9.81);

   /*
   **   The human is constructed from a thin rod torso, two thin rod upper arms,
   **   two lower, two spheres for hands, two thin rod upper legs, two lower, two
   **   disks for feet; a thin rod for a neck and a sphere for a head.
   **
   **           (X, Y) - the root of the torso
   **           UA, LA -- upper/lower arm joints
   **           UL, LL -- upper/lower leg joints
   **           N - neck joint
   */

   DOF *X = new DOF(W, "X");
   DOF *Y = new DOF(W, "Y");

   DOF *TorsoAngle = new DOF(W, "TorsoAngle");

   RigidBody *Torso = new ThinRod(W, "Torso", TorsoAngle, 0.60, 0.08);

   BodyPoint *TorsoMid  = Torso->MakePoint("TorsoMid", 0, 0, 0); 
   BodyPoint *TorsoTop  = Torso->MakePoint("TorsoTop",  0.30, 0, 0);
   BodyPoint *TorsoBot  = Torso->MakePoint("TorsoBot", -0.30, 0, 0);
   BodyPoint *TorsoShoulder = Torso->MakePoint("TorsoShoulder", 0.25, 0, 0);

   DOF *UAAngle = new DOF(W, "UA");

   RigidBody *UA       = new ThinRod(W, "UA", UAAngle, 0.30, 0.06);
   BodyPoint *UATop    = UA->MakePoint("UATop",  .15, 0.0, 0.0);
   BodyPoint *UABot    = UA->MakePoint("UABot", -.15, 0.0, 0.0);

   TorsoShoulder->Attach(UATop);

   DOF *LAAngle = new DOF(W, "LA");

   RigidBody *LA       = new ThinRod(W, "LA", LAAngle, 0.20, 0.04);
   BodyPoint *LATop    = LA->MakePoint("LATop",  .10, 0.0, 0.0);
   BodyPoint *LABot    = LA->MakePoint("LABot", -.10, 0.0, 0.0);

   UABot->Attach(LATop);

   DOF *RHandAngle = new DOF(W, "RHandAngle");

   RigidBody *RHand     = new Sphere(W, "RHand", RHandAngle, 0.06, 0.5);
   BodyPoint *RHandTop  = RHand->MakePoint("RHandTop", 0.08, 0.0, 0.0); 

   LABot->Attach(RHandTop);

   DOF *ULAngle        = new DOF(W, "UL");
   RigidBody *UL       = new ThinRod(W, "UL", ULAngle, 0.50, 0.06);
   BodyPoint *ULTop    = UL->MakePoint("ULTop",  0.25, 0.0, 0.0);
   BodyPoint *ULBot    = UL->MakePoint("ULBot", -0.25, 0.0, 0.0);

   ULTop->Attach(TorsoBot);

   DOF *LLAngle        = new DOF(W, "LL");
   RigidBody *LL       = new ThinRod(W, "LL", LLAngle, 0.40, 0.04);
   BodyPoint *LLTop    = LL->MakePoint("LLTop",  0.20, 0.0, 0.0);
   BodyPoint *LLBot    = LL->MakePoint("LLBot", -0.20, 0.0, 0.0);

   LLTop->Attach(ULBot);

   DOF *NeckAngle       = new DOF(W, "Neck");
   RigidBody *Neck      = new ThinRod(W, "Neck", NeckAngle, 0.30, 0.04);
   BodyPoint *NeckTop   = Neck->MakePoint("NeckTop",  0.15, 0.0, 0.0);
   BodyPoint *NeckBot   = Neck->MakePoint("NeckBot", -0.15, 0.0, 0.0);

   TorsoTop->Attach(NeckBot);

   DOF *HeadAngle       = new DOF(W, "Head");
   RigidBody *Head      = new Sphere(W, "Head", HeadAngle, 0.10);
   BodyPoint *HeadBot   = Head->MakePoint("HeadBot", 0.05, 0.0, 0.0);

   NeckTop->Attach(HeadBot);

   Creature *C = new Creature(W, "Creature", X, Y);
   W->Register(C);
   C->Attach(LLBot);
   

   Integrator *i = new GaussianIntegrator(10);

# define LL_REST         1*PI/8
# define UL_REST         7*PI/8
# define TORSO_REST     -5*PI/8
# define UA_REST        2*PI/8
# define LA_REST        5*PI/8

# define LL_STRETCH        7*PI/16
# define UL_STRETCH        2*PI/16
# define TORSO_STRETCH    -1*PI/16
# define UA_STRETCH        0*PI/16
# define LA_STRETCH        0*PI/16


   Stage *S1 = new Stage(W, i, 8, 0.3);

   new Muscle(S1, TorsoAngle, new PWL(S1), 1);
   new Muscle(S1, UAAngle, new PWL(S1), 1);
   new Muscle(S1, LAAngle, new PWL(S1), 1);
   new Muscle(S1, ULAngle, new PWL(S1), 1);
   new Muscle(S1, LLAngle, new PWL(S1), 1);

   new Constant(S1, X, 0);
   new Constant(S1, Y, 0);

   new Hermlet(S1, LLAngle,    LL_REST, LL_STRETCH,            0,          PI/2);
   new Hermlet(S1, ULAngle,    UL_REST, UL_STRETCH,            0,             15*PI/16);
   new Hermlet(S1, TorsoAngle, TORSO_REST, TORSO_STRETCH,       -7*PI/8,        0);
   new Hermlet(S1, UAAngle,    UA_REST, UA_STRETCH,           0,              PI);
   new Hermlet(S1, LAAngle,    LA_REST, LA_STRETCH,           0,              PI);

   new Constant(S1, RHandAngle, 0);
   new Constant(S1, NeckAngle, 0);
   new Constant(S1, HeadAngle, 0);

   new ValConstraint(S1, 0, 0, TorsoAngle->qVal, TORSO_REST);
   new ValConstraint(S1, 0, 0, TorsoAngle->qDot, 0);
   new ValConstraint(S1, 0, 0, UAAngle->qVal, UA_REST);
   new ValConstraint(S1, 0, 0, UAAngle->qDot, 0);
   new ValConstraint(S1, 0, 0, LAAngle->qVal, LA_REST);
   new ValConstraint(S1, 0, 0, LAAngle->qDot, 0);
   new ValConstraint(S1, 0, 0, ULAngle->qVal, UL_REST);
   new ValConstraint(S1, 0, 0, ULAngle->qDot, 0);
   new ValConstraint(S1, 0, 0, LLAngle->qVal, LL_REST);
   new ValConstraint(S1, 0, 0, LLAngle->qDot, 0);

   Stage *S2 = new Stage(W, i, 8, 1.0);

   new Muscle(S2, TorsoAngle, new PWL(S2), 1);
   new Muscle(S2, UAAngle, new PWL(S2), 1);
   new Muscle(S2, LAAngle, new PWL(S2), 1);
   new Muscle(S2, ULAngle, new PWL(S2), 1);
   new Muscle(S2, LLAngle, new PWL(S2), 1);

   new Hermlet(S2, X, 0, 0);
   new Hermlet(S2, Y, 0, 0);

   new Hermlet(S2, LLAngle,    LL_STRETCH, LL_STRETCH+2*PI,            0,          10*PI);
   new Hermlet(S2, ULAngle,    UL_STRETCH, UL_STRETCH,            0,             15*PI/16);
   new Hermlet(S2, TorsoAngle,  TORSO_STRETCH, TORSO_STRETCH,       -7*PI/8,        10*PI);
   new Hermlet(S2, UAAngle,    UA_STRETCH, UA_STRETCH,           0,              PI);
   new Hermlet(S2, LAAngle,    LA_STRETCH, LA_STRETCH,           0,              PI);


   new Constant(S2, RHandAngle, 0);
   new Constant(S2, NeckAngle, 0);
   new Constant(S2, HeadAngle, 0);

   new Impulse(S2, LLBot, 0, 1);

   Stage *S3 = new Stage(W, i, 8, 0.3);

   new Muscle(S3, TorsoAngle, new PWL(S3), 1);
   new Muscle(S3, UAAngle, new PWL(S3), 1);
   new Muscle(S3, LAAngle, new PWL(S3), 1);
   new Muscle(S3, ULAngle, new PWL(S3), 1);
   new Muscle(S3, LLAngle, new PWL(S3), 1);

   new Constant(S3, X, 0);
   new Constant(S3, Y, 0);

   new Hermlet(S3, LLAngle,    LL_STRETCH+2*PI, LL_REST+2*PI,            0,          10*PI);
   new Hermlet(S3, ULAngle,    UL_STRETCH, UL_REST,            0,             15*PI/16);
   new Hermlet(S3, TorsoAngle, TORSO_STRETCH, TORSO_REST,       -7*PI/8,        0);
   new Hermlet(S3, UAAngle,    UA_STRETCH, UA_REST,           0,              PI);
   new Hermlet(S3, LAAngle,    LA_STRETCH, LA_REST,           0,              PI);

   new Constant(S3, RHandAngle, 0);
   new Constant(S3, NeckAngle, 0);
   new Constant(S3, HeadAngle, 0);

   int end = S3->N - 1;

   new ValConstraint(S3, end, 1, TorsoAngle->qVal, TORSO_REST);
   new ValConstraint(S3, end, 1, TorsoAngle->qDot, 0);
   new ValConstraint(S3, end, 1, UAAngle->qVal, UA_REST);
   new ValConstraint(S3, end, 1, UAAngle->qDot, 0);
   new ValConstraint(S3, end, 1, LAAngle->qVal, LA_REST);
   new ValConstraint(S3, end, 1, LAAngle->qDot, 0);
   new ValConstraint(S3, end, 1, ULAngle->qVal, UL_REST);
   new ValConstraint(S3, end, 1, ULAngle->qDot, 0);
   new ValConstraint(S3, end, 1, LLAngle->qVal, LL_REST+2*PI);
   new ValConstraint(S3, end, 1, LLAngle->qDot, 0);

   new Link(S1, S2);
   new Link(S2, S3);

   W->Initialize(x, c);
}


void Human::update(int kk, 
                  const adoublev &x, const adoublev &u,
                  adoublev &f, adouble &f0, adoublev &c) {
   W->Update(x, c, f0);
}


// propagate the class to the command interface
IF_CLASS_DEFINE("Human", Human, Omu_Program);
