# include "Creature.h"
# include "World.h"
# include "DOF.h"
# include "RigidBody.h"
# include "BodyPoint.h"

Creature::Creature(World *const w, const char *const S, DOF *x, DOF *y) :
   W(w),
   Name(S),
   X(x), Y(y),
   AnchorPoint("creature")
{};

// This is only called over stage transitions...

void Creature::CleanSweep() {
   TotF[0] = TotF[1] = 0;
   TotJ[0] = TotJ[1] = 0;

   for (vector<BodyPoint *>::const_iterator Q = AttachedPoints.begin();
	Q != AttachedPoints.end(); Q ++) {
      (*Q)->Parent->CleanSweep();
   }
}

void Creature::DistImpulse(const adoublev &x) {
   X->JVal = TotJ[0];
   Y->JVal = TotJ[1];
   for (vector<BodyPoint *>::const_iterator Q = AttachedPoints.begin();
	Q != AttachedPoints.end(); Q ++) {
      RigidBody *R = (*Q)->Parent;

      R->DistImpulse(x);

      X->JVal += R->TotJ[0];
      Y->JVal += R->TotJ[1];
   }
}

void Creature::BuildSweep(const adoublev &x) {
//   cerr << "Creature named " << Name << ": recursion started.\n";

   adoublev FlippedVal(2);

   X->qMomentum = X->qCurvature = 0;
   Y->qMomentum = Y->qCurvature = 0;

   Val[0] = X->qVal; Val[1] = Y->qVal;
   Dot[0] = X->qDot; Dot[1] = Y->qDot;
   Bis[0] = X->qBis; Bis[1] = Y->qBis;

   FlippedVal[0] = -Val[1];
   FlippedVal[1] =  Val[0];

   /* resultant force working on root */
   X->QVal = TotF[0];
   Y->QVal = TotF[1];

   for (vector<BodyPoint *>::const_iterator Q = AttachedPoints.begin();
	Q != AttachedPoints.end(); Q ++) {
      RigidBody *R = (*Q)->Parent;

      (*Q)->Val = Val;
      (*Q)->Dot = Dot;
      (*Q)->Bis = Bis;

      R->AVal = 0; R->ADot = 0; R->ABis = 0;

      R->BuildSweep(x, *Q);

      Y->qCurvature += W->G * R->TotM;

//      cerr << "Wrote qCurvature " << Y->qCurvature << " into DOF "
//	   << Y->Name << "...\n";

      X->qMomentum += R->KDot[0];
      Y->qMomentum += R->KDot[1];

      X->QVal += R->TotF[0];
      Y->QVal += R->TotF[1];
   }
}
