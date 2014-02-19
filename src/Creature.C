# include "Stage.h"
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
   TotF.zero();
   TotJ.zero();

   for (vector<BodyPoint *>::const_iterator Q = AttachedPoints.begin();
	Q != AttachedPoints.end(); Q ++) {
      (*Q)->Parent->CleanSweep();
   }
}

void Creature::DistImpulse(const adoublev &x) {
   X->JVal = TotJ.x;
   Y->JVal = TotJ.y;
   for (vector<BodyPoint *>::const_iterator Q = AttachedPoints.begin();
	Q != AttachedPoints.end(); Q ++) {
      RigidBody *R = (*Q)->Parent;

      R->DistImpulse(x);

      X->JVal += R->TotJ.x;
      Y->JVal += R->TotJ.y;
   }
}

void Creature::BuildSweep(const adoublev &x) {
//   cerr << "Creature named " << Name << ": recursion started.\n";

   AVec FlippedVal;

   X->qMomentum = X->qCurvature = 0;
   Y->qMomentum = Y->qCurvature = 0;

   Val.x = X->qVal; Val.y = Y->qVal;
   Dot.x = X->qDot; Dot.y = Y->qDot;
   Bis.x = X->qBis; Bis.y = Y->qBis;

   FlippedVal.x = -Val.y;
   FlippedVal.y =  Val.x;

   /* resultant force working on root */
   X->QVal = TotF.x;
   Y->QVal = TotF.y;

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

      X->qMomentum += R->KDot.x;
      Y->qMomentum += R->KDot.y;

      X->QVal += R->TotF.x;
      Y->QVal += R->TotF.y;
   }
}
