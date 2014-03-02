# include "Stage.h"
# include "RigidBody.h"
# include "BodyPoint.h"
# include "Impulse.h"
# include "DOF.h"
# include "World.h"

BodyPoint *RigidBody::MakePoint(const char *const S,
                                double x, double y, double z) {
   assert(!Points[S]);
   return Points[S] = new BodyPoint(S, this, x, y, z);
}

BodyPoint *RigidBody::GetPoint(const char *const S) {
   return Points[S];
}

void RigidBody::CleanSweep() {
   for (PointMap::const_iterator p = Points.begin();
        p != Points.end(); p ++) {
      BodyPoint *P = ((*p).second);
      for (vector<BodyPoint *>::const_iterator Q = P->AttachedPoints.begin();
           Q != P->AttachedPoints.end(); Q ++) {
         (*Q)->Parent->CleanSweep();
      }
      P->TotF.zero();
      P->TotJ.zero();
   }
}

void RigidBody::DistImpulse(const adoublev &x) {
   JVal = 0;
   TotJ.zero();

   for (PointMap::const_iterator p = Points.begin();
        p != Points.end(); p ++) {
      BodyPoint *P = ((*p).second);
      for (vector<BodyPoint *>::const_iterator Q = P->AttachedPoints.begin();
           Q != P->AttachedPoints.end(); Q ++) {
         RigidBody *R = (*Q)->Parent;

         R->DistImpulse(x);

         TotJ += R->TotJ;
         JVal += R->JVal;
      }
      JVal += P->TotJ * P->Val.flipped();
      TotJ += P->TotJ;
//      cerr << "Added JVal " << P->TotJ*tmp << " and TotJ " << P->TotJ << "\n";
   }
   // FIXME: What the hell is tmp supposed to be here? Entry->Val.flipped() probably?
   assert(false);
//   Angle->JVal += JVal - tmp*TotJ;
//   cerr << "Finally, Angle->Jval: " << JVal - tmp*TotJ << "...\n";
}

void RigidBody::BuildSweep(const adoublev &x, const BodyPoint *const Entry) {
   AVal += Angle->qVal;
   ADot += Angle->qDot;

   AVec RIB = Entry->LocPos.rotated(AVal);
   AVec FIB = RIB.flipped();

   rVal = Entry->Val - RIB;
   rDot = Entry->Dot - ADot*FIB;

   PVal = ADot * Mass * ROG2;

   AVec rValFlip = rVal.flipped();
   UVal = Mass * (rDot*rValFlip);

   KVal = Mass * rVal;
   KDot = Mass * rDot;

   FVal = 0;
   TotF.zero();

   TotM = Mass;

//   cerr << "Body " << Name << ": Pass 1...";

   for (PointMap::const_iterator p = Points.begin();
        p != Points.end(); p ++) {
      BodyPoint *P = ((*p).second);
//      cerr << "Fiddling with point " << P->Name << "...\n";
      AVec ROB = P->LocPos.rotated(AVal);
      AVec FOB = ROB.flipped();

      P->Val = rVal + ROB;
      P->Dot = rDot + ADot*FOB;

      for (vector<BodyPoint *>::const_iterator Q = P->AttachedPoints.begin();
           Q != P->AttachedPoints.end(); Q ++) {
         RigidBody *R = (*Q)->Parent;

         (*Q)->Val = P->Val;
         (*Q)->Dot = P->Dot;

         R->AVal = AVal; R->ADot = ADot;

         R->BuildSweep(x, *Q);

         KVal += R->KVal;
         KDot += R->KDot;

         TotM += R->TotM;
         TotF += R->TotF;
         FVal += R->FVal;

         PVal += R->PVal;
         UVal += R->UVal;
      }
      FVal += P->TotF * P->Val.flipped();
      TotF += P->TotF;
   }

//   cerr << "Done with recursion for " << Name << " ...";

   AVec dVal = (KVal - TotM*Entry->Val).flipped();

   Angle->qCurvature = W->G*dVal.y - Entry->Dot.flipped()*KDot;

   AVec entryValFlip = Entry->Val.flipped();
   Angle->qMomentum = UVal + PVal - entryValFlip*KDot;
//   if (FVal - tmp*TotF != 0) {
//      cerr << "FORCE: " << FVal - tmp*TotF << "\n";
//   }
   Angle->QVal += FVal - entryValFlip*TotF;
//   cerr << "Done!\n";
}
