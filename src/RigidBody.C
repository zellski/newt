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
   if (W->ImplicitMuscles) {
      ABis += Angle->qBis;
   }

   sinVal = sin(AVal); cosVal = cos(AVal);

   AVec RIB = rotated(Entry->LocPos);
   AVec FIB = RIB.flipped();

   rVal = Entry->Val - RIB;
   rDot = Entry->Dot - ADot*FIB;
   if (W->ImplicitMuscles) {
      rBis = Entry->Bis - ABis*FIB + ADot*ADot*RIB;
   }

   PVal = ADot * Mass * ROG2;
   if (W->ImplicitMuscles) {
      PDot = ABis * Mass * ROG2;
   }

   AVec rValFlip = rVal.flipped();
   UVal = Mass * (rDot*rValFlip);
   if (W->ImplicitMuscles) {
      UDot = Mass * (rBis*rValFlip);
   }

   KVal = Mass * rVal;
   KDot = Mass * rDot;
   if (W->ImplicitMuscles) {
      KBis = Mass * rBis;
   }

   FVal = 0;
   TotF.zero();

   TotM = Mass;

//   cerr << "Body " << Name << ": Pass 1...";

   if (W->ImplicitMuscles) {
      Angle->QVal = 0;
   }

   for (PointMap::const_iterator p = Points.begin();
        p != Points.end(); p ++) {
      BodyPoint *P = ((*p).second);
//      cerr << "Fiddling with point " << P->Name << "...\n";
      AVec ROB = rotated(P->LocPos);
      AVec FOB = ROB.flipped();

      P->Val = rVal + ROB;
      P->Dot = rDot + ADot*FOB;
      if (W->ImplicitMuscles) {
         P->Bis = rBis + ABis*FOB - ADot*ADot*ROB;
      }

      for (vector<BodyPoint *>::const_iterator Q = P->AttachedPoints.begin();
           Q != P->AttachedPoints.end(); Q ++) {
         RigidBody *R = (*Q)->Parent;

         (*Q)->Val = P->Val;
         (*Q)->Dot = P->Dot;
         if (W->ImplicitMuscles) {
            (*Q)->Bis = P->Bis;
         }

         R->AVal = AVal; R->ADot = ADot; R->ABis = ABis;

         R->BuildSweep(x, *Q);

         KVal += R->KVal;
         KDot += R->KDot;
         if (W->ImplicitMuscles) {
            KBis += R->KBis;
         }
         TotM += R->TotM;
         TotF += R->TotF;
         FVal += R->FVal;

         PVal += R->PVal;
         UVal += R->UVal;
         if (W->ImplicitMuscles) {
            PDot += R->PDot;
            UDot += R->UDot;
         }
      }
      FVal += P->TotF * P->Val.flipped();
      TotF += P->TotF;
   }

//   cerr << "Done with recursion for " << Name << " ...";

   AVec dVal = (KVal - TotM*Entry->Val).flipped();

   Angle->qCurvature = W->G*dVal.y - Entry->Dot.flipped()*KDot;

   AVec entryValFlip = Entry->Val.flipped();
   Angle->qMomentum = UVal + PVal - entryValFlip*KDot;
   if (W->ImplicitMuscles) {
      Angle->QVal += UDot + PDot - entryValFlip*KBis - W->G*dVal.y;
   }
//   if (FVal - tmp*TotF != 0) {
//      cerr << "FORCE: " << FVal - tmp*TotF << "\n";
//   }
   Angle->QVal += FVal - entryValFlip*TotF;
//   cerr << "Done!\n";
}
