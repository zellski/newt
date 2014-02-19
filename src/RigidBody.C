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
   AVec tmp;

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
      FlipInto(P->Val, tmp);
      JVal += P->TotJ * tmp;
      TotJ += P->TotJ;
//      cerr << "Added JVal " << P->TotJ*tmp << " and TotJ " << P->TotJ << "\n";
   }
   Angle->JVal += JVal - tmp*TotJ;
//   cerr << "Finally, Angle->Jval: " << JVal - tmp*TotJ << "...\n";
}


void RigidBody::BuildSweep(const adoublev &x, const BodyPoint *const Entry) {
   AVec RIB, FIB, ROB, FOB;
   AVec dVal, dDot;
   AVec tmp;

   AVal += Angle->qVal;
   ADot += Angle->qDot;
   if (W->ImplicitMuscles) {
      ABis += Angle->qBis;
   }

   sinVal = sin(AVal); cosVal = cos(AVal);

   RotateInto(Entry->LocPos, RIB);
   FlipInto(RIB, FIB);

   rVal = Entry->Val - RIB;
   rDot = Entry->Dot - ADot*FIB;
   if (W->ImplicitMuscles) {
      rBis = Entry->Bis - ABis*FIB + ADot*ADot*RIB;
   }

   PVal = ADot * Mass * ROG2;
   if (W->ImplicitMuscles) {
      PDot = ABis * Mass * ROG2;
   }

   FlipInto(rVal, tmp);
   UVal = Mass * (rDot*tmp);
   if (W->ImplicitMuscles) {
      UDot = Mass * (rBis*tmp);
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
      RotateInto(P->LocPos, ROB);
      FlipInto(ROB, FOB);

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
      FlipInto(P->Val, tmp);
      FVal += P->TotF * tmp;
      TotF += P->TotF;
   }

//   cerr << "Done with recursion for " << Name << " ...";

   tmp = KVal - TotM*Entry->Val;
   FlipInto(tmp, dVal);

   FlipInto(Entry->Dot, tmp);
   Angle->qCurvature = W->G*dVal.y - tmp*KDot;

   FlipInto(Entry->Val, tmp);
   Angle->qMomentum = UVal + PVal - tmp*KDot;
   if (W->ImplicitMuscles) {
      Angle->QVal += UDot + PDot - tmp*KBis - W->G*dVal.y;
   }
//   if (FVal - tmp*TotF != 0) {
//      cerr << "FORCE: " << FVal - tmp*TotF << "\n";
//   }
   Angle->QVal += FVal - tmp*TotF;
//   cerr << "Done!\n";
}
