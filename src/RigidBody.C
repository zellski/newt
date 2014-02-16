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
      P->TotF[0] = P->TotF[1] = P->TotJ[0] = P->TotJ[1] = 0;
   }
}

void RigidBody::DistImpulse(const adoublev &x) {
   adoublev tmp(2);

   JVal = 0;
   TotJ[0] = TotJ[1] = 0;

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
   adoublev RIB(2), FIB(2), ROB(2), FOB(2);
   adoublev dVal(2), dDot(2);
   adoublev tmp(2);

   AVal += Angle->qVal;
   ADot += Angle->qDot;

   sinVal = sin(AVal); cosVal = cos(AVal);

   RotateInto(Entry->LocPos, RIB);
   FlipInto(RIB, FIB);

   rVal = Entry->Val - RIB;
   rDot = Entry->Dot - ADot*FIB;

   PVal = ADot * Mass * ROG2;

   FlipInto(rVal, tmp);
   UVal = Mass * (rDot*tmp);

   KVal = Mass * rVal;
   KDot = Mass * rDot;

   FVal = 0;
   TotF[0] = TotF[1] = 0;

   TotM = Mass;

//   cerr << "Body " << Name << ": Pass 1...";

   for (PointMap::const_iterator p = Points.begin();
	p != Points.end(); p ++) {
      BodyPoint *P = ((*p).second);
//      cerr << "Fiddling with point " << P->Name << "...\n";
      RotateInto(P->LocPos, ROB);
      FlipInto(ROB, FOB);

      P->Val = rVal + ROB;
      P->Dot = rDot + ADot*FOB;

      for (vector<BodyPoint *>::const_iterator Q = P->AttachedPoints.begin();
	   Q != P->AttachedPoints.end(); Q ++) {
	 RigidBody *R = (*Q)->Parent;

	 (*Q)->Val = P->Val;
	 (*Q)->Dot = P->Dot;

	 R->AVal = AVal; R->ADot = ADot; R->ABis = ABis;

	 R->BuildSweep(x, *Q);

	 KVal += R->KVal;
	 KDot += R->KDot;
	 TotM += R->TotM;
	 TotF += R->TotF;
	 FVal += R->FVal;
	 PVal += R->PVal;
	 UVal += R->UVal;
      }
      FlipInto(P->Val, tmp);
      FVal += P->TotF * tmp;
      TotF += P->TotF;
   }

//   cerr << "Done with recursion for " << Name << " ...";

   tmp = KVal - TotM*Entry->Val;
   FlipInto(tmp, dVal);

   FlipInto(Entry->Dot, tmp);
   Angle->qCurvature = W->G*dVal[1] - tmp*KDot;

   FlipInto(Entry->Val, tmp);
   Angle->qMomentum = UVal + PVal - tmp*KDot;
//   if (FVal - tmp*TotF != 0) {
//      cerr << "FORCE: " << FVal - tmp*TotF << "\n";
//   }
   Angle->QVal += FVal - tmp*TotF;
//   cerr << "Done!\n";
}
