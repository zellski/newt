# include "VRMLVisualizer.h"
# include "DOF.h"
# include "Stage.h"
# include "World.h"
# include "Creature.h"
# include "BodyPoint.h"
# include "Primitives.h"

void VRMLVisualizer::Generate(World *const W, const adoublev &x, int frames) {
   static char buf[256];
   static ofstream VRML;
   static ofstream TOut;
   static ofstream *qOut, *QOut, *qDot, *qC, *qM;

   double T = 0;

   for (vector<Stage *> :: const_iterator S = W->Stages.begin();
	S < W->Stages.end(); S ++) {
      if (value((*S)->T) <= 0) {
	 return;
      }
      T += value((*S)->T);
   }

   cerr << "Starting rendering...\n";

   qOut = new ofstream[W->DOFs.size()];
   qDot = new ofstream[W->DOFs.size()];
   QOut = new ofstream[W->DOFs.size()];

   qC = new ofstream[W->DOFs.size()];
   qM = new ofstream[W->DOFs.size()];

   TOut.open("../res/TDat.m");
   TOut << "T = [ \n";
   for (uint i = 0; i < W->DOFs.size(); i ++) {
      DOF *D = W->DOFs[i];
      sprintf(buf, "../res/q%sDat.m", D->Name);
      qOut[i].open(buf);
      qOut[i] << "q" << D->Name << " = [ \n";
      sprintf(buf, "../res/q%sDotDat.m", D->Name);
      qDot[i].open(buf);
      qDot[i] << "q" << D->Name << "Dot = [ \n";
      sprintf(buf, "../res/Q%sDat.m", D->Name);
      QOut[i].open(buf);
      QOut[i] << "Q" << D->Name << " = [ \n";

      sprintf(buf, "../res/qC%sDat.m", D->Name);
      qC[i].open(buf);
      qC[i] << "qC" << D->Name << " = [ \n";
      sprintf(buf, "../res/qM%sDat.m", D->Name);
      qM[i].open(buf);
      qM[i] << "qM" << D->Name << " = [ \n";
   }

   rename("../res/snapshot.dat", "../res/snapshot.foo");
   VRML.open("../res/snapshot.dat");
   VRML << "#VRML V2.0 utf8\n\n"
	<< "Group {\n"
	<< "children [\n"
	<< "DEF Clock TimeSensor {\n"
	<< "cycleInterval 3\n"
	<< "loop TRUE\n"
	<< "},\n";

   // sweep through the full time-interval at a constant step

   double tBase = 0;
   vector<Stage *>::const_iterator S = W->Stages.begin();

//   for (int frame = 0; frame <= frames; frame ++) {
   int frame = 10;
      double t = T*frame/frames;

      if (frame < frames) {
	 // did we skip into a new Stage? value(T) guaranteed > 0
	 while (t > tBase + value((*S)->T)) {
	    tBase += value((*S)->T);
	    S ++;
	 }
	 double tt = (t - tBase)/value((*S)->h);
	 if (tt >= (*S)->N) {
	    (*S)->SnapShot(x, (*S)->N-1, 1);
	 } else {
	    (*S)->SnapShot(x, (int) tt, tt - (int) tt);
	 }
      } else {
	 // hacky hacky hacky :)
	 cerr << "Doing hacky nonsense...\n";
	 Stage *J = W->Stages[W->Stages.size()-1];
	 J->SnapShot(x, J->N-1, 1);
      }

//      TOut << value(W->T) << "\n";
      TOut << "0\n";
      for (uint i = 0; i < W->DOFs.size(); i ++) {
	 DOF *D = W->DOFs[i];
	 qOut[i] << value(D->qVal) << "\n";
	 qDot[i] << value(D->qDot) << "\n";
	 QOut[i] << value(D->QVal) << "\n";

	 qC[i] << value(D->qCurvature) << "\n";
	 qM[i] << value(D->qMomentum) << "\n";
      }

      for (uint i = 0; i < W->Creatures.size(); i ++) {
	 Render(W->Creatures[i], VRML);
      }
//   }

   VRML << "]}\n";
   VRML.close();

   TOut << "];\n";
   TOut.close();
   for (uint i = 0; i < W->DOFs.size(); i ++) {
      DOF *D = W->DOFs[i];
      qOut[i] << "];\n";
      qDot[i] << "];\n";
      QOut[i] << "];\n";

      qC[i] << "];\n";
      qM[i] << "];\n";

      qOut[i].close();
      qDot[i].close();
      QOut[i].close();
      qC[i].close();
      qM[i].close();
   }
   delete[] qOut;
   delete[] qDot;
   delete[] QOut;
   delete[] qC;
   delete[] qM;
   cerr << "Rendering done.\n";
}

void VRMLVisualizer::Render(const Creature *C, ofstream &VRML) {
   VRML << "DEF Pos" << (PosCnt+++) << " PositionInterpolator {\n" ...

	<< "Transform {\n"
	<< "translation " << value(C->X->qVal) << " " << value(C->Y->qVal) << " 0\n";
   Render((const AnchorPoint *) C, VRML);
   VRML << "}\n";
}

void VRMLVisualizer::Render(const BodyPoint *P, ofstream &VRML) {
   RigidBody *Mom = P->Parent;
   VRML << "Transform {\n"
	<< "rotation 0 0 1 " << value(Mom->Angle->qVal) << "\n"
	<< "children Transform {\n"
	<< "translation " << -value(P->LocPos[0]) << " " << -value(P->LocPos[1]) << " 0\n"
	<< "children [\n";
   Render(Mom, VRML);
   for (PointMap::const_iterator p = Mom->Points.begin();
	p != Mom->Points.end(); p ++) {
      const BodyPoint *Q = (*p).second;
      if (Q->AttachedPoints.begin() != Q->AttachedPoints.end()) {
	 VRML << "Transform {\n";
	 VRML << "translation " << value(Q->LocPos[0]) << " " << value(Q->LocPos[1]) << " 0\n";      
	 Render((const AnchorPoint *) Q, VRML);
	 VRML << "}\n";
      }
   }
   VRML << "]}}\n";
}

void VRMLVisualizer::Render(const AnchorPoint *P, ofstream &VRML) {
   VRML << "children [\n";
   for (vector<BodyPoint *>::const_iterator p = P->AttachedPoints.begin();
	p != P->AttachedPoints.end(); p ++) {
      Render(*p, VRML);
   }
   VRML << "]\n";
}


// OK, this has to be redone :-)

void VRMLVisualizer::Render(const RigidBody *B, ofstream &VRML) {
   const Sphere *S = (const Sphere *) B;
   const ThinRod *T = (const ThinRod *) B;
   const Disk *D = (const Disk *) B;
   const Cylinder *C = (const Cylinder *) B;

   switch(B->BodyType()) {
   case BODY_SPHERE:
      VRML << "Shape {\n"
	   << "geometry Sphere { radius " << S->Radius << " }\n"
	   << "appearance Appearance { material Material { diffuseColor 1 0 0 } }\n"
	   << "}\n";
      break;
   case BODY_THINROD:
      VRML << "Transform {\n"
	   << "rotation 0 0 1 " << (M_PI/2) << "\n"
	   << "children Shape {\n"
	   << "geometry Cylinder { radius " << T->Radius << " height " << T->Length << " }}}\n";
      break;
   case BODY_DISK:
      VRML << "Transform {\n"
	   << "rotation 0 0 1 " << (M_PI/2) << "\n"
	   << "children Shape {\n"
	   << "geometry Cylinder { radius " << D->Radius << " height " << D->Height << " }}}\n";
      break;
   default:
      cerr << "I don't know how to render RigidBody " << B->Name << "!\n";
      assert(0);
   }
}
