# include "RIBVisualizer.h"
# include "DOF.h"
# include "Stage.h"
# include "World.h"
# include "Creature.h"
# include "BodyPoint.h"
# include "Primitives.h"

void RIBVisualizer::Generate(World *const W, const adoublev &x, int frames) {
   static char buf[256];
   static ofstream RIB;
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

//   cerr << "Starting rendering...\n";

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
   RIB.open("../res/snapshot.dat");
   RIB << "Projection \"perspective\" \"fov\" 45\n"
       << "PixelSamples 1 1\n"
       << "Translate 0 0 7\n"
       << "LightSource \"ambientlight\" 1 \"intensity\" 0.4\n"
       << "LightSource \"distantlight\" 1 \"from\" [0 1 -4] \"to\" [0 0 0] \"intensity\" 0.8 \n";


   // sweep through the full time-interval at a constant step


   double tBase = 0;
   vector<Stage *>::const_iterator S = W->Stages.begin();

   for (int frame = 0; frame <= frames; frame ++) {
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
//	 cerr << "Doing hacky nonsense...\n";
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

      RIB << "FrameBegin " << frame << "\n"
	  << "Display \"frame" << frame << ".tiff\" \"file\" \"rgb\"\n"
	  << "WorldBegin\n"
	  << "AttributeBegin\n"
	  << "Color [1 .45 .06]\n"
	  << "Surface \"shinymetal\" \"Kd\" 0.2 \"Ks\" 0.8 \"roughness\" 0.5\n";

      for (uint i = 0; i < W->Creatures.size(); i ++) {
	 Render(W->Creatures[i], RIB);
      }

      RIB << "AttributeEnd\n"
	  << "WorldEnd\n"
	  << "FrameEnd\n";
   }


   RIB.close();
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
//   cerr << "Rendering done.\n";
}

void RIBVisualizer::Render(const Creature *C, ofstream &RIB) {
   RIB << "TransformBegin\n";
   RIB << "Translate " << value(C->X->qVal) << " " << value(C->Y->qVal) << " 0\n";
   Render((const AnchorPoint *) C, RIB);
   RIB << "TransformEnd\n";
}

void RIBVisualizer::Render(const BodyPoint *P, ofstream &RIB) {
   RigidBody *Mom = P->Parent;
   RIB << "Rotate " << 180*value(Mom->Angle->qVal)/M_PI << " 0 0 1\n";
   RIB << "Translate " << -value(P->LocPos[0]) << " " << -value(P->LocPos[1]) << " 0\n";
   Render(Mom, RIB);

   for (PointMap::const_iterator p = Mom->Points.begin();
	p != Mom->Points.end(); p ++) {
      RIB << "TransformBegin\n";
      RIB << "Translate " << value(((*p).second)->LocPos[0]) << " " << value(((*p).second)->LocPos[1]) << " 0\n";      
      Render((const AnchorPoint *) ((*p).second), RIB);
      RIB << "TransformEnd\n";
   }
}

void RIBVisualizer::Render(const AnchorPoint *P, ofstream &RIB) {
   for (vector<BodyPoint *>::const_iterator p = P->AttachedPoints.begin();
	p != P->AttachedPoints.end(); p ++) {
      RIB << "TransformBegin\n";
      Render(*p, RIB);
      RIB << "TransformEnd\n";
   }
}


// OK, this has to be redone :-)

void RIBVisualizer::Render(const RigidBody *B, ofstream &RIB) {
   const Sphere *S = (const Sphere *) B;
   const ThinRod *T = (const ThinRod *) B;
   const Disk *D = (const Disk *) B;
   const Cylinder *C = (const Cylinder *) B;

   switch(B->BodyType()) {
   case BODY_SPHERE:
      RIB << "Sphere " << S->Radius << " " << -S->Radius << " " << S->Radius << " 360\n";
      break;
   case BODY_THINROD:
      RIB << "Rotate 90 0 1 0\n"
	  << "Cylinder " << T->Radius << " " << -.5*T->Length << " " << .5*T->Length << " 360\n"
	  << "Rotate -90 0 1 0\n";
      break;
   case BODY_DISK:
      RIB << "Rotate 90 -1 0 0\n"
	  << "Rotate 90 0 1 0\n"
	  << "Cylinder " << D->Radius << " " << -.5*D->Height << " " << .5*D->Height << " 360\n"
	  << "Disk " << .5*D->Height << " " << D->Radius << " 360\n"
	  << "Rotate -90 0 1 0\n"
	  << "Rotate -90 -1 0 0\n";
      break;
   case BODY_CYLINDER:
      RIB << "Rotate 90 -1 0 0\n"
	  << "Rotate 90 0 1 0\n"
	  << "Cylinder " << C->Radius << " " << -.5*C->Height << " " << .5*C->Height << " 360\n"
	  << "Rotate -90 0 1 0\n"
	  << "Rotate -90 -1 0 0\n";
      break;
   default:
      cerr << "I don't know how to render RigidBody " << B->Name << "!\n";
      assert(0);
   }
}
