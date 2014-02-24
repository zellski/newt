# include "RIBVisualizer.h"
# include "Stage.h"
# include "DOF.h"
# include "World.h"
# include "Creature.h"
# include "BodyPoint.h"
# include "Primitives.h"

# include <sstream>

using namespace std;

void RIBVisualizer::Generate(World *const W, const adoublev &x) {
   static char buf[256];
   static std::ofstream RIB;
   static std::ofstream TOut;
   static std::stringstream *qOut, *QOut, *qDot, *qC, *qM;

   double T = 0;

   for (vector<Stage *> :: const_iterator S = W->Stages.begin();
        S < W->Stages.end(); S ++) {
      if ((*S)->T.value() <= 0) {
         return;
      }
      T += (*S)->T.value();
   }

//   cerr << "Starting rendering...\n";

   qOut = new std::stringstream[W->DOFs.size()];
   qDot = new std::stringstream[W->DOFs.size()];
   QOut = new std::stringstream[W->DOFs.size()];

   qC = new std::stringstream[W->DOFs.size()];
   qM = new std::stringstream[W->DOFs.size()];

   TOut.open("../res/TDat.m");
   TOut << "T = [ \n";

   for (uint i = 0; i < W->DOFs.size(); i ++) {
      DOF *D = W->DOFs[i];
      qOut[i] << "q" << D->Name << " = [ \n";
      qDot[i] << "q" << D->Name << "Dot = [ \n";
      QOut[i] << "Q" << D->Name << " = [ \n";

      qC[i] << "qC" << D->Name << " = [ \n";
      qM[i] << "qM" << D->Name << " = [ \n";
   }
   rename("../res/snapshot.dat", "../res/snapshot.old");
   RIB.open("../res/snapshot.dat");
   RIB << "Projection \"perspective\" \"fov\" 45\n"
       << "PixelSamples 1 1\n"
       << "Translate 0 0 7\n"
       << "LightSource \"ambientlight\" 1 \"intensity\" 0.4\n"
       << "LightSource \"distantlight\" 1 \"from\" [0 1 -4] \"to\" [0 0 0] \"intensity\" 0.8 \n";

   // sweep through the full time-interval at a constant step

   double tBase = 0;
   vector<Stage *>::const_iterator S = W->Stages.begin();

   int frame = 0;
   for (double t = 0.0; t <= T; t += 0.01) {
       // did we skip into a new Stage? value(T) guaranteed > 0
       while (t > tBase + (*S)->T.value()) {
           tBase += (*S)->T.value();
           S ++;
       }
       double tt = (t - tBase)/((*S)->h).value();
       if (tt >= (*S)->N) {
           (*S)->SnapShot(x, (*S)->N-1, 1);
       } else {
           (*S)->SnapShot(x, (int) tt, tt - (int) tt);
       }

//      TOut << value(W->T) << "\n";
      TOut << "0\n";
      for (uint i = 0; i < W->DOFs.size(); i ++) {
         DOF *D = W->DOFs[i];
         qOut[i] << (D->qVal).value() << "\n";
         qDot[i] << (D->qDot).value() << "\n";
         QOut[i] << (D->QVal).value() << "\n";

         qC[i] << D->qCurvature.value() << "\n";
         qM[i] << D->qMomentum.value()<< "\n";
      }

      RIB << "FrameBegin " << frame << "\n"
          << "Display \""
          << (frame < 100 ? (frame < 10 ? "00" : "0") : "") << frame
          << ".tiff\" \"file\" \"rgb\"\n"
          << "WorldBegin\n"
          << "AttributeBegin\n"
          << "Color [1 .45 .06]\n"
          << "Surface \"metal\" \"Ka\" 0.5 \"Ks\" 0.8 \"roughness\" 0.5\n";

      for (uint i = 0; i < W->Creatures.size(); i ++) {
         Render(W->Creatures[i], RIB);
      }

      RIB << "AttributeEnd\n"
          << "WorldEnd\n"
          << "FrameEnd\n";
      frame ++;
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

      qOut[i].flush();
      qDot[i].flush();
      QOut[i].flush();
      qC[i].flush();
      qM[i].flush();

      std::ofstream DOFOut;
      sprintf(buf, "../res/%sDat.m", D->Name);
      DOFOut.open(buf);
      DOFOut << qOut[i].str() << qDot[i].str() << QOut[i].str() << qC[i].str() << qM[i].str();
      DOFOut.close();
   }
   delete[] qOut;
   delete[] qDot;
   delete[] QOut;
   delete[] qC;
   delete[] qM;
//   cerr << "Rendering done.\n";
}

void RIBVisualizer::Render(const Creature *C, std::ofstream &RIB) {
   RIB << "TransformBegin\n";
   RIB << "Translate " << C->X->qVal.value() << " " << C->Y->qVal.value() << " 0\n";
   Render((const AnchorPoint *) C, RIB);
   RIB << "TransformEnd\n";
}

void RIBVisualizer::Render(const BodyPoint *P, std::ofstream &RIB) {
   RigidBody *Mom = P->Parent;
   RIB << "Rotate " << 180*(Mom->Angle->qVal).value()/M_PI << " 0 0 1\n";
   RIB << "Translate " << -(P->LocPos.x).value() << " " << -(P->LocPos.y).value() << " 0\n";
   Render(Mom, RIB);

   for (PointMap::const_iterator p = Mom->Points.begin();
        p != Mom->Points.end(); p ++) {
      RIB << "TransformBegin\n";
      RIB << "Translate " << (((*p).second)->LocPos.x).value() << " " << (((*p).second)->LocPos.y).value() << " 0\n";      
      Render((const AnchorPoint *) ((*p).second), RIB);
      RIB << "TransformEnd\n";
   }
}

void RIBVisualizer::Render(const AnchorPoint *P, std::ofstream &RIB) {
   for (vector<BodyPoint *>::const_iterator p = P->AttachedPoints.begin();
        p != P->AttachedPoints.end(); p ++) {
      RIB << "TransformBegin\n";
      Render(*p, RIB);
      RIB << "TransformEnd\n";
   }
}


// OK, this has to be redone :-)

void RIBVisualizer::Render(const RigidBody *B, std::ofstream &RIB) {
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
