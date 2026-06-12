# include "newt/Sweep.h"

# include "World.h"
# include "Stage.h"
# include "DOF.h"

using namespace newt;

bool Sweeper::requested = false;
double Sweeper::dt = 0.01;
SweepResult Sweeper::result;

// the time-grid walk mirrors RIBVisualizer::Generate, which ran this
// sweep on every evaluation for two decades -- keep its exact stage
// advance and end-of-stage clamping
void Sweeper::Run(World *W, const adoublev &x) {
   requested = false;
   result = SweepResult();

   for (int i = 0; i < W->xIx; i ++) {
      result.x.push_back(x[i].value());
   }

   double T = 0;
   bool durationsOk = true;
   for (vector<Stage *>::const_iterator S = W->Stages.begin();
        S < W->Stages.end(); S ++) {
      durationsOk = durationsOk && (*S)->T.value() > 0;
      T += (*S)->T.value();
      result.stageEnds.push_back(T);
   }
   // a variable stage duration driven nonpositive: record metrics and x,
   // but no frames (the grid is meaningless)
   if (!durationsOk) {
      result.valid = true;
      return;
   }

   double tBase = 0;
   vector<Stage *>::const_iterator S = W->Stages.begin();

   for (double t = 0.0; t <= T; t += dt) {
      // did we skip into a new Stage? T value guaranteed > 0
      while (t > tBase + (*S)->T.value()) {
         tBase += (*S)->T.value();
         S ++;
      }
      double tt = (t - tBase)/((*S)->h).value();
      if (tt >= (*S)->N) {
         (*S)->SnapShot(x, (*S)->N - 1, 1);
      } else {
         (*S)->SnapShot(x, (int) tt, tt - (int) tt);
      }

      result.times.push_back(t);
      for (uint i = 0; i < W->DOFs.size(); i ++) {
         result.frames.push_back(W->DOFs[i]->qVal.value());
         result.frames.push_back(W->DOFs[i]->qDot.value());
      }
   }
   result.valid = true;
}
