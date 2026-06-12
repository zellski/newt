/*
**        Samples the current solution's DOF trajectories on a fixed
**        time grid, for recording. Replaces RIBVisualizer's role of
**        walking the stages with SnapShot -- but runs only when armed,
**        once per accepted SQP iterate, instead of on every objective
**        evaluation.
**
**        The arming dance exists because the authoritative iterate
**        lives inside the solver: the recorder arms the flag and asks
**        HQP for a values-only re-evaluation (prg_update_fbd), and the
**        sweep then runs inside World::Update with that evaluation's x.
*/

# pragma once

# include <vector>
# include "adolc.h"

class World;

namespace newt {

struct SweepResult {
   std::vector<double> x;          // coefficient vector, x[0 .. W->xIx)
   std::vector<double> times;      // global time per frame
   std::vector<double> frames;     // per frame, per DOF: qVal, qDot
   std::vector<double> stageEnds;  // cumulative stage end times
   bool valid = false;
};

class Sweeper {
public:
   static bool requested;
   static double dt;
   static SweepResult result;

   // fills result and clears `requested`; called from World::Update
   static void Run(World *W, const adoublev &x);
};

}
