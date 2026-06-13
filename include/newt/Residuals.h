/*
**        Captures the evaluated constraint vector at the current
**        iterate so a failed solve can be reported against World's
**        labeled rows. Same arming dance as the Sweeper: the caller
**        arms the flag and asks HQP for a values-only re-evaluation
**        (prg_update_fbd); the capture runs inside World::Update with
**        that evaluation's c.
*/

# pragma once

# include <vector>
# include "adolc.h"

class World;

namespace newt {

class Residuals {
public:
   static bool requested;
   static bool valid;
   static std::vector<double> c;    // evaluated rows, c[0 .. W->cIx)

   // fills c and clears `requested`; called from World::Update
   static void Capture(World *W, const adoublev &c);
};

}
