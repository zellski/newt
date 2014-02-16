# ifndef CREATURE_H
# define CREATURE_H

# include "AnchorPoint.h"

class World;
class DOF;

class Creature : public AnchorPoint {
public:
   World *const W;
   const char *const Name;
   DOF *const X, *const Y;

   Creature(World *const w, const char *const S, DOF *x, DOF *y);
   void CleanSweep();
   void BuildSweep(const adoublev &x);

   void DistImpulse(const adoublev &x);
};

# endif
