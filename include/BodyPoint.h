# pragma once

# include <assert.h>
# include "adolc.h"
# include "AnchorPoint.h"

class RigidBody;

class BodyPoint: public AnchorPoint {
public:
   RigidBody *const Parent;
   AVec LocPos;		// our own coordinate system -- constant

   BodyPoint(const char *const S, RigidBody *const Mom,
	     double x, double y, double z = 0) :
      AnchorPoint(S),
      Parent(Mom),
      LocPos(x, y)
   {
	 assert(Mom != 0);
   }
};
