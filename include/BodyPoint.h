# ifndef BODYPOINT_H
# define BODYPOINT_H

# include <adouble.h>
# include "AnchorPoint.h"

class RigidBody;

class BodyPoint: public AnchorPoint {
public:
   RigidBody *const Parent;
   adoublev LocPos;		// our own coordinate system -- constant

   BodyPoint(const char *const S, RigidBody *const Mom,
	     double x, double y, double z = 0) :
      AnchorPoint(S),
      Parent(Mom),
      LocPos(2)
   {
	 assert(Mom != 0);
	 LocPos[0] = x;
	 LocPos[1] = y;
   }
};

# endif
