# ifndef ANCHORPOINT_H
# define ANCHORPOINT_H

# include <stdio.h>
# include <vector>
# include <adouble.h>

/*
**	This class represents a point in the World to which other
**	objects may attach. For example, the Stationary Creature
**	-is- an AP, and RigidBody can provide any number of AP's
**	on request, in the form of BodyPoints.
*/

class adouble;
class adoublev;

class BodyPoint;
class Stage;

class Force;
class Impulse;

class AnchorPoint {
public:
   const char *const Name;
   vector<BodyPoint *> AttachedPoints;
   adoublev TotF, TotJ;
   adoublev Val, Dot, Bis;

   AnchorPoint(const char *const S);

   void Attach(BodyPoint *const P);
};
# endif
