# pragma once

# include <string>
# include <map>

# include "AVec.h"

class BodyPoint;
class World;
class DOF;

typedef map< string, BodyPoint *, less<string> > PointMap;

class RigidBody {
public:
   World *const W;
   const double	Mass, ROG2;

   const char *const Name;
   DOF *const Angle;
   PointMap Points;

   double TotM;
   adouble  UVal, UDot;
   adouble  PVal, PDot;
   AVec     KVal, KDot, KBis;
   AVec     rVal, rDot, rBis;
   adouble  AVal, ADot, ABis;
   adouble  FVal, JVal;
   AVec     TotF, TotJ;

   adouble sinVal, cosVal;

   RigidBody(World *const w, const char *const S, DOF *const d,
	     double M, double rog2) :
      W(w),
      Name(S),
      Points(),
      Mass(M),
      ROG2(rog2),
      Angle(d)
   {};

   virtual int BodyType() const = 0;

   BodyPoint *MakePoint(const char *const S, double x, double y, double z = 0);
   BodyPoint *GetPoint(const char *const S);

   void CleanSweep();
   void BuildSweep(const adoublev &x, const BodyPoint *const Entry);

   void DistImpulse(const adoublev &x);

   void RotateInto(const AVec &a, AVec &b) {
      b.x =  cosVal*a.x - sinVal*a.y;
      b.y =  sinVal*a.x + cosVal*a.y;
   }
   void FlipInto(const AVec &a, AVec &b) {
      b.x = -a.y;
      b.y =  a.x;
   }
};  
