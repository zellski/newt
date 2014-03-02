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
   const double        Mass, ROG2;

   const char *const Name;
   DOF *const Angle;
   PointMap Points;

   double TotM;
   adouble  UVal;
   adouble  PVal;
   AVec     KVal, KDot;
   AVec     rVal, rDot;
   adouble  AVal, ADot;
   adouble  FVal, JVal;
   AVec     TotF, TotJ;

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
};  
