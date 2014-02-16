# ifndef RIGIDBODY_H
# define RIGIDBODY_H

# include <string>
# include <map>
# include <adouble.h>

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
   adoublev KVal, KDot, KBis;
   adoublev rVal, rDot, rBis;
   adouble  AVal, ADot, ABis;
   adouble  FVal, JVal;
   adoublev TotF, TotJ;

   adouble sinVal, cosVal;

   RigidBody(World *const w, const char *const S, DOF *const d,
	     double M, double rog2) :
      W(w),
      Name(S),
      Points(),
      Mass(M),
      ROG2(rog2),
      Angle(d),
      KVal(2), KDot(2), KBis(2),
      rVal(2), rDot(2), rBis(2),
      TotF(2), TotJ(2)
   {};

   virtual int BodyType() const = 0;

   BodyPoint *MakePoint(const char *const S, double x, double y, double z = 0);
   BodyPoint *GetPoint(const char *const S);

   void CleanSweep();
   void BuildSweep(const adoublev &x, const BodyPoint *const Entry);

   void DistImpulse(const adoublev &x);

   void RotateInto(const adoublev &x, adoublev &y) {
      y[0] =  cosVal*x[0] - sinVal*x[1];
      y[1] =  sinVal*x[0] + cosVal*x[1];
   }
   void FlipInto(const adoublev &x, adoublev &y) {
      y[0] = -x[1];
      y[1] =  x[0];
   }
};  
# endif

