# pragma once

# include <Omu_Variables.h>
# include "AVec.h"

class World;
class Stage;
class AnchorPoint;
class DOF;
class RigidBody;

class Constraint {
public:
   virtual void Initialize(Omu_VariableVec &x, Omu_VariableVec &c) = 0;
   virtual void Evaluate(const adoublev &x, adoublev &c) = 0;
};

class ValConstraint: public Constraint {
private:
   double Min, Max;
   const adouble &Watch;
   const adouble * const WatchDiff;
   const int Slice;
   const double t;
   const int cIx;
public:
   Stage *const S;

   ValConstraint(Stage *const s, int n, double loct, const adouble &watch,
		 double min, double max);
   ValConstraint(Stage *const s, int n, double loct, const adouble &watch,
		 double val);

   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
   void Evaluate(const adoublev &x, adoublev &c);
};

class VecConstraint: public Constraint {
private:
   double MinX, MaxX, MinY, MaxY;
   const AVec &Watch;
   const AVec * const WatchDiff;
   const int Slice;
   const double t;
   const int cIx;
public:
   Stage *const S;

   VecConstraint(Stage *const s, int n, double loct, const AVec &watch,
		 double minx, double maxx, double miny, double maxy);
   VecConstraint(Stage *const s, int n, double loct, const AVec &watch,
		 double xval, double yval);
   VecConstraint(Stage *const s, int n, double loct, const AVec &watch,
		 const AVec &diff);

   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
   void Evaluate(const adoublev &x, adoublev &c);
};
