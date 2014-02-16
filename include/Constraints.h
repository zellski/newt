# ifndef CONSTRAINTS_H
# define CONSTRAINTS_H

# include <Omu_Vector.h>
# include <adouble.h>

class World;
class Stage;
class AnchorPoint;
class DOF;
class RigidBody;

class Constraint {
public:
   virtual void Initialize(Omu_Vector &x, Omu_Vector &c) = 0;
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
   ValConstraint(Stage *const s, int n, double loct, const adouble &watch,
		 const adouble &diff);

   void Initialize(Omu_Vector &x, Omu_Vector &c);
   void Evaluate(const adoublev &x, adoublev &c);
};

class VecConstraint: public Constraint {
private:
   double MinX, MaxX, MinY, MaxY;
   const adoublev &Watch;
   const adoublev * const WatchDiff;
   const int Slice;
   const double t;
   const int cIx;
public:
   Stage *const S;

   VecConstraint(Stage *const s, int n, double loct, const adoublev &watch,
		 double minx, double maxx, double miny, double maxy);
   VecConstraint(Stage *const s, int n, double loct, const adoublev &watch,
		 double xval, double yval);
   VecConstraint(Stage *const s, int n, double loct, const adoublev &watch,
		 const adoublev &diff);

   void Initialize(Omu_Vector &x, Omu_Vector &c);
   void Evaluate(const adoublev &x, adoublev &c);
};

# endif
