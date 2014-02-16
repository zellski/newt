# include "Stage.h"
# include <adouble.h>
# include "DOF.h"
# include "Stage.h"
# include "AnchorPoint.h"
# include "RigidBody.h"
# include "Constraints.h"

ValConstraint::ValConstraint(Stage *const s, int n, double loct,
			     const adouble &watch, double min, double max) :
   S(s),
   Slice(n),
   t(loct),
   Watch(watch),
   WatchDiff(0),
   Min(min), Max(max),
   cIx(s->claimCons(1))
{
   cerr << "Constraint: [" << n << "/" << loct << "] -> ["
	<< Min << ", " << Max << "]\n";
   S->Register(this);
}

ValConstraint::ValConstraint(Stage *const s, int n, double loct,
			     const adouble &watch, double val) :
   S(s),
   Slice(n),
   t(loct),
   Watch(watch),
   WatchDiff(0),
   Min(val), Max(val),
   cIx(s->claimCons(1))
{
   cerr << "Constraint: [" << n << "/" << loct << "] -> ["
	<< Min << ", " << Max << "]\n";
   S->Register(this);
}

void ValConstraint::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   c.min[cIx] = Min; c.max[cIx] = Max;
}

void ValConstraint::Evaluate(const adoublev &x, adoublev &c) {
   S->SnapShot(x, Slice, t);
   c[cIx] = Watch;
   if (WatchDiff) {
      c[cIx] -= *WatchDiff;
   }
}

VecConstraint::VecConstraint(Stage *const s, int n, double loct,
			     const adoublev &watch, double minx, double maxx,
			     double miny, double maxy) :
   S(s),
   Slice(n),
   t(loct),
   Watch(watch),
   WatchDiff(0),
   MinX(minx), MaxX(maxx),
   MinY(miny), MaxY(maxy),
   cIx(s->claimCons(2))
{
   cerr << "Constraint: [" << n << "/" << loct << "] -> (["
	<< MinX << ", " << MaxX << "], [" << MinY << ", " << MaxY << "])\n";
   S->Register(this);
}

VecConstraint::VecConstraint(Stage *const s, int n, double loct,
			     const adoublev &watch, double xval, double yval) :
   S(s),
   Slice(n),
   t(loct),
   Watch(watch),
   WatchDiff(0),
   MinX(xval), MaxX(xval),
   MinY(yval), MaxY(yval),
   cIx(s->claimCons(2))
{
   cerr << "Constraint: [" << n << "/" << loct << "] -> (["
	<< MinX << ", " << MaxX << "], [" << MinY << ", " << MaxY << "])\n";
   S->Register(this);
}

VecConstraint::VecConstraint(Stage *const s, int n, double loct,
			     const adoublev &watch, const adoublev &diff) :
   S(s),
   Slice(n),
   t(loct),
   Watch(watch),
   WatchDiff(&diff),
   MinX(0), MaxX(0),
   MinY(0), MaxY(0),
   cIx(s->claimCons(2))
{
   cerr << "Constraint: [" << n << "/" << loct << "] -> (["
	<< MinX << ", " << MaxX << "], [" << MinY << ", " << MaxY << "])\n";
   S->Register(this);
}

void VecConstraint::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   c.min[cIx+0] = MinX; c.max[cIx+0] = MaxX;
   c.min[cIx+1] = MinY; c.max[cIx+1] = MaxY;
}

void VecConstraint::Evaluate(const adoublev &x, adoublev &c) {
   S->SnapShot(x, Slice, t);
   cerr << "Forcing " << Watch << " into ([" << MinX << ", " << MaxX
	<< "], [" << MinY << ", " << MaxY << "])..";
   c[cIx+0] = Watch[0];
   c[cIx+1] = Watch[1];
   if (WatchDiff) {
      cerr << "!!!!";
      c[cIx+0] -= (*WatchDiff)[0];
      c[cIx+1] -= (*WatchDiff)[1];
   }
   cerr << "\n";
}
