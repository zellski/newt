# ifndef STAGE_H
# define STAGE_H

# include <adouble.h>
# include <Omu_Variables.h>

# include <vector>
using namespace std;

class World;
class Muscle;
class Constraint;
class Impulse;
class Fun;
class Force;

class Stage {
public:
   World *const W;

   vector<Muscle *> Muscles;
   vector<Force *> Forces;
   vector<Fun *> Funs;
   vector<Impulse *> Impulses;
   vector<Constraint *> Cons;

   const double Min, Max, Start;

   const int N;

   const int sIx;
   const int tIx;

   adouble T;
   adouble h;

   Stage(World *w, int n, double t);
   Stage(World *w, int n, double min, double max, double start);

   int Register(Muscle *M);
   int Register(Force *F);
   int Register(Fun *F);
   int Register(Impulse *I);
   int Register(Constraint *C);

   int claimVars(int n);
   int claimCons(int n);

   void Update(const adoublev &x, adoublev &c, adouble &f0);
   void SnapShot(const adoublev &x, int slice, double t);
   void FEMEquations(const adoublev &x, adoublev &c, adouble &f0);

   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c);
};

# endif
