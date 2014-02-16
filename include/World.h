# ifndef WORLD_H
# define WORLD_H

# include <Omu_Vector.h>
# include <vector>
# include <adouble.h>

class DOF;
class Constraint;
class Muscle;
class Stage;
class Creature;

class World {
public:
   vector<Creature *> Creatures;
   vector<DOF *> DOFs;
   vector<Stage *> Stages;

   int xIx, cIx;	// allocation indices into Omu_Vector x and c

   const double G;

   World(double g);

   void Update(const adoublev &x, adoublev &c, adouble &f0);
   void Initialize(Omu_Vector &x, Omu_Vector &c);

   int claimVars(int n);
   int claimCons(int n);

   int Register(Creature *C);
   int Register(DOF *D);
   int Register(Stage *S);
};

# endif
