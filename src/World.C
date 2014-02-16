# include "World.h"
# include "Stage.h"
# include "DOF.h"
# include "Creature.h"
# include "Fun.h"
# include "Visualizer.h"

World::World(double g) :
   Creatures(),
   DOFs(),
   Stages(),
   xIx(0), cIx(0),
   G(g)
{};

int World::Register(Creature *C) {
   assert(!!C);
   Creatures.push_back(C);
   return Creatures.size() - 1;
}

int World::Register(DOF *D) {
   assert(!!D);
   DOFs.push_back(D);
   return DOFs.size() - 1;
}

int World::Register(Stage *S) {
   assert(!!S);
   Stages.push_back(S);
   return Stages.size() - 1;
}

int World::claimVars(int n) {
   assert(n > 0);
   int tmp = xIx;
   xIx += n;
   return tmp;
}

int World::claimCons(int n) {
   assert(n > 0);
   int tmp = cIx;
   cIx += n;
   return tmp;
}

void World::Update(const adoublev &x, adoublev &c, adouble &f0) {
   f0 = 0;
   for (int i = 0; i < c.sz(); i ++) {
      c[i] = 0;
   }
   for (uint i = 0; i < Stages.size(); i ++) {
      Stages[i]->Update(x, c, f0);
   }
   Visualizer::Generate(this, x, 100);
}

void World::Initialize(Omu_Vector &x, Omu_Vector &c) {
   cerr << " * Allocating " << xIx << " ix for DOFs, "
	<< cIx << " for Constraints.\n";
   x.alloc(xIx);
   c.alloc(cIx);
   cerr << " * Zapping constraints.\n";
   for (int i = 0; i < cIx; i ++) {
      c.min[i] = c.max[i] = 0;
   }
   for (uint i = 0; i < Stages.size(); i ++) {
      Stages[i]->Initialize(x, c);
   }
}   
