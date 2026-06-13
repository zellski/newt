# include <sstream>

# include "Stage.h"
# include "World.h"
# include "DOF.h"
# include "Creature.h"
# include "Fun.h"
# include "newt/Sweep.h"

World *World::Active = 0;

World::World(double g) :
   Creatures(),
   DOFs(),
   Stages(),
   xIx(0), cIx(0),
   G(g)
{
   Active = this;
};

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

// blocks of more than one row get "label[i]" so every row stays unique
static void appendLabels(std::vector<std::string> &labels, int n,
                         const std::string &label) {
   if (n == 1) {
      labels.push_back(label);
      return;
   }
   for (int i = 0; i < n; i ++) {
      std::ostringstream o;
      o << label << "[" << i << "]";
      labels.push_back(o.str());
   }
}

int World::claimVars(int n, const std::string &label) {
   assert(n > 0);
   int tmp = xIx;
   xIx += n;
   appendLabels(xLabels, n, label);
   return tmp;
}

int World::claimCons(int n, const std::string &label) {
   assert(n > 0);
   int tmp = cIx;
   cIx += n;
   appendLabels(cLabels, n, label);
   return tmp;
}

void World::LabelVar(int ix, const std::string &label) {
   assert(ix >= 0 && ix < (int) xLabels.size());
   xLabels[ix] = label;
}

void World::LabelCon(int ix, const std::string &label) {
   assert(ix >= 0 && ix < (int) cLabels.size());
   cLabels[ix] = label;
}

void World::Update(const adoublev &x, adoublev &c, adouble &f0) {
   f0 = 0;
   for (int i = 0; i < c.sz(); i ++) {
      c[i] = 0;
   }
   for (uint i = 0; i < Stages.size(); i ++) {
      Stages[i]->Update(x, c, f0);
   }
   if (newt::Sweeper::requested) {
      newt::Sweeper::Run(this, x);
   }
}

void World::Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {
   assert((int) xLabels.size() == xIx);
   assert((int) cLabels.size() == cIx);
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
