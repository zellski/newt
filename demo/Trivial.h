# ifndef SNAKE_H
# define SNAKE_H

# include <Omu_Program.h>


//--------------------------------------------------------------------------

class World;

class Trivial: public Omu_Program {
private:
   World *W;
public:
   void setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c);
   void update(int kk, 
	       const adoublev &x, const adoublev &u,
	       adoublev &f, adouble &f0, adoublev &c);
   const char *name() { return "Trivial"; }
};

#endif

