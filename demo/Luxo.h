# ifndef LUXO_H
# define LUXO_H

# include <Omu_Program.h>


//--------------------------------------------------------------------------

class World;
class DOF;

class Luxo: public Omu_Program {
private:
   World *W;
public:
   void setup(int k, Omu_VariableVec &x, Omu_VariableVec &u, Omu_VariableVec &c);
   void update(int kk, 
	       const adoublev &x, const adoublev &u,
	       adoublev &f, adouble &f0, adoublev &c);
   const char *name() { return "Luxo"; }
};

#endif

