# ifndef SNAKE_H
# define SNAKE_H

# include <Omu_Program.h>


//--------------------------------------------------------------------------

class World;

class Snake: public Omu_Program {
private:
   World *W;
public:
   void setup(int k, Omu_Vector &x, Omu_Vector &u, Omu_Vector &c);
   void update(int kk, 
	       const adoublev &x, const adoublev &u,
	       adoublev &f, adouble &f0, adoublev &c);
   char *name() { return "Snake"; }
};

#endif

