# ifndef LINK_H
# define LINK_H

# include <map>
# include <Omu_Vector.h>
# include <adouble.h>
# include "Constraints.h"

class Stage;

class Link: public Constraint {
private:
   Stage *const A;
   Stage *const B;
   int cIx;
public:
   Link(Stage *const a, Stage *const b);
   void Initialize(Omu_Vector &x, Omu_Vector &c) {}
   void Evaluate(const adoublev &x, adoublev &c);
};

# endif
