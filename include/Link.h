# ifndef LINK_H
# define LINK_H


# include <Omu_Variables.h>
# include <adouble.h>
# include "Constraints.h"

# include <map>
using namespace std;

class Stage;

class Link: public Constraint {
private:
   Stage *const A;
   Stage *const B;
   int cIx;
public:
   Link(Stage *const a, Stage *const b);
   void Initialize(Omu_VariableVec &x, Omu_VariableVec &c) {}
   void Evaluate(const adoublev &x, adoublev &c);
};

# endif
