# ifndef PWC_H
# define PWC_H

# include <Omu_Vector.h>
# include <adouble.h>
# include "Fun.h"

/*
**	PWCs! Piecewise constant.
*/

class Stage;

class PWC: public Fun {
protected:
   const int xIx;
public:
   PWC(Stage *const s);

   bool isConstant() { return true; }

   adouble Val(const adoublev &x, int slice, double t) const;
   adouble Dot(const adoublev &x, int slice, double t) const { return 0; }
   adouble Bis(const adoublev &x, int slice, double t) const { return 0; }
};

#endif
