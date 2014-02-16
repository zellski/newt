# ifndef PWL_H
# define PWL_H

# include <Omu_Vector.h>
# include <adouble.h>
# include "Fun.h"

/*
**	PWLs! Piecewise linear.
*/

class Stage;

class PWL: public Fun {
protected:
   const int xIx;
public:
   PWL(Stage *const s);

   void SnapShot(const adoublev &x, int slice, double t);
   bool isConstant() { return false; }
};

#endif
