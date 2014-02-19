# pragma once

# include <Omu_Variables.h>
# include "adolc.h"
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
