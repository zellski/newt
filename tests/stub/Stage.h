/*
**        Test stand-in for the real Stage. The integrators only call
**        Stage::FEMPoint, so this version just records the (t, weight)
**        samples for the test to inspect; Register/claimVars exist so
**        that Force/Impulse/Fun constructors can run unchanged.
*/

# pragma once

# include <string>
# include <vector>
# include "adolc.h"
# include "Omu_Variables.h"

class Stage {
   int nVars;
public:
   struct Sample {
      double t, weight;
   };
   std::vector<Sample> samples;

   Stage() : nVars(0) {}

   template <class T> int Register(T *) { return 0; }

   int claimVars(int n, const std::string & = "?") {
      int ix = nVars;
      nVars += n;
      return ix;
   }

   void FEMPoint(const int slice, const double t, const double weight,
                 const adoublev &x, adoublev &c, adouble &f0) {
      samples.push_back(Sample{t, weight});
   }
};
