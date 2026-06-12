/*
**        Test stand-in for the real Stage. The integrators only call
**        Stage::FEMPoint, so this version just records the (t, weight)
**        samples for the test to inspect.
*/

# pragma once

# include <vector>
# include "adolc.h"

class Stage {
public:
   struct Sample {
      double t, weight;
   };
   std::vector<Sample> samples;

   void FEMPoint(const int slice, const double t, const double weight,
                 const adoublev &x, adoublev &c, adouble &f0) {
      samples.push_back(Sample{t, weight});
   }
};
