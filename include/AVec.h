/**
 * A planar adouble vector.
 */

# pragma once

# include "adolc.h"

class AVec {
  public:
  adouble x, y;

  AVec();
  AVec(const adouble, const adouble);

  void zero();
  void set(const adouble&, const adouble&);

  AVec& operator = (const AVec&);
  AVec& operator += (const AVec&);
  AVec& operator -= (const AVec&);
  AVec& operator *= (const adouble&);
};

adouble operator * (const AVec&, const AVec&);
AVec operator + (const AVec&, const AVec&);
AVec operator - (const AVec&, const AVec&);
AVec operator * (const adouble&, const AVec&);
AVec operator * (const AVec&, const adouble&);

std::ostream& operator << (std::ostream&, const AVec&);
