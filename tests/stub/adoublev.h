/*
**        Minimal stand-in for the ADOL-C/HQP adouble types, so that the
**        pure-math parts of NewT can be unit-tested without the solver
**        stack installed. No taping; an adouble is just a double.
**
**        The default constructor poisons its value with NaN to mimic the
**        "uninitialized location" semantics of the real adouble -- tests
**        that pass against this stub would also have caught code relying
**        on zero-initialization.
*/

# pragma once

# include <cmath>
# include <limits>
# include <iostream>

class adouble {
   double v;
public:
   adouble() : v(std::numeric_limits<double>::quiet_NaN()) {}
   adouble(double d) : v(d) {}

   double value() const { return v; }

   adouble& operator += (const adouble& o) { v += o.v; return *this; }
   adouble& operator -= (const adouble& o) { v -= o.v; return *this; }
   adouble& operator *= (const adouble& o) { v *= o.v; return *this; }
   adouble& operator /= (const adouble& o) { v /= o.v; return *this; }

   adouble operator - () const { return adouble(-v); }
};

inline adouble operator + (const adouble& a, const adouble& b) {
   return adouble(a.value() + b.value());
}
inline adouble operator - (const adouble& a, const adouble& b) {
   return adouble(a.value() - b.value());
}
inline adouble operator * (const adouble& a, const adouble& b) {
   return adouble(a.value() * b.value());
}
inline adouble operator / (const adouble& a, const adouble& b) {
   return adouble(a.value() / b.value());
}

inline adouble cos(const adouble& a) { return adouble(std::cos(a.value())); }
inline adouble sin(const adouble& a) { return adouble(std::sin(a.value())); }
inline adouble sqrt(const adouble& a) { return adouble(std::sqrt(a.value())); }

inline std::ostream& operator << (std::ostream& out, const adouble& a) {
   return out << a.value();
}

class adoublev {
public:
};
