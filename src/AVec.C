# include "AVec.h"

AVec::AVec(const adouble x, const adouble y) :
   x(x),
   y(y)
{
}

AVec::AVec() {
   AVec(0, 0);
}

void AVec::zero() {
   set(0, 0);
}

void AVec::set(const adouble& x, const adouble& y) {
   this->x = x;
   this->y = y;
}

AVec& AVec::operator = (const AVec& o) {
   x = o.x;
   y = o.y;
   return *this;
}

AVec& AVec::operator += (const AVec& o) {
   x += o.x;
   y += o.y;
   return *this;
}

AVec& AVec::operator -= (const AVec& o) {
   x -= o.x;
   y -= o.y;
   return *this;
}

AVec& AVec::operator *= (const adouble& k) {
   x *= k;
   y *= k;
   return *this;
}

adouble operator * (const AVec& a, const AVec& b) {
   return a.x*b.x + a.y*a.y;
}

AVec operator + (const AVec& a, const AVec& b) {
   return AVec(a.x+b.x, a.y+b.y);
}

AVec operator - (const AVec& a, const AVec& b) {
   return AVec(a.x-b.x, a.y-b.y);
}

AVec operator * (const AVec& v, const adouble& k) {
   return AVec(k*v.x, k*v.y);
}

AVec operator * (const adouble& k, const AVec& v) {
   return AVec(k*v.x, k*v.y);
}

//--------------------------------------------------------------------------
std::ostream& operator << (std::ostream& out, const AVec &v) {
   out << "A[" << v.x.value() << ", " << v.y.value() << "]";
  return out;
}

