/*
**        Unit tests for the AVec planar vector algebra. These exercise
**        exactly the operations the dynamics sweep depends on; the dot
**        product and default-constructor cases are regression tests for
**        bugs introduced in the adoublev(2) -> AVec port.
*/

# include <cmath>
# include <cstdio>

# include "AVec.h"

static int failures = 0;

static void check(const char *what, double got, double want, double tol = 1e-12) {
   if (!(std::fabs(got - want) <= tol)) {
      std::fprintf(stderr, "FAIL %s: got %.17g, want %.17g\n", what, got, want);
      failures ++;
   }
}

int main() {
   // default constructor must zero both components
   AVec z;
   check("AVec().x", z.x.value(), 0);
   check("AVec().y", z.y.value(), 0);

   AVec a(3, 4), b(-2, 5);

   // dot product: 3*-2 + 4*5 = 14
   check("dot", (a*b).value(), 14);
   check("dot symmetric", (b*a).value(), 14);
   check("norm2", (a*a).value(), 25);

   // flipped() is a 90-degree CCW rotation (the E matrix of the thesis)
   AVec f = a.flipped();
   check("flipped.x", f.x.value(), -4);
   check("flipped.y", f.y.value(), 3);
   // E[v] . v = 0 for all v
   check("flip orthogonal", (f*a).value(), 0);

   // rotated() by +pi/2 must agree with flipped()
   adouble quarter = M_PI/2;
   AVec r = a.rotated(quarter);
   check("rotated.x", r.x.value(), -4, 1e-12);
   check("rotated.y", r.y.value(), 3, 1e-12);
   // rotation preserves length
   adouble third = M_PI/3;
   AVec r3 = a.rotated(third);
   check("rotation isometry", (r3*r3).value(), 25, 1e-12);

   // arithmetic
   AVec s = a + b;
   check("sum.x", s.x.value(), 1);
   check("sum.y", s.y.value(), 9);
   AVec d = a - b;
   check("diff.x", d.x.value(), 5);
   check("diff.y", d.y.value(), -1);
   AVec k = 2*a;
   check("scale.x", k.x.value(), 6);
   check("scale.y", k.y.value(), 8);
   AVec k2 = a*2;
   check("scale2.x", k2.x.value(), 6);

   AVec c(1, 1);
   c += a;
   check("+=.x", c.x.value(), 4);
   c -= b;
   check("-=.y", c.y.value(), 0);
   c *= 3;
   check("*=.x", c.x.value(), 18);

   c.zero();
   check("zero.x", c.x.value(), 0);
   check("zero.y", c.y.value(), 0);

   if (failures) {
      std::fprintf(stderr, "test_avec: %d failure(s)\n", failures);
      return 1;
   }
   std::printf("test_avec: all tests passed\n");
   return 0;
}
