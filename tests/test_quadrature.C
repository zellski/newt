/*
**        Unit tests for the numerical integration rules. Each integrator
**        is run against a recording stub Stage and its (t, weight) samples
**        are checked for symmetry and for integrating monomials on [0, 1]
**        exactly up to the rule's design order.
*/

# include <cmath>
# include <cstdio>
# include <vector>

# include "Stage.h"
# include "SimpsonIntegrator.h"
# include "GaussianIntegrator.h"

static int failures = 0;

static void check(const char *rule, const char *what,
                  double got, double want, double tol) {
   if (!(std::fabs(got - want) <= tol)) {
      std::fprintf(stderr, "FAIL %s %s: got %.17g, want %.17g\n",
                   rule, what, got, want);
      failures ++;
   }
}

// integral of t^k over [0,1] is 1/(k+1); a degree-`order` rule must
// reproduce it for all k <= order
static void testRule(const char *name, Integrator *rule,
                     int order, double tol) {
   Stage S;
   adoublev x, c;
   adouble f0;
   rule->integrate(&S, 0, x, c, f0);

   for (int k = 0; k <= order; k ++) {
      double sum = 0;
      for (const Stage::Sample &s : S.samples) {
         sum += s.weight * std::pow(s.t, k);
      }
      char what[64];
      std::snprintf(what, sizeof what, "t^%d", k);
      check(name, what, sum, 1.0/(k+1), tol);
   }

   // points and weights must be symmetric about t = 1/2
   size_t n = S.samples.size();
   for (size_t i = 0; i < n/2; i ++) {
      const Stage::Sample &lo = S.samples[i];
      const Stage::Sample &hi = S.samples[n-1-i];
      check(name, "point symmetry", lo.t, 1.0 - hi.t, 1e-9);
      check(name, "weight symmetry", lo.weight, hi.weight, 1e-9);
   }
}

int main() {
   SimpsonIntegrator simpson5(5), simpson25(25);
   GaussianIntegrator gauss5(5), gauss8(8), gauss10(10);

   // composite Simpson is exact for cubics
   testRule("Simpson(5)", &simpson5, 3, 1e-12);
   testRule("Simpson(25)", &simpson25, 3, 1e-12);

   // n-point Gauss-Legendre is exact to degree 2n-1; the tables are
   // transcribed to 8 decimals, so allow that much slop
   testRule("Gauss(5)", &gauss5, 9, 1e-7);
   testRule("Gauss(8)", &gauss8, 15, 1e-7);
   testRule("Gauss(10)", &gauss10, 19, 1e-7);

   if (failures) {
      std::fprintf(stderr, "test_quadrature: %d failure(s)\n", failures);
      return 1;
   }
   std::printf("test_quadrature: all tests passed\n");
   return 0;
}
