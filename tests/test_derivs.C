/*
**        Finite-difference check of the BuildSweep distribution onto the
**        angular DOFs, built from the real RigidBody source against the stub
**        adouble -- no HQP/ADOL-C required.
**
**        BuildSweep hands each angular DOF two generalized quantities that
**        are, by construction, derivatives of scalars the same sweep also
**        accumulates:
**
**          - qCurvature (the gravity term, eq. 4.31) is the derivative of
**            the tree's gravitational potential G*sum(m_i y_i) with respect
**            to that DOF's angle; the sweep leaves sum(m_i y_i) in the
**            root's KVal.y.
**
**          - QVal (the distributed applied force, eq. 4.33-4.35) is the
**            derivative of the virtual work F . p of a constant force at a
**            point p with respect to that DOF's angle.
**
**        With velocities zeroed both reduce to pure configuration gradients,
**        so we can central-difference the scalar by re-running the sweep at
**        perturbed angles and compare against the distributed value. A wrong
**        distribution (mis-pivoted torque, dropped outboard, sign slip) breaks
**        the identity; AD hygiene without the AD stack.
*/

# include <cmath>
# include <cstdio>

# include "World.h"
# include "Stage.h"
# include "DOF.h"
# include "RigidBody.h"
# include "BodyPoint.h"

// DOF.C drags in the real World and Stage; the fields are all we need here
DOF::DOF(World *w, const char *S) : W(w), Name(S) {}

struct TestBody : RigidBody {
   TestBody(World *w, const char *n, DOF *d, double m, double rog2) :
      RigidBody(w, n, d, m, rog2) {}
   int BodyType() const { return -1; }
};

static int failures = 0;

// a relative tolerance is right here: the potentials are O(10), and a
// central difference resolves their gradient to far better than 1e-5
static void checkClose(const char *what, double got, double want,
                       double tol = 1e-5) {
   double scale = std::fabs(want) > 1.0 ? std::fabs(want) : 1.0;
   if (!(std::fabs(got - want) <= tol * scale)) {
      std::fprintf(stderr, "FAIL %s: got %.12g, want %.12g (rel %.3g)\n",
                   what, got, want, std::fabs(got - want) / scale);
      failures ++;
   }
}

// A branched tree rooted at B1: B2 hangs off B1 (chain), B3 off B2, while
// B4 hangs off B1 (branch). Each body's center of mass sits at its origin;
// a constant point load sits on the deepest body B3.
struct Tree {
   static const double FX, FY;        // the constant load on PF

   World W;
   DOF A1, A2, A3, A4;
   TestBody B1, B2, B3, B4;
   BodyPoint *E1, *P12, *P14, *E2, *P23, *E3, *PF, *E4;

   Tree() :
      W(9.81),
      A1(&W, "a1"), A2(&W, "a2"), A3(&W, "a3"), A4(&W, "a4"),
      B1(&W, "b1", &A1, 2.0, 0.5), B2(&W, "b2", &A2, 1.5, 0.4),
      B3(&W, "b3", &A3, 1.0, 0.3), B4(&W, "b4", &A4, 0.8, 0.2)
   {
      E1  = B1.MakePoint("e1",  1.2,  0.0);
      P12 = B1.MakePoint("p12", -0.8, 0.3);
      P14 = B1.MakePoint("p14", 0.4, -0.5);
      E2  = B2.MakePoint("e2",  0.9,  0.0);
      P23 = B2.MakePoint("p23", -0.6, 0.2);
      E3  = B3.MakePoint("e3",  0.7,  0.0);
      PF  = B3.MakePoint("pf",  0.2,  0.5);
      E4  = B4.MakePoint("e4",  0.5,  0.0);

      P12->Attach(E2);
      P23->Attach(E3);
      P14->Attach(E4);
   }

   DOF *dof(int i) {
      DOF *d[4] = { &A1, &A2, &A3, &A4 };
      return d[i];
   }

   void setConfig(double a1, double a2, double a3, double a4) {
      A1.qVal = a1; A2.qVal = a2; A3.qVal = a3; A4.qVal = a4;
      A1.qDot = A2.qDot = A3.qDot = A4.qDot = 0;   // isolate configuration terms
   }

   // common per-sweep reset: root pivot fixed, angles accumulate from zero,
   // velocities zero so only the configuration terms survive
   void prime() {
      B1.AVal = 0; B1.ADot = 0;
      E1->Val.set(0.3, -0.2);
      E1->Dot.set(0, 0);
   }

   // re-run the sweep at the current angles; the gravitational potential
   // G * sum(m_i y_i) lands in the root's KVal.y
   double gravPotential() {
      adoublev x;
      prime();
      B1.CleanSweep();
      B1.BuildSweep(x, E1);
      return W.G * B1.KVal.y.value();
   }

   // re-run the sweep with the constant load applied; returns the virtual
   // work F . p at the loaded point, whose angle-gradient is the DOF's QVal
   double forceWork() {
      adoublev x;
      prime();
      A1.QVal = A2.QVal = A3.QVal = A4.QVal = 0;   // BuildSweep accumulates
      B1.CleanSweep();                             // zeroes PF->TotF
      PF->TotF.set(FX, FY);                        // the distributed load
      B1.BuildSweep(x, E1);
      return (PF->TotF * PF->Val).value();
   }
};

const double Tree::FX = 0.6;
const double Tree::FY = -1.3;

// central-difference a scalar of one DOF's angle, restoring the angle after
template <class Fn>
static double gradWrt(Tree &T, int i, Fn potential, double h = 1e-5) {
   double q0 = T.dof(i)->qVal.value();
   T.dof(i)->qVal = q0 + h;  double up = potential();
   T.dof(i)->qVal = q0 - h;  double dn = potential();
   T.dof(i)->qVal = q0;
   return (up - dn) / (2.0 * h);
}

static void checkConfig(const char *tag,
                        double a1, double a2, double a3, double a4) {
   Tree T;
   const char *names[4] = { "a1", "a2", "a3", "a4" };

   // gravity curvature distributed onto each DOF vs d/dtheta of the potential
   T.setConfig(a1, a2, a3, a4);
   T.gravPotential();
   double curv[4] = { T.A1.qCurvature.value(), T.A2.qCurvature.value(),
                      T.A3.qCurvature.value(), T.A4.qCurvature.value() };
   for (int i = 0; i < 4; i ++) {
      double fd = gradWrt(T, i, [&] { return T.gravPotential(); });
      char what[64];
      std::snprintf(what, sizeof what, "%s curvature dof %s", tag, names[i]);
      checkClose(what, curv[i], fd);
   }

   // applied-force torque distributed onto each DOF vs d/dtheta of the work
   T.setConfig(a1, a2, a3, a4);
   T.forceWork();
   double qval[4] = { T.A1.QVal.value(), T.A2.QVal.value(),
                      T.A3.QVal.value(), T.A4.QVal.value() };
   for (int i = 0; i < 4; i ++) {
      double fd = gradWrt(T, i, [&] { return T.forceWork(); });
      char what[64];
      std::snprintf(what, sizeof what, "%s force torque dof %s", tag, names[i]);
      checkClose(what, qval[i], fd);
   }
}

int main() {
   checkConfig("rest",   0.0,  0.0,  0.0,  0.0);
   checkConfig("bent",   0.4, -0.7,  1.1, -0.5);
   checkConfig("folded", 1.3,  0.6, -0.9,  0.8);

   if (failures) {
      std::fprintf(stderr, "test_derivs: %d failure(s)\n", failures);
      return 1;
   }
   std::printf("test_derivs: all tests passed\n");
   return 0;
}
