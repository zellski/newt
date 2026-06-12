/*
**        Unit tests for the dynamics distribution machinery, built from
**        the real RigidBody/Force/Impulse/AnchorPoint sources against the
**        stub adouble.
**
**        Covered:
**          - Force::SnapShot constant and Fun-driven flavors (the constant
**            flavor was silently never applied before the bugfix sweep)
**          - Impulse::SnapShot fixed and free (optimizer-variable) flavors
**          - RigidBody::DistImpulse over a branched three-joint body tree,
**            checked against independently computed impulse torques
**            (thesis eq. 4.33-4.35 applied to impulses; this replaced an
**            assert(false) left mid-port)
**          - RigidBody::BuildSweep on a single body: angular momentum,
**            gravity curvature term and force distribution (eq. 4.24-4.35)
*/

# include <cmath>
# include <cstdio>

# include "World.h"
# include "Stage.h"
# include "DOF.h"
# include "RigidBody.h"
# include "BodyPoint.h"
# include "Force.h"
# include "Impulse.h"
# include "Fun.h"

// DOF.C drags in the real World and Stage; the fields are all we need here
DOF::DOF(World *w, const char *S) : W(w), Name(S) {}

struct TestBody : RigidBody {
   TestBody(World *w, const char *n, DOF *d, double m, double rog2) :
      RigidBody(w, n, d, m, rog2) {}
   int BodyType() const { return -1; }
};

struct TestFun : Fun {
   const double v;
   TestFun(Stage *s, double val) : Fun(s), v(val) {}
   bool isConstant() { return false; }
   void SnapShot(const adoublev &x, int slice, double t) { Val = v; }
};

static int failures = 0;

static void check(const char *what, double got, double want, double tol = 1e-12) {
   if (!(std::fabs(got - want) <= tol)) {
      std::fprintf(stderr, "FAIL %s: got %.17g, want %.17g\n", what, got, want);
      failures ++;
   }
}

// torque of a planar force/impulse j applied at r, about the pivot: the
// J . E[r] contractions in the sweep must reduce to exactly this
static double torque(const AVec &r, const AVec &j, const AVec &pivot) {
   double rx = r.x.value() - pivot.x.value();
   double ry = r.y.value() - pivot.y.value();
   return rx*j.y.value() - ry*j.x.value();
}

static void testForceAndImpulseApplication() {
   Stage S;
   adoublev x;

   // constant-magnitude force: FVec fixed at construction, must accumulate
   // on every SnapShot
   AnchorPoint P1("p1");
   Force Fc(&S, &P1, 0.4, -1.1, 2.0);
   Fc.SnapShot(x, 0, 0.0);
   check("const force x", P1.TotF.x.value(), 0.8);
   check("const force y", P1.TotF.y.value(), -2.2);
   Fc.SnapShot(x, 0, 0.5);
   check("const force accumulates", P1.TotF.y.value(), -4.4);

   // Fun-driven force: magnitude follows the Fun's snapshot value
   AnchorPoint P2("p2");
   TestFun f(&S, 2.5);
   Force Ff(&S, &P2, 1.0, 0.5, &f);
   Ff.SnapShot(x, 0, 0.0);
   check("fun force x", P2.TotF.x.value(), 2.5);
   check("fun force y", P2.TotF.y.value(), 1.25);

   // fixed-magnitude impulse
   AnchorPoint P3("p3");
   Impulse Jc(&S, &P3, 1.0, 0.0, 3.0);
   Jc.SnapShot(x);
   check("const impulse x", P3.TotJ.x.value(), 3.0);
   check("const impulse y", P3.TotJ.y.value(), 0.0);

   // free impulse: magnitude lives in the optimizer's variable vector
   Impulse Jf(&S, &P3, 0.0, 1.0);
   adoublev x1(1);
   x1[0] = 2.25;
   Jf.SnapShot(x1);
   check("free impulse x", P3.TotJ.x.value(), 3.0);
   check("free impulse y", P3.TotJ.y.value(), 2.25);
}

// A branched tree: B1 is the root (entry E1); B2 hangs off B1 at P12 and
// B3 off B2 at P23 (a chain), while B4 hangs off B1 at P14 (a branch).
// Cartesian impulses sit on points throughout. Each joint's generalized
// impulse must equal the summed torque, about that joint's entry point,
// of every impulse outboard of it.
static void testDistImpulse() {
   World W(9.81);
   DOF A1(&W, "a1"), A2(&W, "a2"), A3(&W, "a3"), A4(&W, "a4");
   TestBody B1(&W, "b1", &A1, 1, 1), B2(&W, "b2", &A2, 1, 1);
   TestBody B3(&W, "b3", &A3, 1, 1), B4(&W, "b4", &A4, 1, 1);

   BodyPoint *E1  = B1.MakePoint("e1", 0, 0);
   BodyPoint *P12 = B1.MakePoint("p12", 0, 0);
   BodyPoint *P14 = B1.MakePoint("p14", 0, 0);
   BodyPoint *E2  = B2.MakePoint("e2", 0, 0);
   BodyPoint *P23 = B2.MakePoint("p23", 0, 0);
   BodyPoint *PJ2 = B2.MakePoint("pj2", 0, 0);
   BodyPoint *E3  = B3.MakePoint("e3", 0, 0);
   BodyPoint *PJ3 = B3.MakePoint("pj3", 0, 0);
   BodyPoint *E4  = B4.MakePoint("e4", 0, 0);
   BodyPoint *PJ4 = B4.MakePoint("pj4", 0, 0);

   P12->Attach(E2);
   P23->Attach(E3);
   P14->Attach(E4);

   // DistImpulse runs after BuildSweep has filled in the world-frame point
   // positions; here we place them by hand (entry points coincide with the
   // parent point they hang from, as BuildSweep guarantees)
   E1->Val.set(0.3, -0.2);
   P12->Val.set(1.1, 0.4);  E2->Val.set(1.1, 0.4);
   P23->Val.set(1.8, 0.9);  E3->Val.set(1.8, 0.9);
   PJ2->Val.set(2.0, -0.5);
   PJ3->Val.set(2.5, 1.5);
   P14->Val.set(-0.6, 0.1); E4->Val.set(-0.6, 0.1);
   PJ4->Val.set(-1.2, -0.7);

   B1.CleanSweep();
   PJ2->TotJ.set(0.7, 1.3);
   PJ3->TotJ.set(-0.4, 0.6);
   PJ4->TotJ.set(0.2, -0.9);
   P12->TotJ.set(0.5, 0.25);
   E1->TotJ.set(1.0, -1.0);   // at the root pivot itself: zero torque on A1

   A1.JVal = 0; A2.JVal = 0; A3.JVal = 0; A4.JVal = 0;

   adoublev x;
   B1.DistImpulse(x, E1);

   check("A3 impulse",
         A3.JVal.value(),
         torque(PJ3->Val, PJ3->TotJ, E3->Val));
   check("A2 impulse",
         A2.JVal.value(),
         torque(PJ2->Val, PJ2->TotJ, E2->Val) +
         torque(PJ3->Val, PJ3->TotJ, E2->Val));
   check("A4 impulse",
         A4.JVal.value(),
         torque(PJ4->Val, PJ4->TotJ, E4->Val));
   check("A1 impulse",
         A1.JVal.value(),
         torque(E1->Val,  E1->TotJ,  E1->Val) +
         torque(P12->Val, P12->TotJ, E1->Val) +
         torque(PJ2->Val, PJ2->TotJ, E1->Val) +
         torque(PJ3->Val, PJ3->TotJ, E1->Val) +
         torque(PJ4->Val, PJ4->TotJ, E1->Val));

   // the root's accumulated TotJ is the net Cartesian impulse on the tree
   check("net impulse x", B1.TotJ.x.value(), 1.0 + 0.5 + 0.7 - 0.4 + 0.2);
   check("net impulse y", B1.TotJ.y.value(), -1.0 + 0.25 + 1.3 + 0.6 - 0.9);
}

// One body pivoting about the world origin at angle 0: a rod of mass M
// with its entry point at body-frame (le, 0), so its center of mass sits
// at world (-le, 0), plus a constant force on a second point. Momentum,
// the gravity term and the distributed force torque all have closed forms.
static void testBuildSweep() {
   const double M = 2.0, ROG2 = 0.5, le = 1.5, omega = 0.8, G = 9.81;

   World W(G);
   DOF A(&W, "a");
   TestBody B(&W, "b", &A, M, ROG2);
   BodyPoint *E  = B.MakePoint("e", le, 0);
   BodyPoint *PF = B.MakePoint("pf", -0.5, 0.3);

   Stage S;
   Force F(&S, PF, 0.4, -1.1, 1.0);

   adoublev x;
   B.CleanSweep();
   F.SnapShot(x, 0, 0.0);

   E->Val.set(0, 0);
   E->Dot.set(0, 0);
   B.AVal = 0;
   B.ADot = 0;
   A.qVal = 0;
   A.qDot = omega;
   A.QVal = 0;

   B.BuildSweep(x, E);

   // world-frame position of the force's point of attack
   check("point val x", PF->Val.x.value(), -le - 0.5);
   check("point val y", PF->Val.y.value(), 0.3);

   // angular momentum about the pivot: M (le^2 + rog^2) omega
   check("momentum", A.qMomentum.value(), M*(le*le + ROG2)*omega);

   // gravity curvature term (eq. 4.31): M G x_com, with x_com = -le
   check("curvature", A.qCurvature.value(), M*G*(-le));

   // distributed force: torque of the constant force about the pivot
   AVec pivot(0, 0);
   check("force torque", A.QVal.value(), torque(PF->Val, PF->TotF, pivot));
}

int main() {
   testForceAndImpulseApplication();
   testDistImpulse();
   testBuildSweep();

   if (failures) {
      std::fprintf(stderr, "test_dynamics: %d failure(s)\n", failures);
      return 1;
   }
   std::printf("test_dynamics: all tests passed\n");
   return 0;
}
