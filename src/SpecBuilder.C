# include "newt/SpecBuilder.h"

# include <cstring>
# include <map>
# include <sstream>

# include "World.h"
# include "DOF.h"
# include "Creature.h"
# include "Primitives.h"
# include "BodyPoint.h"
# include "Stage.h"
# include "Muscle.h"
# include "Constant.h"
# include "Hermite.h"
# include "Hermlet.h"
# include "Hat.h"
# include "PWL.h"
# include "Link.h"
# include "Impulse.h"
# include "Constraints.h"
# include "SimpsonIntegrator.h"
# include "GaussianIntegrator.h"

using namespace newt;
using std::string;
using std::map;

// the solver classes keep const char* names for the process lifetime
static const char *dup(const string &s) {
   return strdup(s.c_str());
}

typedef map<string, DOF *> DofMap;
typedef map<string, BodyPoint *> PtMap;
typedef map<string, Stage *> StMap;

// "<stage>: <dof> q at start = -1.963" -- the row's provenance in full
static string conLabel(const ConstraintSpec &K, Stage *S) {
   std::ostringstream o;
   o << S->Name << ": " << K.dof << " "
     << (K.quantity == ConstraintSpec::Val ? "q" : "qdot") << " at ";
   switch (K.at) {
   case ConstraintSpec::Start:
      o << "start";
      break;
   case ConstraintSpec::End:
      o << "end";
      break;
   case ConstraintSpec::Explicit:
      o << "slice " << K.slice << " t=" << K.t;
      break;
   }
   if (K.isRange) {
      o << " in [" << K.min << ", " << K.max << "]";
   } else {
      o << " = " << K.equals;
   }
   return o.str();
}

static void buildConstraint(const ConstraintSpec &K, Stage *S, DOF *D) {
   int slice = 0;
   double t = 0;
   switch (K.at) {
   case ConstraintSpec::Start:
      slice = 0; t = 0;
      break;
   case ConstraintSpec::End:
      slice = S->N - 1; t = 1;
      break;
   case ConstraintSpec::Explicit:
      slice = K.slice; t = K.t;
      break;
   }
   const adouble &watch = K.quantity == ConstraintSpec::Val ? D->qVal : D->qDot;
   if (K.isRange) {
      new ValConstraint(S, slice, t, watch, K.min, K.max, conLabel(K, S));
   } else {
      new ValConstraint(S, slice, t, watch, K.equals, conLabel(K, S));
   }
}

static void buildImpulse(const ImpulseSpec &I, Stage *S, AnchorPoint *P) {
   if (I.hasMagnitude) {
      new Impulse(S, P, I.sx, I.sy, I.magnitude);
   } else {
      new Impulse(S, P, I.sx, I.sy,
                  string(S->Name) + ": impulse @" + I.point + " magnitude");
   }
}

World *newt::BuildWorld(const ScenarioSpec &S, const CreatureSpec &C,
                        Omu_VariableVec &x, Omu_VariableVec &c) {
   World *W = new World(S.gravity);
   DofMap dofs;
   PtMap points;
   StMap stages;

   DOF *X = new DOF(W, dup(C.rootX));
   DOF *Y = new DOF(W, dup(C.rootY));
   dofs[C.rootX] = X;
   dofs[C.rootY] = Y;
   Creature *Cr = new Creature(W, dup(C.name), X, Y);

   for (size_t i = 0; i < C.bodies.size(); i ++) {
      const BodySpec &B = C.bodies[i];
      DOF *D = new DOF(W, dup(B.dof));
      dofs[B.dof] = D;

      RigidBody *R = 0;
      switch (B.kind) {
      case BodySpec::Sphere:
         R = new Sphere(W, dup(B.name), D, B.radius, B.density);
         break;
      case BodySpec::Rod:
         R = new ThinRod(W, dup(B.name), D, B.length, B.radius, B.density);
         break;
      case BodySpec::Disk:
         R = new Disk(W, dup(B.name), D, B.radius, B.height, B.density);
         break;
      case BodySpec::Cylinder:
         R = new Cylinder(W, dup(B.name), D, B.radius, B.height, B.density);
         break;
      }
      for (size_t j = 0; j < B.points.size(); j ++) {
         const BodySpec::Point &P = B.points[j];
         points[B.name + "." + P.name] = R->MakePoint(dup(P.name), P.x, P.y);
      }
   }

   W->Register(Cr);
   Cr->Attach(points[C.rootAttach]);
   for (size_t i = 0; i < C.attachments.size(); i ++) {
      points[C.attachments[i].parent]->Attach(points[C.attachments[i].child]);
   }

   Integrator *I = S.integrator == ScenarioSpec::Simpson
      ? (Integrator *) new SimpsonIntegrator(S.integratorN)
      : (Integrator *) new GaussianIntegrator(S.integratorN);

   // stage-level construction order is parity-critical: duration variable
   // (Stage ctor), muscles, reps, nested impulses, nested constraints
   for (size_t i = 0; i < S.stages.size(); i ++) {
      const StageSpec &st = S.stages[i];
      Stage *Sg = st.variableDuration
         ? new Stage(W, dup(st.name), I, st.pieces, st.min, st.max, st.start)
         : new Stage(W, dup(st.name), I, st.pieces, st.T);
      stages[st.name] = Sg;

      for (size_t j = 0; j < st.muscles.size(); j ++) {
         const string &dof = st.muscles[j].dof;
         new Muscle(Sg, dofs[dof],
                    new PWL(Sg, st.name + ": " + dof + " torque coeffs"),
                    st.muscles[j].weight);
      }
      for (size_t j = 0; j < st.reps.size(); j ++) {
         const RepSpec &R = st.reps[j];
         DOF *D = dofs[R.dof];
         switch (R.type) {
         case RepSpec::Constant:
            new Constant(Sg, D, R.value);
            break;
         case RepSpec::Hermite:
            if (R.hasBounds) {
               new Hermite(Sg, D, R.from, R.to, R.min, R.max);
            } else {
               new Hermite(Sg, D, R.from, R.to);
            }
            break;
         case RepSpec::Hermlet:
            if (R.hasBounds) {
               new Hermlet(Sg, D, R.from, R.to, R.min, R.max);
            } else {
               new Hermlet(Sg, D, R.from, R.to);
            }
            break;
         case RepSpec::Hat:
            if (R.hasBounds) {
               new Hat(Sg, D, R.from, R.to, R.min, R.max);
            } else {
               new Hat(Sg, D, R.from, R.to);
            }
            break;
         }
      }
      for (size_t j = 0; j < st.impulses.size(); j ++) {
         buildImpulse(st.impulses[j], Sg, points[st.impulses[j].point]);
      }
      for (size_t j = 0; j < st.constraints.size(); j ++) {
         buildConstraint(st.constraints[j], Sg, dofs[st.constraints[j].dof]);
      }
   }

   // then: top-level impulses, links, top-level constraints (Needle order)
   for (size_t i = 0; i < S.impulses.size(); i ++) {
      buildImpulse(S.impulses[i], stages[S.impulses[i].stage],
                   points[S.impulses[i].point]);
   }
   if (S.chainLinks) {
      for (size_t i = 0; i + 1 < S.stages.size(); i ++) {
         new Link(stages[S.stages[i].name], stages[S.stages[i + 1].name]);
      }
   }
   for (size_t i = 0; i < S.links.size(); i ++) {
      new Link(stages[S.links[i].first], stages[S.links[i].second]);
   }
   for (size_t i = 0; i < S.constraints.size(); i ++) {
      buildConstraint(S.constraints[i], stages[S.constraints[i].stage],
                      dofs[S.constraints[i].dof]);
   }

   W->Initialize(x, c);
   return W;
}
