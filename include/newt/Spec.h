/*
**        Plain-data descriptions of creatures and scenarios, as parsed
**        from the YAML files. Deliberately free of any solver types so
**        the parse layer can be built and tested without HQP/ADOL-C.
*/

# pragma once

# include <string>
# include <vector>
# include <utility>

namespace newt {

// thrown by the parser/validator; msg carries "<file>: <path>: <problem>"
struct SpecError {
   std::string msg;
   SpecError(const std::string &m) : msg(m) {}
};

struct BodySpec {
   enum Kind { Sphere, Rod, Disk, Cylinder };
   Kind kind;
   std::string name, dof;
   double radius = 0, length = 0, height = 0, density = 1;

   struct Point {
      std::string name;
      double x = 0, y = 0;
   };
   std::vector<Point> points;
};

struct CreatureSpec {
   std::string name;
   std::string rootX, rootY;        // DOF names claimed by the Creature anchor
   std::string rootAttach;          // "Body.Point" the anchor attaches to
   std::vector<BodySpec> bodies;    // order defines DOF registration order

   struct Attachment {
      std::string parent, child;    // "Body.Point" each
   };
   std::vector<Attachment> attachments;
};

struct RepSpec {
   enum Type { Constant, Hermite, Hermlet, Hat };
   Type type;
   std::string dof;
   double value = 0;                // Constant
   double from = 0, to = 0;         // the others
   bool hasBounds = false;
   double min = 0, max = 0;

   // pin sugar: lowered to ConstraintSpecs at the end of parseStage.
   // q defaults to from/to (value for Constant), qdot to 0
   struct Pin {
      bool on = false;
      bool hasQ = false, hasQdot = false;
      double q = 0, qdot = 0;
   };
   Pin pinStart, pinEnd;
};

struct MuscleSpec {
   std::string dof;
   double weight = 1;               // fun is always PWL in v1
};

struct ImpulseSpec {
   std::string stage;               // empty when nested inside a stage
   std::string point;               // "Body.Point"
   double sx = 0, sy = 0;
   bool hasMagnitude = false;       // absent => free magnitude (claims a var)
   double magnitude = 0;
};

struct ConstraintSpec {
   std::string stage;               // empty when nested inside a stage
   std::string dof;
   enum Quantity { Val, Dot };
   Quantity quantity = Val;
   enum At { Start, End, Explicit };
   At at = Start;
   int slice = 0;                   // Explicit only
   double t = 0;                    // Explicit only
   bool isRange = false;
   double equals = 0, min = 0, max = 0;
   bool fromPin = false;            // desugared from a rep's pin: entry
};

struct StageSpec {
   std::string name;
   int pieces = 0;
   bool variableDuration = false;
   double T = 0;                    // fixed duration
   double min = 0, max = 0, start = 0;  // variable duration
   std::vector<MuscleSpec> muscles;
   std::vector<RepSpec> reps;
   std::vector<ImpulseSpec> impulses;
   std::vector<ConstraintSpec> constraints;
};

struct ScenarioSpec {
   std::string name;
   std::string creaturePath;        // relative to the scenario file's directory
   double gravity = 0;
   enum Integrator { Simpson, Gauss };
   Integrator integrator = Gauss;
   int integratorN = 5;
   double recordDt = 0.01;
   std::vector<StageSpec> stages;
   std::vector<ImpulseSpec> impulses;       // built after all stages
   bool chainLinks = false;                 // links: chain sugar
   std::vector<std::pair<std::string, std::string> > links;
   std::vector<ConstraintSpec> constraints; // built last
};

}
