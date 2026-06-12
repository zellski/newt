/*
**        Unit tests for the YAML -> Spec parse layer and its validation
**        rules. Pure std C++ -- no HQP/ADOL-C stack involved. Also
**        parses any shipped files under ../scenarios as an end-to-end
**        sanity sweep.
*/

# include <cstdio>
# include <cmath>
# include <string>
# include <dirent.h>

# include "newt/SpecParser.h"

using namespace newt;
using std::string;

static int failures = 0;

static void check(const char *what, bool ok) {
   if (!ok) {
      std::fprintf(stderr, "FAIL %s\n", what);
      failures ++;
   }
}

static void checkNum(const char *what, double got, double want) {
   if (!(std::fabs(got - want) <= 1e-15)) {
      std::fprintf(stderr, "FAIL %s: got %.17g, want %.17g\n", what, got, want);
      failures ++;
   }
}

static const char *CREATURE =
   "name: TestBeast\n"
   "root: { x: X, y: Y, attach: Base.Bot }\n"
   "bodies:\n"
   "  - name: Base\n"
   "    type: disk\n"
   "    dof: Alpha\n"
   "    radius: 0.3\n"
   "    height: 0.04\n"
   "    density: 0.2\n"
   "    points:\n"
   "      - { name: Bot, at: [-0.01, 0.0] }\n"
   "      - { name: Top, at: [0.15, 0.0] }\n"
   "  - name: Arm\n"
   "    type: rod\n"
   "    dof: Beta\n"
   "    length: 0.3\n"
   "    points:\n"
   "      - { name: Low, at: [-0.15, 0.0] }\n"
   "      - { name: High, at: [0.15, 0.0] }\n"
   "attachments:\n"
   "  - { parent: Base.Top, child: Arm.Low }\n";

static const char *SCENARIO =
   "name: TestJump\n"
   "creature: beast.creature.yaml\n"
   "gravity: -9.81\n"
   "integrator: { type: gauss, n: 5 }\n"
   "record_dt: 0.02\n"
   "stages:\n"
   "  - name: S1\n"
   "    pieces: 8\n"
   "    duration: 0.2\n"
   "    muscles:\n"
   "      - { dof: Beta, weight: 2 }\n"
   "    reps:\n"
   "      - { dof: X, type: constant, value: 0 }\n"
   "      - { dof: Y, type: constant, value: 0 }\n"
   "      - { dof: Alpha, type: hermlet, from: 0, to: 1.5 }\n"
   "      - { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1 }\n"
   "    impulses:\n"
   "      - { point: Base.Bot, direction: [0, 1] }\n"
   "    constraints:\n"
   "      - { dof: Beta, quantity: q, at: start, equals: 0.5 }\n"
   "      - { dof: Beta, quantity: qdot, at: end, min: -2, max: 2 }\n"
   "  - name: S2\n"
   "    pieces: 4\n"
   "    duration: { min: 0.1, max: 2, start: 0.5 }\n"
   "    reps:\n"
   "      - { dof: X, type: hat, from: 0, to: 1 }\n"
   "      - { dof: Y, type: constant, value: 0 }\n"
   "      - { dof: Alpha, type: constant, value: 1.5 }\n"
   "      - { dof: Beta, type: hermite, from: 0.25, to: 0.5 }\n"
   "impulses:\n"
   "  - { stage: S2, point: Arm.High, direction: [1, 0], magnitude: 3 }\n"
   "links:\n"
   "  - [S1, S2]\n"
   "constraints:\n"
   "  - { stage: S2, dof: Beta, quantity: q, at: { slice: 3, t: 0.5 }, equals: 0.5 }\n";

static void testCreature() {
   CreatureSpec C = ParseCreature(CREATURE, "inline");
   check("creature name", C.name == "TestBeast");
   check("root dofs", C.rootX == "X" && C.rootY == "Y");
   check("root attach", C.rootAttach == "Base.Bot");
   check("two bodies", C.bodies.size() == 2);
   check("disk kind", C.bodies[0].kind == BodySpec::Disk);
   checkNum("disk radius", C.bodies[0].radius, 0.3);
   checkNum("disk height", C.bodies[0].height, 0.04);
   checkNum("disk density", C.bodies[0].density, 0.2);
   check("rod kind", C.bodies[1].kind == BodySpec::Rod);
   checkNum("rod default radius", C.bodies[1].radius, 0.01);
   checkNum("rod default density", C.bodies[1].density, 1.0);
   check("points parsed", C.bodies[0].points.size() == 2);
   checkNum("point coord", C.bodies[0].points[0].x, -0.01);
   check("attachment", C.attachments.size() == 1 &&
         C.attachments[0].parent == "Base.Top" &&
         C.attachments[0].child == "Arm.Low");
}

static void testScenario() {
   ScenarioSpec S = ParseScenario(SCENARIO, "inline");
   check("scenario name", S.name == "TestJump");
   check("creature path", S.creaturePath == "beast.creature.yaml");
   checkNum("gravity", S.gravity, -9.81);
   check("integrator", S.integrator == ScenarioSpec::Gauss && S.integratorN == 5);
   checkNum("record dt", S.recordDt, 0.02);
   check("two stages", S.stages.size() == 2);

   const StageSpec &S1 = S.stages[0];
   check("S1 fixed duration", !S1.variableDuration);
   checkNum("S1 T", S1.T, 0.2);
   check("S1 muscle", S1.muscles.size() == 1 && S1.muscles[0].dof == "Beta");
   checkNum("S1 muscle weight", S1.muscles[0].weight, 2);
   check("S1 reps", S1.reps.size() == 4);
   check("rep types", S1.reps[0].type == RepSpec::Constant &&
         S1.reps[2].type == RepSpec::Hermlet &&
         S1.reps[3].type == RepSpec::Hermite);
   check("hermite bounds", S1.reps[3].hasBounds &&
         S1.reps[3].min == -1 && S1.reps[3].max == 1);
   check("hermlet unbounded", !S1.reps[2].hasBounds);
   check("S1 free impulse", S1.impulses.size() == 1 &&
         !S1.impulses[0].hasMagnitude && S1.impulses[0].sy == 1);
   check("S1 constraints", S1.constraints.size() == 2 &&
         S1.constraints[0].at == ConstraintSpec::Start &&
         !S1.constraints[0].isRange &&
         S1.constraints[1].at == ConstraintSpec::End &&
         S1.constraints[1].isRange &&
         S1.constraints[1].quantity == ConstraintSpec::Dot);

   const StageSpec &S2 = S.stages[1];
   check("S2 variable duration", S2.variableDuration);
   checkNum("S2 start", S2.start, 0.5);
   check("hat rep", S2.reps[0].type == RepSpec::Hat);

   check("top impulse", S.impulses.size() == 1 &&
         S.impulses[0].stage == "S2" && S.impulses[0].hasMagnitude &&
         S.impulses[0].magnitude == 3);
   check("links", S.links.size() == 1 && S.links[0].first == "S1" &&
         S.links[0].second == "S2");
   check("top constraint explicit", S.constraints.size() == 1 &&
         S.constraints[0].at == ConstraintSpec::Explicit &&
         S.constraints[0].slice == 3 && S.constraints[0].t == 0.5);

   // the canonical pair must validate cleanly
   CreatureSpec C = ParseCreature(CREATURE, "inline");
   Validate(S, C);
}

// substitute one line of the canonical scenario and expect rejection
static void expectScenarioFail(const char *what, const string &needle,
                               const string &replacement, bool validateToo = true) {
   string text = SCENARIO;
   string::size_type p = text.find(needle);
   if (p == string::npos) {
      std::fprintf(stderr, "FAIL %s: bad test, needle not found\n", what);
      failures ++;
      return;
   }
   text.replace(p, needle.size(), replacement);
   try {
      ScenarioSpec S = ParseScenario(text, "inline");
      if (validateToo) {
         CreatureSpec C = ParseCreature(CREATURE, "inline");
         Validate(S, C);
      }
      std::fprintf(stderr, "FAIL %s: expected SpecError, got none\n", what);
      failures ++;
   } catch (const SpecError &) {
      // expected
   }
}

static void testRejections() {
   // parse-time
   expectScenarioFail("unknown key", "gravity:", "gravties:");
   expectScenarioFail("bad rep type", "type: hat", "type: spline");
   expectScenarioFail("min without max", "min: -1, max: 1", "min: -1");
   expectScenarioFail("bad quantity", "quantity: qdot", "quantity: momentum");
   expectScenarioFail("zero duration", "duration: 0.2", "duration: 0");
   expectScenarioFail("bad variable duration", "min: 0.1, max: 2, start: 0.5",
                      "min: 0.5, max: 2, start: 0.1");
   expectScenarioFail("zero direction", "direction: [0, 1]", "direction: [0, 0]");
   expectScenarioFail("equals and range", "at: start, equals: 0.5",
                      "at: start, equals: 0.5, min: 0, max: 1");
   expectScenarioFail("bad at", "at: end", "at: middle");

   // validation-time
   expectScenarioFail("missing rep coverage",
                      "      - { dof: Y, type: constant, value: 0 }\n"
                      "      - { dof: Alpha, type: hermlet, from: 0, to: 1.5 }\n",
                      "      - { dof: Alpha, type: hermlet, from: 0, to: 1.5 }\n");
   expectScenarioFail("duplicate rep",
                      "- { dof: Y, type: constant, value: 0 }\n"
                      "      - { dof: Alpha, type: hermlet, from: 0, to: 1.5 }",
                      "- { dof: Y, type: constant, value: 0 }\n"
                      "      - { dof: Y, type: hermlet, from: 0, to: 1.5 }");
   expectScenarioFail("hermlet non-power-of-two", "pieces: 8", "pieces: 6");
   expectScenarioFail("dangling dof", "muscles:\n      - { dof: Beta, weight: 2 }",
                      "muscles:\n      - { dof: Tail, weight: 2 }");
   expectScenarioFail("dangling point", "point: Base.Bot", "point: Base.Nowhere");
   expectScenarioFail("dangling link stage", "- [S1, S2]", "- [S1, S9]");
   expectScenarioFail("slice out of range", "slice: 3", "slice: 4");

   // integrator preconditions (the table-based ctors assert on bad n)
   expectScenarioFail("gauss 6", "type: gauss, n: 5", "type: gauss, n: 6");
   expectScenarioFail("simpson even", "type: gauss, n: 5", "type: simpson, n: 4");
   expectScenarioFail("simpson 1", "type: gauss, n: 5", "type: simpson, n: 1");
}

// substitute one line of the canonical creature and expect rejection
static void expectCreatureFail(const char *what, const string &needle,
                               const string &replacement) {
   string text = CREATURE;
   string::size_type p = text.find(needle);
   if (p == string::npos) {
      std::fprintf(stderr, "FAIL %s: bad test, needle not found\n", what);
      failures ++;
      return;
   }
   text.replace(p, needle.size(), replacement);
   try {
      CreatureSpec C = ParseCreature(text, "inline");
      Validate(ParseScenario(SCENARIO, "inline"), C);
      std::fprintf(stderr, "FAIL %s: expected SpecError, got none\n", what);
      failures ++;
   } catch (const SpecError &) {
      // expected
   }
}

static void testCreatureRejections() {
   expectCreatureFail("duplicate dof", "dof: Beta", "dof: Alpha");

   // attachment topology: BuildSweep recurses without a visited set, so
   // anything but a root-anchored tree must be rejected up front
   expectCreatureFail("self attachment",
                      "{ parent: Base.Top, child: Arm.Low }",
                      "{ parent: Base.Top, child: Base.Bot }");
   expectCreatureFail("double parent",
                      "{ parent: Base.Top, child: Arm.Low }",
                      "{ parent: Base.Top, child: Arm.Low }\n"
                      "  - { parent: Base.Bot, child: Arm.High }");
   expectCreatureFail("root as child",
                      "{ parent: Base.Top, child: Arm.Low }",
                      "{ parent: Base.Top, child: Arm.Low }\n"
                      "  - { parent: Arm.High, child: Base.Top }");
   expectCreatureFail("disconnected body",
                      "attachments:\n"
                      "  - { parent: Base.Top, child: Arm.Low }\n",
                      "");
}

// parse anything shipped under ../scenarios (skip silently if absent)
static void testShippedFiles() {
   DIR *d = opendir("../scenarios");
   if (!d) {
      return;
   }
   while (struct dirent *e = readdir(d)) {
      string n = e->d_name;
      if (n.size() < 5 || n.substr(n.size() - 5) != ".yaml") {
         continue;
      }
      string path = "../scenarios/" + n;
      try {
         ScenarioSpec S = ParseScenarioFile(path);
         CreatureSpec C = ParseCreatureFile("../scenarios/" + S.creaturePath);
         Validate(S, C);
      } catch (const SpecError &e) {
         std::fprintf(stderr, "FAIL shipped %s: %s\n", path.c_str(), e.msg.c_str());
         failures ++;
      }
   }
   closedir(d);
}

int main() {
   try {
      testCreature();
      testScenario();
      testRejections();
      testCreatureRejections();
      testShippedFiles();
   } catch (const SpecError &e) {
      std::fprintf(stderr, "FAIL unexpected SpecError: %s\n", e.msg.c_str());
      failures ++;
   }

   if (failures) {
      std::fprintf(stderr, "test_specparser: %d failure(s)\n", failures);
      return 1;
   }
   std::printf("test_specparser: all tests passed\n");
   return 0;
}
