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
# include "newt/Census.h"

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

// substitute one line of the canonical scenario and parse
static ScenarioSpec parseWith(const char *what, const string &needle,
                              const string &replacement) {
   string text = SCENARIO;
   string::size_type p = text.find(needle);
   if (p == string::npos) {
      std::fprintf(stderr, "FAIL %s: bad test, needle not found\n", what);
      failures ++;
      throw SpecError("bad test");
   }
   text.replace(p, needle.size(), replacement);
   return ParseScenario(text, "inline");
}

// route an expression through a numeric field and return its value
static double evalViaGravity(const string &expr) {
   string text = SCENARIO;
   const string needle = "gravity: -9.81";
   text.replace(text.find(needle), needle.size(), "gravity: " + expr);
   return ParseScenario(text, "inline").gravity;
}

static void testExpressions() {
   // the exact 17-digit literals the shipped scenarios used to carry
   checkNum("pi", evalViaGravity("pi"), 3.141592653589793);
   checkNum("PI/2", evalViaGravity("PI/2"), 1.5707963267948966);
   checkNum("3*pi/11", evalViaGravity("3*pi/11"), 0.8567979964335799);
   checkNum("-5*pi/8", evalViaGravity("-5*pi/8"), -1.9634954084936207);
   checkNum("15*pi/16", evalViaGravity("15*pi/16"), 2.945243112740431);
   checkNum("-7*pi/11", evalViaGravity("-7*pi/11"), -1.9991953250116865);
   checkNum("8*pi/17", evalViaGravity("8*pi/17"), 1.478396542865785);
   checkNum("7*pi/16 + 2*pi", evalViaGravity("7*pi/16 + 2*pi"), 7.6576320931251205);
   checkNum("pi/8 + 2*pi", evalViaGravity("pi/8 + 2*pi"), 6.675884388878311);
   checkNum("sqrt(1.5*2.0/9.81)", evalViaGravity("sqrt(1.5*2.0/9.81)"),
            0.553001263609331);
   checkNum("1/3", evalViaGravity("1/3"), 0.3333333333333333);
   checkNum("1/7", evalViaGravity("1/7"), 0.14285714285714285);

   // grammar mechanics
   checkNum("parens", evalViaGravity("(1 + 2) * (3 - 4)"), -3.0);
   checkNum("nested negation", evalViaGravity("--2"), 2.0);
   checkNum("left assoc", evalViaGravity("8/4/2"), 1.0);
   checkNum("quoted", evalViaGravity("\"-(1 + 2) * 3\""), -9.0);
   checkNum("whitespace", evalViaGravity("\"  2 * pi  \""), 6.283185307179586);

   expectScenarioFail("unknown name", "gravity: -9.81", "gravity: tau", false);
   expectScenarioFail("trailing junk", "gravity: -9.81", "gravity: pi pi", false);
   expectScenarioFail("unbalanced parens", "gravity: -9.81", "gravity: (1+2", false);
   expectScenarioFail("division by zero", "gravity: -9.81", "gravity: 1/0", false);
   expectScenarioFail("sqrt of negative", "gravity: -9.81", "gravity: sqrt(0-1)", false);
   expectScenarioFail("empty expression", "gravity: -9.81", "gravity: \"\"", false);
   expectScenarioFail("dangling operator", "gravity: -9.81", "gravity: 1+", false);
   expectScenarioFail("bare operator", "gravity: -9.81", "gravity: \"*\"", false);
}

static const char *BETA_REP =
   "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1 }";

static void testPins() {
   {
      ScenarioSpec S = parseWith("pin both", BETA_REP,
         "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
         "          pin: both }");
      const StageSpec &S1 = S.stages[0];
      check("pins prepended", S1.constraints.size() == 6);
      const ConstraintSpec &q0 = S1.constraints[0];
      check("pin q start", q0.fromPin && q0.dof == "Beta" &&
            q0.quantity == ConstraintSpec::Val &&
            q0.at == ConstraintSpec::Start && !q0.isRange);
      checkNum("pin q start defaults to from", q0.equals, 0.5);
      const ConstraintSpec &d0 = S1.constraints[1];
      check("pin qdot start", d0.fromPin &&
            d0.quantity == ConstraintSpec::Dot &&
            d0.at == ConstraintSpec::Start);
      checkNum("pin qdot defaults to 0", d0.equals, 0);
      const ConstraintSpec &q1 = S1.constraints[2];
      check("pin q end", q1.fromPin && q1.quantity == ConstraintSpec::Val &&
            q1.at == ConstraintSpec::End);
      checkNum("pin q end defaults to to", q1.equals, 0.25);
      check("pin qdot end", S1.constraints[3].at == ConstraintSpec::End &&
            S1.constraints[3].quantity == ConstraintSpec::Dot);
      check("explicit constraints follow",
            !S1.constraints[4].fromPin && !S1.constraints[5].fromPin &&
            S1.constraints[4].at == ConstraintSpec::Start);
      CreatureSpec C = ParseCreature(CREATURE, "inline");
      Validate(S, C);
   }
   {
      ScenarioSpec S = parseWith("pin overrides", BETA_REP,
         "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
         "          pin: { start: { q: -1/2, qdot: 0.1 } } }");
      const StageSpec &S1 = S.stages[0];
      check("start-only pin", S1.constraints.size() == 4 &&
            S1.constraints[0].fromPin && !S1.constraints[2].fromPin);
      checkNum("pin q override", S1.constraints[0].equals, -0.5);
      checkNum("pin qdot override", S1.constraints[1].equals, 0.1);
   }
   {
      ScenarioSpec S = parseWith("pin on constant",
         "- { dof: Y, type: constant, value: 0 }\n      - { dof: Alpha",
         "- { dof: Y, type: constant, value: 0, pin: start }\n"
         "      - { dof: Alpha");
      const StageSpec &S1 = S.stages[0];
      check("constant pin defaults to value",
            S1.constraints.size() == 4 && S1.constraints[0].dof == "Y" &&
            S1.constraints[0].equals == 0 && S1.constraints[1].equals == 0);
   }

   expectScenarioFail("bad pin token", BETA_REP,
      "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
      "          pin: middle }", false);
   expectScenarioFail("bad pin key", BETA_REP,
      "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
      "          pin: { middle: { q: 0 } } }", false);
   expectScenarioFail("empty pin mapping", BETA_REP,
      "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
      "          pin: {} }", false);
   expectScenarioFail("pin outside rep bounds", BETA_REP,
      "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
      "          pin: { start: { q: 5 } } }", false);
   expectScenarioFail("pin contradicts constant",
      "- { dof: Y, type: constant, value: 0 }\n      - { dof: Alpha",
      "- { dof: Y, type: constant, value: 0, pin: { start: { q: 1 } } }\n"
      "      - { dof: Alpha", false);
}

static void testStructuralValidation() {
   // a constraint that contradicts a constant rep
   expectScenarioFail("contradicts constant",
      "constraints:\n      - { dof: Beta, quantity: q, at: start, equals: 0.5 }",
      "constraints:\n"
      "      - { dof: Y, quantity: q, at: start, equals: 1 }\n"
      "      - { dof: Beta, quantity: q, at: start, equals: 0.5 }");
   // two equalities at the same location with different values
   expectScenarioFail("conflicting duplicates",
      "- { dof: Beta, quantity: q, at: start, equals: 0.5 }",
      "- { dof: Beta, quantity: q, at: start, equals: 0.5 }\n"
      "      - { dof: Beta, quantity: q, at: start, equals: 0.6 }");
   // an equality outside a range at the same location
   expectScenarioFail("equality outside range",
      "- { dof: Beta, quantity: qdot, at: end, min: -2, max: 2 }",
      "- { dof: Beta, quantity: qdot, at: end, min: -2, max: 2 }\n"
      "      - { dof: Beta, quantity: qdot, at: end, equals: 3 }");
   // an identical redundant duplicate stays legal
   {
      ScenarioSpec S = parseWith("redundant duplicate",
         "- { dof: Beta, quantity: q, at: start, equals: 0.5 }",
         "- { dof: Beta, quantity: q, at: start, equals: 0.5 }\n"
         "      - { dof: Beta, quantity: q, at: start, equals: 0.5 }");
      CreatureSpec C = ParseCreature(CREATURE, "inline");
      Validate(S, C);
   }
   // facing boundaries across the S1->S2 link must agree: S1 ends Beta
   // at 0.25, S2 claims it starts at 0.3
   {
      string text = SCENARIO;
      const string n1 = "- { dof: Beta, quantity: qdot, at: end, min: -2, max: 2 }";
      text.replace(text.find(n1), n1.size(),
         "- { dof: Beta, quantity: q, at: end, equals: 0.25 }");
      const string n2 = "  - { stage: S2, dof: Beta, quantity: q, at: { slice: 3, t: 0.5 }, equals: 0.5 }";
      text.replace(text.find(n2), n2.size(),
         "  - { stage: S2, dof: Beta, quantity: q, at: start, equals: 0.3 }");
      try {
         ScenarioSpec S = ParseScenario(text, "inline");
         CreatureSpec C = ParseCreature(CREATURE, "inline");
         Validate(S, C);
         std::fprintf(stderr, "FAIL facing boundaries: expected SpecError\n");
         failures ++;
      } catch (const SpecError &) {
         // expected
      }
   }
   // unanchored DOFs warn: take away Alpha's constant rep and it has
   // no q constraint or constant anywhere
   {
      ScenarioSpec S = parseWith("unanchored dof",
         "- { dof: Alpha, type: constant, value: 1.5 }",
         "- { dof: Alpha, type: hermlet, from: 1.5, to: 1.5 }");
      CreatureSpec C = ParseCreature(CREATURE, "inline");
      std::vector<string> warnings;
      Validate(S, C, &warnings);
      check("unanchored dof warns", warnings.size() == 1 &&
            warnings[0].find("Alpha") != string::npos);
   }
   // the canonical pair is warning-free
   {
      ScenarioSpec S = ParseScenario(SCENARIO, "inline");
      CreatureSpec C = ParseCreature(CREATURE, "inline");
      std::vector<string> warnings;
      Validate(S, C, &warnings);
      check("canonical scenario warning-free", warnings.empty());
   }
}

static void testCensus() {
   ScenarioSpec S = ParseScenario(SCENARIO, "inline");
   CreatureSpec C = ParseCreature(CREATURE, "inline");

   // hand-derived totals for the canonical scenario:
   // vars: S2 duration 1; reps: Alpha hermlet 18 + Beta hermite 18 (S1)
   //       + X hat 5 + Beta hermite 10 (S2) = 51; S1 muscle PWL 16;
   //       S1 free impulse 1  => 69
   // cons: Beta S1 bounds 16; FEM: Alpha 16 + Beta 16 (S1), X 3 +
   //       Beta 8 (S2) = 43; link S1->S2: X, Alpha, Beta -> 6;
   //       explicit constraints 3  => 68
   int nVars, nCons;
   CensusTotals(S, C, nVars, nCons);
   check("census vars", nVars == 69);
   check("census cons", nCons == 68);

   string text = Census(S, C);
   check("census table cells",
         text.find("hermite s[q] e[qd]") != string::npos &&   // S1 Beta
         text.find("hermlet") != string::npos &&              // S1 Alpha
         text.find("hermite m[1]") != string::npos);          // S2 Beta
   check("census links line", text.find("links: S1->S2") != string::npos);
   check("census breakdown",
         text.find("variables: 69") != string::npos &&
         text.find("constraints: 68") != string::npos);

   // pins count separately from explicit constraints
   ScenarioSpec SP = parseWith("census pins", BETA_REP,
      "- { dof: Beta, type: hermite, from: 0.5, to: 0.25, min: -1, max: 1,\n"
      "          pin: both }");
   string ptext = Census(SP, C);
   check("census pin count", ptext.find("pins                  4") != string::npos);
}

static void testChainDefault() {
   {
      ScenarioSpec S = parseWith("chain default", "links:\n  - [S1, S2]\n", "");
      check("multi-stage chains by default", S.chainLinks && S.links.empty());
   }
   {
      ScenarioSpec S = parseWith("chain opt-out", "links:\n  - [S1, S2]\n",
                                 "chain: false\n");
      check("chain: false opts out", !S.chainLinks && S.links.empty());
   }
   {
      // an explicit links: list suppresses the default
      ScenarioSpec S = ParseScenario(SCENARIO, "inline");
      check("explicit links honored", !S.chainLinks && S.links.size() == 1);
   }
   {
      static const char *MONO =
         "name: Mono\n"
         "creature: beast.creature.yaml\n"
         "gravity: 0\n"
         "integrator: { type: gauss, n: 5 }\n"
         "stages:\n"
         "  - name: S1\n"
         "    pieces: 4\n"
         "    duration: 1\n"
         "    reps:\n"
         "      - { dof: X, type: constant, value: 0 }\n"
         "      - { dof: Y, type: constant, value: 0 }\n"
         "      - { dof: Alpha, type: hermite, from: 0, to: 1 }\n"
         "      - { dof: Beta, type: hermite, from: 0, to: 1 }\n";
      ScenarioSpec S = ParseScenario(MONO, "inline");
      check("single stage stays unchained", !S.chainLinks && S.links.empty());
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
      testExpressions();
      testPins();
      testStructuralValidation();
      testCensus();
      testChainDefault();
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
