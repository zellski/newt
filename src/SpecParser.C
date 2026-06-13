# include "newt/SpecParser.h"

# include <fkYAML/node.hpp>

# include <cctype>
# include <cmath>
# include <cstdlib>
# include <fstream>
# include <sstream>
# include <map>
# include <set>

using namespace newt;
using std::string;
using std::vector;
using std::map;
using std::set;

namespace {

typedef fkyaml::node Node;

// error-message context: "<label>: <path>"
struct Ctx {
   string label, path;
   Ctx(const string &l, const string &p = "") : label(l), path(p) {}
   Ctx sub(const string &key) const {
      return Ctx(label, path.empty() ? key : path + "." + key);
   }
   Ctx item(const string &key, size_t i) const {
      std::ostringstream o;
      o << (path.empty() ? "" : path + ".") << key << "[" << i << "]";
      return Ctx(label, o.str());
   }
   void fail(const string &what) const {
      throw SpecError(label + ": " + (path.empty() ? what : path + ": " + what));
   }
};

typedef map<string, const Node *> Fields;

Fields fields(const Node &n, const Ctx &ctx) {
   if (!n.is_mapping()) {
      ctx.fail("expected a mapping");
   }
   Fields f;
   const fkyaml::node::mapping_type &m =
      n.get_value_ref<const fkyaml::node::mapping_type &>();
   for (fkyaml::node::mapping_type::const_iterator p = m.begin();
        p != m.end(); p ++) {
      if (!p->first.is_string()) {
         ctx.fail("mapping keys must be strings");
      }
      f[p->first.get_value<string>()] = &p->second;
   }
   return f;
}

void checkKeys(const Fields &f, const set<string> &allowed, const Ctx &ctx) {
   for (Fields::const_iterator p = f.begin(); p != f.end(); p ++) {
      if (!allowed.count(p->first)) {
         ctx.fail("unknown key '" + p->first + "'");
      }
   }
}

const Node &req(const Fields &f, const string &key, const Ctx &ctx) {
   Fields::const_iterator p = f.find(key);
   if (p == f.end()) {
      ctx.fail("missing required key '" + key + "'");
   }
   return *p->second;
}

const Node *opt(const Fields &f, const string &key) {
   Fields::const_iterator p = f.find(key);
   return p == f.end() ? 0 : p->second;
}

// constant-expression evaluator for float-valued fields: number
// literals, pi/PI, + - * /, unary minus, parens, sqrt(). Deliberately
// a frozen calculator, not a language: no variables, no references to
// other fields, no user-defined names.
struct Expr {
   const char *p, *end;
   const string &src;
   const Ctx &ctx;
   Expr(const string &s, const Ctx &c)
      : p(s.c_str()), end(s.c_str() + s.size()), src(s), ctx(c) {}
   void fail(const string &what) const {
      ctx.fail("bad expression '" + src + "': " + what);
   }
   void ws() {
      while (p < end && std::isspace((unsigned char) *p)) {
         p ++;
      }
   }
   bool eat(char c) {
      ws();
      if (p < end && *p == c) {
         p ++;
         return true;
      }
      return false;
   }
   double sum() {
      double v = term();
      for (;;) {
         if (eat('+')) {
            v += term();
         } else if (eat('-')) {
            v -= term();
         } else {
            return v;
         }
      }
   }
   double term() {
      double v = factor();
      for (;;) {
         if (eat('*')) {
            v *= factor();
         } else if (eat('/')) {
            double d = factor();
            if (d == 0) {
               fail("division by zero");
            }
            v /= d;
         } else {
            return v;
         }
      }
   }
   double factor() {
      ws();
      if (eat('-')) {
         return -factor();
      }
      if (eat('(')) {
         double v = sum();
         if (!eat(')')) {
            fail("expected ')'");
         }
         return v;
      }
      if (p < end && (std::isdigit((unsigned char) *p) || *p == '.')) {
         char *after;
         double v = std::strtod(p, &after);
         if (after == p) {
            fail("malformed number");
         }
         p = after;
         return v;
      }
      if (p < end && std::isalpha((unsigned char) *p)) {
         const char *start = p;
         while (p < end && std::isalpha((unsigned char) *p)) {
            p ++;
         }
         string name(start, p);
         if (name == "pi" || name == "PI") {
            return M_PI;
         }
         if (name == "sqrt") {
            if (!eat('(')) {
               fail("expected '(' after sqrt");
            }
            double v = sum();
            if (!eat(')')) {
               fail("expected ')'");
            }
            if (v < 0) {
               fail("sqrt of a negative number");
            }
            return std::sqrt(v);
         }
         fail("unknown name '" + name + "'");
      }
      fail(p < end ? "unexpected '" + string(1, *p) + "'"
                   : "unexpected end of expression");
      return 0;   // unreached
   }
};

double evalExpr(const string &s, const Ctx &ctx) {
   Expr e(s, ctx);
   double v = e.sum();
   e.ws();
   if (e.p != e.end) {
      e.fail("unexpected trailing '" + string(e.p, e.end) + "'");
   }
   return v;
}

double num(const Node &n, const Ctx &ctx) {
   if (n.is_float_number()) {
      return n.get_value<double>();
   }
   if (n.is_integer()) {
      return (double) n.get_value<long long>();
   }
   if (n.is_string()) {
      return evalExpr(n.get_value<string>(), ctx);
   }
   ctx.fail("expected a number");
   return 0;
}

long long integer(const Node &n, const Ctx &ctx) {
   if (!n.is_integer()) {
      ctx.fail("expected an integer");
   }
   return n.get_value<long long>();
}

string str(const Node &n, const Ctx &ctx) {
   if (!n.is_string()) {
      ctx.fail("expected a string");
   }
   return n.get_value<string>();
}

const fkyaml::node::sequence_type &seq(const Node &n, const Ctx &ctx) {
   if (!n.is_sequence()) {
      ctx.fail("expected a sequence");
   }
   return n.get_value_ref<const fkyaml::node::sequence_type &>();
}

void pair2(const Node &n, double &a, double &b, const Ctx &ctx) {
   const fkyaml::node::sequence_type &s = seq(n, ctx);
   if (s.size() != 2) {
      ctx.fail("expected a sequence of two numbers");
   }
   a = num(s[0], ctx);
   b = num(s[1], ctx);
}

Node parse(const string &text, const Ctx &ctx) {
   try {
      return Node::deserialize(text);
   } catch (const fkyaml::exception &e) {
      ctx.fail(string("YAML parse error: ") + e.what());
   }
   return Node();   // unreached
}

set<string> keys(const char *a, const char *b = 0, const char *c = 0,
                 const char *d = 0, const char *e = 0, const char *f = 0,
                 const char *g = 0, const char *h = 0, const char *i = 0) {
   set<string> s;
   const char *all[] = { a, b, c, d, e, f, g, h, i };
   for (size_t k = 0; k < sizeof(all)/sizeof(*all); k ++) {
      if (all[k]) {
         s.insert(all[k]);
      }
   }
   return s;
}

// --- creature ----------------------------------------------------------

BodySpec parseBody(const Node &n, const Ctx &ctx) {
   Fields f = fields(n, ctx);
   BodySpec B;
   B.name = str(req(f, "name", ctx), ctx.sub("name"));
   B.dof = str(req(f, "dof", ctx), ctx.sub("dof"));

   string type = str(req(f, "type", ctx), ctx.sub("type"));
   set<string> allowed = keys("name", "type", "dof", "density", "points");
   if (type == "sphere") {
      B.kind = BodySpec::Sphere;
      allowed.insert("radius");
      B.radius = num(req(f, "radius", ctx), ctx.sub("radius"));
   } else if (type == "rod") {
      B.kind = BodySpec::Rod;
      allowed.insert("length");
      allowed.insert("radius");
      B.length = num(req(f, "length", ctx), ctx.sub("length"));
      B.radius = opt(f, "radius") ? num(*opt(f, "radius"), ctx.sub("radius")) : 0.01;
   } else if (type == "disk" || type == "cylinder") {
      B.kind = type == "disk" ? BodySpec::Disk : BodySpec::Cylinder;
      allowed.insert("radius");
      allowed.insert("height");
      B.radius = num(req(f, "radius", ctx), ctx.sub("radius"));
      B.height = num(req(f, "height", ctx), ctx.sub("height"));
   } else {
      ctx.sub("type").fail("unknown body type '" + type +
                           "' (sphere|rod|disk|cylinder)");
   }
   checkKeys(f, allowed, ctx);

   if (opt(f, "density")) {
      B.density = num(*opt(f, "density"), ctx.sub("density"));
   }
   if (opt(f, "points")) {
      const fkyaml::node::sequence_type &pts = seq(*opt(f, "points"), ctx.sub("points"));
      for (size_t i = 0; i < pts.size(); i ++) {
         Ctx pc = ctx.item("points", i);
         Fields pf = fields(pts[i], pc);
         checkKeys(pf, keys("name", "at"), pc);
         BodySpec::Point P;
         P.name = str(req(pf, "name", pc), pc.sub("name"));
         pair2(req(pf, "at", pc), P.x, P.y, pc.sub("at"));
         B.points.push_back(P);
      }
   }
   return B;
}

CreatureSpec parseCreature(const Node &root, const Ctx &ctx) {
   Fields f = fields(root, ctx);
   checkKeys(f, keys("name", "root", "bodies", "attachments"), ctx);

   CreatureSpec C;
   C.name = str(req(f, "name", ctx), ctx.sub("name"));

   Ctx rc = ctx.sub("root");
   Fields rf = fields(req(f, "root", ctx), rc);
   checkKeys(rf, keys("x", "y", "attach"), rc);
   C.rootX = str(req(rf, "x", rc), rc.sub("x"));
   C.rootY = str(req(rf, "y", rc), rc.sub("y"));
   C.rootAttach = str(req(rf, "attach", rc), rc.sub("attach"));

   const fkyaml::node::sequence_type &bodies =
      seq(req(f, "bodies", ctx), ctx.sub("bodies"));
   for (size_t i = 0; i < bodies.size(); i ++) {
      C.bodies.push_back(parseBody(bodies[i], ctx.item("bodies", i)));
   }

   if (opt(f, "attachments")) {
      const fkyaml::node::sequence_type &att =
         seq(*opt(f, "attachments"), ctx.sub("attachments"));
      for (size_t i = 0; i < att.size(); i ++) {
         Ctx ac = ctx.item("attachments", i);
         Fields af = fields(att[i], ac);
         checkKeys(af, keys("parent", "child"), ac);
         CreatureSpec::Attachment A;
         A.parent = str(req(af, "parent", ac), ac.sub("parent"));
         A.child = str(req(af, "child", ac), ac.sub("child"));
         C.attachments.push_back(A);
      }
   }
   return C;
}

// --- scenario ----------------------------------------------------------

// one boundary of a pin: mapping with optional q/qdot overrides
RepSpec::Pin parsePinSide(const Node &n, const Ctx &ctx) {
   RepSpec::Pin P;
   P.on = true;
   Fields f = fields(n, ctx);
   checkKeys(f, keys("q", "qdot"), ctx);
   if (opt(f, "q")) {
      P.hasQ = true;
      P.q = num(*opt(f, "q"), ctx.sub("q"));
   }
   if (opt(f, "qdot")) {
      P.hasQdot = true;
      P.qdot = num(*opt(f, "qdot"), ctx.sub("qdot"));
   }
   return P;
}

// pin: start | end | both, or { start: {q, qdot}, end: {q, qdot} }
void parsePin(const Node &n, RepSpec &R, const Ctx &ctx) {
   if (n.is_string()) {
      string s = n.get_value<string>();
      if (s == "start") {
         R.pinStart.on = true;
      } else if (s == "end") {
         R.pinEnd.on = true;
      } else if (s == "both") {
         R.pinStart.on = R.pinEnd.on = true;
      } else {
         ctx.fail("expected start, end, both or {start, end}");
      }
      return;
   }
   Fields f = fields(n, ctx);
   checkKeys(f, keys("start", "end"), ctx);
   if (!opt(f, "start") && !opt(f, "end")) {
      ctx.fail("pin mapping needs start and/or end");
   }
   if (opt(f, "start")) {
      R.pinStart = parsePinSide(*opt(f, "start"), ctx.sub("start"));
   }
   if (opt(f, "end")) {
      R.pinEnd = parsePinSide(*opt(f, "end"), ctx.sub("end"));
   }
}

RepSpec parseRep(const Node &n, const Ctx &ctx) {
   Fields f = fields(n, ctx);
   RepSpec R;
   R.dof = str(req(f, "dof", ctx), ctx.sub("dof"));

   string type = str(req(f, "type", ctx), ctx.sub("type"));
   if (type == "constant") {
      R.type = RepSpec::Constant;
      checkKeys(f, keys("dof", "type", "value", "pin"), ctx);
      R.value = num(req(f, "value", ctx), ctx.sub("value"));
   } else {
      if (type == "hermite") {
         R.type = RepSpec::Hermite;
      } else if (type == "hermlet") {
         R.type = RepSpec::Hermlet;
      } else if (type == "hat") {
         R.type = RepSpec::Hat;
      } else {
         ctx.sub("type").fail("unknown rep type '" + type +
                              "' (constant|hermite|hermlet|hat)");
      }
      checkKeys(f, keys("dof", "type", "from", "to", "min", "max", "pin"), ctx);
      R.from = num(req(f, "from", ctx), ctx.sub("from"));
      R.to = num(req(f, "to", ctx), ctx.sub("to"));
      const Node *mn = opt(f, "min"), *mx = opt(f, "max");
      if (!!mn != !!mx) {
         ctx.fail("min and max must be given together");
      }
      if (mn) {
         R.hasBounds = true;
         R.min = num(*mn, ctx.sub("min"));
         R.max = num(*mx, ctx.sub("max"));
         if (R.min >= R.max) {
            ctx.fail("min must be < max");
         }
      }
   }
   if (opt(f, "pin")) {
      parsePin(*opt(f, "pin"), R, ctx.sub("pin"));
   }
   return R;
}

// lower one boundary pin to its (q, qdot) constraint pair
void desugarPin(const RepSpec &R, const RepSpec::Pin &P,
                ConstraintSpec::At at, vector<ConstraintSpec> &out,
                const Ctx &ctx) {
   if (!P.on) {
      return;
   }
   double q = P.hasQ ? P.q
      : R.type == RepSpec::Constant ? R.value
      : at == ConstraintSpec::Start ? R.from : R.to;
   if (R.type == RepSpec::Constant && q != R.value) {
      ctx.fail("pin contradicts the constant rep's value");
   }
   if (R.hasBounds && (q < R.min || q > R.max)) {
      ctx.fail("pin lies outside the rep's [min, max]");
   }
   ConstraintSpec K;
   K.dof = R.dof;
   K.at = at;
   K.fromPin = true;
   K.quantity = ConstraintSpec::Val;
   K.equals = q;
   out.push_back(K);
   K.quantity = ConstraintSpec::Dot;
   K.equals = P.hasQdot ? P.qdot : 0;
   out.push_back(K);
}

ImpulseSpec parseImpulse(const Node &n, const Ctx &ctx, bool topLevel) {
   Fields f = fields(n, ctx);
   set<string> allowed = keys("point", "direction", "magnitude");
   if (topLevel) {
      allowed.insert("stage");
   }
   checkKeys(f, allowed, ctx);

   ImpulseSpec I;
   if (topLevel) {
      I.stage = str(req(f, "stage", ctx), ctx.sub("stage"));
   }
   I.point = str(req(f, "point", ctx), ctx.sub("point"));
   pair2(req(f, "direction", ctx), I.sx, I.sy, ctx.sub("direction"));
   if (I.sx == 0 && I.sy == 0) {
      ctx.sub("direction").fail("direction must not be (0, 0)");
   }
   if (opt(f, "magnitude")) {
      I.hasMagnitude = true;
      I.magnitude = num(*opt(f, "magnitude"), ctx.sub("magnitude"));
   }
   return I;
}

ConstraintSpec parseConstraint(const Node &n, const Ctx &ctx, bool topLevel) {
   Fields f = fields(n, ctx);
   set<string> allowed = keys("dof", "quantity", "at", "equals", "min", "max");
   if (topLevel) {
      allowed.insert("stage");
   }
   checkKeys(f, allowed, ctx);

   ConstraintSpec K;
   if (topLevel) {
      K.stage = str(req(f, "stage", ctx), ctx.sub("stage"));
   }
   K.dof = str(req(f, "dof", ctx), ctx.sub("dof"));

   string q = str(req(f, "quantity", ctx), ctx.sub("quantity"));
   if (q == "q") {
      K.quantity = ConstraintSpec::Val;
   } else if (q == "qdot") {
      K.quantity = ConstraintSpec::Dot;
   } else {
      ctx.sub("quantity").fail("unknown quantity '" + q + "' (q|qdot)");
   }

   const Node &at = req(f, "at", ctx);
   Ctx ac = ctx.sub("at");
   if (at.is_string()) {
      string s = at.get_value<string>();
      if (s == "start") {
         K.at = ConstraintSpec::Start;
      } else if (s == "end") {
         K.at = ConstraintSpec::End;
      } else {
         ac.fail("expected start, end or {slice, t}");
      }
   } else {
      Fields af = fields(at, ac);
      checkKeys(af, keys("slice", "t"), ac);
      K.at = ConstraintSpec::Explicit;
      K.slice = (int) integer(req(af, "slice", ac), ac.sub("slice"));
      K.t = num(req(af, "t", ac), ac.sub("t"));
      if (K.t < 0 || K.t > 1) {
         ac.sub("t").fail("t must be in [0, 1]");
      }
   }

   const Node *eq = opt(f, "equals"), *mn = opt(f, "min"), *mx = opt(f, "max");
   if (eq && (mn || mx)) {
      ctx.fail("give either equals or min/max, not both");
   }
   if (eq) {
      K.equals = num(*eq, ctx.sub("equals"));
   } else if (mn && mx) {
      K.isRange = true;
      K.min = num(*mn, ctx.sub("min"));
      K.max = num(*mx, ctx.sub("max"));
      if (K.min >= K.max) {
         ctx.fail("min must be < max");
      }
   } else {
      ctx.fail("constraint needs equals or both min and max");
   }
   return K;
}

StageSpec parseStage(const Node &n, const Ctx &ctx) {
   Fields f = fields(n, ctx);
   checkKeys(f, keys("name", "pieces", "duration", "muscles", "reps",
                     "impulses", "constraints"), ctx);

   StageSpec S;
   S.name = str(req(f, "name", ctx), ctx.sub("name"));
   S.pieces = (int) integer(req(f, "pieces", ctx), ctx.sub("pieces"));
   if (S.pieces < 1) {
      ctx.sub("pieces").fail("pieces must be >= 1");
   }

   const Node &dur = req(f, "duration", ctx);
   Ctx dc = ctx.sub("duration");
   if (dur.is_mapping()) {
      Fields df = fields(dur, dc);
      checkKeys(df, keys("min", "max", "start"), dc);
      S.variableDuration = true;
      S.min = num(req(df, "min", dc), dc.sub("min"));
      S.max = num(req(df, "max", dc), dc.sub("max"));
      S.start = num(req(df, "start", dc), dc.sub("start"));
      if (!(0 < S.min && S.min < S.start && S.start < S.max)) {
         dc.fail("need 0 < min < start < max");
      }
   } else {
      S.T = num(dur, dc);
      if (S.T <= 0) {
         dc.fail("duration must be > 0");
      }
   }

   if (opt(f, "muscles")) {
      const fkyaml::node::sequence_type &ms = seq(*opt(f, "muscles"), ctx.sub("muscles"));
      for (size_t i = 0; i < ms.size(); i ++) {
         Ctx mc = ctx.item("muscles", i);
         Fields mf = fields(ms[i], mc);
         checkKeys(mf, keys("dof", "weight"), mc);
         MuscleSpec M;
         M.dof = str(req(mf, "dof", mc), mc.sub("dof"));
         if (opt(mf, "weight")) {
            M.weight = num(*opt(mf, "weight"), mc.sub("weight"));
         }
         S.muscles.push_back(M);
      }
   }

   const fkyaml::node::sequence_type &reps = seq(req(f, "reps", ctx), ctx.sub("reps"));
   for (size_t i = 0; i < reps.size(); i ++) {
      S.reps.push_back(parseRep(reps[i], ctx.item("reps", i)));
   }

   if (opt(f, "impulses")) {
      const fkyaml::node::sequence_type &is = seq(*opt(f, "impulses"), ctx.sub("impulses"));
      for (size_t i = 0; i < is.size(); i ++) {
         S.impulses.push_back(parseImpulse(is[i], ctx.item("impulses", i), false));
      }
   }
   if (opt(f, "constraints")) {
      const fkyaml::node::sequence_type &cs = seq(*opt(f, "constraints"), ctx.sub("constraints"));
      for (size_t i = 0; i < cs.size(); i ++) {
         S.constraints.push_back(parseConstraint(cs[i], ctx.item("constraints", i), false));
      }
   }

   // pins lower to ordinary constraints, placed ahead of any explicit
   // ones: rep document order, start before end, q before qdot
   vector<ConstraintSpec> pins;
   for (size_t i = 0; i < S.reps.size(); i ++) {
      const RepSpec &R = S.reps[i];
      Ctx rc = ctx.item("reps", i).sub("pin");
      desugarPin(R, R.pinStart, ConstraintSpec::Start, pins, rc);
      desugarPin(R, R.pinEnd, ConstraintSpec::End, pins, rc);
   }
   S.constraints.insert(S.constraints.begin(), pins.begin(), pins.end());
   return S;
}

ScenarioSpec parseScenario(const Node &root, const Ctx &ctx) {
   Fields f = fields(root, ctx);
   {
      // the keys() helper caps at nine args; this needs ten
      set<string> allowed = keys("name", "creature", "gravity", "integrator",
                                 "record_dt", "stages", "impulses", "links", "chain");
      allowed.insert("constraints");
      checkKeys(f, allowed, ctx);
   }

   ScenarioSpec S;
   S.name = str(req(f, "name", ctx), ctx.sub("name"));
   S.creaturePath = str(req(f, "creature", ctx), ctx.sub("creature"));
   S.gravity = num(req(f, "gravity", ctx), ctx.sub("gravity"));

   Ctx ic = ctx.sub("integrator");
   Fields nf = fields(req(f, "integrator", ctx), ic);
   checkKeys(nf, keys("type", "n"), ic);
   string itype = str(req(nf, "type", ic), ic.sub("type"));
   if (itype == "simpson") {
      S.integrator = ScenarioSpec::Simpson;
   } else if (itype == "gauss") {
      S.integrator = ScenarioSpec::Gauss;
   } else {
      ic.sub("type").fail("unknown integrator '" + itype + "' (simpson|gauss)");
   }
   // the integrator constructors have hard preconditions: Gauss-Legendre
   // tables exist for 5/8/10 points only, composite Simpson needs an odd
   // sample count >= 3 (and caps at 99)
   S.integratorN = (int) integer(req(nf, "n", ic), ic.sub("n"));
   if (S.integrator == ScenarioSpec::Gauss) {
      if (S.integratorN != 5 && S.integratorN != 8 && S.integratorN != 10) {
         ic.sub("n").fail("gauss supports n = 5, 8 or 10");
      }
   } else {
      if (S.integratorN < 3 || S.integratorN >= 100 || S.integratorN % 2 == 0) {
         ic.sub("n").fail("simpson needs an odd n in [3, 99]");
      }
   }

   if (opt(f, "record_dt")) {
      S.recordDt = num(*opt(f, "record_dt"), ctx.sub("record_dt"));
      if (S.recordDt <= 0) {
         ctx.sub("record_dt").fail("record_dt must be > 0");
      }
   }

   const fkyaml::node::sequence_type &stages = seq(req(f, "stages", ctx), ctx.sub("stages"));
   for (size_t i = 0; i < stages.size(); i ++) {
      S.stages.push_back(parseStage(stages[i], ctx.item("stages", i)));
   }

   if (opt(f, "impulses")) {
      const fkyaml::node::sequence_type &is = seq(*opt(f, "impulses"), ctx.sub("impulses"));
      for (size_t i = 0; i < is.size(); i ++) {
         S.impulses.push_back(parseImpulse(is[i], ctx.item("impulses", i), true));
      }
   }

   const Node *links = opt(f, "links"), *chain = opt(f, "chain");
   if (links && chain) {
      ctx.fail("give either links or chain, not both");
   }
   if (!links && !chain && S.stages.size() > 1) {
      // consecutive stages link by default; opt out with chain: false
      // or an explicit links: list
      S.chainLinks = true;
   }
   if (chain) {
      Ctx cc = ctx.sub("chain");
      if (!chain->is_boolean()) {
         cc.fail("expected a boolean");
      }
      S.chainLinks = chain->get_value<bool>();
   }
   if (links) {
      const fkyaml::node::sequence_type &ls = seq(*links, ctx.sub("links"));
      for (size_t i = 0; i < ls.size(); i ++) {
         Ctx lc = ctx.item("links", i);
         const fkyaml::node::sequence_type &pr = seq(ls[i], lc);
         if (pr.size() != 2) {
            lc.fail("expected a pair [stageA, stageB]");
         }
         S.links.push_back(std::make_pair(str(pr[0], lc), str(pr[1], lc)));
      }
   }

   if (opt(f, "constraints")) {
      const fkyaml::node::sequence_type &cs = seq(*opt(f, "constraints"), ctx.sub("constraints"));
      for (size_t i = 0; i < cs.size(); i ++) {
         S.constraints.push_back(parseConstraint(cs[i], ctx.item("constraints", i), true));
      }
   }
   return S;
}

bool powerOfTwo(int n) {
   return n > 0 && (n & (n - 1)) == 0;
}

// the body part of a "Body.Point" reference
string bodyOf(const string &ref) {
   return ref.substr(0, ref.find('.'));
}

}  // namespace

// --- public entry points ------------------------------------------------

CreatureSpec newt::ParseCreature(const string &text, const string &label) {
   Ctx ctx(label);
   return parseCreature(parse(text, ctx), ctx);
}

ScenarioSpec newt::ParseScenario(const string &text, const string &label) {
   Ctx ctx(label);
   return parseScenario(parse(text, ctx), ctx);
}

CreatureSpec newt::ParseCreatureFile(const string &path) {
   return ParseCreature(ReadFile(path), path);
}

ScenarioSpec newt::ParseScenarioFile(const string &path) {
   return ParseScenario(ReadFile(path), path);
}

string newt::ReadFile(const string &path) {
   std::ifstream in(path.c_str());
   if (!in) {
      throw SpecError(path + ": cannot open file");
   }
   std::ostringstream o;
   o << in.rdbuf();
   return o.str();
}

// --- cross-reference validation ------------------------------------------

void newt::Validate(const ScenarioSpec &S, const CreatureSpec &C) {
   Ctx ctx(S.name);

   // creature-internal name tables
   set<string> dofs, bodies, points;       // points hold "Body.Point"
   dofs.insert(C.rootX);
   dofs.insert(C.rootY);
   if (C.rootX == C.rootY) {
      ctx.fail("creature root x and y DOFs must differ");
   }
   for (size_t i = 0; i < C.bodies.size(); i ++) {
      const BodySpec &B = C.bodies[i];
      if (B.name.find('.') != string::npos) {
         ctx.fail("body name '" + B.name + "' must not contain '.'");
      }
      if (!bodies.insert(B.name).second) {
         ctx.fail("duplicate body name '" + B.name + "'");
      }
      if (!dofs.insert(B.dof).second) {
         ctx.fail("duplicate DOF name '" + B.dof + "'");
      }
      for (size_t j = 0; j < B.points.size(); j ++) {
         if (B.points[j].name.find('.') != string::npos) {
            ctx.fail("point name '" + B.points[j].name + "' must not contain '.'");
         }
         if (!points.insert(B.name + "." + B.points[j].name).second) {
            ctx.fail("duplicate point '" + B.name + "." + B.points[j].name + "'");
         }
      }
   }

   struct Check {
      const set<string> &points;
      const Ctx &ctx;
      void point(const string &ref, const string &what) const {
         if (!points.count(ref)) {
            ctx.fail(what + " references unknown point '" + ref + "'");
         }
      }
   } check = { points, ctx };

   check.point(C.rootAttach, "root.attach");
   for (size_t i = 0; i < C.attachments.size(); i ++) {
      check.point(C.attachments[i].parent, "attachment parent");
      check.point(C.attachments[i].child, "attachment child");
   }

   // the bodies must form a tree rooted at the anchor's body: every other
   // body hangs from exactly one parent, no cycles, nothing disconnected.
   // BuildSweep recurses through attachments without a visited set, so a
   // malformed graph would recurse forever, not just compute nonsense.
   {
      string rootBody = bodyOf(C.rootAttach);
      map<string, string> parentOf;
      for (size_t i = 0; i < C.attachments.size(); i ++) {
         string pb = bodyOf(C.attachments[i].parent);
         string cb = bodyOf(C.attachments[i].child);
         if (pb == cb) {
            ctx.fail("body '" + pb + "' is attached to itself");
         }
         if (cb == rootBody) {
            ctx.fail("root body '" + cb + "' cannot hang from another body");
         }
         if (parentOf.count(cb)) {
            ctx.fail("body '" + cb + "' has more than one parent");
         }
         parentOf[cb] = pb;
      }
      // one-parent-each plus a parentless root means any cycle or stray
      // subtree shows up as unreachable from the root
      set<string> reached;
      reached.insert(rootBody);
      for (bool grew = true; grew; ) {
         grew = false;
         for (map<string, string>::const_iterator p = parentOf.begin();
              p != parentOf.end(); p ++) {
            if (!reached.count(p->first) && reached.count(p->second)) {
               reached.insert(p->first);
               grew = true;
            }
         }
      }
      for (set<string>::const_iterator b = bodies.begin(); b != bodies.end(); b ++) {
         if (!reached.count(*b)) {
            ctx.fail("body '" + *b + "' is not connected to the root body '" +
                     rootBody + "'");
         }
      }
   }

   // stages
   set<string> stageNames;
   for (size_t i = 0; i < S.stages.size(); i ++) {
      const StageSpec &st = S.stages[i];
      Ctx sc = ctx.sub("stage " + st.name);
      if (!stageNames.insert(st.name).second) {
         ctx.fail("duplicate stage name '" + st.name + "'");
      }

      // every DOF must get exactly one representation
      set<string> repped;
      bool hermlet = false;
      for (size_t j = 0; j < st.reps.size(); j ++) {
         const RepSpec &R = st.reps[j];
         if (!dofs.count(R.dof)) {
            sc.fail("rep references unknown DOF '" + R.dof + "'");
         }
         if (!repped.insert(R.dof).second) {
            sc.fail("DOF '" + R.dof + "' has more than one rep");
         }
         hermlet = hermlet || R.type == RepSpec::Hermlet;
      }
      for (set<string>::const_iterator d = dofs.begin(); d != dofs.end(); d ++) {
         if (!repped.count(*d)) {
            sc.fail("DOF '" + *d + "' has no rep on this stage");
         }
      }
      if (hermlet && !powerOfTwo(st.pieces)) {
         sc.fail("hermlet reps require pieces to be a power of two");
      }

      for (size_t j = 0; j < st.muscles.size(); j ++) {
         if (!dofs.count(st.muscles[j].dof)) {
            sc.fail("muscle references unknown DOF '" + st.muscles[j].dof + "'");
         }
      }
      for (size_t j = 0; j < st.impulses.size(); j ++) {
         check.point(st.impulses[j].point, "stage " + st.name + " impulse");
      }
      for (size_t j = 0; j < st.constraints.size(); j ++) {
         const ConstraintSpec &K = st.constraints[j];
         if (!dofs.count(K.dof)) {
            sc.fail("constraint references unknown DOF '" + K.dof + "'");
         }
         if (K.at == ConstraintSpec::Explicit &&
             (K.slice < 0 || K.slice >= st.pieces)) {
            sc.fail("constraint slice out of range [0, pieces)");
         }
      }
   }

   // top-level collections
   for (size_t i = 0; i < S.impulses.size(); i ++) {
      const ImpulseSpec &I = S.impulses[i];
      if (!stageNames.count(I.stage)) {
         ctx.fail("impulse references unknown stage '" + I.stage + "'");
      }
      check.point(I.point, "impulse");
   }
   for (size_t i = 0; i < S.links.size(); i ++) {
      if (!stageNames.count(S.links[i].first) ||
          !stageNames.count(S.links[i].second)) {
         ctx.fail("link references unknown stage '" + S.links[i].first +
                  "' or '" + S.links[i].second + "'");
      }
   }
   for (size_t i = 0; i < S.constraints.size(); i ++) {
      const ConstraintSpec &K = S.constraints[i];
      if (!stageNames.count(K.stage)) {
         ctx.fail("constraint references unknown stage '" + K.stage + "'");
      }
      if (!dofs.count(K.dof)) {
         ctx.fail("constraint references unknown DOF '" + K.dof + "'");
      }
      if (K.at == ConstraintSpec::Explicit) {
         for (size_t j = 0; j < S.stages.size(); j ++) {
            if (S.stages[j].name == K.stage &&
                (K.slice < 0 || K.slice >= S.stages[j].pieces)) {
               ctx.fail("constraint slice out of range for stage '" + K.stage + "'");
            }
         }
      }
   }
}
