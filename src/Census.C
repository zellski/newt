# include "newt/Census.h"

# include <algorithm>
# include <map>
# include <sstream>
# include <vector>

using namespace newt;
using std::string;
using std::vector;
using std::map;

namespace {

// variables a rep claims (mirrors the Hermite/Hermlet/Hat/Constant ctors)
int repVars(const RepSpec &R, int N) {
   switch (R.type) {
   case RepSpec::Constant: return 0;
   case RepSpec::Hermite:
   case RepSpec::Hermlet:  return 2*N + 2;
   case RepSpec::Hat:      return N + 1;
   }
   return 0;
}

// constraint rows a rep claims: interpolation bounds + FEM equations
void repCons(const RepSpec &R, int N, int &bounds, int &fem) {
   switch (R.type) {
   case RepSpec::Constant:
      break;
   case RepSpec::Hermite:
   case RepSpec::Hermlet:
      bounds += R.hasBounds ? 2*N : 0;
      fem += 2*N;
      break;
   case RepSpec::Hat:
      fem += N - 1;
      break;
   }
}

const char *repName(const RepSpec &R) {
   switch (R.type) {
   case RepSpec::Constant: return "constant";
   case RepSpec::Hermite:  return "hermite";
   case RepSpec::Hermlet:  return "hermlet";
   case RepSpec::Hat:      return "hat";
   }
   return "?";
}

struct Counts {
   int durations = 0, repCoeffs = 0, muscleCoeffs = 0, impulseMags = 0;
   int bounds = 0, fem = 0, links = 0, pins = 0, explicits = 0;

   int vars() const {
      return durations + repCoeffs + muscleCoeffs + impulseMags;
   }
   int cons() const {
      return bounds + fem + links + pins + explicits;
   }
};

bool isConstant(const StageSpec &st, const string &dof) {
   for (size_t i = 0; i < st.reps.size(); i ++) {
      if (st.reps[i].dof == dof) {
         return st.reps[i].type == RepSpec::Constant;
      }
   }
   return true;   // unrepped (invalid anyway); Link would skip it
}

// the consecutive-or-explicit link list the builder will create
vector<std::pair<string, string> > linkList(const ScenarioSpec &S) {
   vector<std::pair<string, string> > links;
   if (S.chainLinks) {
      for (size_t i = 0; i + 1 < S.stages.size(); i ++) {
         links.push_back(std::make_pair(S.stages[i].name,
                                        S.stages[i + 1].name));
      }
   }
   links.insert(links.end(), S.links.begin(), S.links.end());
   return links;
}

Counts count(const ScenarioSpec &S, const CreatureSpec &C) {
   Counts n;
   map<string, const StageSpec *> stages;
   for (size_t i = 0; i < S.stages.size(); i ++) {
      const StageSpec &st = S.stages[i];
      stages[st.name] = &st;
      n.durations += st.variableDuration ? 1 : 0;
      n.muscleCoeffs += 2 * st.pieces * st.muscles.size();
      for (size_t j = 0; j < st.reps.size(); j ++) {
         n.repCoeffs += repVars(st.reps[j], st.pieces);
         repCons(st.reps[j], st.pieces, n.bounds, n.fem);
      }
      for (size_t j = 0; j < st.impulses.size(); j ++) {
         n.impulseMags += st.impulses[j].hasMagnitude ? 0 : 1;
      }
      for (size_t j = 0; j < st.constraints.size(); j ++) {
         (st.constraints[j].fromPin ? n.pins : n.explicits) ++;
      }
   }
   for (size_t i = 0; i < S.impulses.size(); i ++) {
      n.impulseMags += S.impulses[i].hasMagnitude ? 0 : 1;
   }
   for (size_t i = 0; i < S.constraints.size(); i ++) {
      (S.constraints[i].fromPin ? n.pins : n.explicits) ++;
   }

   // every DOF non-constant on either side of a link costs a
   // continuity + momentum row pair
   vector<string> dofs;
   dofs.push_back(C.rootX);
   dofs.push_back(C.rootY);
   for (size_t i = 0; i < C.bodies.size(); i ++) {
      dofs.push_back(C.bodies[i].dof);
   }
   vector<std::pair<string, string> > links = linkList(S);
   for (size_t l = 0; l < links.size(); l ++) {
      const StageSpec *a = stages[links[l].first];
      const StageSpec *b = stages[links[l].second];
      for (size_t d = 0; a && b && d < dofs.size(); d ++) {
         if (!isConstant(*a, dofs[d]) || !isConstant(*b, dofs[d])) {
            n.links += 2;
         }
      }
   }
   return n;
}

// boundary/interior coverage marks for one (dof, stage) cell
struct Marks {
   bool sq = false, sqd = false, eq = false, eqd = false;
   int mid = 0;

   void add(const ConstraintSpec &K) {
      bool isq = K.quantity == ConstraintSpec::Val;
      switch (K.at) {
      case ConstraintSpec::Start:
         (isq ? sq : sqd) = true;
         break;
      case ConstraintSpec::End:
         (isq ? eq : eqd) = true;
         break;
      case ConstraintSpec::Explicit:
         mid ++;
         break;
      }
   }
   string render() const {
      std::ostringstream o;
      if (sq || sqd) {
         o << " s[" << (sq ? "q" : "") << (sq && sqd ? "," : "")
           << (sqd ? "qd" : "") << "]";
      }
      if (eq || eqd) {
         o << " e[" << (eq ? "q" : "") << (eq && eqd ? "," : "")
           << (eqd ? "qd" : "") << "]";
      }
      if (mid) {
         o << " m[" << mid << "]";
      }
      return o.str();
   }
};

string pad(const string &s, size_t w) {
   string out = s;
   while (out.size() < w) {
      out += ' ';
   }
   return out;
}

}  // namespace

void newt::CensusTotals(const ScenarioSpec &S, const CreatureSpec &C,
                        int &nVars, int &nCons) {
   Counts n = count(S, C);
   nVars = n.vars();
   nCons = n.cons();
}

string newt::Census(const ScenarioSpec &S, const CreatureSpec &C) {
   vector<string> dofs;
   dofs.push_back(C.rootX);
   dofs.push_back(C.rootY);
   for (size_t i = 0; i < C.bodies.size(); i ++) {
      dofs.push_back(C.bodies[i].dof);
   }

   // cell texts: rep type + coverage marks per (dof, stage)
   map<string, map<string, Marks> > marks;   // dof -> stage -> marks
   for (size_t i = 0; i < S.stages.size(); i ++) {
      const StageSpec &st = S.stages[i];
      for (size_t j = 0; j < st.constraints.size(); j ++) {
         marks[st.constraints[j].dof][st.name].add(st.constraints[j]);
      }
   }
   for (size_t i = 0; i < S.constraints.size(); i ++) {
      marks[S.constraints[i].dof][S.constraints[i].stage]
         .add(S.constraints[i]);
   }

   vector<vector<string> > cells(dofs.size());
   for (size_t d = 0; d < dofs.size(); d ++) {
      for (size_t i = 0; i < S.stages.size(); i ++) {
         const StageSpec &st = S.stages[i];
         string cell = "-";
         for (size_t j = 0; j < st.reps.size(); j ++) {
            if (st.reps[j].dof == dofs[d]) {
               cell = repName(st.reps[j]) + marks[dofs[d]][st.name].render();
            }
         }
         cells[d].push_back(cell);
      }
   }

   size_t dofW = string("DOF").size();
   for (size_t d = 0; d < dofs.size(); d ++) {
      dofW = std::max(dofW, dofs[d].size());
   }
   vector<size_t> colW(S.stages.size());
   for (size_t i = 0; i < S.stages.size(); i ++) {
      colW[i] = S.stages[i].name.size();
      for (size_t d = 0; d < dofs.size(); d ++) {
         colW[i] = std::max(colW[i], cells[d][i].size());
      }
   }

   std::ostringstream o;
   o << "census: scenario '" << S.name << "', creature '" << C.name << "'\n\n";
   o << "  " << pad("DOF", dofW + 2);
   for (size_t i = 0; i < S.stages.size(); i ++) {
      o << pad(S.stages[i].name, colW[i] + 2);
   }
   o << "\n";
   for (size_t d = 0; d < dofs.size(); d ++) {
      o << "  " << pad(dofs[d], dofW + 2);
      for (size_t i = 0; i < S.stages.size(); i ++) {
         o << pad(cells[d][i], colW[i] + 2);
      }
      o << "\n";
   }

   vector<std::pair<string, string> > links = linkList(S);
   o << "\n  links: ";
   if (links.empty()) {
      o << "none";
   }
   for (size_t l = 0; l < links.size(); l ++) {
      o << (l ? ", " : "") << links[l].first << "->" << links[l].second;
   }
   o << "\n";
   o << "  s[..]/e[..]: q and qdot constrained at the stage boundary, "
        "m[n]: n interior constraints\n\n";

   Counts n = count(S, C);
   o << "variables: " << n.vars() << "\n";
   o << "  stage durations       " << n.durations << "\n";
   o << "  rep coefficients      " << n.repCoeffs << "\n";
   o << "  muscle torque coeffs  " << n.muscleCoeffs << "\n";
   o << "  impulse magnitudes    " << n.impulseMags << "\n";
   o << "constraints: " << n.cons() << "\n";
   o << "  interpolation bounds  " << n.bounds << "\n";
   o << "  FEM equations         " << n.fem << "\n";
   o << "  link continuity       " << n.links << "\n";
   o << "  pins                  " << n.pins << "\n";
   o << "  explicit constraints  " << n.explicits << "\n";
   return o.str();
}
