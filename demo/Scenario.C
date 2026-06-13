/*
**        The generic scenario-driven program replacing the per-demo
**        C++ setup() bodies: parse YAML, validate, build the World.
*/

# include "Scenario.h"

# include <sstream>
# include <vector>

# include "World.h"
# include "Newt_Glue.h"
# include "newt/SpecParser.h"
# include "newt/SpecBuilder.h"
# include "newt/Census.h"

using std::string;

// creature paths resolve relative to the scenario file's directory
// (the driver cds into a per-run scratch dir, so it passes an
// absolute scenario path)
static string resolve(const string &scenarioPath, const string &creature) {
   if (!creature.empty() && creature[0] == '/') {
      return creature;
   }
   string::size_type slash = scenarioPath.rfind('/');
   if (slash == string::npos) {
      return creature;
   }
   return scenarioPath.substr(0, slash + 1) + creature;
}

void Scenario::setup(int k, Omu_VariableVec &x, Omu_VariableVec &u,
                     Omu_VariableVec &c) {
   if (Newt_ScenarioPath.empty()) {
      m_error(E_NULL, "Scenario: no scenario file (use newt_scenario <path>)");
   }
   try {
      newt::ScenarioSpec S = newt::ParseScenarioFile(Newt_ScenarioPath);
      string creaturePath = resolve(Newt_ScenarioPath, S.creaturePath);
      newt::CreatureSpec C = newt::ParseCreatureFile(creaturePath);
      std::vector<string> warnings;
      newt::Validate(S, C, &warnings);
      for (size_t i = 0; i < warnings.size(); i ++) {
         cerr << "Warning: " << warnings[i] << "\n";
      }

      // snapshots for the run record
      Newt_ScenarioYaml = newt::ReadFile(Newt_ScenarioPath);
      Newt_CreatureYaml = newt::ReadFile(creaturePath);
      Newt_RecordDt = S.recordDt;

      W = newt::BuildWorld(S, C, x, c);

      // the census for newt_census, cross-checked against the claims
      // the build just made
      {
         std::ostringstream o;
         o << newt::Census(S, C);
         int nVars, nCons;
         newt::CensusTotals(S, C, nVars, nCons);
         if (nVars == W->xIx && nCons == W->cIx) {
            o << "cross-check: census totals match the built World ("
              << nVars << " vars, " << nCons << " cons)\n";
         } else {
            o << "cross-check: MISMATCH -- census says " << nVars
              << " vars / " << nCons << " cons, the World claimed "
              << W->xIx << " / " << W->cIx << "\n";
         }
         Newt_CensusText = o.str();
      }
   } catch (const newt::SpecError &e) {
      cerr << "Scenario: " << e.msg << "\n";
      m_error(E_INPUT, "Scenario: bad scenario/creature file");
   }
}

void Scenario::update(int kk,
                      const adoublev &x, const adoublev &u,
                      adoublev &f, adouble &f0, adoublev &c) {
   W->Update(x, c, f0);
}

// propagate the class to the command interface
IF_CLASS_DEFINE("Scenario", Scenario, Omu_Program);
