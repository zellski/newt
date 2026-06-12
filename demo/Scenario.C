/*
**        The generic scenario-driven program replacing the per-demo
**        C++ setup() bodies: parse YAML, validate, build the World.
*/

# include "Scenario.h"

# include "World.h"
# include "Newt_Glue.h"
# include "newt/SpecParser.h"
# include "newt/SpecBuilder.h"

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
      newt::Validate(S, C);

      // snapshots for the run record
      Newt_ScenarioYaml = newt::ReadFile(Newt_ScenarioPath);
      Newt_CreatureYaml = newt::ReadFile(creaturePath);
      Newt_RecordDt = S.recordDt;

      W = newt::BuildWorld(S, C, x, c);
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
