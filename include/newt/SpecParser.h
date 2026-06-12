/*
**        YAML -> Spec parsing and cross-reference validation. The only
**        code that touches the YAML library. Throws newt::SpecError.
*/

# pragma once

# include "newt/Spec.h"

namespace newt {

// label is used in error messages (typically the file path)
CreatureSpec ParseCreature(const std::string &yamlText, const std::string &label);
ScenarioSpec ParseScenario(const std::string &yamlText, const std::string &label);

CreatureSpec ParseCreatureFile(const std::string &path);
ScenarioSpec ParseScenarioFile(const std::string &path);

// cross-checks between a scenario and its creature; throws SpecError
void Validate(const ScenarioSpec &S, const CreatureSpec &C);

}
