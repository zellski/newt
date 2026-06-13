/*
**        YAML -> Spec parsing and cross-reference validation. The only
**        code that touches the YAML library. Throws newt::SpecError.
*/

# pragma once

# include <vector>

# include "newt/Spec.h"

namespace newt {

// label is used in error messages (typically the file path)
CreatureSpec ParseCreature(const std::string &yamlText, const std::string &label);
ScenarioSpec ParseScenario(const std::string &yamlText, const std::string &label);

CreatureSpec ParseCreatureFile(const std::string &path);
ScenarioSpec ParseScenarioFile(const std::string &path);

// whole-file read; throws SpecError on failure (used to snapshot the
// YAML texts into the run record)
std::string ReadFile(const std::string &path);

// cross-checks between a scenario and its creature; throws SpecError.
// Non-fatal structural findings (e.g. a DOF with no position anchor
// anywhere) append to *warnings when given
void Validate(const ScenarioSpec &S, const CreatureSpec &C,
              std::vector<std::string> *warnings = 0);

}
