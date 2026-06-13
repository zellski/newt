/*
**        A static accounting of the optimization problem a scenario
**        describes, derived purely from the parsed specs: the per-DOF
**        per-stage rep/boundary table and the variable/constraint
**        provenance breakdown. Std-only, like the parser.
*/

# pragma once

# include <string>

# include "newt/Spec.h"

namespace newt {

// the human-readable table + breakdown
std::string Census(const ScenarioSpec &S, const CreatureSpec &C);

// the spec-derived totals, for cross-checking against the built World
void CensusTotals(const ScenarioSpec &S, const CreatureSpec &C,
                  int &nVars, int &nCons);

}
