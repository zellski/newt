/*
**        Instantiates the solver-side World from parsed specs. The one
**        rule that matters: construction follows document order, since
**        optimizer variable and constraint indices are claimed at
**        construction time.
*/

# pragma once

# include "newt/Spec.h"

class World;
class Omu_VariableVec;

namespace newt {

// builds DOFs/bodies/creature/stages/... and calls W->Initialize(x, c)
World *BuildWorld(const ScenarioSpec &S, const CreatureSpec &C,
                  Omu_VariableVec &x, Omu_VariableVec &c);

}
