/*
**        Test stand-in for HQP's Omu_Variables.h -- just enough for the
**        NewT headers to parse. Also supplies the Inf bound constant that
**        the real header chain provides.
*/

# pragma once

# include <limits>

const double Inf = std::numeric_limits<double>::infinity();

class Omu_VariableVec {};
