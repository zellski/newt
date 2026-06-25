// Force C++ standard library headers to be fully parsed BEFORE HQP/meschach.
// Meschach.h defines function-like min(a,b)/max(a,b) macros that otherwise
// clobber identifiers used inside libstdc++ template headers (specfun, tr1,
// streambuf, istream, ...). Including these up front sets their include
// guards so meschach's macros can no longer corrupt them.
// Guarded for __cplusplus so the same -include works for C TUs (Odc_Main.c).
#ifdef __cplusplus
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <istream>
#include <ostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>
#include <complex>
#endif
