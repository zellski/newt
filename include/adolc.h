/***
 ** Single entry point for ADOL-C requirements.
 **
 ** The adoublev.h emulation/rewrite in HQP actually pulls everything we need.
 ***/

# include <adoublev.h>

// this codebase was written against HQP headers that pulled in iostream
// and the std namespace; modern HQP no longer does, so do it here
# include <iostream>
using namespace std;
