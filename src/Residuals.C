# include "newt/Residuals.h"

# include "World.h"

using namespace newt;

bool Residuals::requested = false;
bool Residuals::valid = false;
std::vector<double> Residuals::c;

void Residuals::Capture(World *W, const adoublev &cv) {
   requested = false;
   c.clear();
   for (int i = 0; i < W->cIx; i ++) {
      c.push_back(cv[i].value());
   }
   valid = true;
}
