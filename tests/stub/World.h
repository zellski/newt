/*
**        Test stand-in for the real World. The dynamics sweep only reads
**        the gravitational constant.
*/

# pragma once

class World {
public:
   const double G;

   World(double g) : G(g) {}
};
