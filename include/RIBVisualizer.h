# ifndef RIBVISUALIZER_H
# define RIBVISUALIZER_H

# include <iostream>
# include <fstream>

using namespace std;

class adoublev;
class World;
class Creature;
class Planar;
class BodyPoint;
class AnchorPoint;
class RigidBody;

class RIBVisualizer {
public:
   const World *W;
private:
   static void Render(const Creature *P, ofstream &RIB);

   static void Render(const BodyPoint *P, ofstream &RIB);
   static void Render(const AnchorPoint *P, ofstream &RIB);

   static void Render(const RigidBody *B, ofstream &RIB);
public:
   static void Generate(World *const W, const adoublev &x);
};

#endif