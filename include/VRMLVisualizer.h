# ifndef VRMLVISUALIZER_H
# define VRMLVISUALIZER_H

# include <fstream>
using namespace std;

class adoublev;
class World;
class Creature;
class Planar;
class BodyPoint;
class AnchorPoint;
class RigidBody;

class VRMLVisualizer {
public:
   const World *W;
private:
   static void Render(const Creature *P, ofstream &VRML);

   static void Render(const BodyPoint *P, ofstream &VRML);
   static void Render(const AnchorPoint *P, ofstream &VRML);

   static void Render(const RigidBody *B, ofstream &VRML);
public:
   static void Generate(World *const W, const adoublev &x, const int frames);
};

#endif
