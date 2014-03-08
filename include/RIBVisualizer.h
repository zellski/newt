# pragma once

# include <iostream>
# include <fstream>

class adoublev;
class World;
class Creature;
class Planar;
class BodyPoint;
class AnchorPoint;
class RigidBody;

class RIBVisualizer {
private:
   static void Render(const Creature *P, std::ofstream &RIB);

   static void Render(const BodyPoint *P, std::ofstream &RIB);
   static void Render(const AnchorPoint *P, std::ofstream &RIB);

   static void Render(const RigidBody *B, std::ofstream &RIB);
public:
   static void Generate(World *const W, const adoublev &x);
};
