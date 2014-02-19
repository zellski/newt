# pragma once

# include "RigidBody.h"


// This is for the Visualizer, basically. There is probably some way
// of doing this with run-time dynamic class identification or whatever
// but I am getting increasingly impatient with eternal compsci fiddling.
// This does the job just fine.

# define BODY_SPHERE	1
# define BODY_THINROD	2
# define BODY_DISK	3
# define BODY_CYLINDER	4

class BodyPoint;
class DOF;
class World;

class adoublev;
class adouble;


class Sphere: public RigidBody {
public:
   const double Radius;
   int BodyType() const { return BODY_SPHERE; }
   Sphere(World *const w, const char *const S, DOF *const d,
	   double radius, double density = 1);
};

class ThinRod: public RigidBody {
public:
   const double Length, Radius;
   int BodyType() const { return BODY_THINROD; }
   ThinRod(World *const w, const char *const S, DOF *const d,
	   double length, double radius = 0.01, double density = 1);
};

class Disk: public RigidBody {
public:
   const double Radius, Height;
   int BodyType() const { return BODY_DISK; }
   Disk(World *const w, const char *const S, DOF *const d,
	    double radius, double height, double density = 1);
};

class Cylinder: public RigidBody {
public:
   const double Radius, Height;
   int BodyType() const { return BODY_CYLINDER; }
   Cylinder(World *const w, const char *const S, DOF *const d,
	    double radius, double height, double density = 1);
};
