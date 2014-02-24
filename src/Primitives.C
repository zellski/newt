# include "Stage.h"
# include <math.h>
# include "Primitives.h"
# include "RigidBody.h"
# include "BodyPoint.h"
# include "DOF.h"
# include "World.h"

Sphere::Sphere(World *const w, const char *const S, DOF *const d,
               double radius, double density) :
   RigidBody(w, S, d,
             1000 * density * 4 * M_PI * radius * radius * radius / 3,
             2*radius*radius/5),
   Radius(radius) {
      cerr << "RigidBody<Sphere> named " << Name
           << ": Radius " << Radius
           << ", Mass " << Mass << ", ROG^2 " << ROG2 << "...\n";
}

ThinRod::ThinRod(World *const w, const char *const S, DOF *const d,
                 double length, double radius,
                 double density) :
   RigidBody(w, S, d,
             1000 * density * length * radius * radius,
             length*length/12.0),
   Length(length), Radius(radius) {
      cerr << "RigidBody<ThinRod> named " << Name
           << ": Length " << Length
           << ", Mass " << Mass << ", Radius " << Radius << ", ROG^2 " << ROG2 << "...\n";
}

Disk::Disk(World *const w, const char *const S, DOF *const d,
           double radius, double height,
           double density) :
   RigidBody(w, S, d,
             1000 * density * height * radius * radius * M_PI,        // Mass
             (height*height + 3*radius*radius)/12.0),                // ROG2
   Radius(radius),
   Height(height) {
      cerr << "RigidBody<Disk> named " << Name
           << ": Height " << Height << ", Radius " << Radius
           << ", Mass " << Mass << ", ROG^2 " << ROG2 << "...\n";
}

Cylinder::Cylinder(World *const w, const char *const S, DOF *const d,
                   double radius, double height,
                   double density) :
   RigidBody(w, S, d,
             1000 * density * height * 0.002 * radius * 2 * M_PI,        // Mass
             (height*height + 2*radius*radius)/12.0),                // ROG2
   Radius(radius),
   Height(height) {
      cerr << "RigidBody<Cylinder> named " << Name
           << ": Height " << Height << ", Radius " << Radius
           << ", Mass " << Mass << ", ROG^2 " << ROG2 << "...\n";
}
