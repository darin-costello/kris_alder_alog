#ifndef VECTOR_H
#define VECTOR_H

class Vector {
public:
  double x = 0;
  double y = 0;

  Vector();
  Vector(double, double);
  Vector normalize(double);
  Vector cap(double);
  double length();
  double sqLength();
  Vector subtract(Vector);
  Vector add(Vector);
//  Vector CrossProduct(Vector);
  double dotProduct(Vector);
  Vector scalarMultiply(double);
// Vector FixLength();
  Vector rotate(double);

};

#endif
