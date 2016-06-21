#ifndef POLYGON_H
#define POLYGON_H

#include <vector>
#include "Vector.h"

class Edge {
public:
  Vector one;
  Vector two;

  Edge(Vector, Vector);
  Vector intersection(Edge);

  bool onLine(Vector);
  bool intersects(Edge);

};

class Polygon {
public:
  std::vector<Vector> vertices;

  std::vector<Edge> edges();
};




#endif
