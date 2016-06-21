#include <vector>
#include <algorithm>

#include "Polygon.h"
#include "Vector.h"

std::vector<Edge> Polygon::edges(){
  std::vector<Edge> result;
  for(int i = 0; i< vertices.size() ; i++){
    Vector one = vertices[i];
    Vector two = vertices[(i+1)%vertices.size()];
    result.push_back(Edge(one, two));
  }
  return result;
}


Edge::Edge(Vector one, Vector two){
  this->one = one;
  this->two = two;
}

Vector Edge::intersection(Edge other){
  double otherXDiff = other.one.x - other.two.x;
  double otherYDiff = other.one.y - other.two.y;
  double thisXDiff = other.one.x - other.two.x;
  double thisYDiff = other.one.y - other .two.y;
  double denom = (thisXDiff * otherYDiff) -(thisYDiff * otherXDiff);
  double firstTerm = (one.x * two.y) - (one.y * two.x);
  double secondTerm = (other.one.x * other.two.y) - (other.one.y * other.two.x);

  double x = ((firstTerm * otherXDiff) - (thisXDiff * secondTerm)) / denom;
  double y = ((firstTerm * otherYDiff) - (thisYDiff * secondTerm)) / denom;
//https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
  return Vector(x,y);
}

bool Edge::onLine(Vector other){
  double maxX = std::max(one.x, two.x);
  double minX = std::min(one.x, two.x);

  double maxY = std::max(one.y, two.y);
  double minY = std::min(one.y, two.y);

  bool inX = other.x >= minX && other.x <= maxX;
  bool inY = other.y >= minY && other.y <= maxY;

  return inX && inY;
}

bool Edge::intersects(Edge other){
  Vector inter = intersection(other);

  bool one = onLine(inter);
  bool two = other.onLine(inter);

  return one && two;

}
