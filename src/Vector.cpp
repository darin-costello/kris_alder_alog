#include <math.h>

#include "Vector.h"

Vector::Vector(){

}

Vector::Vector(double x, double y){
  this->x = x;
  this->y = y;
}

Vector Vector::normalize(double factor){
  double length = this->length();

  if (length == 0){
    return Vector(0,0);
  }
  return Vector(x/length * factor, y / length * factor);
}

Vector Vector::cap(double factor){
  double length = this->length();

  if (length > factor) {
    return normalize(factor);
  }

  return *this;
}

double Vector::length(){
  return  sqrt(x * x + y * y);
}

double Vector::sqLength(){
  return  x* x + y * y;
}

Vector Vector::subtract(Vector other){
  return Vector(x - other.x, y - other.y);
}

Vector Vector::add(Vector other) {
  return Vector(x + other.x, y + other.y);
}

double Vector::dotProduct(Vector other){
  return x * other.x + y * other.y;
}


Vector Vector::scalarMultiply(double f) {
  return Vector(x * f, y * f);
}

Vector Vector::rotate(double angle){
  double nx = cos(angle) * x - sin(angle) * y;
  double ny = sin(angle) * x - cos(angle) * y;

  return Vector(nx, ny);
}
