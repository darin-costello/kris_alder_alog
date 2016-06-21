#ifndef CONNECTION_H
#define CONNECTION_H

#include "Vector.h"

class Connection{
public:
  Vector diff;
  double dist;

  double angleWeight;
  double strength;

  int agentNum;

  Connection(Vector diff, double dist, int agentNum);
  Connection(Vector diff, double dist);

};
bool operator< (const Connection& l, const Connection& r);



#endif
