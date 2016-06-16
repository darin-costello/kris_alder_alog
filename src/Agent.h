#ifndef AGENT_h
#define AGENT_H

#include "tuple"
#include "Vector.h"
#include "Connection.h"
#include "Polygon.h"

enum AgentStates = {StateSurround,StateTravelLead, StateTravelFollow  };

class Agent {
private:

  int id;

  Vector pos;

  int numConnections;
  std::Vector<int> neighbours;
  double IARAdius;
  double wallRadius;

  double maxAcceleration;
  double maxVelocity;

  double averageDist;
  double minIARadius;

  int leader

  bool frozen


public:
//  static  std::pair<Vector, Vector> calcWeights(Connection[] conn, double radius);
  //std::Vector<Vector::Vector> checkRadar(Polygon[] polygons);
  //std::pair<Vector, Vector> calcInterAgent(std::map<int, *Agent> candidates);

  bool move(std::map<int, *Agent>, *Polygon[], Vector[]);

}


#endif
