#ifndef AGENT_H
#define AGENT_H


#define RADCAHGNEAMT 1.0


#include <vector>
#include <tuple>
#include <map>

#include "Vector.h"
#include "Connection.h"
#include "Polygon.h"



class Agent {
public:
  enum AgentStates {StateSurround = 0, StateTravelFollow = 1, StateTravelLead = 2,
  StateTransition = -1};
  static constexpr double RADARDIST = 300;
  int id;

  Vector pos;
  Vector heading;

  int numConnections = 3;
  int numRays = 180;
  std::vector<int> neighbours = std::vector<int>();

  double IARadius = 80.0;
  double wallRadius = 60.0;
  double sensorDistance = 600.0;

  double maxAcceleration;
  double maxVelocity = 80;

  double averageDist = 80;
  double minIARadius = 80;

  int leader = -1;

  bool frozen = false;

  AgentStates mode = StateSurround;

  Agent(int id, Vector pos);
  std::pair<Vector, Vector> calcWeights(std::vector<Connection> conns, double radius);
  std::vector<Vector> checkRadar(std::vector<Polygon*> polygons);
  std::pair<Vector, Vector> calcInterAgent(std::map<int, Agent*> candidates);
  std::pair<Vector, Vector> calcPolygons(std::vector<Vector>);
  std::vector<Vector> checkPointSensor(std::vector<Vector> allPoints);
  Vector move(std::map<int, Agent*>, std::vector<Polygon*> polygons,  std::vector<Vector> points);

  void averageRadius(std::map<int, Agent*>);
  void communicate(std::map<int, Agent*>);
};


#endif
