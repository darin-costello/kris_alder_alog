#ifndef AGENT_H
#define AGENT_H


#define RADCAHGNEAMT 0.001


#include <vector>
#include <tuple>
#include <map>

#include "Vector.h"
#include "Connection.h"
#include "Polygon.h"



class Agent {
public:
  enum AgentStates {StateSurround, StateTravelFollow, StateTravelLead };
  static constexpr double RADARDIST = 120;
  int id;

  Vector pos;
  Vector heading;

  int numConnections = 3;
  int numRays = 20;
  std::vector<int> neighbours = std::vector<int>();

  double IARadius = 0.0;
  double wallRadius = 80.0;
  double sensorDistance = 120.0;

  double maxAcceleration;
  double maxVelocity = 80;

  double averageDist = 60;
  double minIARadius = 70;

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
