#include <algorithm>
#include <cmath>
#include <math.h>
#include <queue>
#include <vector>
#include <iostream>
#include "Agent.h"
#include "Vector.h"
#include "Polygon.h"
#include "Connection.h"

Agent::Agent(int id, Vector pos){
  this->id = id;
  this->pos = pos;
}

//Communicates mode changes between agents.
void Agent::communicate(std::map<int, Agent*> others){

  if(this->mode == StateTransition){
    this->mode = StateSurround;
    return;
  }

  for(auto id: neighbours){ // For all my neighbours
    Agent* n = others[id];
    //check if neighour is traveling, and if i'm not allready traveling
    //start to travel, following that neighbour, and break out of loop.
    if(n->mode > mode && mode != StateTravelFollow){
      std::cout << this->id <<": nowFollowing " << id << std::endl;
      mode = StateTravelFollow;
      this->leader = n->id;
      break;
    }
  }
  //if I am travelling, and my leader isn't, stop traveling.
  if (mode == StateTravelFollow){
    Agent* leader = others[this->leader];
      if(leader->mode == StateSurround || leader->mode == StateTransition){
        std::cout << id <<": enter Surround\n";
        mode = StateTransition;
      }
  }
}

//Averages Radius among neighbours
void Agent::averageRadius(std::map<int, Agent*> agents){

  // If in travel mode, keep a close grouping, just use minimum.
  if(mode == StateTravelLead || mode == StateTravelFollow) {
    IARadius = minIARadius;
    return;
  }
// no neighbours no average
  if(neighbours.size() == 0){
    return;
  }

  double averageDist = 0.0;
  double otherAverageDist = 0.0;

//for all nieghbours, get my distance between them, and there interagent averages
//update values at the end.
  for(auto id : neighbours){

    Agent* agent = agents[id];

    double dist = agent->pos.subtract(pos).length();

    averageDist += dist;
    otherAverageDist += agent->averageDist;
  }

  averageDist /= (double)neighbours.size();

  otherAverageDist /= (double)neighbours.size();

  double distDiff = otherAverageDist - IARadius;


//Only change so much per iteration.
  if(distDiff > 0)
    distDiff = std::min(distDiff, RADCAHGNEAMT);
  else if(distDiff < 0)
    distDiff = std::max(distDiff, -1 * RADCAHGNEAMT);

  //std::cout << id << ": dist changed by " << distDiff << std::endl;
  IARadius += distDiff;
//Don't go below minimum;
  if(IARadius < minIARadius) {
    IARadius = minIARadius;
  }

  //update average dist.
  this->averageDist = averageDist;
}


//Calculates agent movements.
Vector Agent::move(std::map<int, Agent*> agents, std::vector<Polygon*> obs, std::vector<Vector> points){

  if(frozen){
    return Vector(0,0);
  }
  //Get all fields
  //first vector is an attractive, the second is a replusive
  std::pair<Vector, Vector> agentFields = calcInterAgent(agents);
  std::pair<Vector, Vector> polyFields = calcPolygons(checkRadar(obs));
  //std::pair<Vector, Vector> pointFields = calcPolygons(checkPointSensor(points));

  Vector goalVelocity;

  if (mode == StateSurround){
    // Do everything as expected.
    Vector agentTotal = agentFields.second.add(agentFields.first);
    Vector polyTotal = polyFields.second.add(polyFields.first);
    //Vector pointTotal = pointFields.second.add(pointFields.first);

    goalVelocity = agentTotal.add(polyTotal);//.add(pointTotal);
  } else if (mode == StateTravelLead){
    // The travel heading exerts a force on leaders.
    // still repeled by other agents and obs
    //Vector polyTotal = polyFields.second.add(polyFields.first);
    goalVelocity = agentFields.second.add(polyFields.second).add(heading.normalize(0.008));
  } else if (mode == StateTravelFollow){
    //follow the leader.
    Agent* leader = agents[this->leader];
    Vector diff = leader->pos.subtract(pos);
    double dist = diff.length();

    Vector leaderAttr;

    if (dist > IARadius) {
      double strength = fabs(((1/(double)IARadius) - (1/dist)) * M_PI);

      leaderAttr = diff.normalize(strength);
    }
    //avoid everything else.
    Vector agentTotal = agentFields.second.add(leaderAttr);
    //Vector polyTotal = polyFields.second.add(polyFields.first);

    goalVelocity = agentTotal.add(polyFields.second);
  }

  return goalVelocity;
}

std::pair<Vector, Vector> Agent::calcInterAgent(std::map<int, Agent*> candidates) {

  std::priority_queue<Connection> pQueue;
  for(auto agentNum : candidates){
    Agent* cand = agentNum.second;

    Vector diff = cand->pos.subtract(pos);
    double dist = diff.length();

    if (agentNum.first != id){
      pQueue.push(Connection(diff, dist, agentNum.first));
    }
  }
  std::vector<Connection> conns;

  neighbours.clear();
  while(neighbours.size() < numConnections && pQueue.size() > 0){
    Connection cnn = pQueue.top();
    pQueue.pop();

    int id = cnn.agentNum;
    neighbours.push_back(id);
    conns.push_back(cnn);
  }
  return calcWeights(conns, IARadius);
}

std::pair<Vector, Vector> Agent::calcPolygons(std::vector<Vector> pings) {

  std::priority_queue<Connection> pQueue;
  for(auto ping : pings){

    Vector diff = ping.subtract(pos);
    double dist = diff.length();

    pQueue.push(Connection(diff, dist));
  }
  std::vector<Connection> conns;
  while(pQueue.size() > 0) {

    conns.push_back(pQueue.top());
    pQueue.pop();
  }
  return calcWeights(conns, wallRadius);
}

std::pair<Vector, Vector> Agent::calcWeights(std::vector<Connection> conns, double radius){

  double angleTotal = 0.0;

  for(int i = 0; i < conns.size(); i++){
    Connection conn = conns[i];
    double smallestAngle = M_PI;

    for(int j = 0; j < i ; j++){
      Connection other = conns[j];

      double dotProd  = conn.diff.dotProduct(other.diff);

      double intermed = dotProd / (other.dist * conn.dist);
      intermed = std::max(intermed, -1.0);
      intermed = std::min(intermed, 1.0);

      double angle = std::acos(intermed);
      smallestAngle = std::min(angle, smallestAngle);
    }
    conn.angleWeight  = smallestAngle;
    conns[i].angleWeight = smallestAngle;
    angleTotal += conn.angleWeight;
  }

  for(int i = 0; i < conns.size(); i++){
    Connection conn = conns[i];
    conn.angleWeight /= angleTotal;
    double offset = std::abs((1/radius) - (1/conn.dist));
    conns[i].strength = offset * conn.angleWeight;
  }
  Vector attr;
  Vector repl;

  for(auto  conn : conns) {
    if (conn.dist >= radius) {
      attr = attr.add(conn.diff.normalize(conn.strength));
    } else {
      repl = repl.subtract(conn.diff.normalize(conn.strength));
    }

  }

  return std::pair<Vector, Vector> (attr, repl);
}

std::vector<Vector> Agent::checkRadar(std::vector<Polygon*> polygons){

  double radius = Agent::RADARDIST;

  std::vector<Edge> rays;
  Vector v = Vector(radius, 0);

  for(double i = 0.0; i< 2*M_PI ; i += (M_PI / (double)numRays)) {
    Vector endPoint = pos.add(v.rotate(i));

    Edge ray(pos, endPoint);

    for(int j = 0 ; j < polygons.size() ; j++){
      std::vector<Edge> edges = polygons[j]->edges();

      for(int k = 0; k < edges.size(); k ++){
        Vector inter = edges[k].intersection(ray);
        if(edges[k].onLine(inter) && ray.onLine(inter)) {
          ray.two = inter;
          if(ray.one.subtract(ray.two).length() < radius){
            rays.push_back(ray);
          }
        }
      }
    }
  }
  std::vector<Vector> pings;
  for(int i = 0; i < rays.size() ;i++){
    pings.push_back(rays[i].two);
  }
  return pings;
}

std::vector<Vector> Agent::checkPointSensor(std::vector<Vector> allPoints){

  std::vector<Vector> inRange;
  for(auto point: allPoints){
    Vector diff = pos.subtract(point);
    double dist = diff.length();

    if (dist <= sensorDistance){
      inRange.push_back(point);
    }
  }

  return inRange;
}
