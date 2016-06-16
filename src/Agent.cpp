
#include <math.h>
#include "Agent.h"

bool Agent::move(std::map<int, *Agent> agents, *Polygon[] obs, Vecotr[] points){
  if(this.frozen){
    return true;
  }
  //Get all fields
  //first vector is an attrective, the second is a replusive
  //TODO interAgent
  std::pair<Vector, Vector> agentFields = this.calcInterAgent(agents);
  //TODO calcPolyGons, checkRAdar
  std::pair<Vector, Vector> polyFields = this.calcPolygons(a.checkRadar(obs));
//TODO checkPointSensor
  std::pair<Vector, Vector> pointFields = this.calcPolygons(a.checkPointSensor(points));

  Vector goalVelocity;

  if (a.mode == StateSurround){
    //TODO Vector calls, vector.add
    Vector agentTotal = agentFields.second.add(agentFields.first);
    Vector polyTotal = polyFields.second.add(polyFields.first);
    Vector pointTotal = pointFields.second.add(pointFields.first);

    goalVelocity = agentTotal.Add(polyTotal).Add(pointTotal).Add(this.userForce);
  } else if (this.mode == StateTravelLead){
    // The travel heading exerts a force on leaders.
    // still repeled by other agents and obs
    Vector polyTotal = polyFields.second.add(polyFields.first);
    goalVelocity = agentFields.second.add(polyTotal).add(this.Heading.normalize(0.1));
  } else if (this.mode == StateTravelFollow){
    Agent* leader = others[a.leader];
    Vector diff = leader->pos.subtract(this.pos);
    double dist = diff.length();

    Vector leaderAttr;

    if (dist > this.IARAdius) {
      double strength = fabs((1/(double)this.IARAdius) - (1/dist) * M_PI);

      leaderAttr = diff.normalize(strength);
    }

    Vector agentTotal = agetnFields.second.add(leaderAttr);
    Vector polyTotal = polyFields.second.add(polyFields.first);

    goalVelocity = agentTotal.add(polyTotal);
  }

}
