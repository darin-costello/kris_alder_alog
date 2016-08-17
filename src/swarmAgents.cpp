#include <map>
#include <string>
#include <iostream>
#include <thread>
#include <tuple>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <geometry_msgs/Pose2D.h>
#include "mm_apriltags_tracker/april_tag_pos.h"
#include "sphero_swarm_node/SpheroTwist.h"
#include "kris_alder_algo/changeLeader.h"

#include "Agent.h"
#include "Polygon.h"
#include "Vector.h"


#define SCALE 9000
#define CAP 30



class App{
public:
  App();
  std::map<int, std::string> numToSphero;
  std::map<int, Agent*> numToAgent;
  std::map<int, Agent*> myAgents;
  std::vector<Polygon*> poly;
  int leader = -1;
  Vector leadDestination;
  ros::NodeHandle n;
  ros::Publisher velPub;
  ros::Subscriber camSub;
  ros::Subscriber leadSub;
  int status = 0;
  void posCallBack(const mm_apriltags_tracker::april_tag_pos::ConstPtr& msg);
  void leadCallBack(const kris_alder_algo::changeLeader::ConstPtr& msg);
  void init();
  void run();
  void moveAgent(std::pair<int, Agent*> pair);
  void stop();

};

App::App(){}

void App::leadCallBack(const kris_alder_algo::changeLeader::ConstPtr& msg){
  int id = msg->id;
  if(id == -1){
    if(leader == -1) return;
    if(myAgents.find(leader) !=  myAgents.end()){
      Agent* lead = myAgents[leader];
      lead->heading = Vector(0,0);
      lead->mode = Agent::StateTransition;
    }else{
      return;
    }
  }else {
    if(leader != -1 && leader != id){
      myAgents[leader]->mode = Agent::StateTravelFollow;
      myAgents[leader]->leader = id;
    }
    if(myAgents.find(id) != myAgents.end()){
      myAgents[id]->mode = Agent::StateTravelLead;
      leadDestination = Vector(msg->pose.x, msg->pose.y);
      leader = id;
    }
  }
}


void App::posCallBack(const mm_apriltags_tracker::april_tag_pos::ConstPtr& msg){

  if(status == 0){
    return;
  } else if (status == 1) {
    for(int i = 0; i< msg->id.size(); i++){
      if(msg->id[i] >= 60){
          std::vector<Vector> vertices;
          vertices.push_back(Vector(msg->pose[i].x - 30, msg->pose[i].y + 30));
          vertices.push_back(Vector(msg->pose[i].x + 30, msg->pose[i].y + 30));
          vertices.push_back(Vector(msg->pose[i].x + 30 , msg->pose[i].y - 30));
          vertices.push_back(Vector(msg->pose[i].x - 30, msg->pose[i].y - 30));
          poly.push_back(new Polygon(vertices));

      }

    }
    status = 2;
    return;
  }

  for(int i = 0; i < msg->id.size(); i++){
    if(msg->id[i] < 60){ // Not an obstacle.
      if(numToAgent.find(msg->id[i]) == numToAgent.end()){
        //checks to see if I've seen this agent before
        // if not create a dummy agent, and add to list.
        numToAgent[msg->id[i]] = new Agent(msg->id[i], Vector(msg->pose[i].x, msg->pose[i].y));
      } else {

        numToAgent[msg->id[i]]->pos = Vector(msg->pose[i].x, msg->pose[i].y);
      }
    }
  }
}

void App::init(){
  velPub = n.advertise<sphero_swarm_node::SpheroTwist>("/cmd_vel", 1);
  camSub = n.subscribe("/april_tag_pos", 1, &App::posCallBack, this);
  leadSub = n.subscribe("/change_leader", 1, &App::leadCallBack, this);
  std::map<std::string, std::string> spheroMap;
  n.getParam("/sphero_swarm/connected", spheroMap);
  for (auto spheroP: spheroMap){
    std::cout << "Enter id number for " << spheroP.first << ":  ";
    int id;
    std::cin >> id;
    numToSphero[id] = spheroP.first;
    numToAgent[id] = new Agent(id, Vector(-1,-1));
    if (id == 1){
     //numToAgent[id]->mode = Agent::StateTravelLead;
    }
    myAgents[id] = numToAgent[id];
  }
  std::cout <<"STarting" << std::endl;
  status = 1;
}

void App::run(){
  ros::Rate loopRate(10);
  while(ros::ok()){
    if(leader != -1)
      myAgents[leader]->heading = leadDestination.subtract(myAgents[leader]->pos);
    for(auto pair: myAgents){
      if(pair.second->pos.x != -1)

       moveAgent(pair);
      //std::thread(&App::moveAgent, pair.id);
    }

    ros::spinOnce();
    loopRate.sleep();
  }
}


void App::moveAgent(std::pair<int, Agent*> pair){

  Vector result = pair.second->move(numToAgent, poly, std::vector<Vector>());
  result = result.scalarMultiply(SCALE);
  result = result.cap(CAP);
  sphero_swarm_node::SpheroTwist twist;

  twist.name = numToSphero[pair.first];
  twist.linear.x = result.x;
  twist.linear.y = -1 * result.y;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  velPub.publish(twist);
  pair.second->averageRadius(numToAgent);
  pair.second->communicate(numToAgent);
}

void App::stop(){
  for (auto spheroP: numToAgent){
    delete spheroP.second;
  }

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "swarmAgents");
  App app;
  app.init();
  app.run();
  app.stop();

}
