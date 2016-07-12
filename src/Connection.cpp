#include "Connection.h"

Connection::Connection(Vector diff, double dist, int agentNum){
  this->diff = diff;
  this->dist = dist;
  this->agentNum = agentNum;
}

Connection::Connection(Vector diff, double dist){
  this->diff = diff;
  this->dist = dist;
}

bool operator< (const Connection& l, const Connection& r){
  return l.dist >= r.dist;
}
