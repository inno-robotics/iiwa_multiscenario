#ifndef COLLISION_REACTIONS_H
#define COLLISION_REACTIONS_H

#include <map>
#include <vector>
#include <string>
//#include <eigen3/Eigen/Dense>

#include "control_interface.h"

class RobotStrategie;

enum RobotEvent {
   EV_NO              = 0,
   EV_COMPLETE        = (1 << 0),
   EV_EE_COLLISION    = (1 << 1),
   EV_ELBOW_COLLISION = (1 << 2),
   EV_OUT_OF_LIMITS   = (1 << 3),
   // critical point
   EV_KEY             = (1 << 15),
   // true for each of events
   EV_OR              = (1 << 16)
};   

//
// RobotStrategie
//

class RobotStrategie {
public:
   RobotStrategie(std::string nm) 
   	: name(nm)
   	, rs(RobotState::getInstance()) {}

   // obligatory methods
   virtual void reset() = 0;
   virtual bool isFinish() = 0;
   virtual bool execute(ControlInterface& ci) = 0;   
   // transitions
   void addTransition(int events, RobotStrategie* strategie);
   RobotStrategie* getTransition(int event);
   // log
   std::string getName() { return name; }

protected:
   // current state
   RobotState& rs;
   // list of transitions
   std::map<int,RobotStrategie*> transitions;
   // list of critical transitions
   std::vector<int> critical;
   // current strategie
   std::string name;
   
}; // CollisionReaction

//
// StopReaction
//

class StopReaction : public RobotStrategie {
public:
   StopReaction()
    : RobotStrategie("StopReaction") {}
   	
   void reset() {}
   //bool isFinish() { return !((rs.events & EV_EE_COLLISION) || (rs.events & EV_ELBOW_COLLISION)); }
   bool isFinish() { return false; }
   bool execute(ControlInterface& ci) { return true; }

}; // StopReaction

//
// WaitReaction
//

class WaitReaction : public RobotStrategie {
public:
   WaitReaction(double waitSec)
    : RobotStrategie("WaitReaction") 
    , duration(waitSec) {}

   bool isFinish() { return rest <= 0.0; }
   bool execute(ControlInterface& ci) { rest -= rs.T; return true; }
   void reset() { rest = duration; }

private:
   double rest, duration;

}; // WaitReaction

//
// ElbowReaction
//

class ElbowReaction : public RobotStrategie {
public:
   ElbowReaction(double nullSpaceVMax)
    : RobotStrategie("ElbowReaction") 
    , velocity(nullSpaceVMax) {}

   bool isFinish();
   bool execute(ControlInterface& ci);
   void reset();

public:
   double velocity;
   clock_t rest;

}; // ElbowReaction

//
// CompliantReaction
//

class CompliantReaction : public RobotStrategie {
public:
   CompliantReaction(double dq)
    : RobotStrategie("CompliantReaction") 
    , velocity(dq) {}

   bool isFinish() { return rest <= 0; }
   bool execute(ControlInterface& ci);
   void reset() {}

private:

   double velocity;
   double rest;

}; // CompliantReaction

//
// EndEffectorReaction
//

class EndEffectorReaction : public RobotStrategie {
public:
   EndEffectorReaction(double distanceMm=30, double stiffnessVal=1)
    : RobotStrategie("EndEffectorReaction") 
    , repulsiveDistance(distanceMm)
    , stiffness(stiffnessVal) {}

    bool isFinish();
    bool execute(ControlInterface& ci);
    void reset();

private:
   void addCurrentPoint();

   Eigen::Matrix<double,JOINT_NO+1,3> joints;

   std::vector<Eigen::Vector3d> obstacles;
   Eigen::Vector3d lastDirection;

   double repulsiveDistance;
   double stiffness;
}; // EndEffectorReaction


#endif // COLLISION_REACTIONS_H
