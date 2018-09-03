#include "collision_reactions.h"
#include "../kuka/iiwa_kinematics.h"

#include <iostream>

#define CART_SQ_ERR   10       // position error in cartesian space
#define ELBOW_REACTION_SEC 1   // execution time for elbow reaction
#define COMPLIANT_REACTION_SEC 0.5
#define THRESHOLD_COMP_TORQUE 8 

#define KSETA 0.1
#define ETA   0.1

using namespace Eigen;

//
// RobotStrategie
//

void RobotStrategie::addTransition(int event, RobotStrategie* strategie) 
{
   if(event & EV_KEY) {
      // add to critical list
      event &= ~EV_KEY;
      critical.push_back(event);
      transitions[event] = strategie;
   } else if(event & EV_OR) {
      // add all internal events
      int i = 0, k = 1;
      while(k != EV_OR) {
         if(event & k) transitions[k] = strategie;
         k = (1 << ++i);
      }
   } else {
      transitions[event] = strategie;
   }
}

RobotStrategie* RobotStrategie::getTransition(int event)
{
   // check critical events
   for(std::vector<int>::iterator it = critical.begin(); it != critical.end(); ++it) {
      if((event & (*it)) == (*it)) return transitions[(*it)];
   }
   // default
   return transitions[event];
}

//
// ElbowReaction
//

bool ElbowReaction::execute(ControlInterface& ci)
{
   // update time
   rest -= rs.T;

   // null space velocity
   Matrix<double,JOINT_NO,1> nsv = rs.externalTorque;
   iiwa14::substituteTorqueNoise(nsv);
   double nsVal = nsv.norm();
   if(nsVal > 0) nsv /= nsVal;
   //nsv.normalize();
   nsv *= velocity;

   Matrix<double,CART_NO,1> cartV;
   // translation
   cartV(0) = rs.goalCartesianPos(0) - rs.cartesianPosition(0);
   cartV(1) = rs.goalCartesianPos(1) - rs.cartesianPosition(1);
   cartV(2) = rs.goalCartesianPos(2) - rs.cartesianPosition(2);
   //cartV /= rs.T;
   double vel= cartV.norm();        
   if(vel > rs.cartVMax) cartV *= (rs.cartVMax / vel); 
   
   // orientaion
   Quaterniond qGoal(rs.goalCartesianPos(6),rs.goalCartesianPos(3),rs.goalCartesianPos(4),rs.goalCartesianPos(5));
   Quaterniond qCurrent(rs.cartesianPosition(6),rs.cartesianPosition(3),rs.cartesianPosition(4),rs.cartesianPosition(5));
   
   Matrix3d mGoal = qGoal.toRotationMatrix();
   Matrix3d mCurrent = qCurrent.toRotationMatrix();
   Matrix3d sRot = (mGoal-mCurrent)*mCurrent.transpose(); // /rs.T;
   Vector3d vRot(sRot(2,1), sRot(0,2), sRot(1,0));
   double w = vRot.norm();
   //if(w > 1) vRot /= w;
   cartV(3) = vRot(0); cartV(4) = vRot(1); cartV(5) = vRot(2);

   //std::cout << cartV << std::endl;
   return ci.setCartVel(cartV, nsv);
}


void ElbowReaction::reset()
{
   rest = ELBOW_REACTION_SEC;
}

bool ElbowReaction::isFinish() 
{
   return rest <= 0;
}

//
// CompliantReaction
//

bool CompliantReaction::execute(ControlInterface& ci)
{
   
   if((rs.events & EV_EE_COLLISION) || (rs.events & EV_ELBOW_COLLISION)) 
      rest = COMPLIANT_REACTION_SEC;
   else
      rest -= rs.T;

   Matrix<double,JOINT_NO,1> dq = rs.externalTorque;
   // filter noise
   /*
   for(int i = 0; i < JOINT_NO; ++i) {
      if(dq(i) > iiwa14::noise[i])
        { dq(i) -= iiwa14::noise[i]; }
      else if(dq(i) < -iiwa14::noise[i])
        { dq(i) += iiwa14::noise[i]; }
      else
        { dq(i) = 0.0; }
   }
   */
   iiwa14::substituteTorqueNoise(dq);
   // all forces
   // find deflection
   double tau = dq.norm();
   if(tau > 0) dq /= tau;
   dq *= (tau > THRESHOLD_COMP_TORQUE ? 1 : (tau / THRESHOLD_COMP_TORQUE)) * velocity * rs.T;

/*
   // maximum force
   double max = 0;
   int joint = -1;
   for(int i = 0; i < JOINT_NO; ++i) {
      if(fabs(dq(i)) > max) {
         max = fabs(dq(i));
	 joint = i;
      }
   }
   if(joint > -1) {
      dq.setZero();
      dq(joint) = velocity * rs.T * max/THRESHOLD_COMP_TORQUE;
   }
   std::cout << dq << std::endl;
*/   
   // find position
   dq += rs.jointPosition;
   
   iiwa14::correctForLimits(dq);

   return ci.setJointPos(dq);
}

//
// EndEffectorReaction
//

//
// EndEffectorReaction
//

bool EndEffectorReaction::execute(ControlInterface& ci)
{
   // check collision
   if((rs.events & EV_EE_COLLISION) ) addCurrentPoint();
   // current point
   Vector3d point(rs.cartesianPosition(0), rs.cartesianPosition(1), rs.cartesianPosition(2));

   Vector3d attr = Vector3d(rs.goalCartesianPos(0),rs.goalCartesianPos(1),rs.goalCartesianPos(2))-point;
   attr.normalize();

   for(int i = 0; i < obstacles.size(); ++i) {
      Vector3d rep = point-obstacles[i];
      double repLen = rep.norm();
      if(repLen < 1E-6) {
         rep = -0.01*attr;
      }
      repLen = (repLen > repulsiveDistance) ? 0 : (repulsiveDistance-repLen);
      rep.normalize();
      // apply repulsive
      std::cout << i << " "  << stiffness*repLen << std::endl;
      attr += stiffness*repLen*rep;
   }
      std::cout << "len " << attr.norm() << std::endl;

   double attrLen = attr.norm();
   // opposite direction
   if(lastDirection.dot(attr) < -0.9*attrLen) {
      std::cout << "Change direction" << std::endl;
      Vector3d dir = point-Vector3d(rs.goalCartesianPos(0),rs.goalCartesianPos(1),rs.goalCartesianPos(2));
      // set direction for avoidance
      Vector3d norm = Vector3d::Zero();
      if(fabs(dir(0)) < 1E-6)
         norm(0) = 1;
      else if(fabs(dir(1)) < 1E-6)
         norm(1) = 1;
      else if(fabs(dir(2)) < 1E-6)
         norm(2) = 1;
      else {
         // generate random direction
         norm(0) = rand() % 100 - 50.0;
	 if(norm(0) == 0) norm(0) = 1;
	 norm(1) = rand() % 100 - 50.0;
	 if(norm(1) == 0) norm(1) = 1;
	 norm(2) = -(norm(0)*dir(0)+norm(1)*dir(1))/dir(2);
      }
      attr = norm;
   }
   //std::cout << attr << std::endl;

   // convert to velocity
   attr.normalize();
   lastDirection = attr;
   attr *= rs.cartVMax;
   std::cout << attr << std::endl;

   Matrix<double,CART_NO,1> cartV;
   cartV.setZero();
   cartV(0) = attr(0);
   cartV(1) = attr(1);
   cartV(2) = attr(2);

   return ci.setCartVel(cartV, iiwa14::zero_null_space);
}

bool EndEffectorReaction::isFinish()
{
   Vector3d d(rs.cartesianPosition(0)-rs.goalCartesianPos(0),rs.cartesianPosition(1)-rs.goalCartesianPos(1),rs.cartesianPosition(2)-rs.goalCartesianPos(2));
   return !((rs.events & EV_EE_COLLISION) || (rs.events & EV_ELBOW_COLLISION)) && d.squaredNorm() < 20;
}

void EndEffectorReaction::reset()
{
   obstacles.clear();
   lastDirection.setZero();

   //addCurrentPoint();
}

void EndEffectorReaction::addCurrentPoint()
{
   obstacles.push_back(Vector3d(rs.cartesianPosition(0), rs.cartesianPosition(1), rs.cartesianPosition(2)));
}


