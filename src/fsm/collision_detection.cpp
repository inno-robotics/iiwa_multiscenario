#include <cmath>

#include "collision_detection.h"
#include "../kuka/iiwa_kinematics.h"
#include "../debug_settings.h"

// temporary
#include <iostream>

#define VIRTUAL_OBSTACLE_FORCE 0.8

using namespace Eigen;

CollisionDetector::CollisionDetector()
{
}

void CollisionDetector::analyze(RobotState& rs, CollisionDetectionResult &res)
{
#ifdef EMULATE_MOVEMENT
   checkVirtualObstacles(rs);
#endif // EMULATE_MOVEMENT
   rs.collision = 0;

   simple_last_torque(rs, res);
}

void CollisionDetector::simple_maximum_torque(RobotState& rs, CollisionDetectionResult &res)
{
   int joint=-1;

   double max = 0, diff;
   for(int i = 0; i < JOINT_NO; i++) {
      diff = fabs(rs.externalTorque(i))-iiwa14::noise[i];
      if(diff > max) {
         max = diff; 
         joint = i;
      }
   }

   res.jointNo = joint;
   res.found = (joint != -1);
   if(res.found) rs.collision |= (1 << joint);
}

void CollisionDetector::simple_last_torque(RobotState& rs, CollisionDetectionResult &res)
{
   int joint=-1;
   for(int i = 0; i < JOINT_NO; i++) {
      if(fabs(rs.externalTorque(i)) > iiwa14::noise[i]) joint = i; 
   }
   
   res.jointNo = joint;
   res.found = (joint != -1);
   if(res.found) rs.collision |= (1 << joint);
}

void CollisionDetector::checkVirtualObstacles(RobotState& rs)
{
   rs.externalTorque.setZero();
   Matrix<double,JOINT_NO+1,3> points = iiwa14::jointCoordinate(rs.jointPosition);

   for(int i = 0; i < obstacles.size(); ++i) {
      bool getCollision = false;
      for(int r = 0; r < points.rows(); ++r) {
         if(obstacles[i].isCollision(points(r,0),points(r,1),points(r,2))) {
	    for(int k = 0; k < r; ++k) { rs.externalTorque(k) += VIRTUAL_OBSTACLE_FORCE; }
	    getCollision = true;
	 } 
      } 
      // life time
      if(getCollision) obstacles[i].updateTime(rs.T);
   }

   // remove "old"
   int i = 0;
   while(i < obstacles.size()) {
      if(obstacles[i].lifeTime > 0) {
         i++;
      }
      else
         obstacles.erase(obstacles.begin()+i);
   }
}

void VirtualObstacle::updateTime(double dt)
{
   lifeTime -= dt;
}
