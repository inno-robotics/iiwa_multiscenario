#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <vector>

#include "current_state.h"

//
// CollisionDetectionResult
//

struct CollisionDetectionResult {
    int  jointNo;
    bool found;
}; //  CollisionDetectionResult

//
// VirtualObstacle
//

struct VirtualObstacle {
   double x, y, z;
   double RR;
   double lifeTime;

   bool isCollision(double px, double py, double pz) {
      return (px-x)*(px-x)+(py-y)*(py-y)+(pz-z)*(pz-z) < RR;
   }

   void updateTime(double dt);

   VirtualObstacle(double px, double py, double pz, double pr)
    : x(px)
    , y(py)
    , z(pz)
    , lifeTime(1E100)
    , RR(pr*pr) {}
  
}; //  VirtualObstacle

//
// CollisionDetector
//

class CollisionDetector {
public:
   CollisionDetector();
   
   void analyze(RobotState& rs, CollisionDetectionResult& res);
   
   //void setThreshold(Eigen::Matrix<double,JOINT_NO,1>& noise) { noiseThreshold = noise; }

   void addObstacle(double x, double y, double z, double R, double T) { 
      obstacles.push_back(VirtualObstacle(x,y,z,R)); 
      if(T > 0) obstacles.back().lifeTime = T;
   }
   
private:
   void checkVirtualObstacles(RobotState& rs);

   void simple_maximum_torque(RobotState& rs, CollisionDetectionResult& res);
   void simple_last_torque(RobotState& rs, CollisionDetectionResult& res);

   std::vector<VirtualObstacle> obstacles;

}; // CollisionDetector

#endif // COLLISION_DETECTION_H
