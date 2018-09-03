#ifndef TASK_DISPETCHER_H
#define TASK_DISPETCHER_H

#include "collision_reactions.h"
#include "collision_detection.h"

class TaskDispatcher {
public:
   TaskDispatcher(ControlInterface& control, RobotStrategie* r, double rate, double vcMax)
       : rs(RobotState::getInstance())
       , ci(control)
       , kernel(r) { rs.T = 1.0/rate; rs.cartVMax = vcMax; }
   
   bool execute();

   void addObstacle(double x, double y, double z, double R, double T=-1) { 
      detector.addObstacle(x,y,z,R,T); 
   }

private:
   int updateEvents();
   
   CollisionDetector detector;
   CollisionDetectionResult collision;   

   ControlInterface& ci;
   RobotState& rs;

   RobotStrategie* kernel;

}; // TaskDispatcher

#endif // TASK_DISPATCHER_H
