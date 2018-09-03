#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "collision_reactions.h"

// 
//	MainTask
//

class MainTask : public RobotStrategie {
public:
   MainTask(Eigen::MatrixXd& pts, bool cycle = true)
     	: RobotStrategie("MainTask")           
	, points(pts)
	, nextPoint(0)
    	, cyclic(cycle) { updateGoal(); }

   void reset() {}
   bool isFinish();
   bool execute(ControlInterface& ci);   

private:
   void updateGoal();

   Eigen::MatrixXd& points;
   
   int nextPoint;
   bool cyclic;
   
}; // MainTask

//
//	InitialPosition
//

class InitialPosition : public RobotStrategie {
public:
   InitialPosition(Eigen::Matrix<double,JOINT_NO,1>& q, double vMax=0.5)
    	: RobotStrategie("InitialPosition")
   	, joints(q) 
    	, maxVelocity(vMax) {}
   
   void reset() {}
   bool isFinish();
   bool execute(ControlInterface& ci);

private:
   Eigen::Matrix<double,JOINT_NO,1>& joints;
      
   double maxVelocity;
   
}; // InitialPosition

//
//	FihishWork
//

class FinishWork : public RobotStrategie {
public:
   FinishWork()
   	: RobotStrategie("FinishWork") {}
   
   void reset() {}
   bool isFinish() { return false; }
   bool execute(ControlInterface& ci) { rs.isContinue = false; }
}; // FinishWork


#endif // MAIN_TASK_H
