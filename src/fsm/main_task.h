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
	, shape(true)
    	, cyclic(cycle) { updateGoal(); }

   void reset() {}
   bool isFinish();
   bool execute(ControlInterface& ci);   

   void saveShape(bool flag) { shape = flag; }

private:
   void updateGoal();

   Eigen::MatrixXd& points;
   
   int nextPoint;
   bool cyclic;
   bool shape;
   
}; // MainTask

//
//	MoveJoints
//

class MoveJoints : public RobotStrategie {
public:
   MoveJoints(Eigen::Matrix<double,JOINT_NO,1>& q, double vMax=0.5)
    	: RobotStrategie("MoveJoint")
   	, joints(q) 
    	, maxVelocity(vMax) {}
   
   void reset() {}
   bool isFinish();
   bool execute(ControlInterface& ci);

   void nextPosition(Eigen::Matrix<double,JOINT_NO,1>& q) { joints = q; }

private:
   Eigen::Matrix<double,JOINT_NO,1>& joints;
      
   double maxVelocity;
   
}; // MoveJoints

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
