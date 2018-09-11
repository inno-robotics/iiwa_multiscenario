#ifndef ROBOT_CURRENT_STATE_H
#define ROBOT_CURRENT_STATE_H

#include <eigen3/Eigen/Dense>
#include <string>

#include "../project_settings.h"

struct RobotState {
   Eigen::Matrix<double,JOINT_NO,1> jointPosition;
   Eigen::Matrix<double,JOINT_NO,1> externalTorque;
   
   Eigen::Matrix<double,POSE_QUATERN,1> goalCartesianPos;  // X, Y, Z, QX, QY, QZ, QW
   Eigen::Matrix<double,POSE_QUATERN,1> cartesianPosition; // X, Y, Z, QX, QY, QZ, QW

   std::string log;    // ?
   
   double T;
   double cartVMax;
   int events;
   int collision;
   bool isRobotConnected;
   bool isContinue;

   static RobotState& getInstance() {
      static RobotState state;
      return state;
   }

private:
   RobotState() 
    : isRobotConnected(false)
    , isContinue(true)
    , cartVMax(50)
    , T(0) {}
   
};  // RobotState


#endif // ROBOT_CURRENT_STATE_H
