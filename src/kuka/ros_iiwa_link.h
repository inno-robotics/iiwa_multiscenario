#ifndef IIWA14_ROBOT_H
#define IIWA14_ROBOT_H

#include <ros/ros.h>
#include <fstream>

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointTorque.h"
#include "iiwa_msgs/ConfigureSmartServo.h"
#include "geometry_msgs/PoseStamped.h"
//#include "iiwa_msgs/JointVelocity.h"

#include "../fsm/control_interface.h"
#include "../kinematics/saturation_in_null_space.h"

class RosIiwaLink : public ControlInterface {
public:
   RosIiwaLink(ros::NodeHandle& nh, std::ofstream *fJoints = 0, std::ofstream *fForces = 0);
   ~RosIiwaLink();
   
   bool setJointPos(Eigen::Matrix<double,JOINT_NO,1>& jp);
   bool setCartVel(Eigen::Matrix<double,CART_NO,1>& cv, Eigen::Matrix<double,JOINT_NO,1>& nsjv);

   void configureSmartServo(double stiffness);
   

   void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
   void jointTorqueCallback(const iiwa_msgs::JointTorque& jt);
   void cartPositionCallback(const geometry_msgs::PoseStamped& cp);

private:   

   bool setSmartServoConfig();

   RobotState& rs;
   
   ros::Subscriber subJointPosition;
   ros::Subscriber subJointTorque;
   ros::Subscriber subCartesPosition;
   ros::Publisher  pubJointPosition;   
   //ros::Publisher  pubJointVelocity;
   ros::Time begin;
   
   iiwa_msgs::JointPosition cmdJointPosition;

   ros::ServiceClient smartClient;
   iiwa_msgs::ConfigureSmartServo smartConfig;
   
   SNSCalculator<JOINT_NO,CART_NO>* sns;
   ros::AsyncSpinner *spinner;
   std::ofstream *fileJoints, *fileForces;

   bool useSmartServo;
   
}; // IiwaRobot

#endif // IIWA14_ROBOT_H
