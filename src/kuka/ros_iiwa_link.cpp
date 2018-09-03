#include "ros_iiwa_link.h"
#include "iiwa_kinematics.h"
#include "../debug_settings.h"
#include "iiwa_msgs/ControlMode.h"

//#include <iostream>


#define SEP    ","

using namespace Eigen;

RosIiwaLink::RosIiwaLink(ros::NodeHandle& nh, std::ofstream *f)
         : ControlInterface()
         , file(f)
	 , useSmartServo(false)
         , rs(RobotState::getInstance())
{   
   // subscribe
   subJointPosition = nh.subscribe("/iiwa/state/JointPosition", 1, &RosIiwaLink::jointPositionCallback, this);
   subJointTorque = nh.subscribe("/iiwa/state/JointExtTorque", 1, &RosIiwaLink::jointTorqueCallback, this);
   subCartesPosition = nh.subscribe("/iiwa/state/CartesianPose", 1, &RosIiwaLink::cartPositionCallback, this);
   // publish
   pubJointPosition = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);
   // smart servo
   smartClient = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
   // calculator
   sns = new SNSCalculator<JOINT_NO,CART_NO>(iiwa14::qmax, iiwa14::qmin, iiwa14::vmax);
   
#ifdef EMULATE_MOVEMENT
   rs.jointPosition = Matrix<double,JOINT_NO,1>::Zero();
   rs.cartesianPosition = Matrix<double,POSE_QUATERN,1>::Zero();
   rs.isRobotConnected = true;   
#endif

   spinner = new ros::AsyncSpinner(1);
   spinner->start();
}

RosIiwaLink::~RosIiwaLink()
{
   spinner->stop();

   delete spinner;
   delete sns;
}

void RosIiwaLink::jointPositionCallback(const iiwa_msgs::JointPosition& jp)
{
   rs.isRobotConnected = true;
   rs.jointPosition(0) = jp.position.a1;
   rs.jointPosition(1) = jp.position.a2;
   rs.jointPosition(2) = jp.position.a3;
   rs.jointPosition(3) = jp.position.a4;
   rs.jointPosition(4) = jp.position.a5;
   rs.jointPosition(5) = jp.position.a6;
   rs.jointPosition(6) = jp.position.a7;
}

void RosIiwaLink::jointTorqueCallback(const iiwa_msgs::JointTorque& jt)
{
   rs.isRobotConnected = true;
   rs.externalTorque(0) = jt.torque.a1;
   rs.externalTorque(1) = jt.torque.a2;
   rs.externalTorque(2) = jt.torque.a3;
   rs.externalTorque(3) = jt.torque.a4;
   rs.externalTorque(4) = jt.torque.a5;
   rs.externalTorque(5) = jt.torque.a6;
   rs.externalTorque(6) = jt.torque.a7;
}

void RosIiwaLink::cartPositionCallback(const geometry_msgs::PoseStamped& cp)
{
   rs.isRobotConnected = true;
   // transform to millimeters
   rs.cartesianPosition(0) = cp.pose.position.x*1000;
   rs.cartesianPosition(1) = cp.pose.position.y*1000;
   rs.cartesianPosition(2) = cp.pose.position.z*1000;

   rs.cartesianPosition(3) = cp.pose.orientation.x;
   rs.cartesianPosition(4) = cp.pose.orientation.y;
   rs.cartesianPosition(5) = cp.pose.orientation.z;
   rs.cartesianPosition(6) = cp.pose.orientation.w;
}

bool RosIiwaLink::setJointPos(Matrix<double,JOINT_NO,1>& jp)
{
   if(useSmartServo && !setSmartServoConfig()) return false;

   cmdJointPosition.position.a1 = jp(0);
   cmdJointPosition.position.a2 = jp(1);
   cmdJointPosition.position.a3 = jp(2);
   cmdJointPosition.position.a4 = jp(3);
   cmdJointPosition.position.a5 = jp(4);
   cmdJointPosition.position.a6 = jp(5);
   cmdJointPosition.position.a7 = jp(6);
   
   pubJointPosition.publish(cmdJointPosition);
   
   if(file) {
      *file << jp(0) << SEP << jp(1) << SEP << jp(2) << SEP << jp(3) << SEP << jp(4) << SEP << jp(5) << SEP << jp(6) << std::endl;
   }

#ifdef EMULATE_MOVEMENT
   rs.cartesianPosition = iiwa14::cartesianState(jp);
   rs.jointPosition = jp;
#endif
   
   return true;
}

bool RosIiwaLink::setCartVel(Matrix<double,CART_NO,1> &cv, Matrix<double,JOINT_NO,1> &nsjv)
{
   if(useSmartServo && !setSmartServoConfig()) return false;

   Matrix<double,JOINT_NO,1> diff = sns->jointVelocity(cv, nsjv, rs.jointPosition,
                                                       iiwa14::Jacobian(rs.jointPosition), rs.T);
                                                       
   //std::cout << diff << std::endl;
   //std::cout << rs.T << std::endl;
   
   //diff *= rs.T;
   diff += rs.jointPosition;
   setJointPos(diff);
   
   if(file) {
      *file << diff(0) << SEP << diff(1) << SEP << diff(2) << SEP << diff(3) << SEP << diff(4) 
                       << SEP << diff(5) << SEP << diff(6) << std::endl;
   }
   
#ifdef EMULATE_MOVEMENT
   rs.jointPosition = diff;
   rs.cartesianPosition = iiwa14::cartesianState(diff);
#endif

   return sns->successfull();
}

void RosIiwaLink::configureSmartServo(double stiffness)
{
#ifdef INDIGO_SUNRISE_17
   smartConfig.request.mode.mode = iiwa_msgs::SmartServoMode::JOINT_IMPEDANCE;
   smartConfig.request.mode.joint_stiffness.stiffness.a1 = stiffness;
   smartConfig.request.mode.joint_stiffness.stiffness.a2 = stiffness;
   smartConfig.request.mode.joint_stiffness.stiffness.a3 = stiffness;
   smartConfig.request.mode.joint_stiffness.stiffness.a4 = stiffness;
   smartConfig.request.mode.joint_stiffness.stiffness.a5 = stiffness;
   smartConfig.request.mode.joint_stiffness.stiffness.a6 = stiffness;
   smartConfig.request.mode.joint_stiffness.stiffness.a7 = stiffness;
#endif

#ifdef KINETIC_SUNRISE_17
   smartConfig.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;
   smartConfig.request.joint_impedance.joint_stiffness.a1 = stiffness;
   smartConfig.request.joint_impedance.joint_stiffness.a2 = stiffness;
   smartConfig.request.joint_impedance.joint_stiffness.a3 = stiffness;
   smartConfig.request.joint_impedance.joint_stiffness.a4 = stiffness;
   smartConfig.request.joint_impedance.joint_stiffness.a5 = stiffness;
   smartConfig.request.joint_impedance.joint_stiffness.a6 = stiffness;
   smartConfig.request.joint_impedance.joint_stiffness.a7 = stiffness;

   smartConfig.request.joint_impedance.joint_damping.a1 = 0.7;
   smartConfig.request.joint_impedance.joint_damping.a2 = 0.7;
   smartConfig.request.joint_impedance.joint_damping.a3 = 0.7;
   smartConfig.request.joint_impedance.joint_damping.a4 = 0.7;
   smartConfig.request.joint_impedance.joint_damping.a5 = 0.7;
   smartConfig.request.joint_impedance.joint_damping.a6 = 0.7;
   smartConfig.request.joint_impedance.joint_damping.a7 = 0.7;
#endif
   
   useSmartServo = true;
}

bool RosIiwaLink::setSmartServoConfig()
{
   useSmartServo = false;
   if(smartClient.call(smartConfig)) {
      if(smartConfig.response.success)
         ROS_INFO("SmartServo Service sucessfully called.");
      else {
         ROS_ERROR("Config failed, Java error: %s", smartConfig.response.error.c_str());
	 rs.isContinue = false;
	 return false;
      }
   } else {
      ROS_ERROR("Config failed - service could not be called - QUITTING NOW !");
      rs.isContinue = false;
      return false;
   }
   return true;
}
