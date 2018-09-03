// Basic robot reactions

//	UNITS
// Length:       mm
// Angle:        rad
// Velocity:     mm/s
// Rad.velocity: rad/s

#include "kuka/ros_iiwa_link.h"
#include "fsm/main_task.h"
#include "fsm/task_dispatcher.h"
#include <iostream>
#include <fstream>

// tmp
#include "kuka/iiwa_kinematics.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define FREQ  25.0              // message frequency

enum BasicReactions {BASIC_NO, BASIC_STOP, BASIC_TOUCH, BASIC_WAIT, BASIC_REDUNDANCY, BASIC_COMPLIANCE, BASIC_AVOIDANCE};

int main(int argc, char **argv) 
{
   // define current reaction
   BasicReactions reaction = BASIC_REDUNDANCY;
   
   // initialise ROS objects
   ros::init(argc, argv, "MultiscenarioControl");
   ros::NodeHandle nh("~");
   ros::Rate rate(FREQ);

   // prepare initial position
   Eigen::Matrix<double,JOINT_NO,1> startConfig;
   startConfig << 0, 0.5, 0, -1.4, 0, 1.22, 0;
   double startVelocity = 0.5;  // rad/s
   
   // points of trajectory
   Eigen::MatrixXd points(4,CART_NO);
   //   EE       position    orientation
   points <<  530,-100, 400,  PI, 0, -PI,
              530, 100, 400,  PI, 0, -PI,
              730, 100, 400,  PI, 0, -PI,
              730,-100, 400,  PI, 0, -PI;
   bool   cyclic = true;       // default value, can be omitted
   double cartVelocity = 40;   // mm/s

   // initial position
   MoveJoints initial(startConfig, startVelocity);
   // main robot task
   MainTask task(points, cyclic);
   // collision reaction
   StopReaction stopping;
   // Break program
   FinishWork exit;
   // touch reaction
   WaitReaction touchStop(0.4), touchRun(0.4), wait(2);
   // use kinematic redundancy
   double nsMaxVelocity = 2;
   ElbowReaction elbow(nsMaxVelocity);
   // compliance
   double complianceVelocity = 10;
   CompliantReaction compliance(complianceVelocity);

   // define transitions
   initial.addTransition(EV_COMPLETE | EV_KEY, &task);
   // reaction specific transitions
   switch(reaction) {
   case BASIC_NO:    // just follow trajectory
      break;
   case BASIC_STOP:
      task.addTransition(EV_EE_COLLISION | EV_ELBOW_COLLISION | EV_OR, &stopping);
      stopping.addTransition(EV_NO, &task);
      break;
   case BASIC_TOUCH:
      task.addTransition(EV_EE_COLLISION | EV_ELBOW_COLLISION | EV_OR, &touchStop);
      touchStop.addTransition(EV_COMPLETE | EV_KEY, &stopping);
      stopping.addTransition(EV_EE_COLLISION | EV_ELBOW_COLLISION | EV_OR, &touchRun);
      touchRun.addTransition(EV_COMPLETE, &task);
      break;
   case BASIC_WAIT:
      task.addTransition(EV_EE_COLLISION | EV_ELBOW_COLLISION | EV_OR, &wait);
      wait.addTransition(EV_COMPLETE, &task);
      break;
   case BASIC_REDUNDANCY:
      task.addTransition(EV_EE_COLLISION, &stopping);
      task.addTransition(EV_ELBOW_COLLISION, &elbow);
      elbow.addTransition(EV_COMPLETE, &task);
      elbow.addTransition(EV_OUT_OF_LIMITS | EV_KEY, &exit);
      stopping.addTransition(EV_NO, &task);
      break; 
   case BASIC_COMPLIANCE:
      task.addTransition(EV_EE_COLLISION | EV_ELBOW_COLLISION | EV_OR, &compliance);
      compliance.addTransition(EV_COMPLETE, &task);
      //compliance.addTransition(EV_OUT_OF_LIMITS | EV_KEY, &exit);
      break;
   default:
      std::cout << "Undefined reaction " << reaction << std::endl;
      return 1;
   }

   task.addTransition(EV_OUT_OF_LIMITS | EV_KEY, &exit);

   // file for log
   std::ofstream* file = 0;
   // uncomment for logging joint angles
/* 
   std::ofstream log;
   log.open("log.csv");
   file = &log;
*/

   // interaction with ROS
   RosIiwaLink robot(nh, file);
   // using smart servo
   // double servoStiffness = 100;
   //robot.configureSmartServo(servoStiffness);

   // controller
   TaskDispatcher dispatcher(robot, &initial, FREQ, cartVelocity);
   // virtual obstacles
   //double obstacleRad = 20;     // mm
   //double obstacleLifeTime = 2; // s, can be omitted by default
   //dispetcher.addObstacle(50,50,1000,obstacleRad,obstacleLifeTime);
   
   // robot state exchange
   RobotState& rs = RobotState::getInstance();

   // run
   while(ros::ok() && rs.isContinue) {
       if(rs.isRobotConnected) {
          // reaction
          dispatcher.execute();
          rate.sleep();
       }
       else {
          ROS_ERROR("Robot is not connected...");
          ros::Duration(3.0).sleep();
       }
   }

   // stoping
   ROS_INFO("Bye!");
   if(file) file->close();

   return 0;
}
