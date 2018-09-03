#include "main_task.h"
#include "../kuka/iiwa_kinematics.h"
#include "../debug_settings.h"

// tmp
#include <iostream>

using namespace Eigen;

bool MainTask::isFinish()
{
   return !cyclic && (nextPoint >= points.rows());
}

bool MainTask::execute(ControlInterface& ci)
{
   Matrix<double,POSE_QUATERN,1> diff = rs.goalCartesianPos - rs.cartesianPosition;
   //std::cout << diff.squaredNorm() << std::endl;
   //std::cout <<  rs.cartesianPosition << std::endl;
   //std::cout << std::endl << rs.jointPosition << std::endl;
   if(diff.squaredNorm() < CART_SQ_ERR) {
      ++ nextPoint;
      if(nextPoint >= points.rows()) {
         if(cyclic)
            nextPoint = 0;
         else
            return true;    // the task is done
      }  
      
      updateGoal();
   }  
   
   // position
   Matrix<double,CART_NO,1> cartV;
   // translation
   cartV(0) = diff(0); cartV(1) = diff(1); cartV(2) = diff(2);
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
     
   //std::cout << diff << std::endl;

   //std::cout << cartV << std::endl;
   Matrix<double,JOINT_NO,1> nsv;
   if(shape) 
      nsv = Matrix<double,JOINT_NO,1>::Zero();
   else
      nsv = iiwa14::nsMaxFromLimits(rs.jointPosition);

   return ci.setCartVel(cartV, nsv);
}

void MainTask::updateGoal()
{
   Matrix<double,CART_NO,1> goal = points.row(nextPoint);
   Quaterniond q = homo::fromZYX(goal(3),goal(4),goal(5));

   rs.goalCartesianPos(0) = goal(0);
   rs.goalCartesianPos(1) = goal(1);
   rs.goalCartesianPos(2) = goal(2);
   rs.goalCartesianPos(3) = q.x();
   rs.goalCartesianPos(4) = q.y();
   rs.goalCartesianPos(5) = q.z();
   rs.goalCartesianPos(6) = q.w();

#ifdef SHOW_GOAL_POINT
   std::cout << "Go to XYZ " << goal(0) << " " << goal(1) << " " << goal(2) 
             << " xyzw " << q.x() << " " << q.y() << " " <<  q.z() << " " << q.w() << std::endl;
#endif

}

//
//	MoveJoints
//

bool MoveJoints::isFinish()
{
   //std::cout << rs.jointPosition << std::endl;
   return (joints - rs.jointPosition).squaredNorm() < JOINT_SQ_ERR;
}

bool MoveJoints::execute(ControlInterface& ci)
{
   double diffMax = /*rs.T * */maxVelocity;
   Matrix<double,JOINT_NO,1> diff = (joints-rs.jointPosition);
   // speed limitation
   for(int i = 1; i < JOINT_NO; ++i) {
     if(diff(i) > diffMax)
        diff(i) = diffMax;
     else if(diff(i) < -diffMax)
        diff(i) = -diffMax;
   }
   diff += rs.jointPosition;
   return ci.setJointPos(diff);   
}
