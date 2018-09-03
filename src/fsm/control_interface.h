#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "current_state.h"

class ControlInterface {
public:
   virtual bool setJointPos(Eigen::Matrix<double,JOINT_NO,1>& jp) = 0;
   virtual bool setCartVel(Eigen::Matrix<double,CART_NO,1>& cv, 
                           Eigen::Matrix<double,JOINT_NO,1>& nsjv) = 0;
};

#endif // CONTROL_INTERFACE_H
