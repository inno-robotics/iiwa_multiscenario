#ifndef IIWA_KINEMATICS
#define IIWA_KINEMATICS

#include "../kinematics/homogenous.h"

#ifndef JOINT_NO
#define JOINT_NO 7
#endif
#ifndef CART_NO
#define CART_NO 6
#endif
#ifndef POSE_QUATERN
#define POSE_QUATERN 7
#endif

namespace iiwa14 {

// angle range
extern double qmax[JOINT_NO], qmin[JOINT_NO];
// velocity range
extern double vmax[JOINT_NO];
// torque threshold approximation
extern double noise[JOINT_NO];

extern Eigen::Matrix<double,JOINT_NO,1> zero_null_space;
// Represent product of 4 matrices
Eigen::Matrix4d RyRzTz(double qy, double qz, double lsum);

// Product Rz * Tz
Eigen::Matrix4d RzTz(double q, double d);
// Derivative of RzTz
Eigen::Matrix4d dRzTz(double q);
// Find matrix for the forward kinematics
Eigen::Matrix4d matFK(Eigen::Matrix<double,JOINT_NO,1>& q);
// Forward kinematics without matrix product
Eigen::Matrix4d FK(Eigen::Matrix<double,JOINT_NO,1>& q);
// Cartesian coordinates
Eigen::Matrix<double,CART_NO,1> cart(Eigen::Matrix<double,JOINT_NO,1>& q, int sign=1);

Eigen::Matrix<double,CART_NO,1> cart(Eigen::Matrix<double,JOINT_NO,1>& q, Eigen::Matrix<double,CART_NO,1>& previous);

// Find Jacobian using matrix product
Eigen::MatrixXd matJacobian(Eigen::Matrix<double,JOINT_NO,1>& q);
// Find Jacobian using precalculated formulas
Eigen::MatrixXd Jacobian(Eigen::Matrix<double,JOINT_NO,1>& q);
// Null space velocity for distance maximization from limits
Eigen::Matrix<double,JOINT_NO,1> nsMaxFromLimits(Eigen::Matrix<double,JOINT_NO,1>& q);
// find joint positions
Eigen::Matrix<double,JOINT_NO+1,3> jointCoordinate(Eigen::Matrix<double,JOINT_NO,1>& q);

Eigen::Matrix<double,POSE_QUATERN,1> cartesianState(Eigen::Matrix<double,JOINT_NO,1>& q);

void correctForLimits(Eigen::Matrix<double,JOINT_NO,1>& q);

void substituteTorqueNoise(Eigen::Matrix<double,JOINT_NO,1>& t);

} // namespace iiwa14

#endif // IIWA_KINEMATICS
