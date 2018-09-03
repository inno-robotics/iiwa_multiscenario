#ifndef HOMOGENOUS_H
#define HOMOGENOUS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// in order to use this macros, do
//   #include <iostream>
//   #include <ctime>
#define CHECK_PERFORMANCE(FN)   {          \
          clock_t begining = std::clock(); \
          (FN);                            \
          std::cout << (std::clock()-begining) << std::endl; }
          
#define PI 3.14159265358979323846
#define RAD(X)  ((X)*PI/180)

namespace homo {

//
//  ROTATION
//

Eigen::Matrix4d Rx(double q);

Eigen::Matrix4d Ry(double q);

Eigen::Matrix4d Rz(double q);

//
// TRANSLATION
//

Eigen::Matrix4d Tx(double d);

Eigen::Matrix4d Ty(double d);

Eigen::Matrix4d Tz(double d);

//
// ROTATION (DERIVATIVE)
//

Eigen::Matrix4d dRx(double q);

Eigen::Matrix4d dRy(double q);

Eigen::Matrix4d dRz(double q);

//
// TRANSLATION (DERIVATIVE)
//

Eigen::Matrix4d dTx(double d);

Eigen::Matrix4d dTy(double d);

Eigen::Matrix4d dTz(double d);

//
// EULER ANGLES
//

Eigen::Vector3d EulerZYZ(const Eigen::Matrix4d& m, int sign = 1);

Eigen::Vector3d EulerZYX(const Eigen::Matrix4d& m, int sign = 1);

Eigen::Quaterniond fromZYX(double xx, double yy, double zz);

//
// DENAVIT-HARTERBERG PARAMETERS
//

Eigen::Matrix4d DH(double a, double alpha, double d, double theta);

//
// PSEUDO-INVERSE MATRIX
//

Eigen::MatrixXd pinv(const Eigen::MatrixXd& src);

} // namespace homo

#endif // HOMOGENOUS_H
