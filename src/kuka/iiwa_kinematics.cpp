#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "iiwa_kinematics.h"
// temporary
#include <iostream>

#define LINK01 360
#define LINK23 420
#define LINK45 400
#define LINK67 126

#define GAP  5     // program limit for position
#define VGAP 3     // program limit for velocity

#define STATE_CORRECTION_COEFF 0.1

using namespace Eigen;

// angle range
double iiwa14::qmax[] = {RAD(170-GAP),RAD(120-GAP),RAD(170-GAP),RAD(120-GAP),RAD(170-GAP),RAD(120-GAP),RAD(175-GAP)};
double iiwa14::qmin[] = {-RAD(170-GAP),-RAD(120-GAP),-RAD(170-GAP),-RAD(120-GAP),-RAD(170-GAP),-RAD(120-GAP),-RAD(175-GAP)};
// velocity range
double iiwa14::vmax[] = {RAD(85-VGAP),RAD(85-VGAP),RAD(100-VGAP),RAD(75-VGAP),RAD(130-VGAP),RAD(135-VGAP),RAD(135-VGAP)};
// external torque thresholds (approximation)
double iiwa14::noise[] = {2.5, 4, 1.5, 3.5, 1.5, 1.5, 1};

Matrix<double,JOINT_NO,1> iiwa14::zero_null_space = Matrix<double,JOINT_NO,1>::Zero();
// Represent product of 4 matrices
Matrix4d iiwa14::RyRzTz(double qy, double qz, double lsum) {
   double sy = sin(qy), cy = cos(qy);
   double sz = sin(qz), cz = cos(qz);
   Matrix4d res;
   res <<  cy*cz,-cy*sz, sy, lsum*sy,
              sz,    cz,  0,       0,
	  -sy*cz, sy*sz, cy, lsum*cy,
	       0,     0,  0,       1;
   return res;
}

// Product Rz * Tz
Matrix4d iiwa14::RzTz(double q, double d) {
   double s = sin(q), c = cos(q);
   Matrix4d m;
   m << c,-s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, d,
        0, 0, 0, 1;
   return m;
}

// Derivative of RzTz
Matrix4d iiwa14::dRzTz(double q) {
   double s = sin(q), c = cos(q);
   Matrix4d m;
   m << -s,-c, 0, 0,
         c,-s, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
   return m;  
}
// Find matrix for the forward kinematics
Matrix4d iiwa14::matFK(Matrix<double,JOINT_NO,1>& q) {
   // base + link1   
   Matrix4d m = iiwa14::RzTz(q(0),LINK01);   
   // link2 + link3
   m *= iiwa14::RyRzTz(q(1),q(2)+PI,LINK23);
   // link4 + link5
   m *= iiwa14::RyRzTz(q(3),q(4)-PI,LINK45);
   // link6 + link7
   m *= iiwa14::RyRzTz(q(5),q(6),LINK67);
   return m;
}

Matrix4d iiwa14::FK(Matrix<double,JOINT_NO,1>& q) {
   // angles
   double c0 = cos(q(0)), s0 = sin(q(0));
   double c1 = cos(q(1)), s1 = sin(q(1));
   double c2 = -cos(q(2)), s2 = -sin(q(2));       // q(2)+pi
   double c3 = cos(-q(3)), s3 = sin(-q(3));
   double c4 = -cos(q(4)), s4 = -sin(q(4));       // q(4)-pi
   double c5 = cos(q(5)), s5 = sin(q(5));
   double c6 = cos(q(6)), s6 = sin(q(6));
   Matrix4d m;
   double _grp0 = (-c0*c1*s2-s0*c2), _grp1 = (c0*s1*s3+(c0*c1*c2-s0*s2)*c3);
   double _grp2 = (c0*s1*c3-(c0*c1*c2-s0*s2)*s3);
   m(0,0) = (_grp0*c4-_grp1*s4)*s6+((_grp0*s4+_grp1*c4)*c5-_grp2*s5)*c6; 
   m(0,1) = (_grp0*c4-_grp1*s4)*c6-((_grp0*s4+_grp1*c4)*c5-_grp2*s5)*s6; 
   m(0,2) = (_grp0*s4+_grp1*c4)*s5+_grp2*c5; 
   m(0,3) = LINK67*((_grp0*s4+_grp1*c4)*s5+_grp2*c5)+LINK45*_grp2+LINK23*c0*s1;
   double _grp3 = (c0*c2-s0*c1*s2), _grp4 = (s0*s1*s3+(c0*s2+s0*c1*c2)*c3);
   double _grp5 = (s0*s1*c3-(c0*s2+s0*c1*c2)*s3);
   m(1,0) = (_grp3*c4-_grp4*s4)*s6+((_grp3*s4+_grp4*c4)*c5-_grp5*s5)*c6; 
   m(1,1) = (_grp3*c4-_grp4*s4)*c6-((_grp3*s4+_grp4*c4)*c5-_grp5*s5)*s6; 
   m(1,2) = (_grp3*s4+_grp4*c4)*s5+_grp5*c5; 
   m(1,3) = LINK67*((_grp3*s4+_grp4*c4)*s5+_grp5*c5)+LINK45*_grp5+LINK23*s0*s1;
   double _grp6 = (c1*s3-s1*c2*c3), _grp7 = (s1*c2*s3+c1*c3);
   m(2,0) = (s1*s2*c4-_grp6*s4)*s6+((s1*s2*s4+_grp6*c4)*c5-_grp7*s5)*c6; 
   m(2,1) = (s1*s2*c4-_grp6*s4)*c6-((s1*s2*s4+_grp6*c4)*c5-_grp7*s5)*s6; 
   m(2,2) = (s1*s2*s4+_grp6*c4)*s5+_grp7*c5; 
   m(2,3) = LINK67*((s1*s2*s4+_grp6*c4)*s5+_grp7*c5)+LINK45*_grp7+LINK23*c1+LINK01; 
   m(3,0) = 0.0; 
   m(3,1) = 0.0; 
   m(3,2) = 0.0; 
   m(3,3) = 1.0; 
   
   return m;
}

Matrix<double,CART_NO,1> iiwa14::cart(Matrix<double,JOINT_NO,1>& q, int sign)
{
   Matrix4d m = iiwa14::FK(q);
   Matrix<double,CART_NO,1> res;
   Vector3d a = homo::EulerZYX(m,sign);

   //std::cout << m << std::endl;

   res(0) = m(0,3);
   res(1) = m(1,3);
   res(2) = m(2,3);
   res(3) = a(0);
   res(4) = a(1);
   res(5) = a(2);

   //std::cout << res << std::endl;

   return res;
}

Matrix<double,CART_NO,1> iiwa14::cart(Matrix<double,JOINT_NO,1>& q, Matrix<double,CART_NO,1>& previous)
{
   Matrix4d m = iiwa14::FK(q);
   Matrix<double,CART_NO,1> res;
   Vector3d a = homo::EulerZYX(m,1);

   if(m(2,0) == -1) {
      a(0) += previous(5);
      a(2) = previous(5);
   } 
   else if(m(2,0) == 1) {
      a(0) = previous(5)-a(0);
      a(2) = previous(5);
   }
   else if(previous(3)*a(0)+previous(4)*a(1)+previous(5)*a(2) < 0) {
      a = homo::EulerZYX(m,-1);
   }

   //std::cout << m << std::endl;

   res(0) = m(0,3);
   res(1) = m(1,3);
   res(2) = m(2,3);
   res(3) = a(0);
   res(4) = a(1);
   res(5) = a(2);

   //std::cout << res << std::endl;

   return res;
}

// Find Jacobian using matrix product
MatrixXd iiwa14::matJacobian(Matrix<double,JOINT_NO,1>& q) {
   // Find fragments
   Matrix4d acc = iiwa14::RzTz(q(0), LINK01);
   Matrix4d m1 = homo::Ry(q(1));
   Matrix4d m2 = iiwa14::RzTz(q(2)+PI, LINK23);
   Matrix4d m3 = homo::Ry(q(3));
   Matrix4d m4 = iiwa14::RzTz(q(4)-PI, LINK45);
   Matrix4d m5 = homo::Ry(q(5));
   Matrix4d m6 = iiwa14::RzTz(q(6), LINK67);
   // Correction for orientation
   Matrix4d full = acc*m1*m2*m3*m4*m5*m6;
   full(0,3) = 0; full(1,3) = 0; full(2,3) = 0;
   Matrix4d rot = full.transpose(); 
   // jacobian
   MatrixXd res(6,7);
   // 0 column   
   Matrix4d jac = iiwa14::dRzTz(q(0))*m1*m2*m3*m4*m5*m6*rot;
   res(0,0) = jac(0,3); res(1,0) = jac(1,3); res(2,0) = jac(2,3); res(3,0) = jac(2,1); res(4,0) = jac(0,2); res(5,0) = jac(1,0);
   // 1 column
   jac = acc*homo::dRy(q(1))*m2*m3*m4*m5*m6*rot;
   res(0,1) = jac(0,3); res(1,1) = jac(1,3); res(2,1) = jac(2,3); res(3,1) = jac(2,1); res(4,1) = jac(0,2); res(5,1) = jac(1,0);
   acc *= m1;
   // 2 column
   jac = acc*iiwa14::dRzTz(q(2)+PI)*m3*m4*m5*m6*rot;
   res(0,2) = jac(0,3); res(1,2) = jac(1,3); res(2,2) = jac(2,3); res(3,2) = jac(2,1); res(4,2) = jac(0,2); res(5,2) = jac(1,0);
   acc *= m2;
   // 3 column
   jac = acc*homo::dRy(q(3))*m4*m5*m6*rot;
   res(0,3) = jac(0,3); res(1,3) = jac(1,3); res(2,3) = jac(2,3); res(3,3) = jac(2,1); res(4,3) = jac(0,2); res(5,3) = jac(1,0);
   acc *= m3;
   // 4 column
   jac = acc*iiwa14::dRzTz(q(4)-PI)*m5*m6*rot;
   res(0,4) = jac(0,3); res(1,4) = jac(1,3); res(2,4) = jac(2,3); res(3,4) = jac(2,1); res(4,4) = jac(0,2); res(5,4) = jac(1,0);
   acc *= m4;
   // 5 column
   jac = acc*homo::dRy(q(5))*m6*rot;
   res(0,5) = jac(0,3); res(1,5) = jac(1,3); res(2,5) = jac(2,3); res(3,5) = jac(2,1); res(4,5) = jac(0,2); res(5,5) = jac(1,0);
   acc *= m5;
   // 6 column
   jac = acc*iiwa14::dRzTz(q(6))*rot;
   res(0,6) = jac(0,3); res(1,6) = jac(1,3); res(2,6) = jac(2,3); res(3,6) = jac(2,1); res(4,6) = jac(0,2); res(5,6) = jac(1,0);
   
   return res;
}

// Find Jacobian using precalculated formulas
MatrixXd iiwa14::Jacobian(Matrix<double,JOINT_NO,1>& q) {
   // angles
   double c0 = cos(q(0)), s0 = sin(q(0));
   double c1 = cos(q(1)), s1 = sin(q(1));
   double c2 = -cos(q(2)), s2 = -sin(q(2));       // cos(q(2)+pi) = -cos(q(2)), the same for sin
   double c3 = cos(-q(3)), s3 = sin(-q(3));       // initially was calculated for negative argument
   double c4 = -cos(q(4)), s4 = -sin(q(4));       // cos(q(4)-pi) = -cos(q(4)), the same for sin
   double c5 = cos(q(5)), s5 = sin(q(5));
   double c6 = cos(q(6)), s6 = sin(q(6));
   // initialization
   MatrixXd m(6,7);
   //Matrix67d m;
   double _grp0 = (c0*s1*s3+(c0*c1*c2-s0*s2)*c3), _grp1 = (c0*s1*c3-(c0*c1*c2-s0*s2)*s3);
   double _grp2 = (c0*c2-s0*c1*s2), _grp3 = (s0*s1*s3+(c0*s2+s0*c1*c2)*c3);
   double _grp4 = (s0*s1*c3-(c0*s2+s0*c1*c2)*s3), _grp5 = (c1*s3-s1*c2*c3);
   double _grp6 = (s1*s2*s4+_grp5*c4)*s5, _grp7 = (s1*c2*s3+c1*c3);
   double _grp8 = (-c0*c1*s2-s0*c2);
   m(0,0) = (-LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5))-LINK45*_grp4-LINK23*s0*s1; 
   m(0,1) = c0*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1); 
   m(0,2) = s0*s1*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1)-c1*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4+LINK23*s0*s1); 
   m(0,3) = _grp2*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7)-s1*s2*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4); 
   m(0,4) = ((-s1*c2*s3)-c1*c3)*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4)+_grp4*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7); 
   m(0,5) = LINK67*(_grp5*s4-s1*s2*c4)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*(_grp2*c4-_grp3*s4)*(_grp6+_grp7*c5); 
   m(0,6) = LINK67*(_grp6+_grp7*c5)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*((-_grp6)-_grp7*c5)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5); 
   m(1,0) = LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1; 
   m(1,1) = s0*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1); 
   m(1,2) = c1*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1)-c0*s1*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1); 
   m(1,3) = s1*s2*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1)+(c0*c1*s2+s0*c2)*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7); 
   m(1,4) = _grp7*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1)+((c0*c1*c2-s0*s2)*s3-c0*s1*c3)*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7); 
   m(1,5) = LINK67*(s1*s2*c4-_grp5*s4)*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK67*(_grp0*s4-_grp8*c4)*(_grp6+_grp7*c5); 
   m(1,6) = LINK67*(_grp6+_grp7*c5)*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK67*(_grp6+_grp7*c5)*((-(_grp8*s4+_grp0*c4)*s5)-_grp1*c5); 
   m(2,0) = 0.0; 
   m(2,1) = (-s0*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4+LINK23*s0*s1))-c0*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1); 
   m(2,2) = c0*s1*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4+LINK23*s0*s1)-s0*s1*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1); 
   m(2,3) = _grp8*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4)+(s0*c1*s2-c0*c2)*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1); 
   m(2,4) = _grp1*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4)+((c0*s2+s0*c1*c2)*s3-s0*s1*c3)*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1); 
   m(2,5) = LINK67*(_grp8*c4-_grp0*s4)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*(_grp3*s4-_grp2*c4)*((_grp8*s4+_grp0*c4)*s5+_grp1*c5); 
   m(2,6) = LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)*((-(_grp2*s4+_grp3*c4)*s5)-_grp4*c5); 
   m(3,0) = 0.0; 
   m(3,1) = -s0; 
   m(3,2) = c0*s1; 
   m(3,3) = (-c0*c1*s2)-s0*c2; 
   m(3,4) = c0*s1*c3-(c0*c1*c2-s0*s2)*s3; 
   m(3,5) = _grp8*c4-_grp0*s4; 
   m(3,6) = (_grp8*s4+_grp0*c4)*s5+_grp1*c5; 
   m(4,0) = 0.0; 
   m(4,1) = c0; 
   m(4,2) = s0*s1; 
   m(4,3) = c0*c2-s0*c1*s2; 
   m(4,4) = s0*s1*c3-(c0*s2+s0*c1*c2)*s3; 
   m(4,5) = _grp2*c4-_grp3*s4; 
   m(4,6) = (_grp2*s4+_grp3*c4)*s5+_grp4*c5; 
   m(5,0) = 1.0; 
   m(5,1) = 0.0; 
   m(5,2) = c1; 
   m(5,3) = s1*s2; 
   m(5,4) = s1*c2*s3+c1*c3; 
   m(5,5) = s1*s2*c4-_grp5*s4; 
   m(5,6) = _grp6+_grp7*c5; 
   
   return m;
}

Matrix<double,JOINT_NO,1> iiwa14::nsMaxFromLimits(Matrix<double,JOINT_NO,1>& q)
{
   Matrix<double,JOINT_NO,1> res;
   for(int i = 0; i < JOINT_NO; ++i) {
      double del = iiwa14::qmax[i]-iiwa14::qmin[i];
      res(i) = -STATE_CORRECTION_COEFF*(q(i)-(iiwa14::qmax[i]+iiwa14::qmin[i])*0.5)/(del);
   }

   return res;
}

Matrix<double,JOINT_NO+1,3> iiwa14::jointCoordinate(Matrix<double,JOINT_NO,1>& q)
{
   Matrix<double,JOINT_NO+1,3> res;
//   res.setZero();
   // base + link1   
   Matrix4d m = iiwa14::RzTz(q(0),LINK01);   
   res(1,0) = m(0,3); res(1,1) = m(1,3); res(1,2) = m(2,3);
   // link2 + link3
   m *= iiwa14::RyRzTz(q(1),q(2)+PI,LINK23);
   res(3,0) = m(0,3); res(3,1) = m(1,3); res(3,2) = m(2,3);
   // link4 + link5
   m *= iiwa14::RyRzTz(q(3),q(4)-PI,LINK45);
   res(5,0) = m(0,3); res(5,1) = m(1,3); res(5,2) = m(2,3);
   // link6 + link7
   m *= iiwa14::RyRzTz(q(5),q(6),LINK67);
   res(7,0) = m(0,3); res(7,1) = m(1,3); res(7,2) = m(2,3);

   res(0,0) = 0.5*(res(1,0));
   res(0,1) = 0.5*(res(1,1));
   res(0,2) = 0.5*(res(1,2));

   res(2,0) = 0.5*(res(3,0)+res(1,0));
   res(2,1) = 0.5*(res(3,1)+res(1,1));
   res(2,2) = 0.5*(res(3,2)+res(1,2));

   res(4,0) = 0.5*(res(5,0)+res(3,0));
   res(4,1) = 0.5*(res(5,1)+res(3,1));
   res(4,2) = 0.5*(res(5,2)+res(3,2));

   res(6,0) = 0.5*(res(7,0)+res(5,0));
   res(6,1) = 0.5*(res(7,1)+res(5,1));
   res(6,2) = 0.5*(res(7,2)+res(5,2));
   
   return res;
}

Matrix<double,POSE_QUATERN,1> iiwa14::cartesianState(Matrix<double,JOINT_NO,1>& q) 
{
   Matrix4d fk = iiwa14::FK(q);
   Matrix3d rot = fk.block(0,0,3,3);
   Quaterniond v(rot);

   Matrix<double,POSE_QUATERN,1> res;
   res(0) = fk(0,3); res(1) = fk(1,3); res(2) = fk(2,3);

   res(3) = v.x(); res(4) = v.y(); res(5) = v.z(); res(6) = v.w();

   return res;
}

void iiwa14::correctForLimits(Matrix<double,JOINT_NO,1>& q)
{
   for(int i = 0; i < JOINT_NO; ++i) {
      if(q(i) > iiwa14::qmax[i])
         q(i) = iiwa14::qmax[i];
      else if(q(i) < iiwa14::qmin[i])
         q(i) = iiwa14::qmin[i];
   }
}

void iiwa14::substituteTorqueNoise(Matrix<double,JOINT_NO,1>& t)
{
   for(int i = 0; i < JOINT_NO; ++i) {
      if(t(i) > iiwa14::noise[i])
         t(i) -= iiwa14::noise[i];
      else if(t(i) < -iiwa14::noise[i])
         t(i) += iiwa14::noise[i];
      else
         t(i) = 0;
   }
}
