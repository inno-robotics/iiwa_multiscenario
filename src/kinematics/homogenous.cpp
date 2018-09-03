#include "homogenous.h"

#include <cmath>

using namespace Eigen;

Matrix4d homo::Rx(double q) {
   double c = cos(q), s = sin(q);
   Matrix4d m;
   m << 1, 0, 0, 0,
        0, c,-s, 0,
        0, s, c, 0,
        0, 0, 0, 1;
   return m;
}

Matrix4d homo::Ry(double q) {
   double c = cos(q), s = sin(q);
   Matrix4d m;
   m << c, 0, s, 0,
        0, 1, 0, 0,
       -s, 0, c, 0,
        0, 0, 0, 1;
   return m;
}

Matrix4d homo::Rz(double q) {
   double c = cos(q), s= sin(q);
   Matrix4d m;
   m << c,-s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
   return m;
}

//
// TRANSLATION
//

Matrix4d homo::Tx(double d) {
   Matrix4d m = Matrix4d::Identity();
   m(0,3) = d;
   return m;
}

Matrix4d homo::Ty(double d) {
   Matrix4d m = Matrix4d::Identity();
   m(1,3) = d;
   return m;
}

Matrix4d homo::Tz(double d) {
   Matrix4d m = Matrix4d::Identity();
   m(2,3) = d;
   return m;
}

//
// ROTATION (DERIVATIVE)
//

Matrix4d homo::dRx(double q) {
   double c = cos(q), s = sin(q);
   Matrix4d m;
   m << 0, 0, 0, 0,
        0,-s,-c, 0,
        0, c,-s, 0,
        0, 0, 0, 0;
   return m;
}

Matrix4d homo::dRy(double q) {
   double c = cos(q), s = sin(q);
   Matrix4d m;
   m <<-s, 0, c, 0,
        0, 0, 0, 0,
       -c, 0,-s, 0,
        0, 0, 0, 0;
   return m;
}

Matrix4d dRz(double q) {
   double c = cos(q), s = sin(q);
   Matrix4d m;
   m <<-s,-c, 0, 0,
        c,-s, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
   return m;
}

//
// TRANSLATION (DERIVATIVE)
//

Matrix4d homo::dTx(double d) {
   Matrix4d m = Matrix4d::Zero();
   m(0,3) = 1;
   return m;
}

Matrix4d homo::dTy(double d) {
   Matrix4d m = Matrix4d::Zero();
   m(1,3) = 1;
   return m;
}

Matrix4d homo::dTz(double d) {
   Matrix4d m = Matrix4d::Zero();
   m(2,3) = 1;
   return m;
}

//
// EULER ANGLES
//

Vector3d homo::EulerZYZ(const Matrix4d& m, int sign) {
   double phi;      // Z
   double theta;    // Y
   double psy;      // Z
   double c = m(2,2);
   if (c > -1 && c < 1) {
      double s = sqrt(1-c*c);
      if(sign > 0) {
         phi   = atan2(m(0,2),m(1,2)); 
         theta = atan2(c,s);
         psy   = atan2(-m(2,0),m(2,1)); }     
      else {
         phi   = atan2(-m(0,2),-m(1,2)); 
         theta = atan2(c,-s);
         psy   = atan2(m(2,0),-m(2,1)); }
   } 
   else if(c == 1) {
      phi   = atan2(m(0,0),m(1,0));   // theta + psy
      theta = 0;
      psy   = 0; }
   else {
      phi   = atan2(-m(0,0),-m(0,1)); // theta-psy
      theta = PI;
      psy   = 0;
   }
   return Vector3d(phi,theta,psy);  
}

Vector3d homo::EulerZYX(const Matrix4d& m, int sign) {
   double phi;    // X 
   double theta;  // Y
   double psy;    // Z
   double s = -m(2,0);
   if(s > -1 && s < 1) {
      double c = sqrt(1-s*s);
      if(sign > 0) {
         phi   = atan2(m(2,1),m(2,2));
         theta = atan2(s,c);
         psy   = atan2(m(1,0),m(0,0)); }
      else {
         phi   = atan2(-m(2,1),-m(2,2));
         theta = atan2(s,-c);
         psy   = atan2(-m(1,0),-m(0,0)); }      
   }
   else if(s == 1) {
      phi   = atan2(m(0,1),m(1,1));
      theta = PI/2;
      psy   = 0; } 
   else {
      phi   = atan2(-m(0,1),m(1,1));
      theta = -PI/2;
      psy   = 0;
   }
   return Vector3d(phi,theta,psy);
}

Quaterniond homo::fromZYX(double xx, double yy, double zz)
{
   Quaterniond q = AngleAxisd(zz, Vector3d::UnitZ()) * AngleAxisd(yy, Vector3d::UnitY()) * AngleAxisd(xx, Vector3d::UnitX());
   return q;
}

//
// DENAVIT-HARTERBERG PARAMETERS
//

Matrix4d homo::DH(double a, double alpha, double d, double theta) {
   double sa = sin(alpha), ca = cos(alpha), st = sin(theta), ct = cos(theta);
   Matrix4d m;
   m << ct,-st*ca, st*sa, a*ct,
        st, ct*ca,-ct*sa, a*st,
	 0,    sa,    ca,    d,
	 0,     0,     0,    1;
   return m;
}

//
// PSEUDO-INVERSE MATRIX
//

MatrixXd homo::pinv(const MatrixXd& src) {
   int m = src.rows(), n = src.cols();
   bool transp = false;
   MatrixXd A;
   MatrixXd tSrc = src.transpose();
   if(m < n) {
      transp = true;
      A = src * tSrc;
      n = m;
   }
   else {
      A = tSrc * src;
   }
   // tolerance
   double tol = 1e100, v = 0;
   for(int i = 0; i < A.rows(); ++i) {
      v = A(i,i);
      tol = fmin(tol, v > 0 ? v : 1e100);
   }
   tol *= 1e-9;

   MatrixXd L = MatrixXd::Zero(A.rows(), A.cols());
   int r = 0;
   for(int k = 0; k < n; ++k) {
      r++;
      MatrixXd B = A.block(k,k, A.rows()-k,1);
      if(r > 1) {
         MatrixXd tmp = L.block(k,0, L.rows()-k,r-1) * L.block(k,0, 1,r-1).transpose();
	 B -= tmp;
      }
      L.block(k,r-1, L.rows()-k,1) = B;
      double lkr = L(k,r-1);
      if(lkr > tol) {
         lkr = sqrt(lkr);
         L(k,r-1) = lkr;
	 if(k < n) 
	    L.block(k+1,r-1, n-k-1,1) *= 1/lkr;
      }
      else
         r --;
   }
   MatrixXd LL = L.block(0,0, L.rows(), r);
   MatrixXd tLL = LL.transpose();
   MatrixXd M = (tLL * LL).inverse();
   if(transp)
      return tSrc*LL*M*M*tLL;
   
   return LL*M*M*tLL*tSrc;
}
