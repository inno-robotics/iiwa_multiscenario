/* Based on "Cotion control of redundant robots under joint constraints: saturation in the null space" *
 * by Fabrizio Flacco, Alessandro De Luca and Oussama Khatib.                                          */
#ifndef SATURATIOJ_IJ_JULL_SPACE
#define SATURATIOJ_IJ_JULL_SPACE

#include "homogenous.h"

#include <iostream>

// J - number of joints
// C - number of coordinates in task space
template <int J, int C>
class SNSCalculator {
public:
   SNSCalculator(double* qMax, double* qMin, double* vMax);
   
   Eigen::Matrix<double,J,1> jointVelocity(const Eigen::Matrix<double,C,1>& cartesianVelocity, const Eigen::Matrix<double,J,1> nsJointVelocity, 
                                           const Eigen::Matrix<double,J,1>& jointAngles, const Eigen::Matrix<double,C,J>& jacobian, double period);  
                                    
   bool successfull() { return success; } 

   EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
private:
   int mostCriticalJoint(const Eigen::Matrix<double,J,1>& dq);
   bool isExceeded(const Eigen::Matrix<double,J,1>& v);
   double scaling(const Eigen::Matrix<double,C,1>& cartesianVelocity);  
    
   // robot limits
   double* lstQMax;
   double* lstQMin;
   double* lstVMax;
   bool    success;
   // temporary variables
   double vMax[J], vMin[J];   
   Eigen::Matrix<double,J,J> W, W_star;         // ident matrix, joint enable/disable
   Eigen::Matrix<double,J,1> dq_n, dq_star;     // joint velocity
   Eigen::Matrix<double,J,C> inv_jw;            // inverted value of jacobyan*W
   Eigen::Matrix<double,C,J> jw;                // product of jacobian*W
   Eigen::Matrix<double,C,1> dx;                // product of jacobian*jaointVelocity   
   
}; // SJSCalculator


template <int J, int C> 
SNSCalculator<J,C>::SNSCalculator(double* qMax, double* qMin, double* vMax) 
{
   lstQMax = qMax; lstQMin = qMin; lstVMax = vMax; 
   success = false;
}

template <int J, int C>
Eigen::Matrix<double,J,1> SNSCalculator<J,C>::jointVelocity(const Eigen::Matrix<double,C,1>& cartesianVelocity, const Eigen::Matrix<double,J,1> nsJointVelocity, 
                                                            const Eigen::Matrix<double,J,1>& jointAngles, const Eigen::Matrix<double,C,J>& jacobian, double period) 
{   
   success = true;
   // define current bounds
   double f = 1/period;
   for(int i = 0; i < J; ++i) {
      vMax[i] = fmin((lstQMax[i]-jointAngles(i))*f, lstVMax[i]);
      vMin[i] = fmax((lstQMin[i]-jointAngles(i))*f,-lstVMax[i]);
   }
   // variables
   Eigen::Matrix<double,J,1> res;
   dq_n = nsJointVelocity;
   dq_star = dq_n;
   W = Eigen::Matrix<double,J,J>::Identity();
   W_star = W;
   jw = jacobian;
   double s = 1, s_star = 0;      
      
   bool limitExceeded;
   do {
      limitExceeded = false;
      // find velocity
      inv_jw = homo::pinv(jw);
      dx = jacobian*dq_n;
      res = dq_n + inv_jw*(s*cartesianVelocity-dx);
      
      // check fisibility      
      if(limitExceeded = isExceeded(res)) {         
         double factor = scaling(cartesianVelocity);   
         if (factor > s_star) {
            s_star  = factor;
            W_star  = W;
            dq_star = dq_n;
         }
         // saturation
         int j = mostCriticalJoint(res);         
         W(j,j) = 0;
         dq_n(j) = (res(j) > vMax[j]) ? vMax[j] : vMin[j];
         jw = jacobian*W;
         
         // check rank
         Eigen::FullPivLU <Eigen::Matrix<double,C,J> > lu(jw);         
         if(lu.rank() < C) {            
            res = dq_star + homo::pinv(jacobian*W_star)*(s_star*cartesianVelocity-jacobian*dq_star);
            limitExceeded = false;
            success = false;
         }         
      }
   } while (limitExceeded);
   //std::cout << res << std::endl;
   //std::cout << (jacobian * res) << std::endl;
   return res;
}

template <int J, int C>
bool SNSCalculator<J,C>::isExceeded(const Eigen::Matrix<double,J,1>& v)
{
   for(int i = 0; i < J; ++i) {
      if(v(i) > vMax[i] || v(i) < vMin[i]) return true;
   }
   return false;
}

template <int J, int C>
double SNSCalculator<J,C>::scaling(const Eigen::Matrix<double,C,1>& cartesianVelocity)
{
   Eigen::Matrix<double,J,1> a = inv_jw * cartesianVelocity;
   Eigen::Matrix<double,J,1> b = dq_n - inv_jw * dx;
   Eigen::Matrix<double,J,1> maxS, minS;   
   // get bounds
   for(int i = 0; i < J; ++i) {
      minS(i) = (vMin[i] - b(i)) / a(i);
      maxS(i) = (vMax[i] - b(i)) / a(i);
      if(maxS(i) < minS(i)) {
         // swap
         double tmp = maxS(i);
	 maxS(i) = minS(i);
	 minS(i) = tmp;
      }
   }
   // minimax
   double smax = maxS.minCoeff(), smin = minS.maxCoeff();
   if (smin > smax || smax < 0 || smin > 1) 
      return 0;
   return fmin(smax, 1);
}

template <int J, int C>
int SNSCalculator<J,C>::mostCriticalJoint(const Eigen::Matrix<double,J,1>& dq)
{
   int j = 0;
   double diff_max = 0, diff;
   for(int i = 0; i < J; ++i) {
      diff = dq(i) - vMax[i];
      if(diff > diff_max) { diff_max = diff; j = i; }
      diff = vMin[i] - dq(i);
      if(diff > diff_max) { diff_max = diff; j = i; }
   }
   return j;
}

#endif // SATURATIOJ_IJ_JULL_SPACE
