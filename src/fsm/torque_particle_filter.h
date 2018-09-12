#ifndef TORQUE_PARTICLE_FILTER_H
#define TORQUE_PARTICLE_FILTER_H

#include <eigen3/Eigen/Dense>
#include <vector>

#include "../project_settings.h"

struct TorqueParticle {
   double pose;
   double p;
   
   TorqueParticle() : pose(0), p(1) {}
   TorqueParticle& operator= (const TorqueParticle& other) {
      if(this != &other) { 
          this->pose = other.pose; 
	  this->p = other.p; 
      }
      return *this;
   }
}; //  TorqueParticle


class TorqueParticleFilter {
public:
   TorqueParticleFilter(int N=100);

   // temporary
   Eigen::Matrix<double,CART_NO,JOINT_NO> findJacobian(double len);
   void prepareForJacobian(Eigen::Matrix<double,JOINT_NO,1>& q);

private:
   int  getJointNo(double x);
   void initializeNewParticles();
   void normalization();
   void resampling();
   double lenRest(double x, int pos);

   std::vector<TorqueParticle> particles;
   Eigen::Matrix<double,4,4> rotations[JOINT_NO];
   Eigen::Vector3d positions[JOINT_NO];
   Eigen::Vector3d orientations[JOINT_NO];
   
   int particleNo;

}; // TorqueParticleFilter


#endif // TORQUE_PARTICLE_FILTER_H
