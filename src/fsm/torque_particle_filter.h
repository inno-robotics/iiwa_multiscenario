#ifndef TORQUE_PARTICLE_FILTER_H
#define TORQUE_PARTICLE_FILTER_H

#include <eigen3/Eigen/Dense>
#include <vector>

#include "../project_settings.h"

struct TorqueParticle {
   double pose;
   double p;
   double alpha;    // z rotation
   double beta;     // y rotation
   
   TorqueParticle() : pose(0), p(1), alpha(0), beta(0) {}

   TorqueParticle& operator= (const TorqueParticle& other);
   void move();

}; //  TorqueParticle


class TorqueParticleFilter {
public:
   TorqueParticleFilter(int N=100);
   void initializeNewParticles();
   TorqueParticle nextCircle(Eigen::Matrix<double,JOINT_NO,1>& tau);
   TorqueParticle estimation();

   // temporary
   Eigen::Matrix<double,3,JOINT_NO> findJacobian(double len);
   void prepareForJacobian(Eigen::Matrix<double,JOINT_NO,1>& q);
   Eigen::Matrix<double,4,4> rotations[JOINT_NO];

private:
   int  getJointNo(double x);
   void normalization();
   void resampling();
   double lenRest(double x, int pos);

   std::vector<TorqueParticle> particles;
   Eigen::Vector3d positions[JOINT_NO];
   Eigen::Vector3d orientations[JOINT_NO];
   
   int particleNo;

}; // TorqueParticleFilter


#endif // TORQUE_PARTICLE_FILTER_H
