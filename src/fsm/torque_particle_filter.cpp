#include "torque_particle_filter.h"
#include "../kinematics/homogenous.h"

using namespace Eigen;

TorqueParticleFilter::TorqueParticleFilter(int N)	
{
   particleNo = N;
   // initialize
   particles.resize(N);
   for(int i = 0; i < N; ++i) { particles[i].p = 1.0/double(N); }
   
   rotations[0] = homo::Tz(LINK_0);
   positions[0]    = Vector3d(0,0,LINK_0);
   orientations[0] = Vector3d(0,0,1);   
}

void TorqueParticleFilter::prepareForJacobian(Matrix<double,JOINT_NO,1>& q)
{
   // joint 0
   rotations[0] *= homo::Rz(q(0));   
   // joint 1
   rotations[1] = rotations[0] * homo::Tz(LINK_1);
   positions[1] = rotations[0].block(0,3,3,4);
   orientations[1] = rotations[0].block(0,1,3,2);  // y
   rotations[1] *= homo::Ry(q(1));
   // joint 2
   rotations[2] = rotations[1] * homo::Tz(LINK_2);
   positions[2] = rotations[1].block(0,3,3,4);
   orientations[2] = rotations[1].block(0,2,3,3);  // z
   rotations[2] *= homo::Ry(q(2));
   // joint 3
   rotations[3] = rotations[2] * homo::Tz(LINK_3);
   positions[3] = rotations[2].block(0,3,3,4);
   orientations[3] = rotations[2].block(0,1,3,2);  // y
   rotations[3] *= homo::Ry(q(3));
   // joint 4
   rotations[4] = rotations[3] * homo::Tz(LINK_4);
   positions[4] = rotations[3].block(0,3,3,4);
   orientations[4] = rotations[3].block(0,2,3,3);  // z
   rotations[4] *= homo::Ry(q(4));
   // joint 5
   rotations[5] = rotations[4] * homo::Tz(LINK_5);
   positions[5] = rotations[4].block(0,3,3,4);
   orientations[5] = rotations[4].block(0,1,3,2);  // y
   rotations[5] *= homo::Ry(q(4));
   // joint 6
   rotations[6] = rotations[5] * homo::Tz(LINK_6);
   positions[6] = rotations[5].block(0,3,3,4);
   orientations[6] = rotations[5].block(0,2,3,3);  // z
   rotations[6] *= homo::Ry(q(7));
   
}
