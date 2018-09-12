#include <cstdlib>
#include <ctime>

#include "torque_particle_filter.h"
#include "../kinematics/homogenous.h"


using namespace Eigen;
using namespace std;

TorqueParticleFilter::TorqueParticleFilter(int N)	
{
   srand(time(NULL));

   particles.resize(N);
   
   rotations[0] = homo::Tz(LINK_0);
   positions[0]    = Vector3d(0,0,LINK_0);
   orientations[0] = Vector3d(0,0,1);   
}

void TorqueParticleFilter::initializeNewParticles()
{
   int len = LINK01 + LINK23 + LINK45 + LINK67;
   for(vector<TorqueParticle>::iterator it = particles.begin(); it != particles.end(); it++) {
      it->pose = rand() % len;
      while(it->pose < LINK_0) { it->pose = rand() % len; } // exclude base
      it->p = 1.0/double(particles.size()); 
   }
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

int TorqueParticleFilter::getJointNo(double x)
{
   if(x < (LINK01+LINK23)) {
      if(x < LINK01) {
         return x < LINK_0 ? -1 : 0;
      } else {
         return x < (LINK01+LINK_2) ? 1 : 2;
      }
   } else {
      if(x < (LINK01+LINK23+LINK45)) {
         return x < (LINK01+LINK23+LINK_4) ? 3 : 4;
      } else {
         return x < (LINK01+LINK23+LINK45+LINK_6) ? 5 : 6;
      }
   }
   return -1;
}

double TorqueParticleFilter::lenRest(double x, int pos)
{
   switch(pos) {
   case 0: return x - LINK_0;
   case 1: return x - LINK01;
   case 2: return x - LINK01 - LINK_2;
   case 3: return x - LINK01 - LINK23;
   case 4: return x - LINK01 - LINK23 - LINK_4;
   case 5: return x - LINK01 - LINK23 - LINK45;
   case 6: return x - LINK01 - LINK23 - LINK45 - LINK_6;
   }
   return x;
}

void TorqueParticleFilter::normalization()
{
   // sum of elements
   double sum = 0;
   for(vector<TorqueParticle>::iterator it = particles.begin(); it != particles.end(); it++) {
      sum += it->p;
   }

   for(vector<TorqueParticle>::iterator it = particles.begin(); it != particles.end(); it++) {
      it->p /= sum;
   }
}

void TorqueParticleFilter::resampling()
{
   // copy of particles
   vector<TorqueParticle> pp;
   pp.resize(particles.size());
   double max = 0; // probability
   for(int i = 0; i < particles.size(); ++i) {
      // copy
      pp[i].pose = particles[i].pose;
      pp[i].p    = particles[i].p;
      if(particles[i].p > max) max = particles[i].p;
   }
   // start
   int index = rand() % particles.size();
   double beta = 0;
   for(int k = 0; k < pp.size(); ++k) {
      beta += (rand() % 1000) / 1000.0 * 2 * max;
      while(beta > pp[index].p) {
         beta -= pp[index].p;
	 index = (index+1) % pp.size();
      }
      // save
      particles[k].pose = pp[index].pose;
      particles[k].p    = pp[index].p;
   }
}

Matrix<double,CART_NO,JOINT_NO> TorqueParticleFilter::findJacobian(double len)
{
   Matrix<double,CART_NO,JOINT_NO> res = Matrix<double,CART_NO,JOINT_NO>::Zero();

   int J = getJointNo(len);
   if(J == -1) return res;
   len = lenRest(len, J);    // update length

   Vector3d point = Vector3d(len*rotations[J](0,2)+rotations[J](0,3), len*rotations[J](1,2)+rotations[J](1,3), len*rotations[J](2,2)+rotations[J](2,3));
   Vector3d product;

   for(int i = 0; i < J+1; ++i) {
      product = orientations[i].cross(point-positions[i]);
      res(0,i) = product(0); res(1,i) = product(1); res(2,i) = product(2);
      res(3,i) = orientations[i](0); res(4,i) = orientations[i](1); res(5,i) = orientations[i](2);
   }
   
   return res;
}
