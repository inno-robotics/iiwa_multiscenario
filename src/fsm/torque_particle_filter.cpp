#include <cstdlib>
#include <ctime>
#include <cmath>

#include "torque_particle_filter.h"
#include "../kinematics/homogenous.h"

#include <iostream>

#define PARTICLE_DISTANCE 20.0    // mm
#define PARTICLE_ANGLE    0.2     // rad

#define URAND ((rand() % 10000) / 10000.0)

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

TorqueParticle TorqueParticleFilter::nextCircle(Matrix<double,JOINT_NO,1>& tau)
{
   Vector3d vec;
   Matrix<double,4,4> prod;
   Matrix<double,JOINT_NO,1> torque = tau, found;
   double nTorque = torque.norm();
   if(nTorque > 0) torque /= nTorque;
   //std::cout << torque << std::endl << std::endl;

   // move and update weights 
   for(vector<TorqueParticle>::iterator it = particles.begin(); it != particles.end(); it++) { 

      it->move(); 

      int J = getJointNo(it->pose);
      if(J == -1) {
         it->p = 0;
      } else {
         prod = rotations[J] * homo::Rz(it->alpha) * homo::Ry(it->beta);    // force direction
	 found = findJacobian(it->pose).transpose() * prod.block(0,0, 3,1);
	 //std::cout << found << std::endl << std::endl;
	 double nFound = found.norm();
	 if(nFound > 0) found /= nFound;
         it->p = exp(-(found-torque).squaredNorm());
	 //std::cout << (found-torque).squaredNorm() << " " << it->p << std::endl;
      }
   }

   normalization();
   // change particle set
   resampling();

   // estimate result
   return estimation();

}

TorqueParticle TorqueParticleFilter::estimation()
{
   TorqueParticle tp;

   for(vector<TorqueParticle>::iterator it = particles.begin(); it != particles.end(); it++) { 
      tp.pose += it->p * it->pose;
      tp.alpha += it->p * it->alpha;
      tp.beta += it->p * it->beta;
   }

   return tp;
}

void TorqueParticleFilter::initializeNewParticles()
{
   int len = LINK01 + LINK23 + LINK45 + LINK67;
   for(vector<TorqueParticle>::iterator it = particles.begin(); it != particles.end(); it++) {
      it->pose = rand() % len;
      while(it->pose < LINK_0) { it->pose = rand() % len; } // exclude base
      it->p = 1.0/double(particles.size()); 

      it->alpha = URAND * 2 * PI;
      it->beta = (2*URAND - 1) * PI;

      //std::cout << it->pose << " " << it->alpha << " " << it->beta << " " << it->p << endl;
   }
}

void TorqueParticleFilter::prepareForJacobian(Matrix<double,JOINT_NO,1>& q)
{
   // joint 0
   rotations[0] *= homo::Rz(q(0));   
   // joint 1
   rotations[1] = rotations[0] * homo::Tz(LINK_1);
   positions[1] = rotations[1].block(0,3, 3,1);
   orientations[1] = rotations[1].block(0,1, 3,1);  // y
   rotations[1] *= homo::Ry(q(1));
   // joint 2
   rotations[2] = rotations[1] * homo::Tz(LINK_2);
   positions[2] = rotations[2].block(0,3, 3,1);
   orientations[2] = rotations[2].block(0,2, 3,1);  // z
   rotations[2] *= homo::Rz(q(2)+PI);
   // joint 3
   rotations[3] = rotations[2] * homo::Tz(LINK_3);
   positions[3] = rotations[3].block(0,3, 3,1);
   orientations[3] = rotations[3].block(0,1, 3,1);  // y
   rotations[3] *= homo::Ry(q(3));
   // joint 4
   rotations[4] = rotations[3] * homo::Tz(LINK_4);
   positions[4] = rotations[4].block(0,3, 3,1);
   orientations[4] = rotations[4].block(0,2, 3,1);  // z
   rotations[4] *= homo::Rz(q(4)-PI);
   // joint 5
   rotations[5] = rotations[4] * homo::Tz(LINK_5);
   positions[5] = rotations[5].block(0,3, 3,1);
   orientations[5] = rotations[5].block(0,1, 3,1);  // y
   rotations[5] *= homo::Ry(q(5));
   // joint 6
   rotations[6] = rotations[5] * homo::Tz(LINK_6);
   positions[6] = rotations[6].block(0,3, 3,1);
   orientations[6] = rotations[6].block(0,2, 3,1);  // z
   rotations[6] *= homo::Rz(q(6));
   
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
      pp[i] = particles[i];
      if(particles[i].p > max) max = particles[i].p;
   }
   // start
   int index = rand() % particles.size();
   double beta = 0;
   for(int k = 0; k < pp.size(); ++k) {
      beta += URAND * 2 * max;
      while(beta > pp[index].p) {
         beta -= pp[index].p;
	 index = (index+1) % pp.size();
      }
      // save
      particles[k] = pp[index];
   }
   normalization();
}

Matrix<double,3,JOINT_NO> TorqueParticleFilter::findJacobian(double len)
{
   Matrix<double,3,JOINT_NO> res = Matrix<double,3,JOINT_NO>::Zero();

   int J = getJointNo(len);
   if(J == -1) return res;
   len = lenRest(len, J);    // update length
   //std::cout << "J " << J << " len " << len  << std::endl;

   Vector3d point = Vector3d(len*rotations[J](0,2)+rotations[J](0,3), len*rotations[J](1,2)+rotations[J](1,3), len*rotations[J](2,2)+rotations[J](2,3)); // s
   Vector3d product;

   for(int i = 0; i < J+1; ++i) {
      product = orientations[i].cross(point-positions[i]);
      res(0,i) = product(0); res(1,i) = product(1); res(2,i) = product(2);
      //res(3,i) = orientations[i](0); res(4,i) = orientations[i](1); res(5,i) = orientations[i](2);
   }
   
   return res;
}

//
// TorqueParticle
//

TorqueParticle& TorqueParticle::operator= (const TorqueParticle& other) 
{
   if(this != &other) { 
      this->pose = other.pose; 
      this->p = other.p; 
      this->alpha = other.alpha;
      this->beta = other.beta;
   }
   return *this;
} 

void TorqueParticle::move()
{
   //std::cout << this->pose << " " << this->alpha << " " << this-> beta << std::endl;

   this->pose += (2*URAND - 1) * PARTICLE_DISTANCE;
   this->alpha += (2*URAND -1) * PARTICLE_ANGLE;
   this->beta += (2*URAND - 1) * PARTICLE_ANGLE;

   //std::cout << this->pose << " " << this->alpha << " " << this-> beta << std::endl << std::endl;
}
