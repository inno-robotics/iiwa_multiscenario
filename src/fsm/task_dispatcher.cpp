#include "task_dispatcher.h"
#include "../kuka/iiwa_kinematics.h"
#include <ros/console.h>
#include "../debug_settings.h"

#include <iostream>

int TaskDispatcher::updateEvents()
{
   int events = EV_NO;

   // check collision
   detector.analyze(rs, collision);
   if(collision.found) {
       if(collision.jointNo < 5)
           events |= EV_ELBOW_COLLISION;
       else
           events |= EV_EE_COLLISION;
      //std::cout << "Collision" << std::endl;
      //std::cout << rs.externalTorque << std::endl;
   }

   // check joint limits
   for(int i = 0; i < JOINT_NO; ++i) {
       if(rs.jointPosition(i) > iiwa14::qmax[i] || rs.jointPosition(i) < iiwa14::qmin[i]) {
           //std::cout << rs.jointPosition << std::endl;
           events |= EV_OUT_OF_LIMITS;
       }
   }

   // share result
   rs.events = events;

   // check completeness
   if(kernel && kernel->isFinish()) {
       events |= EV_COMPLETE;
   }

   // share updated information (?)
   rs.events = events;
   
#ifdef SHOW_EVENTS
   ROS_INFO("Event: %d", events);
#endif

   return events;
}

bool TaskDispatcher::execute()
{
   if(!kernel) {
       // error
       return false;
   }

   // change state if need
   RobotStrategie* newStrategie = kernel->getTransition(updateEvents());
   if(newStrategie) {
      kernel = newStrategie;
      kernel->reset();
#ifdef SHOW_STATES
      ROS_INFO("New state: %s", kernel->getName().c_str());
#endif
   }

   // do work
   return kernel->execute(ci);
}

