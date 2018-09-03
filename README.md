# iiwa_multiscenario
Framework for experiments with kuka iiwa collision reactions. The program provides an interface for development of simple robot reactions and combining them in form of finite state machine. 

Each reaction should be inherited from the _RobotStrategie_ class. It also should redefine the follosing methods:
* _execute(ControlInterface& )_ - robot movement during the reaction
* _isFinish()_ - check if the state can be changed
* _reset()_ - change internal object parameters if need
Information about robot parameters, such as value of external force, joint angles, cartesian position etc. can be found in structure _RobotState_. 

Transitions between states are based on combinations of events (defined in file _src/fsm/collision_reactions.h_). There are following options:
* _EV_COMPLETE | EV_EE_COLLISION_ - transition have place when all events are presented simultaneously
* _EV_EE_COLLISION | EV_OR | EV_ELBOW_COLLISION_ - each of events lead to transition
* _EV_OUT_OF_LIMITS | EV_KEY_ - event is critical and doesn't depend on existance of another events

The program works as a part of [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack) and uses Eigen library for matrix operations. It was tested in XUbuntu 14.04 with ROS Indigo and XUbuntu 16.04 with ROS Kinetic. 

As an example you can see _src/basic_reactions.cpp_.
