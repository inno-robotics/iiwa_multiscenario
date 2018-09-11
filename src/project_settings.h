#ifndef PROJECT_SETTINGS_H
#define PROJECT_SETTINGS_H

//
//	KUKA PARAMETERS
//

// number of coordinates
#define JOINT_NO 7         // joint space
#define CART_NO  6         // cartesian space
#define POSE_QUATERN 7     // ROS parameters for joint space

// link length
#define LINK_0 200         // base
#define LINK_1 160
#define LINK_2 200
#define LINK_3 220
#define LINK_4 200
#define LINK_5 200
#define LINK_6 80
#define LINK_7 46
//
#define LINK01 360         // LINK_0 + LINK_1
#define LINK23 420         // LINK_2 + LINK_3
#define LINK45 400         // LINK_4 + LINK_5
#define LINK67 126         // LINK_6 + LINK_7

// gap between real and program limits
#define GAP  5             // position
#define VGAP 3             // velocity

// speed of shape restoring after manual "deformation"
#define STATE_CORRECTION_COEFF 0.1

//
//	REACTIONS
//

// square of position of error
#define CART_SQ_ERR   10            // in cartesian space
#define JOINT_SQ_ERR  0.0001        // in joint space

// elbow reaction
#define ELBOW_REACTION_SEC 1        // duration

// compliance reaction
#define COMPLIANT_REACTION_SEC 0.5  // duration
#define THRESHOLD_COMP_TORQUE  8    // saturation limit
#define COMPL_MARGIN           0.3  // distance from limits for restoring

//
//	DETECTION
//

// torque generated in simulation
#define VIRTUAL_OBSTACLE_FORCE 0.8

//
//	OTHER
//

#define PI   3.14159265358979323846
#define SEP  ","          // CSV separator


#endif // PROJECT_SETTINGS_H