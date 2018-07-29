/*
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * Purpose: Convert arm joystick message into arm joint control
 *
 */

#include <geometry_msgs/Twist.h>

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#define VEL_ERROR 0.01

typedef struct _armJointVel {
  double armUpperActuator, armLowerActuator, armBaseRotate, clawGrip, clawTwist;
} armJointVel;

armJointVel convertTwistMessagesToJoints(const geometry_msgs::Twist * stick);


#endif