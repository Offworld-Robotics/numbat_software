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

typedef struct _armJointVel {
  double armUpperActuator, armLowerActuator
} armJointVel;

armJointVel convertTwistMessagesToJoints(const geometry_msgs::Twist * stick1, const geometry_msgs::Twist * stick2);


#endif