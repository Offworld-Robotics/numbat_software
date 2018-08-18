/*
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * Purpose: Convert arm joystick message into arm joint control
 *
 */

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#define MAX_JOYSTICK_Y_AXIS 36787.0
#define DEADZONE 5000.0
#define ARM_STICK_UPPER_UD 1
#define ARM_STICK_LOWER_UD 3
#define DPAD_LR 6
#define A_BUTTON 0
#define B_BUTTON 1
#define LEFT_SHOULDER 4
#define RIGHT_SHOULDER 5

typedef struct _armJointVel {
  double armUpperActuator, armLowerActuator, armBaseRotate, clawGrip, clawTwist;
} armJointVel;

armJointVel convertJoystickMessageToJoints(const sensor_msgs::Joy::ConstPtr& joy);


#endif