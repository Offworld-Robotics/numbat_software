/*
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * Purpose: Convert arm joystick message into arm joint control
 *
 */

#include "armControl.hpp"
#include <ros/ros.h>

armJointVel ArmControl::convertJoystickMessageToJoints(const sensor_msgs::Joy::ConstPtr& joy) {
  armJointVel output;
  output.armUpperActuator = joy->axes[ARM_STICK_UPPER_UD];
  output.armLowerActuator = joy->axes[ARM_STICK_LOWER_UD];
  output.armBaseRotate = joy->axes[DPAD_LR];
  if (joy->buttons[A_BUTTON]) {
      output.clawTwist = 1;
  } else if (joy->buttons[B_BUTTON]) {
      output.clawTwist = -1;
  }
  if (joy->buttons[LEFT_SHOULDER]) {
      clawGripAngle-=(180/PI);
  } else if (joy->buttons[RIGHT_SHOULDER]) {
      clawGripAngle+=(180/PI);
  }
  output.clawGrip = clawGripAngle;
  return output;

}
