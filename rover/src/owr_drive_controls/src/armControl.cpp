/*
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * Purpose: Convert arm joystick message into arm joint control
 *
 */

#include "armControl.hpp"
#include <ros/ros.h>

armJointVel convertJoystickMessageToJoints(const sensor_msgs::Joy::ConstPtr& joy) {
  armJointVel output;
  if(fabs(joy->axes[ARM_STICK_UPPER_UD])<DEADZONE) {
        output.armUpperActuator = 0;
  } else {
    if (joy->axes[ARM_STICK_UPPER_UD] > 0) {
      output.armUpperActuator = (joy->axes[ARM_STICK_UPPER_UD]-DEADZONE)/MAX_JOYSTICK_Y_AXIS;
    } else {
      output.armUpperActuator = (joy->axes[ARM_STICK_UPPER_UD]+DEADZONE)/MAX_JOYSTICK_Y_AXIS;
    }
  }
  if (fabs(joy->axes[ARM_STICK_LOWER_UD])<DEADZONE) {
      output.armLowerActuator = 0;
  } else {
    if (joy->axes[ARM_STICK_LOWER_UD] > 0) {
	output.armLowerActuator = (joy->axes[ARM_STICK_LOWER_UD]-DEADZONE)/MAX_JOYSTICK_Y_AXIS;
    } else {
	output.armLowerActuator = (joy->axes[ARM_STICK_LOWER_UD]+DEADZONE)/MAX_JOYSTICK_Y_AXIS;
    }
  }
  output.armBaseRotate = 0;
  output.clawGrip = 0;
  output.clawTwist = 0;
  return output;

}
