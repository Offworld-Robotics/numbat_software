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
  // the arm upper actuator is a number from -1 to 1 that corresponds to the y-position of the left joystick
  output.armUpperActuator = joy->axes[ARM_STICK_UPPER_UD];
  // the arm lower actuator is a number from -1 to 1 that corresponds to the y-position of the right joystick
  output.armLowerActuator = joy->axes[ARM_STICK_LOWER_UD];
  // the arm base rotation is a number -1 to 1 that corresponds to the left-right position of the DPAD
  output.armBaseRotate = joy->axes[DPAD_LR];
  // Pressing the A Button causes the claw to rotate (clockwise/counterclockwise)
  if (joy->buttons[A_BUTTON]) {
      output.clawTwist = 1;
  // Pressing the B button causes the claw to rotate (clockwise/counterclockwise)
  } else if (joy->buttons[B_BUTTON]) {
      output.clawTwist = -1;
  } else {
      output.clawTwist = 0;
  }

  // Pressing the left button causes the claw grip to open
  if (joy->buttons[LEFT_SHOULDER]) {
      clawGripAngle -= 1;
  // Pressing the right button causes the claw grip to close
  } else if (joy->buttons[RIGHT_SHOULDER]) {
      clawGripAngle += 1;
  }
  output.clawGrip = clawGripAngle;
  return output;

}
