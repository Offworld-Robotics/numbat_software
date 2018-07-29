/*
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * Purpose: Convert arm joystick message into arm joint control
 *
 */

#include "armControl.hpp"
#include <ros/ros.h>

armJointVel convertTwistMessagesToJoints(const geometry_msgs::Twist * stick) {
  armJointVel output;
  if(fabs(sqrt(pow(stick->linear.x, 2) + pow(stick->linear.y, 2))) < VEL_ERROR) {
        output.armUpperActuator = 0;
	output.armLowerActuator = 0;
  } 
  // insert maths model of the arm here
  output.armUpperActuator = 0;
  output.armLowerActuator = 0;
  output.armBaseRotate = 0;
  output.clawGrip = 0;
  output.clawTwist = 0;
  return output;

}
