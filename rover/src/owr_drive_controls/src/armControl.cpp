/*
 * Original Author: Elliott Smith
 * Editors:
 * Date Started: 2/6/18
 * Purpose: Convert arm joystick message into arm joint control
 *
 */

#include "armControl.hpp"
#include <ros/ros.h>

armJointVel convertTwistMessagesToJoints(const geometry_msgs::Twist * stick1, const geometry_msgs::Twist * stick2) {
  armJointVel output;
  if(fabs(sqrt(pow(stick1->linear.x, 2) + pow(stick1->linear.y, 2))) < VEL_ERROR) {
        output.armUpperActuator = 0;
  } else if (fabs(sqrt(pow(stick2->linear.x, 2) + pow(stick2->linear.y, 2))) < VEL_ERROR) {
	output.armLowerActuator = 0;
  } else {
      // insert maths model of arm here
  }
  return output;

}
