/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Represents an interface for controlling a motor. 
 */

#include "JointController.hpp"

JointController::JointController(char * topic, ros::NodeHadle nh) {
     sub = nh.subscribe<sensor_msgs::Joy>("/owr/joysticks",2, &JointController::callback, this);
     this->nh = nh;
}