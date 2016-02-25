/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Represents an interface for controlling a motor. 
 */

#include "JointController.hpp"
#include <limits>

JointController::JointController(char * topic, ros::NodeHandle nh, std::string jointName) {
     sub = nh.subscribe<std_msgs::Float64>(topic,2, &JointController::callback, this);
     this->nh = nh;
     requestedValue = std::numeric_limits<double >::quiet_NaN();
     name = jointName;
}

void JointController::callback(const std_msgs::Float64::ConstPtr& msg) {
    requestedValue = msg->data;
}