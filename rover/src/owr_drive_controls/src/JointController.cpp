/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Represents an interface for controlling a motor. 
 */

#include "JointController.hpp"
#include <limits>

JointController::JointController(char * topic, ros::NodeHandle nh, std::string jointName) :
    stopP(false), stopN(false) {
     sub = nh.subscribe<std_msgs::Float64>(topic,2, &JointController::callback, this);
     subStop = nh.subscribe<owr_messages::stop>("/owr/saftey_stop/" + jointName,2, &JointController::stopCallback, this);
     this->nh = nh;
     requestedValue = std::numeric_limits<double >::quiet_NaN();
     name = jointName;
}

void JointController::callback(const std_msgs::Float64::ConstPtr& msg) {
//     printf("recived %f\n", msg->data);
    requestedValue = msg->data;
}

void JointController::stopCallback ( const owr_messages::stop::ConstPtr& requestedValue ) {
    stopP = requestedValue->stopP;
    stopN = requestedValue->stopN;
}
