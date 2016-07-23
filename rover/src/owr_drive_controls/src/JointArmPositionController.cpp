/*
 * Orignal Author: Simon Ireland
 * Editors:
 * Date Started: 23/07/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointArmPositionController.hpp"
#include <math.h>
#include <ros/ros.h>


JointArmPositionController :: JointArmPositionController(int minPositionIn, int maxPositionIn, int minValueIn, int maxValueIn, int minPWMIn, int maxPWMIn, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
        minPosition = minPositionIn;
        maxPosition = maxPositionIn;
        minValue = minValueIn;
        maxValue = maxValueIn;
        minPWM = minPWMIn;
        maxPWM = maxPWMIn;
}

// For driving a linear actuator which converts PWM to absolute position,
// we are linearly mapping the range of position readings to the range of PWM values.
// Thus, use the function Y = (X-A)/(B-A) * (D-C) + C
// where we are mapping X within A to B, to Y from C to D.
int JointArmPositionController :: posToPWM(int currentPos, int futurePos, double updateFrequency){
    double temp = futurePos - minPosition; // X - A
    temp = (temp / ( maxPosition - minPosition )); // (X-A)/(B-A)
    temp = temp * ( maxPWM - minPWM ) + minPWM; // (X-A)/(B-A) * (D-C) + C
    int result = temp;
    lastPos = currentPos;
    return temp;
}

jointInfo JointArmPositionController::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime){
    jointInfo info;
    ros::Duration dif = extrapolationTime - sessionStart;
    
    info.velocity = 0;
    info.position = lastPos + dif.toSec() * info.velocity; //TODO: can make this more accurate if m/s of actuator is known
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}
