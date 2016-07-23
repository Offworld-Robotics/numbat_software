/*
 * Orignal Author: Simon Ireland
 * Editors:
 * Date Started: 23/07/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointArmPositionController.hpp"
#include <math.h>
#include <ros/ros.h>


JointArmPositionController(int minPositionIn, int maxPositionIn, int minValueIn, int maxValueIn, int minPWMIn, int maxPWMIn, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
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
int posToPWM(int currentPos, int futurePos, double updateFrequency){
    double temp = futurePos - this.minPositionIn; // X - A
    temp = (temp / ( this.maxPosition - this.minPosition )); // (X-A)/(B-A)
    temp = temp * ( this.maxPWM - this.minPWM ) + minPWM; // (X-A)/(B-A) * (D-C) + C
    int result = temp;
    this.lastPos = currPos;
    return temp;
}

virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime){
    jointInfo info;
    info.velocity = 0;
    info.position = this.lastPos + (extrapolationTime - sessionStart) * info.velocity; //TODO: can make this more accurate if m/s of actuator is known
    info.pwm = this.lastPWM;
    info.jointName = name;
    return info;
}
