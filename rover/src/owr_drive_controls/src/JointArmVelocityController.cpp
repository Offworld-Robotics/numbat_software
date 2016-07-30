/*
 * Orignal Author: Simon Ireland
 * Editors:
 * Date Started: 23/07/2016
 * Purpose: Represents an interface for controlling an actuator. It takes in a target PWM outputs a pwm rate.
 * Will be expanded to be a position based controller.
 */

#include "JointArmVelocityController.hpp"
#include <math.h>
#include <ros/ros.h>


JointArmVelocityController :: JointArmVelocityController(double minADCIn, double maxADCIn, double minPos, double maxPos, int minPWMIn, int maxPWMIn, char * topic, ros::NodeHandle nh, std::string name) : JointController(topic,nh,name) {
        minPWM = minPWMIn;
        maxPWM = maxPWMIn;
        minADC = minADCIn;
        maxADC = maxADCIn;
        minPosition = minPos;
        maxPosition = maxPos;
        lastPWM = minPWMIn + ( (maxPWMIn - minPWMIn)/2 ); // Initialise lastPWM to be stationary.
}

int JointArmVelocityController :: velToPWM(int futureVel){
    lastPWM = futureVel;
    return futureVel;
}

// Y = (X-A)/(B-A) * (D-C) + C
void JointArmVelocityController :: updatePos(int positionIn){
    
    double temp = (positionIn - minADC)/(maxADC - minADC);
    
    temp = ( temp * (maxPosition - minPosition) ) + minPosition;
    
    if( (lastPos < temp) && stopP){
        // do nothing, we dont want to increase
    } else if( (lastPos > temp) && stopN){
        // do nothing, we dont want to increase
    } else {
        lastPos = temp;
    }
}

jointInfo JointArmVelocityController::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime){
    jointInfo info;
    ros::Duration dif = extrapolationTime - sessionStart;
    
    info.velocity = 0;
    info.position = lastPos; // TODO: make this more accurate by knowing the speed of the actuator at different PWM values.
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}
