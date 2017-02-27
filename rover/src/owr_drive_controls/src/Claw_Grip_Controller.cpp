/*
 * Orignal Author: Harry J.E Day
 * Date Started: 27/02/17
 * Purpose: Used to drive the claw. This is very hacky, dosen't really do what it says
 * Copyright BLUEsat UNSW 2017. Released under the MIT License.
 */

#include "Claw_Grip_Controller.hpp"
#include <math.h>
#include <ros/ros.h>

static float abs_cap(int input, int cap);

Claw_Grip_Controller :: Claw_Grip_Controller(int max_PWM, int min_PWM, char * topic, ros::NodeHandle nh, std::string name)
        : JointController(topic,nh,name) {
        minPWM = max_PWM;
        maxPWM = min_PWM;
        lastPWM = minPWM+ ( (maxPWM- minPWM)/2 ); // Initialise lastPWM to be stationary.
}

int Claw_Grip_Controller :: velToPWM(int futureVel){
    if(stopN || stopP) {
        return minPWM+ ( (maxPWM- minPWM)/2 );
    }
    lastPWM = futureVel;
    return lastPWM;
}

int Claw_Grip_Controller::velToPWM() {
    return velToPWM(requestedValue);
}


jointInfo Claw_Grip_Controller::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime){
    jointInfo info;
    ros::Duration dif = extrapolationTime - sessionStart;
    
    info.velocity = 0;
    info.position = lastPos; // TODO: make this more accurate by knowing the speed of the actuator at different PWM values.
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}

