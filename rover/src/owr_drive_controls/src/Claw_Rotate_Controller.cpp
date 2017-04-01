/*
 * Orignal Author: Harry J.E Day
 * Editors: Elliott Smith
 * Date Started: 27/02/17
 * Purpose: Used to drive the claw. This is very hacky, dosen't really do what it says
 * Copyright BLUEsat UNSW 2017. Released under the MIT License.
 */

#include "Claw_Rotate_Controller.hpp"
#include <math.h>
#include <ros/ros.h>

static float abs_cap(int input, int cap);

Claw_Rotate_Controller :: Claw_Rotate_Controller(int max_PWM, int min_PWM, int * trim, char * topic, ros::NodeHandle nh, std::string name)
        : JointController(topic,nh,name), trim(trim) {
        minPWM = max_PWM;
        maxPWM = min_PWM;
        lastPWM = min_PWM + ( (max_PWM - min_PWM)/2 ); // Initialise lastPWM to be stationary.
}

int Claw_Rotate_Controller :: velToPWM(int futureVel){
    if(stopN || stopP) {
        return minPWM+ ( (maxPWM- minPWM)/2 );
    }
    float pwm = ((maxPWM - minPWM)/2) * abs_cap(futureVel, 1);
    lastPWM = (minPWM+ ( (maxPWM- minPWM)/2 )) + pwm;
    return lastPWM;
}

int Claw_Rotate_Controller::velToPWM() {
    return velToPWM(requestedValue);
}

jointInfo Claw_Rotate_Controller::extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime){
    jointInfo info;
    ros::Duration dif = extrapolationTime - sessionStart;
    
    info.velocity = 0;
    info.position = lastPos; // TODO: make this more accurate by knowing the speed of the actuator at different PWM values.
    info.pwm = lastPWM;
    info.jointName = name;
    return info;
}

float abs_cap(int input, int cap) {
    return fmax(-1 * cap, fmin(cap, input));
}