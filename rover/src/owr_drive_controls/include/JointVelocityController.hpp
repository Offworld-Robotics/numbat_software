/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in velocity and outputs a pwm rate.
 */
#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#include <std_msgs/Float64.h>

#include "JointController.hpp"


class JointVelocityController : public JointController {
    
    public:
        JointVelocityController(int minPWM, int maxPWM, int maxRPM, double wheelRadius, char * topic, ros::NodeHandle nh);
        int velToPWM(double targetVel, double currentVel, double currentAngel);
                                
    private:
        double wheelRadius;
        int  minPWM, maxPWM;
        int maxRPM;
        int currentPWM;
        double deltaPWM;
        double velocityRange;
        
        double lastAngularVelocity;
        int lastPWM;
    
};

#endif