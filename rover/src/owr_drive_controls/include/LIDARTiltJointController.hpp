/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 11/03/2016
 * Purpose: Represents an interface for controlling the LIDAR tilt. 
 */
#ifndef LIDAR_TILT_JOINT_CONTROLLER_H
#define LIDAR_TILT_JOINT_CONTROLLER_H

#include <std_msgs/Float64.h>

#include "JointController.hpp"

enum LIDARTiltMode { CONTINUOUS, STATIONARY, POSITION };

class LIDARTiltJointController : public JointController {
    
    public:
        LIDARTiltJointController(LIDARTiltMode mode, char * topic, ros::NodeHandle nh, std::string name);
        
        int velToPWM(double targetValue);
        int velToPWM();
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
                                
    private:
        LIDARTiltMode mode;
        
        double lastAngularVelocity;
        int lastPWM;
        
        double angle;
        bool currentDirection;
        
    
};

#endif