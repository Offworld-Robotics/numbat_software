/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 11/03/2016
 * Purpose: Represents an interface for controlling the LIDAR tilt. 
 */
#ifndef LIDAR_TILT_JOINT_CONTROLLER_H
#define LIDAR_TILT_JOINT_CONTROLLER_H

#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#include "JointController.hpp"

enum LIDARTiltMode { CONTINUOUS = 0, STATIONARY = 1, POSITION = 2 };

class LIDARTiltJointController : public JointController {
    
    public:
        LIDARTiltJointController(LIDARTiltMode initialMode, char * topic, ros::NodeHandle nh, std::string name);
        
        int velToPWM(double targetValue, double hz);
        int velToPWM(double hz);
        void setModeCallback(const std_msgs::Int16 mode); 
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
                                
    private:
        LIDARTiltMode mode;
        ros::Subscriber modeSub;
        
        double lastAngularVelocity;
        int lastPWM;
        
        double angle;
        bool currentDirection;
        
        double lastHz;
};

#endif