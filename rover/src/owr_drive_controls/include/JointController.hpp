/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Represents an interface for controlling a motor. 
 */

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H



#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "owr_messages/stop.h"

#define SECONDS_IN_MINUTE 60.0
#define FLOATING_PT_ERROR 0.001
#define FLOAT_EQL(x,y) ((fabs(x - y))< FLOATING_PT_ERROR)


typedef struct _jointInfo {
    int pwm;
    double effort, velocity, position, targetPos;
    std::string jointName;
} jointInfo;

class JointController { 
    public:
        JointController(char * topic, ros::NodeHandle nh, std::string name);
        void callback(const std_msgs::Float64::ConstPtr& requestedValue);
        void stopCallback(const owr_messages::stop::ConstPtr& requestedValue);
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime) = 0;
        std::string name;
    protected:
        double requestedValue;
        ros::Subscriber sub;
        ros::Subscriber subStop;
        bool stopP;
        bool stopN;
    private:
        ros::NodeHandle nh;
};

#endif
