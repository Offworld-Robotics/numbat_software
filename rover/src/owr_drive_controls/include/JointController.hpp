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

#define SECONDS_IN_MINUTE 60
#define FLOATING_PT_ERROR 0.001
#define FLOAT_EQL(x,y) ((fabs(x - y)) > FLOATING_PT_ERROR)


typedef struct _jointInfo {
    int pwm;
    double effort, velocity, position;
    std::string jointName;
} jointInfo;

class JointController { 
    public:
        JointController(char * topic, ros::NodeHandle nh);
        void callback(const std_msgs::Float64::ConstPtr& requestedValue);
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, long ros::Time extrapolationTime) = 0;
    protected:
        double requestedValue;
        ros::Subscriber sub;
    private:
        ros::NodeHandle nh;
};

#endif