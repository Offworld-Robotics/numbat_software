/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 16/02/2016
 * Purpose: Represents an interface for controlling a motor. 
 */

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define SECONDS_IN_MINUTE 60
#define FLOATING_PT_ERROR 0.001
#define FLOAT_EQL(x,y) ((fabs(x - y)) > FLOATING_PT_ERROR)

class JointController { 
    public:
        JointController(char * topic, ros::NodeHadle nh);
        void callback(const geometry_msgs::Float64::ConstPtr& requestedValue);
    protected:
        double requestedValue;
        ros::Subscriber sub;
    private:
        NodeHandle nh;
}

#endif