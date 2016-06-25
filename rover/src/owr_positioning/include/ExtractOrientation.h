/*
 * Date Started: 13/06/16
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS Node Name: orientation
 * ROS Package: owr_positioning
 * Purpose: Extracts the orientation of the rover from the imu message
 */


#ifndef DANCE_NODE_H
#define DANCE_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>



class ExtractOrientation {
    public:
        ExtractOrientation();
        void spin();
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    private:
        ros::NodeHandle nh;
        ros::Publisher quatMsgPub; 
        ros::Subscriber odometrySub;
        

};

#endif