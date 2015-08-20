/*
 * Handles detection of obstacles by the lidar.
 */
#ifndef OJECT_AVOIDANCE_H
#define OJECT_AVOIDANCE_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

using namespace std; 

class ObjectAvoidance {
    public:
        ObjectAvoidance();
        void run();
        
    private:
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& joy);

        ros::NodeHandle nh;

	    ros::Subscriber sub;
        ros::Publisher pub;
        
};

#endif
