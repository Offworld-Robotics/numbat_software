/*
 * Handles detection of obstacles by the lidar.
 */
#ifndef OJECT_AVOIDANCE_H
#define OJECT_AVOIDANCE_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <laser_filters/angular_bounds_filter.h>

using namespace std; 

class ObjectAvoidance {
    public:
        ObjectAvoidance();
        void run();
        
    private:
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& joy);

        ros::NodeHandle nh;

	    //ros::Subscriber sub;
        ros::Publisher pub;
        
        tf::TransformListener  listenerL;
        message_filters::Subscriber<sensor_msgs::LaserScan> sub;
        tf::MessageFilter<sensor_msgs::LaserScan> laserNotifierL;
        tf::TransformListener  listenerR;
        tf::MessageFilter<sensor_msgs::LaserScan> laserNotifierR;
        laser_filters::LaserScanAngularBoundsFilter angleFilter;
        
        
};

#endif
