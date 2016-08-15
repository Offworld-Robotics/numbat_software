/*
 * Date Started: 15/08/16
 * Original Author: Harry J.E Day
 * Editors:
 * ROS Node Name: utm_tf
 * ROS Package: owr_positioning
 * Purpose: Provides a UTM co-ordinate system to our map transform
 */

#ifndef UTM_TRANSFORM_H
#define UTM_TRANSFORM_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class UTMTransform {
    public:
        UTMTransform();
        void spin();
        void newPositionCallback(const geometry_msgs::Pose::ConstPtr& msg);
    private:
        ros::NodeHandle nh;
        ros::Subscriber newPosSub;
        tf2_ros::TransformBroadcaster tfBroadcast;
        geometry_msgs::TransformStamped currentTransform;

        

};

#endif