/*
 * Date Started: 15/08/16
 * Original Author: Harry J.E Day
 * Editors:
 * ROS Node Name: utm_tf
 * ROS Package: owr_positioning
 * Purpose: Provides a GPS co-ordinate system to our map transform
 */

#ifndef GPS_TRANSFORM_H
#define GPS_TRANSFORM_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class GPSTransform {
    public:
        GPSTransform();
        void spin();
        void newPositionCallback(const sensor_msgs::NavSatFix::ConstPtr & msg);
        void newMagCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg);
    private:
        geometry_msgs::Vector3 convertToUTM(const sensor_msgs::NavSatFix & msg);
        ros::NodeHandle nh;
        ros::Subscriber newPosSub;
        tf2_ros::TransformBroadcaster tfBroadcast;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        geometry_msgs::TransformStamped currentTransform;

        

};

#endif