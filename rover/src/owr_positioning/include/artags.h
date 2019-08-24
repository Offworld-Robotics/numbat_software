/*
 * Date Started:
 * Original Author: Nikola Medimurac
 * Editors: 
 * ROS Node Name: arTag_localization
 * ROS Package: owr_positioning
 * Purpose: Creates a transform for the camera location in the world frame
 * using ar tags observed
 */

#ifndef ARTAG_LOCALIZATION_H
#define ARTAG_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#include <math.h>
#include <stdio.h>

#define THRESHOLD 0.0001

class artag_localization {
    private:
    //stuff to setup the node	
    ros::Publisher pub;
    ros::Subscriber sub;
	ros::NodeHandle nh;

	//stuff to use transforms
    tf2_ros::TransformBroadcaster tfBroadcaster;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    //Odometry message, needed for ekf
    nav_msgs::Odometry odomMsg;

    //transform messages
    //geometry_msgs::TransformStamped arTransform;    //tranform of ar tag in world frame (for the actual location of the tag)
    //geometry_msgs::TransformStamped relativeTransform; //tranform of ar tag relative to rover (for the location of the tag from what the rover sees)
    //geometry_msgs::TransformStamped roverTransform; dont think i need this one anymore

    //callback function
    void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    
    //functions to get transforms
    geometry_msgs::TransformStamped getMarkerLocation(int id, ros::Time time);
    geometry_msgs::TransformStamped getMarkerDistance(int id, ros::Time time);

    //function for calculating the location of the rover from 2 ar tags
    geometry_msgs::Pose getPosition(double x1, double y1, double r1, double x2, double y2, double r2, double gx, double gy);
    
    public:
    //functions for initialising and running the node
    artag_localization();
    void run();
};


#endif // ARTAG_LOCALIZATION_H
