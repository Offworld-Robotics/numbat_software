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
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

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

    //transform messages
    geometry_msgs::TransformStamped arTransform;    //tranform of ar tag in world frame
    geometry_msgs::TransformStamped localTransform; //transform of camera relative to tags
    geometry_msgs::TransformStamped roverTransform; //transform of camera in world frame

	//callback function
	void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
	
    public:
    //functions for initialising and running the node
	artag_localization();
    void run();
};


#endif // ARTAG_LOCALIZATION_H
