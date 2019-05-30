/*
 * Date Started:
 * Original Author: Nikola Medimurac
 * Editors: 
 * ROS Node Name: arTag_localization
 * ROS Package: owr_positioning
 * Purpose: Creates a transform for the camera location in the world frame
 * using ar tags observed
 */

/*
 * Things to do:
 * Transform camera axis to match the axis of the rover
 * Better method for configuration of ar tags
 * Combining information of multiple ar tags (ekf maybe?)
 */

#include "artags.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <cstdlib>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char *argv[]) {
    ROS_INFO("main"); 
    ros::init(argc, argv, "arTag_localization");
    artag_localization arl;
    arl.run();
    return 0;
}

artag_localization::artag_localization() : tfBuffer(), tfListener(tfBuffer)  {

    //subscribe to the topic that provides the pose of observed ar tags
    sub = nh.subscribe("/ar_pose_marker",1,&artag_localization::callback, this);
    
    //setup the ar transform, this is the markers position in world frame
    arTransform.header.frame_id = "camera_frame";
    arTransform.child_frame_id = "artag";
    arTransform.header.seq = 0;

    //setup the world transform, this is the camera postion in the world frame
    roverTransform.header.frame_id = "rover_model";
    roverTransform.child_frame_id = "base_link";
    roverTransform.header.seq = 0;

    //setup the local transform, this is the camera postion relative to ar tag
    localTransform.header.frame_id = "camera_link";
    localTransform.child_frame_id = "ar_tag";
    localTransform.header.seq = 0;
}


void artag_localization::run() {
    while(ros::ok()) { 
      ros::spinOnce();
    }
}

void artag_localization::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    //find the amount of tags found
    int size = msg->markers.size();

    //for now we will only look at the first tag
    if (size != 0){
	ROS_INFO("tag found!");
	//get the id of the first tag
	int id = msg->markers[0].id;
	
    //get the name of the markers frame, this is ar_marker_(id) and save this as a string
    //to do this we use stringstream to convert the id to a string and attach it to the end
    //of the string "ar_marker_"
    std::stringstream ss;
    ss << id;
	std::string markerFrame = "ar_marker_";
	markerFrame += ss.str();

	//get the transform of the camera to the tag	try {
	    localTransform = tfBuffer.lookupTransform(markerFrame, "camera_link", ros::Time(0));
        } catch (tf2::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    
    //the local transform uses axis relative to the camera, apply a rotation transformation
    //so that it matches the rovers axis

	//look up transform of tag in the world frame
    /*
	std::string worldMarkerFrame = "marker";
	worldMarkerFrame += ss.str();
	try {
	    arTransform = tfBuffer.lookupTransform("world", worldMarkerFrame, ros::Time(0));
        } catch (tf2::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }	
    */
    //get the transofrm of the camera relative to the rover
	try {
	    roverTransform = tfBuffer.lookupTransform("rover_model", "camera_frame", ros::Time(0));
        } catch (tf2::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

    /*	
    //calculate the world transform by subtracting the local trasnform from the ar tags transform
    //first find translations by subtracting x,y,z coordinates
	arTransform.transform.translation.x = roverTransform.transform.translation.x + localTransform.transform.translation.x;
	arTransform.transform.translation.y = roverTransform.transform.translation.y + localTransform.transform.translation.y;
	arTransform.transform.translation.z = roverTransform.transform.translation.z + localTransform.transform.translation.z;

    //next determine the angle by subtracting them, angles are stored as quaternions
    //to find this angle we need to multiple the first by the inverse of the second
    //quaternion is first converted from message type to quaternion type
	tf2::Quaternion quat_ar;
	tf2::Quaternion quat_local;
	tf2::Quaternion quat_world;
	tf2::convert(roverTransform.transform.rotation, quat_ar);
	tf2::convert(localTransform.transform.rotation, quat_local);
	
    //negate the w component so that when multiplying the 2nd quat is inversed
	//quat_local[3] = -quat_local[3]; 
    
    //multiply quaternions to get the quaternion for camera in the world frame
	quat_world = quat_ar * quat_local;
	tf2::convert(quat_world, arTransform.transform.rotation);
    */

    //publish camera transform in world frame
	tfBroadcaster.sendTransform(arTransform);
    } else {
    //if no tags were found
	ROS_INFO("No tags found! :(");
    }   
}

