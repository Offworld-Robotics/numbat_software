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
 * Combining information of multiple ar tags (ekf maybe?)
 */

#include "artags.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <cstdlib>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char *argv[]) {
    ROS_INFO("main"); 
    ros::init(argc, argv, "arTag_localization");
    artag_localization arl;
    arl.run();
    return 0;
}

//constructor, this just initialises some of the values needeed
artag_localization::artag_localization() : tfBuffer(), tfListener(tfBuffer)  {

    //subscribe to the topic that provides the pose of observed ar tags
    sub = nh.subscribe("/ar_pose_marker",1,&artag_localization::callback, this);
    
    //setup publisher to publish odometry message for ekf
    pub =  nh.advertise<nav_msgs::Odometry>("/artags/odometry", 1);
    //odomMsg.header.seq = 0;
    //odomMsg.header.stamp = ros::Time::now();
    //odomMsg.header.frame_id = "odom"
    //odomMsg.child_frame = "artagLocation"
    //set up covariance matrix for odometry message
    double var = 0.1; //change variance to set how reliable the measurement is
    double covar [36] = { }; //set all values to 0 initially
    //set diagonals to the variance value

    //create the covariance matrices, not the nicest way to do this but it works i think
    odomMsg.pose.covariance = {10, 0, 0, 0, 0, 0, 
                        0, 10, 0, 0, 0, 0,
                        0, 0, 10, 0, 0, 0,
                        0, 0, 0, 10, 0, 0,
                        0, 0, 0, 0, 10, 0,
                        0, 0, 0, 0, 0, 10};
    odomMsg.twist.covariance = {0, 0, 0, 0, 0, 0, 
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0};

    //covariance matrix is                        
    //                   {var, 0  , 0  , 0  , 0  , 0
    //                    0  , var, 0  , 0  , 0  , 0
    //                    0  , 0  , var, 0  , 0  , 0
    //                    0  , 0  , 0  , var, 0  , 0
    //                    0  , 0  , 0  , 0  , var, 0
    //                    0  , 0  , 0  , 0  , 0  , var};

    //setup the header parts of the messages used (this is used to define the reference frames)
    odomMsg.child_frame_id = "base_link";
    odomMsg.header.frame_id = "odom";
    odomMsg.header.seq = 0;
    //setup the ar transform, this is the markers position in world frame
    /*
    arTransform.header.frame_id = "";
    arTransform.child_frame_id = "location";
    arTransform.header.seq = 0;

    //setup the world transform, this is the camera postion in the world frame
    relativeTransform.header.frame_id = "rover_model";
    relativeTransform.child_frame_id = "base_link";
    relativeTransform.header.seq = 0;

    //setup the local transform, this is the camera postion relative to ar tag
    //localTransform.header.frame_id = "camera_link";
    //localTransform.child_frame_id = "ar_tag";
    //localTransform.header.seq = 0;
    */
}


void artag_localization::run() {
    while(ros::ok()) { 
      ros::spinOnce();
    }
}

void artag_localization::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    //find the amount of tags found
    int size = msg->markers.size();

    //Dont do anything if no tags were found 
    if (size != 0){
      ROS_INFO("tag found!");
      //get the id of the first tag
      int id = msg->markers[0].id;
	//get the time the ar tag message came
	ros::Time arMsgTime =  msg->header.stamp;
 
      //get header of ar tag message to be header of odom message
      //++odomMsg.header.seq;
      //odomMsg.header.stamp = ros::Time::now();
      //odomMsg.header.frame_id = "odom";
      
      //if only 1 marker was found then use a probability appraoch
      if(size == 1){
	  //get the trransform of the marker from the rover
	  geometry_msgs::TransformStamped distanceTf = getMarkerDistance(id, arMsgTime);
	  
	  //get the trasnform for the markers real location
	  geometry_msgs::TransformStamped markerTf = getMarkerLocation(id, arMsgTime);
      }
      
      //if 2 tags are seen then calculate the position of the rover, only usse first 2 tags found
      if (size > 1){
		  //find location of first tag
		  geometry_msgs::TransformStamped markerTf1 = getMarkerLocation(id, arMsgTime);
		  geometry_msgs::TransformStamped distanceTf1 = getMarkerDistance(id, arMsgTime);
		  
		  //find location of second tag
		  id = msg->markers[1].id;
		  geometry_msgs::TransformStamped markerTf2 = getMarkerLocation(id, arMsgTime);
		  geometry_msgs::TransformStamped distanceTf2 = getMarkerDistance(id, arMsgTime);
          
          //get the position
          double marker1x = markerTf1.transform.translation.x;
          double marker1y = markerTf1.transform.translation.y;
          double marker2x = markerTf2.transform.translation.x;
          double marker2y = markerTf2.transform.translation.y;
    
          //get the current guess of the rovers location
          geometry_msgs::TransformStamped roverTf;

/*
          double guessX = 0;
          double guessY = 0;
          //get the distance of the tags from the rover
          double distance1 = sqrt(pow(distanceTf1.transform.translation.x, 2) + pow(distanceTf1.transform.translation.y, 2));
          double distance2 = sqrt(pow(distanceTf2.transform.translation.x, 2) + pow(distanceTf2.transform.translation.y, 2));
  */  
	double relX1 = distanceTf1.transform.translation.x;
 	double relY1 = distanceTf1.transform.translation.y;
 	double relX2 = distanceTf2.transform.translation.x;
 	double relY2 = distanceTf2.transform.translation.y;      
          //calculate the postion of the rover
          geometry_msgs::Pose roverPose;          
	odomMsg.header.stamp = arMsgTime;
          roverPose = getPosition(marker1x, marker1y, relX1, relY1,  marker2x, marker2y, relX2, relY2);
	      ROS_INFO("****************************************************");
	//setup nav msg to send to ekf
	//put pose and covariance
	odomMsg.pose.pose = roverPose;
	
	//put time stamp and publish msg
//	++odomMsg.header.seq;
//	odomMsg.header.stamp = ros::Time::now();
//	pub.publish(odomMsg);
      }
    } else {
    //if no tags were found
	ROS_INFO("No tags found! :(");
    }   
}

geometry_msgs::TransformStamped artag_localization::getMarkerLocation(int id, ros::Time time){
    //get the name of the marker
    std::stringstream ss;
    ss << id;
    std::string markerFrame = "marker";
    markerFrame += ss.str();
    
   
    geometry_msgs::TransformStamped tf;
    
    //get the transform of the camera to the tag	
	try {
	    tf = tfBuffer.lookupTransform("odom", markerFrame, time);
        } catch (tf2::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        
    return tf;
}

geometry_msgs::TransformStamped artag_localization::getMarkerDistance(int id, ros::Time time){
    //get the name of the marker
    std::stringstream ss;
    ss << id;
    std::string markerFrame = "ar_marker_";
    markerFrame += ss.str();
    
    geometry_msgs::TransformStamped tf;
    
    //get the transform of the camera to the tag	
    try {
	tf = tfBuffer.lookupTransform(markerFrame, "base_link", time);
        } catch (tf2::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    
    return tf;
}

//if the calculated bearing falls out of -180 - 180 degree range fix it so it does
double fixBearingRange(double bearing){
	if (bearing > 180) {
		bearing = 360 - bearing;
	}
	if (bearing < -180) {
		bearing = 360 + bearing;
	}
	return bearing;
}

geometry_msgs::Pose artag_localization::getPosition(double x1, double y1, double relX1, double relY1, double x2, double y2, double relX2, double relY2)
{
    geometry_msgs::Pose pose;
	//std::cout << " x1 is: " << x1 << " y1 is: " << y1 << " relx1 is: " << relX1 << " relY1 is: " << relY1 << " x2 is: " << x2 << " y2 is: " << y2 << " relX1 is: " << relX1 << " relY2 is: " << relY2 << std::endl;
	ROS_INFO("x1 is %lf, y1 is %lf, relX1 is %lf, relY1 is %lf, x2 is %lf, y2 is %lf, relX2 is %lf, relY2 is %lf", x1, y1, relX1, relY1, x2, y2, relX2, relY2);
	//get the distance away from each marker
	double r1 =  sqrt(pow(relX1, 2) + pow(relY1, 2));
	double r2 =  sqrt(pow(relX2, 2) + pow(relY2, 2));

    // find the two probable points using a circle...
    // distance between centers...
    double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

    // check if two points exist
    if (d > r1 + r2 || d < abs(r1 - r2)) {
        // circles are separate (or contained)
        //printf("No intersection points!!\n");
    }
    else if (d < THRESHOLD) {
        // coincident!
        //printf("Coincident!");
    }

    double centerDist = ((r1 * r1) - (r2 * r2) + (d * d)) / (2 * d);
    double height = sqrt(fabs(r1 * r1 - centerDist * centerDist));

    // Evaluate center point
    double cx = x1 + centerDist * (x2 - x1) / d;
    double cy = y1 + centerDist * (y2 - y1) / d;

    // Evaluate postion
    double px1 = cx + height * (y2 - y1) / d;
    double px2 = cx - height * (y2 - y1) / d;
    double py1 = cy - height * (x2 - x1) / d;
    double py2 = cy + height * (x2 - x1) / d;

	ROS_INFO("cd is %lf, height is %lf", centerDist, height);
	//work out which guess is the correct point
	//first check its correct in the X direction (check if both on left or right)
	if(((x1 - px1) >= 0) == (relX1 >= 0)){
		if(((y1 - py1) >= 0) == (relY1 >= 0)){
			pose.position.x = px1;
			pose.position.y = py1;
		}
	}
	//check the other point
	if(((x2 - px2) >= 0) == (relX2 >= 0)){
		if(((y2 - py2) >= 0) == (relY2 >= 0)){
			pose.position.x = px2;
			pose.position.y = py2;
		}
	}

	//if pose is = 0 then we couldnt determine which of the two positions we were at and so return before doing more stuff
	if(pose.position.x == 0 && pose.position.y == 0) {
		return pose;
	}
	
	//determine our orientation now that we know our location
	//find the angle the marker makes from the location of the rover we predicted
	double PI = 3.14159;
	double marker1Angle = atan2((y1 - pose.position.y), (x1 - pose.position.x)) * (180 / PI);
	double marker2Angle = atan2((y2 - pose.position.y), (x2 - pose.position.x)) * (180 / PI);
	//get the angle the rover see the tag is from itself
	double marker1AngleSeen = atan2(relY1, relX1) * (180 / PI);
	double marker2AngleSeen = atan2(relY2, relX2) * (180 / PI);
	//taking the angle the rover is from the marker and subtraction by the angle we see gives the angle the rover is facing in the world frame
	//take the average of the orientation we get from both markers
	double bearing1 = marker1Angle - marker1AngleSeen;
	//bearing1 = fixBearingRange(bearing1);
	double bearing2 = marker2Angle - marker2AngleSeen;
	//bearing2 = fixBearingRange(bearing2);
	//if the calculated bearing differs alot between the two tags, then the ar tag size set is probobly wrong so adjust it
	double bearingDifference = fabs(bearing1 -bearing2);
	if (bearingDifference > 5){
		double markerSize; 
		nh.getParam("/ar_track_alvar/marker_size", markerSize);
		if(bearingDifference > 10){
		//markerSize -= 0.1;
		} else {
		//markerSize -= 0.1;
		}
		nh.setParam("/ar_track_alvar/marker_size", markerSize);
		ROS_INFO("marker size is %lf", markerSize);
	}
	ROS_INFO("a1 is  %lf, as1 is %lf, a2 is %lf, as2 is %lf", marker1Angle, marker1AngleSeen, marker2Angle, marker2AngleSeen);
	double roverBearing = (bearing1 + bearing2) / 2;
	//set up quaternion and put the bearing in it for the pose message
	tf2::Quaternion roverQuat_tf;
	geometry_msgs::Quaternion roverQuat_msg;
	roverQuat_tf.setRPY(0, 0, roverBearing);
	tf2::convert(roverQuat_msg, roverQuat_tf);
	pose.orientation = roverQuat_msg;
	ROS_INFO("1 is  %lf, 2 is %lf, rover is %lf", bearing1, bearing2, roverBearing);
	ROS_INFO("px is %lf, yx is %lf", pose.position.x, pose.position.y);
	++odomMsg.header.seq;
	odomMsg.header.stamp = ros::Time::now();
	pub.publish(odomMsg);
    return pose;
}

/* random code i used before thats useless but im keeping in case i need it again
 *     
    //get the transofrm of the camera relative to the rover
	try {
	    roverTransform = tfBuffer.lookupTransform("rover_model", "camera_frame", ros::Time(0));
        } catch (tf2i::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    */
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
    /* creates a tf for the guess of the rover location probobly dont need this
    roverTransform.header.frame_id = "odom";
    roverTransform.child_frame_id = "base_link";
    ++roverTransform.header.seq;
    roverTransform.header.stamp = ros::Time::now();
    roverTransform.transform.translation.x = odomMsg.pose.pose.position.x;
    roverTransform.transform.translation.y = odomMsg.pose.pose.position.y;
    roverTransform.transform.translation.z = odomMsg.pose.pose.position.z;
    roverTransform.transform.rotation.x = odomMsg.pose.pose.orientation.x;
    roverTransform.transform.rotation.y = odomMsg.pose.pose.orientation.y;
    roverTransform.transform.rotation.z = odomMsg.pose.pose.orientation.z;
    roverTransform.transform.rotation.w = odomMsg.pose.pose.orientation.w;
    //publish camera transform in world frame
	tfBroadcaster.sendTransform(roverTransform);
	*/
    
        /*
    //convert to odom msg, subtract the relative measurement vector from the actual ar tag location
    odomMsg.pose.pose.position.x = arTransform.transform.translation.x - relativeTransform.transform.translation.x;
    odomMsg.pose.pose.position.y = arTransform.transform.translation.y - relativeTransform.transform.translation.y;
    odomMsg.pose.pose.position.z = arTransform.transform.translation.z - relativeTransform.transform.translation.z;

	tf2::Quaternion quat_ar;
	tf2::Quaternion quat_local;
	tf2::convert(arTransform.transform.rotation, quat_ar);
	tf2::convert(relativeTransform.transform.rotation, quat_local);
    //negate the w component so that when multiplying the 2nd quat is inversed
	quat_local[3] = -quat_local[3]; 
    
    //multiply quaternions to get the quaternion for camera in the world frame
	odomMsg.pose.pose.orientation = tf2::toMsg(quat_ar * quat_local);

    //convert transform message from geomertry_msg to tf
    tf2::Transform tf;
    geometry_msgs::Transform geoTF;
    tf2::Stamped<tf2::Transform> relativetf;
    tf2::fromMsg(relativeTransform, relativetf);
    tf2::Stamped<tf2::Transform> artf;
    tf2::fromMsg(arTransform, artf);
    tf = artf.inverseTimes(relativetf);
    geoTF = tf2::toMsg(tf);
    //arTransform = tf2::toMsg(localtf);
    arTransform.transform = geoTF;
    arTransform = tf2::toMsg(artf);
    arTransform.header.frame_id = "camera_link";
    arTransform.child_frame_id = "location";
    //subtract the observed distance to the ar tag from the real world location


    pub.publish(odomMsg);
    */
