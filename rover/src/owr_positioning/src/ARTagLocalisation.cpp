/*
* Date Started:
 * Date: ????-??-??
 * Updated: 2019/09/08
 * Original Author: Nikola Medimurac
 * Editors: Michael Lloyd 
 * ROS Node Name: ARTagLocalisation
 * ROS Package: owr_positioning
 * Purpose: Creates a transform for the camera location in the world frame
 * using ar tags observed
 */

/*
 * Things to do:
 * Combining information of multiple ar tags (ekf maybe?)
 */

#include "ARTagLocalisation.hpp"
#include <geometry_msgs/Pose.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ARTagLocalisation");
    ARTagLocalisation arl;
    arl.run();
    return 0;
}

ARTagLocalisation::ARTagLocalisation() : tfBuffer(), tfListener(tfBuffer) {

    // Subscribe to topic with pose of AR Tags
    ar_pose_sub = nh.subscribe("/ar_pose_marker", 1,
            &ARTagLocalisation::callback, this);

    // Set publisher to topic that provides the post of the observed AR Tags.
    ar_pose_pub = nh.advertise<geometry_msgs::Pose>("/ar_tags/pose", 1);

    // FIXME:
    // The covariance and variance calculations have been converted into
    // magic numbers in the absence of proper (dynamic) calculations.
    // In the future (post ERC 2019) this should be done properly.

    //create the covariance matrices, not the nicest way to do this but it works i think
    odomMsg.pose.covariance = {
            10, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 10
    };

}

void ARTagLocalisation::run() {
    while(ros::ok()) { 
      ros::spinOnce();
    }
}

//void ARTagLocalisation::callback(const
//ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {

//}

//
//  <quack>
//


void ARTagLocalisation::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
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
	ROS_INFO("x1 is %lf, y1 is %lf, relX1 is %lf, relY1 is %lf, x2 is %lf, y2 is %lf, relX2 is %lf, relY2 is %lf", marker1x, marker1y, relX1, relY1, marker2x, marker2y, relX2, relY2);
          //calculate the postion of the rover
          geometry_msgs::Pose roverPose;          
	odomMsg.header.stamp = arMsgTime;
          roverPose = getPosition(marker1x, marker1y, relX1, relY1,  marker2x, marker2y, relX2, relY2);
	      ROS_INFO("****************************************************");
	//setup nav msg to send to ekf
	//put pose and covariance
	
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

geometry_msgs::TransformStamped ARTagLocalisation::getMarkerLocation(int id, ros::Time time){
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

geometry_msgs::TransformStamped ARTagLocalisation::getMarkerDistance(int id, ros::Time time){
    //get the name of the marker
    std::stringstream ss;
    ss << id;
    std::string markerFrame = "ar_marker_";
    markerFrame += ss.str();
    
	ROS_INFO("id is %d", id);
    geometry_msgs::TransformStamped tf;
    
    //get the transform of the camera to the tag	
    try {
	tf = tfBuffer.lookupTransform("base_link", markerFrame, time);
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

geometry_msgs::Pose ARTagLocalisation::getPosition(double x1, double y1, double relX1, double relY1, double x2, double y2, double relX2, double relY2)
{
    geometry_msgs::Pose roverPose;
	//std::cout << " x1 is: " << x1 << " y1 is: " << y1 << " relx1 is: " << relX1 << " relY1 is: " << relY1 << " x2 is: " << x2 << " y2 is: " << y2 << " relX1 is: " << relX1 << " relY2 is: " << relY2 << std::endl;
	ROS_INFO("x1 is %lf, y1 is %lf, relX1 is %lf, relY1 is %lf, x2 is %lf, y2 is %lf, relX2 is %lf, relY2 is %lf", x1, y1, relX1, relY1, x2, y2, relX2, relY2);

	//apply the scaling factor to the observed distances
	relX1 *= scaling;
	relY1 *= scaling;
	relX2 *= scaling;
	relY2 *= scaling;

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
	ROS_INFO("x1 is %lf,x2 is %lf yq is %lf y2 is %lf", (x1 - px1), (x2 - px2), (y1 - py1), (y2 - py2));
	if(((x1 - px1) >= 0) == (relX1 >= 0)){
		if(((y1 - py1) >= 0) == (relY1 >= 0)){
			roverPose.position.x = px1;
			roverPose.position.y = py1;
		}
	}
	if(((x2 - px1) >= 0) == (relX2 >= 0)){
		if(((y2 - py1) >= 0) == (relY2 >= 0)){
			roverPose.position.x = px1;
			roverPose.position.y = py1;
		}
	}

	if(((x1 - px2) >= 0) == (relX1 >= 0)){
		if(((y1 - py2) >= 0) == (relY1 >= 0)){
			roverPose.position.x = px2;
			roverPose.position.y = py2;
		}
	}
	//check the other point
	if(((x2 - px2) >= 0) == (relX2 >= 0)){
		if(((y2 - py2) >= 0) == (relY2 >= 0)){
			roverPose.position.x = px2;
			roverPose.position.y = py2;
		}
	}
	//if pose is = 0 then we couldnt determine which of the two positions we were at and so return before doing more stuff
	if(roverPose.position.x == 0 && roverPose.position.y == 0) {
		return roverPose;
	}
	
	//determine our orientation now that we know our location
	//find the angle the marker makes from the location of the rover we predicted
	double PI = 3.14159;
	double marker1Angle = atan2((y1 - roverPose.position.y), (x1 - roverPose.position.x)) * (180 / PI);
	double marker2Angle = atan2((y2 - roverPose.position.y), (x2 - roverPose.position.x)) * (180 / PI);
	//get the angle the rover see the tag is from itself
	double marker1AngleSeen = atan2(relY1, relX1) * (180 / PI);
	double marker2AngleSeen = atan2(relY2, relX2) * (180 / PI);
	//taking the angle the rover is from the marker and subtraction by the angle we see gives the angle the rover is facing in the world frame
	//take the average of the orientation we get from both markers
	double bearing1 = marker1Angle - marker1AngleSeen;
	bearing1 = fixBearingRange(bearing1);
	double bearing2 = marker2Angle - marker2AngleSeen;
	bearing2 = fixBearingRange(bearing2);
	//if the calculated bearing differs alot between the two tags, then the ar tag size set is probobly wrong so adjust it
	double bearingDifference = fabs(bearing1 -bearing2);
	if (1){
		double markerSize; 
		//find the angle between the vectors joining the ar tags to the rover
		//find this for both what it should be using the calculated position and what is observed and take the difference
		double markerAngleDiff = fabs(marker1Angle -  marker2Angle);
		if (markerAngleDiff > 180){
			markerAngleDiff = 360 - markerAngleDiff;
		}
		double markerAngleDiffSeen = fabs(marker1AngleSeen -  marker2AngleSeen); 
		if (markerAngleDiffSeen > 180){
			markerAngleDiffSeen = 360 - markerAngleDiffSeen;
		}
		double angleDiff = markerAngleDiff - markerAngleDiffSeen;
		if(angleDiff > 2){
		scaling += 0.001;
		}
		if (angleDiff < -2){
		scaling -= 0.001;
		}
		ROS_INFO("scaling factor is %lf", scaling);
		ROS_INFO("ma is  %lf, mas is %lf", markerAngleDiff, markerAngleDiffSeen);
	}
	ROS_INFO("a1 is  %lf, as1 is %lf, a2 is %lf, as2 is %lf", marker1Angle, marker1AngleSeen, marker2Angle, marker2AngleSeen);
	double roverBearing = (bearing1 + bearing2) / 2;
	//set up quaternion and put the bearing in it for the pose message
	tf2::Quaternion quat_tf;
	geometry_msgs::Quaternion roverQuat_msg;
        //std::cout << quat_tf << "\n";
	quat_tf.setRPY(0, 0, (roverBearing * (PI / 180)));
        tf2::convert(quat_tf, roverPose.orientation);
	//odomMsg.pose.pose.orientation = geometry_msgs::Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]);
	ROS_INFO("x is  %lf, y is %lf, z is %lf, w is %lf", bearing1, bearing2, roverBearing);
	ROS_INFO("1 is  %lf, 2 is %lf, rover is %lf", bearing1, bearing2, roverBearing);
	ROS_INFO("px is %lf, yx is %lf", roverPose.orientation.x, roverPose.orientation.y);
	odomMsg.pose.pose = roverPose;
	++odomMsg.header.seq;
	odomMsg.header.stamp = ros::Time::now();
	pub.publish(odomMsg);
    return roverPose;
}

